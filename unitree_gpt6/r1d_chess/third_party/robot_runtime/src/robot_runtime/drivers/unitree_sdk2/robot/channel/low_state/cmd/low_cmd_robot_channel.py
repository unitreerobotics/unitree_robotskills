"""Unitree HG LowCmd command channel.

``LowCmdRobotChannel`` combines the shared ``rt/lowstate`` input with an HG
``LowCmd_`` output topic. It coordinates all components sharing the motor
array, mirrors ``mode_machine``, calculates CRC, and encodes model-specific
arm_sdk ownership weights.
"""

from collections.abc import Mapping
from copy import copy, deepcopy
import time

import logging_mp
from typing_extensions import override
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__MotorCmd_,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as HGLowCmd
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import MotorCmd_, MotorState_
from unitree_sdk2py.utils.crc import CRC

from robot_runtime.drivers.unitree_sdk2.robot.channel.low_state.low_state_robot_channel import (
    LOW_STATE_TOPIC, LowStateRobotChannel,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.body.hg_body import (
    HGBodyInterface,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.sensor.imu import (
    IMUInterface,
)
from robot_runtime.drivers.unitree_sdk2.robot.motion.aisport import AISportFSM
from robot_runtime.drivers.unitree_sdk2.robot.motion.switcher import (
    MotionServiceSwitcher,
)
from robot_runtime.interfaces.robot.component import (
    ComponentCmds,
    ComponentInterface,
    ComponentStates,
)

logger_mp = logging_mp.getLogger(__name__)

CONTROL_WEIGHT_DURATION = 2.0
CONTROL_RELEASE_PERIOD = 1.0 / 250.0


class LowCmdRobotChannel(LowStateRobotChannel):
    """Read HG LowState and publish one complete HG LowCmd envelope.

    Body and writable embedded component commands are encoded into the same
    motor array before one message is published. Every physical motor slot
    must have a single owner; an optional control-weight slot participates in
    the same ownership check.

    Optional ``motor_topics`` routes selected body motors through standalone
    HG MotorState/MotorCmd messages. Local envelope copies preserve the body
    interface's single-message contract; routed slots are disabled on the wire.

    H2 arm_sdk enters FSM 703 from 701 before publishing. Existing FSM 4/703
    is retained. Exit releases the weight before restoring any FSM/Arm SDK
    flag changed during handover.

    Args:
        body_interface: Interface mapping body states and LowCmd motor slots.
        imu_interface: Interface mapping the embedded HG IMU state.
        cmd_topic: DDS topic receiving HG LowCmd messages.
        body_command_schema_keys: Complete body schema keys required per write.
        embedded_interfaces: Additional components sharing LowState/LowCmd.
        writable_embedded_interfaces: Members of ``embedded_interfaces`` that
            accept commands.
        control_weight_motor_index: Legacy override for the control-weight
            slot. Known arm_sdk bodies select their encoding automatically.
        restore_motion_mode: Optional baseline service restored after leaving
            debug control. When configured, active services are released on
            control entry. R1 LowCmd exit keeps debug control released from
            motion services. R1-D stops publishing without clearing arm gains;
            fixed R1 releases arm/hand torque.
        motor_topics: Logical body motor indices mapped to pairs of standalone
            state and command topics. Only rt/lowcmd without a weight is supported.
        start_timeout: Seconds to wait for the first LowState message.

    Attributes:
        cmd_topic: Configured HG LowCmd output topic.
        body_command_schema_keys: Required body command schema keys.
        control_weight_motor_index: Optional control-weight motor slot.
        motion_service_switcher: Optional debug service lifecycle.
        crc: SDK CRC calculator applied to every published LowCmd message.
        mode_machine: Latest LowState machine identifier copied to LowCmd.

    Example:
        channel = LowCmdRobotChannel(
            body_interface=body_interface,
            imu_interface=imu_interface,
            cmd_topic="rt/lowcmd",
            body_command_schema_keys=("left_arm", "right_arm"),
        )
        states = channel.decode_component_states({
            channel.state_topic: low_state_msg,
        })
        channel.encode_component_cmds(
            {
                body_interface.component_id: {
                    "left_arm": left_arm_cmd,
                    "right_arm": right_arm_cmd,
                },
            },
            {channel.cmd_topic: low_cmd_msg},
        )
    """

    def __init__(
        self,
        *,
        body_interface: HGBodyInterface,
        imu_interface: IMUInterface,
        cmd_topic: str,
        body_cmd_schema_keys: tuple[str, ...],
        embedded_interfaces: tuple[ComponentInterface, ...] = (),
        writable_embedded_interfaces: tuple[ComponentInterface, ...] = (),
        control_weight_motor_index: int | None = None,
        restore_motion_mode: str | None = None,
        motor_topics: Mapping[int, tuple[str, str]] | None = None,
        start_timeout: float = 10.0,
    ) -> None:
        if not body_cmd_schema_keys:
            raise ValueError(
                "LowCmdRobotChannel requires at least one body command schema key."
            )
        self.cmd_topic = cmd_topic
        self.body_cmd_schema_keys = body_cmd_schema_keys
        self.motor_topics = dict(motor_topics or {})
        if self.motor_topics:
            if cmd_topic != "rt/lowcmd" or control_weight_motor_index is not None:
                raise ValueError("motor_topics requires rt/lowcmd without a control weight.")
            body_indices = {
                index for key in body_cmd_schema_keys
                for index in body_interface.motor_indices[key]
            }
            topic_names = {LOW_STATE_TOPIC, cmd_topic}
            slot_count = len(unitree_hg_msg_dds__LowCmd_().motor_cmd)
            for index, topics in self.motor_topics.items():
                if index not in body_indices or not 0 <= index < slot_count:
                    raise ValueError(f"motor_topics index {index} is not a writable body motor.")
                if len(set(topics)) != 2 or topic_names.intersection(topics):
                    raise ValueError("motor_topics must use distinct, non-shared DDS topics.")
                topic_names.update(topics)
        body_id = body_interface.component_id
        # R1 uses a percentage in mode_pr; G1/H2 reserve a motor command slot.
        self.control_weight_in_mode_pr = (
            cmd_topic == "rt/arm_sdk"
            and body_id in {"body.r1", "body.r1_edu"}
        )
        if cmd_topic == "rt/arm_sdk":
            if restore_motion_mode is not None:
                raise ValueError("arm_sdk must keep the motion service running.")
            if self.control_weight_in_mode_pr:
                if control_weight_motor_index is not None:
                    raise ValueError("R1 arm_sdk uses mode_pr, not a motor slot.")
            elif control_weight_motor_index is None:
                if body_id == "body.g1_edu":
                    control_weight_motor_index = 29
                elif body_id == "body.h2_edu":
                    control_weight_motor_index = 31
                else:
                    raise ValueError(
                        f"Unknown arm_sdk encoding for {body_id!r}."
                    )
        self.control_weight_motor_index = control_weight_motor_index
        self.control_start_time: float | None = None
        self.control_weight = 0.0
        self.last_cmd_msg: HGLowCmd | None = None
        self.aisport_fsm = (
            AISportFSM(control_fsm_id=703, restore_fsm_id=701)
            if cmd_topic == "rt/arm_sdk" and body_id == "body.h2_edu"
            else None
        )
        self.restore_arm_sdk_status: bool | None = None
        self.motion_service_switcher = (
            None
            if restore_motion_mode is None
            else MotionServiceSwitcher(
                restore_motion_mode=restore_motion_mode,
            )
        )
        writable_components = [body_interface.component_id]
        writable_components.extend(
            component_interface.component_id
            for component_interface in writable_embedded_interfaces
        )
        super().__init__(
            body_interface=body_interface,
            imu_interface=imu_interface,
            cmd_topics={
                cmd_topic: (HGLowCmd, unitree_hg_msg_dds__LowCmd_),
                **{
                    command_topic: (MotorCmd_, unitree_hg_msg_dds__MotorCmd_)
                    for _, command_topic in self.motor_topics.values()
                },
            },
            additional_state_topics={
                state_topic: MotorState_
                for state_topic, _ in self.motor_topics.values()
            },
            writable_components=writable_components,
            embedded_interfaces=embedded_interfaces,
            start_timeout=start_timeout,
        )
        self.crc = CRC()
        self.mode_machine = 0
        self.validate_motor_ownership(embedded_interfaces)

    @override
    def start(self) -> None:
        """Start DDS and initialize motion clients without taking control."""
        super().start()
        try:
            if self.aisport_fsm is not None:
                self.aisport_fsm.initialize()
            if self.motion_service_switcher is not None:
                self.motion_service_switcher.initialize()
        except Exception:
            self.stop()
            raise

    @override
    def stop(self) -> None:
        """Release command ownership, restore services, and stop DDS I/O."""
        try:
            if self.aisport_fsm is not None:
                self.exit_control()
                self.aisport_fsm.shutdown()
            else:
                self.write_control_release()
            if self.motion_service_switcher is not None:
                self.motion_service_switcher.shutdown()
        finally:
            super().stop()

    def enter_control(self) -> None:
        """Begin SDK handover or release the configured debug service."""
        if self.aisport_fsm is not None:
            if self.control_start_time is not None:
                return
            if (
                self.aisport_fsm.restore_required
                or self.restore_arm_sdk_status is not None
            ):
                raise RuntimeError("H2 previous control restoration is incomplete.")
            client = self.aisport_fsm.require_client()
            client.SetTimeout(3.0)
            fsm_id = client.get_fsm_id()
            if fsm_id not in {4, 701, 703}:
                raise RuntimeError(
                    f"H2 arm_sdk requires FSM 4, 701 or 703; actual={fsm_id}. "
                    "Prepare the robot in FSM 701 before starting control."
                )
            self.restore_arm_sdk_status = client.get_arm_sdk_status()
            try:
                if fsm_id == 701:
                    self.aisport_fsm.enter_control_fsm()
                if not client.get_arm_sdk_status():
                    client.set_arm_sdk_status(True)
            except Exception:
                self.exit_control()
                raise
            # Keep the snapshot across an FSM round trip; switching back may
            # also affect the SDK flag. An unchanged FSM/flag needs no restore.
            if self.restore_arm_sdk_status and not self.aisport_fsm.restore_required:
                self.restore_arm_sdk_status = None
            logger_mp.info(
                "H2 arm_sdk ready: FSM=%d, external Arm SDK enabled.",
                703 if fsm_id == 701 else fsm_id,
            )
        if self.motion_service_switcher is not None:
            self.motion_service_switcher.release_control_service()
        self.control_start_time = time.monotonic()
        self.control_weight = 0.0
        self.last_cmd_msg = None

    def exit_control(self) -> None:
        """Hand SDK control back or restore the configured debug service."""
        self.write_control_release()
        if self.aisport_fsm is not None:
            self.aisport_fsm.restore_fsm()
            if self.restore_arm_sdk_status is not None:
                try:
                    client = self.aisport_fsm.require_client()
                    if client.get_arm_sdk_status() != self.restore_arm_sdk_status:
                        client.set_arm_sdk_status(self.restore_arm_sdk_status)
                except Exception as exc:
                    logger_mp.warning("H2 Arm SDK restoration failed: %s", exc)
                else:
                    self.restore_arm_sdk_status = None
        if self.motion_service_switcher is not None:
            self.motion_service_switcher.restore_service()

    def validate_motor_ownership(
        self,
        embedded_interfaces: tuple[ComponentInterface, ...],
    ) -> None:
        """Reject out-of-range or multiply owned LowCmd motor slots."""
        motor_slots = self.body_interface.motor_slots
        control_weight_motor_index = self.control_weight_motor_index
        owners: dict[int, str] = {}
        motor_ownership = [
            (
                self.body_interface.component_id,
                tuple(
                    motor_index
                    for motor_indices in (
                        self.body_interface.motor_indices.values()
                    )
                    for motor_index in motor_indices
                ),
            ),
        ]
        for component_interface in embedded_interfaces:
            message_motor_indices = getattr(
                component_interface,
                "message_motor_indices",
                None,
            )
            if message_motor_indices is None:
                continue
            motor_ownership.append(
                (
                    component_interface.component_id,
                    tuple(
                        motor_index
                        for motor_indices in message_motor_indices.values()
                        for motor_index in motor_indices
                    ),
                )
            )
        if control_weight_motor_index is not None:
            motor_ownership.append(
                ("control weight", (control_weight_motor_index,))
            )
        for component_id, motor_indices in motor_ownership:
            slot_count = (
                len(unitree_hg_msg_dds__LowCmd_().motor_cmd)
                if component_id == "control weight"
                else motor_slots
            )
            for motor_index in motor_indices:
                if motor_index < 0 or motor_index >= slot_count:
                    raise ValueError(
                        f"Component {component_id!r} owns motor index "
                        f"{motor_index} outside [0, {slot_count})."
                    )
                if motor_index in owners:
                    raise ValueError(
                        f"LowCmd motor index {motor_index} is owned by both "
                        f"{owners[motor_index]!r} and {component_id!r}."
                    )
                owners[motor_index] = component_id

    @override
    def decode_component_states(
        self,
        state_msgs: Mapping[str, object],
    ) -> dict[str, ComponentStates]:
        """Capture ``mode_machine`` and decode the shared LowState envelope."""
        self.mode_machine = state_msgs[self.state_topic].mode_machine
        if self.motor_topics:
            # Substitute feedback only in a local decoding view. The original
            # DDS envelope and its motor array remain untouched.
            body_state = copy(state_msgs[self.state_topic])
            body_state.motor_state = list(body_state.motor_state)
            for index, (state_topic, _) in self.motor_topics.items():
                body_state.motor_state[index] = state_msgs[state_topic]
            state_msgs = {**state_msgs, self.state_topic: body_state}
        return super().decode_component_states(state_msgs)

    @override
    def encode_component_cmds(
        self,
        component_cmds: Mapping[str, ComponentCmds],
        cmd_msgs: Mapping[str, object],
    ) -> None:
        """Fill LowCmd motor slots, protocol fields, control weight, and CRC."""
        body_component_id = self.body_interface.component_id
        received_schema_keys = tuple(component_cmds[body_component_id])
        if set(received_schema_keys) != set(self.body_cmd_schema_keys):
            raise RuntimeError(
                f"{type(self).__name__} requires component "
                f"{body_component_id!r} schema keys "
                f"{self.body_cmd_schema_keys!r}, got "
                f"{received_schema_keys!r}."
            )
        cmd_msg = cmd_msgs[self.cmd_topic]
        body_cmd_msg = cmd_msg
        if self.motor_topics:
            # HGBody writes through these local references into the standalone
            # MotorCmd objects; no routed target enters the outgoing LowCmd.
            body_cmd_msg = copy(cmd_msg)
            body_cmd_msg.motor_cmd = list(cmd_msg.motor_cmd)
            for index, (_, command_topic) in self.motor_topics.items():
                body_cmd_msg.motor_cmd[index] = cmd_msgs[command_topic]
                cmd_msg.motor_cmd[index] = unitree_hg_msg_dds__MotorCmd_()
        for component_id in self.writable_components:
            self.components[component_id].write_cmds_to_msgs(
                (body_cmd_msg if component_id == body_component_id else cmd_msg,),
                component_cmds[component_id],
            )
        cmd_msg.mode_pr = 0
        cmd_msg.mode_machine = self.mode_machine
        if self.control_start_time is not None:
            self.control_weight = min(
                1.0,
                max(0.0, time.monotonic() - self.control_start_time)
                / CONTROL_WEIGHT_DURATION,
            )
        self.set_control_weight(cmd_msg, self.control_weight)
        cmd_msg.crc = self.crc.Crc(cmd_msg)
        self.last_cmd_msg = cmd_msg

    def set_control_weight(self, cmd_msg: HGLowCmd, weight: float) -> None:
        """Encode SDK ownership using the body's wire protocol."""
        if self.control_weight_in_mode_pr:
            cmd_msg.mode_pr = int(100.0 * weight)
        if self.control_weight_motor_index is not None:
            control_weight_cmd = cmd_msg.motor_cmd[
                self.control_weight_motor_index
            ]
            control_weight_cmd.mode = 1
            control_weight_cmd.q = weight

    def write_control_release(self) -> None:
        """End R1 LowCmd control or fade arm_sdk ownership out."""
        if (
            self.control_weight_motor_index is None
            and not self.control_weight_in_mode_pr
        ):
            if self.body_interface.component_id not in {
                "body.r1_a5", "body.r1_a7", "body.r1_a7_d",
            } or self.cmd_topic != "rt/lowcmd":
                return
            with self.state_lock:
                cmd_msg = self.last_cmd_msg
                self.control_start_time = None
                if cmd_msg is None:
                    return
                if self.body_interface.component_id in {
                    "body.r1_a7", "body.r1_a7_d",
                }:
                    # Runtime already cleared torque-only hands. End publication
                    # without unloading the arms or enabling another controller.
                    self.last_cmd_msg = None
                    self.control_weight = 0.0
                    if self.motion_service_switcher is not None:
                        self.motion_service_switcher.restore_required = False
                    return
                cmd_msg = deepcopy(cmd_msg)
                cmd_msg.mode_machine = self.mode_machine
            motor_indices = [
                index
                for key in self.body_cmd_schema_keys
                if key in {"left_arm", "right_arm"}
                for index in self.body_interface.motor_indices[key]
            ]
            for component_id in self.writable_components:
                if component_id == self.body_interface.component_id:
                    continue
                for indices in self.components[component_id].message_motor_indices.values():
                    motor_indices.extend(indices)
            for index in motor_indices:
                motor = cmd_msg.motor_cmd[index]
                motor.mode = 1
                motor.q = motor.dq = motor.tau = motor.kp = motor.kd = 0.0
            cmd_msg.crc = self.crc.Crc(cmd_msg)
            with self.dds_io_lock:
                publisher = self.cmd_publishers.get(self.cmd_topic)
                if not self.is_connected() or publisher is None:
                    raise RuntimeError("Cannot release R1 arm torque: DDS is disconnected.")
                # Repeat the final frame briefly before ending publication.
                for _ in range(3):
                    publisher.Write(cmd_msg)
                    time.sleep(CONTROL_RELEASE_PERIOD)
            self.last_cmd_msg = None
            self.control_weight = 0.0
            if self.motion_service_switcher is not None:
                # Keep the explicitly requested zero-torque state. Restoring
                # AI here (or during shutdown) could re-enable arm gains.
                self.motion_service_switcher.restore_required = False
            logger_mp.info("R1 arm/hand zero-torque command sent.")
            return
        with self.state_lock:
            cmd_msg = deepcopy(self.last_cmd_msg)
            start_weight = self.control_weight
            self.control_start_time = None
        try:
            if cmd_msg is None:
                return
            started_at = time.monotonic()
            while True:
                cycle_started_at = time.monotonic()
                elapsed = cycle_started_at - started_at
                weight = start_weight * max(
                    0.0, 1.0 - elapsed / CONTROL_WEIGHT_DURATION
                )
                self.set_control_weight(cmd_msg, weight)
                with self.state_lock:
                    cmd_msg.mode_machine = self.mode_machine
                cmd_msg.crc = self.crc.Crc(cmd_msg)
                with self.dds_io_lock:
                    publisher = self.cmd_publishers.get(self.cmd_topic)
                    if not self.is_connected() or publisher is None:
                        return
                    publisher.Write(cmd_msg)
                if weight == 0.0:
                    break
                # Continue publishing complete frames throughout handover.
                elapsed = time.monotonic() - cycle_started_at
                time.sleep(max(0.0, CONTROL_RELEASE_PERIOD - elapsed))
        except Exception as exc:
            logger_mp.warning(
                "[%s] Failed to release control weight: %s",
                type(self).__name__,
                exc,
            )
        finally:
            self.control_weight = 0.0
            self.last_cmd_msg = None
