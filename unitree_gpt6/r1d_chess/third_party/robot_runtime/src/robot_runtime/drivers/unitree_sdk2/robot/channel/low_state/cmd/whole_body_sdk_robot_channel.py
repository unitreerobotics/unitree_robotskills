"""Unitree whole-body SDK state and JSON command channel."""

import json
from collections.abc import Mapping

import logging_mp
import numpy as np
from typing_extensions import override
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

from robot_runtime.drivers.unitree_sdk2.robot.channel.low_state.low_state_robot_channel import (
    LOW_STATE_TOPIC,
    LowStateRobotChannel,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.base_group.g1_edu_pseudo_base import (
    G1EduPseudoBaseInterface,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.body.hg_body import (
    HGBodyInterface,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.sensor.imu import (
    IMUInterface,
)
from robot_runtime.drivers.unitree_sdk2.robot.motion.switcher import (
    MotionServiceSwitcher,
)
from robot_runtime.interfaces.robot.component import (
    ComponentCmds,
    ComponentInterface,
    ComponentStates,
)
from robot_runtime.interfaces.robot.cmd.base_group import BaseGroupCmd

logger_mp = logging_mp.getLogger(__name__)

DDS_CLOSE_TIMEOUT = 2.0
WHOLE_BODY_SDK_CMD_TOPIC = "rt/whole_body_sdk"
WHOLE_BODY_SDK_ODOM_STATE_TOPIC = "rt/odommodestate"


class WholeBodySdkRobotChannel(LowStateRobotChannel):
    """Publish synchronized arm and mobile-base commands as JSON.

    Each write requires every configured body joint-group schema key plus one
    base-group schema key. The pseudo-base interface exposes odometry;
    LowState waist feedback is exposed by the body joint interface.

    Args:
        body_interface: Interface decoding body state from HG LowState.
        imu_interface: Interface decoding the embedded HG IMU state.
        base_group_interface: Interface decoding the pseudo-base state.
        body_cmd_schema_keys: Arm or body schema keys required per write.
        base_group_schema_key: Base-group schema key required per write.
        control_motion_mode: MotionSwitcher mode used during control.
        restore_motion_mode: MotionSwitcher mode restored after control.
        embedded_interfaces: Additional state-only LowState interfaces.
        start_timeout: Seconds to wait for the first LowState message.

    Attributes:
        cmd_topic: Fixed whole-body JSON command topic.
        body_cmd_schema_keys: Required body command schema keys.
        base_group_schema_key: Required base-group schema key.
        motion_service_switcher: Configured MotionSwitcher service lifecycle.

    Example:
        channel = WholeBodySdkRobotChannel(
            body_interface=body_interface,
            imu_interface=imu_interface,
            base_group_interface=base_group_interface,
            body_cmd_schema_keys=("left_arm", "right_arm"),
            base_group_schema_key="pseudo_base_lift",
            control_motion_mode="teleop",
            restore_motion_mode="ai",
        )
        payload = channel.build_cmd_payload(
            {
                "left_arm": left_arm_cmd,
                "right_arm": right_arm_cmd,
            },
            base_cmd,
        )
    """

    def __init__(
        self,
        *,
        body_interface: HGBodyInterface,
        imu_interface: IMUInterface,
        base_group_interface: G1EduPseudoBaseInterface,
        body_cmd_schema_keys: tuple[str, ...],
        base_group_schema_key: str,
        control_motion_mode: str,
        restore_motion_mode: str,
        embedded_interfaces: tuple[ComponentInterface, ...] = (),
        start_timeout: float = 10.0,
    ) -> None:
        self.cmd_topic = WHOLE_BODY_SDK_CMD_TOPIC
        self.body_cmd_schema_keys = body_cmd_schema_keys
        self.base_group_interface = base_group_interface
        self.base_group_schema_key = base_group_schema_key
        self.motion_service_switcher = MotionServiceSwitcher(
            control_motion_mode=control_motion_mode,
            restore_motion_mode=restore_motion_mode,
        )

        motor_owners: dict[int, str] = {}
        for embedded_interface in embedded_interfaces:
            message_motor_indices = getattr(
                embedded_interface,
                "message_motor_indices",
                None,
            )
            if message_motor_indices is None:
                continue
            for motor_indices in message_motor_indices.values():
                for motor_index in motor_indices:
                    if motor_index >= body_interface.motor_slots:
                        raise ValueError(
                            f"Component interface "
                            f"{embedded_interface.component_id!r} "
                            f"maps to "
                            f"motor index {motor_index} outside "
                            f"[0, {body_interface.motor_slots})."
                        )
                    if motor_index in motor_owners:
                        raise ValueError(
                            f"WholeBodySdk motor index {motor_index} is mapped "
                            f"by both {motor_owners[motor_index]!r} and "
                            f"{embedded_interface.component_id!r}."
                        )
                    motor_owners[motor_index] = embedded_interface.component_id
        super().__init__(
            body_interface=body_interface,
            imu_interface=imu_interface,
            cmd_topics={
                self.cmd_topic: (String_, lambda: String_(data="")),
            },
            writable_components=(
                body_interface.component_id,
                base_group_interface.component_id,
            ),
            embedded_interfaces=embedded_interfaces,
            additional_interfaces=(base_group_interface,),
            additional_state_topics={
                WHOLE_BODY_SDK_ODOM_STATE_TOPIC: SportModeState_,
            },
            start_timeout=start_timeout,
            close_timeout=DDS_CLOSE_TIMEOUT,
        )

    @override
    def decode_component_states(
        self,
        state_msgs: Mapping[str, object],
    ) -> dict[str, ComponentStates]:
        component_states = super().decode_component_states(state_msgs)
        component_states[self.base_group_interface.component_id] = (
            self.base_group_interface.read_states_from_msgs(
                (
                    state_msgs[LOW_STATE_TOPIC],
                    state_msgs[WHOLE_BODY_SDK_ODOM_STATE_TOPIC],
                )
            )
        )
        return component_states

    @override
    def start(self) -> None:
        """Start DDS and initialize the MotionSwitcher controller."""
        super().start()
        try:
            self.motion_service_switcher.initialize()
        except Exception:
            self.stop()
            raise

    @override
    def stop(self) -> None:
        """Restore the MotionSwitcher service, then stop DDS I/O."""
        try:
            self.motion_service_switcher.shutdown()
        except Exception as exc:
            logger_mp.warning(
                "[%s] Failed to restore WholeBodySdk MotionSwitcher on stop: %s",
                type(self).__name__,
                exc,
            )
        super().stop()

    def enter_control(self) -> None:
        """Enter the configured MotionSwitcher service."""
        self.motion_service_switcher.select_control_service()

    def exit_control(self, *, restore_standing: bool = False) -> None:
        """Keep Teleop for a standing exit, otherwise restore the baseline."""
        if restore_standing:
            self.motion_service_switcher.wait_for_service_statuses(
                {"teleop": 0, "ai_sport": 1},
                motion_mode="",
            )
            # Runtime stops WBC publication after this check. Its later
            # shutdown must also leave Teleop running for R3 control.
            self.motion_service_switcher.restore_required = False
            logger_mp.info("Standing exit: keeping Teleop running for R3.")
            return
        self.motion_service_switcher.restore_service()

    @override
    def encode_component_cmds(
        self,
        component_cmds: Mapping[str, ComponentCmds],
        cmd_msgs: Mapping[str, object],
    ) -> None:
        """Validate component commands and fill the whole-body JSON message."""
        body_cmds = component_cmds[self.body_interface.component_id]
        received_body_schema_keys = set(body_cmds)
        if received_body_schema_keys != set(self.body_cmd_schema_keys):
            raise RuntimeError(
                f"{type(self).__name__} requires body schema keys "
                f"{tuple(sorted(self.body_cmd_schema_keys))!r}, got "
                f"{tuple(sorted(received_body_schema_keys))!r}."
            )
        base_group_cmds = component_cmds[
            self.base_group_interface.component_id
        ]
        if tuple(base_group_cmds) != (self.base_group_schema_key,):
            raise RuntimeError(
                f"{type(self).__name__} requires base-group schema key "
                f"{self.base_group_schema_key!r}, got "
                f"{tuple(base_group_cmds)!r}."
            )
        base_group_cmd = base_group_cmds[self.base_group_schema_key]

        payload = self.build_cmd_payload(body_cmds, base_group_cmd)
        cmd_msgs[self.cmd_topic].data = json.dumps(
            payload,
            separators=(",", ":"),
        )

    def build_cmd_payload(
        self,
        body_cmds: ComponentCmds,
        base_group_cmd: object,
    ) -> dict[str, object]:
        """Build JSON-ready arm position, torque, and base command arrays.

        Args:
            body_cmds: Arm commands keyed by body schema key.
            base_group_cmd: Driver-neutral base-group command.

        Returns:
            Mapping containing ``arm_q``, ``arm_tau``, and ``base_cmd`` lists.
        """
        if not isinstance(base_group_cmd, BaseGroupCmd):
            raise TypeError(
                "whole_body_sdk base group resource requires BaseGroupCmd."
            )
        if base_group_cmd.body_height is None:
            raise ValueError("whole_body_sdk base_group cmd requires body_height.")
        arm_q = np.concatenate(
            [
                body_cmds[schema_key].qpos
                for schema_key in self.body_cmd_schema_keys
            ]
        )
        arm_tau = np.concatenate(
            [
                (
                    np.zeros_like(body_cmds[schema_key].qpos)
                    if body_cmds[schema_key].qtau is None
                    else body_cmds[schema_key].qtau
                )
                for schema_key in self.body_cmd_schema_keys
            ]
        )
        for name in (
            "body_linear_vel_x", "body_linear_vel_y", "body_angular_vel_z",
            "waist_yaw_qpos", "waist_pitch_qpos", "waist_roll_qpos",
        ):
            if getattr(base_group_cmd, name) is None:
                raise ValueError(f"whole_body_sdk requires {name}.")
        # Unitree whole-body base_cmd order:
        # [body vx, body vy, body yaw rate, waist yaw, waist pitch,
        #  waist roll, body height]. No vertical velocity is sent here.
        base_group_cmd_array = np.asarray(
            (
                base_group_cmd.body_linear_vel_x,
                base_group_cmd.body_linear_vel_y,
                base_group_cmd.body_angular_vel_z,
                base_group_cmd.waist_yaw_qpos,
                base_group_cmd.waist_pitch_qpos,
                base_group_cmd.waist_roll_qpos,
                base_group_cmd.body_height,
            ),
            dtype=np.float32,
        )
        return {
            "arm_q": arm_q.tolist(),
            "arm_tau": arm_tau.tolist(),
            "base_cmd": base_group_cmd_array.tolist(),
        }
