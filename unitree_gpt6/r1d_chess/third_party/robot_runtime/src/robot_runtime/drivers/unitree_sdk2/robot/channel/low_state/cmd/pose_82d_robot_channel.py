"""Unitree FSM teleoperation channel for 82-dimensional pose commands.

``Pose82dRobotChannel`` decodes the shared HG LowState input and publishes
``Pose82dCmd`` values as compact JSON strings on Unitree's FSM teleoperation
topic. It also owns the FSM control lifecycle.
"""

from __future__ import annotations

import json
from collections.abc import Mapping

import logging_mp
import numpy as np
from typing_extensions import override
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

from robot_runtime.drivers.unitree_sdk2.robot.channel.low_state.low_state_robot_channel import (
    LowStateRobotChannel,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.body.hg_body import (
    HGBodyInterface,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.sensor.imu import (
    IMUInterface,
)
from robot_runtime.drivers.unitree_sdk2.robot.motion.aisport import (
    AISportFSM,
)
from robot_runtime.interfaces.robot.component import ComponentCmds, ComponentInterface
from robot_runtime.interfaces.robot.cmd.pose_82d import Pose82dCmd

logger_mp = logging_mp.getLogger(__name__)

DDS_CLOSE_TIMEOUT = 2.0
POSE_82D_CMD_TOPIC = "rt/fsm/teleop/cmd"
POSE_82D_SCHEMA_KEY = "pose_82d"


class Pose82dRobotChannel(LowStateRobotChannel):
    """Publish structured humanoid pose commands under FSM control.

    One command contains 24 SMPL joint vectors, the root quaternion, and two
    three-axis wrist rotations. These values are flattened into the 82-value
    ``frame`` expected by the Unitree teleoperation endpoint.

    Args:
        body_interface: Interface decoding body state from HG LowState.
        imu_interface: Interface decoding the embedded HG IMU state.
        control_fsm_id: FSM entered while pose commands are active.
        restore_fsm_id: FSM restored when control ends.
        include_frame_id: Whether each JSON payload includes a sequence ID.
        start_timeout: Seconds to wait for the first LowState message.

    Attributes:
        cmd_topic: Fixed FSM teleoperation command topic.
        aisport_fsm: AISport FSM initialization and transitions.
        next_frame_id: Sequence ID for the next H2 pose payload.

    Example:
        channel = Pose82dRobotChannel(
            body_interface=body_interface,
            imu_interface=imu_interface,
            control_fsm_id=505,
            restore_fsm_id=801,
        )
        channel.encode_component_cmds(
            {
                body_interface.component_id: {"pose_82d": pose_cmd},
            },
            {channel.cmd_topic: string_msg},
        )
        payload = json.loads(string_msg.data)
    """

    def __init__(
        self,
        *,
        body_interface: HGBodyInterface,
        imu_interface: IMUInterface,
        control_fsm_id: int,
        restore_fsm_id: int,
        include_frame_id: bool = False,
        start_timeout: float = 10.0,
        additional_interfaces: tuple[ComponentInterface, ...] = (),
    ) -> None:
        self.cmd_topic = POSE_82D_CMD_TOPIC
        self.aisport_fsm = AISportFSM(
            control_fsm_id=control_fsm_id,
            restore_fsm_id=restore_fsm_id,
        )
        self.include_frame_id = include_frame_id
        self.next_frame_id = 0
        super().__init__(
            body_interface=body_interface,
            imu_interface=imu_interface,
            cmd_topics={
                self.cmd_topic: (String_, lambda: String_(data="")),
            },
            writable_components=(
                body_interface.component_id,
                *(interface.component_id for interface in additional_interfaces),
            ),
            additional_interfaces=additional_interfaces,
            start_timeout=start_timeout,
            close_timeout=DDS_CLOSE_TIMEOUT,
        )

    @override
    def start(self) -> None:
        """Start DDS I/O and initialize the FSM controller."""
        super().start()
        try:
            self.aisport_fsm.initialize()
        except Exception:
            self.stop()
            raise

    @override
    def stop(self) -> None:
        """Restore the configured FSM and stop DDS I/O."""
        try:
            self.aisport_fsm.shutdown()
        except Exception as exc:
            logger_mp.warning(
                "[%s] Failed to restore Pose82d FSM on stop: %s",
                type(self).__name__,
                exc,
            )
        super().stop()

    def enter_control(self) -> None:
        """Enter the pose-control FSM."""
        self.aisport_fsm.enter_control_fsm()
        self.next_frame_id = 0

    def exit_control(self) -> None:
        """Leave pose control and restore the configured FSM."""
        self.aisport_fsm.restore_fsm()

    @property
    def control_fsm_id(self) -> int:
        """Return the FSM ID used while pose control is active."""
        return self.aisport_fsm.control_fsm_id

    @property
    def restore_fsm_id(self) -> int:
        """Return the FSM ID restored after pose control."""
        return self.aisport_fsm.restore_fsm_id

    def fsm_state(self) -> tuple[int, object]:
        """Return the current Unitree FSM ID and state description."""
        return self.aisport_fsm.fsm_state()

    @override
    def write_component_cmds(
        self,
        component_cmds: Mapping[str, ComponentCmds],
    ) -> None:
        """Publish one command and advance the optional protocol frame ID."""
        super().write_component_cmds(component_cmds)
        if self.include_frame_id:
            self.next_frame_id += 1

    @override
    def encode_component_cmds(
        self,
        component_cmds: Mapping[str, ComponentCmds],
        cmd_msgs: Mapping[str, object],
    ) -> None:
        """Encode one ``Pose82dCmd`` as the FSM teleoperation JSON payload."""
        cmds_by_schema_key = component_cmds[
            self.body_interface.component_id
        ]
        if set(cmds_by_schema_key) != {POSE_82D_SCHEMA_KEY}:
            raise RuntimeError(
                f"{type(self).__name__} requires schema key "
                f"{POSE_82D_SCHEMA_KEY!r}."
            )
        cmd = cmds_by_schema_key[POSE_82D_SCHEMA_KEY]
        if not isinstance(cmd, Pose82dCmd):
            raise TypeError(
                f"{POSE_82D_SCHEMA_KEY!r} requires Pose82dCmd, "
                f"got {type(cmd).__name__}."
            )
        pose_82d = np.concatenate(
            (
                cmd.smpl_joints.reshape(-1),
                cmd.smpl_root_quat_wxyz,
                cmd.wrist_qpos,
            )
        )
        payload_data = {"frame": pose_82d.tolist(), "name": "pico_smpl"}
        if self.include_frame_id:
            payload_data["frame_id"] = self.next_frame_id
        cmd_msgs[self.cmd_topic].data = json.dumps(
            payload_data,
            separators=(",", ":"),
        )
