"""G1 AISport 36D/82D control on the FSM topic with DDS RPC walking input."""

import json
from collections.abc import Mapping

import numpy as np
from typing_extensions import override
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

from robot_runtime.drivers.unitree_sdk2.robot.channel.low_state.cmd.pose_82d_robot_channel import (
    Pose82dRobotChannel,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.body.hg_body import HGBodyInterface
from robot_runtime.drivers.unitree_sdk2.robot.component.base_group.g1_edu_pseudo_base import (
    G1EduPseudoBaseInterface,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.sensor.imu import IMUInterface
from robot_runtime.interfaces.robot.cmd.base_group import BaseGroupCmd
from robot_runtime.interfaces.robot.cmd.joint_group import JointGroupCmd
from robot_runtime.interfaces.robot.cmd.pose_82d import Pose82dCmd
from robot_runtime.interfaces.robot.component import ComponentCmds


class HybridWBCRobotChannel(Pose82dRobotChannel):
    """Reuse the LowState transport and confirmed 801/505 FSM lifecycle.

    Both submodes use one publisher; changing ``mode`` does not change FSMs.
    Walking goes through the SDK's sport DDS RPC, with a checked return code.
    Runtime validates ActionData.hybrid_mode against pose_82d presence before
    conversion; this channel encodes that command shape into the JSON mode.
    """

    def __init__(
        self,
        *,
        body_interface: HGBodyInterface,
        imu_interface: IMUInterface,
        base_group_interface: G1EduPseudoBaseInterface,
        control_fsm_id: int = 505,
        restore_fsm_id: int = 801,
        start_timeout: float = 10.0,
    ) -> None:
        if body_interface.component_id != "body.g1_edu":
            raise ValueError("Hybrid WBC only supports body.g1_edu.")
        if (control_fsm_id, restore_fsm_id) != (505, 801):
            raise ValueError("G1 hybrid WBC requires FSM 505 and restore FSM 801.")
        self.loco_client: LocoClient | None = None
        self.last_mode: int | None = None
        self.base_group_interface = base_group_interface
        super().__init__(
            body_interface=body_interface,
            imu_interface=imu_interface,
            control_fsm_id=control_fsm_id,
            restore_fsm_id=restore_fsm_id,
            include_frame_id=True,
            start_timeout=start_timeout,
            additional_interfaces=(base_group_interface,),
        )

    @override
    def enter_control(self) -> None:
        if self.loco_client is None:
            loco_client = LocoClient()
            loco_client.SetTimeout(0.01)
            loco_client.Init()
            self.loco_client = loco_client
        super().enter_control()
        self.last_mode = None

    @override
    def exit_control(self) -> None:
        try:
            if self.aisport_fsm.restore_required and self.loco_client is not None:
                self.move_base(np.zeros(3))
        finally:
            super().exit_control()
            self.last_mode = None

    @override
    def stop(self) -> None:
        try:
            self.exit_control()
        finally:
            super().stop()
            self.loco_client = None

    def move_base(self, velocity: np.ndarray) -> None:
        if self.loco_client is None:
            raise RuntimeError("Hybrid WBC walking client is not initialized.")
        # Move() discards SetVelocity's result; call the same API directly so
        # a failed walking/stop request reaches the runtime's error handling.
        code = self.loco_client.SetVelocity(*(float(v) for v in velocity), 1.0)
        if code != 0:
            raise RuntimeError(f"Hybrid WBC velocity RPC failed with code {code}.")

    @override
    def write_component_cmds(
        self,
        component_cmds: Mapping[str, ComponentCmds],
    ) -> None:
        if set(component_cmds) != self.writable_components:
            raise ValueError("Hybrid WBC requires body and base_group commands.")
        mode = int("pose_82d" in component_cmds[self.body_interface.component_id])
        base_cmds = component_cmds[self.base_group_interface.component_id]
        if set(base_cmds) != {self.base_group_interface.schema_key}:
            raise ValueError("Hybrid WBC requires the configured base_group.")
        base_cmd = base_cmds[self.base_group_interface.schema_key]
        if not isinstance(base_cmd, BaseGroupCmd):
            raise TypeError("Hybrid WBC requires BaseGroupCmd.")
        if (
            base_cmd.body_height is not None or base_cmd.lift_linear_vel_z is not None
            or any(value is not None for value in (
                base_cmd.waist_roll_qpos, base_cmd.waist_pitch_qpos, base_cmd.waist_yaw_qpos,
            ))
        ):
            raise ValueError("Hybrid WBC base_group only accepts planar velocity.")
        velocity = np.asarray((
            base_cmd.body_linear_vel_x, base_cmd.body_linear_vel_y, base_cmd.body_angular_vel_z,
        ))
        if mode == 1 and np.any(velocity):
            raise ValueError("Hybrid WBC pose_82d requires zero base velocity.")
        with self.state_lock:
            msg = String_(data="")
            self.encode_component_cmds(
                component_cmds, {self.cmd_topic: msg},
            )
        changed = self.last_mode is not None and mode != self.last_mode
        if changed:
            self.move_base(np.zeros(3))
        with self.dds_io_lock:
            if not self.is_connected():
                raise RuntimeError("Hybrid WBC channel is not connected.")
            if not self.cmd_publishers[self.cmd_topic].Write(msg):
                raise RuntimeError("Hybrid WBC DDS write failed.")
            self.next_frame_id += 1
            self.last_mode = mode
        if mode == 0 and not changed:
            self.move_base(velocity)

    @override
    def encode_component_cmds(
        self,
        component_cmds: Mapping[str, ComponentCmds],
        cmd_msgs: Mapping[str, object],
    ) -> None:
        cmds = component_cmds[self.body_interface.component_id]
        mode = int("pose_82d" in cmds)
        required_keys = {"left_arm", "right_arm"}
        if mode == 1:
            required_keys.add("pose_82d")
        if set(cmds) != required_keys:
            raise ValueError(f"Hybrid WBC mode={mode} requires {sorted(required_keys)}.")
        arms = []
        for key in ("left_arm", "right_arm"):
            arm = cmds[key]
            if not isinstance(arm, JointGroupCmd) or arm.qpos.shape != (7,):
                raise ValueError(f"Hybrid WBC {key} requires 7 joint targets.")
            arms.append(arm.qpos)
        arm_q_ref = np.concatenate(arms).astype(np.float32)
        if mode == 0:
            # DDSRobotChannel holds state_lock while encoding; use its coherent
            # decoded snapshot directly instead of acquiring that lock again.
            states = self.component_states[self.body_interface.component_id]
            joints = np.concatenate(tuple(
                states[key].qpos for key in
                ("left_leg", "right_leg", "waist", "left_arm", "right_arm")
            )).astype(np.float32)
            if joints.shape != (29,) or not np.all(np.isfinite(joints)):
                raise ValueError("Hybrid WBC requires 29 finite LowState joints.")
            joints[15:29] = arm_q_ref
            frame = np.concatenate(([0, 0, 0, 0, 0, 0, 1], joints))
        else:
            pose = cmds["pose_82d"]
            if not isinstance(pose, Pose82dCmd):
                raise TypeError("pose_82d requires Pose82dCmd.")
            frame = np.concatenate((
                pose.smpl_joints.reshape(-1),
                pose.smpl_root_quat_wxyz,
                pose.wrist_qpos,
            ))
        cmd_msgs[self.cmd_topic].data = json.dumps(
            {
                "mode": mode,
                "name": "teleop_robot" if mode == 0 else "teleop_smpl",
                "frame": frame.tolist(),
                "arm_q_ref": arm_q_ref.tolist(),
                "frame_id": self.next_frame_id,
            },
            separators=(",", ":"),
        )
