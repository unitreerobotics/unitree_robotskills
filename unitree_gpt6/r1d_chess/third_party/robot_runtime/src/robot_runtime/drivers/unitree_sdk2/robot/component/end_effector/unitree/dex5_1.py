"""Unitree Dex5-1 joint-group mappings.

``Dex5_1Interface`` converts HG ``HandState_`` feedback into driver-neutral
joint states and fills ``HandCmd_`` messages with per-joint position, effort,
and gain values. The Dex5 service maps the 20 URDF joints to 16 physical
motors.
"""

from collections.abc import Mapping

import numpy as np
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__HandCmd_,
    unitree_hg_msg_dds__MotorCmd_,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_

from robot_runtime.interfaces.robot.component import (
    ComponentCmds,
    ComponentInterface,
    ComponentStates,
)
from robot_runtime.interfaces.robot.state.joint_group import JointGroupState


class Dex5_1Interface(ComponentInterface):
    """Map Dex5-1 upper-layer hand states and commands.

    Each schema key selects an ordered subset of the 20 URDF joint slots.
    Dex5 firmware converts these joint commands to its 16 physical motors.

    Args:
        component_id: Stable ID of the mapped Dex5-1 component.
        motor_slots: Total number of URDF joints in each hand message.
        motor_indices: Local joint indices for every joint-group schema key.
        kp: Position gains indexed by local joint index.
        kd: Velocity gains indexed by local joint index.

    Attributes:
        motor_slots: Total local joint count.
        motor_indices: Schema-keyed local joint mappings.
        kp: Position gains indexed by local joint index.
        kd: Velocity gains indexed by local joint index.

    Example:
        interface = Dex5_1Interface(
            component_id="end_effector.unitree.dex5_1.left",
            motor_slots=20,
            motor_indices={"left_ee": tuple(range(20))},
            kp=np.full(20, 0.1, dtype=np.float32),
            kd=np.full(20, 0.001, dtype=np.float32),
        )
        cmd_msg = interface.CMD_MSG_FACTORY()
        interface.write_cmds_to_msgs(
            (cmd_msg,),
            {"left_ee": joint_group_cmd},
        )
    """

    POSITION_MODE = 1
    CMD_MSG_TYPE = HandCmd_
    STATE_MSG_TYPE = HandState_
    CMD_MSG_FACTORY = staticmethod(unitree_hg_msg_dds__HandCmd_)

    def __init__(
        self,
        *,
        component_id: str,
        motor_slots: int,
        motor_indices: Mapping[str, tuple[int, ...]],
        kp: np.ndarray,
        kd: np.ndarray,
    ) -> None:
        super().__init__(component_id=component_id)
        self.motor_slots = motor_slots
        # Local URDF joint indices keyed by schema key.
        self.motor_indices = motor_indices
        self.kp = kp
        self.kd = kd

    def read_states_from_msgs(
        self,
        state_msgs: tuple[HandState_],
    ) -> ComponentStates:
        """Decode Dex5-1 motor feedback into joint-group states."""
        (state_msg,) = state_msgs
        component_states = {}
        for schema_key, motor_indices in self.motor_indices.items():
            motor_states = [
                state_msg.motor_state[index] for index in motor_indices
            ]
            component_states[schema_key] = JointGroupState(
                num_joints=len(motor_indices),
                qpos=np.asarray(
                    [state.q for state in motor_states], dtype=np.float32
                ),
                qvel=np.asarray(
                    [state.dq for state in motor_states], dtype=np.float32
                ),
                qtau=np.asarray(
                    [state.tau_est for state in motor_states],
                    dtype=np.float32,
                ),
                casing_temp=np.asarray(
                    [state.temperature[0] for state in motor_states],
                    dtype=np.float32,
                ),
                winding_temp=np.asarray(
                    [state.temperature[1] for state in motor_states],
                    dtype=np.float32,
                ),
            )
        return component_states

    def write_cmds_to_msgs(
        self,
        cmd_msgs: tuple[HandCmd_],
        component_cmds: ComponentCmds,
    ) -> None:
        """Initialize all URDF joint slots and encode joint commands."""
        (cmd_msg,) = cmd_msgs
        cmd_msg.motor_cmd = []
        for motor_index in range(self.motor_slots):
            motor_cmd = unitree_hg_msg_dds__MotorCmd_()
            motor_cmd.mode = self.POSITION_MODE
            motor_cmd.q = 0.0
            motor_cmd.dq = 0.0
            motor_cmd.tau = 0.0
            motor_cmd.kp = float(self.kp[motor_index])
            motor_cmd.kd = float(self.kd[motor_index])
            cmd_msg.motor_cmd.append(motor_cmd)
        for schema_key, cmd in component_cmds.items():
            motor_indices = self.motor_indices[schema_key]
            for joint_index, (motor_index, qpos) in enumerate(
                zip(motor_indices, cmd.qpos, strict=True)
            ):
                motor_cmd = cmd_msg.motor_cmd[motor_index]
                motor_cmd.q = float(qpos)
                motor_cmd.tau = (
                    0.0
                    if cmd.qtau is None
                    else float(cmd.qtau[joint_index])
                )
