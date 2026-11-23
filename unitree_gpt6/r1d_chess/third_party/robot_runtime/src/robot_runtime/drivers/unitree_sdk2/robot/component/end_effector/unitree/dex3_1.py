"""Unitree Dex3-1 joint-group mappings.

``Dex3_1Interface`` converts HG ``HandState_`` motor feedback into
driver-neutral joint states and fills ``HandCmd_`` messages with position,
effort, gain, motor-ID, and RIS status fields.
"""

from collections.abc import Mapping

import numpy as np
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_

from robot_runtime.interfaces.robot.component import (
    ComponentCmds,
    ComponentInterface,
    ComponentStates,
)
from robot_runtime.interfaces.robot.state.joint_group import JointGroupState


class Dex3_1Interface(ComponentInterface):
    """Map Dex3-1 hand states and commands.

    Each schema key selects an ordered subset of the hand's local motor slots.
    Command initialization encodes the motor ID in mode bits 0..3 and the RIS
    status in bits 4..6 before applying schema-keyed position targets.

    Args:
        component_id: Stable ID of the mapped Dex3-1 component.
        motor_slots: Total number of motors in each hand message.
        motor_indices: Local motor indices for every joint-group schema key.
        kp: Position gains indexed by local motor index.
        kd: Velocity gains indexed by local motor index.

    Attributes:
        motor_slots: Total local motor count.
        motor_indices: Schema-keyed local motor mappings.
        kp: Position gains indexed by local motor index.
        kd: Velocity gains indexed by local motor index.

    Example:
        interface = Dex3_1Interface(
            component_id="end_effector.unitree.dex3_1.left",
            motor_slots=7,
            motor_indices={"left_ee": tuple(range(7))},
            kp=np.full(7, 1.5, dtype=np.float32),
            kd=np.full(7, 0.2, dtype=np.float32),
        )
        cmd_msg = interface.CMD_MSG_FACTORY()
        interface.write_cmds_to_msgs(
            (cmd_msg,),
            {"left_ee": joint_group_cmd},
        )
    """

    MODE_STATUS = 0x01
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
        # Local motor indices keyed by schema key.
        self.motor_indices = motor_indices
        self.kp = kp
        self.kd = kd

    def read_states_from_msgs(
        self,
        state_msgs: tuple[HandState_],
    ) -> ComponentStates:
        """Decode Dex3-1 motor feedback into joint-group states."""
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
        """Initialize RIS motor modes and encode joint-group commands."""
        (cmd_msg,) = cmd_msgs
        for motor_index in range(self.motor_slots):
            motor_cmd = cmd_msg.motor_cmd[motor_index]
            # Dex3 RIS mode packs motor id in bits 0..3 and status in bits 4..6.
            motor_cmd.mode = (motor_index & 0x0F) | (
                (self.MODE_STATUS & 0x07) << 4
            )
            motor_cmd.q = 0.0
            motor_cmd.dq = 0.0
            motor_cmd.tau = 0.0
            motor_cmd.kp = float(self.kp[motor_index])
            motor_cmd.kd = float(self.kd[motor_index])
        for schema_key, cmd in component_cmds.items():
            motor_indices = self.motor_indices[schema_key]
            for joint_index, (motor_index, qpos) in enumerate(
                zip(motor_indices, cmd.qpos, strict=True)
            ):
                motor_cmd = cmd_msg.motor_cmd[motor_index]
                motor_cmd.q = qpos
                motor_cmd.tau = (
                    0.0 if cmd.qtau is None else cmd.qtau[joint_index]
                )
