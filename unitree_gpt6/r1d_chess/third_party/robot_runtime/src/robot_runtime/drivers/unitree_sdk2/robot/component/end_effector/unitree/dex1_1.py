"""Unitree Dex1-1 mappings for external and internal wiring.

External Dex1-1 wiring uses dedicated ``MotorStates_`` and ``MotorCmds_``
messages. Internal wiring shares the body's HG ``LowState_`` and ``LowCmd_``
motor arrays. ``Dex1_1Interface`` presents both transports as the same
schema-keyed joint-group state and command mapping.

"""

from collections.abc import Mapping
from typing import Literal

import numpy as np
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    LowCmd_ as HGLowCmd,
    LowState_ as HGLowState,
)

from robot_runtime.interfaces.robot.component import (
    ComponentCmds,
    ComponentInterface,
    ComponentStates,
)
from robot_runtime.interfaces.robot.state.joint_group import JointGroupState

DexWiring = Literal["external", "internal"]
HG_MOTOR_MODE = 0x01


class Dex1_1Interface(ComponentInterface):
    """Map Dex1-1 motor states and commands for either wiring layout.

    ``motor_indices`` always uses the Dex component's local motor numbering.
    For internal wiring, ``shared_motor_indices`` maps those same schema keys
    to slots in the body's shared ``LowState_`` and ``LowCmd_`` messages. For
    external wiring it is omitted and message slots equal local indices.

    Args:
        component_id: Stable ID of the mapped Dex1-1 component.
        wiring: ``"external"`` for dedicated messages or ``"internal"`` for
            shared HG low-level messages.
        motor_slots: Total number of motors in the standalone Dex component.
        motor_indices: Local motor indices for every joint-group schema key.
        shared_motor_indices: Optional shared HG message slots by schema key.
        kp: Position gains indexed by local motor index.
        kd: Velocity gains indexed by local motor index.

    Attributes:
        wiring: Selected external or internal wiring layout.
        motor_slots: Total local motor count.
        motor_indices: Schema-keyed local motor mappings.
        message_motor_indices: Schema-keyed indices in the active message.
        kp: Position gains indexed by local motor index.
        kd: Velocity gains indexed by local motor index.

    Example:
        interface = Dex1_1Interface(
            component_id="end_effector.unitree.dex1_1.left",
            wiring="internal",
            motor_slots=1,
            motor_indices={"left_ee": (0,)},
            shared_motor_indices={"left_ee": (31,)},
            kp=np.array([1.0], dtype=np.float32),
            kd=np.array([0.1], dtype=np.float32),
        )
        assert interface.motor_indices["left_ee"] == (0,)
        assert interface.message_motor_indices["left_ee"] == (31,)
    """

    CMD_MSG_TYPE = MotorCmds_
    STATE_MSG_TYPE = MotorStates_
    CMD_MSG_FACTORY = MotorCmds_

    def __init__(
        self,
        *,
        component_id: str,
        wiring: DexWiring,
        motor_slots: int,
        motor_indices: Mapping[str, tuple[int, ...]],
        shared_motor_indices: Mapping[str, tuple[int, ...]] | None = None,
        kp: np.ndarray,
        kd: np.ndarray,
    ) -> None:
        super().__init__(component_id=component_id)
        self.wiring = wiring
        self.motor_slots = motor_slots
        # Component-local motor indices keyed by schema key.
        self.motor_indices = motor_indices
        # External messages use local indices; internal wiring overrides them
        # with the component's slots in the shared LowState/LowCmd messages.
        self.message_motor_indices = (
            motor_indices
            if shared_motor_indices is None
            else shared_motor_indices
        )
        self.kp = kp
        self.kd = kd

    def read_states_from_msgs(
        self,
        state_msgs: tuple[MotorStates_ | HGLowState],
    ) -> ComponentStates:
        """Decode dedicated or shared Dex1-1 motor states by schema key."""
        (state_msg,) = state_msgs
        component_states = {}
        for schema_key, motor_indices in self.message_motor_indices.items():
            if self.wiring == "external":
                motor_states = [
                    state_msg.states[index] for index in motor_indices
                ]
                casing_temp = np.asarray(
                    [state.temperature for state in motor_states],
                    dtype=np.float32,
                )
                winding_temp = None
            else:
                motor_states = [
                    state_msg.motor_state[index] for index in motor_indices
                ]
                casing_temp = np.asarray(
                    [state.temperature[0] for state in motor_states],
                    dtype=np.float32,
                )
                winding_temp = np.asarray(
                    [state.temperature[1] for state in motor_states],
                    dtype=np.float32,
                )
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
                casing_temp=casing_temp,
                winding_temp=winding_temp,
            )
        return component_states

    def write_cmds_to_msgs(
        self,
        cmd_msgs: tuple[MotorCmds_ | HGLowCmd],
        component_cmds: ComponentCmds,
    ) -> None:
        """Write Dex1-1 commands into dedicated or shared motor slots."""
        (cmd_msg,) = cmd_msgs
        if self.wiring == "external":
            cmd_msg.cmds = [
                unitree_go_msg_dds__MotorCmd_()
                for _ in range(self.motor_slots)
            ]
            motor_cmds = cmd_msg.cmds
            for motor_index, motor_cmd in enumerate(motor_cmds):
                motor_cmd.q = 0.0
                motor_cmd.dq = 0.0
                motor_cmd.tau = 0.0
                motor_cmd.kp = float(self.kp[motor_index])
                motor_cmd.kd = float(self.kd[motor_index])
        else:
            motor_cmds = cmd_msg.motor_cmd

        for schema_key, cmd in component_cmds.items():
            for joint_index, (
                motor_index,
                channel_motor_index,
                qpos,
            ) in enumerate(
                zip(
                    self.motor_indices[schema_key],
                    self.message_motor_indices[schema_key],
                    cmd.qpos,
                    strict=True,
                )
            ):
                motor_cmd = motor_cmds[channel_motor_index]
                if self.wiring == "internal":
                    motor_cmd.mode = HG_MOTOR_MODE
                motor_cmd.q = qpos
                motor_cmd.dq = 0.0
                motor_cmd.tau = (
                    0.0 if cmd.qtau is None else cmd.qtau[joint_index]
                )
                motor_cmd.kp = float(self.kp[motor_index])
                motor_cmd.kd = float(self.kd[motor_index])
