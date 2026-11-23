"""Unitree HG body mappings for shared low-level messages.

``HGBodyInterface`` maps schema-keyed body joint groups to the motor slots in
an HG ``LowState_`` or mutable low-level command message. The owning
``RobotChannel`` selects the transport topics, creates messages, and publishes
the completed command after all components sharing it have written their
slots.
"""

from collections.abc import Mapping

import numpy as np
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HGLowState

from robot_runtime.interfaces.robot.component import (
    ComponentCmds,
    ComponentInterface,
    ComponentStates,
)
from robot_runtime.interfaces.robot.state.joint_group import JointGroupState

HG_MOTOR_MODE = 0x01


class HGBodyInterface(ComponentInterface):
    """Map an HG body's joint groups to shared low-level motor slots.

    ``motor_indices`` is keyed by schema key and preserves the schema-defined
    joint order. ``kp`` and ``kd`` use the complete message's motor-slot
    numbering, so each selected motor index also selects its gains.

    Args:
        component_id: Stable ID of the mapped body component.
        motor_slots: Total number of motor slots in the shared message.
        motor_indices: Motor-slot indices for every joint-group schema key.
        kp: Position gains indexed by motor slot.
        kd: Velocity gains indexed by motor slot.

    Attributes:
        motor_slots: Total shared motor-slot count.
        motor_indices: Schema-keyed motor-slot mappings.
        kp: Position gains indexed by motor slot.
        kd: Velocity gains indexed by motor slot.

    Example:
        interface = HGBodyInterface(
            component_id="body.g1_edu",
            motor_slots=35,
            motor_indices={
                "left_arm": (15, 16, 17, 18, 19, 20, 21),
                "right_arm": (22, 23, 24, 25, 26, 27, 28),
            },
            kp=np.zeros(35, dtype=np.float32),
            kd=np.zeros(35, dtype=np.float32),
        )
        states = interface.read_states_from_msgs((lowstate_msg,))
        left_arm_state = states["left_arm"]
    """

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
        # Body motor indices keyed by schema key.
        self.motor_indices = motor_indices
        self.motor_slots = motor_slots
        self.kp = kp
        self.kd = kd

    def read_states_from_msgs(
        self,
        state_msgs: tuple[HGLowState],
    ) -> ComponentStates:
        """Decode configured HG motor slots into joint-group states."""
        (state_msg,) = state_msgs
        component_states = {}
        for schema_key, motor_indices in self.motor_indices.items():
            motor_states = [
                state_msg.motor_state[motor_index]
                for motor_index in motor_indices
            ]
            component_states[schema_key] = JointGroupState(
                num_joints=len(motor_indices),
                qpos=np.asarray(
                    [state.q for state in motor_states],
                    dtype=np.float32,
                ),
                qvel=np.asarray(
                    [state.dq for state in motor_states],
                    dtype=np.float32,
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
        cmd_msgs: tuple[object],
        component_cmds: ComponentCmds,
    ) -> None:
        """Write joint-group commands into their shared HG motor slots."""
        (cmd_msg,) = cmd_msgs
        for schema_key, cmd in component_cmds.items():
            motor_indices = self.motor_indices[schema_key]
            for joint_index, (motor_index, qpos) in enumerate(
                zip(motor_indices, cmd.qpos, strict=True)
            ):
                motor_cmd = cmd_msg.motor_cmd[motor_index]
                motor_cmd.mode = HG_MOTOR_MODE
                motor_cmd.q = qpos
                motor_cmd.dq = 0.0
                motor_cmd.tau = (
                    0.0
                    if cmd.qtau is None
                    else cmd.qtau[joint_index]
                )
                motor_cmd.kp = self.kp[motor_index]
                motor_cmd.kd = self.kd[motor_index]
