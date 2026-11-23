"""BrainCo Revo2 joint-group mappings over Unitree motor messages.

``Revo2Interface`` converts normalized Revo2 position feedback into physical
joint positions and normalizes physical position commands to the device's
``[0, 1]`` range. The owning ``RobotChannel`` supplies the DDS messages and
transport.
"""

from collections.abc import Mapping

import logging_mp
import numpy as np
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_

from robot_runtime.interfaces.robot.component import (
    ComponentCmds,
    ComponentInterface,
    ComponentStates,
)
from robot_runtime.interfaces.robot.cmd.joint_group import JointGroupCmd
from robot_runtime.interfaces.robot.state.joint_group import JointGroupState


logger_mp = logging_mp.getLogger(__name__)


class Revo2Interface(ComponentInterface):
    """Map BrainCo Revo2 states and position commands.

    Physical joint positions are converted to and from normalized motor
    commands using ``qpos_limits``. Every mapping is keyed by schema key and
    follows the joint order declared by the robot component specification.

    Args:
        component_id: Stable ID of the mapped Revo2 component.
        motor_slots: Total number of motors in each Revo2 message.
        motor_indices: Local motor indices for every joint-group schema key.
        qpos_limits: Per-joint physical position limits by schema key.
        kp: Position gains indexed by local motor index.
        kd: Velocity gains indexed by local motor index.

    Attributes:
        motor_slots: Total local motor count.
        motor_indices: Schema-keyed local motor mappings.
        qpos_limits: Schema-keyed physical position limits.
        kp: Position gains indexed by local motor index.
        kd: Velocity gains indexed by local motor index.

    Example:
        interface = Revo2Interface(
            component_id="end_effector.brainco.revo2.left",
            motor_slots=6,
            motor_indices={"left_ee": tuple(range(6))},
            qpos_limits={
                "left_ee": (
                    (0.0, 1.52),
                    (0.0, 1.05),
                    (0.0, 1.47),
                    (0.0, 1.47),
                    (0.0, 1.47),
                    (0.0, 1.47),
                )
            },
            kp=np.zeros(6, dtype=np.float32),
            kd=np.zeros(6, dtype=np.float32),
        )
        cmd_msg = interface.CMD_MSG_FACTORY()
        interface.write_cmds_to_msgs(
            (cmd_msg,),
            {
                "left_ee": JointGroupCmd(
                    num_joints=6,
                    qpos=np.zeros(6, dtype=np.float32),
                )
            },
        )
    """

    CMD_MSG_TYPE = MotorCmds_
    STATE_MSG_TYPE = MotorStates_
    CMD_MSG_FACTORY = MotorCmds_

    def __init__(
        self,
        *,
        component_id: str,
        motor_slots: int,
        motor_indices: Mapping[str, tuple[int, ...]],
        qpos_limits: Mapping[str, tuple[tuple[float, float], ...]],
        kp: np.ndarray,
        kd: np.ndarray,
    ) -> None:
        super().__init__(component_id=component_id)
        self.motor_slots = motor_slots
        # Local motor indices keyed by schema key.
        self.motor_indices = motor_indices
        self.qpos_limits = qpos_limits
        self.kp = kp
        self.kd = kd

    def read_states_from_msgs(
        self,
        state_msgs: tuple[MotorStates_],
    ) -> ComponentStates:
        """Decode Revo2 feedback and denormalize its joint positions."""
        (state_msg,) = state_msgs
        component_states = {}
        for schema_key, motor_indices in self.motor_indices.items():
            states = [state_msg.states[index] for index in motor_indices]
            qpos_limits = self.qpos_limits[schema_key]
            component_states[schema_key] = JointGroupState(
                num_joints=len(motor_indices),
                qpos=np.asarray(
                    [
                        minimum + state.q * (maximum - minimum)
                        for state, (minimum, maximum) in zip(
                            states,
                            qpos_limits,
                            strict=True,
                        )
                    ],
                    dtype=np.float32,
                ),
                qvel=np.asarray(
                    [state.dq for state in states], dtype=np.float32
                ),
                qtau=np.asarray(
                    [state.tau_est for state in states], dtype=np.float32
                ),
                casing_temp=np.asarray(
                    [state.temperature for state in states], dtype=np.float32
                ),
            )
        return component_states

    def write_cmds_to_msgs(
        self,
        cmd_msgs: tuple[MotorCmds_],
        component_cmds: ComponentCmds,
    ) -> None:
        """Normalize physical positions and fill a Revo2 command message."""
        (cmd_msg,) = cmd_msgs
        received_schema_keys = tuple(component_cmds)
        expected_schema_keys = tuple(self.motor_indices)
        if set(received_schema_keys) != set(expected_schema_keys):
            raise RuntimeError(
                f"{type(self).__name__} {self.component_id!r} requires "
                f"joint groups {expected_schema_keys!r}, got "
                f"{received_schema_keys!r}."
            )

        cmd_msg.cmds = [
            unitree_go_msg_dds__MotorCmd_() for _ in range(self.motor_slots)
        ]
        for motor_index, motor_cmd in enumerate(cmd_msg.cmds):
            motor_cmd.q = 0.0
            motor_cmd.dq = 1.0
            motor_cmd.tau = 0.0
            motor_cmd.kp = float(self.kp[motor_index])
            motor_cmd.kd = float(self.kd[motor_index])
        for schema_key, motor_indices in self.motor_indices.items():
            cmd = component_cmds[schema_key]
            if not isinstance(cmd, JointGroupCmd):
                raise TypeError(
                    f"{type(self).__name__} requires JointGroupCmd values."
                )
            qpos_limits = np.asarray(
                self.qpos_limits[schema_key],
                dtype=np.float64,
            )
            clipped_qpos = np.clip(
                cmd.qpos,
                qpos_limits[:, 0],
                qpos_limits[:, 1],
            )
            if np.any(cmd.qpos != clipped_qpos):
                logger_mp.warning(
                    "[%s] %s qpos command clipped: requested=%s "
                    "clipped=%s limits=%s",
                    self.component_id,
                    schema_key,
                    cmd.qpos.tolist(),
                    clipped_qpos.tolist(),
                    qpos_limits.tolist(),
                )
            for motor_index, qpos, (minimum, maximum) in zip(
                motor_indices,
                clipped_qpos,
                self.qpos_limits[schema_key],
                strict=True,
            ):
                normalized_q = (qpos - minimum) / (maximum - minimum)
                cmd_msg.cmds[motor_index].q = min(
                    max(float(normalized_q), 0.0),
                    1.0,
                )
