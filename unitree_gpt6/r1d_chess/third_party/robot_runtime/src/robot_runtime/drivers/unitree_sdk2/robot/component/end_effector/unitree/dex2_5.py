"""Internal dex2_5 tendon hand: two torque-controlled motors per hand."""

from collections.abc import Mapping

import numpy as np
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

from robot_runtime.interfaces.robot.component import ComponentCmds, ComponentInterface, ComponentStates
from robot_runtime.interfaces.robot.state.joint_group import JointGroupState


class Dex2_5Interface(ComponentInterface):
    """Map fingers/thumb to shared HG slots without position control.

    Encoder q/dq are diagnostic motor readings, not finger joint coordinates.
    Positive tau closes; negative tau opens. Zero torque does not open the hand.
    """

    def __init__(
        self,
        *,
        component_id: str,
        motor_indices: Mapping[str, tuple[int, ...]],
        shared_motor_indices: Mapping[str, tuple[int, ...]],
        qtau_limits: Mapping[str, tuple[tuple[float, float], ...]],
    ) -> None:
        super().__init__(component_id=component_id)
        self.motor_indices = motor_indices
        self.message_motor_indices = shared_motor_indices
        self.qtau_limits = qtau_limits

    def read_states_from_msgs(self, state_msgs: tuple[LowState_]) -> ComponentStates:
        (msg,) = state_msgs
        states = {}
        for key, indices in self.message_motor_indices.items():
            motors = [msg.motor_state[index] for index in indices]
            states[key] = JointGroupState(
                num_joints=len(indices),
                qpos=np.asarray([motor.q for motor in motors], dtype=np.float32),
                qvel=np.asarray([motor.dq for motor in motors], dtype=np.float32),
                qtau=np.asarray([motor.tau_est for motor in motors], dtype=np.float32),
                casing_temp=np.asarray([motor.temperature[0] for motor in motors], dtype=np.float32),
                winding_temp=np.asarray([motor.temperature[1] for motor in motors], dtype=np.float32),
            )
        return states

    def write_cmds_to_msgs(self, cmd_msgs: tuple[LowCmd_], component_cmds: ComponentCmds) -> None:
        (msg,) = cmd_msgs
        for key, cmd in component_cmds.items():
            if cmd.qpos is not None or cmd.qtau is None:
                raise ValueError("dex2_5 requires qtau-only commands.")
            limits = np.asarray(self.qtau_limits[key])
            # Also bound direct interface users; q/kp/kd can never enable a position loop.
            qtau = np.clip(cmd.qtau, np.maximum(limits[:, 0], -0.3), np.minimum(limits[:, 1], 0.3))
            for slot, tau in zip(self.message_motor_indices[key], qtau, strict=True):
                motor = msg.motor_cmd[slot]
                motor.mode = 1
                motor.q = motor.dq = motor.kp = motor.kd = 0.0
                motor.tau = float(tau)
