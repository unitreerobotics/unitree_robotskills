"""HG motor and lift message encoding; chassis movement is never enabled."""

from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, geometry_msgs_msg_dds__Twist_
from unitree_sdk2py.utils.crc import CRC
from .model import GROUPS, HOLD, ARM_KD, ARM_KP, ARM_TAU_LIMIT, finite, validate_targets

ARM_DQ_LIMIT = 1.0  # rad/s, bound of the commanded arm joint velocity


HOLD_GAINS = ((50.0, 3.0), (50.0, 2.0), (10.0, 0.1))  # waist yaw, head pitch, head yaw
HOLD_TAU_LIMIT = (6.0, 3.0, 0.6)  # Nm, bound of the waist/head feedforward


def low_command(
    state, targets, hold_positions, feedforward=None, velocity=None, hold_tau=None, hold_gains=None
):
    """Encode PD gains, mode_machine and CRC. Other slots disabled.

    feedforward optionally maps left_arm/right_arm to seven bounded torques (Nm),
    velocity to the seven trajectory velocities (rad/s) the damping term refers to.
    """
    targets = validate_targets(targets)
    msg = unitree_hg_msg_dds__LowCmd_()
    msg.mode_pr = 0
    msg.mode_machine = state.mode_machine
    for index, (kp, kd), limit in zip(HOLD, hold_gains or HOLD_GAINS, HOLD_TAU_LIMIT):
        motor = msg.motor_cmd[index]
        motor.mode, motor.q, motor.kp, motor.kd = 1, hold_positions[index], kp, kd
        tau = finite((hold_tau or {}).get(index, 0.0), "hold feedforward")
        if abs(tau) > limit:
            raise ValueError(f"motor {index}: feedforward torque {tau:.2f} Nm exceeds {limit} Nm")
        motor.tau = tau
    for name, values in targets.items():
        for j, (index, q) in enumerate(zip(GROUPS[name], values)):
            motor = msg.motor_cmd[index]
            motor.mode, motor.q = 1, q
            motor.kp = ARM_KP[j] if name.endswith("arm") else 5.0
            motor.kd = ARM_KD if name.endswith("arm") else 0.05
    for name, values in (feedforward or {}).items():
        if not name.endswith("arm") or name not in GROUPS or len(values) != len(GROUPS[name]):
            raise ValueError(f"Feedforward torque is only supported for complete arms: {name}")
        for index, tau, limit in zip(GROUPS[name], values, ARM_TAU_LIMIT):
            if abs(finite(tau, name + " feedforward")) > limit:
                raise ValueError(f"{name}: feedforward torque {tau:.2f} Nm exceeds {limit} Nm")
            msg.motor_cmd[index].tau = float(tau)
    for name, values in (velocity or {}).items():
        if not name.endswith("arm") or name not in GROUPS or len(values) != len(GROUPS[name]):
            raise ValueError(f"Velocity is only supported for complete arms: {name}")
        for index, dq in zip(GROUPS[name], values):
            if abs(finite(dq, name + " velocity")) > ARM_DQ_LIMIT:
                raise ValueError(f"{name}: velocity {dq:.2f} rad/s exceeds {ARM_DQ_LIMIT} rad/s")
            msg.motor_cmd[index].dq = float(dq)
    msg.crc = CRC().Crc(msg)
    return msg


def arm_unload_command(state):
    """R1 LowCmd zero-effort frame for arms and grippers only.

    Use mode 1 with zero gains, velocity and feedforward torque as in the
    extracted R1 LowCmd release path. Disabled slots remain untouched.
    """
    msg = unitree_hg_msg_dds__LowCmd_()
    msg.mode_pr = 0
    msg.mode_machine = state.mode_machine
    for ids in GROUPS.values():
        for index in ids:
            motor = msg.motor_cmd[index]
            motor.mode = 1
            motor.q = motor.dq = motor.tau = motor.kp = motor.kd = 0.0
    msg.crc = CRC().Crc(msg)
    return msg


def lift_command(velocity):
    velocity = finite(velocity, "lift velocity")
    if abs(velocity) > 0.1:
        raise ValueError("Lift normalized velocity limited to [-0.1, 0.1]")
    msg = geometry_msgs_msg_dds__Twist_()
    msg.linear.z = velocity  # All chassis components remain zero.
    return msg
