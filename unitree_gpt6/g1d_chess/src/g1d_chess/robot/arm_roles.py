"""Role view for the existing actor/observer transaction engine.

In left-actor mode, logical right is physical left (actor), and logical left
is physical right (observer). Cartesian base coordinates are NEVER mirrored.
All motor feedback, commands, forces and model solutions use the same explicit
permutation. The physical DDS publisher and its fault handling are unchanged.
"""
import numpy as np
from g1d_chess.robot.motion_harness import MotionHarness


def swap_arm_vector(value):
    value = np.asarray(value)
    if value.shape != (14,):
        raise ValueError("arm role mapping requires 14 joints")
    return np.concatenate((value[7:], value[:7]))


def physical_motor(index):
    if 15 <= index <= 21:
        return index + 7
    if 22 <= index <= 28:
        return index - 7
    return {31: 33, 33: 31}.get(index, index)


class MessageView:
    def __init__(self, message):
        self._message = message
        self.motor_state = [message.motor_state[physical_motor(i)]
                            for i in range(len(message.motor_state))]

    def __getattr__(self, name):
        return getattr(self._message, name)


class LeftActorDDS:
    def __init__(self, physical):
        object.__setattr__(self, "_physical", physical)

    def __getattr__(self, name):
        value = getattr(self._physical, name)
        return swap_arm_vector(value) if name in ("arm_target", "arm_sent", "tau") else value

    def __setattr__(self, name, value):
        if name in ("arm_target", "arm_sent", "tau"):
            value = swap_arm_vector(value)
        setattr(self._physical, name, value)

    def state(self):
        q, dq, message, age = self._physical.state()
        order = [physical_motor(i) for i in range(len(q))]
        return q[order].copy(), dq[order].copy(), MessageView(message), age

    def command_arm(self, q, tau):
        return self._physical.command_arm(swap_arm_vector(q), swap_arm_vector(tau))

    def command_gripper(self, side, target):
        if side not in ("left", "right"):
            raise ValueError("unknown logical gripper")
        return self._physical.command_gripper("left" if side == "right" else "right", target)

    physical_motor = staticmethod(physical_motor)


class LeftActorMotion(MotionHarness):
    def __init__(self, physical):
        self.physical = physical

    def fk(self, q):
        poses = self.physical.fk(swap_arm_vector(q))
        return [poses[1].copy(), poses[0].copy()]

    def solve(self, targets, seed, check=True):
        return swap_arm_vector(self.physical.solve(
            [targets[1], targets[0]], swap_arm_vector(seed), check=check))

    def tau(self, q):
        return swap_arm_vector(self.physical.tau(swap_arm_vector(q)))
