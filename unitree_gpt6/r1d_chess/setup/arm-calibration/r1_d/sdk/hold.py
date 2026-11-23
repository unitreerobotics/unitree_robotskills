"""Feedforward torque for the waist and head joints: bias, friction and a slow integral.

No SDK imports: the same class runs in the 250 Hz session here and in the 50 Hz chess
service. Parameters come from assets/r1d/hold_joints.json (tools/hold_calib.py fit).
"""

import math

JOINTS = ("waist_yaw", "head_pitch", "head_yaw")


class HoldCompensator:
    """One joint. torque() is called once per control cycle."""

    def __init__(self, params):
        self.coulomb = float(params.get("coulomb", 0.0))  # Nm, dry friction while moving
        self.viscous = float(params.get("viscous", 0.0))  # Nm s/rad
        # Position dependent holding torque: constant + sin + cos of the joint angle.
        self.bias = tuple(float(v) for v in params.get("bias", (0.0, 0.0, 0.0)))
        self.ki = float(params.get("ki", 0.0))  # Nm/(rad s)
        self.integral_limit = float(params.get("integral_limit", 0.0))  # Nm
        self.deadzone = float(params.get("deadzone", 0.001))  # rad, no integration inside
        self.limit = float(params.get("limit", 0.0))  # Nm, bound of the sum
        self.fade = float(params.get("fade_seconds", 0.5))
        # Extra stiffness and damping on top of the motor PD gains (Nm/rad, Nm s/rad).
        self.stiffness = float(params.get("stiffness", 0.0))
        self.damping = float(params.get("damping", 0.0))
        self.creep = float(params.get("creep", 0.004))  # rad/s, measured speed counted as moving
        self.reset()

    def reset(self):
        self.tripped = False  # oscillation seen: stiffness, damping and integral stay off
        self.swings = []  # times at which the velocity error changed sign with some size
        self.swing_sign = 0.0
        self.clock = 0.0
        self.integral = 0.0
        self.direction = 0.0  # -1..1, fades to 0 at rest

    def static(self, q):
        c, s, k = self.bias
        return c + s * math.sin(q) + k * math.cos(q)

    def torque(self, q, target, target_velocity, dt, dq=0.0):
        if abs(target_velocity) > 1e-4:
            self.direction = math.copysign(1.0, target_velocity)
            self.integral = 0.0
        else:
            self.direction = math.copysign(
                max(0.0, abs(self.direction) - dt / self.fade), self.direction
            )
            error = target - q
            if error * self.integral < 0.0:  # passed the target: the stored torque is too much
                self.integral = 0.0
            # Only a joint that is stuck needs more torque: once it creeps, wait.
            if abs(self.direction) == 0.0 and abs(error) > self.deadzone and abs(dq) < self.creep:
                self.integral += self.ki * error * dt
                self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        # Oscillation guard: four reversals of a velocity error above 0.08 rad/s within a
        # second can only be the extra feedback loop ringing; it is then switched off.
        self.clock += dt
        swing = dq - target_velocity
        if (self.stiffness or self.damping) and abs(swing) > 0.3 and swing * self.swing_sign <= 0.0:
            self.swing_sign = math.copysign(1.0, swing)
            self.swings = [t for t in self.swings if self.clock - t < 1.0] + [self.clock]
            if len(self.swings) >= 4:
                self.tripped = True
        if self.tripped:
            self.integral = 0.0
            tau = self.static(target) + self.viscous * target_velocity
            return max(-self.limit, min(self.limit, tau))
        tau = (
            self.static(target)
            + self.coulomb * self.direction
            + self.viscous * target_velocity
            + self.integral
            + self.stiffness * (target - q)
            + self.damping * (target_velocity - dq)
        )
        return max(-self.limit, min(self.limit, tau))
