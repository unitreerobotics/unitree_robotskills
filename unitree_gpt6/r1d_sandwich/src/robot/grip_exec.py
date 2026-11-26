"""Execution layer: r1-ik deploy's Executor, plus grippers in the same lowcmd frame and an operator stop.

Nothing here changes what r1-ik/deploy checks; it only adds:
  * GripperFrameBuilder -- motors 31/33 follow a commanded opening that PERSISTS across motions.
    The stock FrameBuilder holds the grippers at their *measured* position each motion, which drops the
    squeeze on a held object (fingers blocked by the object read open, so "hold measured" = let go).
    Here the commanded target is kept until changed, ramped at GRIP_RATE. Gains stay r1_d's (kp 5 /
    kd 0.05): a soft position servo, so a closed target on a slice squeezes gently instead of crushing.
  * StoppableExecutor -- a stop Event checked every tick; set -> WatchdogTrip -> the executor's own
    soft stop (hold last measured pose 0.5 s, then stop sending).
Every real motion is run_shadow -> run_real on the SAME executor and builder state (single-use token).
"""
from __future__ import annotations

import threading

import numpy as np

from service.paths import ensure_external_paths

ensure_external_paths()

from robot import ik_executor as X  # noqa: E402

X.set_lift_rate(0.03)      # m/s: verified on the real lift at 30 and 45 mm/s (2026-11-14, r1-ik deploy README)
GRIP_MAX = 5.4            # motor rad, fully open (r1_d LIMITS, 0 = closed)
GRIP_RATE = 8.0           # rad/s commanded ramp: full stroke in ~0.7 s
GRIP_KP, GRIP_KD = 5.0, 0.05   # r1_d low_command values for non-arm targets (verified on this robot)
SIDE_MOTOR = {"left": X.GRIPPER_MOTORS[0], "right": X.GRIPPER_MOTORS[1]}
HEAD_PITCH, HEAD_YAW = X.HEAD_MOTORS            # 29 (+ = nose down), 30
HEAD_LIMIT = {HEAD_PITCH: (-0.60, 0.60), HEAD_YAW: (-0.50, 0.50)}   # official ±0.6283 / ±2.0071, kept tighter
HEAD_RATE = 0.4                                 # rad/s ramp (r1_d hold gains: pitch kp 50 kd 2, yaw kp 10 kd 0.1)
RAMP = {**{m: GRIP_RATE for m in X.GRIPPER_MOTORS}, HEAD_PITCH: HEAD_RATE, HEAD_YAW: HEAD_RATE}
# Grasp squeeze limit: while grasping, the command never runs more than GRASP_SQUEEZE rad ahead of the measured
# finger position. A blocked finger with the target at 0 stalls continuously (kp 5 x 0.7 rad) and latched the
# Dex1 over-current fault 0x200 twice on the real robot (2026-11-16); this caps it at kp x 0.3 rad.
GRASP_SQUEEZE = 0.3


class GripperFrameBuilder(X.FrameBuilder):
    def __init__(self, *a, grip_kp=GRIP_KP, grip_kd=GRIP_KD, **kw):
        super().__init__(*a, **kw)
        if not (0.0 < grip_kp <= 40.0 and 0.0 <= grip_kd <= 2.0):
            raise ValueError("gripper gains must be 0 < kp <= 40, 0 <= kd <= 2 (official Dex1-1 initial gains 40 / 2)")
        self.grip_kp, self.grip_kd = float(grip_kp), float(grip_kd)
        self.target = {}      # motor -> commanded q (persisting); empty = hold measured (stock behaviour)
        self.grasping = set() # gripper motors in squeeze-limited grasp mode (persisting until opened)
        self._cur = {}        # motor -> ramped command during one run
        self._last_t = None

    # -- called by Robot before every shadow / real / sim run
    def begin(self, measured_extra):
        self._cur = {m: float(self.target.get(m, measured_extra[m])) for m in RAMP}
        self._goal = dict(self._cur)
        self._ticks = 0

    def set_goal(self, motor, q):
        lo, hi = HEAD_LIMIT.get(motor, (0.0, GRIP_MAX))
        self._goal[motor] = float(np.clip(q, lo, hi))

    def commit(self):
        """After a successful run: what was LAST COMMANDED becomes the persistent target (never the goal: a ramp
        cut short by a short run would otherwise make the next run start with a step)."""
        self.target.update(self._cur)

    def build(self, state, q16, v16, hold_extra, gain_scale=1.0):
        extra = dict(hold_extra)
        if self._cur:
            for m, rate in RAMP.items():
                step = rate * X.DT
                g = self._goal.get(m, self._cur[m])
                self._cur[m] += float(np.clip(g - self._cur[m], -step, step))
                if m in self.grasping:
                    self._cur[m] = max(self._cur[m], float(state.motor_state[m].q) - GRASP_SQUEEZE)
                extra[m] = self._cur[m]
        f = super().build(state, q16, v16, extra, gain_scale)
        # r1_d's low_command already uses kp 5 / kd 0.05 for grippers; pin it explicitly so a change there
        # cannot silently stiffen the squeeze
        changed = False
        for m in X.GRIPPER_MOTORS:
            c = f.msg.motor_cmd[m]
            if c.kp != self.grip_kp or c.kd != self.grip_kd:
                c.kp, c.kd = self.grip_kp, self.grip_kd
                changed = True
        if changed:
            f.msg.crc = self.crc.Crc(f.msg)
        return f


class StoppableExecutor(X.Executor):
    def __init__(self, *a, stop_event: threading.Event | None = None, ignore_fault_motors=(), **kw):
        super().__init__(*a, **kw)
        self.stop_event = stop_event or threading.Event()
        # recovery only: gripper motors whose fault bits are ignored (never arm/waist motors)
        bad = set(ignore_fault_motors) - set(X.GRIPPER_MOTORS)
        if bad:
            raise ValueError(f"only gripper motors {X.GRIPPER_MOTORS} may be ignored, got {sorted(bad)}")
        self.ignore_fault_motors = tuple(ignore_fault_motors)

    def _watchdog(self, state, age, ref16, gap, mode0, link_count, rep, tol):
        if self.stop_event.is_set():
            raise X.WatchdogTrip("operator stop (/stop)")
        for m in self.ignore_fault_motors:      # this sample only (each DDS callback delivers a new message)
            state.motor_state[m].motorstate = 0
        return super()._watchdog(state, age, ref16, gap, mode0, link_count, rep, tol)


class SimGripperRobot(X.SimRobot):
    """SimRobot whose gripper motors follow their command; `blocker(side, q_cmd)` (the sim world) may
    stop a closing finger at an object's width. Arm dynamics as SimRobot (kinematic, no gravity)."""

    def __init__(self, *a, blocker=None, **kw):
        super().__init__(*a, **kw)
        self.blocker = blocker

    def publish(self, msg, capability=None):
        super().publish(msg, capability)
        ms = self.state.motor_state
        for m in X.HEAD_MOTORS:
            c = msg.motor_cmd[m]
            if c.mode == 1 and c.kp > 0:
                ms[m].q += float(np.clip(c.q - ms[m].q, -HEAD_RATE * X.DT * 1.2, HEAD_RATE * X.DT * 1.2))
        for side, m in SIDE_MOTOR.items():
            c = msg.motor_cmd[m]
            if c.mode != 1:
                continue
            q = ms[m].q + float(np.clip(c.q - ms[m].q, -GRIP_RATE * X.DT * 1.2, GRIP_RATE * X.DT * 1.2))
            if self.blocker is not None:
                q = self.blocker(side, q)
            ms[m].dq = (q - ms[m].q) / X.DT
            ms[m].q = q


class FastShadowIO:
    """Read-only view of a robot IO for the executor's shadow run, without the real-time waits.

    Every check of the shadow run is unchanged (frames, CRC, feedforward, limits, fault bits, mode_machine, feedback
    age on live samples); only wait_until() returns at once, so a 4 s motion is pre-checked in ~0.1 s instead of 4 s.
    identity() is the real IO's, so the single-use token is valid for run_real on that IO. It never publishes.
    """

    def __init__(self, io):
        self._io = io

    armed = False

    def identity(self):
        return self._io.identity()

    def now(self):
        return self._io.now()

    def read(self):
        return self._io.read()

    def measured_q16(self):
        return self._io.measured_q16()

    def hold_extra(self):
        return self._io.hold_extra()

    @property
    def height_raw(self):
        return getattr(self._io, "height_raw", None)

    def wait_until(self, deadline):
        return None

    def publish(self, msg, capability=None):
        raise PermissionError("FastShadowIO never publishes")

    def publish_lift(self, vnorm, capability=None):
        raise PermissionError("FastShadowIO never publishes")
