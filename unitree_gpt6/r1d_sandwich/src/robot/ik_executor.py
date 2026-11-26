# Copied from r1-ik deploy/ik_executor.py (90285d2); only imports and paths changed. Keep in sync by copying; do not restyle.
"""IK-to-lowcmd execution bridge for R1-D (r1_a7_d / r1-ik variant a7d).

Input : IK waypoints addressed BY JOINT NAME: (names, [q, ...]). Recommended upstream:
        ReducedModel("a7d", "arm_waist", locked_q={"lift_stage1_joint": measured_height})
        -> 15 names (waist + 14 arm); the lift is filled from the measured height. A 16-name
        "wbc" solution is accepted only if its lift equals the measured height (wbc ignores
        locked_q).
Output: rt/lowcmd frames at 250 Hz: r1_d PD gains + calibrated gravity/friction feedforward,
        CRC'd. Waist motor 13 follows IK (kp 50 / kd 3); head and grippers hold their measured
        positions; the lift is never commanded.

SAFETY (summary in README.md)
* Any rejected waypoint rejects the WHOLE plan (allow_skip=True to skip explicitly). Limits are
  never clamped. Each segment moves every joint at most MAX_SEGMENT_STEP rad.
* Feedforward pre-scan over the whole path: any frame needing more than FF_LIMIT (wrist 1.5 Nm,
  others ARM_TAU_LIMIT) refuses the plan.
* Tracking tolerance per arm joint from kp*tol + FF_LIMIT + kd*DQ_MARGIN <= URDF effort, scaled by
  1/gain_scale (cap 0.15 rad) so reduced-gain runs keep the same PD torque budget. Waist 2.5°.
* After planning, the executor waits for fresh feedback and re-checks the start pose before the
  first frame (planning can stall the DDS callback thread).
* Watchdog every tick: tracking, feedback age, cycle gap (40 ms), mode_machine, fault bits,
  link-loss bit31 > LINK_LOST_FRAMES. Any trip or exception soft-stops; soft-stop never raises.
* Publishing to DdsRobotIO requires a capability that only run_real creates, after a passed,
  unexpired, single-use shadow token from a read-only DdsRobotIO on the same interface/domain
  with identical execution parameters, plus confirm_supervised=True.
"""
from __future__ import annotations

import hashlib
import math
import secrets
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# unitree_sdk2py is installed, or UNITREE_SDK2_ROOT points at a checkout (service.paths). The original
# inserted two absolute paths here and imported these three modules as r1_d.sdk.*.
from unitree_sdk2py.utils.crc import CRC                                   # noqa: E402
from robot.protocol import low_command, arm_unload_command               # noqa: E402
from robot.dynamics import ArmGravity                                   # noqa: E402
from robot.model import (                                               # noqa: E402
    GROUPS, ARM_KP, ARM_KD, ARM_TAU_LIMIT, LIMITS, calibrated_urdf, urdf as OFFICIAL_URDF,
)

INTERFACE = "enp4s0"
HZ = 250.0
DT = 1.0 / HZ

ARM_JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
              "wrist_roll", "wrist_pitch", "wrist_yaw")
IK_ACTIVE_NAMES = ("lift_stage1_joint", "waist_yaw_joint") + tuple(
    f"{s}_{j}_joint" for s in ("left", "right") for j in ARM_JOINTS)      # 16, wbc order
ARM_WAIST_NAMES = IK_ACTIVE_NAMES[1:]                                     # 15, arm_waist order
I_LIFT, I_WAIST = 0, 1
L_SL, R_SL = slice(2, 9), slice(9, 16)
WAIST_MOTOR, HEAD_MOTORS, GRIPPER_MOTORS = 13, (29, 30), (31, 33)
ARM_MOTORS = GROUPS["left_arm"] + GROUPS["right_arm"]
WAIST_KP, WAIST_KD = 150.0, 8.0     # raised from r1_d HOLD 50/3: kp 50 let arm reaction push the waist 2.5° (real robot, 2026-11-13); effort 60 Nm
LIFT_LOCK_TOL = 0.002               # m

# planning
PEAK_RATE = 0.3            # default peak joint speed, rad/s (Executor(peak_rate=...) to change)
PEAK_RATE_MAX = 0.8        # hard cap; r1_d ARM_DQ_LIMIT is 1.0
# Driven lift (Executor(drive_lift=True)): the lift is velocity-commanded on rt/cmd_vel (normalised).
# Model fitted to the real robot (2026-11-14 lift_check log, rms 0.62 mm): speed = 0.068 m/s per unit,
# ~20 ms delay, acceleration limited to ~0.15 m/s^2. The acceleration is what matters: a plan that
# reverses at 40 mm/s made the lift overshoot 4 mm and fall 15 mm behind (watchdog trip). So the plan's
# height goes through a reference generator (speed <= LIFT_RATE*1.2, acceleration <= LIFT_ACC), and the
# command is velocity feedforward v_ref / LIFT_UNIT_MPS plus LIFT_KP * (ref - measured).
# dds_io.publish_lift enforces LIFT_VMAX itself (r1_d's lift_command hard-caps at 0.1).
LIFT_RATE = 0.015          # m/s, planned lift speed; set_lift_rate() raises it up to LIFT_RATE_MAX
LIFT_RATE_BASE = LIFT_RATE  # the verified default (real robot 2026-11-14: lag 3.1 mm, end error 1.5 mm)
LIFT_RATE_MAX = 0.045      # m/s (3x base): ~0.66 of full scale at 0.068 m/s per unit
LIFT_UNIT_MPS = 0.068      # m/s per normalised command unit (fitted)
LIFT_ACC = 0.10            # m/s^2, reference acceleration (the lift itself manages ~0.15)
LIFT_VMAX = 1.0            # normalised rt/cmd_vel z cap (was 0.1, r1_d's limit)
LIFT_KP = 80.0             # normalised command per metre of (ref - measured); loop gain 80*0.068 = 5.4/s
LIFT_DEADBAND = 0.0015     # m
LIFT_TRACK_TOL = 0.015     # m, watchdog
LIFT_RANGE = (0.35, 0.84)  # m, r1_d height-test range
LIFT_FLOOR_TOL = 0.002     # m: measurement noise allowed below a start height under LIFT_RANGE[0]
LIFT_MAX_STEP = 0.02       # m per waypoint segment
LIFT_EVERY = 5             # send rt/cmd_vel every 5 ticks (50 Hz, as r1_d)
LIFT_SETTLE_S = 4.0        # after the arm path, keep closing the lift error at most this long
MIN_SEGMENT_S = 1.0
MAX_SEGMENT_STEP = 0.2
FRICTION_VEL_BAND = 0.05
# feedforward limit: r1_d ARM_TAU_LIMIT, wrist pitch/yaw tightened to 1.5 Nm
FF_LIMIT = np.array(ARM_TAU_LIMIT, float)
FF_LIMIT[5] = FF_LIMIT[6] = 1.5

# watchdog
ARM_TRACK_TOL = 0.15
WAIST_TRACK_TOL = math.radians(4.0)    # kp 150: 4° is ~10 Nm of 60; 2.5° tripped on a fast sideways move
# Motor winding temperature (°C, temperature[0]). Holding the forearms level at kp 200 heated the
# elbows to 85/71 °C over an afternoon of tests (2026-11-13); r1_d refuses at 85.
TEMP_START_MAX = 70
TEMP_TRIP = 80
FEEDBACK_MAX_AGE = 0.04
CYCLE_MAX_GAP = 0.04
FRESH_WAIT_S = 0.3                  # max wait for a fresh sample after planning
LINK_LOST_FRAMES = 25
MOTOR_LINK_LOST = 0x80000000
DQ_MARGIN = 0.3
SOFTSTOP_HOLD_S = 0.5
SOFTSTOP_RAMP_S = 1.0
START_TOL = 0.01                    # rad: start drift allowed (planning->loop, shadow->real)
TOKEN_TTL_S = 120.0
ESTOP_HINT = "立即按 L2+B 进入阻尼模式"


def _urdf_effort():
    root = ET.parse(OFFICIAL_URDF).getroot()
    lim = {j.attrib["name"]: j.find("limit") for j in root.findall("joint")}
    arm = tuple(float(lim[f"left_{n}_joint"].attrib["effort"]) for n in ARM_JOINTS)
    return arm, float(lim["waist_yaw_joint"].attrib["effort"]), (
        float(lim["waist_yaw_joint"].attrib["lower"]), float(lim["waist_yaw_joint"].attrib["upper"]))


URDF_EFFORT, WAIST_EFFORT, WAIST_LIMIT = _urdf_effort()
# Official R1-A-D SDK guide/spec: waist yaw ±0.6109 rad (±35°); r1d.urdf says ±2.618. Use the tighter.
WAIST_LIMIT = (max(WAIST_LIMIT[0], -0.6109), min(WAIST_LIMIT[1], 0.6109))
ARM_KP_ARR = np.array(ARM_KP, float)
PD_BUDGET = np.array(URDF_EFFORT) - FF_LIMIT - ARM_KD * DQ_MARGIN
TRACK_TOL = np.minimum(ARM_TRACK_TOL, PD_BUDGET / ARM_KP_ARR)   # at gain_scale 1
assert np.all(PD_BUDGET > 0), "kp/FF/kd budget exceeds URDF effort"


def track_tol(gain_scale):
    """Per-joint arm tolerance. kp is scaled by gain_scale, so tol/gain_scale keeps kp*tol."""
    return np.minimum(ARM_TRACK_TOL, TRACK_TOL / gain_scale)


# ---------------------------------------------------------------- input / planning
def to_q16(names, q, measured_height):
    names = tuple(names)
    q = np.asarray(q, float)
    if len(names) != len(q):
        raise ValueError(f"{len(names)} names vs {len(q)} values")
    unknown = set(names) - set(IK_ACTIVE_NAMES)
    if unknown:
        raise ValueError(f"unknown joints {sorted(unknown)}")
    missing = set(IK_ACTIVE_NAMES[1:]) - set(names)
    if missing:
        raise ValueError(f"missing joints {sorted(missing)}")
    out = np.zeros(16)
    out[I_LIFT] = measured_height
    for n, v in zip(names, q):
        out[IK_ACTIVE_NAMES.index(n)] = v
    return out


def set_lift_rate(rate):
    """Planned lift speed (m/s) for this process, 0.003..LIFT_RATE_MAX."""
    global LIFT_RATE
    rate = float(rate)
    if not 0.003 <= rate <= LIFT_RATE_MAX + 1e-12:
        raise ValueError(f"lift rate {rate} outside [0.003, {LIFT_RATE_MAX}] m/s")
    LIFT_RATE = rate
    return LIFT_RATE


def smooth_lift_column(traj):
    """Zero-phase moving average of the lift heights in a sampled plan (list of (q16, v16), edited in
    place), window 2 * peak lift speed / LIFT_ACC: a reversal at the planned speed then needs at most
    LIFT_ACC, turning a little short of the planned peak (never beyond it, no lag, no net drift)."""
    if len(traj) < 3:
        return
    h = np.array([q[I_LIFT] for q, _ in traj])
    vmax = float(np.max(np.abs(np.diff(h)))) * HZ
    w = int(round(2.0 * vmax / LIFT_ACC * HZ)) | 1
    if w < 3:
        return
    pad = w // 2
    hp = np.concatenate([np.full(pad, h[0]), h, np.full(pad, h[-1])])
    hs = np.convolve(hp, np.ones(w) / w, mode="valid")
    for (q, _), x in zip(traj, hs):
        q[I_LIFT] = x


class LiftRef:
    """Height reference that follows a target height with speed <= 1.2*LIFT_RATE and acceleration
    <= LIFT_ACC, braking in time so it never overshoots a target that stops. Its speed is the
    feedforward for the lift command."""

    def __init__(self, h, lo=None, hi=None):
        self.h, self.v, self.prev_tgt = float(h), 0.0, float(h)
        self.lo = lift_floor(h, 0.0) if lo is None else float(lo)
        self.hi = LIFT_RANGE[1] if hi is None else float(hi)

    def step(self, tgt, dt):
        tgt = float(tgt)
        v_tgt = float(np.clip((tgt - self.prev_tgt) / dt, -LIFT_RATE * 1.2, LIFT_RATE * 1.2))
        self.prev_tgt = tgt
        e = tgt - self.h
        v_brk = math.sqrt(2.0 * LIFT_ACC * abs(e)) * (1.0 if e >= 0 else -1.0)
        corr = 4.0 * e
        if abs(corr) > abs(v_brk):
            corr = v_brk
        v_des = float(np.clip(v_tgt + corr, -LIFT_RATE * 1.2, LIFT_RATE * 1.2))
        # brake for the target: never faster toward it than can stop at it
        if e * v_des > 0 and abs(v_des) > abs(v_brk) and abs(v_tgt) < 1e-6:
            v_des = v_brk
        # never past [lo, hi]: brake so the reference stops at the bound
        v_des = min(v_des, math.sqrt(2.0 * LIFT_ACC * max(0.0, self.hi - self.h)))
        v_des = max(v_des, -math.sqrt(2.0 * LIFT_ACC * max(0.0, self.h - self.lo)))
        self.v += float(np.clip(v_des - self.v, -LIFT_ACC * dt, LIFT_ACC * dt))
        self.h += self.v * dt
        return self.h, self.v


def lift_floor(start_height, tol=LIFT_FLOOR_TOL):
    """Lowest lift a move may command: LIFT_RANGE[0], or the start height if the lift already sits
    lower (moving up from there is fine; going below where it started is not). Targets use tol=0;
    checks allow LIFT_FLOOR_TOL of measurement noise."""
    return min(LIFT_RANGE[0], float(start_height) - tol)


def check_waypoint(q16, measured_height, drive_lift=False, lift_lo=None):
    q = np.asarray(q16, float)
    if q.shape != (16,) or not np.all(np.isfinite(q)):
        return [("shape/finite", str(q.shape), "16 finite values")]
    bad = []
    for side, sl in (("left_arm", L_SL), ("right_arm", R_SL)):
        lo, hi = LIMITS[side]
        for j in range(7):
            v = q[sl.start + j]
            if not lo[j] <= v <= hi[j]:
                bad.append((IK_ACTIVE_NAMES[sl.start + j], round(math.degrees(float(v)), 2),
                            f"[{math.degrees(lo[j]):.1f}, {math.degrees(hi[j]):.1f}]°"))
    if not WAIST_LIMIT[0] <= q[I_WAIST] <= WAIST_LIMIT[1]:
        bad.append(("waist_yaw_joint", round(math.degrees(float(q[I_WAIST])), 2), "waist limit"))
    if drive_lift:
        lo = LIFT_RANGE[0] if lift_lo is None else lift_lo
        if not lo <= q[I_LIFT] <= LIFT_RANGE[1]:
            bad.append(("lift_stage1_joint", round(float(q[I_LIFT]), 4), f"[{lo:.4f}, {LIFT_RANGE[1]}] m"))
    elif abs(q[I_LIFT] - measured_height) > LIFT_LOCK_TOL:
        bad.append(("lift_stage1_joint", round(float(q[I_LIFT]), 4),
                    f"must equal measured {measured_height:.4f} m (use arm_waist + locked lift)"))
    return bad


def seg_time(a, b, peak_rate):
    """Time a segment needs at full speed: arm/waist joints at peak_rate, lift at LIFT_RATE."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    return max(float(np.max(np.abs(b[1:] - a[1:]))) / peak_rate, abs(b[I_LIFT] - a[I_LIFT]) / LIFT_RATE)


def path_seconds(points, peak_rate):
    """Duration of one continuous pass through all points (smoothstep in path length)."""
    d = sum(seg_time(a, b, peak_rate) for a, b in zip(points[:-1], points[1:]))
    return max(MIN_SEGMENT_S, 1.5 * d)


def trajectory(points, T, hz=HZ, peak_rate=PEAK_RATE):
    """Move through every point without stopping: linear between points, one smoothstep
    speed profile over the whole path (accelerate at the start, decelerate at the end).
    The joint with the largest displacement peaks at 1.5*D/T = peak_rate."""
    pts = [np.asarray(p, float) for p in points]
    seg = np.array([seg_time(a, b, peak_rate) for a, b in zip(pts[:-1], pts[1:])])
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = cum[-1]
    n = max(1, int(round(T * hz)))
    prev = pts[0]
    for i in range(n + 1):
        t = i / n
        s = total * t * t * (3.0 - 2.0 * t)
        k = min(int(np.searchsorted(cum, s, side="right")) - 1, len(seg) - 1)
        k = max(k, 0)
        a = 0.0 if seg[k] == 0.0 else (s - cum[k]) / seg[k]
        q = pts[k] + (pts[k + 1] - pts[k]) * min(max(a, 0.0), 1.0)
        v = (q - prev) * hz if i else np.zeros_like(q)
        prev = q
        yield q, v


@dataclass
class Plan:
    ok: bool
    segments: list = field(default_factory=list)
    rejected: list = field(default_factory=list)
    reasons: list = field(default_factory=list)
    start: object = None
    points: list = field(default_factory=list)
    peak_rate: float = PEAK_RATE
    peak_ff: np.ndarray = field(default_factory=lambda: np.zeros(7))


# ---------------------------------------------------------------- frames
@dataclass
class Frame:
    msg: object
    ff: dict
    ff_clipped: list


class FrameBuilder:
    def __init__(self, urdf=calibrated_urdf, friction_share=1.0):
        self.urdf = Path(urdf)
        self.urdf_sha = hashlib.sha256(self.urdf.read_bytes()).hexdigest()
        self.grav = ArmGravity(self.urdf)
        if not 0.0 <= float(friction_share) <= 1.0:
            raise ValueError(f"friction_share must be in [0, 1], got {friction_share}")
        self.friction_share = float(friction_share)
        self.crc = CRC()

    def feedforward_raw(self, side, q7, v7):
        d7 = np.clip(np.asarray(v7) / FRICTION_VEL_BAND, -1.0, 1.0)
        tau = self.grav.torques(side, q7)
        coulomb, viscous = self.grav.friction[side]
        dry = coulomb + self.grav.load_friction[side] * np.abs(tau)
        return tau + self.friction_share * dry * d7 + viscous * np.asarray(v7)

    def clip_feedforward(self, q16, v16):
        ff, raw, clipped = {}, {}, []
        for side, sl in (("left_arm", L_SL), ("right_arm", R_SL)):
            r = self.feedforward_raw(side.split("_")[0], q16[sl], v16[sl])
            raw[side] = r
            clipped += [(side, j, float(r[j])) for j in range(7) if abs(r[j]) > FF_LIMIT[j] + 1e-9]
            ff[side] = [float(x) for x in np.clip(r, -FF_LIMIT, FF_LIMIT)]
        return ff, raw, clipped

    def build(self, state, q16, v16, hold_extra, gain_scale=1.0):
        v16 = np.asarray(v16, float)
        targets = {
            "left_arm": list(q16[L_SL]), "right_arm": list(q16[R_SL]),
            "left_gripper": [hold_extra[GRIPPER_MOTORS[0]]],
            "right_gripper": [hold_extra[GRIPPER_MOTORS[1]]],
        }
        hold = {WAIST_MOTOR: float(q16[I_WAIST]),
                HEAD_MOTORS[0]: hold_extra[HEAD_MOTORS[0]],
                HEAD_MOTORS[1]: hold_extra[HEAD_MOTORS[1]]}
        vel = {"left_arm": list(v16[L_SL]), "right_arm": list(v16[R_SL])}
        ff, _, clipped = self.clip_feedforward(q16, v16)
        msg = low_command(state, targets, hold, feedforward=ff, velocity=vel)
        if not 0.0 < gain_scale <= 1.0:
            raise ValueError(f"gain_scale must be in (0, 1], got {gain_scale}")
        if not 0.0 <= self.friction_share <= 1.0:
            raise ValueError(f"friction_share must be in [0, 1], got {self.friction_share}")
        # r1_d's HOLD gains for the waist (50/3) are too soft once the arms move; use ours.
        msg.motor_cmd[WAIST_MOTOR].kp = WAIST_KP
        msg.motor_cmd[WAIST_MOTOR].kd = WAIST_KD
        if gain_scale != 1.0:
            for i in ARM_MOTORS:
                msg.motor_cmd[i].kp *= gain_scale
                msg.motor_cmd[i].kd *= gain_scale
        msg.crc = self.crc.Crc(msg)
        return Frame(msg, ff, clipped)

    def crc_ok(self, msg):
        return self.crc.Crc(msg) == msg.crc


def make_plan(names, waypoints, start_q16, builder, allow_skip=False, peak_rate=PEAK_RATE,
              drive_lift=False):
    h = float(start_q16[I_LIFT])
    lo = lift_floor(h)
    p = Plan(ok=False, start=np.asarray(start_q16, float))
    kept = []
    for k, w in enumerate(waypoints):
        try:
            q = to_q16(names, w, h)
        except ValueError as e:
            p.rejected.append((k, [("input", str(e), "names/values")])); continue
        bad = check_waypoint(q, h, drive_lift, lift_lo=lo)
        if bad:
            p.rejected.append((k, bad))
        else:
            kept.append(q)
    if p.rejected and not allow_skip:
        k0, b0 = p.rejected[0]
        p.reasons.append(f"{len(p.rejected)} 个路点被拒绝,整个计划拒绝(allow_skip=False);"
                         f"第一个: 路点 {k0} {b0[:3]}")
        return p
    if not kept:
        p.reasons.append("没有有效路点"); return p
    pts = [p.start] + kept
    for i in range(len(pts) - 1):
        step = np.abs(pts[i + 1][1:] - pts[i][1:])
        if step.max() > MAX_SEGMENT_STEP:
            j = int(np.argmax(step)) + 1
            p.reasons.append(f"段 {i}({'实测→路点0' if i == 0 else f'路点{i-1}→{i}'}) "
                             f"{IK_ACTIVE_NAMES[j]} 位移 {math.degrees(step.max()):.1f}° > "
                             f"{math.degrees(MAX_SEGMENT_STEP):.1f}°,请加密路点")
        dh = abs(pts[i + 1][I_LIFT] - pts[i][I_LIFT])
        if dh > LIFT_MAX_STEP:
            p.reasons.append(f"段 {i} 升降位移 {dh*1e3:.0f} mm > {LIFT_MAX_STEP*1e3:.0f} mm,请加密路点")
    if p.reasons:
        return p
    T = path_seconds(pts, peak_rate)
    segs = [(pts[0], pts[-1], T)]                 # one continuous pass; kept for reporting
    p.points = pts
    p.peak_rate = peak_rate
    for q, v in trajectory(pts, T, peak_rate=peak_rate):
        _, raw, clipped = builder.clip_feedforward(q, v)
        for side in raw:
            p.peak_ff = np.maximum(p.peak_ff, np.abs(raw[side]))
        if clipped:
            s, j, r = clipped[0]
            p.reasons.append(f"前馈预扫: {s} {ARM_JOINTS[j]} 需要 {r:.2f} Nm,超出上限 ±{FF_LIMIT[j]} Nm")
            return p
    p.segments = segs
    p.ok = True
    return p


# ---------------------------------------------------------------- simulated robot / fake feedback
class _Motor:
    __slots__ = ("q", "dq", "motorstate", "temperature")

    def __init__(self, q=0.0):
        self.q, self.dq, self.motorstate = q, 0.0, 0
        self.temperature = [40, 40]


class SimState:
    def __init__(self, n=35, mode_machine=11):
        self.motor_state = [_Motor() for _ in range(n)]
        self.mode_machine = mode_machine


class SimRobot:
    """Kinematic first-order follower: NO gravity, NO friction, NO payload. Control-logic tests only;
    its torque numbers are not evidence for the real robot. Shares the IO interface with
    dds_io.DdsRobotIO: identity(), now(), read(), measured_q16(), hold_extra(), height_raw,
    publish(msg, capability), wait_until(t). Never touches the network."""

    def __init__(self, start_q16, head=(0.0, 0.0), gripper=(0.52, 0.50), lag=0.06):
        self.state = SimState()
        ms = self.state.motor_state
        q = np.asarray(start_q16, float)
        for i, v in zip(ARM_MOTORS, list(q[L_SL]) + list(q[R_SL])):
            ms[i].q = float(v)
        ms[WAIST_MOTOR].q = float(q[I_WAIST])
        ms[HEAD_MOTORS[0]].q, ms[HEAD_MOTORS[1]].q = head
        ms[GRIPPER_MOTORS[0]].q, ms[GRIPPER_MOTORS[1]].q = gripper
        self.height = float(q[I_LIFT])
        self.height_raw = self.height * 1000.0
        self.alpha = 1.0 - math.exp(-DT / lag)
        self.t, self.stamp, self.sent = 0.0, 0.0, 0
        self.faults = {}
        self.fail_publish = False

    def identity(self):
        return ("sim",)

    armed = False

    def _arm(self, capability):
        self.armed = True

    def _disarm(self):
        self.armed = False

    def inject(self, name, at, **kw):
        self.faults[name] = (at, kw)

    def _on(self, name):
        return name in self.faults and self.t >= self.faults[name][0]

    def now(self):
        return self.t

    def read(self):
        if self._on("link_loss"):
            until = self.faults["link_loss"][1].get("until")
            m = self.state.motor_state[GROUPS["right_arm"][6]]
            m.motorstate = 0 if (until is not None and self.t >= until) else MOTOR_LINK_LOST
        if self._on("motor_fault"):
            self.state.motor_state[GROUPS["right_arm"][2]].motorstate = 0x1
        if self._on("mode_change"):
            self.state.mode_machine = 12
        if self._on("overheat"):
            self.state.motor_state[GROUPS["right_arm"][3]].temperature = [82, 70]
        return self.state, self.t - self.stamp

    lift_v = 0.0                    # m/s, actual
    lift_v_cmd = 0.0
    LIFT_MPS_PER_UNIT = LIFT_UNIT_MPS   # fitted to the real lift (2026-11-14)
    LIFT_PHYS_ACC = 0.15                # m/s^2, fitted

    def publish_lift(self, vnorm, capability=None):
        if abs(vnorm) > LIFT_VMAX + 1e-9:
            raise ValueError(f"lift command {vnorm} > {LIFT_VMAX}")
        self.lift_v_cmd = 0.0 if self._on("stuck_lift") else vnorm * self.LIFT_MPS_PER_UNIT
        if self._on("stuck_lift"):
            self.lift_v = 0.0
        self.lift_sent = getattr(self, "lift_sent", 0) + 1

    def publish(self, msg, capability=None):
        if self.fail_publish:
            raise OSError("simulated DDS write failure")
        self.sent += 1
        self.lift_v += float(np.clip(self.lift_v_cmd - self.lift_v, -self.LIFT_PHYS_ACC * DT,
                                     self.LIFT_PHYS_ACC * DT))
        self.height += self.lift_v * DT
        self.height_raw = self.height * 1000.0
        ms = self.state.motor_state
        for i in ARM_MOTORS + (WAIST_MOTOR,):
            if self._on("stuck_elbow") and i == GROUPS["left_arm"][3]:
                continue
            if self._on("stuck_waist") and i == WAIST_MOTOR:
                continue
            c = msg.motor_cmd[i]
            if c.kp <= 0.0:
                continue
            new = ms[i].q + (c.q - ms[i].q) * self.alpha
            ms[i].dq = (new - ms[i].q) / DT
            ms[i].q = new
        if self._on("fail_after"):
            self.fail_publish = True

    def wait_until(self, deadline):
        if self._on("stall"):
            at, kw = self.faults.pop("stall")
            self.t += kw.get("seconds", 0.05)
        self.t = max(self.t, deadline)
        if not self._on("stale_feedback"):
            self.stamp = self.t

    def measured_q16(self):
        ms = self.state.motor_state
        q = np.zeros(16)
        q[I_LIFT], q[I_WAIST] = self.height, ms[WAIST_MOTOR].q
        q[L_SL] = [ms[i].q for i in GROUPS["left_arm"]]
        q[R_SL] = [ms[i].q for i in GROUPS["right_arm"]]
        return q

    def hold_extra(self):
        ms = self.state.motor_state
        return {i: float(ms[i].q) for i in HEAD_MOTORS + GRIPPER_MOTORS}


# ---------------------------------------------------------------- executor
class WatchdogTrip(RuntimeError):
    pass


@dataclass
class Report:
    mode: str = ""
    plan_ok: bool = False
    plan_reasons: list = field(default_factory=list)
    rejected: list = field(default_factory=list)
    segments: list = field(default_factory=list)
    plan_ms: float = 0.0
    post_plan_age_ms: float = 0.0       # feedback age right after planning, before re-sync
    peak_ff: np.ndarray = field(default_factory=lambda: np.zeros(7))
    height_raw: float | None = None     # lift reading as received (height_state.z)
    height_m: float | None = None       # after the /1000 conversion
    frames: int = 0
    published: int = 0
    crc_bad: int = 0
    ff_clip_count: int = 0
    max_dq: float = 0.0
    max_track_err: np.ndarray = field(default_factory=lambda: np.zeros(7))
    max_waist_err: float = 0.0
    max_total_torque: np.ndarray = field(default_factory=lambda: np.zeros(7))
    first_cmd_jump: float = 0.0
    max_lift_err: float = 0.0
    lift_cmds: int = 0
    max_lateness_ms: float = 0.0
    trip: str | None = None
    softstop_frames: int = 0
    softstop_failed: str | None = None
    errors: list = field(default_factory=list)
    shadow_token: str | None = None
    log_t: list = field(default_factory=list)      # per frame: time (s)
    log_cmd: list = field(default_factory=list)    # per frame: commanded q16
    log_meas: list = field(default_factory=list)   # per frame: measured q16 (feedback used that tick)


class Executor:
    def __init__(self, builder=None, release_on_trip=False, gain_scale=1.0, peak_rate=PEAK_RATE,
                 temp_guard=True, drive_lift=False):
        if not 0.0 < gain_scale <= 1.0:
            raise ValueError(f"gain_scale must be in (0, 1], got {gain_scale}")
        if not 0.0 < peak_rate <= PEAK_RATE_MAX:
            raise ValueError(f"peak_rate must be in (0, {PEAK_RATE_MAX}] rad/s, got {peak_rate}")
        self._peak_rate = float(peak_rate)
        self.drive_lift = bool(drive_lift)  # True: follow the plan's lift column on rt/cmd_vel
        self.temp_guard = bool(temp_guard)   # False: skip the 70/80 °C checks (motor fault bits still trip)
        self.b = builder or FrameBuilder()
        self.release_on_trip = bool(release_on_trip)
        self._gain_scale = float(gain_scale)
        self._tokens = {}                # token -> (params digest, start16, io identity, issued)
        self._raise_at = None            # test hook

    @property
    def peak_rate(self):
        r = self._peak_rate
        if not 0.0 < r <= PEAK_RATE_MAX:
            raise ValueError(f"peak_rate must be in (0, {PEAK_RATE_MAX}] rad/s, got {r}")
        return r

    @property
    def gain_scale(self):
        """Read-only after construction; re-validated on every use."""
        g = self._gain_scale
        if not 0.0 < g <= 1.0:
            raise ValueError(f"gain_scale must be in (0, 1], got {g}")
        return g

    # -- execution parameters that change what gets sent
    def params_digest(self, names, waypoints, allow_skip):
        h = hashlib.sha256()
        h.update(repr(tuple(names)).encode())
        for w in waypoints:
            h.update(np.round(np.asarray(w, float), 6).tobytes())
        h.update(repr((self.gain_scale, bool(allow_skip), self.release_on_trip,
                       str(self.b.urdf), self.b.urdf_sha, self.b.friction_share,
                       HZ, self.peak_rate, MAX_SEGMENT_STEP, tuple(FF_LIMIT), tuple(ARM_KP), ARM_KD,
                       WAIST_KP, WAIST_KD, self.temp_guard, self.drive_lift, LIFT_RATE, LIFT_KP,
                       LIFT_TRACK_TOL)).encode())
        return h.hexdigest()

    # -- watchdog
    def _watchdog(self, state, age, ref16, gap, mode0, link_count, rep, tol):
        if gap > CYCLE_MAX_GAP:
            raise WatchdogTrip(f"cycle gap {gap * 1e3:.1f} ms > {CYCLE_MAX_GAP * 1e3:.0f} ms")
        if age > FEEDBACK_MAX_AGE:
            raise WatchdogTrip(f"feedback age {age * 1e3:.1f} ms > {FEEDBACK_MAX_AGE * 1e3:.0f} ms")
        if state.mode_machine != mode0:
            raise WatchdogTrip(f"mode_machine changed {mode0} -> {state.mode_machine}")
        ms = state.motor_state
        for i in ARM_MOTORS + (WAIST_MOTOR,) + GRIPPER_MOTORS + HEAD_MOTORS:
            if self.temp_guard and ms[i].temperature[0] >= TEMP_TRIP:
                raise WatchdogTrip(f"motor {i} temperature {ms[i].temperature[0]} °C >= {TEMP_TRIP} °C")
            s = ms[i].motorstate
            if s & ~MOTOR_LINK_LOST:
                raise WatchdogTrip(f"motor {i} fault bits 0x{s:x}")
            link_count[i] = link_count.get(i, 0) + 1 if s & MOTOR_LINK_LOST else 0
            if link_count[i] > LINK_LOST_FRAMES:
                raise WatchdogTrip(f"motor {i} link lost {link_count[i]} frames > {LINK_LOST_FRAMES}")
        for sl, ids in ((L_SL, GROUPS["left_arm"]), (R_SL, GROUPS["right_arm"])):
            for j, i in enumerate(ids):
                err = abs(ms[i].q - ref16[sl.start + j])
                rep.max_track_err[j] = max(rep.max_track_err[j], err)
                if err > tol[j]:
                    raise WatchdogTrip(f"motor {i} tracking {math.degrees(err):.2f}° > "
                                       f"{math.degrees(tol[j]):.2f}°")
        werr = abs(ms[WAIST_MOTOR].q - ref16[I_WAIST])
        rep.max_waist_err = max(rep.max_waist_err, werr)
        if werr > WAIST_TRACK_TOL:
            raise WatchdogTrip(f"waist tracking {math.degrees(werr):.2f}° > "
                               f"{math.degrees(WAIST_TRACK_TOL):.2f}°")

    def _send(self, io, msg, cap, rep):
        io.publish(msg, cap)
        rep.published += 1

    def _lift_reset(self, h, lo=None, hi=None):
        self._lref = LiftRef(h, lo, hi)

    def _lift_step(self, io, q_target16, k, send, cap, rep):
        """Drive the lift toward the plan's height (called every tick; rt/cmd_vel every LIFT_EVERY):
        LiftRef smooths the target, command = feedforward + P on (ref - measured).
        Returns (target - measured, ref height). Trips the watchdog when the lift is LIFT_TRACK_TOL
        away from the reference."""
        h = float(io.measured_q16()[I_LIFT])
        if getattr(self, "_lref", None) is None:
            self._lift_reset(h)
        href, vref = self._lref.step(q_target16[I_LIFT], DT)
        err = href - h
        rep.max_lift_err = max(rep.max_lift_err, abs(err))
        if abs(err) > LIFT_TRACK_TOL:
            raise WatchdogTrip(f"lift tracking {err*1e3:.1f} mm > {LIFT_TRACK_TOL*1e3:.0f} mm")
        if k % LIFT_EVERY == 0:
            if abs(vref) < 1e-4 and abs(err) < LIFT_DEADBAND:
                vcmd = 0.0
            else:
                vcmd = float(np.clip(vref / LIFT_UNIT_MPS + LIFT_KP * err, -LIFT_VMAX, LIFT_VMAX))
            if send:
                io.publish_lift(vcmd, cap)
                rep.lift_cmds += 1
        return float(q_target16[I_LIFT]) - h, href

    def _lift_zero(self, io, cap, rep):
        for _ in range(3):
            try:
                io.publish_lift(0.0, cap)
                rep.lift_cmds += 1
            except Exception as e:
                rep.softstop_failed = f"lift stop: {type(e).__name__}: {e}"
                print(f"[升降停止失败] {rep.softstop_failed} — {ESTOP_HINT}", flush=True)
                return

    def _softstop(self, io, rep, have_feedback, last_cmd16, send, cap):
        """Hold the last pose for SOFTSTOP_HOLD_S, then stop sending. Never raises: a failed
        publish is recorded and the operator is told to engage damping. The R1-D firmware keeps
        holding the last frame after the stream stops (verified on the real robot, 2026-11-13)."""
        if not send:
            return
        if self.drive_lift:
            self._lift_zero(io, cap, rep)
        try:
            state, _ = io.read()
            hold_q = io.measured_q16() if have_feedback else last_cmd16
            extra = io.hold_extra()
        except Exception as e:
            rep.softstop_failed = f"read: {type(e).__name__}: {e}"
            print(f"[软停失败] {rep.softstop_failed} — {ESTOP_HINT}", flush=True)
            return
        n_hold = int(SOFTSTOP_HOLD_S * HZ)
        n_ramp = int(SOFTSTOP_RAMP_S * HZ) if self.release_on_trip else 0
        deadline = io.now()
        for k in range(n_hold + n_ramp + (1 if self.release_on_trip else 0)):
            try:
                if k < n_hold + n_ramp:
                    f = self.b.build(state, hold_q, np.zeros(16), extra, self.gain_scale)
                    if k >= n_hold:
                        scale = 1.0 - (k - n_hold + 1) / n_ramp
                        for i in ARM_MOTORS:
                            c = f.msg.motor_cmd[i]
                            c.kp *= scale; c.kd *= scale; c.tau *= scale
                        f.msg.crc = CRC().Crc(f.msg)
                    if not self.b.crc_ok(f.msg):
                        rep.crc_bad += 1
                    msg = f.msg
                else:
                    msg = arm_unload_command(state)
                self._send(io, msg, cap, rep)
                rep.softstop_frames += 1
                deadline += DT
                io.wait_until(deadline)
            except Exception as e:
                rep.softstop_failed = f"publish at soft-stop frame {k}: {type(e).__name__}: {e}"
                print(f"[软停失败] {rep.softstop_failed} — {ESTOP_HINT}", flush=True)
                return

    def _wait_fresh(self, io):
        """Planning can starve the DDS callback thread; wait for a fresh sample."""
        end = io.now() + FRESH_WAIT_S
        while True:
            state, age = io.read()
            if age <= FEEDBACK_MAX_AGE / 2:
                return state, age
            if io.now() >= end:
                return state, age
            io.wait_until(io.now() + 0.002)

    # -- core loop shared by sim / shadow / real
    def _execute(self, io, names, waypoints, send, mode, allow_skip=False, cap=None):
        rep = Report(mode=mode)
        state, age = io.read()
        if age > FEEDBACK_MAX_AGE:
            rep.trip = f"start refused: feedback age {age * 1e3:.1f} ms > {FEEDBACK_MAX_AGE * 1e3:.0f} ms"
            return rep, None
        hot = [(i, state.motor_state[i].temperature[0])
               for i in ARM_MOTORS + (WAIST_MOTOR,) + GRIPPER_MOTORS + HEAD_MOTORS
               if state.motor_state[i].temperature[0] >= TEMP_START_MAX]
        if hot and self.temp_guard:
            rep.trip = (f"start refused: motors too hot {hot} (>= {TEMP_START_MAX} °C); "
                        "put the robot in damping (L2+B) and let it cool")
            return rep, None
        rep.height_raw = getattr(io, "height_raw", None)
        start = io.measured_q16()
        rep.height_m = float(start[I_LIFT])
        t0 = time.perf_counter()
        plan = make_plan(names, waypoints, start, self.b, allow_skip, self.peak_rate, self.drive_lift)
        rep.plan_ms = (time.perf_counter() - t0) * 1e3
        rep.plan_ok, rep.plan_reasons, rep.rejected = plan.ok, plan.reasons, plan.rejected
        rep.segments = [round(T, 2) for _, _, T in plan.segments]
        rep.peak_ff = plan.peak_ff
        if not plan.ok:
            return rep, plan
        # re-sync after planning: fresh feedback + start pose unchanged
        rep.post_plan_age_ms = io.read()[1] * 1e3
        state, age = self._wait_fresh(io)
        if age > FEEDBACK_MAX_AGE:
            rep.trip = (f"start refused after planning: feedback age {age * 1e3:.1f} ms "
                        f"> {FEEDBACK_MAX_AGE * 1e3:.0f} ms")
            return rep, plan
        drift = float(np.max(np.abs(io.measured_q16()[1:] - start[1:])))
        if drift > START_TOL:
            rep.trip = f"start refused: pose moved {math.degrees(drift):.2f}° during planning"
            return rep, plan
        mode0 = state.mode_machine
        extra = io.hold_extra()
        tol = track_tol(self.gain_scale)
        link_count = {}
        last_cmd = start
        t_prev = io.now()
        deadline = t_prev
        try:
            traj = list(trajectory(plan.points, plan.segments[0][2], peak_rate=plan.peak_rate))
            if self.drive_lift:
                smooth_lift_column(traj)
            for k, (q, v) in enumerate(traj):
                if self._raise_at is not None and k == self._raise_at:
                    raise RuntimeError("injected exception")
                now = io.now()
                gap = now - t_prev if k else 0.0
                t_prev = now
                state, age = io.read()
                if k == 0:
                    rep.first_cmd_jump = float(np.max(np.abs(q[1:] - start[1:])))
                ref = q if send else start          # shadow: robot is not driven
                rep.log_t.append(now)
                rep.log_cmd.append(np.asarray(q, float).copy())
                rep.log_meas.append(io.measured_q16().copy())
                self._watchdog(state, age, ref, gap, mode0, link_count, rep, tol)
                if self.drive_lift:
                    if k == 0:
                        hs = [p[I_LIFT] for p in plan.points]
                        self._lift_reset(io.measured_q16()[I_LIFT], min(hs) - LIFT_FLOOR_TOL,
                                         max(hs) + LIFT_FLOOR_TOL)
                    _, href = self._lift_step(io, q if send else start, k, send, cap, rep)
                    rep.log_cmd[-1][I_LIFT] = href       # log the lift reference actually tracked
                f = self.b.build(state, q, v, extra, self.gain_scale)
                if not self.b.crc_ok(f.msg):
                    rep.crc_bad += 1
                rep.ff_clip_count += len(f.ff_clipped)
                rep.max_dq = max(rep.max_dq, float(np.max(np.abs(v[1:]))))
                ms = state.motor_state
                for side, sl, ids in (("left_arm", L_SL, GROUPS["left_arm"]),
                                      ("right_arm", R_SL, GROUPS["right_arm"])):
                    err = np.abs(np.array([ms[i].q for i in ids]) - q[sl])
                    dqerr = np.abs(np.array([ms[i].dq for i in ids]) - v[sl])
                    tot = (ARM_KP_ARR * self.gain_scale * err + ARM_KD * self.gain_scale * dqerr
                           + np.abs(np.array(f.ff[side])))
                    rep.max_total_torque = np.maximum(rep.max_total_torque, tot)
                if send:
                    self._send(io, f.msg, cap, rep)
                rep.frames += 1
                last_cmd = q
                deadline += DT
                io.wait_until(deadline)
                rep.max_lateness_ms = max(rep.max_lateness_ms, (io.now() - deadline) * 1e3)
            if self.drive_lift and send:
                # arms hold the last pose while the lift closes the remaining error, then stops
                q_end, zero = last_cmd, np.zeros(16)
                for k2 in range(int(LIFT_SETTLE_S * HZ)):
                    state, age = io.read()
                    now = io.now(); gap = now - t_prev; t_prev = now
                    self._watchdog(state, age, q_end, gap, mode0, link_count, rep, tol)
                    err, _ = self._lift_step(io, q_end, k2, send, cap, rep)
                    self._send(io, self.b.build(state, q_end, zero, extra, self.gain_scale).msg, cap, rep)
                    deadline += DT
                    io.wait_until(deadline)
                    if abs(err) < LIFT_DEADBAND and abs(self._lref.v) < 1e-3:
                        break
                self._lift_zero(io, cap, rep)
        except WatchdogTrip as e:
            rep.trip = str(e)
            self._softstop(io, rep, "feedback age" not in rep.trip, last_cmd, send, cap)
        except Exception as e:
            self._softstop(io, rep, True, last_cmd, send, cap)
            rep.errors.append(f"{type(e).__name__}: {e}")
        return rep, plan

    # -- public entry points
    def run_sim(self, robot, names, waypoints, allow_skip=False):
        """Closed loop against SimRobot only (refuses real IO)."""
        if robot.identity()[0] != "sim":
            raise PermissionError("run_sim 只接受 SimRobot;真机请走 run_shadow → run_real")
        return self._execute(robot, names, waypoints, send=True, mode="sim",
                             allow_skip=allow_skip)[0]

    def run_shadow(self, io, names, waypoints, allow_skip=False):
        """Subscribe-only: watchdog and frame building on real feedback, zero frames sent."""
        if getattr(io, "armed", False):
            raise PermissionError("影子模式必须在未授权(只读)的 IO 上运行")
        rep, plan = self._execute(io, names, waypoints, send=False, mode="shadow",
                                  allow_skip=allow_skip)
        if plan is not None and plan.ok and rep.trip is None and not rep.errors \
                and rep.published == 0 and rep.crc_bad == 0 and rep.ff_clip_count == 0:
            token = secrets.token_hex(8)
            self._tokens[token] = (self.params_digest(names, waypoints, allow_skip),
                                   plan.start, io.identity(), time.monotonic())
            rep.shadow_token = token
        return rep

    def run_real(self, io, names, waypoints, shadow_token=None, confirm_supervised=False,
                 allow_skip=False):
        """Publish to the robot. The token is consumed on use (single-use), even if refused."""
        if not confirm_supervised:
            raise PermissionError("拒绝真机下发:需现场操作者显式传入 confirm_supervised=True。")
        entry = self._tokens.pop(shadow_token, None) if shadow_token else None
        if entry is None:
            raise PermissionError("拒绝真机下发:没有有效的影子令牌(未跑影子模式,或令牌已用过)。")
        digest, start0, ident0, issued = entry
        if time.monotonic() - issued > TOKEN_TTL_S:
            raise PermissionError(f"拒绝真机下发:影子令牌已超过 {TOKEN_TTL_S:.0f}s 有效期,请重跑影子模式。")
        if ident0 != io.identity():
            raise PermissionError(f"拒绝真机下发:影子模式运行在 {ident0},实发 IO 是 {io.identity()},"
                                  "必须同类型、同网卡、同 domain。")
        if digest != self.params_digest(names, waypoints, allow_skip):
            raise PermissionError("拒绝真机下发:计划或执行参数(增益/URDF/摩擦/allow_skip…)与影子模式不一致。")
        io.read()
        drift = float(np.max(np.abs(io.measured_q16()[1:] - start0[1:])))
        if drift > START_TOL:
            raise PermissionError(f"拒绝真机下发:起点比影子模式偏离 {math.degrees(drift):.2f}° "
                                  f"> {math.degrees(START_TOL):.2f}°,请重跑影子模式。")
        cap = secrets.token_bytes(16)
        io._arm(cap)                      # publisher exists only after every check above passed
        try:
            return self._execute(io, names, waypoints, send=True, mode="real",
                                 allow_skip=allow_skip, cap=cap)[0]
        finally:
            io._disarm()
