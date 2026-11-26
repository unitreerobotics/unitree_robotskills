"""TCP motion planning on the R1-D arms with r1ik (waist and lift locked at their measured values).

plan(q14_start, side, T_goal, linear) -> PlanResult with a dense 14-joint path, checked waypoint by waypoint:
  * joint limits with a 0.03 rad margin (the executor re-checks against r1_d LIMITS)
  * self-collision clearance >= SELF_CLEARANCE (URDF collision meshes, r1ik PostureChecker)
  * scene: TCP >= table_z + tcp_min_above_table, arm links >= table_z + link_min_above_table over the table
  * IK accuracy at the goal <= 2 mm / 1 deg; linear paths also <= 5 mm off the line on the way
  * continuity: no joint step > MAX_STEP between consecutive path points (no branch flips)
linear=True: straight line + slerp (approach / retreat / descend).  linear=False: posture-checked goal
solve (global multi-start if tracking misses) + joint-space interpolation (transfers, re-orientations).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
import pinocchio as pin

from ik.kin import EE_FRAME, ReducedModel, pose_error
from ik.quality import PostureChecker
from ik.solvers import IKWeights, RoboKitIK, make_solver

SIDES = ("left", "right")
LIMIT_MARGIN = 0.03
SELF_CLEARANCE = 0.02
POS_TOL, ROT_TOL = 5e-3, 1.0           # m, deg at the goal (2 -> 5 mm 2026-11-20: agents burned turns on 2-4 mm misses)
LINE_TOL = 5e-3                         # m off the straight line on the way
MAX_STEP = 0.08                         # rad between consecutive path points
LIN_STEP_POS, LIN_STEP_ROT = 0.004, math.radians(1.5)
JOINT_STEP = 0.03                       # rad, joint-space interpolation step
MAX_TRAVEL = 0.6                        # m TCP travel per motion


@dataclass
class PlanResult:
    ok: bool
    reason: str | None = None
    path: list = field(default_factory=list)          # 14-vectors (left 7, right 7), path[0] = start
    pos_err_mm: float | None = None
    rot_err_deg: float | None = None
    min_clearance_m: float | None = None
    min_table_gap_m: float | None = None
    final_tcp: dict | None = None


def se3(pos, R) -> pin.SE3:
    return pin.SE3(np.asarray(R, float), np.asarray(pos, float))


def interp(T0: pin.SE3, T1: pin.SE3, s: float) -> pin.SE3:
    p = T0.translation + (T1.translation - T0.translation) * s
    w = pin.log3(T0.rotation.T @ T1.rotation)
    return pin.SE3(T0.rotation @ pin.exp3(w * s), p)


def tcp_dict(T: pin.SE3) -> dict:
    q = pin.Quaternion(T.rotation)
    R = T.rotation
    # rpy as R = Rz(y) Ry(p) Rx(r)
    p = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    r = math.atan2(R[2, 1], R[2, 2])
    y = math.atan2(R[1, 0], R[0, 0])
    return {"pos": [round(float(v), 5) for v in T.translation],
            "quat_wxyz": [round(float(v), 6) for v in (q.w, q.x, q.y, q.z)],
            "rpy_deg": [round(math.degrees(v), 3) for v in (r, p, y)]}


class Planner:
    def __init__(self, lift: float, waist: float, scene: dict | None = None):
        self.lift, self.waist = float(lift), float(waist)
        self.rm = ReducedModel("a7d_cal", "dual_arm", {"lift_stage1_joint": self.lift, "waist_yaw_joint": self.waist})
        self.checker = PostureChecker(self.rm)
        if self.checker.geom is None or not self.checker.pair_names:
            raise RuntimeError("collision model unavailable; refusing to plan")
        self.track = make_solver("robokit_track", self.rm)
        self.fine = make_solver("robokit_track", self.rm, IKWeights(posture=0.002, smooth=0.002, limit_weight=3.0),
                                lm_steps=80)
        self.search = RoboKitIK(self.rm, tracking=False, select_k=8, checker=self.checker)
        self.scene = scene or {}
        self.dz = 0.0      # measured lift - self.lift: the lift is a pure vertical offset of everything above it
        self.lo = self.rm.lower + LIMIT_MARGIN
        self.hi = self.rm.upper - LIMIT_MARGIN
        names = [f.name for f in self.rm.model.frames]
        self.link_frames = {s: [self.rm.model.getFrameId(n) for n in names
                                if n.startswith(s + "_") and n.endswith("_link")
                                and any(k in n for k in ("elbow", "wrist"))] for s in SIDES}

    def matches(self, lift, waist, tol=2e-3):
        """Usable for this lift/waist (any lift: set_lift shifts the frame; the waist must match)."""
        return abs(waist - self.waist) < tol

    def set_lift(self, lift):
        self.dz = float(lift) - self.lift
        return self

    # ------------------------------------------------------------------ kinematics
    def fk(self, q14) -> dict:
        """TCP poses in the base frame at the current lift."""
        out = self.rm.fk(np.asarray(q14, float))
        for T in out.values():
            T.translation = T.translation + np.array([0.0, 0.0, self.dz])
        return out

    def _fk_model(self, q14):
        """TCP poses in the planner's own frame (lift = self.lift)."""
        return self.rm.fk(np.asarray(q14, float))

    def tcp(self, q14) -> dict:
        return {s: tcp_dict(T) for s, T in self.fk(q14).items()}

    def _err(self, q, targets):
        ee = self._fk_model(q)
        e = [pose_error(targets[s], ee[s]) for s in SIDES]
        return max(x[0] for x in e), max(x[1] for x in e)

    # ------------------------------------------------------------------ checks
    def _table_gap(self, q, side):
        sc = self.scene
        if "table_z" not in sc:
            return None, None
        tz = float(sc["table_z"]) - self.dz          # table in the planner's own frame
        x_min = float(sc.get("table_x_min", 0.25))
        pin.forwardKinematics(self.rm.model, self.rm.data, np.asarray(q, float))
        pin.updateFramePlacements(self.rm.model, self.rm.data)
        ee = self.rm.data.oMf[self.rm.model.getFrameId(EE_FRAME[side])].translation
        gap = float(ee[2] - tz)
        if ee[0] > x_min and gap < float(sc.get("tcp_min_above_table", 0.015)):
            return gap, f"TCP {gap * 1e3:.0f} mm above the table < {sc.get('tcp_min_above_table', 0.015) * 1e3:.0f} mm"
        for fid in self.link_frames[side]:
            p = self.rm.data.oMf[fid].translation
            g = float(p[2] - tz)
            gap = min(gap, g)
            if p[0] > x_min and g < float(sc.get("link_min_above_table", 0.05)):
                return gap, f"{self.rm.model.frames[fid].name} {g * 1e3:.0f} mm above the table"
        return gap, None

    def _obstacle_hit(self, q, side):
        """Scene boxes (e.g. the toaster): TCP, fingertips and wrist/elbow links must stay outside each box grown by
        margin_xy horizontally, unless they are at least margin_z above its top. Returns a reason or None."""
        obs = self.scene.get("obstacles") or []
        if not obs:
            return None
        pin.forwardKinematics(self.rm.model, self.rm.data, np.asarray(q, float))
        pin.updateFramePlacements(self.rm.model, self.rm.data)
        T = self.rm.data.oMf[self.rm.model.getFrameId(EE_FRAME[side])]
        pts = [("TCP", T.translation)]
        for dz in (-0.020, 0.020):            # finger tips, upper/lower edge (Dex1: 2.3 cm beyond the pads)
            pts.append(("fingertip", T.translation + T.rotation @ np.array([0.0233, 0.0, dz])))
        pts += [(self.rm.model.frames[f].name, self.rm.data.oMf[f].translation) for f in self.link_frames[side]]
        for o in obs:
            cx, cy = o["center"]
            hx, hy = o["size"][0] / 2 + o.get("margin_xy", 0.03), o["size"][1] / 2 + o.get("margin_xy", 0.03)
            top = o["top_z"] - self.dz + o.get("margin_z", 0.01)
            yaw = o.get("yaw", 0.0)
            c_, s_ = math.cos(yaw), math.sin(yaw)
            for name, p in pts:
                dx, dy = p[0] - cx, p[1] - cy
                lx, ly = c_ * dx + s_ * dy, -s_ * dx + c_ * dy
                if abs(lx) < hx and abs(ly) < hy and p[2] < top:
                    return f"{name} inside the {o.get('name', 'obstacle')} ({(top - p[2]) * 100:.0f} cm below its top + margin)"
        return None

    def check_path(self, path, side):
        mc, tg = 1e9, 1e9
        # escape rule: if the START already violates the table clearance (hands resting near the table after a
        # damping/zero-torque drop), the path is allowed while the clearance keeps growing; normal checks resume
        # once it is clear. Moving down or along the table from such a start is still refused.
        escaping, prev_gap = False, None
        if path:
            g0, why0 = self._table_gap(path[0], side)
            escaping, prev_gap = why0 is not None, g0
        for k, q in enumerate(path):
            if not np.all(np.isfinite(q)):
                return f"waypoint {k}: non-finite", mc, tg
            excess = np.maximum(self.lo - q, q - self.hi)          # > 0: inside the joint-limit margin
            if k == 0:
                limit_escape = np.maximum(excess, 0.0)               # the start itself may sit in the margin
            # joint-limit escape (2026-11-21: an agent left a wrist 1.7 deg from its limit and every path, even one
            # turning it away, was refused at waypoint 0): a joint that starts inside the margin may stay there while
            # it does not move further toward the limit, and never past the hard limit
            bad = [j for j in np.where(excess > 0)[0]
                   if not (limit_escape[j] > 0 and excess[j] <= limit_escape[j] + 1e-6 and excess[j] < LIMIT_MARGIN)]
            if k:
                limit_escape = np.where(excess > 0, np.minimum(limit_escape, np.maximum(excess, 0.0)), 0.0)
            if len(bad):
                j = int(bad[0])
                return (f"waypoint {k}: {self.rm.active_names[j]} {math.degrees(q[j]):.1f}° within "
                        f"{math.degrees(LIMIT_MARGIN):.1f}° of its limit"), mc, tg
            if k and np.max(np.abs(q - path[k - 1])) > MAX_STEP:
                return f"waypoint {k}: joint jump {math.degrees(np.max(np.abs(q - path[k - 1]))):.1f}° (branch flip)", mc, tg
            if k % 2 == 0 or k == len(path) - 1:
                d, pair = self.checker.min_distance(q)
                if not math.isfinite(d):
                    return "self-collision distance unavailable", mc, tg
                mc = min(mc, float(d))
                if d < SELF_CLEARANCE:
                    return f"waypoint {k}: self-collision clearance {d * 1e3:.0f} mm < {SELF_CLEARANCE * 1e3:.0f} mm ({pair})", mc, tg
                g, why = self._table_gap(q, side)
                if g is not None:
                    tg = min(tg, g)
                if why and escaping and g is not None and prev_gap is not None and g >= prev_gap - 0.001:
                    prev_gap = g
                    why = None
                elif not why:
                    escaping = False
                if why:
                    return f"waypoint {k}: {why}", mc, tg
            why = self._obstacle_hit(q, side)       # every waypoint: obstacles are what the hand moves around
            if why:
                return f"waypoint {k}: {why}", mc, tg
        return None, mc, tg

    # ------------------------------------------------------------------ planning
    def _goal_solve(self, q0, targets):
        """Posture-checked goal: warm-started tracking first, global search if it misses or is flagged."""
        cands = []
        q = np.array(q0, float)
        for _ in range(6):
            q = self.track.solve(targets, q, q).q
        cands.append(q.copy())        # keep the tracked solution: the fine stage can drift it into a joint margin
        for _ in range(8):
            q = self.fine.solve(targets, q, q).q
        cands.append(q)
        def acceptable(c):
            pe, re = self._err(c, targets)
            return pe <= POS_TOL and re <= ROT_TOL and not np.any(c < self.lo) and not np.any(c > self.hi)

        if not acceptable(q) or self.checker.evaluate(q)["flags"]:
            # multi-start: the sampled global search is random, one miss is not "unreachable" (11-17: a pose solved
            # to 0 mm but next to a joint limit was rejected every time on the server, accepted offline)
            seeds = [q0, self.rm.nominal_q(), q0, self.rm.nominal_q()]
            for sd in seeds:
                alt = self.search.solve(targets, sd, sd).q
                for _ in range(10):
                    alt = self.fine.solve(targets, alt, alt).q
                cands.append(alt)
                if acceptable(alt) and not self.checker.evaluate(alt)["flags"]:
                    break
        best = None
        self.last_goal_miss = None     # why every candidate was rejected (for an honest refusal reason)
        for c in cands:
            pe, re = self._err(c, targets)
            if pe > POS_TOL or re > ROT_TOL:
                continue
            over = np.maximum(self.lo - c, c - self.hi)
            if np.any(over > 0):
                j = int(np.argmax(over))
                name = self.rm.model.names[int(self.rm.idx_q[j]) + 1] if hasattr(self.rm, "idx_q") else f"joint {j}"
                self.last_goal_miss = (f"the pose is reached ({pe * 1e3:.0f} mm / {re:.0f}°) only with {name} "
                                       f"{np.degrees(over[j]):.0f}° beyond its limit")
                continue
            flags = self.checker.evaluate(c)["flags"]
            score = float(np.sum(np.abs(c - q0))) + (2.0 if flags else 0.0)
            if best is None or score < best[0]:
                best = (score, c)
        return None if best is None else best[1]

    def plan(self, q0, side, T_goal: pin.SE3, linear=True, dz=None) -> PlanResult:
        """T_goal in the base frame at the current lift (or at lift self.lift + dz if dz is given)."""
        q0 = np.asarray(q0, float)
        saved = self.dz
        if dz is not None:
            self.dz = float(dz)
        try:
            return self._plan(q0, side, pin.SE3(T_goal.rotation, T_goal.translation - np.array([0, 0, self.dz])), linear)
        finally:
            self.dz = saved

    def _plan(self, q0, side, T_goal: pin.SE3, linear=True) -> PlanResult:
        start = self._fk_model(q0)
        other = "left" if side == "right" else "right"
        travel = float(np.linalg.norm(T_goal.translation - start[side].translation))
        if travel > MAX_TRAVEL:
            return PlanResult(False, f"TCP travel {travel:.2f} m > {MAX_TRAVEL} m per motion; split it")
        goal_targets = {side: T_goal, other: start[other]}
        path = [q0.copy()]
        if linear:
            ang = float(np.linalg.norm(pin.log3(start[side].rotation.T @ T_goal.rotation)))
            n = max(2, int(math.ceil(travel / LIN_STEP_POS)), int(math.ceil(ang / LIN_STEP_ROT)))
            q = q0.copy()
            for i in range(1, n + 1):
                tg = {side: interp(start[side], T_goal, i / n), other: start[other]}
                for _ in range(2):
                    q = self.track.solve(tg, q, q).q
                pe, _ = self._err(q, tg)
                if pe > LINE_TOL:
                    return PlanResult(False, f"straight line leaves the reachable set at {100 * i / n:.0f}% "
                                             f"({pe * 1e3:.0f} mm off); try linear=False or another pose")
                path.append(q.copy())
            for _ in range(4):
                q = self.fine.solve(goal_targets, q, q).q
            path.append(q.copy())
        else:
            qg = self._goal_solve(q0, goal_targets)
            if qg is None:
                if getattr(self, "last_goal_miss", None):
                    return PlanResult(False, f"no IK solution within the joint limits: {self.last_goal_miss}; "
                                             f"pose out of reach")
                pe, re = self._err(self.track.solve(goal_targets, q0, q0).q, goal_targets)
                return PlanResult(False, f"no IK solution within {POS_TOL * 1e3:.0f} mm / {ROT_TOL:.0f}° "
                                         f"(best {pe * 1e3:.0f} mm / {re:.0f}°): pose out of reach")
            n = max(2, int(math.ceil(np.max(np.abs(qg - q0)) / JOINT_STEP)))
            path += [q0 + (qg - q0) * (i / n) for i in range(1, n + 1)]
        pe, re = self._err(path[-1], goal_targets)
        res = PlanResult(False, None, path, round(pe * 1e3, 2), round(re, 2))
        if pe > POS_TOL or re > ROT_TOL:
            res.reason = f"goal error {pe * 1e3:.1f} mm / {re:.1f}° > {POS_TOL * 1e3:.0f} mm / {ROT_TOL:.0f}°: out of reach"
            return res
        why, mc, tg = self.check_path(path, side)
        res.min_clearance_m = None if mc == 1e9 else round(mc, 4)
        res.min_table_gap_m = None if tg == 1e9 else round(tg, 4)
        res.final_tcp = self.tcp(path[-1])
        if why:
            res.reason = why
            return res
        res.ok = True
        return res

    def plan_joints(self, q0, q1, dz=None) -> PlanResult:
        """Joint-space path q0 -> q1, checked like plan(); dz = lift offset to check at (default: current)."""
        saved = self.dz
        if dz is not None:
            self.dz = float(dz)
        try:
            return self._plan_joints(q0, q1)
        finally:
            self.dz = saved

    def plan_both(self, q0, T_left: pin.SE3, T_right: pin.SE3, linear=True) -> PlanResult:
        """Both TCPs to their goals at the same time, checked like plan(). linear=True: straight Cartesian lines
        (e.g. pushing a plate with two hands); linear=False: one joint-space path for both arms.
        Goals are in the base frame at the current lift."""
        q0 = np.asarray(q0, float)
        off = np.array([0.0, 0.0, self.dz])
        goals = {"left": pin.SE3(T_left.rotation, T_left.translation - off),
                 "right": pin.SE3(T_right.rotation, T_right.translation - off)}
        start = self._fk_model(q0)
        n = 2
        for s in SIDES:
            travel = float(np.linalg.norm(goals[s].translation - start[s].translation))
            if travel > MAX_TRAVEL:
                return PlanResult(False, f"{s} TCP travel {travel:.2f} m > {MAX_TRAVEL} m per motion; split it")
        if not linear:
            qg = self._goal_solve(q0, goals)
            if qg is None:
                pe, re = self._err(self.track.solve(goals, q0, q0).q, goals)
                return PlanResult(False, f"no IK solution within {POS_TOL * 1e3:.0f} mm / {ROT_TOL:.0f}° "
                                         f"(best {pe * 1e3:.0f} mm / {re:.0f}°): pose out of reach")
            res = self._plan_joints(q0, qg)
            pe, re = self._err(qg, goals)
            res.pos_err_mm, res.rot_err_deg = round(pe * 1e3, 2), round(re, 2)
            return res
        for s in SIDES:
            ang = float(np.linalg.norm(pin.log3(start[s].rotation.T @ goals[s].rotation)))
            n = max(n, int(math.ceil(travel / LIN_STEP_POS)), int(math.ceil(ang / LIN_STEP_ROT)))
        path, q = [q0.copy()], q0.copy()
        for i in range(1, n + 1):
            tg = {s: interp(start[s], goals[s], i / n) for s in SIDES}
            for _ in range(2):
                q = self.track.solve(tg, q, q).q
            pe, _ = self._err(q, tg)
            if pe > LINE_TOL:
                return PlanResult(False, f"straight line leaves the reachable set at {100 * i / n:.0f}% "
                                         f"({pe * 1e3:.0f} mm off); try another pose")
            path.append(q.copy())
        for _ in range(4):
            q = self.fine.solve(goals, q, q).q
        path.append(q.copy())
        pe, re = self._err(path[-1], goals)
        res = PlanResult(False, None, path, round(pe * 1e3, 2), round(re, 2))
        if pe > POS_TOL or re > ROT_TOL:
            res.reason = f"goal error {pe * 1e3:.1f} mm / {re:.1f}° > {POS_TOL * 1e3:.0f} mm / {ROT_TOL:.0f}°: out of reach"
            return res
        why, mc, tg = self.check_path(path, "left")
        if why is None:
            why, mc2, tg2 = self.check_path(path, "right")
            mc, tg = min(mc, mc2), min(tg, tg2)
        res.min_clearance_m = None if mc == 1e9 else round(mc, 4)
        res.min_table_gap_m = None if tg == 1e9 else round(tg, 4)
        res.final_tcp = self.tcp(path[-1])
        res.reason, res.ok = why, why is None
        return res

    def _plan_joints(self, q0, q1) -> PlanResult:
        q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
        n = max(2, int(math.ceil(np.max(np.abs(q1 - q0)) / JOINT_STEP)))
        path = [q0 + (q1 - q0) * (i / n) for i in range(n + 1)]
        res = PlanResult(False, None, path)
        why, mc, tg = self.check_path(path, "left")
        if why is None:
            why, mc2, tg2 = self.check_path(path, "right")
            mc, tg = min(mc, mc2), min(tg, tg2)
        res.min_clearance_m = None if mc == 1e9 else round(mc, 4)
        res.min_table_gap_m = None if tg == 1e9 else round(tg, 4)
        res.final_tcp = self.tcp(q1)
        res.reason, res.ok = why, why is None
        return res
