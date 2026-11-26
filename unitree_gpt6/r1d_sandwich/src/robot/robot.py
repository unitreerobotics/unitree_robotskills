"""R1-D robot backend for the skill server: state, TCP motions, grippers, home, stop.

Modes
  sim     SimGripperRobot (r1-ik deploy's kinematic SimRobot + grippers) and a toy world (simworld.py).
  shadow  Real feedback, nothing published. Each motion: (1) real feedback health (age, temperature,
          mode_machine, fault bits); (2) while the twin still sits at the real pose (first motion, or after
          /resync), the executor's real shadow run on DdsRobotIO; (3) the executor's closed loop on a twin
          SimGripperRobot seeded from the real pose, which then advances, so a whole task can be dry-run.
  real    Each motion: executor shadow run -> operator confirmation (server terminal) -> run_real with the
          single-use token. A watchdog trip latches `stopped` until /stop {clear:true}.
"""
from __future__ import annotations

import json
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

from ik.ik_path import Planner, se3
from robot import grip_exec as G
from robot.grip_exec import X

ARM_NAMES = X.IK_ACTIVE_NAMES[2:]                  # 14, left then right
EXEC_NAMES = list(X.ARM_WAIST_NAMES)               # waist + 14 arms (lift filled from measurement)
SIDE_SLICE = {"left": slice(0, 7), "right": slice(7, 14)}
# One lock for every use of the planner's GPU IK (robokit / Warp). The HTTP server is threaded: a /state that
# rebuilt the planner while another request ran IK crashed the whole process on 2026-11-20 ("Cannot synchronize
# device cuda:0 while graph capture is active", CUDA abort). Re-entrant: motion paths nest planner calls.
GPU_LOCK = threading.RLock()


class _LockedPlanner:
    """The Planner with every method call (and attribute read) under GPU_LOCK."""

    def __init__(self, planner):
        object.__setattr__(self, "_p", planner)

    def __getattr__(self, name):
        attr = getattr(self._p, name)
        if not callable(attr):
            return attr

        def call(*a, **kw):
            with GPU_LOCK:
                out = attr(*a, **kw)
            return self if out is self._p else out
        return call

    def __setattr__(self, name, value):
        with GPU_LOCK:
            setattr(self._p, name, value)


def confirm_real_start(read=input) -> bool:
    """The operator's go for real mode, asked once before connecting: ``s`` (any case) starts, anything else
    or a closed stdin does not."""
    try:
        answer = read("\n[r1d-skills] real robot: press s to start (emergency stop: controller L2+B): ")
    except EOFError:
        return False
    return answer.strip().lower() == "s"


def make_planner(lift, waist, scene):
    with GPU_LOCK:
        return _LockedPlanner(Planner(lift, waist, scene))


HOLD_MIN_OPENING = 0.04
# Transit rule (native agent runs, scene["transit_z"]): a move that carries the TCP more than TRANSIT_LATERAL_M
# sideways must keep it at or above transit_z except within TRANSIT_END_M (horizontal) of its start and its goal,
# i.e. rise, cross, then descend. Added 2026-11-20 after an agent swept the arm low across the table.
TRANSIT_LATERAL_M, TRANSIT_END_M = 0.10, 0.05       # measured opening above this after a full close = something is held
GRIP_SETTLE_S = 0.35


def Rz(d):
    return pin.utils.rotate("z", math.radians(d))


def Ry(d):
    return pin.utils.rotate("y", math.radians(d))


def Rx(d):
    return pin.utils.rotate("x", math.radians(d))


FAIL_CODES = (  # first match wins; lower-case substring of the reason -> code
    ("operator declined", "operator_declined"), ("server stopped", "stopped"), ("operator stop", "stopped"),
    ("soft-stopped", "watchdog"), ("watchdog", "watchdog"), ("robot not ready", "not_ready"),
    ("refused before sending", "not_ready"), ("shadow", "executor_refused"), ("above the table", "table"),
    ("transit rule", "transit"), ("self-collision", "collision"), ("of its limit", "joint_limit"), ("lift ", "lift_range"),
    ("out of reach", "unreachable"), ("reachable set", "unreachable"), ("no ik", "unreachable"),
    ("travel", "too_far"), ("branch flip", "unreachable"), ("must", "invalid_argument"),
    ("unknown pose", "invalid_argument"), ("side must", "invalid_argument"),
)


def fail_code(reason):
    r = str(reason or "").lower()
    for k, c in FAIL_CODES:
        if k in r:
            return c
    return "refused"


def rot_dist(A, B):
    return float(np.linalg.norm(pin.log3(A.T @ B)))


def orientation_candidates(spec: dict, R_now):
    """Rotation candidates for a target, closest to the current orientation first.
    Families exploit the parallel gripper's 180° symmetry about the fingers and allow the fingers
    to tilt inside the closing plane (the closing axis is unchanged by that tilt)."""
    yaw = float(spec.get("yaw_deg") or 0.0)
    if spec.get("quat_wxyz") is not None:
        w, x, y, z = spec["quat_wxyz"]
        cands = [pin.Quaternion(w, x, y, z).normalized().toRotationMatrix()]
    elif spec.get("rpy_deg") is not None:
        r, p, y = spec["rpy_deg"]
        cands = [Rz(y) @ Ry(p) @ Rx(r)]
    else:
        o = spec.get("orient")
        if o is None:
            return [R_now] if spec.get("yaw_deg") is None else [Rz(yaw) @ R_now]
        if o == "down":
            cands = [Rz(yaw + f) @ Ry(90 - t) for t in (0, 15, 30, 45, 55, 65) for f in (0, 180)]
        elif o == "forward":
            cands = [Rz(yaw) @ Ry(t) @ Rx(f) for t in (0, 15, 30) for f in (0, 180)]
        elif o == "flat":
            cands = [Rz(yaw + dh) @ Ry(t) @ Rx(r) for r in (90, -90) for t in (0, 10, 20) for dh in (0, -15, 15)]
        else:
            raise ValueError(f"orient must be forward/down/flat, got {o!r}")
    return sorted(cands, key=lambda R: rot_dist(R_now, R))


class Robot:
    confirm_wait_s = 0.0   # total time spent waiting for the operator at _ask

    def __init__(self, mode: str, scene: dict, log_dir: Path, confirm: str = "all", temp_guard: bool = True,
                 interface: str = "enp4s0", world=None, poses_path: Path | None = None,
                 grip_kp: float = G.GRIP_KP, grip_kd: float = G.GRIP_KD, ignore_gripper_fault=()):
        assert mode in ("sim", "shadow", "real")
        self.mode, self.scene, self.confirm, self.temp_guard = mode, scene, confirm, temp_guard
        self.world = world
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.poses_path = poses_path
        self.lock = threading.Lock()          # one motion at a time
        self.stop_event = threading.Event()
        self.stopped: str | None = None       # latched reason after a trip / stop
        self.builder = G.GripperFrameBuilder(grip_kp=grip_kp, grip_kd=grip_kd)
        self.fixed_lift = None            # set by the server (--fixed-lift): the lift may only go to this height
        self.ignore_fault = tuple(G.SIDE_MOTOR[s] for s in ignore_gripper_fault)
        self.n_calls = 0
        self.real_io = None
        if mode == "sim":
            lift = float(scene.get("lift_m", 0.45))
            from robot.brain import ready16
            start = ready16(lift)
            self.twin = G.SimGripperRobot(start, gripper=(G.GRIP_MAX, G.GRIP_MAX),
                                          blocker=world.blocker if world else None)
        else:                           # the robot, through ik_executor over unitree_sdk2py
            if mode == "real" and not confirm_real_start():
                raise PermissionError("R1-D real control not confirmed")
            from robot.dds_io import DdsRobotIO
            self.real_io = DdsRobotIO(interface=interface)
            self.twin = None
            if mode == "shadow":
                self._resync_twin()
        self.planner: Planner | None = None

    # ------------------------------------------------------------------ io helpers
    @property
    def io(self):
        """Where the pose the next motion starts from lives: the sim / shadow twin, or the real robot."""
        return self.real_io if self.mode == "real" else self.twin

    def _resync_twin(self):
        q16 = self.real_io.measured_q16()
        extra = self.real_io.hold_extra()
        self.twin = G.SimGripperRobot(q16, head=(extra[X.HEAD_MOTORS[0]], extra[X.HEAD_MOTORS[1]]),
                                      gripper=(extra[X.GRIPPER_MOTORS[0]], extra[X.GRIPPER_MOTORS[1]]))
        self.twin_at_real = True

    def _planner(self, q16):
        lift, waist = float(q16[X.I_LIFT]), float(q16[X.I_WAIST])
        with GPU_LOCK:
            # 5 mrad (~2.5 mm at 0.5 m): the arms' load nudges the measured waist ~2 mrad after a motion, which
            # used to rebuild the planner (2.5 s of GPU warm-up) on nearly every /state
            if self.planner is None or not self.planner.matches(lift, waist, tol=5e-3):
                self.planner = make_planner(lift, waist, self.scene)
            return self.planner.set_lift(lift)

    def _feedback_health(self):
        """Real-robot feedback problems that must block any motion (shadow and real)."""
        io = self.real_io
        if io is None:
            return None
        # planning can starve the DDS callback thread (GIL): give it up to 0.3 s to deliver a fresh sample
        t_end = time.perf_counter() + 0.3
        state, age = io.read()
        while age > X.FEEDBACK_MAX_AGE / 2 and time.perf_counter() < t_end:
            time.sleep(0.005)
            state, age = io.read()
        if age > X.FEEDBACK_MAX_AGE * 2:
            return f"feedback age {age * 1e3:.0f} ms"
        bad = []
        for i in X.ARM_MOTORS + (X.WAIST_MOTOR,) + X.GRIPPER_MOTORS:
            m = state.motor_state[i]
            if i in self.ignore_fault:
                continue
            if m.motorstate & ~X.MOTOR_LINK_LOST:
                bad.append(f"motor {i} fault 0x{m.motorstate:x}")
            if self.temp_guard and m.temperature[0] >= X.TEMP_START_MAX:
                bad.append(f"motor {i} {m.temperature[0]}°C >= {X.TEMP_START_MAX}°C")
        return "; ".join(bad) if bad else None

    # ------------------------------------------------------------------ state
    def opening(self, q):
        return float(np.clip(q / G.GRIP_MAX, 0.0, 1.0))

    def state(self) -> dict:
        io = self.io
        q16 = io.measured_q16()
        extra = io.hold_extra()
        p = self._planner(q16)
        tcp = p.tcp(q16[2:])
        out = {"mode": self.mode, "lift_m": round(float(q16[X.I_LIFT]), 4), "waist_rad": round(float(q16[X.I_WAIST]), 4),
               "arm_q": {s: [round(float(v), 4) for v in q16[2:][SIDE_SLICE[s]]] for s in ("left", "right")},
               "head_q": [round(extra[X.HEAD_MOTORS[0]], 4), round(extra[X.HEAD_MOTORS[1]], 4)],
               "head_pitch": round(extra[X.HEAD_MOTORS[0]], 4), "head_yaw": round(extra[X.HEAD_MOTORS[1]], 4),
               "tcp": tcp, "gripper": {}, "stopped": self.stopped}
        for side, m in G.SIDE_MOTOR.items():
            meas = self.opening(extra[m])
            cmd = self.builder.target.get(m)
            cmd_o = None if cmd is None else self.opening(cmd)
            grasping = m in self.builder.grasping
            holding = grasping and meas > HOLD_MIN_OPENING
            out["gripper"][side] = {"opening": round(meas, 3), "cmd_opening": None if cmd_o is None else round(cmd_o, 3),
                                    "holding": bool(holding), "grasping": grasping}
        if self.real_io is not None:
            st, age = self.real_io.read()
            temps = [(i, int(st.motor_state[i].temperature[0])) for i in X.ARM_MOTORS + (X.WAIST_MOTOR,) + X.GRIPPER_MOTORS]
            out.update(feedback_age_ms=round(age * 1e3, 1), mode_machine=int(st.mode_machine),
                       temps_max_c=max(t for _, t in temps), hot_motors=[i for i, t in temps if t >= X.TEMP_START_MAX])
            if self.mode == "shadow":
                rq = self.real_io.measured_q16()
                out["real_tcp"] = p.tcp(rq[2:])
                out["twin_at_real"] = bool(self.twin_at_real)
        if self.world is not None:
            self.world.sync(tcp, out["gripper"])
        return out

    def _sync_world(self):
        q16 = self.twin.measured_q16()
        extra = self.twin.hold_extra()
        tcp = self._planner(q16).tcp(q16[2:])
        grip = {}
        for side, m in G.SIDE_MOTOR.items():
            cmd = self.builder.target.get(m)
            grip[side] = {"opening": self.opening(extra[m]), "cmd_opening": None if cmd is None else self.opening(cmd),
                          "grasping": m in self.builder.grasping}
        self.world.sync(tcp, grip)

    def robot_q(self) -> dict:
        """Joint values the camera poses depend on (for /project)."""
        q16 = (self.real_io or self.io).measured_q16()
        extra = (self.real_io or self.io).hold_extra()
        return {"head_pitch": extra[X.HEAD_MOTORS[0]], "head_yaw": extra[X.HEAD_MOTORS[1]], "lift": float(q16[X.I_LIFT]),
                "waist": float(q16[X.I_WAIST]), "left_arm": list(map(float, q16[2:9])), "right_arm": list(map(float, q16[9:16]))}

    # ------------------------------------------------------------------ execution core
    def _log(self, rec):
        rec["t"] = time.time()
        with open(self.log_dir / "calls.jsonl", "a") as f:
            f.write(json.dumps(rec, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")

    @staticmethod
    def _rep_reason(rep):
        return str(rep.trip or (rep.plan_reasons and rep.plan_reasons[0]) or (rep.errors and rep.errors[0])
                   or (rep.rejected and f"waypoint rejected {rep.rejected[0]}") or "executor refused")

    def _ask(self, summary):
        if self.confirm == "none":
            return True
        print(f"\n[r1d-skills] about to execute on the real robot: {summary}\n"
              "  Enter = execute,  n + Enter = refuse  (emergency stop: controller L2+B) > ",
              end="", flush=True)
        try:   # drop keys typed before the prompt: a stray Enter must not approve a motion nobody has seen
            import termios
            termios.tcflush(sys.stdin, termios.TCIFLUSH)
        except Exception:  # noqa: BLE001 - not a terminal (tests, pipes): nothing buffered to drop
            pass
        t0 = time.time()
        ans = sys.stdin.readline().strip().lower()
        wait = time.time() - t0
        self.confirm_wait_s += wait          # operator time, reported apart from robot and model time
        self._log({"call": "confirm", "label": summary, "wait_s": round(wait, 2), "answer": ans or "enter"})
        return ans in ("", "y", "yes")

    def _run(self, label, path14, peak_rate, grip_goal: dict | None = None, path16=None, min_seconds=None,
             path15=None) -> dict:
        saved = X.MIN_SEGMENT_S
        if min_seconds is not None:      # stationary arms (gripper / head runs): run long enough for the ramp
            X.MIN_SEGMENT_S = max(saved, float(min_seconds))
        try:
            out = self._run_inner(label, path14, peak_rate, grip_goal, path16, path15)
        finally:
            X.MIN_SEGMENT_S = saved
        out["stopped"] = bool(self.stopped)
        out["code"] = "ok" if out["ok"] else fail_code(out.get("reason"))
        return out

    def _run_inner(self, label, path14, peak_rate, grip_goal=None, path16=None, path15=None) -> dict:
        """Execute a dense 14-joint path (path14[0] must be the current pose) and/or gripper goals.
        path16: full 16-joint waypoints instead (lift driven on rt/cmd_vel)."""
        if self.stopped:
            return {"ok": False, "executed": False, "reason": f"server stopped: {self.stopped}; POST /stop {{clear:true}}"}
        why = self._feedback_health()
        if why:
            return {"ok": False, "executed": False, "reason": f"robot not ready: {why}"}
        io = self.io
        q16 = io.measured_q16()
        waist = float(q16[X.I_WAIST])
        names = EXEC_NAMES
        if path15 is not None:                       # waist + 14 arms (waist moves, lift locked)
            wps = [np.asarray(w, float) for w in path15]
        elif path16 is not None:
            names, wps = list(X.IK_ACTIVE_NAMES), [np.asarray(w, float) for w in path16]
        else:
            idx = np.unique(np.r_[np.arange(0, len(path14), 3), len(path14) - 1])
            wps = [np.r_[waist, path14[i]] for i in idx]
            if len(wps) < 2:
                wps = [wps[0], wps[0]]
        ex = G.StoppableExecutor(builder=self.builder, peak_rate=peak_rate, temp_guard=self.temp_guard,
                                 stop_event=self.stop_event, drive_lift=path16 is not None,
                                 ignore_fault_motors=getattr(self, "ignore_fault", ()))
        extra = io.hold_extra()

        def arm():
            self.builder.begin(extra)
            for m, q in (grip_goal or {}).items():
                self.builder.set_goal(m, q)

        t0 = time.time()
        out = {"ok": False, "executed": False, "reason": None, "check": None}
        if self.mode in ("sim", "shadow"):
            if self.mode == "shadow" and self.twin_at_real:
                arm()
                sh = ex.run_shadow(G.FastShadowIO(self.real_io), names, wps)
                if not sh.shadow_token:
                    out["reason"] = "shadow (real feedback): " + self._rep_reason(sh)
                    return out
                out["check"] = "shadow on real feedback + twin"
            arm()
            rep = ex.run_sim(self.twin, names, wps)
            out["check"] = out["check"] or ("sim" if self.mode == "sim" else "twin (planned from the previous shadow pose)")
            if rep.trip or rep.errors or not rep.plan_ok:
                out["reason"] = self._rep_reason(rep)
                return out
            self.builder.commit()
            if self.world is not None:
                self._sync_world()
            if self.mode == "shadow":
                self.twin_at_real = False
            out.update(ok=True, executed=self.mode == "sim", duration_s=round(rep.frames * X.DT, 2))
            if self.mode == "shadow":
                out["note"] = "shadow mode: checked only, the robot did not move"
            return out
        # real
        arm()
        sh = ex.run_shadow(G.FastShadowIO(self.real_io), names, wps)
        if not sh.shadow_token:
            out["reason"] = "shadow: " + self._rep_reason(sh)
            return out
        if not self._ask(f"{label} ({sh.frames * X.DT:.1f} s)"):
            out["reason"] = "operator declined"
            return out
        self.stop_event.clear()
        arm()
        import gc
        gc.collect()
        gc.disable()                      # a full collection can stall the 250 Hz loop past CYCLE_MAX_GAP
        try:
            rep = ex.run_real(self.real_io, names, wps, shadow_token=sh.shadow_token, confirm_supervised=True)
            if rep.published == 0 and str(rep.trip or "").startswith("start refused"):
                # nothing was sent (e.g. one late feedback sample): not a trip -- re-shadow once and retry
                time.sleep(0.05)
                arm()
                sh = ex.run_shadow(G.FastShadowIO(self.real_io), names, wps)
                if sh.shadow_token:
                    arm()
                    rep = ex.run_real(self.real_io, names, wps, shadow_token=sh.shadow_token, confirm_supervised=True)
                if rep.published == 0:
                    out["reason"] = f"real run refused before sending: {rep.trip or self._rep_reason(sh)}"
                    return out
        except PermissionError as e:      # token expired (> 120 s to confirm), start pose drifted, ...
            out["reason"] = f"real run refused before sending: {e}"
            return out
        finally:
            gc.enable()
        try:   # per-frame log of every real run (commanded vs measured) for diagnosis
            fn = self.log_dir / f"real-{time.strftime('%H%M%S')}-{label.split()[0]}.npz"
            np.savez_compressed(fn, t=np.array(rep.log_t), cmd=np.array(rep.log_cmd), meas=np.array(rep.log_meas),
                                label=np.array(label), trip=np.array(str(rep.trip)))
            out["frame_log"] = str(fn)
        except Exception:
            pass
        out.update(executed=rep.published > 0, duration_s=round(time.time() - t0, 2), frames=rep.published,
                   max_track_err_deg=[round(math.degrees(v), 2) for v in rep.max_track_err])
        if rep.trip or rep.errors:
            self.stopped = self._rep_reason(rep)
            out["reason"] = "real run soft-stopped: " + self.stopped
            return out
        self.builder.commit()
        out["ok"] = True
        return out

    def close(self):
        pass                            # the DDS participant ends with the process

    def _transit_violation(self, p, path, side):
        z_min = self.scene.get("transit_z")
        if z_min is None or len(path) < 2:
            return None
        step = max(1, len(path) // 40)
        pts = np.array([p.tcp(q)[side]["pos"] for q in list(path[::step]) + [path[-1]]], float)
        a, b = pts[0, :2], pts[-1, :2]
        if np.linalg.norm(b - a) < TRANSIT_LATERAL_M:
            return None
        for pt in pts:
            if min(np.linalg.norm(pt[:2] - a), np.linalg.norm(pt[:2] - b)) > TRANSIT_END_M and pt[2] < z_min - 1e-3:
                return (f"transit rule: a move of {np.linalg.norm(b - a) * 100:.0f} cm sideways passes the TCP at "
                        f"z {pt[2]:.3f} < {z_min:.3f} m; rise to z >= {z_min:.3f} first, cross, then descend")
        return None

    # ------------------------------------------------------------------ public API
    def _speed(self, speed):
        s = float(np.clip(0.5 if speed is None else speed, 0.0, 1.0))
        return 0.12 + 0.68 * s          # speed 1.0 -> 0.8 rad/s = the executor's hard cap (PEAK_RATE_MAX)

    def resolve_pose(self, side, spec, q16):
        p = self._planner(q16)
        T_now = p.fk(q16[2:])[side]
        if spec.get("pose"):
            poses = self.load_poses()
            if spec["pose"] not in poses:
                raise ValueError(f"unknown pose {spec['pose']!r}; known: {sorted(poses)}")
            ps = poses[spec["pose"]]
            spec = {"pos": ps["pos"], "quat_wxyz": ps.get("quat_wxyz"), **{k: v for k, v in spec.items() if k != "pose"}}
        pos = T_now.translation.copy() if spec.get("pos") is None else np.asarray(spec["pos"], float)
        if pos.shape != (3,) or not np.all(np.isfinite(pos)):
            raise ValueError("pos must be 3 finite numbers [x, y, z] in metres")
        return pos, orientation_candidates(spec, T_now.rotation)

    def _find_lift(self, q16, side, pos, cands, linear):
        """Lift height (nearest to the current one) at which some candidate orientation is plannable."""
        p = self._planner(q16)
        h0 = float(q16[X.I_LIFT])
        lo, hi = max(X.LIFT_RANGE[0], X.lift_floor(h0, 0.0)), X.LIFT_RANGE[1] - 0.01
        hs = sorted(np.arange(lo, hi + 1e-9, 0.025), key=lambda h: abs(h - h0))
        for h in hs:
            if abs(h - h0) < 0.01:
                continue
            for R in cands[:8]:
                if p.plan(q16[2:], side, se3(pos, R), linear=linear, dz=h - p.lift).ok:
                    return float(h)
        return None

    def move(self, side, spec: dict, linear=True, speed=None, label=None, auto_lift=True, check_only=False) -> dict:
        if self.fixed_lift is not None:
            auto_lift = False
        if check_only:
            res = self._move(side, spec, linear, speed, label, check_only=True)
            if res["ok"] or not auto_lift or "plan refused" not in str(res.get("reason")):
                return res
            q16 = self.io.measured_q16()
            try:
                pos, cands = self.resolve_pose(side, spec, q16)
            except ValueError:
                return res
            h = self._find_lift(q16, side, pos, cands, linear)
            if h is None:
                res["reason"] += " (no lift height in range makes it reachable either)"
                return res
            return {"ok": True, "executed": False, "reason": None, "check_only": True, "code": "ok",
                    "stopped": bool(self.stopped), "needs_lift_m": round(h, 3)}
        res = self._move(side, spec, linear, speed, label)
        if res["ok"] or not auto_lift or "plan refused" not in str(res.get("reason")) or self.stopped:
            return res
        with self.lock:
            q16 = self.io.measured_q16()
            try:
                pos, cands = self.resolve_pose(side, spec, q16)
            except ValueError:
                return res
            h = self._find_lift(q16, side, pos, cands, linear)
        if h is None:
            res["reason"] += " (no lift height in range makes it reachable either)"
            return res
        lr = self.lift(h)
        if not lr["ok"]:
            res["reason"] += f" (auto lift to {h:.3f} m refused: {lr['reason']})"
            return res
        res2 = self._move(side, spec, linear, speed, label)
        res2["auto_lift_m"] = round(h, 3)
        res2["lift_result"] = {k: lr.get(k) for k in ("ok", "executed", "lift_m")}
        return res2

    def _move(self, side, spec: dict, linear=True, speed=None, label=None, check_only=False) -> dict:
        if side not in ("left", "right"):
            return {"ok": False, "executed": False, "reason": "side must be left or right"}
        with self.lock:
            q16 = self.io.measured_q16()
            try:
                pos, cands = self.resolve_pose(side, spec, q16)
            except ValueError as e:
                return {"ok": False, "executed": False, "reason": str(e)}
            p = self._planner(q16)
            tried, plan = [], None
            for R in cands[:12]:
                r = p.plan(q16[2:], side, se3(pos, R), linear=linear)
                if r.ok:
                    why = self._transit_violation(p, r.path, side)
                    if why:
                        tried.append(why)
                        continue
                    plan = r
                    break
                tried.append(r.reason)
                if linear and len(cands) > 1 and "leaves the reachable set" in (r.reason or ""):
                    continue
            if plan is None:
                why = tried[0] if tried else "no candidate"
                res = {"ok": False, "executed": False, "reason": f"plan refused: {why}",
                       "tried_orientations": len(tried), "code": fail_code(why), "stopped": bool(self.stopped)}
                self._log({"call": "move", "side": side, "spec": spec, "result": res})
                return res
            if check_only:
                return {"ok": True, "executed": False, "reason": None, "check_only": True, "stopped": bool(self.stopped),
                        "pos_err_mm": plan.pos_err_mm, "rot_err_deg": plan.rot_err_deg,
                        "min_clearance_m": plan.min_clearance_m, "min_table_gap_m": plan.min_table_gap_m,
                        "planned_tcp": plan.final_tcp[side]}
            res = self._run(label or f"{side} TCP -> {np.round(pos, 3).tolist()}", plan.path, self._speed(speed))
            res.update(pos_err_mm=plan.pos_err_mm, rot_err_deg=plan.rot_err_deg, min_clearance_m=plan.min_clearance_m,
                       min_table_gap_m=plan.min_table_gap_m, planned_tcp=plan.final_tcp[side],
                       final_tcp=self._planner(self.io.measured_q16()).tcp(self.io.measured_q16()[2:])[side])
            self._log({"call": "move", "side": side, "spec": spec, "linear": linear, "result": res})
            return res

    def move_both(self, left: dict, right: dict, speed=None, check_only=False, linear=True) -> dict:
        """Both TCPs at the same time: straight lines (linear=True) or one joint-space path (linear=False).
        `left` / `right` are pose specs as for move().
        No orientation search and no lift change: each spec must give the exact goal."""
        with self.lock:
            q16 = self.io.measured_q16()
            try:
                goals = {}
                for side, spec in (("left", left), ("right", right)):
                    pos, cands = self.resolve_pose(side, spec or {}, q16)
                    goals[side] = se3(pos, cands[0])
            except (ValueError, TypeError) as e:
                return {"ok": False, "executed": False, "reason": str(e), "code": "invalid_argument",
                        "stopped": bool(self.stopped)}
            p = self._planner(q16)
            plan = p.plan_both(q16[2:], goals["left"], goals["right"], linear=bool(linear))
            if not plan.ok:
                res = {"ok": False, "executed": False, "reason": f"plan refused: {plan.reason}",
                       "code": fail_code(plan.reason), "stopped": bool(self.stopped)}
                self._log({"call": "move_both", "left": left, "right": right, "result": res})
                return res
            if check_only:
                return {"ok": True, "executed": False, "reason": None, "check_only": True, "code": "ok",
                        "stopped": bool(self.stopped), "pos_err_mm": plan.pos_err_mm, "rot_err_deg": plan.rot_err_deg,
                        "min_clearance_m": plan.min_clearance_m, "min_table_gap_m": plan.min_table_gap_m,
                        "planned_tcp": plan.final_tcp}
            res = self._run("both TCPs, " + ("straight lines" if linear else "joint space"), plan.path, self._speed(speed))
            tcp = self._planner(self.io.measured_q16()).tcp(self.io.measured_q16()[2:])
            res.update(pos_err_mm=plan.pos_err_mm, rot_err_deg=plan.rot_err_deg, min_clearance_m=plan.min_clearance_m,
                       min_table_gap_m=plan.min_table_gap_m, planned_tcp=plan.final_tcp, final_tcp_both=tcp)
            self._log({"call": "move_both", "left": left, "right": right, "result": res})
            return res

    def gripper(self, side, opening=None, action=None) -> dict:
        if side not in G.SIDE_MOTOR:
            return {"ok": False, "executed": False, "reason": "side must be left or right"}
        if action is not None:
            opening = {"open": 1.0, "close": 0.0, "grasp": 0.0}.get(action)
            if opening is None:
                return {"ok": False, "executed": False, "reason": "action must be open/close/grasp"}
        if opening is None or not 0.0 <= float(opening) <= 1.0:
            return {"ok": False, "executed": False, "reason": "opening must be in [0, 1]"}
        m = G.SIDE_MOTOR[side]
        if m in getattr(self, "ignore_fault", ()):
            return {"ok": False, "executed": False, "code": "not_ready", "stopped": bool(self.stopped),
                    "reason": f"{side} gripper fault ignored for recovery: gripper commands are disabled"}
        with self.lock:
            if self.world is not None:
                self._sync_world()
            q16 = self.io.measured_q16()
            if action == "grasp":
                self.builder.grasping.add(m)
            elif float(opening) > 0.05:
                self.builder.grasping.discard(m)
            res = self._run(f"{side} gripper -> {float(opening):.2f}", [q16[2:], q16[2:]], 0.3,
                            grip_goal={m: float(opening) * G.GRIP_MAX}, min_seconds=G.GRIP_MAX / G.GRIP_RATE + 0.4)
            if res["ok"] and self.mode == "real":
                time.sleep(GRIP_SETTLE_S)
            meas = self.opening(self.io.hold_extra()[m])
            res["opening"] = round(meas, 3)
            if action == "grasp" and res["ok"]:
                res["holding"] = bool(meas > HOLD_MIN_OPENING) if self.mode != "shadow" else None
            if self.world is not None and res["ok"]:
                self._sync_world()
            self._log({"call": "gripper", "side": side, "opening": opening, "action": action, "result": res})
            return res

    def _home_target(self, q16, side):
        from robot.brain import ready16
        tgt = ready16(q16[X.I_LIFT])[2:]
        q1 = q16[2:].copy()
        for s in (("left", "right") if side == "both" else (side,)):
            q1[SIDE_SLICE[s]] = tgt[SIDE_SLICE[s]]
        return q1

    def move_joints(self, side, q7, speed=None) -> dict:
        """Joint-space move of one arm to recorded joint angles (replay): no IK search, the path is still checked
        (limits, self-collision, table, obstacles) and executed through the usual shadow/real gate."""
        if side not in SIDE_SLICE or len(q7) != 7:
            return {"ok": False, "executed": False, "reason": "side + 7 joint values", "code": "invalid_argument",
                    "stopped": bool(self.stopped)}
        with self.lock:
            q16 = self.io.measured_q16()
            q1 = q16[2:].copy()
            q1[SIDE_SLICE[side]] = np.asarray(q7, float)
            p = self._planner(q16)
            plan = p.plan_joints(q16[2:], q1)
            if not plan.ok:
                return {"ok": False, "executed": False, "code": fail_code(plan.reason), "stopped": bool(self.stopped),
                        "reason": f"replay refused: {plan.reason}"}
            why = self._transit_violation(p, plan.path, side)
            if why:
                return {"ok": False, "executed": False, "code": "transit", "stopped": bool(self.stopped),
                        "reason": f"replay refused: {why}"}
            res = self._run(f"{side} replay", plan.path, self._speed(speed))
            res["final_tcp"] = self._planner(self.io.measured_q16()).tcp(self.io.measured_q16()[2:])[side]
            self._log({"call": "move_joints", "side": side, "result": res})
            return res

    def head(self, pitch=None, yaw=None) -> dict:
        """Point the head camera: pitch rad (+ = look down, |pitch| <= 0.60), yaw rad (+ = left, |yaw| <= 0.50).
        Arms and grippers hold; the target persists across later motions."""
        goal = {}
        for m, v, name in ((G.HEAD_PITCH, pitch, "pitch"), (G.HEAD_YAW, yaw, "yaw")):
            if v is None:
                continue
            lo, hi = G.HEAD_LIMIT[m]
            if not (isinstance(v, (int, float)) and lo <= float(v) <= hi):
                return {"ok": False, "executed": False, "code": "invalid_argument", "stopped": bool(self.stopped),
                        "reason": f"head {name} must be in [{lo}, {hi}] rad"}
            goal[m] = float(v)
        if not goal:
            return {"ok": False, "executed": False, "code": "invalid_argument", "stopped": bool(self.stopped),
                    "reason": "give pitch and/or yaw"}
        with self.lock:
            q16 = self.io.measured_q16()
            extra = self.io.hold_extra()
            start = {m: self.builder.target.get(m, extra[m]) for m in goal}
            dur = max(abs(goal[m] - start[m]) for m in goal) / G.HEAD_RATE
            res = self._run(f"head -> {goal}", [q16[2:], q16[2:]], 0.3, grip_goal=goal, min_seconds=dur + 0.5)
            if res["ok"] and self.mode == "real":
                time.sleep(0.3)
            ex2 = self.io.hold_extra()
            res["head_pitch"], res["head_yaw"] = round(ex2[G.HEAD_PITCH], 4), round(ex2[G.HEAD_YAW], 4)
            self._log({"call": "head", "goal": goal, "result": res})
            return res

    def home(self, side="both", speed=None) -> dict:
        """Glide to the ready pose. If the swing would pass below the table (e.g. starting from arms hanging in
        zero-torque mode), raise the lift first to the lowest height where the whole swing clears the table."""
        with self.lock:
            q16 = self.io.measured_q16()
            q1 = self._home_target(q16, side)
            p = self._planner(q16)
            plan = p.plan_joints(q16[2:], q1)
            h_new = None
            if not plan.ok and "above the table" in str(plan.reason) and self.fixed_lift is None:
                h0 = float(q16[X.I_LIFT])
                for h in np.arange(round(h0, 3) + 0.02, X.LIFT_RANGE[1] - 0.01 + 1e-9, 0.02):
                    p.set_lift(h0)
                    if p.plan_joints(q16[2:], q1, dz=h - p.lift).ok and \
                            all(p.check_path([q16[2:]], s)[0] is None for s in ("left", "right")):
                        h_new = float(h)
                        break
                p.set_lift(h0)
                if h_new is None:
                    return {"ok": False, "executed": False, "code": "table", "stopped": bool(self.stopped),
                            "reason": f"home refused: {plan.reason}; no lift height clears the table "
                                      "(check scene table_z / clear the space in front)"}
            elif not plan.ok:
                return {"ok": False, "executed": False, "code": fail_code(plan.reason), "stopped": bool(self.stopped),
                        "reason": f"home refused: {plan.reason}"}
        if h_new is not None:
            lr = self.lift(h_new)
            if not lr["ok"]:
                return {**lr, "reason": f"home: raising the lift to {h_new:.3f} m first failed: {lr['reason']}"}
        with self.lock:
            q16 = self.io.measured_q16()
            q1 = self._home_target(q16, side)
            plan = self._planner(q16).plan_joints(q16[2:], q1)
            if not plan.ok:
                return {"ok": False, "executed": False, "code": fail_code(plan.reason), "stopped": bool(self.stopped),
                        "reason": f"home refused: {plan.reason}"}
            res = self._run(f"home {side}", plan.path, self._speed(0.4 if speed is None else speed))
            if h_new is not None:
                res["auto_lift_m"] = round(h_new, 3)
            self._log({"call": "home", "side": side, "result": res})
            return res

    def waist(self, target=0.0) -> dict:
        """Turn the waist (torso yaw) to `target` rad at a slow rate; arms keep their joint angles, lift fixed.
        Only for re-centring away from the table (the hands swing sideways with the torso)."""
        t = float(target)
        if not -0.35 <= t <= 0.35:
            return {"ok": False, "executed": False, "code": "invalid_argument", "stopped": bool(self.stopped),
                    "reason": "waist target must be within ±0.35 rad"}
        with self.lock:
            q16 = self.io.measured_q16()
            w0 = float(q16[X.I_WAIST])
            if abs(t - w0) < 0.005:
                return {"ok": True, "executed": False, "reason": None, "code": "ok", "stopped": bool(self.stopped),
                        "waist_rad": w0, "note": "already there"}
            p = make_planner(float(q16[X.I_LIFT]), t, self.scene)
            for side in ("left", "right"):
                why, _, _ = p.check_path([q16[2:]], side)
                if why:
                    return {"ok": False, "executed": False, "code": fail_code(why), "stopped": bool(self.stopped),
                            "reason": f"at waist {t:.3f} the {side} arm: {why}"}
            n = max(2, int(np.ceil(abs(t - w0) / 0.02)))
            wps = [np.r_[w0 + (t - w0) * k / n, q16[2:]] for k in range(n + 1)]
            res = self._run(f"waist {w0:.3f} -> {t:.3f} rad", None, 0.12, path15=wps)
            res["waist_rad"] = round(float(self.io.measured_q16()[X.I_WAIST]), 4)
            self.planner = None
            self._log({"call": "waist", "target": t, "result": res})
            return res

    def lift(self, height_m, speed=None) -> dict:
        """Drive the lift to height_m (arms and waist keep their joint angles, so both hands move by the
        same vertical amount). Planned at LIFT_SPEED, checked for the table at the new height."""
        h1 = float(height_m)
        if self.fixed_lift is not None and abs(h1 - self.fixed_lift) > 0.003:
            return {"ok": False, "executed": False, "code": "lift_range", "stopped": bool(self.stopped),
                    "reason": f"fixed-lift mode: the lift stays at {self.fixed_lift:.3f} m"}
        with self.lock:
            q16 = self.io.measured_q16()
            h0 = float(q16[X.I_LIFT])
            lo, hi = X.lift_floor(h0, 0.0), X.LIFT_RANGE[1]
            if not lo <= h1 <= hi:
                return {"ok": False, "executed": False, "reason": f"lift {h1:.3f} m outside [{lo:.3f}, {hi:.3f}] m"}
            if abs(h1 - h0) < 0.003:
                return {"ok": True, "executed": False, "reason": None, "lift_m": h0, "note": "already there"}
            p_new = self._planner(q16)
            p_new.set_lift(h1)
            for s in ("left", "right"):
                why, _, _ = p_new.check_path([q16[2:]], s)
                if why:
                    return {"ok": False, "executed": False, "reason": f"at lift {h1:.3f} m the {s} arm: {why}"}
            n = max(2, int(np.ceil(abs(h1 - h0) / (0.5 * X.LIFT_MAX_STEP))))
            path16 = [np.r_[h0 + (h1 - h0) * k / n, q16[1:]] for k in range(n + 1)]
            res = self._run(f"lift {h0:.3f} -> {h1:.3f} m", None, 0.3, path16=path16)
            res["lift_m"] = round(float(self.io.measured_q16()[X.I_LIFT]), 4)
            self._log({"call": "lift", "height_m": h1, "result": res})
            return res

    def stop(self, clear=False) -> dict:
        if clear:
            self.stop_event.clear()
            self.stopped = None
            if self.mode == "shadow":
                self._resync_twin()
            return {"ok": True, "stopped": None}
        self.stop_event.set()
        self.stopped = self.stopped or "operator stop"
        return {"ok": True, "stopped": self.stopped}

    def resync(self) -> dict:
        if self.mode != "shadow":
            return {"ok": False, "reason": "resync is for shadow mode"}
        with self.lock:
            self._resync_twin()
        return {"ok": True}

    # ------------------------------------------------------------------ taught poses
    def load_poses(self) -> dict:
        if self.poses_path and self.poses_path.exists():
            return json.loads(self.poses_path.read_text()).get("poses", {})
        return {}
