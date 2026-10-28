"""SLAMWARE map navigation over the slamware_ros_sdk_server_node DDS topics.

DDS is lazy and shares RobotController's channel initialization. This is NOT the open-loop odometry used by the VLM pilot.
"""
from __future__ import annotations

import math
import threading
import time

from core.places import finite_pose


def source_stamp(msg):
    try:
        stamp = msg.header.stamp
        value = float(stamp.sec)+float(stamp.nanosec)*1e-9
        return value if math.isfinite(value) and value > 0 else None
    except (AttributeError, TypeError, ValueError):
        return None


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def pre_align_turn_deg(pose, goal, threshold_deg, min_distance_m=0.5):
    """Signed in-place turn (deg, +left) that faces `goal` before an onboard goto, or 0.

    On the robot, goals ahead drive fine, but goals placed behind it (e.g. several
    metres back / return-to-start) may never move the base and stall at stall_s.
    The lidar has no returns behind ±133°, so the goal is pointed forward first. Near goals
    (< min_distance_m) are left to SLAMWARE's own final-yaw alignment."""
    if threshold_deg <= 0:
        return 0.0
    dx, dy = goal["x"] - pose["x"], goal["y"] - pose["y"]
    if math.hypot(dx, dy) < min_distance_m:
        return 0.0
    err = math.degrees(wrap_angle(math.atan2(dy, dx) - pose["yaw"]))
    return err if abs(err) > threshold_deg else 0.0


def resolve_goal_frame(nav_cfg):
    """frame_id for onboard goals: explicit navigation.goal_frame, else the pose frame.

    Decoupled because onboard goto goals are published in 'map' while the
    pose stream (rt/robot_pose) may carry a different header frame (e.g. 'robot_pose').
    An empty/absent goal_frame falls back to the pose frame, preserving prior behavior.
    """
    return (nav_cfg.get('goal_frame') or nav_cfg.get('frame'))


class Localization:
    """Monotonic freshness for map pose, measured twist and SLAM state separately."""
    def __init__(self, frame, max_age_s=1.0, now=time.monotonic, *, strict_timestamps=False, source_offset_s=0., wall_now=time.time, receipt_time_freshness=False,
                 state_max_age_s=None):
        if not frame or not math.isfinite(float(max_age_s)) or float(max_age_s) <= 0:
            raise ValueError("frame 和 max_age_s 无效")
        if state_max_age_s is not None and (not math.isfinite(float(state_max_age_s)) or float(state_max_age_s) <= 0):
            raise ValueError("state_max_age_s 无效")
        self.frame, self.max_age_s, self.now = frame, float(max_age_s), now
        # slamware_ros_sdk_server_node/state is published at ~1Hz (measured
        # mean gap ~1.005s, most gaps >1s), so the 1.0s pose/odom limit flags it stale
        # for a few ms every second and the 10Hz nav safety loop aborts goto with
        # navigation_safety_lost. None keeps the old shared limit.
        self.state_max_age_s = self.max_age_s if state_max_age_s is None else float(state_max_age_s)
        self._lock = threading.Lock()
        self._pose = self._odom = self._state = None
        self.strict_timestamps, self.source_offset_s, self.wall_now = strict_timestamps, source_offset_s, wall_now
        self.receipt_time_freshness = receipt_time_freshness
        self._sources = {}

    def _timestamp(self, name, msg):
        source, received = source_stamp(msg), self.now()
        if self.strict_timestamps:
            previous = self._sources.get(name)
            if self.receipt_time_freshness:
                # The onboard SLAMWARE clock runs ~181s behind (and drifts a few s/day)
                # of the host wall clock, so freshness is judged by receipt time instead of
                # the header stamp. We keep the anti-replay guard:
                # a missing/duplicate/older source stamp is still rejected so a replayed
                # sample cannot masquerade as new post-stop feedback. Only the freshness
                # clock changes — snapshot() gates on receipt age (0 <= now-ts <= max_age).
                if source is None or (previous is not None and source <= previous):
                    return -math.inf
                self._sources[name] = source
                return received
            if (source is None or not 0 <= self.wall_now()-(source+self.source_offset_s) <= self.max_age_s
                    or (previous is not None and source <= previous)):
                return -math.inf
            self._sources[name] = source
            # Event time on the monotonic axis, not receipt time. Delayed but
            # admissible telemetry must not count as post-stop feedback.
            return received-(self.wall_now()-(source+self.source_offset_s))
        return received

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        norm = sum(float(v)*float(v) for v in values[2:])
        valid = all(math.isfinite(float(v)) for v in values) and 0.8 < norm < 1.2
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z)) if valid else float("nan")
        with self._lock:
            self._pose = {"x": p.x, "y": p.y, "yaw": yaw, "frame": msg.header.frame_id,
                          "ts": self._timestamp("pose", msg), "source_stamp": source_stamp(msg)}

    def on_odom(self, msg):
        t = msg.twist.twist
        with self._lock:
            self._odom = {"speed": math.hypot(t.linear.x, t.linear.y), "wz": t.angular.z,
                          "vx": t.linear.x, "vy": t.linear.y, "child_frame": getattr(msg, "child_frame_id", ""),
                          "source_stamp": source_stamp(msg), "ts": self._timestamp("odom", msg)}

    def on_state(self, msg):
        # slamware_ros_sdk_server_node/state is std_msgs/String (headerless).
        # In strict mode receipt time is the only trustworthy freshness clock;
        # treating a missing header as stale would block every real preflight.
        with self._lock:
            self._state = {"state": msg.data, "ts": self.now(), "source_stamp": None}

    def snapshot(self):
        now = self.now()
        with self._lock:
            p = dict(self._pose) if self._pose else None
            o = dict(self._odom) if self._odom else None
            s = dict(self._state) if self._state else None
        errors = []
        for name, sample, limit in (("pose", p, self.max_age_s), ("odom", o, self.max_age_s),
                                    ("slam_state", s, self.state_max_age_s)):
            if sample is None or not 0 <= now - sample["ts"] <= limit:
                errors.append(name + "_stale")
        if p:
            if p["frame"] != self.frame:
                errors.append("frame_mismatch")
            if not all(math.isfinite(p[k]) for k in ("x", "y", "yaw")):
                errors.append("invalid_pose")
        if o and not all(math.isfinite(o[k]) for k in ("speed", "wz")):
            errors.append("invalid_twist")
        if s and s["state"] != "connected":
            errors.append("slam_not_connected")
        # Never emit NaN/Infinity in MCP JSON, even for invalid telemetry.
        for sample in (p, o):
            if sample:
                for key, value in sample.items():
                    if isinstance(value, (int, float)) and not math.isfinite(value):
                        sample[key] = None
        return {"ok": not errors, "errors": errors, "pose": p, "odom": o,
                "slam_state": s["state"] if s else "unknown",
                "pose_age_s": None if p is None or p["ts"] is None else round(now-p["ts"], 3)}


class SlamwareBackend:
    def __init__(self, robot, localization, goal_frame=None):
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.builtin_interfaces.msg.dds_ import Time_
        from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import (
            Point_,
            Pose_,
            PoseStamped_,
            Quaternion_,
        )
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_

        from services.slamware_idl import CancelActionRequest_
        self.robot = robot
        self._goal = ChannelPublisher("rt/move_base_simple/goal", PoseStamped_)
        self._goal.Init()
        self._cancel = ChannelPublisher("rt/slamware_ros_sdk_server_node/cancel_action", CancelActionRequest_)
        self._cancel.Init()
        self._subs = self.subscribe(localization)
        self._make_cancel = lambda: CancelActionRequest_(0)
        # Goal frame is independent of the pose frame; the onboard SLAMWARE planner reads
        # this header frame_id when planning the point-to-point route (typically 'map').
        goal_frame = goal_frame or localization.frame
        def make_goal(p):
            ns = time.time_ns()
            return PoseStamped_(header=Header_(stamp=Time_(sec=ns//10**9, nanosec=ns%10**9),
                                               frame_id=goal_frame),
                                pose=Pose_(position=Point_(x=p["x"], y=p["y"], z=0.),
                                           orientation=Quaternion_(0., 0., math.sin(p["yaw"]/2), math.cos(p["yaw"]/2))))
        self._make_goal = make_goal

    @staticmethod
    def subscribe(localization):
        """Read-only DDS subscriptions, usable before a map/navigation manager exists."""
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_
        from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
        subs = []
        for topic, typ, cb in (("rt/robot_pose", PoseStamped_, localization.on_pose),
                               ("rt/agv/odom", Odometry_, localization.on_odom),
                               ("rt/slamware_ros_sdk_server_node/state", String_, localization.on_state)):
            sub = ChannelSubscriber(topic, typ)
            sub.Init(cb, 1)
            subs.append(sub)
        return subs

    def acquire(self):
        self.robot.acquire_navigation()

    def release(self):
        self.robot.release_navigation()

    def goal(self, pose):
        if self._goal.Write(self._make_goal(pose)) is False:
            raise RuntimeError("导航目标 DDS 发布失败")

    def halt(self):
        # No fallback goal with stale pose. Cancellation is best effort at the DDS
        # layer: manager additionally requires fresh measured stationary feedback.
        error = None
        try:
            if self._cancel.Write(self._make_cancel()) is False:
                raise RuntimeError("cancel DDS 发布失败")
        except Exception as exc:
            error = exc
        for _ in range(3):
            try:
                self.robot.navigation_zero()
            except Exception as exc:
                error = exc
            time.sleep(0.05)
        if error:
            raise error


class NavigationManager:
    """One async goal; a fault holds exclusive ownership until stop is confirmed.

    It never declares arrival using commanded odometry, nor treats missing twist
    as zero. The outer sidecar arbitrates this mode against move/turn/sequence.
    """
    def __init__(self, localization, backend, store, *, timeout_s=120., max_distance_m=15.,
                 position_tolerance_m=0.15, yaw_tolerance_deg=10., settle_s=1.,
                 stall_s=20., now=time.monotonic, start_thread=True, sleep=time.sleep, safety_check=None,
                 odom_required=True, position_hold_s=8.):
        for value in (timeout_s, max_distance_m, position_tolerance_m, yaw_tolerance_deg, settle_s, stall_s, position_hold_s):
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError("导航参数须为有限正数")
        if not isinstance(odom_required, bool):
            raise ValueError("odom_required 须为布尔值")
        self.sleep = sleep
        self.safety_check = safety_check
        self.localization, self.backend, self.store = localization, backend, store
        self.timeout_s, self.max_distance_m = timeout_s, max_distance_m
        self.pos_tol, self.yaw_tol = position_tolerance_m, math.radians(yaw_tolerance_deg)
        self.settle_s, self.stall_s, self.now = settle_s, stall_s, now
        self.odom_required, self.position_hold_s = odom_required, position_hold_s
        self.start_thread = start_thread
        self._lock = threading.RLock()
        self._cancel = threading.Event()
        self._stop_waiters_lock = threading.Lock()
        self._stop_waiters = {}
        self._thread = None
        self._owned = False
        self._result = {"ok": True, "state": "idle", "active": False}
        self._settled = None
        self._best = math.inf
        self._started = self._progress_at = 0.
        self._goal = None

    def status(self):
        with self._lock:
            return {**self._result, "active": self._owned, "map_id": self.store.map_id,
                    "frame": self.store.frame}

    @staticmethod
    def _stationary(snapshot):
        return snapshot["ok"] and snapshot["odom"]["speed"] < 0.03 and abs(snapshot["odom"]["wz"]) < 0.05

    def _nav_ok(self, snapshot):
        """snapshot['ok'], but tolerant of a missing measured twist when odom isn't required.

        Onboard SLAM goto needs only a fresh map pose + connected SLAM; odom is
        optional. smooth keeps odom_required=True, so its freshness gate is byte-unchanged.
        """
        if self.odom_required:
            return snapshot["ok"]
        tolerated = {"odom_stale", "invalid_twist"}
        return not [e for e in snapshot["errors"] if e not in tolerated]

    def _safety(self, goal=None):
        if self.safety_check is None:
            return {"ok": True, "errors": []}
        try:
            return self.safety_check(goal)
        except Exception as exc:
            return {"ok": False, "errors": ["navigation_safety_unavailable: " + str(exc)]}

    def save_place(self, name, overwrite=False):
        with self._lock:
            checked = self._safety()
            if not checked['ok']:
                raise ValueError('navigation_safety_blocked: ' + ','.join(checked['errors']))
            s = self.localization.snapshot()
            if self._owned or not self._stationary(s):
                raise ValueError("标定须导航空闲、定位新鲜、底盘静止；" + ",".join(s["errors"]))
            return self.store.save(name, s["pose"], overwrite=overwrite)

    def start(self, name, *, cancel=None, request_id=None):
        with self._lock:
            if self._owned:
                return {**self.status(), "ok": False, "error": "navigation_busy_or_stop_unconfirmed"}
            self._cancel = cancel if cancel is not None else threading.Event()
            if self._cancel.is_set():
                return {"ok": False, "active": False, "state": "cancelled"}
            goal = self.store.get(name)
            checked = self._safety(goal)
            if not checked['ok']:
                return dict(ok=False, active=False, error='navigation_safety_blocked', safety=checked)
            s = self.localization.snapshot()
            # Onboard SLAM pose-only start cannot measure stationarity without odom; it
            # requires only a fresh map pose + connected SLAM (safety_check already ran).
            ready = self._stationary(s) if self.odom_required else self._nav_ok(s)
            if not ready:
                return {"ok": False, "error": "定位/实测速度不可用，或底盘未停稳", "localization": s}
            p = finite_pose(s["pose"])
            if math.hypot(goal["x"]-p["x"], goal["y"]-p["y"]) > self.max_distance_m:
                return {"ok": False, "error": "目标超过地图导航距离上限"}
            self._owned = True
            self._goal = goal
            self._progress_position = (p["x"], p["y"])
            self._started = self._progress_at = self.now()
            self._settled, self._best = None, math.inf
            self._result = {"ok": True, "state": "running", "place": name, "goal": goal, "request_id": request_id}
            try:
                self.backend.acquire()
                if not self._cancel.is_set():
                    self.backend.goal(goal)
                else:
                    self._result.update(state="stopping", ok=False, completion_reason="cancelled")
            except Exception as exc:
                self._result = {"ok": False, "state": "fault", "error": str(exc)}
                try:
                    self.backend.halt()
                except Exception:
                    pass
                # Keep ownership: a publish timeout may still have delivered a goal.
                return self.status()
            if self.start_thread:
                self._thread = threading.Thread(target=self._run, daemon=True)
                self._thread.start()
            return self.status()

    def step(self):
        """Pure progress verdict; the worker performs cancellation / final release."""
        with self._lock:
            now, s = self.now(), self.localization.snapshot()
            if self._cancel.is_set():
                return "cancelled"
            if now - self._started >= self.timeout_s:
                return "timeout"
            checked = self._safety(self._goal)
            if not checked['ok']:
                self._result['safety'] = checked
                return "navigation_safety_lost"
            if not self._nav_ok(s):
                return "localization_lost"
            p, g = s["pose"], self._goal
            d = math.hypot(g["x"]-p["x"], g["y"]-p["y"])
            angle = abs(wrap_angle(g["yaw"]-p["yaw"]))
            self._result.update(distance_m=round(d, 3), yaw_error_deg=round(math.degrees(angle), 1),
                                elapsed_s=round(now-self._started, 2))
            in_tol = d <= self.pos_tol and angle <= self.yaw_tol
            # With odom, confirm arrival by measured stationarity held for settle_s. Without
            # it (onboard pose-only), fall back to holding pose+yaw within tolerance for
            # position_hold_s.
            arrived = (in_tol and self._stationary(s)) if self.odom_required else in_tol
            hold_s = self.settle_s if self.odom_required else self.position_hold_s
            if arrived:
                self._settled = now if self._settled is None else self._settled
                if now-self._settled >= hold_s:
                    return "arrived"
            else:
                self._settled = None
            xy = (p["x"], p["y"])
            # Onboard routes are unavailable. Net measured displacement permits
            # detours; small stationary jitter cannot keep resetting the watchdog.
            if d < self._best - 0.03 or math.dist(xy, self._progress_position) >= .1:
                self._best, self._progress_at, self._progress_position = d, now, xy
            # Orientation convergence at the target is bounded by the total timeout.
            if d > self.pos_tol and now-self._progress_at >= self.stall_s:
                return "stalled"
            return "running"

    def _finish(self, verdict):
        with self._lock:
            self._result.update(state="stopping", ok=False, completion_reason=verdict)
        try:
            self.backend.halt()
            # New feedback must show stationary for a continuous interval. A failed
            # cancellation does not release manual control just because the RPC ended.
            cutoff = self.localization.now()
            deadline, settled = self.now()+3., None
            prev_xy = prev_ts = None
            pose_stationary = False  # last pose-only verdict, held across repeated/stale reads
            while self.now() < deadline:
                s = self.localization.snapshot()
                if self.odom_required:
                    sample_ts = min(s["pose"]["ts"], s["odom"]["ts"]) if s["ok"] else -math.inf
                    stationary = self._stationary(s)
                else:
                    # Pose-only stop confirmation. Measure SPEED across consecutive DISTINCT
                    # poses and gate it at the odom _stationary bound (0.03 m/s). A per-sample
                    # displacement gate would read a slow creep as "stationary" (0.05s loop ×
                    # 0.15 m/s < 2cm) and release the servo lock mid-motion — unsafe. Zero-mean
                    # localization jitter stays under the gate; a real creep accumulates and
                    # keeps resetting the dwell, so the lock is held (fault) rather than freed.
                    ok = self._nav_ok(s)
                    ts = s["pose"]["ts"] if (ok and s.get("pose")) else None
                    if ts is not None and (prev_ts is None or ts > prev_ts):
                        xy = (s["pose"]["x"], s["pose"]["y"])
                        pose_stationary = (prev_xy is not None and prev_ts is not None
                                           and math.dist(xy, prev_xy) < 0.03*(ts-prev_ts))
                        prev_xy, prev_ts = xy, ts
                    # A repeated or stale sample carries the last verdict forward: it must
                    # neither confirm new stillness nor reset dwell already accumulated.
                    stationary = pose_stationary
                    sample_ts = prev_ts if prev_ts is not None else -math.inf
                if stationary and sample_ts > cutoff:
                    settled = sample_ts if settled is None else settled
                    if sample_ts-settled >= 0.5:
                        with self._lock:
                            self.backend.release()
                            self._owned = False
                            if verdict == "arrived":
                                p, g = s["pose"], self._goal
                                if self._cancel.is_set():
                                    verdict = "cancelled"
                                elif (math.hypot(g["x"]-p["x"], g["y"]-p["y"]) > self.pos_tol
                                      or abs(wrap_angle(g["yaw"]-p["yaw"])) > self.yaw_tol):
                                    verdict = "arrival_lost"
                            self._result.update(state=verdict, ok=verdict == "arrived", arrival_sample_ts=sample_ts)
                        return
                else:
                    settled = None
                self.sleep(0.05)
            raise RuntimeError("未确认停稳；保留控制锁，现场检查/硬件急停后再次 stop")
        except Exception as exc:
            with self._lock:
                self._result.update(state="fault", ok=False, error=str(exc))

    def _run(self):
        try:
            while True:
                verdict = self.step()
                if verdict != "running":
                    break
                self._cancel.wait(0.1)
        except Exception as exc:
            with self._lock:
                self._result["error"] = str(exc)
            verdict = "fault"
        self._finish(verdict)

    def _defer_stop(self, cancelled):
        """Finish a stop after in-flight manager I/O, not on its caller.

        The event is already latched. One waiter per session also handles failed
        startup (where no navigation worker exists). An old waiter must never
        stop a new session. A failed stop confirmation still retains ownership.
        """
        with self._stop_waiters_lock:
            if cancelled not in self._stop_waiters:
                def wait():
                    try:
                        with self._lock:
                            if self._cancel is cancelled and cancelled.is_set():
                                self.stop()
                    finally:
                        with self._stop_waiters_lock:
                            self._stop_waiters.pop(cancelled, None)
                thread = threading.Thread(target=wait, daemon=True)
                self._stop_waiters[cancelled] = thread
                thread.start()
        return {"ok": False, "active": True, "state": "stop_requested", "stop_requested": True}

    def stop(self):
        cancelled = self._cancel
        cancelled.set()  # visible even while startup/command I/O owns the lock
        if not self._lock.acquire(blocking=False):
            return self._defer_stop(cancelled)
        try:
            if self._owned and (self._thread is None or not self._thread.is_alive()):
                self._thread = threading.Thread(target=self._finish, args=("cancelled",), daemon=True)
                self._thread.start()
            return {**self.status(), "stop_requested": True}
        finally:
            self._lock.release()
