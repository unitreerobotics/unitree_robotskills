# -*- coding: utf-8 -*-
"""Lidar reflex layer: collision-avoidance speed clamping that bypasses the LLM.

Subscribes to the SLAMWARE lidar DDS topic, keeps the latest scan, and clamps the
executor's commanded velocity every tick:
  nearest obstacle ahead < stop_m        -> forward vx zeroed (reflex_blocked)
  stop_m <= nearest < slow_m             -> vx scaled linearly into [0, 1]x (legacy, G1D default)
  or a distance-based speed envelope (headway_s>0, R1D default):
    allowed = max(creep_mps, (clearance - stop_m) / headway_s), i.e. at the current speed the
    robot cannot reach the stop line within headway_s; near obstacles it still creeps at
    creep_mps and only stops at stop_m
  no / stale scan (> max_age_s)          -> forward blocked if require=True, else allowed (offline stub)
Reversing applies the same rules to the rear sector. Yaw (vyaw) is never clamped: turning
in place is the escape manoeuvre.
Optional travel corridor (corridor_half_width_m>0): near returns inside a body-width strip
also count as clearance, covering the gap where the sector is narrower than the chassis.
With sector_guard=False (R1D) forward motion only considers the travel path (path_min):
a body-width strip when driving straight, the annulus swept along the arc when turning.
In a corridor the +/-30 deg sector hits both side walls (front_m ~1 m in a 1 m corridor), so
limiting on it slows the robot despite a clear path; sector readings stay in status for reference.
Caveat: the lidar is a single plane; objects below the scan plane (fallen robot limbs,
cables, low bars) are invisible to both methods.

Sector-min / scan-summary logic is pure (offline-testable); the DDS subscription is imported
lazily in LidarGuard.start(), so this module imports without the SDK.
"""
from __future__ import annotations

import math
import threading
import time

RANGE_VALID_MIN_M = 0.05      # below this is sensor noise, not a wall

FRONT_SECTOR_DEG = (-30.0, 30.0)
LEFT_SECTOR_DEG = (30.0, 150.0)
RIGHT_SECTOR_DEG = (-150.0, -30.0)
REAR_SECTOR_DEG = (150.0, -150.0)     # wraps across the +/-180 seam

T_SCAN = "rt/slamware_ros_sdk_server_node/scan"


def sector_min(angle_min: float, angle_inc: float, ranges,
               lo_deg: float, hi_deg: float, yaw_offset_deg: float = 0.0) -> float | None:
    """Minimum valid range within [lo_deg, hi_deg]. Handles negative angle_increment and sectors wrapping +/-180."""
    best: float | None = None
    for i, r in enumerate(ranges):
        if not (r and math.isfinite(r)) or r < RANGE_VALID_MIN_M:
            continue
        a = math.degrees(angle_min + i * angle_inc) - yaw_offset_deg
        a = (a + 180.0) % 360.0 - 180.0
        inside = (lo_deg <= a <= hi_deg) if lo_deg <= hi_deg else (a >= lo_deg or a <= hi_deg)
        if inside and (best is None or r < best):
            best = r
    return best


def corridor_min(angle_min: float, angle_inc: float, ranges, half_width_m: float,
                 yaw_offset_deg: float = 0.0, rear: bool = False) -> float | None:
    """Longitudinal distance (m) to the nearest obstacle in the body-width travel corridor
    (None = no returns in the corridor, or disabled).

    At short range the +/-30 deg sector is narrower than the chassis (half-width only ~0.26 m
    at 0.45 m), so door frames / table legs clipping a chassis corner would be missed. Returns
    are projected into the body frame; among points with |y| <= half_width_m on the travel
    side (x>0, or x<0 when rear=True) the minimum |x| is taken. Longitudinal distance <=
    slant range, so this is more conservative than the sector."""
    if half_width_m <= 0:
        return None
    best: float | None = None
    for i, r in enumerate(ranges):
        if not (r and math.isfinite(r)) or r < RANGE_VALID_MIN_M:
            continue
        a = math.radians(math.degrees(angle_min + i * angle_inc) - yaw_offset_deg)
        x, y = r * math.cos(a), r * math.sin(a)
        if rear:
            x = -x
        if x > 0 and abs(y) <= half_width_m and (best is None or x < best):
            best = x
    return best


STRAIGHT_CURVATURE = 1e-3     # |vyaw/vx| below this (turn radius > 1 km) counts as straight
PATH_MAX_TURN_RAD = math.pi / 2   # look a quarter turn ahead only; farther points aren't reachable by this command


def path_min(angle_min: float, angle_inc: float, ranges, half_width_m: float, vx: float, vyaw: float,
             yaw_offset_deg: float = 0.0) -> float | None:
    """Distance along the path (m) to the nearest obstacle on the travel path of the current
    command (vx, vyaw); None = no returns on the path.

    The front edge (scan origin) moves along an arc of radius R = vx/vyaw, sweeping an annulus
    centred on the turn centre with radii |R| +/- half_width. For points in the annulus the
    distance is the arc length until the front reaches their bearing. For large |R| this
    degenerates to the corridor_min strip. Forward only (vx>0); half_width_m<=0 returns None."""
    if half_width_m <= 0 or vx <= 0:
        return None
    k = vyaw / vx
    if abs(k) < STRAIGHT_CURVATURE:
        return corridor_min(angle_min, angle_inc, ranges, half_width_m, yaw_offset_deg)
    r_turn = 1.0 / k                       # signed: positive = turn centre on the left at (0, R)
    s = 1.0 if k > 0 else -1.0
    best: float | None = None
    for i, r in enumerate(ranges):
        if not (r and math.isfinite(r)) or r < RANGE_VALID_MIN_M:
            continue
        a = math.radians(math.degrees(angle_min + i * angle_inc) - yaw_offset_deg)
        x, y = r * math.cos(a), r * math.sin(a)
        if abs(math.hypot(x, y - r_turn) - abs(r_turn)) > half_width_m:
            continue
        # relative to the turn centre the front is at (0, -R) and the point at (x, y-R); measure the angle in the travel direction
        phi = math.atan2(s * r_turn * x, -r_turn * (y - r_turn)) % (2 * math.pi)
        if phi <= PATH_MAX_TURN_RAD:
            d = abs(r_turn) * phi
            if best is None or d < best:
                best = d
    return best


def scan_summary(scan: dict, yaw_offset_deg: float = 0.0, corridor_half_width_m: float = 0.0) -> dict:
    """Min range (m) per front/left/right/rear sector (None = no valid returns); adds corridor_front/rear_m when the corridor is enabled."""
    args = (scan["angle_min"], scan["angle_inc"], scan["ranges"])
    s = {
        "front_m": sector_min(*args, *FRONT_SECTOR_DEG, yaw_offset_deg),
        "left_m": sector_min(*args, *LEFT_SECTOR_DEG, yaw_offset_deg),
        "right_m": sector_min(*args, *RIGHT_SECTOR_DEG, yaw_offset_deg),
        "rear_m": sector_min(*args, *REAR_SECTOR_DEG, yaw_offset_deg),
        "n_valid": sum(1 for r in scan["ranges"]
                       if r and math.isfinite(r) and r >= RANGE_VALID_MIN_M),
    }
    if corridor_half_width_m > 0:
        s["corridor_front_m"] = corridor_min(*args, corridor_half_width_m, yaw_offset_deg)
        s["corridor_rear_m"] = corridor_min(*args, corridor_half_width_m, yaw_offset_deg, rear=True)
    return s


def _nearest(*vals):
    vals = [v for v in vals if v is not None]
    return min(vals) if vals else None


def limit_forward(vx: float, clear_m: float | None, stop_m: float, slow_m: float,
                  data_ok: bool, require: bool, headway_s: float = 0.0, creep_mps: float = 0.0) -> tuple[float, str]:
    """Single-axis clamp: returns (clamped speed, reason), reason in ok / slow / blocked / no_data.

    vx is the positive component along the travel direction (caller passes the front or rear
    clearance accordingly). clear_m=inf means no returns on the path. With headway_s>0 the
    envelope max(creep_mps, (clear_m - stop_m) / headway_s) applies; otherwise the linear slow_m zone."""
    if vx <= 0:
        return vx, "ok"
    if not data_ok or clear_m is None:
        return (0.0, "no_data") if require else (vx, "ok")
    if clear_m < stop_m:
        return 0.0, "blocked"
    if headway_s > 0:
        allowed = max(creep_mps, (clear_m - stop_m) / headway_s)
        return (allowed, "slow") if vx > allowed else (vx, "ok")
    if clear_m < slow_m:
        k = (clear_m - stop_m) / max(slow_m - stop_m, 1e-6)
        return vx * k, "slow"
    return vx, "ok"


class LidarGuard:
    """Lidar reflex layer: DDS subscription + per-tick clamping. start() never raises (records error; require decides whether motion is allowed)."""

    def __init__(self, *, stop_m=0.45, slow_m=1.2, max_age_s=1.0, require=False,
                 yaw_offset_deg=0.0, corridor_half_width_m=0.0, now_fn=time.monotonic, log=None,
                 topic=T_SCAN, rear_blind=False, sector_guard=True, headway_s=0.0, creep_mps=0.0):
        self.topic = str(topic or T_SCAN)                           # from robot profile.lidar_topic
        # The rear is a structural blind spot (R1D's Mid-360 only sees ceiling behind): require
        # applies to forward motion only. Blocking reverse on missing rear data adds no safety
        # (reverse has no coverage anyway) and would leave the robot unable to back out.
        self.rear_blind = bool(rear_blind)
        self.stop_m = float(stop_m)
        self.corridor_half_width_m = float(corridor_half_width_m)   # 0 = +/-30 deg sector only
        # False = forward limited by travel path (path_min) only, ignoring the sector; requires the corridor
        self.sector_guard = bool(sector_guard) or self.corridor_half_width_m <= 0
        self.headway_s = float(headway_s)                           # >0 = speed envelope (see module docstring)
        self.creep_mps = float(creep_mps)
        self.slow_m = float(slow_m)
        self.max_age_s = float(max_age_s)
        self.require = bool(require)
        self.yaw_offset_deg = float(yaw_offset_deg)
        self.now_fn = now_fn
        self._log = log or (lambda *a, **k: None)
        self._lock = threading.Lock()
        self._scan: dict | None = None
        self._scan_t: float | None = None
        self.n_recv = 0
        self.error: str | None = None
        self._sub = None
        self.last_reason = "ok"          # reason from the latest limit() (for status)
        self.blocked_since: float | None = None

    def on_scan(self, msg) -> None:
        with self._lock:
            self._scan = {"angle_min": float(msg.angle_min), "angle_inc": float(msg.angle_increment),
                          "ranges": list(msg.ranges)}
            self._scan_t = self.now_fn()
            self.n_recv += 1

    def feed(self, angle_min: float, angle_inc: float, ranges) -> None:
        """Inject one scan (tests / offline)."""
        class _M:  # noqa: D401
            pass
        m = _M(); m.angle_min = angle_min; m.angle_increment = angle_inc; m.ranges = ranges
        self.on_scan(m)

    def start(self) -> bool:
        """Subscribe to the lidar topic. Must be called AFTER RobotController: the DDS channel
        (ChannelFactoryInitialize) is set up by the robot controller in services/robots/; this
        module never creates its own (lidar and chassis share the same NIC)."""
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber

            from services.slamware_idl import LaserScan_
            sub = ChannelSubscriber(self.topic, LaserScan_)
            sub.Init(self.on_scan, 1)           # queueLen 1: latest scan only
            self._sub = sub
            self._log(f"雷达已订阅 {self.topic}", "lidar")
            return True
        except Exception as e:
            self.error = f"雷达订阅失败: {type(e).__name__}: {e}"
            self._log(self.error, "lidar")
            return False

    def summary(self) -> dict:
        now = self.now_fn()
        with self._lock:
            scan, t = self._scan, self._scan_t
        if scan is None:
            return {"ok": False, "age_s": None, "n_recv": self.n_recv, "error": self.error,
                    "front_m": None, "left_m": None, "right_m": None, "rear_m": None}
        age = now - t
        s = scan_summary(scan, self.yaw_offset_deg, self.corridor_half_width_m)
        s = {k: (round(v, 2) if isinstance(v, float) else v) for k, v in s.items()}
        return {"ok": age <= self.max_age_s, "age_s": round(age, 2), "n_recv": self.n_recv,
                "error": self.error, **s}

    def limit(self, vx: float, vyaw: float) -> tuple[float, float, dict]:
        """Called by the executor every tick: returns (vx', vyaw, info). vyaw is never clamped."""
        s = self.summary()
        if vx >= 0 and not self.sector_guard:
            with self._lock:
                scan = self._scan
            clear = None
            if scan is not None:
                clear = path_min(scan["angle_min"], scan["angle_inc"], scan["ranges"],
                                 self.corridor_half_width_m, vx, vyaw, self.yaw_offset_deg)
                if clear is None:
                    clear = math.inf               # data present, no returns on path = clear
        else:
            clear = (_nearest(s["front_m"], s.get("corridor_front_m")) if vx >= 0
                     else _nearest(s["rear_m"], s.get("corridor_rear_m")))
        require = self.require and not (vx < 0 and self.rear_blind)
        v, reason = limit_forward(abs(vx), clear, self.stop_m, self.slow_m, s["ok"], require,
                                  self.headway_s, self.creep_mps)
        v = math.copysign(v, vx) if vx != 0 else 0.0
        if clear is not None and math.isinf(clear):
            clear = None                           # keep Infinity out of status JSON
        elif clear is not None:
            clear = round(clear, 2)
        now = self.now_fn()
        if reason in ("blocked", "no_data"):
            if self.blocked_since is None:
                self.blocked_since = now
                self._log(f"雷达反射：{reason}，前方净空 {clear} m，vx {vx:.2f}→0", "lidar")
        else:
            self.blocked_since = None
        self.last_reason = reason
        info = {"reason": reason, "clear_m": clear, "blocked_s":
                (round(now - self.blocked_since, 1) if self.blocked_since else 0.0)}
        if reason == "slow" and self.headway_s > 0:
            info["cap_mps"] = abs(v)               # absolute envelope cap; executor applies the final clamp
        return v, vyaw, info
