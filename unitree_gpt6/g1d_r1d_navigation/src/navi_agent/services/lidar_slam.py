# -*- coding: utf-8 -*-
"""2D lidar SLAM: R1D Mid-360 points -> robot pose + occupancy grid + relocalization.

R1D has no onboard SLAMWARE and no usable odometry (measured: rt/sportmodestate carries no
pose, rt/state_vel is unreliable, IMU yaw drifts ~8 deg/min at rest). Only the front Mid-360
is usable.

Method (Hector-SLAM-style scan-to-map matching, no odometry):
  1. Structure points: robot-frame points with height z_lo..z_hi (walls, pillars, furniture:
     vertical structure projects to a stable 2D outline; floor and ceiling excluded), 2D voxel
     downsampled; each point de-skewed by its packet timestamp using the current velocity estimate.
  2. Matching: 3-level pyramid of the occupancy map (5/10/20 cm, max-pooled + Gaussian smoothed);
     coarse-to-fine Gauss-Newton finds the pose that puts the most points on walls (initial
     guess = constant-velocity prediction).
  3. Mapping: a scan is integrated only if the match is trusted (hit cells log-odds +, scan
     sector log-odds -, so dynamic objects get cleared); at most once per second when stationary.
  4. Relocalization: saved maps carry trajectory keyposes. After restart these (+ calibrated
     points) seed a full-rotation search; top coarse scores are refined, and a result is accepted
     only with a high enough hit ratio and a clear margin over other candidates.

Limitations: no loop closure, so error accumulates over long/large runs and the map can ghost;
in long corridors or open halls ("looks the same wherever you shift") matching degenerates
(quality drops, state goes degraded/lost).

Pure numpy + OpenCV, no network or hardware: LidarSlam.process() can be fed synthetic or
recorded data for tests and replay.
"""
from __future__ import annotations

import math
import threading
import time
import uuid
from pathlib import Path

import cv2
import numpy as np

FRAME = "lidar_map"

L_OCC, L_FREE, L_MIN, L_MAX = 0.9, -0.35, -2.0, 3.5


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def compose(pose, delta):
    """pose o delta, with delta expressed in pose's own frame."""
    x, y, th = pose
    dx, dy, dth = delta
    c, s = math.cos(th), math.sin(th)
    return (x + c * dx - s * dy, y + s * dx + c * dy, wrap(th + dth))


def relative(a, b):
    """Pose of b in a's frame (a^-1 o b)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(a[2]), math.sin(a[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2]))


def structure_points(pts: np.ndarray, *, z_lo=0.15, z_hi=2.2, self_box=(-0.90, 0.03, 0.40),
                     max_range=12.0, min_range=0.3, voxel=0.05) -> np.ndarray:
    """Robot-frame 3D points -> 2D structure points [N,2] (within height band and range, outside the body box, voxel-deduplicated)."""
    if not len(pts):
        return np.zeros((0, 2), np.float32)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r = np.hypot(x, y)
    x0, x1, yh = self_box
    keep = ((z >= z_lo) & (z <= z_hi) & (r >= min_range) & (r <= max_range)
            & ~((x >= x0) & (x <= x1) & (np.abs(y) <= yh)))
    xy = pts[keep, :2]
    if not len(xy) or voxel <= 0:
        return xy.astype(np.float32)
    cells = np.floor(xy / voxel).astype(np.int64)
    _, idx = np.unique(cells[:, 0] * 1_000_003 + cells[:, 1], return_index=True)
    return xy[idx].astype(np.float32)


def deskew(pts: np.ndarray, t: np.ndarray, t_ref: float, vel) -> np.ndarray:
    """Move each point from the robot frame at its capture time to the robot frame at t_ref (constant-velocity model, vel=(vx, vy, wz) in robot frame)."""
    vx, vy, wz = vel
    if not len(pts) or (abs(vx) < 1e-4 and abs(vy) < 1e-4 and abs(wz) < 1e-4):
        return pts
    dt = (t - t_ref).astype(np.float32)                 # <= 0: points precede the reference time
    th = wz * dt
    c, s = np.cos(th), np.sin(th)
    out = pts.copy()
    # robot pose at time t = reference pose o (vx*dt, vy*dt, wz*dt)
    out[:, 0] = c * pts[:, 0] - s * pts[:, 1] + vx * dt
    out[:, 1] = s * pts[:, 0] + c * pts[:, 1] + vy * dt
    return out


# --------------------------------------------------------------------------
# Occupancy grid
# --------------------------------------------------------------------------
class GridMap:
    """Log-odds occupancy grid. Cell (row=v, col=u): u = (wx - ox)/res, v = (wy - oy)/res."""

    def __init__(self, res: float = 0.05, size_m: float = 80.0, logodds=None, origin=None):
        self.res = float(res)
        n = int(round(size_m / res)) if logodds is None else logodds.shape[0]
        self.L = np.zeros((n, n), np.float32) if logodds is None else logodds.astype(np.float32)
        self.origin = (-n * res / 2.0, -n * res / 2.0) if origin is None else (float(origin[0]), float(origin[1]))
        self.version = 0                    # +1 per write; pyramid and dilated occupancy use it for staleness
        self._levels = None
        self._levels_version = -1
        self.occ_dilated = None
        self.levels_origin = self.origin    # pyramid covers only the occupied area (plus margin); world coords of its lower-left corner
        self._bounds = None                 # cell bounds with occupancy evidence (u0, v0, u1, v1), updated incrementally
        if logodds is not None:
            ys, xs = np.nonzero(self.L > 0.5)
            if len(xs):
                self._bounds = (int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1)

    @property
    def n(self) -> int:
        return self.L.shape[0]

    def cells(self, wxy: np.ndarray) -> np.ndarray:
        return np.floor((wxy - np.asarray(self.origin, np.float32)) / self.res).astype(np.int64)

    def integrate(self, pose, xy: np.ndarray, *, bins: int = 360, free_half_fov_deg: float = 100.0):
        """Integrate robot-frame structure points xy at pose: scan sector marked free, hit cells occupied.

        Free space is marked only within +/-free_half_fov_deg ahead: the Mid-360 is pitched 30 deg
        down-forward and behind it sees only things above ~0.6 m, so low cabinets may sit in front of
        the nearest rear return; marking free space behind would erase them from the map."""
        if not len(xy):
            return
        c, s = math.cos(pose[2]), math.sin(pose[2])
        w = np.empty_like(xy)
        w[:, 0] = c * xy[:, 0] - s * xy[:, 1] + pose[0]
        w[:, 1] = s * xy[:, 0] + c * xy[:, 1] + pose[1]
        hit = self.cells(w)
        inside = (hit >= 0).all(1) & (hit < self.n).all(1)
        if not inside.any():
            return
        # Free space: nearest point per angular bin; when adjacent bins both have points, mark triangle (robot, p_i, p_i+1) free
        ang = np.arctan2(xy[:, 1], xy[:, 0])
        rng = np.hypot(xy[:, 0], xy[:, 1])
        b = ((ang + math.pi) / (2 * math.pi) * bins).astype(np.int64) % bins
        order = np.lexsort((rng, b))
        first = order[np.r_[True, b[order][1:] != b[order][:-1]]]
        near = np.full(bins, -1, np.int64)
        near[b[first]] = first
        robot = self.cells(np.array([[pose[0], pose[1]]], np.float32))[0]
        half = int(round(free_half_fov_deg / 360.0 * bins))
        tris = []
        for i in range(bins // 2 - half, bins // 2 + half):     # bin bins/2 = straight ahead (atan2=0)
            j = i + 1
            if near[i] >= 0 and near[j] >= 0:
                tris.append(np.array([robot, hit[near[i]], hit[near[j]]], np.int32))
        h = np.unique(hit[inside], axis=0)
        bonus = np.full(len(h), L_OCC, np.float32)
        if tris:
            lo = np.min([t.min(0) for t in tris], axis=0).clip(0, self.n - 1)
            hi = np.max([t.max(0) for t in tris], axis=0).clip(0, self.n - 1) + 1
            mask = np.zeros((hi[1] - lo[1], hi[0] - lo[0]), np.uint8)
            cv2.fillPoly(mask, [t - lo for t in tris], 1)
            free = mask.astype(bool)
            self.L[lo[1]:hi[1], lo[0]:hi[0]][free] += L_FREE
            inroi = (h[:, 0] >= lo[0]) & (h[:, 0] < hi[0]) & (h[:, 1] >= lo[1]) & (h[:, 1] < hi[1])
            was_free = np.zeros(len(h), bool)
            was_free[inroi] = free[h[inroi, 1] - lo[1], h[inroi, 0] - lo[0]]
            bonus[was_free] -= L_FREE                          # hit cells were just marked free by the sector; undo that
        self.L[h[:, 1], h[:, 0]] += bonus
        np.clip(self.L, L_MIN, L_MAX, out=self.L)
        b = (int(h[:, 0].min()), int(h[:, 1].min()), int(h[:, 0].max()) + 1, int(h[:, 1].max()) + 1)
        self._bounds = b if self._bounds is None else (min(self._bounds[0], b[0]), min(self._bounds[1], b[1]),
                                                       max(self._bounds[2], b[2]), max(self._bounds[3], b[3]))
        self.version += 1

    @property
    def stale(self) -> bool:
        return self._levels_version != self.version

    def levels(self, count: int = 3, sigma_cells: float = 1.0):
        """Matching pyramid [(M, res), ...]. M is the likelihood field exp(-d^2/2 sigma^2), d = distance to
        the nearest occupied cell centre: M=1 on occupied cells (zero residual when aligned, stable
        Gauss-Newton), decaying smoothly to 0 within a cell or two. Coarse levels are exact max-pools of
        occupancy (coarse cell k <- fine cells [k*s, (k+1)*s)). Also recomputes occ_dilated (for hit ratio).

        A previous "normalized log-odds + Gaussian blur" field peaked at only 0.5 for walls hit once, so
        residuals stayed at 0.5 when aligned and the fine level stepped off the peak (in synthetic tests a
        scan matched against its own map drifted 11 cm / 4 deg)."""
        if self._levels is not None and not self.stale:
            return self._levels
        # Compute only over the occupied area plus margin: the full 80 m map is 2.5M cells (~100 ms per
        # rebuild), while the visited area is much smaller. The margin must cover the coarsest field's decay
        # (sigma = 1 coarse cell = 4 fine cells, ~0 at e^-8); crop bounds aligned to 4 cells.
        step_max, margin = 1 << (count - 1), 8 << (count - 1)
        b = self._bounds or (0, 0, 0, 0)
        u0 = max(0, (b[0] - margin) // step_max * step_max)
        v0 = max(0, (b[1] - margin) // step_max * step_max)
        u1 = min(self.n, -(-(b[2] + margin) // step_max) * step_max)
        v1 = min(self.n, -(-(b[3] + margin) // step_max) * step_max)
        occ = self.L[v0:v1, u0:u1] > 0.5
        self.levels_origin = (self.origin[0] + u0 * self.res, self.origin[1] + v0 * self.res)
        out = []
        for k in range(count):
            step = 1 << k
            if step == 1:
                o = occ
            else:                                                 # exact max-pool (not cv2.dilate with an even kernel: centre anchor shifts everything by s/2 cells)
                rv, ru = (occ.shape[0] // step) * step, (occ.shape[1] // step) * step
                o = occ[:rv, :ru].reshape(rv // step, step, ru // step, step).any(axis=(1, 3))
            if o.any():
                d = cv2.distanceTransform((~o).astype(np.uint8), cv2.DIST_L2, 5)
                m = np.exp(-(d * d) / (2.0 * sigma_cells * sigma_cells)).astype(np.float32)
            else:
                m = np.zeros(o.shape, np.float32)
            out.append((m, self.res * step))
        self.occ_dilated = cv2.dilate(occ.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
        self._levels, self._levels_version = out, self.version
        return out

    def bounds(self):
        """Area with occupancy evidence in cell coords (u0, v0, u1, v1); None for an empty map."""
        return self._bounds


def _sample(M: np.ndarray, u: np.ndarray, v: np.ndarray):
    """Bilinear value and gradient (w.r.t. u/v). Out-of-bounds points yield 0 value and gradient.

    u/v are continuous coords with cell centres at integers: cell k spans [k, k+1) with centre k+0.5,
    so callers must subtract 0.5 (see _cell_coords). Coarse cell k is the max-pool of fine [k*s, (k+1)*s),
    so the same formula holds at every level. Missing this half cell biases the coarse level by 10 cm,
    which compounds as scans are written back (1.7 m measured in a synthetic test)."""
    n_v, n_u = M.shape
    u0 = np.floor(u).astype(np.int64)
    v0 = np.floor(v).astype(np.int64)
    ok = (u0 >= 0) & (v0 >= 0) & (u0 < n_u - 1) & (v0 < n_v - 1)
    u0c = np.where(ok, u0, 0)
    v0c = np.where(ok, v0, 0)
    du = (u - u0).astype(np.float32)
    dv = (v - v0).astype(np.float32)
    m00 = M[v0c, u0c]
    m10 = M[v0c, u0c + 1]
    m01 = M[v0c + 1, u0c]
    m11 = M[v0c + 1, u0c + 1]
    val = (1 - dv) * ((1 - du) * m00 + du * m10) + dv * ((1 - du) * m01 + du * m11)
    gu = (1 - dv) * (m10 - m00) + dv * (m11 - m01)
    gv = (1 - du) * (m01 - m00) + du * (m11 - m10)
    z = np.zeros_like(val)
    return np.where(ok, val, z), np.where(ok, gu, z), np.where(ok, gv, z)


def _cell_coords(wx, wy, origin, res):
    """World coords -> continuous cell coords for bilinear sampling (cell centres at integers, see _sample)."""
    return (wx - origin[0]) / res - 0.5, (wy - origin[1]) / res - 0.5


def match(levels, origin, xy: np.ndarray, pose, *, iters=(10, 6, 4), max_step=(0.25, 0.12)):
    """Coarse-to-fine Gauss-Newton scan-to-map matching. iters gives per-level iterations, coarse to fine. Returns (pose, mean field value at the finest level)."""
    x, y, th = pose
    px, py = xy[:, 0].astype(np.float32), xy[:, 1].astype(np.float32)
    score = 0.0
    for lvl in range(len(levels) - 1, -1, -1):
        M, res = levels[lvl]
        for _ in range(iters[min(len(levels) - 1 - lvl, len(iters) - 1)]):
            c, s = math.cos(th), math.sin(th)
            wx = c * px - s * py + x
            wy = s * px + c * py + y
            val, gu, gv = _sample(M, *_cell_coords(wx, wy, origin, res))
            gx, gy = gu / res, gv / res
            dth = gx * (-s * px - c * py) + gy * (c * px - s * py)
            J = np.stack([gx, gy, dth], 1).astype(np.float64)
            r = (1.0 - val).astype(np.float64)
            H = J.T @ J
            H[np.diag_indices(3)] += 1e-3 + 1e-2 * np.diag(H)
            try:
                d = np.linalg.solve(H, J.T @ r)
            except np.linalg.LinAlgError:
                break
            lim = max_step[0] * (res / levels[0][1]) ** 0.5
            n = math.hypot(d[0], d[1])
            if n > lim:
                d[:2] *= lim / n
            d[2] = max(-max_step[1], min(max_step[1], d[2]))
            x, y, th = x + d[0], y + d[1], wrap(th + d[2])
            if abs(d[0]) < 1e-4 and abs(d[1]) < 1e-4 and abs(d[2]) < 1e-4:
                break
        score = float(val.mean()) if len(px) else 0.0
    return (x, y, th), score


def hit_ratio(occ_dilated: np.ndarray, res, origin, xy: np.ndarray, pose) -> float:
    """Fraction of structure points on (1-cell dilated) occupied cells: an intuitive match-quality reading."""
    if not len(xy):
        return 0.0
    c, s = math.cos(pose[2]), math.sin(pose[2])
    u = np.floor((c * xy[:, 0] - s * xy[:, 1] + pose[0] - origin[0]) / res).astype(np.int64)
    v = np.floor((s * xy[:, 0] + c * xy[:, 1] + pose[1] - origin[1]) / res).astype(np.int64)
    ok = (u >= 0) & (v >= 0) & (u < occ_dilated.shape[1]) & (v < occ_dilated.shape[0])
    return float(occ_dilated[v[ok], u[ok]].sum()) / len(xy)


# --------------------------------------------------------------------------
# SLAM
# --------------------------------------------------------------------------
class LidarSlam:
    """Online 2D SLAM. Call process(xy_robot, t) per scan (de-skewed robot-frame structure points); snapshot() reads the pose.

    States: tracking (normal) / degraded (this scan failed to match, prediction used) / lost (many
    consecutive failures) / relocalizing (old map loaded, not yet localized on it)."""

    def __init__(self, *, res=0.05, size_m=80.0, min_quality=0.45, lost_after=10,
                 integrate_every_m=0.05, integrate_every_deg=2.0, integrate_idle_s=1.0,
                 keypose_every_m=0.5, warmup_scans=3, now_fn=time.monotonic, log=None):
        self.map = GridMap(res, size_m)
        self.map_id = uuid.uuid4().hex[:12]
        self.min_quality = float(min_quality)
        self.lost_after = int(lost_after)
        self.integrate_every_m = float(integrate_every_m)
        self.integrate_every_rad = math.radians(integrate_every_deg)
        self.integrate_idle_s = float(integrate_idle_s)
        self.keypose_every_m = float(keypose_every_m)
        self.warmup_scans = int(warmup_scans)
        self.now_fn = now_fn
        self._log = log or (lambda *a, **k: None)
        self._lock = threading.Lock()
        self.pose = (0.0, 0.0, 0.0)
        self.vel = (0.0, 0.0, 0.0)          # robot frame (vx, vy, wz)
        self.state = "initializing"
        self.quality = None
        self.scans = 0
        self.fails = 0
        self.keyposes: list[tuple[float, float, float]] = []
        self._t_last = None
        self._pose_t = None
        self._last_integrated = None
        self._last_integrated_t = None
        self._levels_t = None
        self._wall_t = None                 # wall time of the last processed scan; localization becomes unavailable if data stops
        self.dirty_since_save = False

    def process(self, xy: np.ndarray, t: float) -> str:
        """Feed one scan (robot-frame 2D structure points, reference time t in s). Returns the resulting state."""
        with self._lock:
            return self._process(xy, t)

    def _process(self, xy, t):
        self.scans += 1
        self._wall_t = self.now_fn()
        if len(xy) < 30:
            return self._fail("too_few_points")
        if self.state == "relocalizing":
            return self.state
        dt = 0.0 if self._t_last is None else max(0.0, min(0.5, t - self._t_last))
        predicted = compose(self.pose, (self.vel[0] * dt, self.vel[1] * dt, self.vel[2] * dt))
        if self.state == "initializing" or self.map.bounds() is None:
            self._accept(predicted, xy, t, dt, quality=1.0, force_integrate=True)
            return self.state
        warmup = self.scans <= self.warmup_scans                    # integrate the first few scans to firm up the walls
        levels = self._levels(t)
        est, _ = match(levels, self.map.levels_origin, xy, predicted)
        quality = hit_ratio(self._occ_dilated(), self.map.res, self.map.levels_origin, xy, est)
        jump = relative(predicted, est)
        if quality < self.min_quality or math.hypot(jump[0], jump[1]) > 0.5 or abs(jump[2]) > math.radians(25):
            self.quality = quality
            self.pose = predicted
            self._t_last = t
            return self._fail("low_quality")
        self._accept(est, xy, t, dt, quality, force_integrate=warmup)
        return self.state

    def _levels(self, t, max_stale_s: float = 0.25):
        """Matching pyramid, rebuilt at most every max_stale_s even if the map changed (blur/dilate ~15 ms; the map is written several times per second while moving)."""
        cached = self.map._levels
        if cached is not None and self.map.stale and self._levels_t is not None and t - self._levels_t < max_stale_s:
            return cached
        self._levels_t = t
        return self.map.levels()

    def _occ_dilated(self):
        if self.map.occ_dilated is None:
            self.map.levels()
        return self.map.occ_dilated

    def _accept(self, pose, xy, t, dt, quality, force_integrate=False):
        if dt > 0:
            d = relative(self.pose, pose)
            a = 0.5                                                     # velocity low-pass
            self.vel = tuple(a * (d[i] / dt) + (1 - a) * self.vel[i] for i in range(3))
        self.pose, self.quality, self.fails = pose, quality, 0
        self._t_last = self._pose_t = t
        self.state = "tracking"
        last = self._last_integrated
        moved = last is None or math.hypot(pose[0] - last[0], pose[1] - last[1]) >= self.integrate_every_m \
            or abs(wrap(pose[2] - last[2])) >= self.integrate_every_rad
        idle = self._last_integrated_t is None or t - self._last_integrated_t >= self.integrate_idle_s
        if force_integrate or moved or idle:
            self.map.integrate(pose, xy)
            self._last_integrated, self._last_integrated_t = pose, t
            self.dirty_since_save = True
        if not self.keyposes or math.hypot(pose[0] - self.keyposes[-1][0], pose[1] - self.keyposes[-1][1]) >= self.keypose_every_m:
            self.keyposes.append(tuple(round(v, 3) for v in pose))

    def _fail(self, reason):
        self.fails += 1
        self.vel = (0.0, 0.0, 0.0) if self.fails > 2 else self.vel     # stop extrapolating stale velocity after repeated failures
        if self.fails >= self.lost_after:
            if self.state != "lost":
                self._log(f"雷达定位丢失（连续 {self.fails} 帧匹配失败：{reason}）", "slam")
            self.state = "lost"
        elif self.state == "tracking":
            self.state = "degraded"
        return self.state

    def snapshot(self, max_age_s: float = 1.0) -> dict:
        """ok=True only when tracking with fresh data; degraded (occasional mismatch, prediction used) still reports a pose but ok=False."""
        with self._lock:
            now = self.now_fn()
            pose, state, quality, wall_t = self.pose, self.state, self.quality, self._wall_t
        errors = []
        if state in ("initializing", "relocalizing", "lost", "degraded"):
            errors.append("slam_" + state)
        age = None if wall_t is None else round(max(0.0, now - wall_t), 2)
        if age is None or age > max_age_s:
            errors.append("slam_stale")
        out = {"ok": not errors, "errors": errors, "state": state, "age_s": age,
               "pose": None if state in ("initializing", "relocalizing") else
               {"x": round(pose[0], 3), "y": round(pose[1], 3), "yaw": round(pose[2], 4), "frame": FRAME},
               "quality": None if quality is None else round(quality, 2),
               "map_id": self.map_id, "scans": self.scans}
        return out

    def save(self, path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with self._lock:
            data = dict(logodds=self.map.L.astype(np.float16), res=self.map.res, origin=np.array(self.map.origin),
                        map_id=self.map_id, keyposes=np.array(self.keyposes or [(0.0, 0.0, 0.0)], np.float32),
                        last_pose=np.array(self.pose, np.float64), saved_at=time.time())
            self.dirty_since_save = False
        tmp = path.with_name(path.name + ".tmp.npz")
        np.savez_compressed(tmp, **data)
        tmp.replace(path)

    @classmethod
    def load(cls, path, **kw) -> LidarSlam:
        d = np.load(path)
        slam = cls(res=float(d["res"]), **kw)
        slam.map = GridMap(float(d["res"]), logodds=d["logodds"], origin=tuple(d["origin"]))
        slam.map_id = str(d["map_id"])
        slam.keyposes = [tuple(map(float, k)) for k in d["keyposes"]]
        slam.saved_last_pose = tuple(map(float, d["last_pose"]))
        slam.state = "relocalizing"
        return slam

    def relocalize(self, xy: np.ndarray, t: float, *, seeds=(), rot_step_deg=5.0, top_k=8,
                   min_quality=0.6, min_margin=0.08) -> dict:
        """Locate the current scan on the loaded map. Seeds = trajectory keyposes + last pose + extra seeds (calibrated points).
        On success switches to tracking and returns {"ok": True, ...}; if uncertain stays relocalizing."""
        with self._lock:
            if len(xy) < 50:
                return {"ok": False, "reason": "too_few_points"}
            self._wall_t = self.now_fn()
            levels = self.map.levels()
            cand = list(self.keyposes) + list(seeds)
            if hasattr(self, "saved_last_pose"):
                cand.insert(0, self.saved_last_pose)
            pos = []
            for c in cand:                                       # deduplicate positions (one per 0.3 m)
                if all(math.hypot(c[0] - p[0], c[1] - p[1]) > 0.3 for p in pos):
                    pos.append(c)
            if not pos:
                return {"ok": False, "reason": "no_seeds"}
            sub = xy[np.random.default_rng(0).choice(len(xy), min(len(xy), 300), replace=False)]
            M, res = levels[-1]
            ox, oy = self.map.levels_origin
            ths = np.radians(np.arange(0.0, 360.0, rot_step_deg))
            P = np.asarray([(p[0], p[1]) for p in pos], np.float32)
            scores = np.zeros((len(P), len(ths)), np.float32)
            for j, th in enumerate(ths):
                c, s = math.cos(th), math.sin(th)
                rx = c * sub[:, 0] - s * sub[:, 1]
                ry = s * sub[:, 0] + c * sub[:, 1]
                u, v = _cell_coords(rx[None, :] + P[:, :1], ry[None, :] + P[:, 1:], (ox, oy), res)
                val, _, _ = _sample(M, u.ravel(), v.ravel())
                scores[:, j] = val.reshape(len(P), -1).mean(1)
            flat = np.argsort(scores.ravel())[::-1][:top_k]
            occ = self._occ_dilated()
            results = []
            for f in flat:
                i, j = divmod(int(f), len(ths))
                est, _ = match(levels, self.map.levels_origin, xy, (float(P[i, 0]), float(P[i, 1]), float(ths[j])))
                results.append((hit_ratio(occ, self.map.res, self.map.levels_origin, xy, est), est))
            results.sort(key=lambda r: -r[0])
            best_q, best = results[0]
            rival = next((q for q, e in results[1:] if math.hypot(e[0] - best[0], e[1] - best[1]) > 0.5
                          or abs(wrap(e[2] - best[2])) > math.radians(15)), 0.0)
            info = {"quality": round(best_q, 2), "rival": round(rival, 2), "seeds": len(pos),
                    "pose": {"x": round(best[0], 3), "y": round(best[1], 3), "yaw": round(best[2], 4)}}
            if best_q < min_quality or best_q - rival < min_margin:
                return {"ok": False, "reason": "ambiguous" if best_q >= min_quality else "low_quality", **info}
            self.pose, self.vel, self.quality, self.fails = best, (0.0, 0.0, 0.0), best_q, 0
            self._t_last = self._pose_t = t
            self._wall_t = self.now_fn()
            self._last_integrated, self._last_integrated_t = best, t
            self.state = "tracking"
        self._log(f"雷达重定位成功：quality {best_q:.2f}（次优 {rival:.2f}）", "slam")
        return {"ok": True, **info}

    def render(self, scale: int = 2, crop: bool = True) -> np.ndarray:
        """BGR image for the map PNG: white=free, black=occupied, grey=unknown; red=current pose, blue=keyposes. Up = +y."""
        L = self.map.L
        img = np.full(L.shape, 128, np.uint8)
        img[L < -0.5] = 255
        img[L > 0.5] = 0
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        for k in self.keyposes:
            u, v = self.map.cells(np.array([[k[0], k[1]]], np.float32))[0]
            cv2.circle(img, (int(u), int(v)), 2, (255, 120, 0), -1)
        u, v = self.map.cells(np.array([[self.pose[0], self.pose[1]]], np.float32))[0]
        cv2.circle(img, (int(u), int(v)), 4, (0, 0, 255), -1)
        tip = self.map.cells(np.array([[self.pose[0] + 0.5 * math.cos(self.pose[2]),
                                        self.pose[1] + 0.5 * math.sin(self.pose[2])]], np.float32))[0]
        cv2.line(img, (int(u), int(v)), (int(tip[0]), int(tip[1])), (0, 0, 255), 2)
        if crop:
            b = self.map.bounds()
            if b is not None:
                m = 20
                u0, v0, u1, v1 = max(0, b[0] - m), max(0, b[1] - m), min(L.shape[1], b[2] + m), min(L.shape[0], b[3] + m)
                img = img[v0:v1, u0:u1]
        img = cv2.flip(img, 0)                                           # row index grows with y; flip so up = +y
        return cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)


# --------------------------------------------------------------------------
# Runner: Livox packet batches -> SLAM
# --------------------------------------------------------------------------
class SlamRunner:
    """Feeds LivoxFeeder's 0.1 s packet batches into LidarSlam: parse -> extrinsics -> de-skew -> structure
    points -> process; relocalizes automatically after loading a map or getting lost; saves the map
    periodically. Runs in its own thread and merges batches when falling behind.

    submit() runs on the receiver thread and only enqueues: slow SLAM must never stall collision avoidance."""

    def __init__(self, slam: LidarSlam, mount, *, map_path=None, autosave_s: float = 60.0,
                 relocalize_every_s: float = 3.0, seeds_fn=None, struct_kw=None, log=None):
        self.slam, self.mount = slam, mount
        self.map_path = None if map_path is None else Path(map_path)
        self.autosave_s = float(autosave_s)
        self.relocalize_every_s = float(relocalize_every_s)
        self.seeds_fn = seeds_fn or (lambda: [])
        self.struct_kw = dict(struct_kw or {})
        self._log = log or (lambda *a, **k: None)
        self._pending: list = []
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._running = False
        self._thread = None
        self._last_save = time.monotonic()
        self._last_reloc = 0.0
        self.last_relocalize: dict | None = None
        self.error: str | None = None
        self.batches = self.merged = 0
        self.proc_ms = None

    def submit(self, packets) -> None:
        with self._lock:
            if self._pending:
                self.merged += 1
            self._pending.extend(packets)
            if len(self._pending) > 2000:                 # backlog > ~1 s: keep only the newest to bound growth
                self._pending = self._pending[-500:]
        self._event.set()

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="lidar-slam", daemon=True)
        self._thread.start()

    def stop(self, save: bool = True) -> None:
        self._running = False
        self._event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if save:
            self.save()

    def save(self) -> bool:
        """Save the map, only when localization is trusted; while relocalizing the old map is left intact rather than overwritten by an unaligned state."""
        if self.map_path is None or self.slam.state not in ("tracking", "degraded"):
            return False
        try:
            self.slam.save(self.map_path)
            self._last_save = time.monotonic()
            return True
        except OSError as exc:
            self.error = f"地图保存失败: {exc}"
            self._log(self.error, "slam")
            return False

    def stats(self) -> dict:
        return {"batches": self.batches, "merged": self.merged, "proc_ms": self.proc_ms,
                "map_path": None if self.map_path is None else str(self.map_path),
                "last_relocalize": self.last_relocalize, "error": self.error}

    def step(self, packets) -> str:
        """Process one packet batch (shared by the loop and tests)."""
        from services.livox_mid360 import parse_point_packets
        t0 = time.monotonic()
        xyz, t, _ = parse_point_packets(packets, with_time=True)
        if not len(xyz):
            return self.slam.state
        pts = self.mount.to_robot(xyz)
        t_ref = float(t.max())
        pts = deskew(pts, t, t_ref, self.slam.vel)
        xy = structure_points(pts, **self.struct_kw)
        if self.slam.state == "lost":                      # lost: switch to relocalizing on the current map
            self.slam.state = "relocalizing"
        if self.slam.state == "relocalizing":
            now = time.monotonic()
            if now - self._last_reloc >= self.relocalize_every_s:
                self._last_reloc = now
                self.last_relocalize = self.slam.relocalize(xy, t_ref, seeds=self.seeds_fn())
        else:
            self.slam.process(xy, t_ref)
        self.batches += 1
        self.proc_ms = round((time.monotonic() - t0) * 1000, 1)
        return self.slam.state

    def _loop(self):
        while self._running:
            self._event.wait(0.5)
            self._event.clear()
            with self._lock:
                packets, self._pending = self._pending, []
            if packets:
                try:
                    self.step(packets)
                    self.error = None
                except Exception as exc:                   # record only; localization will go stale
                    self.error = f"SLAM 处理失败: {type(exc).__name__}: {exc}"
            if self.slam.dirty_since_save and time.monotonic() - self._last_save >= self.autosave_s:
                self.save()
