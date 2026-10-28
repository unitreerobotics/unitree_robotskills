# -*- coding: utf-8 -*-
"""Livox Mid-360 point-cloud input: 3D cloud -> virtual 2D scan -> services/lidar.LidarGuard.

The R1D chassis carries a Mid-360 at the front (LAN 192.168.123.120). It does not use DDS but
the Livox SDK2 UDP protocol:
  56100/56101  commands (host -> lidar 56100; replies return to the sending port)
  56200/56201  status push (lidar broadcast, ~1 Hz)
  56300/56301  point cloud (lidar -> configured point host :56301, ~2084 pkt/s x 96 pts = 200k pts/s)
Points go only to the single host address configured in the lidar. The factory setting may
point elsewhere (no such machine on the LAN), so nobody receives them; on start this module
rewrites it to this host, as the Livox driver does (ensure_point_host, persisted in the lidar;
revert with tools/livox_diag.py --set-host <ip>).

Mounting (measured on the robot, cross-checked against five planes: floor, two ceiling
levels, two walls; refined on 6 s of points):
  Lidar at the chassis front, 0.205 m above ground, pitched 30.4 deg down-forward (roll -0.55 deg);
  the body lies along its -x axis, +x is straight ahead.
  The lidar is recessed in the nose: in front of it a downward-sloping deflector runs from the
  lower edge of the window to near the ground 0.25-0.28 m ahead, +/-0.30 m wide (the lowest
  beams put a dozen points on it every revolution; unfiltered, 1/4 of scans would report an
  obstacle 0.1 m ahead). Clean ground starts 0.30 m ahead of the lidar.
  The robot-frame origin is on the ground below the front edge of the nose (0.27 m ahead of the
  lidar), so clearance = distance ahead of the bumper.
  Forward it sees ground beyond 0.3 m and objects below 0.4 m nearby (0.6 m at 1 m, 1.0 m at 2 m);
  backward it only sees ceiling, occluded by the body: no rear data.

Virtual scan: points are transformed to the robot frame (X forward, Y left, Z height above
ground); per bearing the nearest point within the obstacle height band is taken, yielding a
LaserScan-shaped (angle_min, angle_inc, ranges) fed to LidarGuard.feed(), so the G1D front
sector / corridor / stop_m / slow_m / status logic is reused unchanged.

Limitations:
  - overhangs above the upper FOV edge nearby (a 0.4 m+ tabletop at 0.5 m) are invisible; only legs are seen;
  - objects below z_min (default 5 cm), e.g. cables, thresholds, count as ground;
  - glass, mirrors and black absorbent surfaces may give no returns; no rear data.

Pure functions (protocol codec, packet parsing, transforms, virtual scan, floor fit) need no
network and are offline-testable.
"""
from __future__ import annotations

import gzip
import math
import socket
import struct
import threading
import time
import zlib
from collections import deque

import numpy as np

LIDAR_CMD_PORT = 56100
HOST_CMD_PORT = 56101
HOST_PUSH_PORT = 56201
HOST_POINT_PORT = 56301
HOST_IMU_PORT = 56401        # IMU (lidar -> configured IMU host :56401, ~200 Hz)

CMD_SET, CMD_GET, CMD_PUSH = 0x0100, 0x0101, 0x0102
KEY_POINT_HOST, KEY_IMU_HOST, KEY_WORK_STATE = 0x0006, 0x0007, 0x8006
WORK_SAMPLING = 1

FRAME_HEADER = 24            # sof ver len seq cmd_id cmd_type sender rsvd[6] crc16 crc32
POINT_HEADER = 36            # version len interval dot_num udp_cnt frame_cnt data_type time_type rsvd[12] crc32 ts[8]
DATA_TYPE_CARTESIAN_HIGH = 1
DATA_TYPE_IMU = 0
_IMU = struct.Struct("<6f")  # gyro xyz (rad/s) + acc xyz (g), lidar frame
_POINT = np.dtype([("x", "<i4"), ("y", "<i4"), ("z", "<i4"), ("refl", "u1"), ("tag", "u1")])    # mm, 14 bytes


# --------------------------------------------------------------------------
# Protocol (Livox SDK2 frames)
# --------------------------------------------------------------------------
def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE over the first 18 header bytes (verified against real status packets)."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(cmd_id: int, data: bytes, seq: int = 1) -> bytes:
    """Host request frame (cmd_type=0, sender=0)."""
    head = struct.pack("<BBHIHBB6x", 0xAA, 0, FRAME_HEADER + len(data), seq, cmd_id, 0, 0)
    return head + struct.pack("<HI", crc16_ccitt(head), zlib.crc32(data) & 0xFFFFFFFF) + data


def parse_frame(frame: bytes) -> dict:
    """Split and verify a frame; raises ValueError on bad format or checksum."""
    if len(frame) < FRAME_HEADER or frame[0] != 0xAA:
        raise ValueError("不是 Livox SDK2 帧")
    _, _, length, seq, cmd_id, cmd_type, sender = struct.unpack_from("<BBHIHBB", frame, 0)
    crc16, crc32 = struct.unpack_from("<HI", frame, 18)
    if length != len(frame) or crc16 != crc16_ccitt(frame[:18]):
        raise ValueError("Livox 帧头校验失败")
    data = frame[FRAME_HEADER:]
    if crc32 != zlib.crc32(data) & 0xFFFFFFFF:
        raise ValueError("Livox 数据校验失败")
    return {"cmd_id": cmd_id, "cmd_type": cmd_type, "sender": sender, "seq": seq, "data": data}


def encode_get(keys) -> bytes:
    keys = list(keys)
    return struct.pack("<HH", len(keys), 0) + b"".join(struct.pack("<H", k) for k in keys)


def encode_set(items: dict) -> bytes:
    return struct.pack("<HH", len(items), 0) + b"".join(
        struct.pack("<HH", k, len(v)) + bytes(v) for k, v in items.items())


def decode_key_values(body: bytes, offset: int, count: int) -> dict:
    out = {}
    for _ in range(count):
        if offset + 4 > len(body):
            break
        key, length = struct.unpack_from("<HH", body, offset)
        out[key] = body[offset + 4:offset + 4 + length]
        offset += 4 + length
    return out


def decode_get_ack(body: bytes) -> tuple[int, dict]:
    ret, count = struct.unpack_from("<BH", body, 0)
    return ret, decode_key_values(body, 3, count)


def decode_host_cfg(value: bytes) -> dict:
    """Host IP config value: ip[4] + host_port(u16) + lidar_port(u16)."""
    host_port, lidar_port = struct.unpack_from("<HH", value, 4)
    return {"ip": ".".join(str(b) for b in value[:4]), "host_port": host_port, "lidar_port": lidar_port}


def with_host_ip(value: bytes, ip: str) -> bytes:
    """Replace only the 4 IP bytes; ports and other fields keep the lidar's values."""
    return bytes(int(x) for x in ip.split(".")) + bytes(value[4:])


def local_ip_towards(lidar_ip: str) -> str:
    """Local address on the NIC facing the lidar (sends nothing: UDP connect only resolves the route)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((lidar_ip, LIDAR_CMD_PORT))
        return s.getsockname()[0]
    finally:
        s.close()


class LivoxCommander:
    """Command channel: request/response over UDP with timeout. sock_factory is injectable for tests."""

    def __init__(self, lidar_ip: str, host_ip: str = "", *, timeout_s: float = 1.5, sock_factory=None):
        self.lidar_ip, self.host_ip, self.timeout_s = lidar_ip, host_ip, float(timeout_s)
        self._sock_factory = sock_factory or self._open
        self._seq = 0

    def _open(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host_ip or "0.0.0.0", HOST_CMD_PORT))
        return s

    def call(self, cmd_id: int, data: bytes) -> bytes:
        self._seq += 1
        s = self._sock_factory()
        try:
            s.settimeout(self.timeout_s)
            s.sendto(build_frame(cmd_id, data, self._seq), (self.lidar_ip, LIDAR_CMD_PORT))
            deadline = time.monotonic() + self.timeout_s
            while time.monotonic() < deadline:
                raw, _ = s.recvfrom(4096)
                try:
                    f = parse_frame(raw)
                except ValueError:
                    continue
                if f["cmd_id"] == cmd_id and f["cmd_type"] == 1:
                    return f["data"]
            raise TimeoutError(f"Livox 命令 0x{cmd_id:04X} 无应答")
        finally:
            s.close()

    def get(self, keys) -> dict:
        ret, kv = decode_get_ack(self.call(CMD_GET, encode_get(keys)))
        if ret != 0:
            raise RuntimeError(f"Livox 查询失败 ret_code={ret}")
        return kv

    def set(self, items: dict) -> None:
        ret, err_key = struct.unpack_from("<BH", self.call(CMD_SET, encode_set(items)), 0)
        if ret != 0:
            raise RuntimeError(f"Livox 参数写入失败 ret_code={ret} key=0x{err_key:04X}")


def ensure_point_host(lidar_ip: str, host_ip: str = "", *, commander=None, log=None) -> dict:
    """Point the lidar's point-cloud and IMU output at this host: read the config, rewrite it if it
    differs (IP only, ports untouched), then read back to confirm.

    The Livox driver does this on every start; the write persists in the lidar. IMU feeds the
    chassis heading loop (core/heading_hold.py); if the lidar doesn't return the IMU key only the
    point host is handled. Returns {"ok", "changed", "host", "previous", "imu_previous",
    "work_state", "error"}; never raises."""
    log = log or (lambda *a, **k: None)
    out = {"ok": False, "changed": False, "host": None, "previous": None, "imu_previous": None,
           "work_state": None, "error": None}
    try:
        host_ip = host_ip or local_ip_towards(lidar_ip)
        cmd = commander or LivoxCommander(lidar_ip, host_ip)
        kv = cmd.get([KEY_POINT_HOST, KEY_IMU_HOST, KEY_WORK_STATE])
        cur = decode_host_cfg(kv[KEY_POINT_HOST])
        out.update(host=host_ip, previous=cur["ip"], work_state=kv[KEY_WORK_STATE][0])
        fix = {}
        if cur["ip"] != host_ip:
            fix[KEY_POINT_HOST] = with_host_ip(kv[KEY_POINT_HOST], host_ip)
        if KEY_IMU_HOST in kv:
            imu = decode_host_cfg(kv[KEY_IMU_HOST])
            out["imu_previous"] = imu["ip"]
            if imu["ip"] != host_ip:
                fix[KEY_IMU_HOST] = with_host_ip(kv[KEY_IMU_HOST], host_ip)
        if fix:
            names = "、".join(n for k, n in ((KEY_POINT_HOST, "点云"), (KEY_IMU_HOST, "IMU")) if k in fix)
            log(f"Livox {names}目的地址 {cur['ip']} / {out['imu_previous']} → {host_ip}（点云原值可用 "
                f"tools/livox_diag.py --set-host {cur['ip']} 改回）", "lidar")
            cmd.set(fix)
            now = cmd.get(list(fix))
            for key in fix:
                if decode_host_cfg(now[key])["ip"] != host_ip:
                    raise RuntimeError(f"写入后读回仍是 {decode_host_cfg(now[key])['ip']}")
            out["changed"] = True
        out["ok"] = True
    except Exception as exc:
        out["error"] = f"{type(exc).__name__}: {exc}"
    return out


# --------------------------------------------------------------------------
# Point-cloud packets
# --------------------------------------------------------------------------
def parse_point_packets(packets, with_time: bool = False):
    """Batch of point UDP packets -> (xyz float32 [N,3] in m, lidar frame; lost packet count).

    Only high-precision Cartesian packets (data_type=1) are used; all-zero points (no return) and
    tag!=0 (lidar-flagged noise / low confidence) are dropped. With with_time=True also returns
    per-point time (s, float64, from the packet header timestamp; a 96-point packet spans only
    ~0.5 ms) as (xyz, t, lost), used for SLAM motion de-skewing."""
    chunks, counts, stamps, sizes = [], [], [], []
    for d in packets:
        if len(d) < POINT_HEADER or d[10] != DATA_TYPE_CARTESIAN_HIGH:
            continue
        n = struct.unpack_from("<H", d, 5)[0]
        body = d[POINT_HEADER:POINT_HEADER + _POINT.itemsize * n]
        if len(body) == _POINT.itemsize * n:
            chunks.append(body)
            counts.append(struct.unpack_from("<H", d, 7)[0])
            stamps.append(struct.unpack_from("<Q", d, 28)[0])
            sizes.append(n)
    if not chunks:
        empty = np.zeros((0, 3), np.float32)
        return (empty, np.zeros(0), 0) if with_time else (empty, 0)
    a = np.frombuffer(b"".join(chunks), dtype=_POINT)
    keep = (a["tag"] == 0) & ((a["x"] != 0) | (a["y"] != 0) | (a["z"] != 0))
    xyz = np.stack([a["x"][keep], a["y"][keep], a["z"][keep]], axis=1).astype(np.float32) / 1000.0
    c = np.asarray(counts, dtype=np.int64)
    lost = int(((np.diff(c) - 1) % 65536).sum()) if len(c) > 1 else 0
    if not with_time:
        return xyz, lost
    t = np.repeat(np.asarray(stamps, dtype=np.float64) * 1e-9, sizes)[keep]
    return xyz, t, lost


def parse_imu_packet(d: bytes):
    """IMU packet -> (gyro xyz rad/s, acc xyz g) in the lidar frame (same axes as points); None if not IMU.
    Verified on the robot: at rest acc ~= (-0.50, 0.00, 0.86) g = -lidar_down, matching the floor fit."""
    if len(d) < POINT_HEADER + _IMU.size or d[10] != DATA_TYPE_IMU:
        return None
    v = _IMU.unpack_from(d, POINT_HEADER)
    return v[:3], v[3:]


# --------------------------------------------------------------------------
# Mounting extrinsics
# --------------------------------------------------------------------------
class Mount:
    """Lidar frame -> robot frame (X forward, Y left, Z = height above ground; origin on the ground
    below the front edge of the nose).

    down: unit vector toward the ground in the lidar frame (from the floor fit); height_m: lidar
    height above ground; yaw_deg: yaw of the lidar x-axis' horizontal projection relative to robot
    forward (+ = left); front_offset_m: how far the nose front edge is ahead of the (recessed) lidar."""

    def __init__(self, down=(0.505, -0.008, -0.863), height_m: float = 0.205, yaw_deg: float = 0.0,
                 front_offset_m: float = 0.27):
        d = np.asarray(down, dtype=np.float64)
        norm = np.linalg.norm(d)
        if not norm > 0:
            raise ValueError("lidar_down 不能是零向量")
        up = -d / norm
        fwd = np.array([1.0, 0.0, 0.0]) - up[0] * up            # lidar x-axis projected onto the horizontal plane
        if np.linalg.norm(fwd) < 1e-6:
            raise ValueError("雷达 x 轴与竖直方向重合，无法确定正前方")
        fwd /= np.linalg.norm(fwd)
        left = np.cross(up, fwd)
        yaw = math.radians(float(yaw_deg))
        x_axis = math.cos(yaw) * fwd - math.sin(yaw) * left     # robot forward = lidar forward rotated back by yaw
        y_axis = math.sin(yaw) * fwd + math.cos(yaw) * left
        self.rot = np.stack([x_axis, y_axis, up]).astype(np.float32)   # rows = robot-frame axes expressed in the lidar frame
        self.height_m = float(height_m)
        self.front_offset_m = float(front_offset_m)
        self.pitch_deg = math.degrees(math.asin(max(-1.0, min(1.0, float(d[0] / norm)))))

    def to_robot(self, xyz: np.ndarray) -> np.ndarray:
        out = xyz @ self.rot.T
        out[:, 0] -= self.front_offset_m
        out[:, 2] += self.height_m
        return out


def fit_floor(xyz: np.ndarray, *, min_range_m=0.5, tol_m=0.03, max_height_m=1.0, iters=400, seed=0):
    """Calibration: RANSAC the floor as the dominant plane near the lidar; returns (down unit vector,
    height m, inlier count) or None.

    Ceilings and walls are large planes too, so the floor is the plane with distance < max_height_m
    and the most inliers."""
    r = np.linalg.norm(xyz, axis=1)
    p = xyz[r > min_range_m].astype(np.float64)
    if len(p) < 500:
        return None
    rng = np.random.default_rng(seed)
    sub = p[rng.choice(len(p), min(len(p), 20000), replace=False)]
    best = (0, None)
    for _ in range(iters):
        a, b, c = sub[rng.choice(len(sub), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        dist = abs(float(a @ n))
        if dist > max_height_m:
            continue
        count = int((np.abs((sub - a) @ n) < tol_m).sum())
        if count > best[0]:
            best = (count, (n, a))
    if best[1] is None:
        return None
    n, a = best[1]
    inliers = p[np.abs((p - a) @ n) < tol_m]
    centre = inliers.mean(axis=0)
    n = np.linalg.svd(inliers - centre, full_matrices=False)[2][2]
    height = float(centre @ n)
    if height < 0:
        n, height = -n, -height                                  # orient from lidar toward the ground
    return n, height, len(inliers)


# --------------------------------------------------------------------------
# Virtual scan
# --------------------------------------------------------------------------
def virtual_scan(points: np.ndarray, *, bin_deg: float = 2.0, z_min_m: float = 0.05, z_max_m: float = 1.5,
                 z_slope: float = 0.015, self_box=(-0.90, 0.03, 0.40), max_range_m: float = 10.0,
                 min_points: int = 2,
                 near_m: float = 0.25, near_min_points: int = 5, near_min_span_m: float = 0.10,
                 near_neighbor_bins: int = 2) -> tuple[float, float, list]:
    """Robot-frame points -> one LaserScan-shaped ring of horizontal ranges.

    Per bearing (0 deg = forward, + = left):
      >= min_points obstacle points (height z_min..z_max) -> range of the min_points-th nearest
        (a single outlier doesn't count);
      no obstacle points but some returns (ground / far) -> max_range_m (seen, empty);
      no returns at all (rear blind spot) -> nan.
    z_slope widens the ground band with range (extrinsic error / uneven floor grow with
    distance): obstacles must be above z_min + z_slope*r.
    self_box=(x_min, x_max, y_half): the robot's own body box (origin at the nose front edge, so
    the whole robot is at x<=0; x_max has a 3 cm margin); points inside are dropped.

    Near-range plausibility gate (within near_m): close to large panels (cabinets / islands /
    charging dock) multipath produces sparse, low, isolated phantom points (the robot repeatedly
    reported 0.03-0.1 m in a clear corridor). A reading < near_m is accepted only if it passes
    2 of 3 checks:
      1) obstacle points in the bin >= near_min_points (a real panel is dense, artefacts are 2-3 points);
      2) height span >= near_min_span_m (a real door is a vertical strip, artefacts a small cluster);
      3) another near bin within +/-near_neighbor_bins (real walls are contiguous, artefacts isolated).
    Failing bins become max_range_m if they have returns outside the obstacle band (e.g. ground),
    otherwise nan (no reliable data): neither treated as obstacle nor claimed as clear."""
    n_bins = int(round(360.0 / bin_deg))
    inc = 2.0 * math.pi / n_bins
    ranges = np.full(n_bins, np.nan, dtype=np.float64)
    if len(points):
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        x0, x1, yh = self_box
        keep = ~((x >= x0) & (x <= x1) & (np.abs(y) <= yh))
        x, y, z = x[keep], y[keep], z[keep]
        r = np.hypot(x, y)
        idx = np.floor((np.arctan2(y, x) + math.pi) / inc).astype(np.int64) % n_bins
        ranges[np.unique(idx)] = max_range_m
        obst = (z >= z_min_m + z_slope * r) & (z <= z_max_m) & (r <= max_range_m)
        if obst.any():
            ob_idx, ob_r, ob_z = idx[obst], r[obst], z[obst]
            order = np.lexsort((ob_r, ob_idx))
            ob_idx, ob_r, ob_z = ob_idx[order], ob_r[order], ob_z[order]
            start = np.flatnonzero(np.r_[True, ob_idx[1:] != ob_idx[:-1]])
            count = np.diff(np.r_[start, len(ob_idx)])
            ok = count >= min_points
            hit_bins = ob_idx[start[ok]]
            ranges[hit_bins] = ob_r[start[ok] + min_points - 1]
            # Near-range gate: only bins < near_m; far bins unaffected.
            cand = hit_bins[ranges[hit_bins] < near_m]
            if len(cand):
                cnt = dict(zip(hit_bins, count[ok]))
                span = {}
                for b, s, c in zip(hit_bins, start[ok], count[ok]):
                    seg = ob_z[s:s + c]
                    span[int(b)] = float(seg.max() - seg.min())
                cand_set = set(int(b) for b in cand)
                n_all = np.bincount(idx, minlength=n_bins)          # all returns per bin (incl. outside the obstacle band)
                for b in cand_set:
                    lo, hi = max(0, b - near_neighbor_bins), min(n_bins, b + near_neighbor_bins + 1)
                    neighbor = any(o in cand_set for o in range(lo, hi) if o != b)
                    checks = (cnt.get(b, 0) >= near_min_points,
                              span.get(b, 0.0) >= near_min_span_m,
                              neighbor)
                    if sum(checks) < 2:
                        ranges[b] = max_range_m if n_all[b] > cnt.get(b, 0) else np.nan
    return -math.pi + inc / 2.0, inc, ranges.tolist()          # bin-centre bearings (beam i of the LaserScan)


# --------------------------------------------------------------------------
# Receiver thread
# --------------------------------------------------------------------------
class LivoxFeeder:
    """Background point receiver: every period_s turns the last window_s of points into a virtual
    scan for sink(angle_min, angle_inc, ranges).

    The Mid-360 scans non-repetitively, so a single pass is sparse: accumulating window_s
    (default 0.2 s ~= 40k points) gives thin nearby objects enough points. While no points arrive
    the point host is re-checked every host_retry_s (self-heals after lidar reboot or reconfiguration)."""

    def __init__(self, sink, mount: Mount, *, lidar_ip: str = "192.168.123.120", host_ip: str = "",
                 port: int = HOST_POINT_PORT, window_s: float = 0.2, period_s: float = 0.1,
                 auto_host: bool = True, host_retry_s: float = 5.0, scan_kw: dict | None = None,
                 log=None, sock_factory=None, ensure_host=ensure_point_host,
                 batch_sink=None, record_path=None, record_max_bytes: int = 2 << 30):
        """batch_sink(packets): every period_s receives the packets new since the last call
        (non-overlapping), for SLAM; must not block.
        record_path: append raw packets with receive time to a gzip file (offline replay / tuning,
        see tools/lidar_slam_replay.py); recording stops after record_max_bytes (uncompressed)."""
        self.sink, self.mount = sink, mount
        self.batch_sink = batch_sink
        self._record = None
        self._record_left = int(record_max_bytes)
        self.record_path = record_path
        self.lidar_ip, self.host_ip, self.port = lidar_ip, host_ip, int(port)
        self.window_s, self.period_s = float(window_s), float(period_s)
        self.auto_host, self.host_retry_s = bool(auto_host), float(host_retry_s)
        self.scan_kw = dict(scan_kw or {})
        self._log = log or (lambda *a, **k: None)
        self._sock_factory = sock_factory or self._open
        self._ensure_host = ensure_host
        self._sock = None
        self._thread = None
        self._running = False
        self.error: str | None = None
        self.host_cfg: dict | None = None
        self.n_packets = self.n_lost = self.n_scans = self.n_points = 0
        self._last_packet_t: float | None = None

    def _open(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        s.bind(("0.0.0.0", self.port))
        return s

    def start(self) -> bool:
        """Bind the port and start the thread. Returns False if binding fails (reason in error); never raises."""
        if self._running:
            return True
        try:
            self._sock = self._sock_factory()
            self._sock.settimeout(0.05)
        except OSError as exc:
            self.error = f"Livox 点云端口 {self.port} 绑定失败: {exc}"
            self._log(self.error, "lidar")
            return False
        if self.record_path:
            try:
                self._record = gzip.open(self.record_path, "ab", compresslevel=1)
                self._log(f"Livox 点云录制到 {self.record_path}", "lidar")
            except OSError as exc:
                self._log(f"Livox 点云录制打不开 {self.record_path}: {exc}", "lidar")
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="livox-feeder", daemon=True)
        self._thread.start()
        self._log(f"Livox Mid-360 点云接收已启动（UDP {self.port}，雷达 {self.lidar_ip}，"
                  f"俯角 {self.mount.pitch_deg:.1f}°，离地 {self.mount.height_m:.2f}m）", "lidar")
        return True

    def stop(self):
        self._running = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        if self._record is not None:
            self._record.close()
            self._record = None

    def stats(self) -> dict:
        age = None if self._last_packet_t is None else round(time.monotonic() - self._last_packet_t, 2)
        return {"kind": "livox_mid360", "lidar_ip": self.lidar_ip, "packets": self.n_packets,
                "lost": self.n_lost, "scans": self.n_scans, "points_per_scan": self.n_points,
                "packet_age_s": age, "host_cfg": self.host_cfg, "error": self.error}

    def emit(self, packets) -> int:
        """Packet batch -> one virtual scan -> sink. Returns points used (shared by the loop and tests)."""
        xyz, lost = parse_point_packets(packets)
        self.n_lost += lost
        if not len(xyz):
            return 0
        self.sink(*virtual_scan(self.mount.to_robot(xyz), **self.scan_kw))
        self.n_scans += 1
        self.n_points = len(xyz)
        return len(xyz)

    def _check_host(self):
        self.host_cfg = self._ensure_host(self.lidar_ip, self.host_ip, log=self._log)
        if not self.host_cfg["ok"]:
            self.error = f"Livox 点云目的地址核对失败: {self.host_cfg['error']}"
        elif self.host_cfg.get("work_state") != WORK_SAMPLING:
            self.error = f"Livox 未在采样（work_state={self.host_cfg.get('work_state')}）"

    def _write_record(self, now: float, packet: bytes):
        if self._record_left <= 0:
            return
        try:
            self._record.write(struct.pack("<dH", now, len(packet)) + packet)
            self._record_left -= len(packet) + 10
            if self._record_left <= 0:
                self._log("Livox 点云录制到达上限，停录", "lidar")
        except (OSError, ValueError) as exc:
            self._log(f"Livox 点云录制失败，停录: {exc}", "lidar")
            self._record_left = 0

    def _loop(self):
        window = deque()                          # (receive time, packet)
        fresh = []                                # packets since the last emit (for batch_sink)
        next_emit = time.monotonic() + self.period_s
        next_host_check = 0.0
        while self._running:
            now = time.monotonic()
            try:
                packet = self._sock.recv(2048)
                window.append((now, packet))
                fresh.append(packet)
                if self._record is not None:
                    self._write_record(now, packet)
                self.n_packets += 1
                self._last_packet_t = now
            except TimeoutError:
                pass
            except OSError as exc:
                if self._running:
                    self.error = f"Livox 点云接收失败: {exc}"
                break
            silent = self._last_packet_t is None or now - self._last_packet_t > 1.0
            if silent and self.auto_host and now >= next_host_check:
                next_host_check = now + self.host_retry_s
                self._check_host()
            if now >= next_emit:
                next_emit = now + self.period_s
                while window and now - window[0][0] > self.window_s:
                    window.popleft()
                try:
                    if window and self.emit([p for _, p in window]):
                        self.error = None
                except Exception as exc:          # record only: the scan goes stale and LidarGuard treats it as no data
                    self.error = f"Livox 点云处理失败: {type(exc).__name__}: {exc}"
                if fresh and self.batch_sink is not None:
                    batch, fresh = fresh, []
                    try:
                        self.batch_sink(batch)
                    except Exception as exc:      # SLAM errors must not take down collision avoidance
                        self._log(f"Livox 批量回调失败: {type(exc).__name__}: {exc}", "lidar")
                else:
                    fresh = []


# --------------------------------------------------------------------------
# IMU (for the chassis heading loop, see core/heading_hold.py)
# --------------------------------------------------------------------------
class LivoxImu:
    """Background receiver for the Mid-360's built-in IMU; yields robot-frame yaw rate (rad/s, + = left).

    The lidar is fixed to the chassis, so the gyro projected on the vertical axis is the chassis
    rotation: omega = gyro . up (up = mount.rot[2], vertical in the lidar frame). Bias is not removed
    here: only the chassis controller knows when the robot is stationary. The IMU host must be this
    machine (set via tools/livox_diag.py); without data this returns None and the heading loop runs open-loop."""

    def __init__(self, up, *, port: int = HOST_IMU_PORT, window_s: float = 0.05, now_fn=time.monotonic,
                 log=None, sock_factory=None):
        u = np.asarray(up, dtype=np.float64)
        self.up = u / np.linalg.norm(u)
        self.port, self.window_s = int(port), float(window_s)
        self.now_fn = now_fn
        self._log = log or (lambda *a, **k: None)
        self._sock_factory = sock_factory or self._open
        self._lock = threading.Lock()
        self._samples: deque = deque(maxlen=256)          # (receive time, yaw rate)
        self._sock = None
        self._thread = None
        self._running = False
        self.n_packets = 0
        self.error: str | None = None

    def _open(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", self.port))
        return s

    def feed(self, packet: bytes, now: float | None = None) -> bool:
        """Feed one UDP packet (shared by the loop and tests). Returns whether it was a valid IMU packet."""
        imu = parse_imu_packet(packet)
        if imu is None:
            return False
        rate = float(np.dot(imu[0], self.up))
        with self._lock:
            self._samples.append((self.now_fn() if now is None else now, rate))
            self.n_packets += 1
        return True

    def yaw_rate(self):
        """(mean yaw rate over the last window_s, age of newest sample in s); None if no samples yet."""
        with self._lock:
            if not self._samples:
                return None
            t_last = self._samples[-1][0]
            vals = [r for t, r in self._samples if t_last - t <= self.window_s]
        return sum(vals) / len(vals), self.now_fn() - t_last

    def start(self) -> bool:
        if self._running:
            return True
        try:
            self._sock = self._sock_factory()
            self._sock.settimeout(0.1)
        except OSError as exc:
            self.error = f"Livox IMU 端口 {self.port} 绑定失败: {exc}"
            self._log(self.error, "lidar")
            return False
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="livox-imu", daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._running = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def stats(self) -> dict:
        s = self.yaw_rate()
        return {"packets": self.n_packets, "age_s": None if s is None else round(s[1], 2), "error": self.error}

    def _loop(self):
        while self._running:
            try:
                self.feed(self._sock.recv(2048))
            except TimeoutError:
                continue
            except OSError as exc:
                if self._running:
                    self.error = f"Livox IMU 接收失败: {exc}"
                break
