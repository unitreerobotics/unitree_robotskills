# -*- coding: utf-8 -*-
"""Tests for services/livox_mid360.py: protocol codec, point-cloud parsing, mount extrinsics, virtual scan, receiver thread, server wiring.

Fully offline (no network). Constants marked "real robot" are raw bytes captured from the Mid-360 on an R1D
and serve as ground truth for the checksum algorithms and field layout."""
import math
import socket
import struct
import time
import zlib
from types import SimpleNamespace

import numpy as np
import pytest

import config
from services import livox_mid360 as lv
from services.lidar import LidarGuard, scan_summary
from services.robots.profile import build_profile

# Real robot: first 18 bytes of a status push frame (cmd 0x0102, length 425) and its CRC16
REAL_PUSH_HEADER = bytes.fromhex("aa00a901b104000002010001000000000000")
REAL_PUSH_CRC16 = 0xF03D
# Real robot: original point-cloud host config (192.168.123.12:56301) and the value read back after pointing it at this host
REAL_POINT_HOST = bytes.fromhex("c0a87b0ceddb0000")
REAL_POINT_HOST_AFTER = bytes.fromhex("c0a87ba4eddb0000")
# Real robot: mount extrinsics refined from 6s of points (= Mount and R1D profile defaults)
DOWN = np.array([0.505, -0.008, -0.863]) / np.linalg.norm([0.505, -0.008, -0.863])
HEIGHT, FRONT = 0.205, 0.27


# ---------------- protocol ----------------
def ack_frame(cmd_id, data, seq=1):
    """Lidar ACK frame (cmd_type=1, sender=1)."""
    head = struct.pack("<BBHIHBB6x", 0xAA, 0, lv.FRAME_HEADER + len(data), seq, cmd_id, 1, 1)
    return head + struct.pack("<HI", lv.crc16_ccitt(head), zlib.crc32(data) & 0xFFFFFFFF) + data


def test_crc16_matches_real_device_header():
    assert lv.crc16_ccitt(REAL_PUSH_HEADER) == REAL_PUSH_CRC16


def test_frame_roundtrip_and_corruption_is_rejected():
    frame = lv.build_frame(lv.CMD_GET, lv.encode_get([lv.KEY_POINT_HOST]), seq=7)
    f = lv.parse_frame(frame)
    assert (f["cmd_id"], f["cmd_type"], f["sender"], f["seq"]) == (lv.CMD_GET, 0, 0, 7)
    assert f["data"] == struct.pack("<HHH", 1, 0, lv.KEY_POINT_HOST)
    for i in (3, 20, len(frame) - 1):                 # length / data CRC / payload
        bad = bytearray(frame)
        bad[i] ^= 0x01
        with pytest.raises(ValueError):
            lv.parse_frame(bytes(bad))
    with pytest.raises(ValueError):
        lv.parse_frame(b"\x00" * 30)


def test_host_cfg_decode_and_ip_swap_match_real_device():
    assert lv.decode_host_cfg(REAL_POINT_HOST) == {"ip": "192.168.123.12", "host_port": 56301, "lidar_port": 0}
    assert lv.with_host_ip(REAL_POINT_HOST, "192.168.123.164") == REAL_POINT_HOST_AFTER    # only the IP changes, port kept


def test_get_ack_decoding():
    body = struct.pack("<BH", 0, 2) + struct.pack("<HH", lv.KEY_POINT_HOST, 8) + REAL_POINT_HOST \
        + struct.pack("<HH", lv.KEY_WORK_STATE, 1) + b"\x01"
    ret, kv = lv.decode_get_ack(body)
    assert ret == 0 and kv == {lv.KEY_POINT_HOST: REAL_POINT_HOST, lv.KEY_WORK_STATE: b"\x01"}


class FakeSock:
    def __init__(self, replies):
        self.replies, self.sent, self.closed = list(replies), [], False

    def settimeout(self, t):
        pass

    def sendto(self, data, addr):
        self.sent.append((data, addr))

    def recvfrom(self, n):
        if not self.replies:
            raise TimeoutError()
        return self.replies.pop(0), ("192.168.123.120", lv.LIDAR_CMD_PORT)

    def close(self):
        self.closed = True


def test_commander_skips_garbage_and_other_commands():
    ok = struct.pack("<BH", 0, 1) + struct.pack("<HH", lv.KEY_WORK_STATE, 1) + b"\x01"
    sock = FakeSock([b"garbage", ack_frame(lv.CMD_PUSH, b"\x00" * 4), ack_frame(lv.CMD_GET, ok)])
    cmd = lv.LivoxCommander("192.168.123.120", sock_factory=lambda: sock)
    assert cmd.get([lv.KEY_WORK_STATE]) == {lv.KEY_WORK_STATE: b"\x01"}
    sent, addr = sock.sent[0]
    assert addr == ("192.168.123.120", lv.LIDAR_CMD_PORT) and lv.parse_frame(sent)["cmd_id"] == lv.CMD_GET
    assert sock.closed


def test_commander_timeout_and_error_code():
    cmd = lv.LivoxCommander("192.168.123.120", timeout_s=0.05, sock_factory=lambda: FakeSock([]))
    with pytest.raises((TimeoutError, socket.timeout)):
        cmd.get([lv.KEY_WORK_STATE])
    cmd = lv.LivoxCommander("192.168.123.120", sock_factory=lambda: FakeSock(
        [ack_frame(lv.CMD_SET, struct.pack("<BH", 3, lv.KEY_POINT_HOST))]))
    with pytest.raises(RuntimeError, match="ret_code=3"):
        cmd.set({lv.KEY_POINT_HOST: REAL_POINT_HOST})


class FakeCommander:
    def __init__(self, value=REAL_POINT_HOST, work_state=1, apply=True, fail_set=False, imu=None):
        self.value, self.work_state, self.apply, self.fail_set, self.sets = value, work_state, apply, fail_set, []
        self.imu = imu                              # None = lidar does not report the IMU key (legacy firmware)

    def get(self, keys):
        kv = {lv.KEY_POINT_HOST: self.value, lv.KEY_WORK_STATE: bytes([self.work_state])}
        if self.imu is not None:
            kv[lv.KEY_IMU_HOST] = self.imu
        return {k: v for k, v in kv.items() if k in keys}

    def set(self, items):
        if self.fail_set:
            raise RuntimeError("Livox 参数写入失败 ret_code=1 key=0x0006")
        self.sets.append(items)
        if self.apply:
            self.value = items.get(lv.KEY_POINT_HOST, self.value)
            self.imu = items.get(lv.KEY_IMU_HOST, self.imu)


def test_ensure_point_host_rewrites_only_when_needed():
    c = FakeCommander()
    out = lv.ensure_point_host("192.168.123.120", "192.168.123.164", commander=c)
    assert out == {"ok": True, "changed": True, "host": "192.168.123.164", "previous": "192.168.123.12",
                   "imu_previous": None, "work_state": 1, "error": None}
    assert c.sets == [{lv.KEY_POINT_HOST: REAL_POINT_HOST_AFTER}]
    out = lv.ensure_point_host("192.168.123.120", "192.168.123.164", commander=c)     # already this host: no write
    assert out["ok"] and not out["changed"] and len(c.sets) == 1


def test_ensure_point_host_also_redirects_imu():
    """Factory config may send both points and IMU to a nonexistent address (e.g. 192.168.1.5); heading hold needs the IMU."""
    c = FakeCommander(imu=REAL_POINT_HOST)
    out = lv.ensure_point_host("192.168.123.120", "192.168.123.164", commander=c)
    assert out["ok"] and out["changed"] and out["imu_previous"] == "192.168.123.12"
    assert c.sets == [{lv.KEY_POINT_HOST: REAL_POINT_HOST_AFTER, lv.KEY_IMU_HOST: REAL_POINT_HOST_AFTER}]
    c2 = FakeCommander(value=REAL_POINT_HOST_AFTER, imu=REAL_POINT_HOST)      # only IMU host wrong: only IMU rewritten
    out = lv.ensure_point_host("192.168.123.120", "192.168.123.164", commander=c2)
    assert out["changed"] and c2.sets == [{lv.KEY_IMU_HOST: REAL_POINT_HOST_AFTER}]
    out = lv.ensure_point_host("192.168.123.120", "192.168.123.164", commander=c2)
    assert out["ok"] and not out["changed"] and len(c2.sets) == 1


def test_ensure_point_host_reports_failures_without_raising():
    out = lv.ensure_point_host("192.168.123.120", "192.168.123.164", commander=FakeCommander(fail_set=True))
    assert not out["ok"] and "ret_code=1" in out["error"]
    out = lv.ensure_point_host("192.168.123.120", "192.168.123.164", commander=FakeCommander(apply=False))
    assert not out["ok"] and "读回仍是 192.168.123.12" in out["error"]


# ---------------- point packets ----------------
def point_packet(points_mm, udp_cnt=0, data_type=1, tags=None):
    """points_mm: [(x, y, z), ...] in millimeters, lidar frame."""
    tags = tags or [0] * len(points_mm)
    head = bytearray(lv.POINT_HEADER)
    struct.pack_into("<H", head, 5, len(points_mm))
    struct.pack_into("<H", head, 7, udp_cnt & 0xFFFF)
    head[10] = data_type
    body = b"".join(struct.pack("<iiiBB", int(x), int(y), int(z), 10, t) for (x, y, z), t in zip(points_mm, tags))
    return bytes(head) + body


def test_parse_drops_empty_and_low_confidence_points():
    pkt = point_packet([(1000, -2000, 300), (0, 0, 0), (500, 0, 0), (700, 0, 0)], tags=[0, 0, 0x04, 0])
    xyz, lost = lv.parse_point_packets([pkt])
    assert lost == 0
    np.testing.assert_allclose(xyz, [[1.0, -2.0, 0.3], [0.7, 0.0, 0.0]], atol=1e-6)


def test_parse_counts_lost_packets_and_skips_other_types():
    pkts = [point_packet([(100, 0, 0)], udp_cnt=65534), point_packet([(200, 0, 0)], udp_cnt=65535),
            point_packet([(300, 0, 0)], udp_cnt=2),                       # after wraparound, packets 0 and 1 lost
            point_packet([(400, 0, 0)], udp_cnt=3, data_type=0),          # IMU packet: skipped
            point_packet([(500, 0, 0)], udp_cnt=3)[:-3], b"short"]        # truncated packet: skipped
    xyz, lost = lv.parse_point_packets(pkts)
    assert lost == 2 and xyz[:, 0].tolist() == pytest.approx([0.1, 0.2, 0.3])
    xyz, lost = lv.parse_point_packets([])
    assert xyz.shape == (0, 3) and lost == 0


# ---------------- mount extrinsics ----------------
def to_lidar(mount, pts_robot):
    """Robot frame -> lidar frame (inverse of Mount.to_robot, for building test data)."""
    p = np.asarray(pts_robot, dtype=np.float32) + np.array([mount.front_offset_m, 0.0, -mount.height_m], np.float32)
    return p @ mount.rot


def test_mount_measured_pose():
    m = lv.Mount()
    assert m.pitch_deg == pytest.approx(30.36, abs=0.1) and m.height_m == pytest.approx(HEIGHT)
    np.testing.assert_allclose(m.rot @ DOWN, [0, 0, -1], atol=1e-6)          # measured "down" maps to robot -Z
    # lidar origin in the robot frame: 0.27m behind the front edge, 0.205m above ground
    np.testing.assert_allclose(m.to_robot(np.zeros((1, 3), np.float32))[0], [-FRONT, 0.0, HEIGHT], atol=1e-6)
    # a point along lidar +x lands dead ahead (y~0); ground points get z~0
    ahead = m.to_robot(np.array([[1.0, 0.0, 0.0]], np.float32))[0]
    assert ahead[0] > 0.5 and abs(ahead[1]) < 0.01
    floor = m.to_robot(to_lidar(m, [[1.0, 0.3, 0.0], [3.0, -1.0, 0.0]]))
    np.testing.assert_allclose(floor, [[1.0, 0.3, 0.0], [3.0, -1.0, 0.0]], atol=1e-5)


def test_mount_yaw_and_validation():
    left = lv.Mount(yaw_deg=90.0).to_robot(to_lidar(lv.Mount(), [[1.0, 0.0, 0.0]]) )[0]
    # with lidar x mounted 90° left, what is "ahead" of the lidar is on the robot's left
    assert left[1] == pytest.approx(1.0 + FRONT, abs=0.01) and abs(left[0] + FRONT) < 0.01
    with pytest.raises(ValueError):
        lv.Mount(down=(0, 0, 0))
    with pytest.raises(ValueError):
        lv.Mount(down=(1, 0, 0))


def test_fit_floor_recovers_measured_pose_among_ceiling_and_walls():
    m = lv.Mount(front_offset_m=0.0)
    rng = np.random.default_rng(3)
    n = 6000
    floor = np.c_[rng.uniform(0.4, 5, n), rng.uniform(-3, 3, n), rng.normal(0, 0.01, n)]
    ceiling = np.c_[rng.uniform(-6, 2, n), rng.uniform(-5, 5, n), 3.6 + rng.normal(0, 0.01, n)]       # larger plane
    wall = np.c_[np.full(n // 2, 5.0), rng.uniform(-3, 3, n // 2), rng.uniform(0, 3, n // 2)]
    body = np.c_[rng.uniform(-0.3, 0, 300), rng.uniform(-0.1, 0.1, 300), rng.uniform(0.2, 1.5, 300)]   # self-occlusion near the body
    cloud = to_lidar(m, np.r_[floor, ceiling, wall, body])
    down, height, inliers = lv.fit_floor(cloud)
    assert height == pytest.approx(HEIGHT, abs=0.01) and inliers > 0.8 * n
    np.testing.assert_allclose(down, DOWN, atol=0.01)
    assert lv.fit_floor(cloud[:100]) is None                                   # too few points: no forced answer


# ---------------- virtual scan ----------------
def floor_fan(rng, n=4000):
    """Ground points ahead of the robot (looking backward the lidar sees only ceiling, so ground covers the front half only)."""
    r, a = rng.uniform(0.1, 8, n), rng.uniform(-1.5, 1.5, n)
    return np.c_[r * np.cos(a), r * np.sin(a), rng.normal(0, 0.012, n)]


def box(rng, x, y, w=0.3, z0=0.05, z1=0.6, n=300):
    return np.c_[np.full(n, x), rng.uniform(y - w / 2, y + w / 2, n), rng.uniform(z0, z1, n)]


def summary(points, **kw):
    a, inc, ranges = lv.virtual_scan(np.asarray(points, dtype=np.float32), **kw)
    return scan_summary({"angle_min": a, "angle_inc": inc, "ranges": ranges}, 0.0, 0.35), ranges


def test_scan_open_floor_reports_clear_front_and_blind_rear():
    s, ranges = summary(floor_fan(np.random.default_rng(1)))
    assert s["front_m"] == pytest.approx(10.0) and s["corridor_front_m"] == pytest.approx(10.0, abs=0.05)
    assert s["rear_m"] is None                                 # no returns behind: blind spot, not "no obstacle"
    assert sum(math.isnan(r) for r in ranges) > 30


def test_scan_obstacle_ahead_and_beside():
    rng = np.random.default_rng(2)
    s, _ = summary(np.r_[floor_fan(rng), box(rng, 0.8, 0.0)])
    assert s["front_m"] == pytest.approx(0.8, abs=0.03) and s["corridor_front_m"] == pytest.approx(0.8, abs=0.03)
    # object front-side, outside the sector but clipping the body corner: sector misses it, corridor catches it
    s, _ = summary(np.r_[floor_fan(rng), box(rng, 0.4, 0.30, w=0.1)])
    assert s["front_m"] == pytest.approx(10.0) and s["corridor_front_m"] == pytest.approx(0.4, abs=0.03)
    beside = np.c_[rng.uniform(-0.2, 0.2, 300), np.full(300, 1.2), rng.uniform(0.05, 0.6, 300)]     # 1.2m directly left
    s, _ = summary(np.r_[floor_fan(rng), beside])
    assert s["left_m"] == pytest.approx(1.2, abs=0.05) and s["front_m"] == pytest.approx(10.0)


def test_scan_ignores_own_body_strays_low_and_overhead_points():
    rng = np.random.default_rng(4)
    floor = floor_fan(rng)
    nose = np.c_[rng.uniform(-0.19, 0.0, 200), rng.uniform(-0.3, 0.3, 200), rng.uniform(0.05, 0.16, 200)]    # own nose
    column = np.c_[rng.uniform(-0.5, -0.25, 300), rng.uniform(-0.15, 0.15, 300), rng.uniform(0.2, 1.8, 300)]  # own mast
    stray = np.array([[0.5, 0.0, 0.3]])                                             # single outlier
    cable = np.c_[np.full(100, 0.6), rng.uniform(-0.3, 0.3, 100), np.full(100, 0.03)]   # 3cm-high cable: treated as ground
    lamp = np.c_[np.full(200, 0.7), rng.uniform(-0.3, 0.3, 200), rng.uniform(1.6, 2.5, 200)]   # above head height
    far_floor = np.c_[np.full(200, 6.0), rng.uniform(-0.3, 0.3, 200), np.full(200, 0.10)]     # height error of distant ground
    s, _ = summary(np.r_[floor, nose, column, stray, cable, lamp, far_floor])
    assert s["front_m"] == pytest.approx(10.0) and s["corridor_front_m"] == pytest.approx(10.0, abs=0.05)
    # Regression from the first real-robot version: with the origin still at the lidar (nose on the x>0 side) and the body box
    # not covering the nose, the robot's own nose was reported as an obstacle ~10-20cm ahead (in 1/4 of scans)
    nose_from_lidar = nose + np.array([0.25, 0.0, 0.0])
    s, _ = summary(np.r_[floor, nose_from_lidar], self_box=(-0.9, 0.03, 0.4))
    assert s["corridor_front_m"] < 0.25


def test_scan_empty_cloud_is_all_blind():
    a, inc, ranges = lv.virtual_scan(np.zeros((0, 3), np.float32))
    assert inc == pytest.approx(math.radians(2.0)) and a == pytest.approx(-math.pi + inc / 2) and len(ranges) == 180
    assert all(math.isnan(r) for r in ranges)


# ---------------- near-range confidence gate (near-range artifacts) ----------------
def test_near_artifact_specks_rejected():
    """Sparse, low, isolated phantom points (multipath off a large panel at close range) are not obstacles:
    with ground returns beyond them the bearing reads as clear; with only artifacts it reads nan (no reliable data)."""
    rng = np.random.default_rng(7)
    specks = np.array([[0.05, 0.0, 0.06], [0.055, 0.001, 0.058], [0.08, 0.0, 0.06]])   # 3 points on one bearing, just above the old threshold
    s, _ = summary(np.r_[floor_fan(rng), specks])
    assert s["corridor_front_m"] == pytest.approx(10.0, abs=0.05)
    _, ranges = summary(specks)                                  # artifacts only
    assert all(math.isnan(r) or r > 0.25 for r in ranges)


def test_near_real_wall_passes_gate():
    """A real nearby panel (dense, tall, contiguous) is still blocked."""
    rng = np.random.default_rng(8)
    s, _ = summary(np.r_[floor_fan(rng), box(rng, 0.1, 0.0)])
    assert s["corridor_front_m"] == pytest.approx(0.1, abs=0.03)


def test_near_thin_leg_passes_gate():
    """A thin nearby leg (lamp stand/table leg), a vertical strip over a few adjacent bearings, must not be filtered by the gate."""
    rng = np.random.default_rng(9)
    leg = np.c_[np.full(150, 0.15), rng.uniform(-0.015, 0.015, 150), rng.uniform(0.05, 0.5, 150)]
    s, _ = summary(np.r_[floor_fan(rng), leg])
    assert s["corridor_front_m"] == pytest.approx(0.15, abs=0.03)


# ---------------- guard: rear blind spot ----------------
def test_guard_fed_by_virtual_scan_blocks_forward_and_lets_reverse_through():
    rng = np.random.default_rng(5)
    g = LidarGuard(stop_m=0.45, slow_m=1.2, require=True, corridor_half_width_m=0.35, rear_blind=True)
    assert g.limit(0.15, 0.0)[2]["reason"] == "no_data"                # no scan yet: forward blocked
    assert g.limit(-0.1, 0.0)[0] == pytest.approx(-0.1)                # reverse not blocked by the blind spot
    g.feed(*lv.virtual_scan(np.r_[floor_fan(rng), box(rng, 0.3, 0.0)].astype(np.float32)))
    vx, vyaw, info = g.limit(0.15, 0.3)
    assert (vx, vyaw) == (0.0, 0.3) and info["reason"] == "blocked" and info["clear_m"] == pytest.approx(0.3, abs=0.03)
    assert g.limit(-0.1, 0.0)[:2] == (pytest.approx(-0.1), 0.0)
    g.feed(*lv.virtual_scan(np.r_[floor_fan(rng), box(rng, 0.8, 0.0)].astype(np.float32)))
    vx, _, info = g.limit(0.15, 0.0)
    assert info["reason"] == "slow" and 0 < vx < 0.15
    g.feed(*lv.virtual_scan(floor_fan(rng).astype(np.float32)))
    assert g.limit(0.15, 0.0)[0] == pytest.approx(0.15)


def test_guard_without_rear_blind_keeps_old_behaviour():
    g = LidarGuard(require=True)
    assert g.limit(-0.1, 0.0)[2]["reason"] == "no_data"


# ---------------- receiver thread ----------------
def packets_for(mount, pts_robot, start=0):
    mm = np.round(to_lidar(mount, pts_robot) * 1000).astype(int)
    return [point_packet(mm[i:i + 96].tolist(), udp_cnt=start + k) for k, i in enumerate(range(0, len(mm), 96))]


def test_feeder_emit_turns_packets_into_one_scan():
    rng = np.random.default_rng(6)
    m, scans = lv.Mount(), []
    f = lv.LivoxFeeder(lambda *scan: scans.append(scan), m)
    used = f.emit(packets_for(m, np.r_[floor_fan(rng), box(rng, 1.0, 0.0)]))
    assert used > 4000 and f.n_scans == 1 and len(scans) == 1
    a, inc, ranges = scans[0]
    assert scan_summary({"angle_min": a, "angle_inc": inc, "ranges": ranges})["front_m"] == pytest.approx(1.0, abs=0.03)
    assert f.emit([]) == 0 and f.n_scans == 1                          # no points: no scan emitted (the guard ages out)


class StreamSock:
    """Yield the given packets, then time out forever."""

    def __init__(self, packets):
        self.packets, self.closed = list(packets), False

    def settimeout(self, t):
        self.timeout = t

    def recv(self, n):
        if self.packets:
            return self.packets.pop(0)
        time.sleep(0.005)
        raise TimeoutError()

    def close(self):
        self.closed = True


def wait_for(cond, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_feeder_thread_streams_scans_and_reports_stats():
    rng = np.random.default_rng(7)
    m, scans, checks = lv.Mount(), [], []
    sock = StreamSock(packets_for(m, floor_fan(rng)))
    f = lv.LivoxFeeder(lambda *scan: scans.append(scan), m, period_s=0.02, window_s=1.0, sock_factory=lambda: sock,
                       ensure_host=lambda *a, **k: checks.append(a) or {"ok": True, "changed": False, "work_state": 1})
    assert f.start() and f.start()                                     # repeated start is a no-op
    try:
        assert wait_for(lambda: f.n_scans >= 1)
        st = f.stats()
        assert st["packets"] > 0 and st["lost"] == 0 and st["points_per_scan"] > 1000 and st["error"] is None
    finally:
        f.stop()
    assert sock.closed and checks == []                                # points streaming: lidar config left alone


def test_feeder_silent_lidar_surfaces_host_problem():
    m = lv.Mount()
    for cfg, text in (({"ok": False, "error": "TimeoutError: Livox 命令 0x0101 无应答"}, "无应答"),
                      ({"ok": True, "changed": False, "work_state": 2}, "未在采样")):
        f = lv.LivoxFeeder(lambda *scan: None, m, period_s=0.02, sock_factory=lambda: StreamSock([]),
                           ensure_host=lambda *a, cfg=cfg, **k: cfg)
        f.start()
        try:
            assert wait_for(lambda: f.error is not None) and text in f.error and f.n_scans == 0  # noqa: B023
        finally:
            f.stop()


def test_feeder_start_reports_bind_failure():
    def boom():
        raise OSError(98, "Address already in use")
    f = lv.LivoxFeeder(lambda *scan: None, lv.Mount(), sock_factory=boom)
    assert f.start() is False and "56301" in f.error and "Address already in use" in f.error


# ---------------- robot profile ----------------
def test_r1d_profile_declares_livox_lidar():
    p = build_profile("r1d")
    assert p.has_lidar is True and p.lidar_kind == "livox_mid360" and p.lidar_ready
    assert p.capabilities()["lidar"] is True
    assert p.extra["lidar_ip"] == "192.168.123.120" and p.extra["lidar_require"] is True
    m = lv.Mount(p.extra["lidar_down"], p.extra["lidar_height_m"], p.extra["lidar_yaw_deg"], p.extra["lidar_front_offset_m"])
    default = lv.Mount()                                               # profile and Mount defaults share the same measured values
    np.testing.assert_allclose(m.rot, default.rot, atol=1e-6)
    assert (m.height_m, m.front_offset_m) == (default.height_m, default.front_offset_m) == (HEIGHT, FRONT)
    off = build_profile("r1d", {"has_lidar": False})                   # can be disabled in config.yaml
    assert not off.lidar_ready
    assert build_profile("r1d", {"lidar_height_m": 0.25}).extra["lidar_height_m"] == 0.25


def test_g1d_profile_still_needs_a_scan_topic():
    g = build_profile("g1d")
    assert g.lidar_kind == "dds_scan" and g.lidar_ready
    assert not build_profile("g1d", {"lidar_topic": ""}).lidar_ready


# ---------------- server wiring ----------------
class FakeFeeder:
    instances = []
    start_ok = True

    def __init__(self, sink, mount, **kw):
        self.sink, self.mount, self.kw, self.error = sink, mount, kw, None
        FakeFeeder.instances.append(self)

    def start(self):
        if not FakeFeeder.start_ok:
            self.error = "Livox 点云端口 56301 绑定失败: [Errno 98] Address already in use"
        return FakeFeeder.start_ok

    def stats(self):
        return {"kind": "livox_mid360", "packets": 4168, "lost": 0, "error": self.error}


class Cam:
    def __init__(self, fresh):
        self.fresh = fresh

    def snapshot(self):
        return {"fresh": self.fresh}


@pytest.fixture
def server(monkeypatch):
    import navi_mcp_server as srv
    for name, value in (("_lidar", None), ("_lidar_feeder", None), ("_lidar_error", None),
                        ("_lidar_checked", False), ("_lidar_next_try", 0.0), ("_imu", None)):
        monkeypatch.setattr(srv, name, value)
    monkeypatch.delenv("NAVI_NO_LIDAR", raising=False)
    monkeypatch.setattr(srv, "_section", lambda name: config.navi_section({}, name))   # do not read the on-site config.yaml
    monkeypatch.setattr(lv, "LivoxFeeder", FakeFeeder)
    FakeFeeder.instances, FakeFeeder.start_ok = [], True
    return srv


def use_r1d(srv, monkeypatch, camera=None, **overrides):
    d = SimpleNamespace(profile=build_profile("r1d", overrides), camera=camera, _log=lambda *a, **k: None)
    monkeypatch.setattr(srv.driver_factory, "current_driver", lambda: d)
    return d


def test_livox_guard_build_error_backs_off_and_stops_slam(server, monkeypatch):
    """Guard construction raises (e.g. missing calibration fields): stop the started localization, record the reason, retry on an interval rather than rebuilding every tick."""
    use_r1d(server, monkeypatch)
    runner = SimpleNamespace(submit=lambda *a: None, stopped=[])
    runner.stop = lambda save: runner.stopped.append(save)
    builds = []
    monkeypatch.setattr(server, "_build_slam", lambda profile, log: builds.append(1) or runner)

    def boom(*a, **k):
        raise KeyError("lidar_down")
    monkeypatch.setattr(server, "_build_livox_guard", boom)
    assert server._get_lidar_guard() is None
    assert runner.stopped == [False] and "KeyError" in server._lidar_error
    assert server._get_lidar_guard() is None and len(builds) == 1                     # no rebuild within the interval
    assert server._reflex_limit(0.3, 0.0)[0] == 0.0                                   # R1D without lidar data: forward zeroed


def test_server_builds_livox_guard_for_r1d(server, monkeypatch):
    use_r1d(server, monkeypatch)
    g = server._get_lidar_guard()
    feeder = FakeFeeder.instances[0]
    assert g is server._get_lidar_guard() and len(FakeFeeder.instances) == 1          # singleton
    assert g.require and g.rear_blind and g.corridor_half_width_m == 0.35 and g.stop_m == 0.45
    assert not g.sector_guard and g.headway_s == 1.5 and g.creep_mps == 0.2                  # travel-path check + speed envelope
    assert feeder.sink == g.feed and feeder.mount.height_m == pytest.approx(HEIGHT)
    assert feeder.kw["lidar_ip"] == "192.168.123.120" and feeder.kw["auto_host"] is True
    assert feeder.kw["scan_kw"] == {"z_min_m": 0.05, "z_max_m": 1.5, "self_box": (-0.90, 0.03, 0.40)}

    assert server._reflex_limit(0.15, 0.0)[2]["reason"] == "no_data"                  # no scan yet
    rng = np.random.default_rng(8)
    feeder.sink(*lv.virtual_scan(np.r_[floor_fan(rng), box(rng, 0.3, 0.0)].astype(np.float32)))
    vx, vyaw, info = server._reflex_limit(0.15, 0.2)
    assert (vx, vyaw) == (0.0, 0.2) and info["reason"] == "blocked"
    st = server._lidar_status()
    assert st["state"] == "healthy" and st["corridor_front_m"] == pytest.approx(0.3, abs=0.03)
    assert st["source"]["kind"] == "livox_mid360" and st["source"]["packets"] == 4168


def test_server_livox_start_failure_blocks_forward_only_and_retries(server, monkeypatch):
    use_r1d(server, monkeypatch, camera=Cam(fresh=True))
    FakeFeeder.start_ok = False
    assert server._get_lidar_guard() is None and "56301" in server._lidar_error
    vx, _, info = server._reflex_limit(0.15, 0.0)
    assert vx == 0.0 and info["reason"] == "no_data" and "56301" in info["error"]
    assert server._reflex_limit(-0.1, 0.0)[0] == pytest.approx(-0.1)                  # reverse not blocked
    assert server._lidar_status()["state"] == "failed"
    assert len(FakeFeeder.instances) == 1                                             # rate-limited: no per-tick rebuild
    FakeFeeder.start_ok = True
    monkeypatch.setattr(server, "_lidar_next_try", 0.0)                               # retry time reached
    assert server._get_lidar_guard() is not None and server._lidar_error is None


def test_server_r1d_falls_back_to_camera_gate_when_lidar_disabled(server, monkeypatch):
    monkeypatch.setenv("NAVI_NO_LIDAR", "1")
    use_r1d(server, monkeypatch, camera=Cam(fresh=False))
    vx, _, info = server._reflex_limit(0.15, 0.0)
    assert vx == 0.0 and info["reason"] == "camera_stale_no_lidar" and FakeFeeder.instances == []
    use_r1d(server, monkeypatch, camera=Cam(fresh=True))
    vx, _, info = server._reflex_limit(0.15, 0.0)
    assert vx == pytest.approx(0.15) and info["reason"] == "disabled"


def test_server_r1d_lidar_can_be_switched_off_in_config(server, monkeypatch):
    use_r1d(server, monkeypatch, camera=Cam(fresh=True), has_lidar=False)
    assert server._get_lidar_guard() is None and FakeFeeder.instances == []
    vx, _, info = server._reflex_limit(0.15, 0.0)
    assert vx == pytest.approx(0.15) and info["reason"] == "no_lidar_on_r1d"
    assert server._lidar_status()["state"] == "unsupported_on_r1d"


def test_get_status_waits_for_first_scan(server):
    g = SimpleNamespace(n_recv=0)
    t0 = time.monotonic()
    server._wait_first_scan(g, timeout_s=0.2)                    # no scan ever: gives up at timeout instead of hanging
    assert 0.15 < time.monotonic() - t0 < 0.6
    import threading
    threading.Timer(0.08, lambda: setattr(g, "n_recv", 1)).start()
    t0 = time.monotonic()
    server._wait_first_scan(g, timeout_s=2.0)                    # returns as soon as a scan arrives
    assert time.monotonic() - t0 < 0.5 and g.n_recv == 1
    server._wait_first_scan(None)                                # no guard (lidar off): no wait


def test_frame_state_tells_whether_models_frame_is_still_current(server, monkeypatch):
    from collections import OrderedDict
    monkeypatch.setattr(server, "_frames", OrderedDict())
    monkeypatch.setattr(server, "_frame_seq", 0)
    assert server._frame_state() is None                         # no frame given to the model yet
    clock = {"t": 5.0}
    monkeypatch.setattr(server, "_seq_exec", SimpleNamespace(motion_clock=lambda: clock["t"]))
    fid, _ = server._register_frame()
    st = server._frame_state()
    assert st["id"] == fid and st["moved_since_s"] == 0.0 and 0 <= st["age_s"] < 1.0
    clock["t"] = 7.5                                             # sequence ran 2.5s after the frame was taken
    assert server._frame_state()["moved_since_s"] == 2.5


# ---------------- blind-travel cap on suggested horizon / per-frame clearance ----------------
def test_suggested_horizon_capped_by_blind_distance(server, monkeypatch):
    cfg = {**config.navi_section({}, "sequence"), "max_vx": 1.0}          # sequence config with the R1D speed limit
    assert cfg["max_blind_m"] == 5.0
    monkeypatch.setattr(server, "_latency_ema", [30.0])                    # real robot: 20-30s per step
    need, cap = server._horizon_need(cfg, 4.0, speed=0.2)
    assert need == pytest.approx(43.0) and cap == pytest.approx(25.0)     # 5m / planned speed 0.2m/s
    assert server._suggested_horizon(cfg, 4.0, speed=0.2) == 25.0
    assert server._suggested_horizon(cfg, 4.0, speed=0.5) == 10.0         # faster speed: distance-based cap is tighter
    assert server._suggested_horizon(cfg, 4.0) == 5.0                      # no planned speed: uses the model speed limit
    monkeypatch.setattr(server, "_latency_ema", [2.0])                     # fast model: unaffected
    assert server._suggested_horizon(cfg, 4.0, speed=0.2) == pytest.approx(6.6)
    assert server._suggested_horizon({**cfg, "max_blind_m": 0}, 4.0) == pytest.approx(6.6)
    monkeypatch.setattr(server, "_latency_ema", [30.0])
    assert server._suggested_horizon({**cfg, "max_blind_m": 0.1}, 4.0) == 3.0   # not below horizon_min_s


def test_clearance_attached_only_when_lidar_fresh(server, monkeypatch):
    assert server._clearance() is None                                     # guard not built
    clock = {"t": 100.0}
    g = LidarGuard(corridor_half_width_m=0.35, rear_blind=True, now_fn=lambda: clock["t"])
    monkeypatch.setattr(server, "_lidar", g)
    assert server._clearance() is None                                     # no scan yet
    rng = np.random.default_rng(9)
    g.feed(*lv.virtual_scan(np.r_[floor_fan(rng), box(rng, 1.5, 0.0)].astype(np.float32)))
    c = server._clearance()
    assert set(c) == {"ahead_m", "front_m", "left_m", "right_m"}            # ahead_m = travel-corridor clearance
    assert c["front_m"] == pytest.approx(1.5, abs=0.05) and c["ahead_m"] == pytest.approx(1.5, abs=0.05)
    clock["t"] += 5.0                                                       # stale scan: omitted so the model does not trust old data
    assert server._clearance() is None


# ---------------- IMU (gyro for heading hold) ----------------
def imu_packet(gyro, acc=(-0.5, 0.0, 0.86)):
    head = bytearray(lv.POINT_HEADER)
    head[10] = lv.DATA_TYPE_IMU
    return bytes(head) + struct.pack("<6f", *gyro, *acc)


def test_parse_imu_packet_and_reject_points():
    gyro, acc = lv.parse_imu_packet(imu_packet((0.1, -0.2, 0.3)))
    assert gyro == pytest.approx((0.1, -0.2, 0.3)) and acc == pytest.approx((-0.5, 0.0, 0.86))
    assert lv.parse_imu_packet(point_packet([[1000, 0, 0]])) is None          # a point packet is not IMU
    assert lv.parse_imu_packet(b"short") is None


def test_imu_projects_gyro_on_vertical_axis():
    """Lidar pitched down 30°: a 0.3 rad/s left turn about vertical is 0.3*up in the lidar frame; projecting back yields chassis yaw rate."""
    m = lv.Mount()
    clock = {"t": 10.0}
    imu = lv.LivoxImu(m.rot[2], now_fn=lambda: clock["t"])
    assert imu.yaw_rate() is None
    up = np.asarray(m.rot[2], dtype=np.float64)
    for k in range(5):
        imu.feed(imu_packet(tuple(0.3 * up)), now=10.0 + 0.005 * k)
    clock["t"] = 10.03
    rate, age = imu.yaw_rate()
    assert rate == pytest.approx(0.3, abs=1e-4) and age == pytest.approx(0.01)
    side = np.cross(up, [0.0, 1.0, 0.0])                                       # rotation perpendicular to vertical is not yaw
    imu.feed(imu_packet(tuple(0.5 * side / np.linalg.norm(side))), now=10.2)
    assert imu.yaw_rate()[0] == pytest.approx(0.0, abs=1e-4)
    assert not imu.feed(point_packet([[1000, 0, 0]]))


class FakeImu:
    instances = []

    def __init__(self, up, **kw):
        self.up, self.kw = up, kw
        FakeImu.instances.append(self)

    def start(self):
        return True

    def yaw_rate(self):
        return (0.0, 0.01)

    def stats(self):
        return {"packets": 10, "age_s": 0.01, "error": None}


def test_server_attaches_imu_heading_hold_on_r1d(server, monkeypatch):
    monkeypatch.setattr(lv, "LivoxImu", FakeImu)
    FakeImu.instances = []
    attached = {}
    robot = SimpleNamespace(attach_yaw_rate=lambda source, log=None, **kw: attached.update(source=source, **kw))
    d = use_r1d(server, monkeypatch)
    d.robot = robot
    server._get_lidar_guard()
    assert len(FakeImu.instances) == 1 and attached["kp"] == 1.5 and attached["max_corr"] == 0.25
    assert attached["source"]() == (0.0, 0.01)
    np.testing.assert_allclose(FakeImu.instances[0].up, lv.Mount().rot[2], atol=1e-6)
    assert server._lidar_status()["imu"]["packets"] == 10


def test_server_heading_hold_can_be_switched_off(server, monkeypatch):
    monkeypatch.setattr(lv, "LivoxImu", FakeImu)
    FakeImu.instances = []
    d = use_r1d(server, monkeypatch, heading_hold=False)
    d.robot = SimpleNamespace(attach_yaw_rate=lambda *a, **k: pytest.fail("不该挂航向闭环"))
    server._get_lidar_guard()
    assert FakeImu.instances == [] and "imu" not in server._lidar_status()
