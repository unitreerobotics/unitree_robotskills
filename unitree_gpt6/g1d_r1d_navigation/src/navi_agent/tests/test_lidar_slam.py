# -*- coding: utf-8 -*-
"""Tests for services/lidar_slam.py on a synthetic scene (ray-cast 2D structure points): tracking, mapping,
relocalization, loss, deskew, and the end-to-end SlamRunner path fed with raw Livox packets. Fully offline.

The scene is an asymmetric L-shaped room (pillar, partition, box); a symmetric room cannot be relocalized
uniquely, which is a separate case."""
import math
import struct
import time

import numpy as np
import pytest

pytest.importorskip("cv2")

from services import lidar_slam as ls  # noqa: E402
from services import livox_mid360 as lv  # noqa: E402

ROOM = [((0, 0), (14, 0)), ((14, 0), (14, 6)), ((14, 6), (9, 6)), ((9, 6), (9, 10)),
        ((9, 10), (0, 10)), ((0, 10), (0, 0)),
        ((5, 0), (5, 1.2)),                                                 # half partition (trajectory passes beside it at y=2)
        ((3, 3.2), (4, 3.2)), ((4, 3.2), (4, 4.2)), ((4, 4.2), (3, 4.2)), ((3, 4.2), (3, 3.2)),   # box
        ((10, 2), (11, 2)), ((11, 2), (11, 2.6)), ((11, 2.6), (10, 2.6)), ((10, 2.6), (10, 2)),
        ((6, 7), (6.4, 7)), ((6.4, 7), (6.4, 7.4)), ((6.4, 7.4), (6, 7.4)), ((6, 7.4), (6, 7)),   # pillar
        ((2, 8), (2.4, 8)), ((2.4, 8), (2.4, 8.4)), ((2.4, 8.4), (2, 8.4)), ((2, 8.4), (2, 8))]


def raycast(pose, segs=ROOM, n=900, max_range=12.0, noise=0.01, rng=None):
    """Cast 360° rays from the pose to the nearest wall -> noisy 2D points in the robot frame."""
    rng = rng or np.random.default_rng(0)
    x, y, th = pose
    a = np.linspace(-math.pi, math.pi, n, endpoint=False) + th
    d = np.stack([np.cos(a), np.sin(a)], 1)
    P = np.array([s[0] for s in segs], float)
    Q = np.array([s[1] for s in segs], float)
    E = Q - P
    W = P - np.array([x, y])
    cross = lambda u, v: u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]        # noqa: E731
    den = cross(d[:, None, :], E[None, :, :])
    with np.errstate(divide="ignore", invalid="ignore"):
        s_ = cross(W[None, :, :], E[None, :, :]) / den
        t_ = cross(W[None, :, :], d[:, None, :]) / den
    s_ = np.where((np.abs(den) > 1e-9) & (s_ > 0.05) & (t_ >= 0) & (t_ <= 1), s_, np.inf)
    r = s_.min(1)
    ok = r <= max_range
    r = r[ok] + rng.normal(0, noise, ok.sum())
    rel = a[ok] - th
    return np.stack([r * np.cos(rel), r * np.sin(rel)], 1).astype(np.float32)


def drive(waypoints, v=0.3, w=0.4, dt=0.1):
    """Follow a polyline: turn in place toward the next point, then drive straight. Returns per-frame ground-truth poses."""
    x, y, th = waypoints[0]
    poses = [(x, y, th)]
    for gx, gy in waypoints[1:]:
        target = math.atan2(gy - y, gx - x)
        while abs(ls.wrap(target - th)) > 1e-6:
            step = max(-w * dt, min(w * dt, ls.wrap(target - th)))
            th = ls.wrap(th + step)
            poses.append((x, y, th))
        dist = math.hypot(gx - x, gy - y)
        for _ in range(int(dist / (v * dt))):
            x, y = x + v * dt * math.cos(th), y + v * dt * math.sin(th)
            poses.append((x, y, th))
    return poses


def err(est, truth, origin):
    """est is in the SLAM map frame (origin at start); truth is in the world frame. Convert truth to the start frame before comparing."""
    rel = ls.relative(origin, truth)
    return math.hypot(est[0] - rel[0], est[1] - rel[1]), abs(math.degrees(ls.wrap(est[2] - rel[2])))


# ---------------- tracking + mapping ----------------
@pytest.fixture(scope="module")
def run():
    poses = drive([(2.0, 2.0, 0.0), (8.0, 2.0), (8.0, 4.8), (12.0, 4.8), (12.0, 1.2)])
    rng = np.random.default_rng(1)
    slam = ls.LidarSlam(size_m=40.0)
    errs, t0 = [], time.monotonic()
    for k, p in enumerate(poses):
        slam.process(raycast(p, rng=rng), 0.1 * k)
        errs.append(err(slam.pose, p, poses[0]))
    return dict(slam=slam, poses=poses, errs=errs, ms=(time.monotonic() - t0) * 1000 / len(poses))


def test_tracks_a_multi_leg_drive(run):
    pos, ang = zip(*run["errs"])
    print("\\n帧 %d  末端误差 %.3fm %.2f°  最大 %.3fm %.2f°  %.1fms/帧" % (
        len(pos), pos[-1], ang[-1], max(pos), max(ang), run["ms"]))
    assert run["slam"].state == "tracking"
    assert max(pos) < 0.10 and max(ang) < 2.0                    # 13m polyline, three 90° turns
    assert run["slam"].quality > 0.6
    assert run["ms"] < 60                                        # each frame must finish within 100ms at 10Hz


def test_snapshot_reports_pose_in_map_frame(run):
    slam = run["slam"]
    slam._wall_t = slam.now_fn()
    snap = slam.snapshot()
    assert snap["ok"] and snap["state"] == "tracking" and snap["errors"] == []
    assert snap["pose"]["frame"] == ls.FRAME and set(snap["pose"]) == {"x", "y", "yaw", "frame"}
    slam._wall_t -= 5.0                                          # data stalled: localization no longer trusted
    snap = slam.snapshot()
    assert not snap["ok"] and "slam_stale" in snap["errors"]


def test_keyposes_follow_the_path(run):
    k = run["slam"].keyposes
    assert len(k) > 15                                           # 13m / 0.5m
    gaps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(k, k[1:])]
    assert min(gaps) >= 0.49


def test_map_has_walls_and_free_space(run):
    slam = run["slam"]
    origin = run["poses"][0]
    def cell(wx, wy):                                            # world frame -> map cell
        r = ls.relative(origin, (wx, wy, 0.0))
        u, v = slam.map.cells(np.array([[r[0], r[1]]], np.float32))[0]
        return slam.map.L[v, u]
    assert max(cell(5.0, 0.6), cell(4.95, 0.6), cell(5.05, 0.6)) > 0.5      # partition
    assert max(cell(7.0, 0.0), cell(7.0, 0.05), cell(7.0, -0.05)) > 0.5    # south wall
    assert cell(7.0, 2.0) < 0                                    # traversed floor
    assert cell(20.0, 20.0) == 0                                 # never visited: unknown


# ---------------- persistence + relocalization ----------------
def test_save_load_relocalize(run, tmp_path):
    path = tmp_path / "map.npz"
    run["slam"].save(path)
    slam = ls.LidarSlam.load(path)
    assert slam.state == "relocalizing" and slam.map_id == run["slam"].map_id
    assert slam.snapshot()["pose"] is None
    origin = run["poses"][0]
    truth = (8.3, 4.5, math.radians(130))                        # somewhat off the trajectory, arbitrary heading
    res = slam.relocalize(raycast(truth, rng=np.random.default_rng(5)), 0.0)
    assert res["ok"], res
    e_pos, e_ang = err(slam.pose, truth, origin)
    assert e_pos < 0.08 and e_ang < 2.0 and slam.state == "tracking"
    slam.process(raycast(truth, rng=np.random.default_rng(6)), 0.1)   # tracking resumes normally
    assert slam.state == "tracking"


def test_relocalize_refuses_unknown_place(run, tmp_path):
    path = tmp_path / "map.npz"
    run["slam"].save(path)
    slam = ls.LidarSlam.load(path)
    elsewhere = [((0, 0), (3, 0)), ((3, 0), (3, 20)), ((3, 20), (0, 20)), ((0, 20), (0, 0))]   # long corridor
    res = slam.relocalize(raycast((1.5, 10.0, 0.3), segs=elsewhere), 0.0)
    assert not res["ok"] and slam.state == "relocalizing"


def test_lost_after_consecutive_mismatches(run):
    slam = ls.LidarSlam(size_m=40.0, lost_after=5)
    for k in range(3):
        slam.process(raycast((2.0, 2.0, 0.0)), 0.1 * k)
    elsewhere = [((0, 0), (3, 0)), ((3, 0), (3, 20)), ((3, 20), (0, 20)), ((0, 20), (0, 0))]
    states = [slam.process(raycast((1.5, 10.0, 1.0), segs=elsewhere), 1.0 + 0.1 * k) for k in range(6)]
    assert states[0] == "degraded" and states[-1] == "lost"
    assert "slam_lost" in slam.snapshot()["errors"]


# ---------------- preprocessing ----------------
def test_structure_points_filters_and_dedups():
    pts = np.array([[2.0, 0.0, 0.05],          # ground
                    [2.0, 0.0, 1.0],           # wall
                    [2.01, 0.01, 1.2],         # same voxel
                    [-0.4, 0.0, 1.0],          # robot body
                    [2.0, 0.5, 3.0],           # ceiling
                    [20.0, 0.0, 1.0]], np.float32)   # too far
    xy = ls.structure_points(pts)
    assert xy.shape == (1, 2) and xy[0, 0] == pytest.approx(2.0, abs=0.02)


def test_deskew_moves_points_into_reference_frame():
    # Turning in place at 0.4 rad/s: a point 2m dead ahead captured 0.1s earlier should appear ~2.3° to the right in the reference-time robot frame
    pts = np.array([[2.0, 0.0, 1.0]], np.float32)
    out = ls.deskew(pts, np.array([0.0]), 0.1, (0.0, 0.0, 0.4))
    assert math.degrees(math.atan2(out[0, 1], out[0, 0])) == pytest.approx(-2.29, abs=0.05)
    out = ls.deskew(pts, np.array([0.0]), 0.1, (0.3, 0.0, 0.0))     # straight at 0.3m/s: point ends up 3cm closer
    assert out[0, 0] == pytest.approx(1.97, abs=1e-4)
    assert ls.deskew(pts, np.array([0.0]), 0.1, (0.0, 0.0, 0.0)) is pts


def test_pose_algebra():
    a, b = (1.0, 2.0, 0.5), (3.0, -1.0, 2.0)
    r = ls.relative(a, b)
    back = ls.compose(a, r)
    assert back == pytest.approx(b, abs=1e-9)


# ---------------- end-to-end: raw Livox packets -> SlamRunner ----------------
def packets_from(mount, xy_robot, t_s, heights=(0.4, 0.9, 1.5), cnt0=0):
    """2D structure points -> 3D points at several heights -> lidar frame -> raw Livox packets of 96 points (with lidar timestamps)."""
    pts = np.concatenate([np.c_[xy_robot, np.full(len(xy_robot), h)] for h in heights]).astype(np.float32)
    p = pts + np.array([mount.front_offset_m, 0.0, -mount.height_m], np.float32)
    lid = np.round(p @ mount.rot * 1000).astype(int)
    out = []
    for k, i in enumerate(range(0, len(lid), 96)):
        chunk = lid[i:i + 96]
        head = bytearray(lv.POINT_HEADER)
        struct.pack_into("<H", head, 5, len(chunk))
        struct.pack_into("<H", head, 7, (cnt0 + k) & 0xFFFF)
        head[10] = lv.DATA_TYPE_CARTESIAN_HIGH
        struct.pack_into("<Q", head, 28, int(t_s * 1e9))
        out.append(bytes(head) + b"".join(struct.pack("<iiiBB", *c, 10, 0) for c in chunk.tolist()))
    return out


def test_runner_consumes_raw_livox_packets(tmp_path):
    mount = lv.Mount()
    poses = drive([(2.0, 2.0, 0.0), (6.0, 2.0), (6.0, 4.0)])
    slam = ls.LidarSlam(size_m=40.0)
    runner = ls.SlamRunner(slam, mount, map_path=tmp_path / "map.npz")
    for k, p in enumerate(poses):
        runner.step(packets_from(mount, raycast(p, n=400), 0.1 * k, cnt0=k * 20))
    e_pos, e_ang = err(slam.pose, poses[-1], poses[0])
    assert slam.state == "tracking" and e_pos < 0.1 and e_ang < 2.0
    assert runner.save() and (tmp_path / "map.npz").exists()
    st = runner.stats()
    assert st["batches"] == len(poses) and st["proc_ms"] is not None


def test_runner_relocalizes_after_reload_and_after_loss(run, tmp_path):
    mount = lv.Mount()
    path = tmp_path / "map.npz"
    run["slam"].save(path)
    slam = ls.LidarSlam.load(path)
    runner = ls.SlamRunner(slam, mount, map_path=path, relocalize_every_s=0.0)
    assert not runner.save()                                     # not yet matched to the old map: must not overwrite the file
    truth = (6.0, 2.2, 0.2)
    runner.step(packets_from(mount, raycast(truth, n=500), 0.0))
    assert slam.state == "tracking" and runner.last_relocalize["ok"]
    slam.state = "lost"                                          # lost: next batch switches to relocalization automatically
    runner.step(packets_from(mount, raycast(truth, n=500), 1.0))
    assert slam.state == "tracking"


# ---------------- server: R1D place-marking tools ----------------
@pytest.fixture
def r1d(monkeypatch, tmp_path, run):
    from types import SimpleNamespace

    import config
    import navi_mcp_server as srv
    from services.robots.profile import build_profile
    profile = build_profile("r1d", {"slam_dir": str(tmp_path)})
    d = SimpleNamespace(profile=profile, camera=None, motion=SimpleNamespace(is_moving=lambda: False),
                        _log=lambda *a, **k: None)
    monkeypatch.setattr(srv.driver_factory, "current_driver", lambda: d)
    monkeypatch.setattr(srv, "get_driver", lambda: d)
    monkeypatch.setattr(srv, "_section", lambda name: config.navi_section({}, name))
    monkeypatch.setattr(srv, "_seq_exec", None)
    monkeypatch.setattr(srv, "_lidar", object())                    # guard already built: _get_lidar_guard returns it directly
    slam = run["slam"]
    slam._wall_t = slam.now_fn()
    slam.state = "tracking"
    runner = ls.SlamRunner(slam, lv.Mount(), map_path=tmp_path / "default" / "map.npz")
    monkeypatch.setattr(srv, "_slam_runner", runner)
    return SimpleNamespace(srv=srv, slam=slam, runner=runner, dir=tmp_path / "default", driver=d)


def test_r1d_localization_and_capabilities(r1d):
    assert r1d.driver.profile.capabilities()["localization"] == "lidar_slam"
    loc = r1d.srv.get_localization()
    assert loc["ok"] and loc["source"] == "lidar_slam" and loc["pose"]["frame"] == ls.FRAME
    assert r1d.srv.mapping_capabilities()["backend"] == "lidar_slam"


def test_r1d_save_place_then_list_with_distance_and_bearing(r1d):
    slam = r1d.slam
    home = slam.pose
    res = r1d.srv.save_place("起点")
    assert res["ok"] and res["place"]["source"] == "lidar_slam" and res["place"]["frame"] == ls.FRAME
    assert (r1d.dir / "map.npz").exists() and (r1d.dir / "places.json").exists()
    again = r1d.srv.save_place("起点")
    assert again["ok"] and again["place"]["replaced"] is True      # same name overwrites by default: places are fresh for each task
    guard = r1d.srv.save_place("起点", overwrite=False)
    assert not guard["ok"] and "overwrite" in guard["error"]       # pass overwrite=False explicitly to keep the old place
    # Robot moves to (2, -2) front-right of the start place and turns 90° left: the start is 45° to its front-left
    slam.pose = ls.compose(home, (2.0, -2.0, math.pi / 2))
    listed = r1d.srv.list_places()
    p = listed["places"][0]
    assert listed["ok"] and p["name"] == "起点"
    assert p["distance_m"] == pytest.approx(2.83, abs=0.01)
    assert p["bearing_deg"] == pytest.approx(45.0, abs=0.5)        # positive = left
    assert p["heading_diff_deg"] == pytest.approx(-90.0, abs=0.5)


def test_r1d_save_place_refused_while_moving_or_unlocalized(r1d):
    from types import SimpleNamespace
    r1d.srv._seq_exec = SimpleNamespace(status=lambda: {"active": True})
    res = r1d.srv.save_place("A")
    assert not res["ok"] and "停稳" in res["error"]
    r1d.srv._seq_exec = None
    r1d.slam.state = "relocalizing"
    res = r1d.srv.save_place("A")
    assert not res["ok"] and "slam_relocalizing" in res["error"]
    assert "已标定点附近" in res["localization"]["hint"]
    listed = r1d.srv.list_places()
    assert listed["ok"] and listed["places"] == [] and not listed["localization"]["ok"]


def test_r1d_navigate_to_is_explicitly_unsupported(r1d):
    res = r1d.srv.navigate_to("起点")
    assert not res["ok"] and res["error"].startswith("unsupported_on_r1d") and "bearing_deg" in res["error"]


def test_build_slam_loads_archives_and_starts_fresh(r1d, monkeypatch):
    srv, profile = r1d.srv, r1d.driver.profile
    assert srv.save_place("A")["ok"]                                   # saves one map + one place
    runner = srv._build_slam(profile, None)
    try:
        assert runner.slam.state == "relocalizing" and runner.slam.map_id == r1d.slam.map_id
        assert [p["name"] for p in srv._slam_place_store(runner.slam).list()] == ["A"]
    finally:
        runner.stop(save=False)
    monkeypatch.setenv("NAVI_SLAM_NEW_MAP", "1")                      # operator requested a new map: old map/places are renamed and kept
    runner = srv._build_slam(profile, None)
    try:
        assert runner.slam.state == "initializing" and runner.slam.map_id != r1d.slam.map_id
        assert not (r1d.dir / "map.npz").exists() and not (r1d.dir / "places.json").exists()
        assert len(list(r1d.dir.glob("map.*.replaced.npz"))) == 1
    finally:
        runner.stop(save=False)
    monkeypatch.delenv("NAVI_SLAM_NEW_MAP")
    (r1d.dir / "map.npz").write_bytes(b"not a map")                   # corrupt file: renamed and kept, new map created, nothing overwritten
    runner = srv._build_slam(profile, None)
    try:
        assert runner.slam.state == "initializing" and list(r1d.dir.glob("map.*.broken.npz"))
    finally:
        runner.stop(save=False)


def test_r1d_panel_lists_only_places_taught_this_run(r1d, monkeypatch):
    import datetime as dt
    assert r1d.srv.save_place("卧室")["ok"]
    monkeypatch.setattr(r1d.srv, "_RUN_STARTED", dt.datetime.now(dt.timezone.utc))   # "restart": earlier places belong to the previous run
    assert r1d.srv._panel_places() == []
    assert r1d.srv.save_place("厨房")["ok"]
    assert r1d.srv.save_place("卧室", overwrite=True)["ok"]                      # old names re-marked this run are still shown
    assert sorted(p["name"] for p in r1d.srv._panel_places()) == ["卧室", "厨房"]
    assert len(r1d.srv.list_places()["places"]) == 2                              # the model-facing list is unaffected


def test_r1d_spin_note_uses_slam_position(r1d, monkeypatch):
    monkeypatch.setattr(r1d.srv, "_spin", r1d.srv.robot_tools.SpinWatch())
    res = {}
    for a in (-90, 40, 70, -120):
        r1d.srv._spin_check(res, a, 0.0)
    assert "spin" not in res
    r1d.srv._spin_check(res, 45, 0.0)
    assert "已累计转了 365°" in res["spin"]
    res = {}
    r1d.slam.pose = ls.compose(r1d.slam.pose, (1.5, 0.0, 0.0))                   # lidar localization: actually moved 1.5m away
    r1d.srv._spin_check(res, 30, 0.0)
    assert "spin" not in res and r1d.srv._spin.turned_deg == 30
