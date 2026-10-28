# src/tests/test_robot_tools.py
# -*- coding: utf-8 -*-
"""Tests for the pure-logic layer of the Navi tools."""
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

import robot_tools
from services.robots.profile import build_profile


def _fake_driver(frame_shape=(480, 1280, 3), connected=True, has_frame=True, eye="left", binocular=True,
                 crop_ratio=None):
    """Duck-typed fake driver exposing only what observe_frame uses."""
    class FakeCamera:
        def __init__(self):
            self.connected = connected
            self.binocular = binocular
            frame = np.zeros(frame_shape, dtype=np.uint8)
            if frame_shape[1] == 1280:   # stereo: left/right halves get different gray levels for assertions
                frame[:, :640] = 60
                frame[:, 640:] = 200
            _, buf = cv2.imencode(".jpg", frame)
            self._raw = buf.tobytes() if has_frame else None

        def get_latest_frame(self):
            return self._raw

    class FakeDriver:
        camera = FakeCamera()
        camera_cfg = {"eye": eye}

    drv = FakeDriver()
    if crop_ratio is not None:
        drv.profile = SimpleNamespace(camera_crop_ratio=crop_ratio)
    return drv


def test_observe_frame_left_eye_takes_left_half():
    drv = _fake_driver()
    out = robot_tools.observe_frame(drv, eye=None)     # eye=None uses the configured default (left)
    assert out["ok"] is True
    frame = cv2.imdecode(np.frombuffer(out["jpeg"], np.uint8), cv2.IMREAD_COLOR)
    assert frame.shape[1] == 640                     # half of 1280
    assert frame[:, :].mean() < 130                  # left half is dark (60)


def test_observe_frame_right_eye():
    drv = _fake_driver()
    out = robot_tools.observe_frame(drv, eye="right")
    assert out["ok"] is True
    frame = cv2.imdecode(np.frombuffer(out["jpeg"], np.uint8), cv2.IMREAD_COLOR)
    assert frame[:, :].mean() > 130                  # right half is bright (200)


def test_observe_frame_full_keeps_whole():
    drv = _fake_driver()
    out = robot_tools.observe_frame(drv, eye="full")
    frame = cv2.imdecode(np.frombuffer(out["jpeg"], np.uint8), cv2.IMREAD_COLOR)
    assert frame.shape[1] == 1280


def test_observe_frame_monocular_keeps_full_frame():
    drv = _fake_driver(binocular=False)
    out = robot_tools.observe_frame(drv, eye="left")
    assert out["ok"] is True
    frame = cv2.imdecode(np.frombuffer(out["jpeg"], np.uint8), cv2.IMREAD_COLOR)
    assert frame.shape[1] == 1280                     # mono: not split, full width kept


def test_observe_frame_without_crop_ratio_has_no_crop():
    out = robot_tools.observe_frame(_fake_driver(binocular=False), eye="full")
    assert "crop_jpeg" not in out and "crop_ratio" not in out


def test_observe_frame_crop_ratio_adds_center_crop_and_keeps_full():
    drv = _fake_driver(frame_shape=(448, 544, 3), binocular=False, crop_ratio=0.8)
    out = robot_tools.observe_frame(drv, eye=None)
    full = cv2.imdecode(np.frombuffer(out["jpeg"], np.uint8), cv2.IMREAD_COLOR)
    crop = cv2.imdecode(np.frombuffer(out["crop_jpeg"], np.uint8), cv2.IMREAD_COLOR)
    assert full.shape[:2] == (448, 544)              # jpeg stays full-frame (for memory records / observer)
    assert crop.shape[:2] == (448, 544)              # center 80% crop upscaled back to full size (normal-FOV view)
    assert out["crop_ratio"] == 0.8


def test_center_crop_takes_middle():
    frame = np.zeros((10, 20, 3), dtype=np.uint8)
    frame[1:9, 2:18] = 255                           # center 80% white, black border
    crop = robot_tools.center_crop(frame, 0.8)
    assert crop.shape[:2] == (8, 16) and crop.min() == 255
    assert robot_tools.center_crop(frame, 1.0).shape == frame.shape


@pytest.mark.parametrize("bad", [0, -0.5, 1.2, float("nan")])
def test_center_crop_rejects_bad_ratio(bad):
    with pytest.raises(ValueError):
        robot_tools.center_crop(np.zeros((4, 4, 3), dtype=np.uint8), bad)


def test_profile_crop_ratio_defaults_and_override():
    assert build_profile("g1d").camera_crop_ratio == 1.0
    assert build_profile("r1d").camera_crop_ratio == 0.5
    assert build_profile("r1d", {"camera_crop_ratio": 0.7}).camera_crop_ratio == 0.7
    with pytest.raises(ValueError):
        build_profile("r1d", {"camera_crop_ratio": 0})


def test_observe_frame_no_camera():
    drv = _fake_driver(connected=False)
    out = robot_tools.observe_frame(drv, eye=None)
    assert out == {"ok": False, "error": "相机未连接（observe 需要 image_server 在线）"}


def test_observe_frame_no_frame_yet():
    drv = _fake_driver(has_frame=False)
    out = robot_tools.observe_frame(drv, eye=None)
    assert out["ok"] is False and "无帧" in out["error"]


def test_frame_to_jpeg_roundtrip():
    frame = np.full((120, 160, 3), 128, dtype=np.uint8)
    jpeg = robot_tools.frame_to_jpeg(frame)
    back = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert back is not None and back.shape == (120, 160, 3)


# ==========================================================================
# Guard clamping (guard_move / guard_turn)
# ==========================================================================
import pytest  # noqa: E402


def test_guard_move_normal():
    out = robot_tools.guard_move("forward", 1.5, max_m=2.5)
    assert out == {"direction": "forward", "distance_m": 1.5, "clamped": False}


def test_guard_move_clamps_over_cap():
    out = robot_tools.guard_move("forward", 5.0, max_m=2.5)
    assert out["distance_m"] == 2.5 and out["clamped"] is True


def test_guard_move_rejects_non_positive():
    with pytest.raises(ValueError):
        robot_tools.guard_move("forward", 0.0, max_m=2.5)
    with pytest.raises(ValueError):
        robot_tools.guard_move("forward", -0.3, max_m=2.5)


def test_guard_move_rejects_unknown_direction():
    with pytest.raises(ValueError):
        robot_tools.guard_move("diagonal", 1.0, max_m=2.5)


def test_guard_turn_normal_and_clamp():
    assert robot_tools.guard_turn(45.0, max_deg=90.0) == {"angle_deg": 45.0, "clamped": False}
    out = robot_tools.guard_turn(120.0, max_deg=90.0)
    assert out == {"angle_deg": 90.0, "clamped": True}


def test_guard_turn_rejects_zero():
    with pytest.raises(ValueError):
        robot_tools.guard_turn(0.0, max_deg=90.0)


def test_config_has_navi_guard_defaults():
    import config as cfg_mod
    defaults = cfg_mod.DEFAULT_CONFIG["navi"]["guard"]
    assert defaults["max_move_distance_m"] == 5.0
    assert defaults["max_turn_angle_deg"] == 360.0


# ---------- guard_sequence ----------
SEQ_LIMITS = {"max_vx": 0.3, "max_vyaw": 0.5, "max_duration_s": 10.0,
              "max_move_distance_m": 2.0, "max_turn_angle_deg": 180.0}


def test_guard_sequence_passes_clean_input():
    g = robot_tools.guard_sequence(
        [{"vx": 0.3, "vyaw": 0.0, "duration": 2.0}, {"vx": 0.0, "vyaw": 0.5, "duration": 1.0}],
        SEQ_LIMITS)
    assert g["segments"] == [(0.3, 0.0, 2.0), (0.0, 0.5, 1.0)]
    assert g["clamped"] is False and g["total_s"] == 3.0


def test_guard_sequence_clamps_speed_keeping_sign():
    g = robot_tools.guard_sequence([{"vx": -1.0, "vyaw": 2.0, "duration": 1.0}], SEQ_LIMITS)
    assert g["segments"][0][:2] == (-0.3, 0.5) and g["clamped"] is True


def test_guard_sequence_truncates_on_distance_budget():
    """0.3 m/s * 10s = 3m exceeds the 2m hard cap: truncated to 6.67s, later segments dropped."""
    g = robot_tools.guard_sequence(
        [{"vx": 0.3, "duration": 10.0}, {"vx": 0.3, "duration": 5.0}], SEQ_LIMITS)
    assert len(g["segments"]) == 1
    assert g["segments"][0][2] == pytest.approx(2.0 / 0.3)
    assert g["clamped"] is True


def test_guard_sequence_truncates_on_turn_budget():
    """0.5 rad/s * 10s = 286° exceeds 180°: truncated to pi/0.5 s."""
    import math
    g = robot_tools.guard_sequence([{"vyaw": 0.5, "duration": 10.0}], SEQ_LIMITS)
    assert g["segments"][0][2] == pytest.approx(math.pi / 0.5)


def test_guard_sequence_truncates_on_duration_budget():
    g = robot_tools.guard_sequence(
        [{"vx": 0.1, "duration": 8.0}, {"vx": 0.1, "duration": 8.0}], SEQ_LIMITS)
    assert g["total_s"] == pytest.approx(10.0) and g["clamped"] is True


def test_guard_sequence_missing_fields_default_zero():
    g = robot_tools.guard_sequence([{"duration": 0.4}], SEQ_LIMITS)
    assert g["segments"] == [(0.0, 0.0, 0.4)]


@pytest.mark.parametrize("bad", [
    [], None, "x", [{"vx": 0.3}], [{"vx": 0.3, "duration": 0}],
    [{"vx": "fast", "duration": 1}], [3],
])
def test_guard_sequence_rejects_malformed(bad):
    with pytest.raises(ValueError):
        robot_tools.guard_sequence(bad, SEQ_LIMITS)


def test_guard_sequence_strips_trailing_zero_segments():
    """Trailing zero-speed segments are stripped (not counted against the budget); motion_s counts only moving segments."""
    g = robot_tools.guard_sequence(
        [{"vx": 0.3, "duration": 4.0}, {"vx": 0, "vyaw": 0, "duration": 36.0}], SEQ_LIMITS)
    assert g["segments"] == [(0.3, 0.0, 4.0)]
    assert g["motion_s"] == 4.0 and g["stripped_zero_s"] == 36.0 and g["total_s"] == 4.0


def test_guard_sequence_zero_padding_does_not_eat_duration_budget():
    """Real-robot case: [move 9s, zero 36s] under a 10s budget must not truncate the moving segment."""
    g = robot_tools.guard_sequence(
        [{"vx": 0.2, "duration": 9.0}, {"vx": 0, "duration": 36.0}], SEQ_LIMITS)
    assert g["segments"] == [(0.2, 0.0, 9.0)] and g["clamped"] is False


def test_guard_sequence_pure_zero_becomes_short_stop():
    g = robot_tools.guard_sequence([{"vx": 0, "vyaw": 0, "duration": 45.0}], SEQ_LIMITS)
    assert g["segments"] == [(0.0, 0.0, 0.5)] and g["motion_s"] == 0.0


def test_guard_sequence_keeps_interior_zero_segments():
    """A zero-speed segment in the middle (move-stop-move) is a deliberate pause and is kept."""
    g = robot_tools.guard_sequence(
        [{"vx": 0.3, "duration": 1.0}, {"duration": 2.0}, {"vx": 0.3, "duration": 1.0}], SEQ_LIMITS)
    assert [d for *_, d in g["segments"]] == [1.0, 2.0, 1.0]


@pytest.mark.parametrize("deg", [0.0, 5.0, -8.0, 9.9])
def test_guard_turn_rejects_below_min_angle(deg):
    with pytest.raises(ValueError, match="至少 10"):
        robot_tools.guard_turn(deg, max_deg=90.0)


def test_guard_turn_accepts_min_angle():
    assert robot_tools.guard_turn(10.0, max_deg=90.0)["angle_deg"] == 10.0
    assert robot_tools.guard_turn(-10.0, max_deg=90.0)["angle_deg"] == -10.0


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_motion_rejected(value):
    with pytest.raises(ValueError): robot_tools.guard_move('forward', value, 5.)
    with pytest.raises(ValueError): robot_tools.guard_turn(value, 90.)
    limits = {'max_vx': .5, 'max_vyaw': .5, 'max_duration_s': 10.,
              'max_move_distance_m': 5., 'max_turn_angle_deg': 90.}
    for key in ('vx', 'vyaw', 'duration'):
        with pytest.raises(ValueError):
            robot_tools.guard_sequence([{'vx': .2, 'vyaw': 0., 'duration': 1., key: value}], limits)


def test_stale_camera_is_rejected_and_source_id_is_preserved():
    drv = _fake_driver()
    data = dict(raw=drv.camera._raw, fresh=False, source_frame_seq=7, received_at=10., frame_age_ms=2000.)
    drv.camera.snapshot = lambda: dict(data)
    out = robot_tools.observe_frame(drv, None)
    assert not out['ok'] and out['source_frame_seq'] == 7 and 'jpeg' not in out
    data.update(fresh=True, frame_age_ms=50.)
    a, b = robot_tools.observe_frame(drv, None), robot_tools.observe_frame(drv, None)
    assert a['ok'] and a['source_frame_seq'] == b['source_frame_seq'] == 7


def test_camera_snapshot_detects_cached_frame_age(monkeypatch):
    import services.camera as camera
    from services.camera import CameraStream
    cam = CameraStream('unused', 0, False)
    cam._received_at, cam._latest_frame, cam._n_recv = 10., b'frame', 3
    monkeypatch.setattr(camera.time, 'monotonic', lambda: 12.)
    assert cam.snapshot()['fresh'] is False
    assert cam.snapshot()['frame_age_ms'] == 2000.


def test_observe_waits_briefly_for_a_fresh_frame():
    # After packet loss the R1D camera has no new frame for ~1s (decoder skips to the next keyframe): wait briefly rather than reporting a camera failure
    drv = _fake_driver()
    calls = {"n": 0}

    def snap():
        calls["n"] += 1
        return dict(raw=drv.camera._raw, fresh=calls["n"] > 3, source_frame_seq=calls["n"],
                    received_at=10., frame_age_ms=50.)
    drv.camera.snapshot = snap
    out = robot_tools.observe_frame(drv, None, wait_fresh_s=1.0)
    assert out["ok"] and out["source_frame_seq"] == 4
    calls["n"] = 0
    assert not robot_tools.observe_frame(drv, None)["ok"]                  # no wait by default (observer polling)
    drv.camera.snapshot = lambda: dict(raw=drv.camera._raw, fresh=False, source_frame_seq=1,
                                       received_at=10., frame_age_ms=3000.)
    import time
    t0 = time.monotonic()
    out = robot_tools.observe_frame(drv, None, wait_fresh_s=0.2)            # genuinely stalled: still errors at timeout
    assert not out["ok"] and "过期" in out["error"] and 0.15 < time.monotonic() - t0 < 0.6


# ---------- task board validation ----------

def test_normalize_tasks_keeps_order_and_defaults_todo():
    out = robot_tools.normalize_tasks([
        {"name": " A1 找到卧室并标记 ", "status": "done", "note": "已标记"},
        {"name": "A2 找到厨房并标记", "status": "doing"},
        {"name": "B1 检查冰箱门"},
    ])
    assert out == [{"name": "A1 找到卧室并标记", "status": "done", "note": "已标记"},
                   {"name": "A2 找到厨房并标记", "status": "doing"},
                   {"name": "B1 检查冰箱门", "status": "todo"}]


def test_normalize_tasks_maps_common_status_aliases():
    out = robot_tools.normalize_tasks([{"name": "a", "status": "in_progress"},
                                       {"name": "b", "status": "Completed"},
                                       {"name": "c", "status": "skipped", "note": "用户说不找了"}])
    assert [t["status"] for t in out] == ["doing", "done", "dropped"]


def test_normalize_tasks_truncates_long_text():
    out = robot_tools.normalize_tasks([{"name": "长" * 100, "note": "注" * 200}])
    assert len(out[0]["name"]) == 60 and len(out[0]["note"]) == 80


def test_normalize_tasks_empty_list_clears_board():
    assert robot_tools.normalize_tasks([]) == []


@pytest.mark.parametrize("bad", [
    "A1 卧室",                                    # not a list
    ["A1 卧室"],                                  # item not an object
    [{"status": "todo"}],                         # missing name
    [{"name": "a"}, {"name": "a"}],               # duplicate name
    [{"name": "a", "status": "maybe"}],           # unknown status
    [{"name": str(i)} for i in range(robot_tools.MAX_TASKS + 1)],
])
def test_normalize_tasks_rejects_malformed(bad):
    with pytest.raises(ValueError):
        robot_tools.normalize_tasks(bad)


# ---------- spin-in-place detection ----------

def test_spin_watch_flags_back_and_forth_turning_in_place():
    # turn sequence recorded on the real robot: turning back and forth with almost no displacement
    w = robot_tools.SpinWatch()
    pose = (-2.6, -3.0)
    hits = [w.record(a, 0.0, pose) for a in (-90, 40, 70, -120, 45)]
    assert hits[:4] == [None] * 4                                   # 320° is not yet a full turn
    assert hits[4] == {"turned_deg": 365, "moved_m": 0.0}
    w2 = robot_tools.SpinWatch()
    for a in (-90, 40, 70, -120):
        assert w2.record(a, 0.0, pose) is None                      # 320°
    hit = w2.record(-80, 0.0, (-2.4, -3.1))
    assert hit == {"turned_deg": 400, "moved_m": 0.2}


def test_spin_watch_resets_after_real_displacement():
    w = robot_tools.SpinWatch()
    for a in (90, -90, 90):
        w.record(a, 0.0, (0.0, 0.0))
    assert w.record(0, 1.2, (1.1, 0.0)) is None                     # actually moved 1.1m away: reset
    assert w.turned_deg == 0 and w.anchor == (1.1, 0.0)
    assert w.record(90, 0.0, (1.1, 0.0)) is None


def test_spin_watch_falls_back_to_commanded_translation_without_pose():
    w = robot_tools.SpinWatch()
    for a in (120, -120):
        w.record(a, 0.0)
    w.record(0, 1.0)                                                # commanded translation reaches 1m
    assert w.record(150, 0.0) is None and w.turned_deg == 150       # next step counts afresh after the move reset
    assert w.record(-150, 0.3) is None
    assert w.record(90, 0.0) == {"turned_deg": 390, "moved_m": 0.3}


def test_spin_watch_does_not_nag_the_step_that_leaves():
    w = robot_tools.SpinWatch()
    for a in (180, -180):
        w.record(a, 0.0, (0.0, 0.0))
    assert w.record(20, 2.0, (0.0, 0.0)) is None                    # this step intends to drive 2m away
    assert w.record(30, 0.4, (0.1, 0.0)) == {"turned_deg": 410, "moved_m": 0.1}
