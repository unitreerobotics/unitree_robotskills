# -*- coding: utf-8 -*-
"""Tests for services/lidar.py: sector minimum (decreasing increment, ±180 seam, invalid returns), speed-limit curve, guard state machine.

Scan layout matches the real SLAMWARE: 360 beams, angle_min=+180°, angle_increment=-1° (decreasing)."""
import math

import pytest

from services.lidar import (
    LidarGuard,
    corridor_min,
    limit_forward,
    path_min,
    scan_summary,
    sector_min,
)


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


def make_scan(obstacles=None, far=10.0, n=360, angle_min=180.0, inc=-1.0):
    """obstacles: {angle_deg: range}; unlisted directions read `far` meters. Returns (angle_min, inc, ranges)
    with angles in radians per the LaserScan convention (sector_min/corridor_min read radians)."""
    ranges = [far] * n
    for ang, r in (obstacles or {}).items():
        i = int(round((ang - angle_min) / inc))
        assert 0 <= i < n, f"角度 {ang}° 不在扫描范围内"
        ranges[i] = r
    return math.radians(angle_min), math.radians(inc), ranges


# ---------- pure function: sector_min ----------
def test_sector_min_picks_obstacle_in_front():
    a0, inc, r = make_scan({0.0: 0.8})
    assert sector_min(a0, inc, r, -30, 30) == pytest.approx(0.8)
    assert sector_min(a0, inc, r, 30, 150) == pytest.approx(10.0)   # left side clear


def test_sector_min_handles_decreasing_increment():
    """Real angle_increment is negative: an obstacle at +40° must land in the left sector, not the right."""
    a0, inc, r = make_scan({40.0: 0.7})
    assert inc < 0
    assert sector_min(a0, inc, r, 30, 150) == pytest.approx(0.7)
    assert sector_min(a0, inc, r, -150, -30) == pytest.approx(10.0)


def test_sector_min_skips_invalid_and_near_noise():
    """None / NaN / inf / <5cm returns are not obstacles (under 5cm is sensor noise, not a wall)."""
    a0, inc, r = make_scan({0.0: 2.0, 10.0: 0.01, 20.0: float("nan"), -20.0: float("inf")})
    i_none = int(round((5.0 - math.degrees(a0)) / math.degrees(inc)))
    r[i_none] = None
    assert sector_min(a0, inc, r, -30, 30) == pytest.approx(2.0)


def test_sector_min_crosses_pm180_seam():
    """Rear sector (150,-150) spans ±180: both +179° and -179° must count."""
    a0, inc, r = make_scan({179.0: 0.9, -179.0: 0.6})
    assert sector_min(a0, inc, r, 150, -150) == pytest.approx(0.6)
    assert sector_min(a0, inc, r, -30, 30) == pytest.approx(10.0)


def test_sector_min_all_invalid_returns_none():
    a0, inc, r = make_scan({}, far=None)
    assert sector_min(a0, inc, r, -30, 30) is None


def test_scan_summary_reports_four_sectors_and_yaw_offset():
    a0, inc, r = make_scan({0.0: 0.5, 90.0: 2.0, -90.0: 3.0})
    s = scan_summary({"angle_min": a0, "angle_inc": inc, "ranges": r})
    assert (s["front_m"], s["left_m"], s["right_m"], s["rear_m"]) == (0.5, 2.0, 3.0, 10.0)
    # Lidar mounted 90° off: the 0° obstacle falls in the right sector, the 90° one straight ahead (mount offset rotates all sectors)
    off = scan_summary({"angle_min": a0, "angle_inc": inc, "ranges": r}, yaw_offset_deg=90.0)
    assert off["front_m"] == pytest.approx(2.0) and off["right_m"] == pytest.approx(0.5)
    assert off["rear_m"] == pytest.approx(3.0)


# ---------- pure function: limit_forward ----------
@pytest.mark.parametrize("vx,clear,data_ok,require,expect_v,expect_reason", [
    (0.3, 0.2, True, False, 0.0, "blocked"),      # too close: forward component zeroed
    (0.3, 0.44, True, False, 0.0, "blocked"),
    (0.3, 0.45, True, False, 0.3 * 0.0, "slow"),  # exactly at stop_m: factor 0
    (0.3, 0.825, True, False, 0.15, "slow"),      # linear slowdown in between
    (0.3, 1.2, True, False, 0.3, "ok"),           # above slow_m: pass
    (0.3, None, False, False, 0.3, "ok"),         # no lidar data, not required: pass
    (0.3, None, False, True, 0.0, "no_data"),     # required: block
    (-0.3, 0.2, True, False, -0.3, "ok"),         # reverse is not clamped here
])
def test_limit_forward(vx, clear, data_ok, require, expect_v, expect_reason):
    v, reason = limit_forward(vx, clear, 0.45, 1.2, data_ok, require)
    assert (v, reason) == (pytest.approx(expect_v), expect_reason)


# ---------- guard ----------
def make_guard(clock=None, **kw):
    clock = clock or Clock()
    kw.setdefault("log", lambda *a, **k: None)
    return LidarGuard(now_fn=clock, **kw), clock


def test_guard_no_data_follows_require_flag():
    g, clock = make_guard()
    assert g.limit(0.3, 0.1)[0] == pytest.approx(0.3)        # passes by default (offline/stub)
    g2, _ = make_guard(require=True)
    v, w, info = g2.limit(0.3, 0.1)
    assert v == 0.0 and w == pytest.approx(0.1) and info["reason"] == "no_data"


def test_guard_blocks_when_obstacle_inside_stop_distance():
    g, clock = make_guard(max_age_s=10.0)
    g.feed(*make_scan({0.0: 0.3}))
    v, w, info = g.limit(0.4, 0.0)
    assert v == 0.0 and info["reason"] == "blocked" and info["clear_m"] == pytest.approx(0.3)
    clock.t += 1.5
    assert g.limit(0.4, 0.0)[2]["blocked_s"] == pytest.approx(1.5)   # blocked duration accumulates


def test_guard_slows_down_in_transition_band():
    g, _ = make_guard()
    g.feed(*make_scan({0.0: 0.8}))
    v, _, info = g.limit(0.4, 0.0)
    assert 0.0 < v < 0.4 and info["reason"] == "slow"


def test_guard_never_clamps_turn_and_uses_rear_for_backward():
    g, _ = make_guard()
    g.feed(*make_scan({179.0: 0.3}))          # obstacle only behind
    assert g.limit(0.4, 0.5)[0] == pytest.approx(0.4)       # forward unaffected
    v, w, info = g.limit(-0.4, 0.5)
    assert v == 0.0 and info["reason"] == "blocked"         # reverse blocked
    assert w == pytest.approx(0.5)                          # vyaw is never clamped: turning in place is how to escape


# ---------- travel corridor (straight strip of body width) ----------
def test_corridor_min_catches_side_obstacle_the_cone_misses():
    """Door frame at 50°, 0.5m: body frame (0.32, 0.38); invisible to the ±30° sector, visible to a 0.4m half-width corridor."""
    a0, inc, r = make_scan({50.0: 0.5, 70.0: 1.0})          # 70°/1m -> y=0.94, outside the corridor
    assert sector_min(a0, inc, r, -30, 30) == pytest.approx(10.0)
    assert corridor_min(a0, inc, r, 0.4) == pytest.approx(0.5 * math.cos(math.radians(50)))
    assert corridor_min(a0, inc, r, 0.0) is None            # disabled
    assert corridor_min(a0, inc, r, 0.4, rear=True) > 9.9   # only the 10m returns (longitudinal projection of the ±2° beams)


def test_corridor_rear_uses_backward_half():
    a0, inc, r = make_scan({130.0: 0.5})
    assert corridor_min(a0, inc, r, 0.4, rear=True) == pytest.approx(0.5 * math.cos(math.radians(50)))
    assert corridor_min(a0, inc, r, 0.4) > 9.9


def test_guard_corridor_blocks_only_when_enabled():
    scan = make_scan({50.0: 0.5})
    off, _ = make_guard()
    off.feed(*scan)
    assert off.limit(0.3, 0.0)[0] == pytest.approx(0.3)     # sector-only by default: pass (legacy behavior unchanged)
    assert "corridor_front_m" not in off.summary()
    on, _ = make_guard(corridor_half_width_m=0.4)
    on.feed(*scan)
    v, w, info = on.limit(0.3, 0.2)
    assert v == 0.0 and info["reason"] == "blocked" and w == pytest.approx(0.2)
    assert info["clear_m"] == pytest.approx(0.32, abs=0.01)
    assert on.summary()["corridor_front_m"] == pytest.approx(0.32, abs=0.01)


def test_guard_stale_scan_counts_as_no_data():
    g, clock = make_guard(max_age_s=1.0, require=True)
    g.feed(*make_scan({0.0: 5.0}))
    assert g.limit(0.4, 0.0)[2]["reason"] == "ok"
    clock.t += 2.0
    v, _, info = g.limit(0.4, 0.0)
    assert v == 0.0 and info["reason"] == "no_data"


def test_guard_summary_reports_age_and_error():
    g, clock = make_guard(max_age_s=1.0)
    clock.t += 0.5
    s = g.summary()
    assert s["ok"] is False and s["n_recv"] == 0 and s["front_m"] is None
    s0 = g.summary()
    assert "error" in s0


# ---------- speed envelope (headway, R1D) ----------
@pytest.mark.parametrize("vx,clear,expect_v,expect_reason", [
    (1.0, 0.40, 0.0, "blocked"),                  # still zero inside stop_m
    (1.0, 0.50, 0.2, "slow"),                     # near an obstacle: keep creeping at 0.2, not 0.03
    (0.2, 0.50, 0.2, "ok"),                       # 0.2 is already inside the envelope: not reduced to 0.13
    (1.0, 1.20, 0.5, "slow"),                     # (1.2-0.45)/1.5 = 0.5
    (1.0, 1.95, 1.0, "ok"),                       # 1m/s only with ~2m clear ahead
    (0.5, math.inf, 0.5, "ok"),                   # no returns on the path
])
def test_limit_forward_headway_envelope(vx, clear, expect_v, expect_reason):
    v, reason = limit_forward(vx, clear, 0.45, 1.2, True, True, headway_s=1.5, creep_mps=0.2)
    assert (v, reason) == (pytest.approx(expect_v), expect_reason)


# ---------- travel path (R1D: sector_guard=False) ----------
def corridor_walls(half=0.6, extra=None):
    """Endless straight corridor with walls at y=±half (origin at the front edge; no returns dead ahead/behind); extra={angle: range} adds obstacles."""
    obstacles = {float(ang): half / abs(math.sin(math.radians(ang)))
                 for ang in range(-179, 180) if ang != 0}
    obstacles.update(extra or {})
    return make_scan(obstacles, far=None)


def test_path_min_straight_equals_corridor():
    a0, inc, r = make_scan({50.0: 0.5})
    assert path_min(a0, inc, r, 0.4, 0.3, 0.0) == pytest.approx(corridor_min(a0, inc, r, 0.4))
    assert path_min(a0, inc, r, 0.4, 0.0, 0.3) is None       # turning in place: not forward motion
    assert path_min(a0, inc, r, 0.0, 0.3, 0.0) is None       # disabled


def test_path_min_follows_the_arc():
    """Left arc R=1m: an obstacle 1m dead ahead is off the arc (the arc has curved away); one at 45° along the arc is ~0.78m away along it."""
    a0, inc, r = make_scan({0.0: 1.0})
    assert corridor_min(a0, inc, r, 0.35) == pytest.approx(1.0)        # the straight strip treats it as ahead
    assert path_min(a0, inc, r, 0.35, 0.5, 0.5) is None                # missed by the arc (10m returns are off the arc too)
    a0, inc, r = make_scan({22.0: 0.765})                              # point reached after 45° along the arc
    assert path_min(a0, inc, r, 0.35, 0.5, 0.5) == pytest.approx(0.78, abs=0.03)
    a0, inc, r = make_scan({-22.0: 0.765})                             # right arc, mirrored
    assert path_min(a0, inc, r, 0.35, 0.5, -0.5) == pytest.approx(0.78, abs=0.03)
    assert path_min(a0, inc, r, 0.35, 0.5, 0.5) is None                # left arc does not hit a right-side point


def test_path_guard_ignores_corridor_walls_that_slow_the_sector_guard():
    """1m-wide corridor: the ±30° sector sees the side walls ~1m away and slows down; the travel-path check sees a clear path."""
    scan = corridor_walls(half=0.5)
    old, _ = make_guard(corridor_half_width_m=0.35)
    old.feed(*scan)
    assert old.summary()["front_m"] == pytest.approx(1.0, abs=0.05)
    assert old.limit(0.4, 0.0)[2]["reason"] == "slow"
    new, _ = make_guard(corridor_half_width_m=0.35, sector_guard=False, headway_s=1.5, creep_mps=0.2)
    new.feed(*scan)
    v, _, info = new.limit(0.8, 0.0)
    assert v == pytest.approx(0.8) and info["reason"] == "ok" and info["clear_m"] is None


def test_path_guard_caps_speed_by_distance_and_blocks_at_stop():
    g, _ = make_guard(corridor_half_width_m=0.35, sector_guard=False, headway_s=1.5, creep_mps=0.2)
    g.feed(*corridor_walls(half=0.6, extra={0.0: 1.2}))
    v, _, info = g.limit(1.0, 0.0)
    assert v == pytest.approx(0.5) and info["reason"] == "slow" and info["cap_mps"] == pytest.approx(0.5)
    g.feed(*corridor_walls(half=0.6, extra={0.0: 0.4}))
    v, _, info = g.limit(0.2, 0.0)
    assert v == 0.0 and info["reason"] == "blocked" and info["clear_m"] == pytest.approx(0.4)
    assert "cap_mps" not in info


def test_path_guard_without_scan_still_requires_data():
    g, _ = make_guard(corridor_half_width_m=0.35, sector_guard=False, require=True)
    v, _, info = g.limit(0.3, 0.0)
    assert v == 0.0 and info["reason"] == "no_data"


def test_sector_guard_cannot_be_turned_off_without_corridor():
    g, _ = make_guard(sector_guard=False)                   # no corridor width means no path check: falls back to sector
    assert g.sector_guard is True
