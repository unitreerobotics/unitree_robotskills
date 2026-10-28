# -*- coding: utf-8 -*-
"""Tests for core/sequence.py: sampling, conflict detection, merging, frame-time alignment, watchdog, ramp limiting."""
import numpy as np
import pytest

from core.sequence import SequenceExecutor, is_conflict, merge_timelines, sample_segments


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


class FakeRobot:
    dt = 0.1

    def __init__(self):
        self.cmds = []

    def send_command(self, vx=0.0, vy=0.0, ang_z=0.0, **k):
        self.cmds.append((round(vx, 4), round(ang_z, 4)))


def make(clock=None, **kw):
    clock = clock or Clock()
    robot = FakeRobot()
    kw.setdefault("ramp_time", 0.001)      # ramp effectively off by default to focus on timeline semantics
    ex = SequenceExecutor(robot, now_fn=clock, start_thread=False, **kw)
    return ex, robot, clock


def run(ex, clock, seconds):
    n = int(round(seconds / ex.dt))
    for _ in range(n):
        ex.tick()
        clock.t += ex.dt


# ---------- pure functions ----------
def test_sample_segments_rounds_to_ticks():
    tl = sample_segments([(0.3, 0.0, 1.0), (0.0, 0.5, 0.55)], dt=0.1)
    assert tl.shape == (10 + 6, 2)
    assert tl[0].tolist() == [0.3, 0.0] and tl[-1].tolist() == [0.0, 0.5]


@pytest.mark.parametrize("old,new,expect", [
    ((0.3, 0.0), (0.3, 0.0), False),     # same direction
    ((0.3, 0.0), (0.2, 0.3), False),     # straight plus turn: blending acceptable
    ((0.0, 0.0), (0.3, 0.0), False),     # start from rest: blend acts as the ramp
    ((0.3, 0.0), (-0.3, 0.0), True),     # linear velocity reverses
    ((0.0, 0.5), (0.0, -0.5), True),     # angular velocity reverses
    ((0.3, 0.0), (0.0, 0.0), True),      # old moving, new stops: stop must take effect immediately
    ((0.0, 0.0), (0.0, 0.0), False),     # both at rest
])
def test_is_conflict(old, new, expect):
    assert is_conflict(old, new) is expect


def test_merge_blend_crossfades_then_follows_new():
    old = np.array([[0.2, 0.0]] * 10)
    new = np.array([[0.4, 0.0]] * 10)
    out = merge_timelines(old, new, "blend", n_blend=4)
    assert out[0, 0] == pytest.approx(0.2 * 0.75 + 0.4 * 0.25)
    assert out[3, 0] == pytest.approx(0.4)
    assert (out[4:, 0] == 0.4).all()


def test_merge_blend_short_old_only_blends_over_old_length():
    old = np.array([[0.2, 0.0]] * 2)
    new = np.array([[0.4, 0.0]] * 10)
    out = merge_timelines(old, new, "blend", n_blend=5)
    assert out[1, 0] == pytest.approx(0.4)          # window narrowed to the old duration
    assert (out[2:, 0] == 0.4).all()


def test_merge_replace_drops_old_tail():
    old = np.array([[0.3, 0.0]] * 20)
    new = np.array([[0.0, 0.5]] * 5)
    out = merge_timelines(old, new, "replace", n_blend=5)
    assert out.shape == (5, 2) and (out[:, 1] == 0.5).all()


# ---------- executor ----------
def test_fresh_submit_executes_then_settles_zero():
    ex, robot, clock = make()
    r = ex.submit([(0.3, 0.0, 0.5)])
    assert r["ok"] and r["mode"] == "fresh" and r["remaining_s"] == pytest.approx(0.5)
    run(ex, clock, 0.5)
    assert robot.cmds[0] == (0.3, 0.0) and len(robot.cmds) == 5
    run(ex, clock, 0.3)
    assert robot.cmds[5] == (0.0, 0.0)               # zero speed appended at the end
    assert len(robot.cmds) == 6                       # zero not repeated afterwards
    assert ex.status()["active"] is False


def test_corrective_plan_does_not_trim_turn_by_old_forward_time():
    """Time spent on the old straight segment must not count as executed time of the new turn; the full new plan is kept by default."""
    ex, robot, clock = make(blend_s=0.1)             # 1-tick blend window so target values can be asserted directly
    ex.submit([(0.3, 0.0, 5.0)])
    fm = ex.motion_clock()
    frame_ts = clock.t
    run(ex, clock, 0.3)
    r = ex.submit([(0.3, 0.0, 0.2), (0.0, 0.5, 0.5)], frame_ts=frame_ts, frame_motion_t=fm)
    assert r["driven_s"] == pytest.approx(0.3) and r["dropped_s"] == 0.0
    assert r["offset_s"] == pytest.approx(0.3)
    assert r["remaining_s"] == pytest.approx(0.7)
    assert r["dropped_s"] == 0
    run(ex, clock, 0.7)
    assert robot.cmds[-1] == (0.0, 0.5)              # first segment fully elapsed, now in the second


def test_idle_robot_does_not_trim_no_matter_how_long_thinking():
    """While the robot is idle (no sequence running) frames do not go stale: even 50s of thinking neither trims nor rejects.
    Regression: on the real robot 13 of 23 submits were wrongly rejected because trimming used wall-clock time."""
    ex, robot, clock = make()
    fm = ex.motion_clock()
    frame_ts = clock.t
    clock.t += 50.0
    r = ex.submit([(0.3, 0.0, 8.0)], frame_ts=frame_ts, frame_motion_t=fm)
    assert r["ok"] is True and r["driven_s"] == 0.0 and r["dropped_s"] == 0.0
    assert r["offset_s"] == pytest.approx(50.0)      # latency still reported accurately
    assert r["remaining_s"] == pytest.approx(8.0)


def test_submit_rejects_when_driven_exceeds_plan():
    """The old sequence ran 2s after the frame and the new one plans only 1s: it lies entirely in the past and is rejected."""
    ex, robot, clock = make()
    ex.submit([(0.3, 0.0, 5.0)])
    fm = ex.motion_clock()
    run(ex, clock, 2.0)
    r = ex.submit([(0.3, 0.0, 1.0)], frame_ts=100., frame_motion_t=fm, alignment="continue")
    assert r["ok"] is False and "过期" in r["error"] and r["driven_s"] == pytest.approx(2.0)
    assert ex.status()["active"] is True             # rejection does not interrupt the running sequence


def test_motion_clock_stops_when_sequence_finishes():
    ex, robot, clock = make()
    ex.submit([(0.3, 0.0, 1.0)])
    run(ex, clock, 1.0)
    t1 = ex.motion_clock()
    clock.t += 30.0
    assert ex.motion_clock() == pytest.approx(t1)    # idle after finishing: clock does not advance
    assert t1 == pytest.approx(1.0)


def test_motion_clock_survives_replace_and_stop():
    ex, robot, clock = make()
    ex.submit([(0.3, 0.0, 5.0)])
    run(ex, clock, 1.0)
    ex.submit([(-0.3, 0.0, 5.0)])                    # replace: first 1s counted
    run(ex, clock, 0.5)
    ex.stop()                                        # stop: another 0.5s counted
    clock.t += 10.0
    assert ex.motion_clock() == pytest.approx(1.5)


def test_same_direction_blends():
    ex, robot, clock = make(blend_s=0.4)
    ex.submit([(0.2, 0.0, 2.0)])
    run(ex, clock, 0.5)
    r = ex.submit([(0.4, 0.0, 2.0)])
    assert r["mode"] == "blend"
    ex.tick()
    assert 0.2 < robot.cmds[-1][0] < 0.4            # intermediate blended value
    run(ex, clock, 0.5)
    assert robot.cmds[-1] == (0.4, 0.0)


def test_conflict_replaces_immediately():
    ex, robot, clock = make()
    ex.submit([(0.3, 0.0, 3.0)])
    run(ex, clock, 0.5)
    r = ex.submit([(0.0, 0.0, 0.5)])                # request stop
    assert r["mode"] == "replace"
    ex.tick()
    assert robot.cmds[-1] == (0.0, 0.0)
    assert ex.status()["remaining_s"] == pytest.approx(0.5, abs=0.11)


def test_new_sequence_supersedes_old_tail():
    ex, robot, clock = make()
    ex.submit([(0.3, 0.0, 5.0)])
    run(ex, clock, 0.5)
    ex.submit([(0.3, 0.0, 1.0)])                    # newer plan is shorter
    assert ex.status()["remaining_s"] == pytest.approx(1.0)


def test_watchdog_stops_when_upper_layer_silent():
    ex, robot, clock = make(watchdog_s=1.0)
    ex.submit([(0.3, 0.0, 5.0)])
    run(ex, clock, 0.9)
    assert robot.cmds[-1] == (0.3, 0.0) and not ex.starved
    run(ex, clock, 0.3)
    assert ex.starved and robot.cmds[-1] == (0.0, 0.0)
    assert ex.status()["active"] is False


def test_resubmit_resets_watchdog():
    ex, robot, clock = make(watchdog_s=1.0)
    ex.submit([(0.3, 0.0, 5.0)])
    run(ex, clock, 0.8)
    ex.submit([(0.3, 0.0, 5.0)])
    run(ex, clock, 0.8)
    assert not ex.starved and robot.cmds[-1] == (0.3, 0.0)


def test_ramp_limits_step_change():
    ex, robot, clock = make(ramp_time=0.5, max_vx=0.3)   # at most 0.06 change per tick
    ex.submit([(0.3, 0.0, 1.0)])
    ex.tick()
    assert robot.cmds[-1][0] == pytest.approx(0.06)
    run(ex, clock, 0.5)
    assert robot.cmds[-1][0] == pytest.approx(0.3)


def test_stop_clears_and_sends_zero():
    ex, robot, clock = make()
    ex.submit([(0.3, 0.0, 5.0)])
    run(ex, clock, 0.3)
    ex.stop()
    assert robot.cmds[-1] == (0.0, 0.0)
    assert ex.status()["active"] is False
    ex.tick()
    assert len([c for c in robot.cmds if c == (0.0, 0.0)]) == 1   # zero not repeated


def test_motion_clock_ignores_zero_speed_ticks():
    """Zero-speed segments do not advance the motion clock: for [move 1s, stop 5s] with the frame at submit time, after 6s current still holds the whole correction sequence."""
    ex, robot, clock = make()
    fm = ex.motion_clock()
    ex.submit([(0.3, 0.0, 1.0), (0.0, 0.0, 5.0)])
    run(ex, clock, 6.0)
    assert ex.motion_clock() == pytest.approx(1.0)
    r = ex.submit([(0.3, 0.0, 3.0)], frame_motion_t=fm)
    assert r["ok"] and r["driven_s"] == pytest.approx(1.0) and r["remaining_s"] == pytest.approx(3.0)


# ---------- reflex layer (speed_limiter): pause the plan while blocked instead of consuming it ----------
def _limiter(reason="blocked", clear=0.3, blocked_s=1.0):
    def lim(vx, vyaw):
        return 0.0, vyaw, {"reason": reason, "clear_m": clear, "blocked_s": blocked_s}
    return lim


def test_blocked_forward_pauses_timeline_instead_of_burning_it():
    """The timeline does not advance during the 0.5s blocked ahead: once the obstacle clears, the remaining 1s resumes from the pause
    instead of "plan finished but robot never moved"."""
    state = {"blocked": True}

    def lim(vx, vyaw):
        if state["blocked"]:
            return 0.0, vyaw, {"reason": "blocked", "clear_m": 0.3, "blocked_s": 1.0}
        return vx, vyaw, {"reason": "ok", "clear_m": 5.0, "blocked_s": 0.0}

    ex, robot, clock = make(speed_limiter=lim)
    ex.submit([(0.3, 0.0, 1.0)])
    run(ex, clock, 0.5)
    assert robot.cmds[-1] == (0.0, 0.0)                       # stopped as required
    assert ex.status()["remaining_s"] == pytest.approx(1.0)   # plan not consumed
    assert ex.status()["reflex"]["reason"] == "blocked"
    state["blocked"] = False
    run(ex, clock, 1.0)
    assert ex.status()["active"] is False                     # completes after the obstacle clears
    assert robot.cmds[-1] == (0.3, 0.0)


def test_blocked_turn_in_place_does_not_pause():
    """Turning in place bypasses the reflex layer: vyaw is never clamped and the timeline advances (otherwise the robot would be trapped)."""
    ex, robot, clock = make(speed_limiter=_limiter())
    ex.submit([(0.0, 0.5, 1.0)])
    run(ex, clock, 0.5)
    assert ex.status()["remaining_s"] == pytest.approx(0.5)
    assert robot.cmds[-1][1] == pytest.approx(0.5)            # still turning


def test_no_data_reason_pauses_too_when_required():
    ex, robot, clock = make(speed_limiter=_limiter("no_data", None))
    ex.submit([(0.3, 0.0, 1.0)])
    run(ex, clock, 0.4)
    assert ex.status()["remaining_s"] == pytest.approx(1.0)


# ---------- blocked-motion safety rules (from a real-robot wall collision post-mortem) ----------
def test_blocked_arc_also_kills_rotation():
    """A blocked arc with forward motion also brakes the turn rather than degrading into spinning against the wall; the plan pauses without being consumed."""
    ex, robot, clock = make(speed_limiter=_limiter())
    ex.submit([(0.25, 0.35, 2.0)])
    run(ex, clock, 0.3)
    assert robot.cmds[-1] == (0.0, 0.0)
    assert ex.status()["remaining_s"] == pytest.approx(2.0)


def test_blocked_during_deceleration_kills_rotation_too():
    """Sudden block mid-arc: the protective clamp zeroes angular velocity immediately too (previously only linear)."""
    ex, robot, clock = make(ramp_time=1.0)
    ex.submit([(0.3, 0.3, 4.0)])
    run(ex, clock, 1.0)                                   # ramped up to (0.3, 0.3)
    assert robot.cmds[-1][0] > 0 and robot.cmds[-1][1] > 0
    ex.speed_limiter = _limiter()
    ex.tick()
    assert robot.cmds[-1] == (0.0, 0.0)


def test_blocked_sequence_aborts_after_threshold():
    """Blocked continuously for blocked_abort_s: abandon the remaining plan and stop without waiting for the model (it takes tens of seconds);
    do not resume when the obstacle clears, but accept a new plan and reset aborted."""
    state = {"blocked": True, "t": 0.0}

    def lim(vx, vyaw):
        if state["blocked"]:
            return 0.0, vyaw, {"reason": "blocked", "clear_m": 0.3, "blocked_s": state["t"]}
        return vx, vyaw, {"reason": "ok", "clear_m": 5.0, "blocked_s": 0.0}

    ex, robot, clock = make(speed_limiter=lim)
    ex.submit([(0.3, 0.0, 5.0)])
    for _ in range(30):                                   # blocked for 3.0s straight (threshold 2.0s)
        state["t"] += ex.dt
        ex.tick()
        clock.t += ex.dt
    st = ex.status()
    assert st["aborted"] is True and st["active"] is False
    assert robot.cmds[-1] == (0.0, 0.0)
    state["blocked"] = False                              # no auto-resume after the obstacle clears
    run(ex, clock, 0.5)
    assert robot.cmds[-1] == (0.0, 0.0) and ex.status()["active"] is False
    res = ex.submit([(0.3, 0.0, 1.0)])                    # new plan accepted
    assert res["ok"] and ex.status()["aborted"] is False
    run(ex, clock, 1.0)
    assert robot.cmds[-1] == (0.3, 0.0)


def test_brief_block_does_not_abort():
    """Blocked below the threshold (blocked_s < blocked_abort_s): pause only, then finish once cleared."""
    ex, robot, clock = make(speed_limiter=_limiter(blocked_s=1.0))
    ex.submit([(0.3, 0.0, 1.0)])
    run(ex, clock, 3.0)                                   # wall clock exceeds the threshold but blocked_s does not
    assert ex.status()["aborted"] is False and ex.status()["active"] is True


def test_limiter_not_called_without_motion():
    """The reflex layer is not called while idle and settled, so a missing live scan produces no no_data log noise."""
    calls = []

    def lim(vx, vyaw):
        calls.append(vx)
        return vx, vyaw, {"reason": "ok", "clear_m": 5.0, "blocked_s": 0.0}
    ex, robot, clock = make(speed_limiter=lim)
    run(ex, clock, 0.3)
    assert calls == [] and robot.cmds == []


def test_continue_rejects_different_historical_prefix():
    ex, _, clock = make()
    ex.submit([(.3, 0., 5.)])
    fm, ts = ex.motion_clock(), clock.t
    run(ex, clock, .5)
    result = ex.submit([(0., .5, 2.)], frame_ts=ts, frame_motion_t=fm, alignment='continue')
    assert not result['ok'] and 'current' in result['error']


def test_continue_allows_only_matching_constant_prefix():
    ex, _, clock = make()
    ex.submit([(.3, 0., 5.)])
    fm, ts = ex.motion_clock(), clock.t
    run(ex, clock, .5)
    result = ex.submit([(.3, 0., 1.), (0., .5, 1.)], frame_ts=ts, frame_motion_t=fm, alignment='continue')
    assert result['ok'] and result['dropped_s'] == .5 and result['remaining_s'] == 1.5


def test_protection_bypasses_slow_deceleration_ramp():
    ex, robot, clock = make(ramp_time=1.)
    ex.submit([(.3, 0., 4.)])
    run(ex, clock, 1.)
    assert robot.cmds[-1][0] > 0
    ex.speed_limiter = _limiter()
    ex.tick()
    assert robot.cmds[-1][0] == 0.


def test_slow_factor_not_applied_twice():
    ex, robot, clock = make()
    ex.speed_limiter = lambda vx, w: (vx*.5, w, {'reason': 'slow'})
    ex.submit([(.4, 0., 2.)])
    ex.tick()
    assert robot.cmds[-1][0] == .2


def test_stop_cannot_be_overwritten_by_inflight_tick():
    import threading
    entered, release = threading.Event(), threading.Event()
    ex, robot, _ = make()
    def limiter(vx, w):
        entered.set()
        assert release.wait(1.)
        return vx, w, {'reason': 'ok'}
    ex.speed_limiter = limiter
    ex.submit([(.3, 0., 2.)])
    tick = threading.Thread(target=ex.tick)
    tick.start()
    assert entered.wait(1.)
    stop = threading.Thread(target=ex.stop)
    stop.start()
    release.set()
    tick.join(1.)
    stop.join(1.)
    assert not tick.is_alive() and not stop.is_alive()
    assert robot.cmds[-1] == (0., 0.)
    ex.tick()
    assert robot.cmds[-1] == (0., 0.) and not ex.status()['active']


def test_blocked_duration_survives_final_envelope_check():
    import math

    from services.lidar import LidarGuard
    ex, robot, clock = make()
    guard = LidarGuard(require=True, now_fn=clock)
    guard.feed(-math.pi, math.pi/180, [.2]*360)
    ex.speed_limiter = guard.limit
    ex.submit([(.3, 0., 5.)])
    run(ex, clock, .8)
    assert ex.status()['reflex']['blocked_s'] >= .6
    assert guard.blocked_since is not None and all(vx == 0 for vx, _ in robot.cmds)


def test_max_accel_overrides_ramp_scaling_with_max_vx():
    """Raising the cap to 1m/s must not double start-up aggressiveness: when max_accel is set it limits acceleration."""
    ex, robot, clock = make(ramp_time=0.5, max_vx=1.0, max_accel=0.6)   # at most 0.06 change per tick
    ex.submit([(0.6, 0.0, 2.0)])
    ex.tick()
    assert robot.cmds[-1][0] == pytest.approx(0.06)
    legacy, robot2, _ = make(ramp_time=0.5, max_vx=1.0)                  # unset: max_vx / ramp_time
    legacy.submit([(0.6, 0.0, 2.0)])
    legacy.tick()
    assert robot2.cmds[-1][0] == pytest.approx(0.2)


def test_absolute_cap_applies_immediately_despite_ramp():
    """The lidar speed envelope (info.cap_mps) is the final command bound: approaching an obstacle fast does not wait for the deceleration ramp."""
    ex, robot, clock = make(ramp_time=1.0, max_vx=1.0)
    ex.submit([(0.8, 0.0, 4.0)])
    run(ex, clock, 1.0)
    assert robot.cmds[-1][0] == pytest.approx(0.8)
    ex.speed_limiter = lambda vx, w: (min(vx, 0.3), w, {"reason": "slow", "cap_mps": 0.3})
    ex.tick()
    assert robot.cmds[-1][0] == pytest.approx(0.3)                       # not 0.8 - 0.1
