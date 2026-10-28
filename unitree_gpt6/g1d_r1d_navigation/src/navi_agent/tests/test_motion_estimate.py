"""Reported motion is a command integral, not speed*time or measured travel."""
import threading
from types import SimpleNamespace as NS

import pytest

import core.motion as motion


@pytest.fixture
def executor(monkeypatch):
    t = [100.]
    sent = []
    def sleep(dt): t[0] += dt
    monkeypatch.setattr(motion, 'time', NS(monotonic=lambda: t[0], sleep=sleep))
    monkeypatch.setattr(motion.config, 'RAMP_TIME', .5)
    monkeypatch.setattr(motion.config, 'STOP_HOLD_TIME', 0.)
    robot = NS(dt=.1, send_command=lambda vx=0., vy=0., ang_z=0.: sent.append((vx, ang_z)))
    return motion.MotionExecutor(robot), sent


def test_linear_estimate_integrates_limited_ramped_command(executor):
    ex, sent = executor
    ex.speed_limiter = lambda vx, w: (vx*.5, w, {'reason': 'slow'})
    ex._run_linear(1., 0., 'test', .4, .4, threading.Event())
    assert ex._last_moved == pytest.approx(sum(vx*.1 for vx, _ in sent))
    assert 0. < ex._last_moved < .2  # Cannot claim requested .4m was traveled.


def test_turn_estimate_integrates_ramp(executor):
    ex, sent = executor
    ex._run_turn_angle(-.5, 'test', .5, threading.Event())
    assert ex._last_turned == pytest.approx(sum(abs(w)*.1 for _, w in sent))
    # No gyro: stop on integrated command. The ramp-up shortfall is compensated (otherwise every turn undershoots ~6°) and the last tick covers the remainder.
    assert ex._last_turned == pytest.approx(.5, abs=1e-9)
    assert ex._last_turn_mode == 'command' and ex._last_measured is None
    assert all(w <= 0 for _, w in sent)


class _Chassis:
    """Gyro reading = previous tick's commanded yaw rate * gain (gain < 1: chassis undershoots); always 0 when stuck."""
    def __init__(self, sent, gain=.85, stuck=False):
        self.sent, self.gain, self.stuck = sent, gain, stuck

    def measured_yaw_rate(self):
        return 0. if self.stuck or not self.sent else self.sent[-1][1] * self.gain


@pytest.fixture
def gyro_executor(monkeypatch, executor):
    ex, sent = executor
    chassis = _Chassis(sent)
    ex.robot.measured_yaw_rate = chassis.measured_yaw_rate
    ex.turn_stop_lead_s = 0.
    return ex, sent, chassis


def test_turn_stops_on_gyro_angle_not_command(gyro_executor):
    ex, sent, _ = gyro_executor
    ex._run_turn_angle(.5, 'test', .5, threading.Event())
    assert ex._last_turn_mode == 'gyro' and ex._last_turn_stop == 'done'
    assert ex._last_measured == pytest.approx(.5, abs=.05)        # measured reaches target (quantized at 0.0425 rad per tick)
    assert ex._last_turned > ex._last_measured                    # chassis tracks only 85%, so more must be commanded


def test_turn_times_out_when_gyro_sees_no_rotation(gyro_executor):
    ex, sent, chassis = gyro_executor
    chassis.stuck = True
    ex._run_turn_angle(.5, 'test', .5, threading.Event())
    assert ex._last_turn_stop == 'timeout' and ex._last_measured == pytest.approx(0.)
    assert ex._last_turned < 2.                                   # timeout = (1s + 0.25s) * 1.5 + 1s; does not spin forever


def test_gyro_turns_off_falls_back_to_command_integral(gyro_executor):
    ex, sent, _ = gyro_executor
    ex.gyro_turns = False
    ex._run_turn_angle(.5, 'test', .5, threading.Event())
    assert ex._last_turn_mode == 'command' and ex._last_turned == pytest.approx(.5)
    assert ex._last_measured == pytest.approx(.5 * .85, abs=.05)  # gyro still records the measurement: only 85% reached


def test_linear_reports_gyro_yaw_change(executor):
    ex, sent = executor
    ex.robot.measured_yaw_rate = lambda: -.1                      # nose drifts right at 0.1 rad/s while driving straight
    ex._run_linear(1., 0., 'test', .4, .4, threading.Event())
    assert ex._last_yaw_change == pytest.approx(-.1, abs=.02)           # 0.4m at 0.4m/s takes 1s (float may add a tick): ~-6° drift


def test_turn_not_falsely_blocked_after_blocked_move(executor):
    """After a blocked forward move, a turn must execute and return done (regression: a stale
    _last_blocked made a completed turn report blocked)."""
    ex, sent = executor
    ex.speed_limiter = lambda vx, w: (0.0, w, {'reason': 'blocked', 'clear_m': 0.3, 'blocked_s': 3.0})
    res = ex.move_linear_sync(1., 0., speed=.4, distance=.4)
    assert res["status"] == "blocked"
    n = len(sent)
    res = ex.turn_angle_sync(.3, turn_speed=.5)
    assert res["status"] == "done"
    assert len(sent) > n                                                 # turn was actually executed
