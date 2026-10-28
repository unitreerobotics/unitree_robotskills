# -*- coding: utf-8 -*-
"""Tests for core/heading_hold.py: bias calibration, sign check, correcting drift at start-up, and the open-loop fallbacks.

Plant model: actual yaw rate = command (first-order lag 0.1s) + disturbance; gyro reading = actual * sign + bias."""
import math

import pytest

from core.heading_hold import HeadingHold

DT = 0.02
BIAS = 0.012               # real robot reads ~0.01 rad/s at rest


class Plant:
    def __init__(self, hold, sign=1.0, stuck=False):
        self.hold, self.sign, self.stuck = hold, sign, stuck
        self.rate = 0.0        # actual yaw rate
        self.yaw = 0.0         # actual heading
        self.sent = []

    def run(self, vx, vyaw, seconds, disturb=lambda t: 0.0, imu=True):
        for k in range(int(round(seconds / DT))):
            sample = (self.sign * self.rate + BIAS, 0.01) if imu else None
            out = self.hold.step(vx, vyaw, sample, DT)
            self.sent.append(out)
            target = 0.0 if self.stuck else out + disturb(k * DT)
            self.rate += (target - self.rate) * DT / 0.1
            self.yaw += self.rate * DT


def calibrated(**kw):
    h = HeadingHold(**kw)
    p = Plant(h)
    p.run(0.0, 0.0, 2.2)                          # at rest: bias estimation starts after 1s, runs 1.2s
    return h, p


def test_no_correction_until_bias_calibrated_and_direction_verified():
    h = HeadingHold()
    p = Plant(h)
    p.run(0.0, 0.0, 1.5)
    assert h.state == "calibrating"
    p.run(0.5, 0.0, 0.5)
    assert p.sent[-1] == 0.0                      # bias not yet estimated: open loop
    p.run(0.0, 0.0, 2.0)
    assert h.bias == pytest.approx(BIAS, abs=0.002) and h.state == "verifying"
    p.run(0.5, 0.0, 1.0, disturb=lambda t: -0.2)
    assert all(v == 0.0 for v in p.sent[-50:])    # sign unverified: no correction even under drift
    p.run(0.0, 0.3, 1.0)                          # one turn with matching sign enables the hold
    assert h.verified and h.state == "active"


def test_straight_run_recovers_heading_after_wheel_kick():
    """Wheels pull the nose right for the first 0.6s (-0.2 rad/s, ~-7° total): open loop ends 7° off, closed loop within 1°."""
    h, p = calibrated()
    p.run(0.0, 0.3, 1.0)                          # sign check
    p.run(0.0, 0.0, 1.2)
    assert h.verified
    kick = lambda t: -0.2 if t < 0.6 else 0.0     # noqa: E731
    p.yaw = 0.0
    p.run(0.5, 0.0, 4.0, disturb=kick)
    assert abs(math.degrees(p.yaw)) < 1.0
    assert max(abs(v) for v in p.sent[-200:]) <= h.max_corr + 1e-9   # correction is clamped
    open_loop = Plant(HeadingHold())              # same disturbance, no hold
    open_loop.run(0.5, 0.0, 4.0, disturb=kick)
    assert math.degrees(open_loop.yaw) < -6.0


def test_turn_angle_follows_command():
    """Open loop a 90° turn only reaches ~84° (chassis undershoots the rate); closed loop tracks the command."""
    h, p = calibrated()
    p.run(0.0, 0.3, 1.0)
    p.run(0.0, 0.0, 1.2)
    p.yaw = 0.0
    weak = lambda t: -0.02                        # actual rate 0.02 rad/s below command  # noqa: E731
    p.run(0.0, 0.3, math.pi / 2 / 0.3, disturb=weak)
    assert math.degrees(p.yaw) == pytest.approx(90.0, abs=2.5)


def test_reversed_gyro_disables_permanently():
    h = HeadingHold()
    p = Plant(h, sign=-1.0)
    p.run(0.0, 0.0, 2.2)
    p.run(0.0, 0.3, 1.0)
    assert h.state == "disabled" and "相反" in h.disabled
    p.run(0.0, 0.0, 2.0)
    p.run(0.5, 0.0, 1.0, disturb=lambda t: -0.2)
    assert all(v == 0.0 for v in p.sent[-50:])    # permanently open loop, even after stopping


def test_stopped_never_outputs_correction():
    h, p = calibrated()
    p.run(0.0, 0.3, 1.0)
    p.run(0.5, 0.0, 0.5, disturb=lambda t: -0.3)
    assert h.err != 0.0
    out = h.step(0.0, 0.0, (0.2 + BIAS, 0.01), DT)
    assert out == 0.0 and h.err == 0.0            # stop means stop; error is reset


def test_stale_or_missing_imu_falls_back_to_open_loop():
    h, p = calibrated()
    p.run(0.0, 0.3, 1.0)
    assert h.step(0.5, 0.1, None, DT) == 0.1 and h.state == "no_imu"
    assert h.step(0.5, 0.1, (0.0, 0.5), DT) == 0.1                   # stale sample
    assert h.err == 0.0


def test_bias_ignores_pushes_while_stopped():
    h, p = calibrated()
    b = h.bias
    for _ in range(100):
        h.step(0.0, 0.0, (0.25 + BIAS, 0.01), DT)                    # rotating while stopped: pushed by hand
    assert h.bias == pytest.approx(b)


def test_saturated_error_suspends_until_stopped():
    h, p = calibrated()
    p.run(0.0, 0.3, 1.0)
    p.run(0.0, 0.0, 1.2)
    p.stuck = True                                                    # stuck, cannot rotate
    p.run(0.0, 0.3, 4.0)
    assert h.state == "suspended" and p.sent[-1] == pytest.approx(0.3)
    p.stuck = False
    p.run(0.0, 0.0, 0.2)
    assert h.state == "active"


def test_disabled_correction_still_measures():
    """enabled=False: no correction (the open-loop arm of A/B tests), but bias and sign checks still run and measured_rate still works."""
    h, p = calibrated()
    assert h.measured_rate((0.3 + BIAS, 0.01)) is None          # sign unverified: reading not trusted
    p.run(0.0, 0.3, 1.0)
    p.run(0.0, 0.0, 1.2)
    h.enabled = False
    assert h.state == "off"
    kick = lambda t: -0.2 if t < 0.6 else 0.0     # noqa: E731
    p.yaw = 0.0
    p.run(0.5, 0.0, 4.0, disturb=kick)
    assert math.degrees(p.yaw) < -6.0                             # no correction: still drifts
    assert all(v == 0.0 for v in p.sent[-200:])
    assert h.measured_rate((0.25 + BIAS, 0.01)) == pytest.approx(0.25, abs=1e-3)
    assert h.measured_rate((0.25, 0.5)) is None                   # stale sample


def test_measured_rate_none_once_disabled_for_reversed_gyro():
    h = HeadingHold()
    p = Plant(h, sign=-1.0)
    p.run(0.0, 0.0, 2.2)
    p.run(0.0, 0.3, 1.0)
    assert h.disabled and h.measured_rate((0.3, 0.01)) is None
