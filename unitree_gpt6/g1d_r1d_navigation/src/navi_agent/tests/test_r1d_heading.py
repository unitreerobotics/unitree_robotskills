# -*- coding: utf-8 -*-
"""Heading-hold wiring in services/robots/r1d.py: correction goes into the published yaw component, none when stopped, odometry integrates the command.

Channel injection (channels=(write, subscribe)) bypasses the SDK; the three feedback streams are fed by the test."""
from types import SimpleNamespace

import pytest

from services.robots.profile import build_profile
from services.robots.r1d import R1DRobot


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


def make_robot():
    clock = Clock()
    writes, subs = [], {}
    robot = R1DRobot(build_profile("r1d"), channels=(lambda f, lift, t: writes.append((f, t)) or True,
                                                     lambda topic, cb: subs.__setitem__(topic, cb)),
                     now_fn=clock)

    def feedback():
        subs["rt/state_vel"](SimpleNamespace(linear=SimpleNamespace(x=0.0, z=0.0), angular=SimpleNamespace(z=0.0)))
        subs["rt/height_state"](SimpleNamespace(z=354.0))
        subs["rt/height_alarm"](SimpleNamespace(source=200, state=0))
    return robot, clock, writes, feedback


def ticks(robot, clock, feedback, n, cmd=None):
    for _ in range(n):
        feedback()
        if cmd is not None:
            robot.send_command(cmd[0], 0.0, cmd[1])
        robot._tick()
        clock.t += robot.dt


def test_heading_hold_corrects_published_turn_but_not_odometry():
    robot, clock, writes, feedback = make_robot()
    gyro = {"rate": 0.012}                         # bias only
    robot.attach_yaw_rate(lambda: (gyro["rate"], 0.01))
    ticks(robot, clock, feedback, 120, cmd=(0.0, 0.0))            # 2.4s at rest: estimate bias
    assert robot.health()["heading_hold"]["state"] == "verifying"
    gyro["rate"] = 0.3 + 0.012                     # gyro agrees with turn: sign check passes
    ticks(robot, clock, feedback, 30, cmd=(0.0, 0.3))
    assert robot.heading.verified
    ticks(robot, clock, feedback, 60, cmd=(0.0, 0.0))
    gyro["rate"] = -0.2 + 0.012                    # nose drifts right while driving straight
    odom_yaw = robot.odom["yaw"]
    ticks(robot, clock, feedback, 25, cmd=(0.5, 0.0))
    f, turn = writes[-1]
    assert f == pytest.approx(0.5)                 # forward component untouched
    vyaw_sent = turn * robot.rps_full / robot.turn_sign
    assert vyaw_sent > 0.05                        # corrects back to the left
    assert robot.odom["yaw"] == pytest.approx(odom_yaw)          # odometry integrates the command (vyaw=0)
    assert robot.health()["heading_hold"]["corr_rps"] > 0


def test_stop_publishes_pure_zero_even_with_heading_error():
    robot, clock, writes, feedback = make_robot()
    robot.attach_yaw_rate(lambda: (0.25, 0.01))   # gyro keeps reporting rotation
    robot.heading.verified, robot.heading.bias, robot.heading.bias_s = True, 0.0, 5.0
    ticks(robot, clock, feedback, 10, cmd=(0.5, 0.0))
    assert writes[-1][1] != 0.0                    # moving: correcting
    ticks(robot, clock, feedback, 5, cmd=(0.0, 0.0))
    assert writes[-1] == (0.0, 0.0)                # stopped: zero correction


def test_without_imu_robot_is_open_loop():
    robot, clock, writes, feedback = make_robot()
    ticks(robot, clock, feedback, 5, cmd=(0.5, 0.1))
    assert writes[-1][1] == pytest.approx(robot.turn_sign * 0.1 / robot.rps_full)
    assert "heading_hold" not in robot.health()


def test_measured_rate_and_yaw_totals_track_command_vs_gyro():
    robot, clock, writes, feedback = make_robot()
    assert robot.measured_yaw_rate() is None and robot.yaw_totals() is None   # no gyro attached
    gyro = {"rate": 0.012}
    robot.attach_yaw_rate(lambda: (gyro["rate"], 0.01))
    ticks(robot, clock, feedback, 120, cmd=(0.0, 0.0))
    assert robot.measured_yaw_rate() is None                      # sign unverified: not a measurement
    gyro["rate"] = 0.3 + 0.012
    ticks(robot, clock, feedback, 30, cmd=(0.0, 0.3))             # gyro untrusted during sign check: recorded as a gap
    assert robot.heading.verified and robot.yaw_totals()["gaps"] > 0
    gyro["rate"] = 0.012
    ticks(robot, clock, feedback, 60, cmd=(0.0, 0.0))
    assert robot.measured_yaw_rate() == pytest.approx(0.0, abs=1e-3)          # bias removed
    before = dict(robot.yaw_totals())
    gyro["rate"] = -0.1 + 0.012                                   # nose drifts right at 0.1 rad/s while driving straight
    ticks(robot, clock, feedback, 50, cmd=(0.5, 0.0))             # 1s
    after = robot.yaw_totals()
    assert after["gaps"] == before["gaps"]
    assert after["cmd"] - before["cmd"] == pytest.approx(0.0, abs=1e-6)       # command requested no rotation
    assert after["gyro"] - before["gyro"] == pytest.approx(-0.1, abs=0.01)    # gyro sees ~-5.7° drift
