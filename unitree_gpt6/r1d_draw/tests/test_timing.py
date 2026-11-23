"""Timing of joint paths: stops, runs at one tip speed, ramps."""

import unittest

import numpy as np

from unitree_gpt6.r1d_draw.src.onboard.controller import MOTORS, checked_route
from unitree_gpt6.r1d_draw.src.timing import constant_speed, rest_to_rest


def line(count, step=0.003):
    """A straight tip path: joint 0 turns 1 rad per metre of tip travel."""
    tips = np.array([[step * n, 0.0, 0.0] for n in range(count + 1)])
    joints = np.zeros((count + 1, 14))
    joints[:, 0] = tips[:, 0]
    return joints, tips


def accepted(start, timed):
    """The onboard controller's own check of the same route."""
    points = [
        {
            "q": {str(15 + n): float(v) for n, v in enumerate(q)},
            "duration_s": float(d),
            "profile": p,
        }
        for q, d, p in timed
    ]
    q35 = [0.0] * MOTORS
    q35[15:29] = [float(v) for v in start]
    return checked_route({"points": points}, q35, [-9.0] * MOTORS, [9.0] * MOTORS)


class RestToRestTests(unittest.TestCase):
    def test_every_point_is_a_stop_at_the_speed_limit(self):
        joints, _ = line(3, 0.1)
        timed = rest_to_rest(joints[0], joints[1:], 0.05)
        self.assertEqual([profile for _, _, profile in timed], ["quintic"] * 3)
        self.assertAlmostEqual(timed[0][1], 1.875 * 0.1 / 0.05)


class ConstantSpeedTests(unittest.TestCase):
    def test_a_straight_stroke_is_one_run_at_the_requested_tip_speed(self):
        joints, tips = line(10)
        timed = constant_speed(joints[0], joints[1:], tips, tip_speed=0.01, joint_speed=0.05)
        profiles = [profile for _, _, profile in timed]
        self.assertEqual(profiles, ["ease_in", *["linear"] * 8, "ease_out"])
        self.assertAlmostEqual(timed[5][1], 0.3)  # 3 mm at 10 mm/s
        self.assertAlmostEqual(timed[0][1], 0.6)  # the ends take twice as long
        accepted(joints[0], timed)

    def test_ramps_shorten_the_slow_ends_without_changing_the_path(self):
        joints, tips = line(10)
        timed = constant_speed(
            joints[0], joints[1:], tips, tip_speed=0.01, joint_speed=0.05, ramp_m=0.001
        )
        self.assertEqual(len(timed), 12)  # one target added at each end
        self.assertEqual([p for _, _, p in timed[:2]], ["ease_in", "linear"])
        self.assertEqual([p for _, _, p in timed[-2:]], ["linear", "ease_out"])
        self.assertAlmostEqual(timed[0][0][0], 0.001)
        self.assertAlmostEqual(timed[0][1], 0.2)
        self.assertAlmostEqual(sum(d for _, d, _ in timed), 0.03 / 0.01 + 0.2)
        accepted(joints[0], timed)

    def test_the_joint_speed_limit_lowers_the_tip_speed(self):
        joints, tips = line(10)
        joints[:, 0] *= 20  # 20 rad per metre: 10 mm/s would need 0.2 rad/s
        timed = constant_speed(joints[0], joints[1:], tips, tip_speed=0.01, joint_speed=0.05)
        self.assertAlmostEqual(timed[5][1], 0.06 / 0.05)
        accepted(joints[0], timed)

    def test_the_tip_stops_at_a_sharp_corner(self):
        joints, tips = line(8)
        joints[5:, 1] = joints[5:, 0] - joints[4, 0]  # from the fifth target on, joint 1 moves
        joints[5:, 0] = joints[4, 0]
        tips[5:] = tips[4] + np.array([[0.0, 0.003 * n, 0.0] for n in range(1, 5)])
        timed = constant_speed(joints[0], joints[1:], tips, tip_speed=0.01, joint_speed=0.05)
        profiles = [profile for _, _, profile in timed]
        self.assertEqual(profiles, ["ease_in", "linear", "linear", "ease_out"] * 2)
        accepted(joints[0], timed)

    def test_a_single_step_and_a_step_without_tip_travel_are_plain_stops(self):
        joints, tips = line(1)
        timed = constant_speed(joints[0], joints[1:], tips, tip_speed=0.01, joint_speed=0.05)
        self.assertEqual([(round(d, 6), p) for _, d, p in timed], [(0.5625, "quintic")])
        tips[1] = tips[0]  # the wrist turns, the tip stays
        timed = constant_speed(joints[0], joints[1:], tips, tip_speed=0.01, joint_speed=0.05)
        self.assertEqual([(round(d, 6), p) for _, d, p in timed], [(0.1125, "quintic")])


if __name__ == "__main__":
    unittest.main()
