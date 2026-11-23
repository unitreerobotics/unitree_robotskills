"""Arm kinematics on a real R1-D URDF. The URDF is not part of this repository:

    R1D_DRAW_TEST_URDF=/path/to/r1d.urdf python -m unittest ...

Without that variable these tests are skipped.
"""

import os
import unittest

import numpy as np

from unitree_gpt6.r1d_draw.src.config import normalize_config

URDF = os.environ.get("R1D_DRAW_TEST_URDF")
REST = [0.1, 0.15, 0.29, 0.19, -0.32, -0.22, -0.07, 0.6, -0.15, 0.13, -0.4, 0.1, -0.25, 1.1]


@unittest.skipUnless(URDF, "set R1D_DRAW_TEST_URDF to a local R1-D URDF")
class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from unitree_gpt6.r1d_draw.src import kinematics

        cls.module = kinematics
        tool = [[1, 0, 0, 0.18], [0, 1, 0, 0.02], [0, 0, 1, -0.05], [0, 0, 0, 1]]
        cls.config = normalize_config(
            {
                "device": {"host": "192.168.123.161", "mac": "aa:bb:cc:dd:ee:ff"},
                "model": {"urdf": URDF, "right_tool_transform": tool},
            },
            "/tmp",
        )
        cls.model = kinematics.PinocchioKinematics(cls.config, -0.07)

    def test_ik_returns_to_a_pose_it_was_given_and_keeps_the_other_arm(self):
        target = self.module.transform_pose(self.model.fk(REST, "right"))
        target["x"] += 0.02
        target["z"] -= 0.01
        solved = self.model.solve(target, REST, "right")
        self.assertEqual(solved[:7].tolist(), REST[:7])
        reached = self.model.fk(solved, "right")[:3, 3]
        self.assertLess(np.linalg.norm(reached - [target["x"], target["y"], target["z"]]), 0.003)
        self.model.check_pose(solved, target, "right")
        with self.assertRaises(ValueError):
            self.model.check_pose(REST, target, "right")

    def test_pose_and_transform_are_inverse(self):
        transform = self.model.fk(REST, "right")
        again = self.module.pose_transform(self.module.transform_pose(transform))
        np.testing.assert_allclose(again, transform, atol=1e-9)

    def test_tool_transform_moves_the_tip_and_waist_yaw_turns_the_arms(self):
        plain = normalize_config({"device": self.config["device"], "model": {"urdf": URDF}}, "/tmp")
        wrist = self.module.PinocchioKinematics(plain, -0.07).fk(REST, "right")
        tip = self.model.fk(REST, "right")
        self.assertAlmostEqual(
            np.linalg.norm(tip[:3, 3] - wrist[:3, 3]), np.linalg.norm([0.18, 0.02, -0.05]), places=9
        )
        turned = self.module.PinocchioKinematics(self.config, 0.2).fk(REST, "right")
        self.assertGreater(np.linalg.norm(turned[:3, 3] - tip[:3, 3]), 0.02)
        self.assertAlmostEqual(turned[2, 3], tip[2, 3], places=9)  # yaw keeps the height

    def test_limits_and_unreachable_targets_are_refused(self):
        beyond = list(REST)
        beyond[10] = self.model.upper[10] + 0.01
        with self.assertRaises(ValueError):
            self.model.fk(beyond, "right")
        self.model.fk(beyond, "right", check_limits=False)
        far = self.module.transform_pose(self.model.fk(REST, "right"))
        far["x"] += 2.0
        with self.assertRaises(ValueError):
            self.model.solve(far, REST, "right")
        with self.assertRaises(ValueError):
            self.model.fk(REST, "middle")


if __name__ == "__main__":
    unittest.main()
