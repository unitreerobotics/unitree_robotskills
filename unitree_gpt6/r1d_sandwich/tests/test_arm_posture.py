from __future__ import annotations

import sys
import unittest
from pathlib import Path

SKILL_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SKILL_ROOT / "scripts"))

import arm_posture  # noqa: E402


class PostureThresholdTest(unittest.TestCase):
    def test_natural_pose(self):
        self.assertFalse(arm_posture.is_ugly(45.0, 0.0, 2.5))

    def test_abduction_above_threshold(self):
        self.assertTrue(arm_posture.is_ugly(45.1, 0.1, 0.0))

    def test_elbow_above_shoulder(self):
        self.assertTrue(arm_posture.is_ugly(10.0, -0.001, 0.0))

    def test_shoulder_yaw_load_above_threshold(self):
        self.assertTrue(arm_posture.is_ugly(10.0, 0.1, -2.51))


if __name__ == "__main__":
    unittest.main()
