"""Regression against a recorded bounded IK stall; no robot connection."""
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from r1d_chess.robot.r1d_config import MODEL_PATH
from r1d_chess.robot.r1d_urdf import Arm


class BoundedFallbackTests(unittest.TestCase):
    def setUp(self):
        self.arm = Arm(ET.parse(MODEL_PATH).getroot(), 'left')
        self.record = json.loads((Path(__file__).parent / 'fixtures/horse7_ik_stall.json').read_text())

    def test_recorded_stall_reaches_original_pose_within_joint_limits(self):
        target = np.array(self.record['target'])
        seed = np.array(self.record['seed'])
        result = self.arm.solve(target, seed)
        self.assertTrue(result['accepted'])
        q = np.array(result['q_rad'])
        error = self.arm.error(q, target)
        self.assertLess(np.linalg.norm(error[:3]), 1e-4)
        self.assertLess(np.linalg.norm(error[3:]), 1e-3)
        self.assertTrue(np.all(q >= self.arm.lower))
        self.assertTrue(np.all(q <= self.arm.upper))
        self.assertLessEqual(result['evaluations'], 800)

    def test_unreachable_target_remains_rejected(self):
        target = np.array(self.record['target'])
        target[0, 3] += 5.
        result = self.arm.solve(target, self.record['seed'])
        self.assertFalse(result['accepted'])
        self.assertLessEqual(result['evaluations'], 800)


if __name__ == '__main__':
    unittest.main()
