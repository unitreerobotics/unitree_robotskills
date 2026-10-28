import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

"""Numeric regression for removing unitree_deploy; synthetic states, no DDS."""
import json
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
from g1d_chess.robot.motion_harness import MotionHarness


class G1ModelTests(unittest.TestCase):
    def test_preserves_fk_gravity_and_joint_order(self):
        reference = json.loads((ROOT / 'tests/g1_model_synthetic_reference.json').read_text())
        harness = MotionHarness()
        self.assertEqual(list(harness.model.names)[1:], reference['joint_names'])
        for sample in reference['samples']:
            with self.subTest(q=sample['q']):
                q = np.array(sample['q'])
                np.testing.assert_allclose(harness.fk(q), sample['fk'], atol=1e-10, rtol=1e-10)
                np.testing.assert_allclose(harness.tau(q), sample['tau'], atol=1e-10, rtol=1e-10)

    def test_ik_recovers_synthetic_target(self):
        harness = MotionHarness()
        q = np.array([0.2, 0.1, -0.15, -0.4, 0.1, 0.2, 0.1,
                      0.2, -0.1, 0.15, -0.4, -0.1, 0.2, -0.1])
        targets = harness.fk(q)
        solved = harness.solve(targets, q + 0.01)
        for position, rotation in harness.errors(harness.fk(solved), targets):
            self.assertLess(position, 1e-4)
            self.assertLess(rotation, 1e-3)
