"""Measured feedback slightly past a URDF limit must not stop execution (seen: left wrist pitch +0.1 mrad)."""
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from r1d_chess.robot.r1d_config import load_profile
from r1d_chess.robot.r1d_kinematics import R1DKinematics


class JointLimitTests(unittest.TestCase):
    def setUp(self):
        self.k = R1DKinematics(load_profile())
        self.lower, self.upper = self.k.limits()
        p = load_profile()
        self.q = np.r_[p['init_joint_rad']['left'], p['init_joint_rad']['right']]
        self.q[5] = self.upper[5]+1e-4                     # left wrist pitch just past its limit

    def test_fk_accepts_measured_feedback_past_limit(self):
        poses = self.k.fk(self.q)
        self.assertEqual(len(poses), 2)

    def test_commands_are_clipped_small_and_rejected_large(self):
        clipped = self.k.clip_joints(self.q)
        self.assertAlmostEqual(clipped[5], self.upper[5])
        far = self.q.copy(); far[5] = self.upper[5]+.05
        with self.assertRaisesRegex(ValueError, 'outside R1-D URDF limits'):
            self.k.clip_joints(far)

    def test_planned_targets_stay_strict(self):
        with self.assertRaisesRegex(ValueError, 'outside R1-D URDF limits'):
            self.k.check_joints(self.q)

    def test_simulated_service_state_with_feedback_past_limit(self):
        import tempfile, json, os
        from r1d_chess.robot.serve import R1DService
        sys.path.insert(0, str(ROOT/'tests'))
        from test_pipeline import synthetic_profile
        f = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False); json.dump(synthetic_profile(), f); f.close()
        os.environ['R1D_CHESS_PROFILE'] = f.name
        try:
            s = R1DService('u', 'u', Path(tempfile.mkdtemp())/'logs', simulate=True)
            s.physical_dds._q[15:29] = self.q
            self.assertIn('right_arm', s.dispatch({'cmd': 'get_state'}))
        finally:
            os.environ.pop('R1D_CHESS_PROFILE', None)


if __name__ == '__main__':
    unittest.main()
