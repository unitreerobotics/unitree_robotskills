"""Profiles must be derived on the active kinematic URDF."""
from pathlib import Path
import copy
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from r1d_chess.robot.r1d_config import load_profile, check_kinematic_model, model_sha256, require_calibration


class ProfileModelTests(unittest.TestCase):
    def test_robot_profile_matches_active_model(self):
        p = load_profile(ROOT / 'config/robot.json')
        self.assertEqual(p['kinematic_model_sha256'], model_sha256())
        check_kinematic_model(p)

    def test_model_estimate_profile_passes_the_hardware_gate(self):
        # calibration_verified records provenance only; it must never be required to execute.
        p = load_profile(ROOT / 'config/robot.json')
        self.assertFalse(p['calibration_verified'])
        self.assertEqual(p['geometry_source'], 'model_estimate')
        self.assertIs(require_calibration(p, hardware=True), p)

    def test_old_or_unlabelled_profile_rejected(self):
        p = load_profile(ROOT / 'config/robot.json')
        for value in (None, '0' * 64):
            q = copy.deepcopy(p)
            if value is None:
                q.pop('kinematic_model_sha256')
            else:
                q['kinematic_model_sha256'] = value
            with self.assertRaisesRegex(ValueError, 'kinematic_model_sha256'):
                check_kinematic_model(q)
            with self.assertRaisesRegex(ValueError, 'kinematic_model_sha256'):
                require_calibration(q)


if __name__ == '__main__':
    unittest.main()
