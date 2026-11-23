"""Primary IK = analytic-Jacobian damped least squares, scipy only as fallback."""
from pathlib import Path
import sys
import time
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from r1d_chess.robot.r1d_kinematics import R1DKinematics


class DLSIKTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.arms = R1DKinematics().arms
        # Recorded planning pose of the physical left (actor) and right (observer) arms.
        cls.seeds = [np.array([-0.3659, 0.5493, 0.1022, 0.5218, -0.5236, -0.0472, -0.2554]),
                     np.array([-0.2646, -0.5965, -0.3520, 0.5601, 0.4565, -0.0705, 0.3000])]
        for arm, seed in zip(cls.arms, cls.seeds):
            assert np.all(seed >= arm.lower) and np.all(seed <= arm.upper)

    def test_frames_match_fk(self):
        rng = np.random.default_rng(0)
        for arm in self.arms:
            for _ in range(20):
                q = rng.uniform(arm.lower, arm.upper)
                np.testing.assert_allclose(arm.frames(q)[0], arm.fk(q), atol=1e-12)

    def test_stationary_returns_seed_without_iterating(self):
        for arm, seed in zip(self.arms, self.seeds):
            r = arm.solve(arm.fk(seed), seed)
            self.assertTrue(r['accepted'])
            self.assertEqual((r['method'], r['evaluations']), ('dls', 0))
            np.testing.assert_allclose(r['q_rad'], seed, atol=0)

    def test_small_steps_accepted_near_seed_and_within_limits(self):
        rng = np.random.default_rng(1)
        for arm, seed in zip(self.arms, self.seeds):
            start = arm.fk(seed)
            for _ in range(100):
                target = start.copy()
                d = rng.normal(size=3)
                target[:3, 3] += .008*d/np.linalg.norm(d)        # one preflight sample step
                r = arm.solve(target, seed)
                self.assertTrue(r['accepted'], r)
                self.assertEqual(r['method'], 'dls')
                q = np.asarray(r['q_rad'])
                self.assertTrue(np.all(q >= arm.lower) and np.all(q <= arm.upper))
                self.assertLess(r['max_delta_from_seed_rad'], .3)   # preflight branch-jump limit
                self.assertLess(r['position_error_m'], 1e-6)
                self.assertLess(r['rotation_error_rad'], 1e-5)

    def test_unreachable_still_rejected(self):
        for arm, seed in zip(self.arms, self.seeds):
            target = arm.fk(seed)
            target[2, 3] += 1.5
            self.assertFalse(arm.solve(target, seed)['accepted'])

    def test_faster_than_scipy_budget(self):
        arm, seed = self.arms[0], self.seeds[0]
        target = arm.fk(seed)
        target[0, 3] += .008
        started = time.perf_counter()
        for _ in range(50):
            arm.solve(target, seed)
        self.assertLess((time.perf_counter()-started)/50, .01)   # scipy needed ~50 ms here


if __name__ == '__main__':
    unittest.main()
