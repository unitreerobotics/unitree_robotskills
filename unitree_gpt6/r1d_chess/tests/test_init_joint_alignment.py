"""a joint_q waypoint compiles to a joint-space segment that ends exactly on those joints."""
from pathlib import Path
import json, sys, unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
import numpy as np
from r1d_chess.robot.motion_harness import MotionHarness, waypoint


class _Guard:
    def check_segment(self, a, b): pass
    def check_gripper(self, a): pass


class JointAlignmentTests(unittest.TestCase):
    def test_ends_on_joint_q(self):
        prof = json.loads((ROOT/'config/robot.json').read_text())
        m = MotionHarness()
        init = np.r_[prof['init_joint_rad']['left'], prof['init_joint_rad']['right']]
        start = init.copy(); start[7:] += .15            # same arm, different joint config
        poses = m.fk(start)
        plan = {'left_arm': [waypoint(poses[0])],
                'right_arm': [dict(waypoint(poses[1]), joint_q=list(init[7:]))], 'speed_scale': 1.}
        steps = m.prepare(plan, start, _Guard())
        end = steps[-1]['samples'][-1][0]
        np.testing.assert_allclose(end, np.r_[start[:7], init[7:]], atol=1e-12)
        self.assertTrue(all(np.max(np.abs(b[0]-a[0])) <= .0501
                            for a, b in zip(steps[0]['samples'], steps[0]['samples'][1:])))

    def test_within_tolerance_holds(self):
        m = MotionHarness(); q = np.zeros(14); q[7] = .3; poses = m.fk(q)
        target = list(q[7:]); target[0] += .08
        plan = {'left_arm': [waypoint(poses[0])], 'right_arm': [dict(waypoint(poses[1]), joint_q=target)]}
        steps = m.prepare(plan, q, _Guard())
        np.testing.assert_allclose(steps[-1]['samples'][-1][0], q)

    def test_rejects_bad_joint_q(self):
        m = MotionHarness(); q = np.zeros(14); poses = m.fk(q)
        plan = {'left_arm': [waypoint(poses[0])], 'right_arm': [dict(waypoint(poses[1]), joint_q=[0.]*6)]}
        with self.assertRaises(ValueError):
            m.prepare(plan, q, _Guard())


if __name__ == '__main__':
    unittest.main()
