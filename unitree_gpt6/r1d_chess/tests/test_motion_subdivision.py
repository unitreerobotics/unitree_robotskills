"""Offline regression for a failed R1-A7-D upward interpolation segment."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from r1d_chess.robot.board_guard import BoardGuard, GuardViolation
from r1d_chess.robot.motion_harness import MotionHarness, waypoint
from r1d_chess.robot.r1d_config import load_profile
from r1d_chess.robot.r1d_kinematics import R1DKinematics


class InterpolationTests(unittest.TestCase):
    def setUp(self):
        profile = load_profile()
        profile.update(simulation_only=True, board_release_z_m=-.005,
                       tray_release_z_m=-.005)
        profile['tcp_xyz_rpy'] = {s: [.05, 0., 0., 0., 0., 0.] for s in ('left', 'right')}
        profile['scene'] = dict(surface_z=-.04, uncertainty=.008, finger_end=.15,
                                half_width=.03, half_thickness=.01)
        self.guard = BoardGuard.from_dict(profile['scene'])
        self.motion = MotionHarness.__new__(MotionHarness)
        self.motion.robot = R1DKinematics(profile)
        self.motion.base = np.eye(4)
        self.q = np.array([
            .13550965120032665, -.026531991956175446, -.045016377370521214,
            -.5315515803966598, -.019849262640221146, .4425896692291576,
            -.0798799308029715, -.11934076994657516, -.6347172260284424,
            -.43399372696876526, .48022720217704773, .5574447512626648,
            -.04513724893331528, .4878111779689789,
        ])
        self.target = np.array([
            [.994758101964024, .09363726182685785, .04108992302865471, .23954611],
            [-.09555534388979293, .9942900668300358, .0475019921395523, .12730894],
            [-.03640734583893352, -.05117935326565926, .9980256404362932, .07],
            [0., 0., 0., 1.],
        ])
        self.plan = {'left_arm': [waypoint(self.target)],
                     'right_arm': [waypoint(self.motion.fk(self.q)[1])],
                     'speed_scale': .25, 'skip_preflight': False}

    def test_recorded_upward_segment_reaches_same_target(self):
        steps = self.motion.prepare(self.plan, self.q, self.guard)
        samples = steps[0]['samples']
        last = self.motion.fk(samples[-1][0])[0]
        position, angle = self.motion.errors([last], [self.target])[0]
        self.assertLess(position, 1e-4)
        self.assertLess(angle, 1e-3)
        # A bounded solver fallback may now converge without subdivision.
        self.assertGreaterEqual(len(samples), 1)
        previous = self.q
        for q, duration in samples:
            self.assertGreater(duration, 0.)
            self.assertLessEqual(float(np.max(np.abs(q - previous))), .3)
            for beta in np.linspace(0, 1, 5):
                for pose in self.motion.fk(previous + beta * (q - previous)):
                    self.assertGreaterEqual(self.guard.clearance(pose), self.guard.minimum_clearance)
            previous = q

    def test_unreachable_ik_stops_at_bounded_subdivision_depth(self):
        with patch.object(self.motion, 'solve', side_effect=ValueError('unreachable')) as solve:
            with self.assertRaisesRegex(ValueError, 'waypoint 0.*unreachable'):
                self.motion.prepare(self.plan, self.q, self.guard)
            self.assertEqual(solve.call_count, 6)

    def test_surface_collision_still_rejected_before_solving(self):
        # only the actor (logical right) is checked.
        self.plan['right_arm'][0]['z'] = -.05
        with patch.object(self.motion, 'solve') as solve:
            with self.assertRaisesRegex(GuardViolation, 'waypoint 0.*touch surface'):
                self.motion.prepare(self.plan, self.q, self.guard)
            solve.assert_not_called()

    def test_observer_is_not_checked(self):
        from r1d_chess.robot import motion_harness
        self.assertFalse(motion_harness.CHECK_OBSERVER)
        self.plan['left_arm'][0]['z'] = -.05
        with patch.object(self.motion, 'solve', side_effect=ValueError('reached IK')):
            with self.assertRaisesRegex(ValueError, 'reached IK'):
                self.motion.prepare(self.plan, self.q, self.guard)


if __name__ == '__main__':
    unittest.main()
