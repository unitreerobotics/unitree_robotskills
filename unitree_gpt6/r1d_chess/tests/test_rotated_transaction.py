"""a target with a different orientation is reached at travel height with the source
orientation and rotated in place above it; a capture first stage rotates back in place there."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from r1d_chess.chess import round_tools as rt

P = lambda x, y, z, yaw: dict(x=x, y=y, z=z, roll=0., pitch=.8, yaw=yaw, gripper='hold')
CFG = dict(travel=.23, gate=.175, staging=[.34, .13], speed=1., open_q=2.3, max_attempts=3,
           max_low_z_adjustments=0, max_low_pitch_adjustments=2, ignore_source_envelope=True,
           physical_actor='left', init_xyz_rpy={'left': [.35, .30, .02, 0, 0, 0], 'right': [.32, -.35, .03, 0, 0, 0]},
           init_joint_rad={'left': [.1]*7, 'right': [-.1]*7})
STATE = {'right_arm': P(.35, .30, .02, 0.), 'left_arm': P(.32, -.35, .03, 0.)}


class RotatedTransactionTests(unittest.TestCase):
    def test_capture_first_stage_rotates_above_tray_and_back(self):
        src, tray = P(.386, .123, .118, -1.5708), P(.249, .278, .163, -.8)
        pts = rt.transaction(STATE, src, tray, Path('/tmp/x'), 'remove_opponent', CFG, end_at_staging=True)['success_plan']['right_arm']
        yaws = [round(p['yaw'], 3) for p in pts]
        # source, close, source high, carry (source yaw) to tray high, rotate, descend, open, rise, rotate back
        self.assertEqual(yaws, [-1.571, -1.571, -1.571, -1.571, -.8, -.8, -.8, -.8, -1.571])
        self.assertEqual([(round(p['x'], 3), round(p['y'], 3)) for p in pts[3:]], [(.249, .278)]*6)
        self.assertTrue(all(p['z'] == .23 for p in (pts[3], pts[4], pts[7], pts[8])))
        self.assertEqual(pts[6]['gripper'], 'open')

    def test_same_orientation_unchanged(self):
        src, dst = P(.386, .123, .118, -1.5708), P(.35, .19, .15, -1.5708)
        pts = rt.transaction(STATE, src, dst, Path('/tmp/x'), 'move', CFG)['success_plan']['right_arm']
        self.assertEqual(len(pts), 12)          # no extra rotation waypoints for an ordinary move
        # above init (init roll/yaw, pitch 0.4), then the joint_q step to the actor's
        # init_joint_rad at the init TCP, then close.
        self.assertEqual((pts[-3]['x'], pts[-3]['y'], round(pts[-3]['z'], 3), pts[-3]['pitch']), (.35, .30, .23, .4))
        self.assertEqual(pts[-2]['joint_q'], [.1]*7)
        self.assertEqual((pts[-2]['x'], pts[-2]['y'], pts[-2]['z']), (.35, .30, .02))
        self.assertEqual(pts[-1]['gripper'], 'close')
        self.assertNotIn('joint_q', pts[-1])


if __name__ == '__main__':
    unittest.main()
