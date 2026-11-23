"""spec 位姿是 TCP：夹持点（离法兰 137 mm）换算、按 pitch 的抓放高度与 round build 校验。"""
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from r1d_chess.chess import grip_pose as g


class GripPoseTests(unittest.TestCase):
    def setUp(self):
        self.board = g.load_board()

    def test_round_trip(self):
        for pitch in (.35, .8):
            grip = self.board['intersections_xy_m']['c5'] + [.019]
            tcp = g.tcp_from_grip(grip, pitch, -math.pi/2)
            back = g.grip_point(tcp, [0, pitch, -math.pi/2])
            for a, b in zip(back, grip):
                self.assertAlmostEqual(a, b, places=12)

    def test_release_height_keeps_calibrated_fingertip(self):
        from r1d_chess.robot.r1d_config import load_profile
        prof = load_profile()
        fe = prof['scene']['finger_end']
        # Board: TCP z 0.150 m for every move and any pitch; tray: profile value at its pitch.
        for pitch in (.35, .6, .8, 1.0):
            self.assertEqual(g.release_tcp_z(prof, pitch), .15)
        self.assertAlmostEqual(g.release_tcp_z(prof, prof['release_pitch_rad'], 'tray'), prof['tray_release_z_m'], places=12)
        # Grasp: fingertip 3 mm above the board surface.
        self.assertAlmostEqual(g.source_tcp_z(prof, .8) - fe*math.sin(.8), prof['scene']['surface_z'] + .003, places=12)

    def test_tcp_on_intersection_is_rejected(self):
        pose = {'xy_m': self.board['intersections_xy_m']['c5'], 'z_m': .1177, 'rpy_rad': [0, .8, -math.pi/2]}
        with self.assertRaisesRegex(ValueError, 'TCP 本身就在该交叉点上'):
            g.check_pose(pose, 'c5', self.board, label='source')

    def test_converted_pose_passes(self):
        tcp = g.tcp_from_grip(self.board['intersections_xy_m']['d7'] + [.03], .8, -math.pi/2)
        pose = {'xy_m': tcp[:2], 'z_m': tcp[2], 'rpy_rad': [0, .8, -math.pi/2]}
        result = g.check_pose(pose, 'd7', self.board, label='target')
        self.assertLess(result['grip_error_m'], 1e-6)
        self.assertEqual(result['nearest_to_grip'], 'd7')

    def test_finger_direction_is_tcp_x_axis(self):
        import numpy as np
        sys.path.insert(0, str(ROOT/'src'))
        from r1d_chess.robot.motion_harness import matrix
        for pitch, yaw in ((.35, -math.pi/2), (.2, .3), (.5, math.pi/2)):
            m = matrix({'x': 0, 'y': 0, 'z': 0, 'roll': .1, 'pitch': pitch, 'yaw': yaw})
            np.testing.assert_allclose(m[:3, 0], g.finger_direction(pitch, yaw), atol=1e-12)


if __name__ == '__main__':
    unittest.main()
