"""Planned poses keep the uncertainty margin (>= 0); measured poses only stop at the surface estimate."""
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from r1d_chess.robot.board_guard import BoardGuard, GuardViolation


def flat(z):
    T = np.eye(4)
    T[2, 3] = z
    return T


class FloorTests(unittest.TestCase):
    def setUp(self):
        self.g = BoardGuard.from_dict({'surface_z': 0., 'uncertainty': .004, 'finger_end': .14, 'half_width': .05,
                                       'half_thickness': .005, 'travel_clearance': .04, 'minimum_clearance': 0.})

    def test_planned_floor_is_zero(self):
        self.g.check_gripper(flat(.0091))                   # clearance +0.1 mm
        with self.assertRaises(GuardViolation):
            self.g.check_gripper(flat(.0089))               # clearance -0.1 mm

    def test_measured_floor_is_minus_uncertainty(self):
        self.g.check_gripper(flat(.0060), measured=True)    # clearance -3 mm: tracking error, allowed
        with self.assertRaises(GuardViolation):
            self.g.check_gripper(flat(.0049), measured=True)  # envelope below the surface estimate
        start = desired = flat(.03)
        self.g.check_feedback(start, flat(.02), flat(.0060))
        with self.assertRaises(GuardViolation):
            self.g.check_feedback(start, flat(.02), flat(.0049))

    def test_negative_minimum_rejected(self):
        with self.assertRaises(GuardViolation):
            BoardGuard.from_dict({'surface_z': 0., 'uncertainty': .004, 'finger_end': .14, 'half_width': .05,
                                  'half_thickness': .005, 'minimum_clearance': -.001})


class TwoLevelTests(unittest.TestCase):
    SCENE = {'surface_z': .012, 'uncertainty': .004, 'finger_end': .14, 'half_width': .05, 'half_thickness': .005,
             'travel_clearance': .04, 'minimum_clearance': 0., 'table_z': -.008, 'board_centre_xy': [.3, 0.],
             'board_yaw': 0., 'board_half_extent': [.18, .17]}

    def test_board_top_over_board_and_table_beside_it(self):
        g = BoardGuard.from_dict(self.SCENE)
        over = flat(.03); over[:3, 3] = [.25, 0., .03]
        beside = flat(.03); beside[:3, 3] = [.30, .35, .03]
        self.assertAlmostEqual(g.surface_under(g.corners(over)), .012)
        self.assertAlmostEqual(g.surface_under(g.corners(beside)), -.008)
        self.assertAlmostEqual(g.clearance(beside)-g.clearance(over), .020, places=6)

    def test_box_reaching_over_the_edge_uses_board_top(self):
        g = BoardGuard.from_dict(self.SCENE)
        T = flat(.03); T[:3, 3] = [.30, .17+.05-.01, .03]      # box edge 1 cm inside the footprint
        self.assertAlmostEqual(g.surface_under(g.corners(T)), .012)
        T[1, 3] = .17+.05+.01                                     # box edge 1 cm outside: table
        self.assertAlmostEqual(g.surface_under(g.corners(T)), -.008)

    def test_legacy_single_plane_and_partial_fields(self):
        legacy = {k: v for k, v in self.SCENE.items() if k not in ('table_z', 'board_centre_xy', 'board_yaw', 'board_half_extent')}
        g = BoardGuard.from_dict(legacy)
        T = flat(.03); T[:3, 3] = [.30, .35, .03]
        self.assertAlmostEqual(g.surface_under(g.corners(T)), .012)
        with self.assertRaises(GuardViolation):
            BoardGuard.from_dict(dict(legacy, table_z=-.008))
        with self.assertRaises(GuardViolation):
            BoardGuard.from_dict(dict(self.SCENE, table_z=.02))       # table above the board top


if __name__ == '__main__':
    unittest.main()
