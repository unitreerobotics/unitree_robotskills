"""Simulated service: retract replays the executed trace in reverse, clears a latched fault, closes the actor gripper."""
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'tests')]


class RetractTests(unittest.TestCase):
    def setUp(self):
        from test_pipeline import synthetic_profile
        f = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False)
        json.dump(synthetic_profile(), f); f.close()
        os.environ['R1D_CHESS_PROFILE'] = f.name
        from r1d_chess.robot.serve import R1DService
        self.s = R1DService('u', 'u', Path(tempfile.mkdtemp())/'logs', simulate=True)
        self.s.RETRACT_SLOWDOWN = .05

    def tearDown(self):
        os.environ.pop('R1D_CHESS_PROFILE', None)

    def test_no_trace_is_an_error(self):
        with self.assertRaisesRegex(ValueError, 'no retract path'):
            self.s.dispatch({'cmd': 'retract'})

    def test_replays_reverse_and_clears_fault(self):
        dds = self.s.physical_dds
        q0 = np.zeros(14); q1 = q0+.05; q2 = q0+.1
        dds.enable(); dds.command_arm(q2, np.zeros(14)); dds.state(); dds.release()
        dds.fault = 'arm target outside R1-D URDF limits'
        dds.gripper_target[:] = [.3, .7]
        self.s.retract_trace = [(q0, 0.), (q1, .1), (q2, .1)]
        r = self.s.retract()
        self.assertTrue(r['retracted'], r)
        self.assertIsNone(dds.fault)
        np.testing.assert_allclose(dds.arm_target, q0, atol=1e-9)
        # Actor (right) gripper closed during the return; the observer gripper untouched.
        np.testing.assert_allclose(dds.gripper_target, [.3, 0.])
        self.assertTrue(r['gripper_closed'])
        self.assertFalse(dds.enabled)
        self.assertIsNone(self.s.retract_trace)
        # arms are unloaded (session ended) after a successful retract.
        self.assertTrue(r['arm_unload']['unloaded'], r)
        self.assertFalse(dds.session_active)

    def test_no_unload_after_fault_outcome(self):
        self.assertNotIn('stopped_after_fault', self.s.UNLOAD_AFTER_OUTCOMES)
        self.assertNotIn('sequence_stopped', self.s.UNLOAD_AFTER_OUTCOMES)
        dds = self.s.physical_dds
        dds.enable(); dds.release(); dds.fault = 'tracking'
        with self.assertRaisesRegex(RuntimeError, 'latched fault'):
            self.s.unload_arms_now('test')


if __name__ == '__main__':
    unittest.main()
