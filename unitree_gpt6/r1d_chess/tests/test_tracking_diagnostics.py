"""Fault evidence must survive session shutdown without sending another frame."""
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from r1d_chess.robot.r1d_transport import ARM, InternalDDS


class TrackingDiagnosticTests(unittest.TestCase):
    def adapter(self):
        adapter = InternalDDS.__new__(InternalDDS)
        adapter.lock = threading.RLock()
        adapter.profile = {'arm_tracking_limit_rad': .15, 'max_motor_temperature_c': 75}
        adapter.session_started_at = time.monotonic()
        adapter.arm_target = np.zeros(14)
        adapter.arm_sent = np.zeros(14)
        adapter.published = 9
        adapter.enabled = True
        adapter.head_waist_sent = None
        adapter._check_reference = lambda state, q, target=None, entry=False: None
        q, dq = np.zeros(35), np.zeros(35)
        q[16] = -.151
        motors = [SimpleNamespace(motorstate=0, temperature=[30, 31]) for _ in range(35)]
        adapter.state = lambda: (q, dq, SimpleNamespace(tick=123, motor_state=motors), .002)
        frames = []
        client = SimpleNamespace(
            state_receiver=SimpleNamespace(latest_state=lambda **kwargs:
                                           SimpleNamespace(control_active=True, runtime_error=None,
                                                           thermal_protection_active=False)),
            send_joint_targets=frames.append)
        return adapter, client, frames

    def test_physical_axis_and_commands_are_preserved_without_publishing(self):
        adapter, client, frames = self.adapter()
        motors = [SimpleNamespace(q=.01, dq=0., kp=50., kd=2., tau=0.) for _ in range(35)]
        adapter._receive_command(SimpleNamespace(motor_cmd=motors))
        with self.assertRaisesRegex(RuntimeError, 'arm tracking error'):
            adapter._publish_once(client, .02)
        adapter.enabled = False
        fault = json.loads(json.dumps(adapter.tracking_fault_snapshot))
        self.assertEqual(frames, [])
        self.assertEqual(fault['worst_motor_index'], 16)
        self.assertEqual(fault['feedback_tick'], 123)
        self.assertEqual(fault['motor_indices'], ARM)
        self.assertAlmostEqual(fault['signed_error_rad'], -.151)
        self.assertEqual(fault['observed_lowcmd']['16']['q'], .01)
        self.assertEqual(fault['published_commands'], 9)
        self.assertEqual(fault['threshold_rad'], .15)

    def test_missing_lowcmd_does_not_mask_tracking_fault(self):
        adapter, client, frames = self.adapter()
        with self.assertRaisesRegex(RuntimeError, 'arm tracking error'):
            adapter._publish_once(client, .02)
        self.assertIsNone(adapter.tracking_fault_snapshot['observed_lowcmd'])
        self.assertIsNone(adapter.tracking_fault_snapshot['lowcmd_age_s'])
        self.assertEqual(frames, [])


if __name__ == '__main__':
    unittest.main()
