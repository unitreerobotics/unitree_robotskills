"""Service shutdown unloads the arms: zero-gain arm slots, other slots unchanged, refused if another publisher."""
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from r1d_chess.robot.r1d_transport import ARM, InternalDDS


class FakePublisher:
    written = []

    def __init__(self, *a):
        pass

    def Init(self):
        pass

    def Write(self, m):
        FakePublisher.written.append(m)

    def Close(self):
        pass


class UnloadTests(unittest.TestCase):
    def adapter(self, last_command_age):
        d = InternalDDS.__new__(InternalDDS)
        d.lock = threading.RLock()
        cmd = [SimpleNamespace(mode=1, q=.1, dq=0., kp=50., kd=2., tau=1.5) for _ in range(35)]
        d._last_lowcmd = SimpleNamespace(motor_cmd=cmd, mode_machine=0, crc=0)
        d._last_command_at = time.monotonic()-last_command_age
        q = np.full(35, .2)
        d.state = lambda: (q, np.zeros(35), SimpleNamespace(mode_machine=11), 0.)
        return d

    def test_zero_gain_arm_frame(self):
        FakePublisher.written = []
        d = self.adapter(5.)
        with patch('unitree_sdk2py.core.channel.ChannelPublisher', FakePublisher), \
                patch('unitree_sdk2py.utils.crc.CRC', lambda: SimpleNamespace(Crc=lambda m: 123)):
            r = d.unload_arms(frames=3)
        self.assertTrue(r['unloaded'])
        # 1.5 s gain ramp (75 frames at 50 Hz, monotonic to zero), then the zero frames.
        self.assertEqual(r['ramp_s'], 1.5)
        self.assertEqual(len(FakePublisher.written), 75+3)
        kps = [m.motor_cmd[15].kp for m in FakePublisher.written]
        self.assertTrue(all(a >= b for a, b in zip(kps, kps[1:])))
        self.assertTrue(40. < kps[0] < 50.)
        self.assertAlmostEqual(FakePublisher.written[37].motor_cmd[15].tau, 1.5*(1-38/75))
        p = FakePublisher.written[-1]
        for i in ARM:
            self.assertEqual((p.motor_cmd[i].kp, p.motor_cmd[i].kd, p.motor_cmd[i].tau), (0., 0., 0.))
            self.assertAlmostEqual(p.motor_cmd[i].q, .2)
        self.assertEqual(p.motor_cmd[29].kp, 50.)            # head keeps its last command
        self.assertEqual(d._last_lowcmd.motor_cmd[15].kp, 50.)  # original frame not mutated

    def test_refuses_with_active_publisher(self):
        d = self.adapter(0.)
        d._last_command_at = time.monotonic()+10              # a frame "arrives" during the quiet window
        r = d.unload_arms(frames=3)
        self.assertFalse(r['unloaded'])


if __name__ == '__main__':
    unittest.main()
