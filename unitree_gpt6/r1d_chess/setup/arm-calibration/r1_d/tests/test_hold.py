"""Waist/head feedforward: compensator behaviour and LowCmd encoding."""
import math
import unittest
from types import SimpleNamespace

from r1_d.sdk.hold import HoldCompensator
from r1_d.sdk.model import GROUPS, HOLD
from r1_d.sdk.protocol import HOLD_GAINS, low_command

PARAMS = {"coulomb": 0.6, "viscous": 0.1, "bias": [-0.2, -1.3, 0.0], "ki": 30.0,
          "integral_limit": 0.5, "deadzone": 0.001, "limit": 3.0, "fade_seconds": 0.5}


class CompensatorTests(unittest.TestCase):
    def test_friction_follows_motion_and_fades_at_rest(self):
        c = HoldCompensator(dict(PARAMS, bias=[0.0, 0.0, 0.0], ki=0.0))
        self.assertAlmostEqual(c.torque(0.0, 0.0, 0.2, 0.02), 0.6 + 0.02)
        self.assertAlmostEqual(c.torque(0.0, 0.0, -0.2, 0.02), -0.6 - 0.02)
        for _ in range(30):
            tau = c.torque(0.0, 0.0, 0.0, 0.02)
        self.assertEqual(tau, 0.0)

    def test_bias_is_the_holding_torque_at_the_target(self):
        c = HoldCompensator(dict(PARAMS, ki=0.0))
        self.assertAlmostEqual(c.torque(0.62, 0.62, 0.0, 0.02), -0.2 - 1.3 * math.sin(0.62))

    def test_integral_only_while_stuck_and_resets_past_the_target(self):
        c = HoldCompensator(dict(PARAMS, bias=[0.0, 0.0, 0.0]))
        for _ in range(100):
            c.torque(0.0, 0.004, 0.0, 0.02)
        self.assertGreater(c.integral, 0.2)
        self.assertLessEqual(c.integral, 0.5)
        before = c.integral
        c.torque(0.0, 0.004, 0.0, 0.02, dq=0.02)  # creeping: wait
        self.assertEqual(c.integral, before)
        c.torque(0.006, 0.004, 0.0, 0.02)  # passed the target
        self.assertLessEqual(c.integral, 0.0)

    def test_oscillation_switches_added_stiffness_off(self):
        c = HoldCompensator({"stiffness": 100.0, "limit": 6.0})
        for k in range(60):
            tau = c.torque(0.01, 0.0, 0.0, 0.02, 0.5 * math.sin(2 * math.pi * 6 * k * 0.02))
        self.assertTrue(c.tripped)
        self.assertEqual(tau, 0.0)


class EncodingTests(unittest.TestCase):
    def test_hold_torque_and_gains_reach_the_motors(self):
        state = SimpleNamespace(mode_machine=11)
        targets = {name: [0.0] * len(ids) for name, ids in GROUPS.items()}
        targets["right_arm"][5] = -0.21
        hold = {i: 0.0 for i in HOLD}
        msg = low_command(state, targets, hold, hold_tau={HOLD[1]: -1.0},
                          hold_gains=[(150.0, 6.0), (150.0, 4.0), (40.0, 0.4)])
        self.assertEqual(msg.motor_cmd[HOLD[1]].tau, -1.0)
        self.assertEqual(msg.motor_cmd[HOLD[0]].kp, 150.0)
        self.assertEqual(low_command(state, targets, hold).motor_cmd[HOLD[0]].kp, HOLD_GAINS[0][0])
        with self.assertRaises(ValueError):
            low_command(state, targets, hold, hold_tau={HOLD[2]: 1.0})


if __name__ == "__main__":
    unittest.main()
