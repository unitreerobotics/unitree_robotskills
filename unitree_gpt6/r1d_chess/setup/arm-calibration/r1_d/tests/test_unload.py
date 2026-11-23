"""Offline checks of the explicit R1-D zero-effort tool; no DDS connection."""

import unittest
from unittest.mock import Mock, patch

from r1_d.tools.unload_arms import main, unload
from r1_d.sdk.model import GROUPS
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_


class UnloadTests(unittest.TestCase):
    def setUp(self):
        self.state = unitree_hg_msg_dds__LowState_()
        self.state.mode_machine = 11
        self.robot = Mock()
        self.robot.CheckMode.return_value = (0, {"name": ""})
        self.robot.arm_state.return_value = self.state
        self.robot.sent = {"rt/lowcmd": 3, "rt/cmd_vel": 1}

    def test_cli_defaults_to_read_only(self):
        with patch("sys.argv", ["unload_arms.py"]), patch(
            "r1_d.tools.unload_arms.R1D", return_value=self.robot
        ), patch("builtins.print"):
            main()
        self.robot.publish.assert_not_called()
        self.robot.close.assert_called_once()

    def test_zero_lift_then_three_arm_frames(self):
        report = unload(self.robot, sleep=lambda _: None)
        topics = [call.args[0] for call in self.robot.publish.call_args_list]
        self.assertEqual(topics, ["rt/cmd_vel"] + ["rt/lowcmd"] * 3)
        self.assertEqual(self.robot.publish.call_args_list[0].args[1].linear.z, 0)
        for call in self.robot.publish.call_args_list[1:]:
            enabled = {i for i, m in enumerate(call.args[1].motor_cmd) if m.mode}
            self.assertEqual(enabled, {i for ids in GROUPS.values() for i in ids})
        self.assertEqual(report["sent"]["rt/lowcmd"], 3)

    def test_active_mode_or_motion_refuses_all_publishes(self):
        self.robot.CheckMode.return_value = (0, {"name": "ai"})
        with self.assertRaises(RuntimeError):
            unload(self.robot, sleep=lambda _: None)
        self.robot.CheckMode.return_value = (0, {"name": ""})
        self.state.motor_state[15].dq = 0.2
        with self.assertRaises(RuntimeError):
            unload(self.robot, sleep=lambda _: None)
        self.robot.publish.assert_not_called()

    def test_mode_change_stops_arm_frames(self):
        changed = unitree_hg_msg_dds__LowState_()
        changed.mode_machine = 12
        self.robot.arm_state.side_effect = [self.state, changed]
        with self.assertRaisesRegex(RuntimeError, "mode_machine"):
            unload(self.robot, sleep=lambda _: None)
        self.assertEqual([c.args[0] for c in self.robot.publish.call_args_list], ["rt/cmd_vel"])


if __name__ == "__main__":
    unittest.main()
