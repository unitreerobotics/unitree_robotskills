"""Offline protocol and failure-path checks; never opens DDS or commands a robot."""

import unittest
import json
from unittest.mock import Mock
from types import SimpleNamespace

from r1_d.sdk import R1D
from r1_d.sdk.client import MOTOR_LINK_LOST, FeedbackUnavailable
from r1_d.sdk.model import GROUPS, HOLD, LIMITS, validate_targets, json_default
from r1_d.sdk.protocol import arm_unload_command, low_command, lift_command, CRC
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import Error_


class ProtocolTests(unittest.TestCase):
    def test_close_is_idempotent_and_cleans_up_after_stop_failure(self):
        robot = object.__new__(R1D)
        robot.closed = False
        robot.relay = Mock()
        robot.writers, robot.readers = {"x": object()}, {"y": object()}
        robot.stop_lift = Mock(side_effect=RuntimeError("connection lost"))
        with self.assertRaises(RuntimeError):
            robot.close()
        robot.relay.close.assert_called_once()
        self.assertEqual(robot.writers, {})
        self.assertEqual(robot.readers, {})
        robot.close()
        robot.stop_lift.assert_called_once()
        with self.assertRaises(RuntimeError):
            robot.publish("rt/cmd_vel", lift_command(0.0))

    def test_feedback_byte_array_serialization(self):
        self.assertEqual(
            json.loads(json.dumps({"wireless_remote": bytes((0, 255))}, default=json_default)),
            {"wireless_remote": [0, 255]},
        )

    def test_motor_scope_crc_and_machine_mode(self):
        state = unitree_hg_msg_dds__LowState_()
        state.mode_machine = 7
        positions = {name: [0.0] * len(ids) for name, ids in GROUPS.items()}
        hold = {i: 0.12 for i in HOLD}
        msg = low_command(state, positions, hold)
        enabled = {i for i, m in enumerate(msg.motor_cmd) if m.mode}
        self.assertEqual(enabled, set(HOLD) | {i for ids in GROUPS.values() for i in ids})
        self.assertEqual(msg.mode_machine, 7)
        self.assertEqual(msg.crc, CRC().Crc(msg))
        for i in HOLD:
            self.assertEqual(msg.motor_cmd[i].q, 0.12)
        for i in (0, 1, 12, 14, 32, 34):
            self.assertEqual(msg.motor_cmd[i].kp, 0.0)

    def test_chassis_always_zero(self):
        for z in (-0.1, 0.0, 0.1):
            msg = lift_command(z)
            self.assertEqual(
                (msg.linear.x, msg.linear.y, msg.angular.x, msg.angular.y, msg.angular.z),
                (0.0,) * 5,
            )
            self.assertEqual(msg.linear.z, z)
        for value in (float("nan"), float("inf"), 0.11, -0.11):
            with self.assertRaises(ValueError):
                lift_command(value)

    def test_arm_unload_has_zero_effort_only_on_arms_and_grippers(self):
        state = unitree_hg_msg_dds__LowState_()
        state.mode_machine = 11
        msg = arm_unload_command(state)
        enabled = {i for i, motor in enumerate(msg.motor_cmd) if motor.mode}
        self.assertEqual(enabled, {i for ids in GROUPS.values() for i in ids})
        self.assertEqual(msg.mode_machine, 11)
        self.assertEqual(msg.mode_pr, 0)
        for motor in msg.motor_cmd:
            self.assertEqual((motor.q, motor.dq, motor.tau, motor.kp, motor.kd), (0.0,) * 5)
        self.assertEqual(msg.crc, CRC().Crc(msg))

    def test_reject_invalid_and_non_arm_targets(self):
        for targets in (
            {"waist": [0.0]},
            {"head": [0.0, 0.0]},
            {"left_arm": [0.0]},
            {"left_gripper": [-0.1]},
            {"right_gripper": [5.5]},
            {"left_gripper": [float("nan")]},
            {"left_arm": [0.0, 0.0, 0.0, 0.0, 0.0, 0.6, 0.0]},
        ):
            with self.assertRaises(ValueError):
                validate_targets(targets)
        self.assertEqual(LIMITS["left_arm"][1][5], 0.5236)

    def test_stale_feedback_rejected(self):
        robot = object.__new__(R1D)
        robot.poll = Mock()
        robot.latest, robot.stamps = {"rt/lowstate": object()}, {"rt/lowstate": 0.0}
        with self.assertRaises(FeedbackUnavailable) as raised:
            robot.fresh("rt/lowstate")
        self.assertEqual(raised.exception.topic, "rt/lowstate")
        self.assertGreater(raised.exception.age, 0.3)
        with self.assertRaises(FeedbackUnavailable) as raised:
            robot.fresh("rt/height_state")
        self.assertIsNone(raised.exception.age)

    def test_lost_motor_link_ends_the_session_unless_that_motor_is_ignored(self):
        robot = object.__new__(R1D)
        state = unitree_hg_msg_dds__LowState_()
        state.motor_state[28].motorstate = MOTOR_LINK_LOST
        robot.fresh = Mock(return_value=state)
        robot.link_lost_since, robot.link_lost_samples = {}, {}
        robot.link_seconds, robot.link_ignored = 0.0, ()
        robot.stamps = {"rt/lowstate": 1.0}
        robot.arm_state()  # first sample of the loss
        robot.stamps = {"rt/lowstate": 2.0}
        with self.assertRaises(RuntimeError):
            robot.arm_state()
        robot.link_ignored = (28, 33)
        robot.stamps = {"rt/lowstate": 3.0}
        self.assertIs(robot.arm_state(), state)
        self.assertEqual(robot.link_lost_samples, {28: 3})
        # Any other motor still ends it.
        state.motor_state[22].motorstate = MOTOR_LINK_LOST
        robot.arm_state()
        robot.stamps = {"rt/lowstate": 4.0}
        with self.assertRaises(RuntimeError):
            robot.arm_state()

    def test_unhomed_or_alarm_blocks_lift(self):
        robot = object.__new__(R1D)
        for source, bits in ((200, 1), (200, 2), (0, 0)):
            robot.fresh = Mock(
                side_effect=[Error_(source=source, state=bits), SimpleNamespace(z=300.0)]
            )
            with self.assertRaises(RuntimeError):
                robot.height()
        robot.fresh = Mock(side_effect=[Error_(source=200, state=0), SimpleNamespace(z=300.0)])
        self.assertEqual(robot.height(), 0.3)


if __name__ == "__main__":
    unittest.main()
