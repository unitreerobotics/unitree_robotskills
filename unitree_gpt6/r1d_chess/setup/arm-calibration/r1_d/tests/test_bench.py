"""Deterministic off-robot tests; fake clock and motors, no DDS participant."""

import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
from copy import deepcopy

import r1_d.sdk.model as model
from r1_d.sdk.model import GROUPS, HOLD, model_info
from r1_d.sdk import ControlSession
from r1_d.sdk.client import FeedbackUnavailable
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from r1_d.tools.commands import execute_command, plan_command, width_to_motor
from r1_d.sdk.motion_mode import prepare


class FakeRobot:
    def __init__(self):
        self.state = unitree_hg_msg_dds__LowState_()
        self.state.mode_machine = 11
        self.state.motor_state[18].q = self.state.motor_state[25].q = 1.47
        self.state.motor_state[31].q = self.state.motor_state[33].q = 0.5
        self.z, self.speed, self.now = 0.33, 0.0, 0.0
        self.low_count = self.lift_count = self.stops = 0
        self.last_low = None

    def arm_state(self):
        return self.state

    def measured_targets(self, state):
        return {k: [state.motor_state[i].q for i in ids] for k, ids in GROUPS.items()}

    def feedback_status(self):
        return {}

    def height(self):
        return self.z

    def publish(self, topic, message):
        if topic == "rt/lowcmd":
            self.low_count += 1
            self.last_low = message
            for i, m in enumerate(message.motor_cmd):
                if m.mode:
                    self.state.motor_state[i].q = m.q
        elif topic == "rt/cmd_vel":
            assert (
                message.linear.x,
                message.linear.y,
                message.angular.x,
                message.angular.y,
                message.angular.z,
            ) == (0.0,) * 5
            self.speed = message.linear.z
            self.lift_count += 1

    def stop_lift(self):
        self.speed = 0.0
        self.stops += 1

    def advance(self, dt=0.02):
        self.now += dt
        self.z += self.speed * 0.08 * dt


class BenchTests(unittest.TestCase):
    def setUp(self):
        self.robot = FakeRobot()
        self.session = ControlSession(self.robot, clock=lambda: self.robot.now)
        self.base = deepcopy(self.session.baseline)

    def command(self, line):
        execute_command(self.session, line)

    def finish(self, timeout=120):
        deadline = self.robot.now + timeout
        while self.session.motion is not None:
            self.robot.advance()
            self.session.tick()
            self.assertLess(self.robot.now, deadline)

    def test_relative_60_is_not_cumulative_and_preserves_other_targets(self):
        target = plan_command("upper both 60", self.base, self.base, 0.33)
        self.assertAlmostEqual(target["q"]["left_arm"][0], -math.pi / 3)
        second = plan_command("upper both 60", target, self.base, 0.33)
        self.assertEqual(second, target)
        forearm = plan_command("forearm left 60", target, self.base, 0.33)
        self.assertAlmostEqual(forearm["q"]["left_arm"][3], 1.47 - math.pi / 3)
        self.assertEqual(forearm["q"]["left_arm"][0], target["q"]["left_arm"][0])
        self.assertEqual(forearm["q"]["right_arm"], target["q"]["right_arm"])
        absolute = plan_command("joint right elbow 60", self.base, self.base, 0.33)
        self.assertAlmostEqual(absolute["q"]["right_arm"][3], math.pi / 3)

    def test_limits_and_invalid_inputs_never_clip_silently(self):
        base = deepcopy(self.base)
        base["q"]["left_arm"][3] = 0.0
        for command in (
            "forearm left 60",
            "upper both 61",
            "grip both 101",
            "grip both nan",
            "grip-mm both 20",
            "down 500",
            "up -1",
            "height inf",
            "upper waist 30",
        ):
            with self.assertRaises(ValueError, msg=command):
                plan_command(command, base, base, 0.33)

    def test_gripper_units_require_real_calibration(self):
        q = plan_command("grip both 50", self.base, self.base, 0.33)["q"]
        self.assertEqual(q["left_gripper"], [2.7])
        # Synthetic test fixture, not a calibration for a real robot.
        cal = {"left": [[0.0, 1.0], [2.0, 20.0], [5.4, 70.0]]}
        self.assertAlmostEqual(width_to_motor(cal, "left", 45.0), 3.7)
        for value in (-1, 71, float("nan")):
            with self.assertRaises(ValueError):
                width_to_motor(cal, "left", value)
        with self.assertRaises(ValueError):
            width_to_motor({"left": [[0, 10], [1, 5]]}, "left", 8)

    def test_hold_keeps_command_instead_of_adopting_sag(self):
        self.session.tick()
        count = self.robot.low_count
        self.robot.state.motor_state[18].q -= 0.02
        for _ in range(20):
            self.robot.advance()
            self.session.tick()
        self.assertEqual(self.robot.low_count - count, 20)
        self.assertEqual(self.robot.last_low.motor_cmd[18].q, 1.47)

    def test_complete_workflow_and_direct_home(self):
        for cmd in ("1", "2", "grip both 40", "up 20", "down 10"):
            self.command(cmd)
            self.finish()
        self.assertAlmostEqual(self.robot.z, 0.34, delta=0.004)
        self.command("0")
        motion = deepcopy(self.session.motion)
        self.command("home")  # Repeated home keeps the running motion.
        self.assertEqual(motion, self.session.motion)
        self.assertEqual(motion["to"], self.base)  # one move, straight to the target
        self.finish(timeout=180)
        self.assertIsNone(self.session.motion)
        self.assertEqual(self.session.commanded["q"], self.base["q"])
        self.assertAlmostEqual(self.robot.z, 0.33, delta=0.002)
        self.assertEqual(self.robot.speed, 0.0)
        for i in HOLD:
            self.assertEqual(self.robot.last_low.motor_cmd[i].q, self.session.hold[i])
        self.assertGreater(self.robot.low_count, 100)

    def test_home_ends_at_saved_pose(self):
        saved = deepcopy(self.base)
        saved["q"]["left_arm"][3] -= 0.2
        saved["q"]["right_arm"][3] -= 0.2
        saved["q"]["left_gripper"] = [4.0]
        saved["height"] = self.base["height"] + 0.01
        session = ControlSession(self.robot, clock=lambda: self.robot.now, home_pose=saved)
        self.session = session
        self.command("upper both 10")
        self.finish()
        self.command("home")
        self.assertEqual(session.motion["to"], saved)  # no retrace through the takeover pose
        self.finish(timeout=180)
        self.assertEqual(session.commanded["q"], saved["q"])
        self.assertAlmostEqual(self.robot.z, saved["height"], delta=0.002)

    def test_stop_and_resume_home_from_partial_motion(self):
        self.command("upper both 60")
        for _ in range(50):
            self.robot.advance()
            self.session.tick()
        self.command("stop")
        stopped = deepcopy(self.session.commanded)
        for _ in range(20):
            self.robot.advance()
            self.session.tick()
        self.assertEqual(self.session.commanded, stopped)
        self.command("home")
        self.robot.advance()
        self.session.tick()
        self.command("stop")
        self.command("home")
        self.finish()
        self.assertEqual(self.session.commanded["q"], self.base["q"])

    def test_busy_invalid_command_preserves_trajectory(self):
        self.command("1")
        original = deepcopy(self.session.motion)
        with self.assertRaises(ValueError):
            self.command("grip both 50")
        self.assertEqual(self.session.motion, original)

    def test_partial_or_invalid_sdk_target_is_rejected(self):
        for target in (
            {"q": {"left_gripper": [0.5]}, "height": 0.33},
            {"q": self.base["q"], "height": float("nan")},
            {"q": self.base["q"], "height": 0.85},
        ):
            with self.assertRaises(ValueError):
                self.session.move_to(target)
            self.assertIsNone(self.session.motion)

    def test_mode_change_and_cycle_gap_abort_before_publish(self):
        self.session.tick()
        count = self.robot.low_count
        self.robot.state.mode_machine += 1
        with self.assertRaises(RuntimeError):
            self.session.tick()
        self.assertEqual(count, self.robot.low_count)
        self.robot.state.mode_machine -= 1
        self.robot.advance(0.3)
        with self.assertRaises(RuntimeError):
            self.session.tick()
        self.assertEqual(count, self.robot.low_count)

    def test_height_loss_pauses_long_home_without_losing_return_path(self):
        self.command("upper both 10")
        self.finish()
        self.command("up 200")
        self.finish()
        self.command("home")
        for _ in range(580):  # Reproduce dropout ~11.6 seconds into descending home.
            self.robot.advance()
            self.session.tick()
        before = deepcopy(self.session.commanded)
        lift_count, arm_count = self.robot.lift_count, self.robot.low_count
        start = self.session.motion["start"]
        with patch.object(
            self.robot, "height", side_effect=FeedbackUnavailable("rt/height_state", 0.31, 0.3)
        ):
            for _ in range(25):
                self.robot.advance()
                self.session.tick()
                self.assertEqual(self.robot.speed, 0.0)
                self.assertEqual(self.session.commanded, before)
        self.assertEqual(lift_count, self.robot.lift_count)
        self.assertEqual(arm_count + 25, self.robot.low_count)
        for _ in range(8):
            self.robot.advance()
            self.session.tick()
        self.assertIsNone(self.session.height_pause_started)
        self.assertGreater(self.session.motion["start"], start + 0.5)
        self.finish()
        self.assertEqual(self.session.commanded["q"], self.base["q"])
        self.assertAlmostEqual(self.robot.z, 0.33, delta=0.002)

    def test_height_loss_during_arm_motion_does_not_jump_on_recovery(self):
        self.command("upper both 60")
        for _ in range(200):
            self.robot.advance()
            self.session.tick()
        before = deepcopy(self.session.commanded)
        with patch.object(
            self.robot, "height", side_effect=FeedbackUnavailable("rt/height_alarm", 1.1, 1.0)
        ):
            for _ in range(40):
                self.robot.advance()
                self.session.tick()
        self.assertEqual(before, self.session.commanded)
        for _ in range(8):
            self.robot.advance()
            self.session.tick()
        delta = abs(self.session.commanded["q"]["left_arm"][0] - before["q"]["left_arm"][0])
        self.assertLess(delta, self.session.arm_rate * 0.08)

    def test_height_pause_timeout_includes_unstable_recovery(self):
        self.command("up 200")

        def height():
            if self.robot.now % 0.1 < 0.05:
                raise FeedbackUnavailable("rt/height_state", 0.31, 0.3)
            return self.robot.z

        with patch.object(self.robot, "height", side_effect=height):
            with self.assertRaisesRegex(RuntimeError, "2 秒"):
                for _ in range(110):
                    self.robot.advance()
                    self.session.tick()
        self.assertEqual(self.robot.speed, 0.0)

    def test_home_and_stop_available_during_height_pause(self):
        self.command("upper both 10")
        self.finish()
        with patch.object(
            self.robot, "height", side_effect=FeedbackUnavailable("rt/height_state", 0.31, 0.3)
        ):
            for _ in range(30):
                self.robot.advance()
                self.session.tick()
            self.command("home")
            self.command("stop")
            self.command("home")
            self.assertEqual(self.session.motion["to"], self.base)
        frozen = deepcopy(self.session.commanded)
        for _ in range(5):
            self.robot.advance()
            self.session.tick()
        self.assertEqual(frozen, self.session.commanded)
        for _ in range(8):
            self.robot.advance()
            self.session.tick()
        self.assertLessEqual(self.session.motion["start"], self.robot.now)
        self.finish()
        self.assertEqual(self.session.commanded["q"], self.base["q"])

    def test_real_alarm_is_fatal_and_arm_loss_sends_nothing_during_pause(self):
        with patch.object(
            self.robot, "height", side_effect=FeedbackUnavailable("rt/height_state", 0.31, 0.3)
        ):
            self.session.tick()
        count = self.robot.low_count
        with patch.object(self.robot, "height", side_effect=RuntimeError("Lift alarm")):
            with self.assertRaisesRegex(RuntimeError, "Lift alarm"):
                self.session.tick()
        with patch.object(
            self.robot, "arm_state", side_effect=FeedbackUnavailable("rt/lowstate", 0.31, 0.3)
        ):
            self.session.tick()
            self.assertIsNotNone(self.session.arm_pause_started)
        self.assertEqual(count, self.robot.low_count)

    def arm_gap(self, ticks):
        with patch.object(
            self.robot, "arm_state", side_effect=FeedbackUnavailable("rt/lowstate", 0.31, 0.3)
        ):
            for _ in range(ticks):
                self.robot.advance()
                self.session.tick()

    def test_short_arm_feedback_gap_pauses_trajectory_and_resumes_without_jump(self):
        self.command("upper both 10")
        for _ in range(50):
            self.robot.advance()
            self.session.tick()
        before = deepcopy(self.session.commanded)
        count, start = self.robot.low_count, self.session.motion["start"]
        self.arm_gap(25)
        self.assertEqual(self.robot.low_count, count)  # nothing sent on stale feedback
        self.assertEqual(self.session.commanded, before)
        self.robot.advance()
        self.session.tick()
        self.assertIsNone(self.session.arm_pause_started)
        self.assertAlmostEqual(self.session.motion["start"], start + 25 * 0.02)
        step = abs(self.session.commanded["q"]["left_arm"][0] - before["q"]["left_arm"][0])
        self.assertLess(step, self.session.arm_rate * 0.02 * 1.01)  # one tick at the speed limit
        self.finish()
        self.assertAlmostEqual(
            self.session.commanded["q"]["left_arm"][0],
            self.base["q"]["left_arm"][0] - math.radians(10),
        )

    def test_long_arm_feedback_gap_ends_the_session(self):
        with self.assertRaises(FeedbackUnavailable):
            self.arm_gap(110)  # 2.2 s
        self.assertEqual(self.robot.low_count, 0)

    def test_stop_and_home_during_arm_feedback_gap(self):
        self.command("upper both 10")
        for _ in range(50):
            self.robot.advance()
            self.session.tick()
        self.arm_gap(5)
        held = deepcopy(self.session.commanded["q"])
        with patch.object(
            self.robot, "arm_state", side_effect=FeedbackUnavailable("rt/lowstate", 0.31, 0.3)
        ):
            self.session.stop()
            self.assertEqual(self.session.commanded["q"], held)  # keeps the last sent target
            self.assertIsNone(self.session.motion)
            with self.assertRaisesRegex(ValueError, "双臂反馈恢复前"):
                self.command("forearm both 5")
            self.session.home()
        self.arm_gap(5)
        self.assertEqual(self.session.commanded["q"], held)
        self.finish()
        self.assertEqual(self.session.commanded["q"], self.base["q"])

    def test_official_model_hash_and_missing_model_fail_closed(self):
        info = model_info(require_official=True)
        self.assertTrue(info["official_download_matches"])
        root = Path(info["path"])
        manifest = json.loads((root.parent / "download_manifest.json").read_text())
        expected = next(f["git_blob_sha"] for f in manifest["files"] if f["path"] == "r1d.urdf")
        data = root.read_bytes()
        self.assertEqual(
            hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest(), expected
        )
        with tempfile.TemporaryDirectory() as d:
            with patch.object(model, "urdf", Path(d) / "missing.urdf"):
                with self.assertRaises(RuntimeError):
                    model_info(require_official=True)

    def test_motion_mode_check_does_not_release_without_option(self):
        client = Mock()
        client.CheckMode.return_value = (0, {"name": "ai"})
        with self.assertRaises(RuntimeError):
            prepare(client, release=False)
        client.ReleaseMode.assert_not_called()
        client.CheckMode.side_effect = [(0, {"name": "ai"}), (0, {"name": ""})]
        client.ReleaseMode.return_value = (0, None)
        with patch("r1_d.sdk.motion_mode.time.sleep"):
            prepare(client, release=True)
        client.ReleaseMode.assert_called_once()


if __name__ == "__main__":
    unittest.main()
