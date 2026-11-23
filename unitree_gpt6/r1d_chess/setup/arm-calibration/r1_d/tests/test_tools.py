"""Offline checks of the joint/height test tools and the extended console commands."""

from copy import deepcopy
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from r1_d.sdk import ControlSession
from r1_d.sdk import session as session_module
from r1_d.sdk.model import ARM_NAMES, GROUPS, LIMITS, RIGHT_WRIST_PITCH_MAX
from r1_d.tests.test_bench import FakeRobot
from r1_d.tools import common, height_test, joint_test
from r1_d.tools.commands import BASE, MIRROR, execute_command, plan_command


class SteppingRobot(FakeRobot):
    """Fake robot at the base pose whose clock advances one control period per command."""

    def __init__(self):
        super().__init__()
        for name, q in BASE.items():
            for i, value in zip(GROUPS[name], q):
                self.state.motor_state[i].q = value
        self.z = 0.55
        self.lag = {}  # motor slot -> offset added to the reported position

    def publish(self, topic, message):
        super().publish(topic, message)
        if topic == "rt/lowcmd":
            for i, offset in self.lag.items():
                self.state.motor_state[i].q = message.motor_cmd[i].q + offset
            self.advance(0.004)  # the lift travels 8 mm/s at command 0.1

    def close(self):
        self.stop_lift()


def session_for(robot):
    session = ControlSession(robot, clock=lambda: robot.now)
    session.arm_rate = 0.5
    return session


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.robot = SteppingRobot()
        self.session = session_for(self.robot)
        self.base = deepcopy(self.session.baseline)

    def plan(self, line, commanded=None):
        return plan_command(line, commanded or self.base, self.base, 0.55)

    def test_every_arm_joint_can_be_set_and_jogged(self):
        for index, name in enumerate(ARM_NAMES):
            if name == "wrist_pitch":
                continue
            target = self.plan(f"joint left {name} 10")
            self.assertAlmostEqual(target["q"]["left_arm"][index], math.radians(10))
            self.assertEqual(target["q"]["right_arm"], self.base["q"]["right_arm"])
            jogged = self.plan(f"jog left {name} -4", target)
            self.assertAlmostEqual(jogged["q"]["left_arm"][index], math.radians(6))
        self.assertEqual(
            self.plan("joint left shoulder 10"), self.plan("joint left shoulder_pitch 10")
        )

    def test_both_mirrors_roll_and_yaw_joints(self):
        for index, name in enumerate(ARM_NAMES):
            if name == "wrist_pitch":
                continue
            target = self.plan(f"joint both {name} 12")
            self.assertAlmostEqual(target["q"]["left_arm"][index], math.radians(12))
            self.assertAlmostEqual(target["q"]["right_arm"][index], MIRROR[index] * math.radians(12))
        # A single side is never mirrored.
        self.assertAlmostEqual(
            self.plan("joint right shoulder_roll -12")["q"]["right_arm"][1], math.radians(-12)
        )

    def test_right_wrist_pitch_stays_below_the_faulty_range(self):
        limit = math.degrees(RIGHT_WRIST_PITCH_MAX)
        for line in ("joint right wrist_pitch 0", "jog right wrist_pitch 10", "joint both wrist_pitch 5"):
            with self.assertRaises(ValueError, msg=line):
                self.plan(line)
        ok = self.plan(f"joint right wrist_pitch {limit - 1}")
        self.assertAlmostEqual(ok["q"]["right_arm"][5], math.radians(limit - 1))
        self.assertAlmostEqual(self.plan("joint left wrist_pitch 20")["q"]["left_arm"][5], math.radians(20))
        # A pose that is already above the limit can still be left unchanged by other commands.
        high = deepcopy(self.base)
        high["q"]["right_arm"][5] = 0.0
        self.assertEqual(plan_command("grip both 40", high, high, 0.55)["q"]["right_arm"][5], 0.0)

    def test_unknown_joint_limit_and_speed_are_rejected(self):
        for line in ("joint left wrist 10", "joint left elbow 200", "jog left elbow -200", "base now"):
            with self.assertRaises(ValueError, msg=line):
                self.plan(line)
        for line in ("speed 0.6", "speed 0", "speed nan"):
            with self.assertRaises(ValueError, msg=line):
                execute_command(self.session, line)
        execute_command(self.session, "speed 0.25")
        self.assertEqual(self.session.arm_rate, 0.25)

    def test_base_returns_both_arms_to_the_test_pose(self):
        moved = self.plan("joint both elbow 30")
        target = self.plan("base", moved)
        self.assertEqual({k: target["q"][k] for k in BASE}, BASE)
        self.assertEqual(target["q"]["left_gripper"], moved["q"]["left_gripper"])


class SessionWaitTests(unittest.TestCase):
    def setUp(self):
        self.robot = SteppingRobot()
        self.session = session_for(self.robot)

    def test_wait_runs_a_motion_to_its_end_and_holds_for_a_time(self):
        with patch.object(session_module.time, "sleep", lambda s: None):
            q = list(BASE["left_arm"])
            q[3] -= 0.5
            common.move(self.session, arms={"left_arm": q})
            self.assertIsNone(self.session.motion)
            self.assertAlmostEqual(self.robot.state.motor_state[18].q, q[3])
            start, ticks = self.robot.now, []
            self.session.wait(0.5, each=lambda: ticks.append(self.robot.now))
            self.assertAlmostEqual(self.robot.now - start, 0.5, delta=0.01)
            self.assertAlmostEqual(len(ticks), 125, delta=3)

    def test_trajectory_waits_for_an_arm_that_stays_behind(self):
        q = {k: list(v) for k, v in self.session.commanded["q"].items()}
        q["left_arm"][3] -= 0.8
        self.session.move_to({"q": q, "height": self.session.commanded["height"]})
        for _ in range(400):
            self.session.tick()
        moving = self.session.commanded["q"]["left_arm"][3]
        self.assertLess(moving, BASE["left_arm"][3] - 0.1)
        # Commands no longer arrive: the elbow stays 0.1 rad behind the trajectory.
        frozen = self.robot.state.motor_state[18].q + 0.1
        events = []
        self.session.log = events.append
        publish = self.robot.publish

        def lost(topic, message):
            publish(topic, message)
            self.robot.state.motor_state[18].q = frozen

        self.robot.publish = lost
        self.robot.state.motor_state[18].q = frozen
        for _ in range(250):
            self.session.tick()
        self.assertEqual([e["event"] for e in events], ["arm_wait"])
        self.assertAlmostEqual(self.session.commanded["q"]["left_arm"][3], moving, delta=0.01)
        # The link is back: the trajectory goes on from where it waited, without a jump.
        self.robot.publish = publish
        self.session.tick()
        self.assertAlmostEqual(self.session.commanded["q"]["left_arm"][3], moving, delta=0.02)
        while self.session.motion is not None:
            self.session.tick()
        self.assertAlmostEqual(self.robot.state.motor_state[18].q, q["left_arm"][3])

    def test_arm_that_never_catches_up_ends_the_session(self):
        q = {k: list(v) for k, v in self.session.commanded["q"].items()}
        q["left_arm"][3] -= 0.8
        self.session.move_to({"q": q, "height": self.session.commanded["height"]})
        for _ in range(400):
            self.session.tick()
        self.robot.lag[18] = 0.1
        with self.assertRaisesRegex(RuntimeError, "落后"):
            for _ in range(700):
                self.session.tick()


class JointTestTests(unittest.TestCase):
    def test_ranges_contain_the_base_pose_and_stay_inside_the_limits(self):
        for name, ranges in joint_test.RANGE.items():
            low, high = LIMITS[name]
            for index, (a, b) in enumerate(ranges):
                base = math.degrees(BASE[name][index])
                self.assertTrue(a <= base <= b, (name, index))
                self.assertGreaterEqual(math.radians(a), low[index] + 0.03, (name, index))
                self.assertLessEqual(math.radians(b), high[index] - 0.03, (name, index))
        self.assertLess(math.radians(joint_test.RANGE["right_arm"][5][1]), RIGHT_WRIST_PITCH_MAX)

    def test_stops_visit_both_ends_through_the_middle_and_return(self):
        self.assertEqual(
            joint_test.stops(80.0, -40.0, 100.0, 1.0),
            [20.0, -40.0, 20.0, 80.0, 90.0, 100.0, 90.0, 80.0],
        )
        self.assertEqual(joint_test.stops(80.0, -40.0, 100.0, 0.5), [50.0, 20.0, 50.0, 80.0, 85.0, 90.0, 85.0, 80.0])
        # Short sides have no middle stop; sides below 2 deg are left out.
        self.assertEqual(joint_test.stops(5.0, 0.0, 6.0, 1.0), [0.0, 5.0])
        for name, ranges in joint_test.RANGE.items():
            for index, (a, b) in enumerate(ranges):
                points = joint_test.stops(math.degrees(BASE[name][index]), a, b, 1.0)
                self.assertTrue(points and min(points) >= a - 1e-9 and max(points) <= b + 1e-9)
                self.assertAlmostEqual(points[-1], math.degrees(BASE[name][index]))

    def test_summary_reports_errors_hysteresis_and_verdict(self):
        def point(target, measured, others=0.05):
            return {"target": target, "measured": measured, "error": measured - target, "others_max_error": others}

        points = [point(20, 20.1), point(-40, -40.05), point(20, 19.8), point(80, 80.02)]
        summary = joint_test.summarize(points, 0.3, 0.3)
        self.assertAlmostEqual(summary["max_error"], 0.2)
        self.assertAlmostEqual(summary["return_error"], 0.02)
        self.assertAlmostEqual(summary["hysteresis"], 0.3)
        self.assertTrue(summary["pass"])
        self.assertFalse(joint_test.summarize(points, 0.15, 0.3)["pass"])
        worse = dict(point(80, 80, others=0.4), others_worst="left_arm elbow")
        summary = joint_test.summarize(points + [worse], 0.3, 0.3)
        self.assertFalse(summary["pass"])
        self.assertEqual(summary["others_worst"], "left_arm elbow")

    def test_one_joint_runs_against_the_fake_robot_and_ends_at_base(self):
        robot = SteppingRobot()
        args = SimpleNamespace(rate=0.5, settle=0.6, fraction=0.2, go_base=False)
        runner = joint_test.Runner(args)
        with patch.object(joint_test, "connect", lambda a: (robot, session_for(robot))), patch.object(
            session_module.time, "sleep", lambda s: None
        ), patch("builtins.print"):
            robot.lag[18] = math.radians(0.1)
            points = runner.arm_joint("left_arm", 3)
            gripper = runner.gripper("left_gripper")
        self.assertEqual(len(points), 6)  # the short upper side has no middle stop
        self.assertTrue(all(abs(p["error"] - 0.1) < 1e-6 for p in points))
        self.assertTrue(all(p["others_max_error"] < 1e-6 for p in points))
        self.assertAlmostEqual(points[1]["target"], math.degrees(BASE["left_arm"][3]) + 0.2 * (-38 - math.degrees(BASE["left_arm"][3])))
        self.assertEqual(runner.session.commanded["q"]["left_arm"], BASE["left_arm"])
        self.assertEqual([round(p["target"], 3) for p in gripper], [1.0, 3.0, 5.0, 3.0, 1.0, 0.5])
        self.assertTrue(joint_test.summarize(gripper, 0.1, 0.3)["pass"])
        self.assertEqual(gripper[0]["others_worst"], "left_arm elbow")  # the lagging joint is named
        self.assertAlmostEqual(gripper[0]["others_max_error"], 0.1, places=5)
        runner.close()
        self.assertIsNone(runner.session)
        # A retry after a dropout still ends where the gripper was found, not where it stopped.
        robot.state.motor_state[GROUPS["left_gripper"][0]].q = 3.0
        with patch.object(joint_test, "connect", lambda a: (robot, session_for(robot))), patch.object(
            session_module.time, "sleep", lambda s: None
        ), patch("builtins.print"):
            runner.ensure()
            self.assertAlmostEqual(runner.gripper("left_gripper")[-1]["target"], 0.5)
        runner.close()

    def test_low_lift_or_far_pose_is_refused(self):
        for change, flag in (("z", False), ("pose", False), ("pose", True)):
            robot = SteppingRobot()
            if change == "z":
                robot.z = 0.30
            else:
                robot.state.motor_state[15].q = -1.0
            args = SimpleNamespace(rate=0.5, settle=0.6, fraction=1.0, go_base=flag)
            runner = joint_test.Runner(args)
            with patch.object(joint_test, "connect", lambda a: (robot, session_for(robot))), patch.object(
                session_module.time, "sleep", lambda s: None
            ):
                if flag:
                    runner.ensure()
                    self.assertAlmostEqual(robot.state.motor_state[15].q, BASE["left_arm"][0])
                else:
                    with self.assertRaises(SystemExit):
                        runner.ensure()


class HeightTestTests(unittest.TestCase):
    def test_targets_are_relative_to_the_start_and_clipped(self):
        self.assertEqual(
            [round(t, 4) for t in height_test.targets(0.5572, height_test.DEFAULT_STEPS)],
            [0.6072, 0.6572, 0.6072, 0.5572, 0.5072, 0.5572],
        )
        self.assertEqual(height_test.targets(0.36, [-50, 600]), [0.35, 0.84])

    def test_travel_time_and_speed(self):
        samples = [(t * 0.1, 0.5 + min(max(t - 10, 0), 50) * 0.001) for t in range(100)]
        seconds, speed = height_test.travel(samples, 0.5, 0.55)
        self.assertAlmostEqual(seconds, 4.8, delta=0.25)
        self.assertAlmostEqual(speed, 0.010, delta=0.0006)
        self.assertEqual(height_test.travel(samples[:5], 0.5, 0.5), (None, None))

    def test_lift_segment_against_the_fake_robot(self):
        robot = SteppingRobot()
        session = session_for(robot)
        samples = []
        with patch.object(session_module.time, "sleep", lambda s: None):
            common.move(session, height=0.60, each=lambda: samples.append((robot.now, session.last_height)))
            measured = common.measure(session, 0.6)
        self.assertAlmostEqual(measured["height"], 0.60, delta=0.0025)
        self.assertEqual(robot.speed, 0.0)
        seconds, speed = height_test.travel(samples, 0.55, measured["height"])
        self.assertAlmostEqual(speed, 0.008, delta=0.001)
        for a, b in zip(measured["q"]["left_arm"], BASE["left_arm"]):
            self.assertAlmostEqual(a, b)


if __name__ == "__main__":
    unittest.main()
