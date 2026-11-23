"""The onboard control law against a simulated robot: no DDS, no hardware."""

import math
import unittest

from unitree_gpt6.r1d_draw.src.onboard import controller as c

LOWER = [-math.inf] * c.MOTORS
UPPER = [math.inf] * c.MOTORS
for _slot in c.MODELED:
    LOWER[_slot], UPPER[_slot] = -2.0, 2.0
MODEL_GRAVITY = [0.0] * c.MOTORS
for _first in (15, 22):
    for _j, _value in enumerate((-3.0, -1.5, 0.4, -2.0, 0.2, -0.3, 0.1)):
        MODEL_GRAVITY[_first + _j] = _value
MODEL_GRAVITY[29] = 0.3


class Gravity:
    def compute(self, q):
        return list(MODEL_GRAVITY)


class SimulatedRobot:
    """Joints driven by motor PD + feed-forward against a load the model underestimates."""

    INERTIA = 0.05
    TRUE_LOAD_FACTOR = 1.3  # the real arm needs more torque than the model says

    def __init__(self):
        self.clock = 100.0
        self.q = [0.0] * c.MOTORS
        for slot in c.CONTROLLED:
            self.q[slot] = 0.01 * slot
        self.dq = [0.0] * c.MOTORS
        self.load = [-self.TRUE_LOAD_FACTOR * g for g in MODEL_GRAVITY]
        # Whoever held the arms before left one frame latched in the motors.
        self.command = (list(self.q), [-x for x in self.load], 1.0)
        self.temperature = [45.0] * c.MOTORS
        self.motor_state = [0] * c.MOTORS
        self.tick, self.frozen_feedback = 0, False
        self.foreign, self.factory_released = 0, False
        self.published, self.statuses, self.requests = [], [], []
        self.events, self.simulated_until = [], self.clock
        self.closed = False

    # physics
    def advance(self):
        while self.simulated_until < self.clock - 1e-9:
            dt = min(0.001, self.clock - self.simulated_until)
            target, torque, arm_gain = self.command
            for slot in c.MODELED:
                kp, kd = c.GAINS[slot]
                kp *= arm_gain if slot in c.ARMS else 1.0
                motor = kp * (target[slot] - self.q[slot]) - kd * self.dq[slot] + torque[slot]
                self.dq[slot] += (motor + self.load[slot]) / self.INERTIA * dt
                self.q[slot] += self.dq[slot] * dt
            self.simulated_until += dt

    def measured_torque(self, slot):
        target, torque, arm_gain = self.command
        kp, kd = c.GAINS[slot]
        kp *= arm_gain if slot in c.ARMS else 1.0
        return kp * (target[slot] - self.q[slot]) - kd * self.dq[slot] + torque[slot]

    # io
    def now(self):
        return self.clock

    def sleep(self, seconds):
        self.clock += seconds
        for event in [e for e in self.events if e[0] <= self.clock]:
            self.events.remove(event)
            event[1]()

    def read(self):
        self.advance()
        if not self.frozen_feedback:
            self.tick += 1
        return {
            "tick": self.tick,
            "mode_machine": c.MODE_MACHINE,
            "q": list(self.q),
            "dq": list(self.dq),
            "tau": [self.measured_torque(s) if s in c.MODELED else 0.0 for s in range(c.MOTORS)],
            "temperature_c": list(self.temperature),
            "motor_mode": [1] * c.MOTORS,
            "motor_state": list(self.motor_state),
        }

    def foreign_commands(self, seconds):
        self.sleep(seconds)
        return self.foreign

    def release_factory_controller(self):
        self.factory_released = True

    def open_publisher(self):
        pass

    def publish(self, target, torque, arm_gain=1.0):
        self.advance()
        self.command = (list(target), list(torque), arm_gain)
        self.published.append((self.clock, list(target), list(torque)))

    def read_request(self):
        return self.requests.pop(0) if self.requests else None

    def write_status(self, status):
        self.statuses.append(dict(status))

    def close(self):
        self.closed = True


class ControllerTestCase(unittest.TestCase):
    def setUp(self):
        self.robot = SimulatedRobot()
        self.controller = c.Controller(self.robot, Gravity(), LOWER, UPPER, "/tmp/x", "session-1")

    def run_for(self, seconds):
        end = self.robot.clock + seconds
        while self.robot.clock < end:
            if not self.controller.step(self.robot.now()):
                return False
            self.robot.sleep(c.PERIOD_S)
        return True

    def start(self, settle=0.0):
        self.controller.start(self.robot.now())
        self.robot.sleep(c.PERIOD_S)
        if settle:
            self.run_for(settle)

    def error(self):
        return max(abs(self.robot.q[s] - self.controller.target[s]) for s in c.MODELED)


class TakeOverTests(ControllerTestCase):
    def test_first_frame_repeats_the_measured_torque_at_the_measured_posture(self):
        before = self.robot.read()
        self.start()
        _, target, torque = self.robot.published[0]
        for slot in c.CONTROLLED:
            self.assertAlmostEqual(target[slot], before["q"][slot], places=9)
        for slot in c.ARMS:
            self.assertAlmostEqual(torque[slot], before["tau"][slot], places=6)
        self.assertTrue(self.robot.factory_released)
        self.assertEqual(self.controller.state, "taking_over")

    def test_a_loaded_arm_stays_where_it_is_and_the_integral_removes_the_model_error(self):
        start = list(self.robot.q)
        self.start(settle=c.TAKEOVER_S + 0.2)
        self.assertEqual(self.controller.state, "holding")
        self.assertTrue(self.controller.takeover["completed"])
        self.assertLess(self.controller.takeover["max_drift_rad"], 0.03)
        self.assertLess(self.controller.takeover["max_speed_rad_s"], 0.1)
        self.run_for(20.0)
        self.assertLess(self.error(), 0.002)
        self.assertLess(max(abs(self.robot.q[s] - start[s]) for s in c.MODELED), 0.002)
        self.assertLessEqual(max(abs(v) for v in self.controller.integral.values()), 2.0)

    def test_refuses_when_another_program_commands_the_robot(self):
        self.robot.foreign = 3
        with self.assertRaisesRegex(RuntimeError, "another program"):
            self.controller.start(self.robot.now())
        self.assertEqual(self.robot.published, [])

    def test_refuses_a_moving_or_hot_robot(self):
        self.robot.temperature[24] = 72.0
        with self.assertRaisesRegex(RuntimeError, "motor 24"):
            self.controller.start(self.robot.now())
        self.robot.temperature[24] = 45.0
        self.robot.dq[22] = 0.2
        self.robot.simulated_until = self.robot.clock
        with self.assertRaisesRegex(RuntimeError, "moving"):
            self.controller.start(self.robot.now())
        self.assertEqual(self.robot.published, [])


class RouteTests(ControllerTestCase):
    def request(self, points, identifier="r1"):
        self.robot.requests.append(
            {"id": identifier, "action": "move", "route": {"points": points}}
        )

    def test_route_reaches_its_points_within_the_speed_limit(self):
        self.start(settle=c.TAKEOVER_S + 5.0)
        start = self.controller.target[22]
        self.request(
            [
                {
                    "q": {"22": start + 0.2, "25": self.controller.target[25] - 0.1},
                    "duration_s": 2.0,
                    "dwell_s": 0.5,
                },
                {"q": {"22": start + 0.1}, "duration_s": 1.5},
            ]
        )
        self.run_for(0.1)
        self.assertEqual(self.controller.state, "moving")
        self.run_for(8.0)
        self.assertEqual(self.controller.state, "holding")
        result = self.controller.last_route
        self.assertEqual((result["id"], result["status"], result["points"]), ("r1", "completed", 2))
        self.assertTrue(result["settled"])
        self.assertLess(result["end_error_rad"], 0.02)
        self.assertAlmostEqual(self.controller.target[22], start + 0.1, places=9)
        moves = [(t, q[22]) for t, q, _ in self.robot.published]
        speed = max(
            abs(b[1] - a[1]) / (b[0] - a[0]) for a, b in zip(moves, moves[1:], strict=False)
        )
        self.assertLessEqual(speed, c.COMMAND_SPEED_MAX_RAD_S + 1e-6)
        self.assertLess(max(abs(v) for v in self.robot.dq), c.SPEED_MAX_RAD_S)

    def test_a_run_passes_through_its_inner_points_without_stopping(self):
        self.start(settle=c.TAKEOVER_S + 5.0)
        start = self.controller.target[22]
        step, speed = 0.01, 0.02  # rad per point, rad/s through the run
        points = [{"q": {"22": start + step}, "duration_s": 2 * step / speed, "profile": "ease_in"}]
        points += [
            {"q": {"22": start + step * n}, "duration_s": step / speed, "profile": "linear"}
            for n in range(2, 6)
        ]
        points.append(
            {"q": {"22": start + step * 6}, "duration_s": 2 * step / speed, "profile": "ease_out"}
        )
        self.request(points)
        self.run_for(0.1)
        began = len(self.robot.published)
        self.run_for(2 * step / speed + 4 * step / speed)
        moves = [(t, q[22]) for t, q, _ in self.robot.published[began:]]
        rates = [(b[1] - a[1]) / (b[0] - a[0]) for a, b in zip(moves, moves[1:], strict=False)]
        first = moves[0][0]
        inner = [r for (t, _), r in zip(moves, rates, strict=False) if 1.2 < t - first < 2.7]
        self.assertGreater(len(inner), 100)
        for rate in inner:  # constant commanded speed across four point boundaries
            self.assertAlmostEqual(rate, speed, places=6)
        self.run_for(6.0)
        self.assertEqual(self.controller.last_route["status"], "completed")
        self.assertAlmostEqual(self.controller.target[22], start + step * 6, places=9)

    def test_rejected_routes_do_not_move_the_target(self):
        self.start(settle=c.TAKEOVER_S + 0.5)
        target = list(self.controller.target)
        cases = {
            "gripper": [{"q": {"31": 0.2}, "duration_s": 1.0}],
            "profile": [{"q": {"22": target[22] + 0.01}, "duration_s": 1.0, "profile": "fast"}],
            "ends at speed": [
                {"q": {"22": target[22] + 0.01}, "duration_s": 1.0, "profile": "ease_in"}
            ],
            "starts at speed": [
                {"q": {"22": target[22] + 0.01}, "duration_s": 1.0, "profile": "linear"}
            ],
            "speed jump": [
                {"q": {"22": target[22] + 0.01}, "duration_s": 1.0, "profile": "ease_in"},
                {"q": {"22": target[22] + 0.05}, "duration_s": 1.0, "profile": "ease_out"},
            ],
            "unknown": [{"q": {"3": 0.0}, "duration_s": 1.0}],
            "limit": [{"q": {"22": 2.5}, "duration_s": 60.0}],
            "fast": [{"q": {"22": target[22] + 0.5}, "duration_s": 1.0}],
            "duration": [{"q": {"22": target[22]}, "duration_s": 0}],
            "nan": [{"q": {"22": float("nan")}, "duration_s": 1.0}],
            "empty": [],
            "no-joints": [{"q": {}, "duration_s": 1.0}],
        }
        for name, points in cases.items():
            with self.subTest(name=name):
                self.request(points, name)
                self.run_for(0.05)
                self.assertEqual(self.controller.state, "holding")
                self.assertEqual(self.controller.last_route["status"], "rejected")
                self.assertEqual(self.controller.last_route["id"], name)
                self.assertEqual(self.controller.target, target)

    def test_a_second_route_is_rejected_while_one_is_running_and_ids_run_once(self):
        self.start(settle=c.TAKEOVER_S + 0.5)
        goal = self.controller.target[22] + 0.1
        point = [{"q": {"22": goal}, "duration_s": 2.0}]
        self.request(point, "first")
        self.run_for(0.5)
        self.request(point, "second")
        self.run_for(0.1)
        self.assertEqual(self.controller.last_route["status"], "rejected")
        self.assertEqual(self.controller.route["id"], "first")
        self.run_for(6.0)
        self.assertEqual(self.controller.state, "holding")
        self.assertAlmostEqual(self.controller.target[22], goal, places=9)
        self.request([{"q": {"22": goal + 0.1}, "duration_s": 2.0}], "second")  # same id again
        self.run_for(3.0)
        self.assertAlmostEqual(self.controller.target[22], goal, places=9)

    def test_route_before_holding_is_rejected(self):
        self.start()
        self.request([{"q": {"22": self.controller.target[22]}, "duration_s": 1.0}])
        self.run_for(0.05)
        self.assertEqual(self.controller.last_route["status"], "rejected")
        self.assertEqual(self.controller.state, "taking_over")


class FaultTests(ControllerTestCase):
    def fault_after(self, event, pattern):
        self.robot.events.append((self.robot.clock + c.TAKEOVER_S + 1.0, event))
        state = self.controller.run()
        self.assertEqual(state, "fault")
        self.assertRegex(self.controller.error, pattern)
        self.assertTrue(self.robot.closed)
        self.assertEqual(self.robot.statuses[-1]["state"], "fault")
        frames = len(self.robot.published)
        self.robot.sleep(1.0)
        self.assertEqual(len(self.robot.published), frames)  # nothing is published after a fault
        return frames

    def test_stale_feedback(self):
        self.fault_after(lambda: setattr(self.robot, "frozen_feedback", True), "stale")

    def test_another_publisher_appears(self):
        self.fault_after(lambda: setattr(self.robot, "foreign", 1), "another program")

    def test_motor_overheats(self):
        self.fault_after(lambda: self.robot.temperature.__setitem__(24, 75.0), "motor 24")

    def test_motor_reports_a_fault(self):
        self.fault_after(lambda: self.robot.motor_state.__setitem__(18, 2), "motor 18")

    def test_arm_pushed_away_from_its_target(self):
        def push():
            self.robot.q[22] += 0.3

        self.fault_after(push, "tracking error|moves at")

    def test_control_loop_stalls(self):
        def stall():
            self.robot.clock += 0.2

        self.fault_after(stall, "command gap|stale")

    def test_fault_during_a_route_is_reported_on_the_route(self):
        request = {
            "id": "r9",
            "action": "move",
            "route": {"points": [{"q": {"22": self.robot.q[22] + 0.2}, "duration_s": 3.0}]},
        }
        start = self.robot.clock + c.QUIET_S + c.TAKEOVER_S
        self.robot.events.append((start + 1.0, lambda: self.robot.requests.append(request)))
        self.robot.events.append((start + 2.0, lambda: setattr(self.robot, "foreign", 1)))
        self.assertEqual(self.controller.run(), "fault")
        self.assertEqual(self.controller.last_route["id"], "r9")
        self.assertEqual(self.controller.last_route["status"], "fault")
        self.assertIsNone(self.controller.route)


class RelaxTests(ControllerTestCase):
    def test_relax_fades_arm_stiffness_and_torque_and_keeps_the_grip(self):
        start = self.robot.clock + c.QUIET_S + c.TAKEOVER_S
        self.robot.events.append(
            (start + 1.0, lambda: self.robot.requests.append({"id": "x1", "action": "relax"}))
        )
        before = list(self.robot.q)
        self.assertEqual(self.controller.run(), "relaxed")
        self.assertIsNone(self.controller.error)
        self.assertEqual(self.robot.statuses[-1]["state"], "relaxed")
        target, torque, arm_gain = self.robot.command
        self.assertEqual(arm_gain, 0.0)
        self.assertEqual([torque[s] for s in c.ARMS], [0.0] * 14)
        self.assertAlmostEqual(target[31], before[31], places=9)  # grippers still commanded
        # The loaded joints sank under their load; nothing held them in place.
        self.assertGreater(abs(self.robot.q[22] - before[22]), 0.2)
        gains = [g for _, _, g in [self.robot.command]]
        self.assertEqual(gains, [0.0])

    def test_relax_does_not_jerk_the_arms_at_the_start(self):
        self.start(settle=c.TAKEOVER_S + 5.0)
        self.robot.requests.append({"id": "x1", "action": "relax"})
        self.run_for(0.2)
        self.assertEqual(self.controller.state, "relaxing")
        self.assertLess(max(abs(v) for v in self.robot.dq), 0.2)


class CloseTests(ControllerTestCase):
    def test_close_request_stops_publishing_and_reports_closed(self):
        self.robot.events.append(
            (
                self.robot.clock + c.TAKEOVER_S + 1.0,
                lambda: self.robot.requests.append({"id": "c1", "action": "close"}),
            )
        )
        self.assertEqual(self.controller.run(), "closed")
        self.assertIsNone(self.controller.error)
        self.assertEqual(self.robot.statuses[-1]["state"], "closed")
        self.assertTrue(self.robot.closed)
        last_time, last_target, _ = self.robot.published[-1]
        self.assertEqual(last_target, self.controller.target)
        self.assertGreater(len(self.robot.published), 800)

    def test_status_is_written_about_ten_times_per_second(self):
        self.start(settle=2.0)
        self.assertTrue(15 <= len(self.robot.statuses) <= 25)
        status = self.robot.statuses[-1]
        self.assertEqual((status["session_id"], status["state"]), ("session-1", "taking_over"))
        self.assertEqual(len(status["q"]), c.MOTORS)


class HelperTests(unittest.TestCase):
    def test_quintic(self):
        self.assertEqual(
            (c.quintic(-1), c.quintic(0), c.quintic(1), c.quintic(2)), (0.0, 0.0, 1.0, 1.0)
        )
        self.assertAlmostEqual(c.quintic(0.5), 0.5)
        steps = [c.quintic(i / 1000) for i in range(1001)]
        peak = max(b - a for a, b in zip(steps, steps[1:], strict=False)) * 1000
        self.assertAlmostEqual(peak, 1.875, places=3)

    def test_gains_and_law_cover_exactly_the_joints_they_should(self):
        self.assertEqual(set(c.GAINS), set(c.CONTROLLED))
        self.assertEqual(set(c.LAW), set(c.MODELED))
        self.assertEqual(len(c.MODELED), 17)


if __name__ == "__main__":
    unittest.main()
