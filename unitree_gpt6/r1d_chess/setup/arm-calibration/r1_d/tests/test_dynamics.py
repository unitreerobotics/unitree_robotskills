"""Offline gravity/friction model and feedforward encoding checks; no robot."""

import math
import unittest

import numpy as np

from r1_d.sdk import ControlSession
from r1_d.sdk.dynamics import ArmGravity, GRAVITY, SIDES
from r1_d.sdk.model import ARM_KD, ARM_KP, ARM_TAU_LIMIT, GROUPS, HOLD, calibrated_urdf, urdf
from r1_d.sdk.protocol import CRC, low_command
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_


class StaticArmRobot:
    """Motors settle where PD plus feedforward balances a 'true' gravity model."""

    def __init__(self, truth):
        self.truth = truth
        self.state = unitree_hg_msg_dds__LowState_()
        self.state.mode_machine = 11
        for side in SIDES:
            self.state.motor_state[GROUPS[side + "_arm"][3]].q = 1.4
        self.latest = {"rt/lowstate": self.state}
        self.last_low = None

    def arm_state(self):
        return self.state

    def measured_targets(self, state):
        return {k: [state.motor_state[i].q for i in ids] for k, ids in GROUPS.items()}

    def feedback_status(self):
        return {}

    def height(self):
        return 0.5

    def stop_lift(self):
        pass

    def publish(self, topic, message):
        self.last_low = message
        self.state.tick += 1
        for side in SIDES:
            ids = GROUPS[side + "_arm"]
            q = np.array([message.motor_cmd[i].q for i in ids])
            ff = np.array([message.motor_cmd[i].tau for i in ids])
            # Warm-started fixed point; converges while the command moves slowly.
            actual = np.array([self.state.motor_state[i].q for i in ids])
            for _ in range(2):
                actual = q + (ff - self.truth.torques(side, actual)) / np.array(ARM_KP)
            for i, value, tau in zip(ids, actual, self.truth.torques(side, actual)):
                self.state.motor_state[i].q = float(value)
                self.state.motor_state[i].tau_est = float(tau)


class FakeTime:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        # Coarse 50 Hz steps keep the simulated trajectories short to compute.
        self.now += max(seconds, 0.02)


def frictionless(path):
    model = ArmGravity(path)
    model.friction = {side: (np.zeros(7), np.zeros(7)) for side in SIDES}
    model.load_friction = {side: np.zeros(7) for side in SIDES}
    return model


class GravityModelTests(unittest.TestCase):
    def test_torque_is_gradient_of_potential_energy(self):
        model = ArmGravity(urdf)
        q = np.array([-0.7, 0.3, 0.2, 0.5, 0.4, -0.2, 0.3])
        for side in SIDES:
            q = q * (1 if side == "left" else -1)

            def energy(angles):
                positions, rotations, _ = model.frames(side, angles)
                return -sum(
                    (p[0] * x + r @ p[1:]) @ GRAVITY
                    for p, x, r in zip(model.parameters[side], positions, rotations)
                )

            numeric = [
                (energy(q + 1e-6 * e) - energy(q - 1e-6 * e)) / 2e-6 for e in np.eye(7)
            ]
            np.testing.assert_allclose(model.torques(side, q), numeric, atol=1e-6)

    def test_hanging_arm_needs_little_torque_and_raised_arm_negative_shoulder_torque(self):
        model = ArmGravity(urdf)
        hanging = model.torques("left", [0.06, 0, 0, 1.4, 0, 0, 0])
        raised = model.torques("left", [-1.0, 0, 0, 1.4, 0, 0, 0])
        self.assertLess(abs(hanging[0]), 1.0)
        self.assertLess(raised[0], -5.0)

    def test_gripper_links_are_lumped_into_last_body(self):
        model = ArmGravity(urdf)
        names = {name for name, _, _ in model.chains["left"]["attached"][6]}
        self.assertEqual(len(names), 5)
        self.assertIn("left_dex1_base_link", names)
        total = sum(model.inertials[n][0] for n in names)
        self.assertAlmostEqual(model.parameters["left"][6, 0], total)


class CalibratedModelTests(unittest.TestCase):
    """The installed calibrated URDF: same kinematics, plausible fitted values."""

    def test_only_arm_inertials_and_joint_dynamics_differ_from_the_official_urdf(self):
        skip = ("<origin", "<mass", "<inertia", "<dynamics")
        original = [a for a in urdf.read_text().split("\n") if not any(s in a for s in skip)]
        fitted = [a for a in calibrated_urdf.read_text().split("\n") if not any(s in a for s in skip)]
        self.assertEqual(original, fitted)

    def test_friction_is_small_and_nonnegative(self):
        model = ArmGravity(calibrated_urdf)
        for side in SIDES:
            coulomb, viscous = model.friction[side]
            self.assertTrue(np.all(np.array(coulomb) >= 0) and np.all(np.array(viscous) >= 0))
            self.assertLess(max(coulomb), 3.0)
            self.assertLess(max(viscous), 3.0)
            self.assertTrue(np.all(model.load_friction[side] >= 0))
            self.assertLess(max(model.load_friction[side]), 0.5)

    def test_friction_feedforward_follows_direction_and_velocity(self):
        model = ArmGravity(calibrated_urdf)
        q = {"left_arm": [0.06, 0.1, 0, 1.4, 0, 0.09, 0], "right_arm": [0.09, -0.1, 0, 1.4, 0, -0.21, 0]}
        rest = model.feedforward(q)
        zero, one = {k: [0.0] * 7 for k in q}, {k: [1.0] * 7 for k in q}
        np.testing.assert_allclose(rest["left_arm"], model.torques("left", q["left_arm"]), atol=1e-9)
        coulomb, viscous = (np.array(v) for v in model.friction["left"])
        # Dry friction: constant part plus the part that grows with the gravity load.
        coulomb = coulomb + model.load_friction["left"] * np.abs(rest["left_arm"])
        limit = np.array(ARM_TAU_LIMIT)
        moving = model.feedforward(q, one, {k: [0.1] * 7 for k in q})
        expected = np.clip(np.array(rest["left_arm"]) + coulomb + 0.1 * viscous, -limit, limit)
        np.testing.assert_allclose(moving["left_arm"], expected, atol=1e-9)
        half = model.feedforward(q, one, zero, 0.5)
        np.testing.assert_allclose(
            half["left_arm"], np.clip(np.array(rest["left_arm"]) + 0.5 * coulomb, -limit, limit), atol=1e-9
        )
        back = model.feedforward(q, {k: [-1.0] * 7 for k in q}, zero)
        np.testing.assert_allclose(
            back["left_arm"], np.clip(np.array(rest["left_arm"]) - coulomb, -limit, limit), atol=1e-9
        )

    def test_load_friction_grows_with_the_gravity_torque(self):
        model = ArmGravity(calibrated_urdf)
        model.friction = {side: (np.zeros(7), np.zeros(7)) for side in SIDES}
        model.load_friction = {side: np.full(7, 0.1) for side in SIDES}
        q = {"left_arm": [-1.2, 0.3, 0, 0.6, 0, 0, 0], "right_arm": [-1.2, -0.3, 0, 0.6, 0, 0, 0]}
        one = {k: [1.0] * 7 for k in q}
        for side in SIDES:
            gravity = model.torques(side, q[side + "_arm"])
            self.assertGreater(abs(gravity[0]), 5.0)
            np.testing.assert_allclose(
                model.feedforward(q, one)[side + "_arm"], gravity + 0.1 * np.abs(gravity), atol=1e-9
            )

    def test_feedforward_stays_inside_the_torque_bound_over_the_joint_test(self):
        from r1_d.tools.commands import BASE
        from r1_d.tools.joint_test import RANGE

        model = ArmGravity(calibrated_urdf)
        one = {k: [1.0] * 7 for k in BASE}
        for name, side in (("left_arm", "left"), ("right_arm", "right")):
            coulomb = np.array(model.friction[side][0])
            for index, (low, high) in enumerate(RANGE[name]):
                for angle in np.linspace(low, high, 40):
                    q = list(BASE[name])
                    q[index] = math.radians(angle)
                    gravity = np.abs(model.torques(side, q))
                    torque = gravity + coulomb + model.load_friction[side] * gravity
                    self.assertTrue(np.all(torque < 0.85 * np.array(ARM_TAU_LIMIT)), (name, index, angle))
                    low_command(
                        unitree_hg_msg_dds__LowState_(),
                        {**{k: list(v) for k, v in BASE.items()}, name: q, "left_gripper": [0.0], "right_gripper": [0.0]},
                        {i: 0.0 for i in HOLD},
                        model.feedforward({**BASE, name: q}, one),
                    )


class FeedforwardTests(unittest.TestCase):
    def setUp(self):
        self.state = unitree_hg_msg_dds__LowState_()
        self.targets = {name: [0.0] * len(ids) for name, ids in GROUPS.items()}
        self.hold = {i: 0.0 for i in HOLD}

    def test_feedforward_is_encoded_for_arms_only_with_valid_crc(self):
        ff = {"left_arm": [-9.0, 0.5, 0, -3.0, 0, 0.2, 0], "right_arm": [0.0] * 7}
        velocity = {"left_arm": [0.2, 0, 0, -0.1, 0, 0, 0], "right_arm": [0.0] * 7}
        msg = low_command(self.state, self.targets, self.hold, ff, velocity)
        self.assertEqual(msg.motor_cmd[15].tau, -9.0)
        self.assertEqual(msg.motor_cmd[18].tau, -3.0)
        self.assertAlmostEqual(msg.motor_cmd[15].dq, 0.2)
        self.assertAlmostEqual(msg.motor_cmd[18].dq, -0.1)
        self.assertEqual([msg.motor_cmd[i].kp for i in GROUPS["left_arm"]], list(ARM_KP))
        self.assertEqual([msg.motor_cmd[i].kd for i in GROUPS["right_arm"]], [ARM_KD] * 7)
        self.assertEqual(msg.crc, CRC().Crc(msg))
        for i in (*HOLD, 31, 33, 0, 12):
            self.assertEqual(msg.motor_cmd[i].tau, 0.0)
            self.assertEqual(msg.motor_cmd[i].dq, 0.0)
        plain = low_command(self.state, self.targets, self.hold)
        self.assertTrue(all(m.tau == 0.0 and m.dq == 0.0 for m in plain.motor_cmd))

    def test_out_of_range_or_misplaced_feedforward_is_rejected(self):
        for ff in (
            {"left_arm": [25.0, 0, 0, 0, 0, 0, 0]},
            {"left_arm": [0, 0, 0, 0, 0, 5.0, 0]},
            {"left_arm": [math.nan, 0, 0, 0, 0, 0, 0]},
            {"left_arm": [0.0] * 6},
            {"left_gripper": [0.1]},
        ):
            with self.assertRaises(ValueError):
                low_command(self.state, self.targets, self.hold, ff)
        for velocity in (
            {"left_arm": [1.5, 0, 0, 0, 0, 0, 0]},
            {"right_arm": [math.inf, 0, 0, 0, 0, 0, 0]},
            {"left_gripper": [0.1]},
        ):
            with self.assertRaises(ValueError):
                low_command(self.state, self.targets, self.hold, None, velocity)

    def test_session_feedforward_removes_static_sag(self):
        truth = frictionless(urdf)
        for side in SIDES:  # a heavier, longer forearm and gripper than the official model
            truth.parameters[side] = truth.parameters[side].copy()
            truth.parameters[side][3:] *= 1.4
            truth.parameters[side][6, 1] += 0.03
        clock = FakeTime()
        for gravity, limit in ((None, None), (truth, 1e-6)):
            robot = StaticArmRobot(truth)
            session = ControlSession(robot, clock=clock.monotonic, gravity=gravity)
            target = {"q": {k: list(v) for k, v in session.baseline["q"].items()}, "height": 0.5}
            for side in SIDES:
                target["q"][side + "_arm"][0] -= math.radians(60)
            session.move_to(target)
            while session.motion is not None:
                session.tick()
                clock.sleep(0.02)
            error = abs(robot.state.motor_state[15].q - target["q"]["left_arm"][0])
            if gravity is None:
                self.assertGreater(error, 0.02)
                self.assertEqual(robot.last_low.motor_cmd[15].tau, 0.0)
            else:
                self.assertLess(error, limit)
                self.assertLess(robot.last_low.motor_cmd[15].tau, -5.0)

    def test_friction_feedforward_fades_out_after_the_stop(self):
        model = ArmGravity(calibrated_urdf)
        clock = FakeTime()
        robot = StaticArmRobot(frictionless(calibrated_urdf))
        session = ControlSession(robot, clock=clock.monotonic, gravity=model)
        target = {"q": {k: list(v) for k, v in session.baseline["q"].items()}, "height": 0.5}
        target["q"]["left_arm"][0] -= math.radians(30)
        session.move_to(target)
        seen, ended = [], None
        while session.motion is not None:
            session.tick()
            seen.append(session.direction["left_arm"][0])
            m = session.motion
            if ended is None and m is not None and clock.now >= m["start"] + m["duration"]:
                ended = (clock.now, session.direction["left_arm"][0])
            clock.sleep(0.02)
        self.assertEqual(min(seen), -1.0)  # full friction along the motion while it moves
        self.assertLess(ended[1], -0.9)  # still there when the trajectory ends
        self.assertEqual(session.direction["left_arm"], [0.0] * 7)  # gone a second later
        self.assertEqual(session.direction["right_arm"], [0.0] * 7)  # joints that did not move
        rest = model.torques("left", target["q"]["left_arm"])
        np.testing.assert_allclose(session.feedforward()["left_arm"], rest, atol=1e-9)
        self.assertAlmostEqual(robot.last_low.motor_cmd[15].tau, rest[0], places=6)


if __name__ == "__main__":
    unittest.main()
