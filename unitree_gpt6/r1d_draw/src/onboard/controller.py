"""Runs on the robot (Python 3.8): holds the arms in place and runs joint routes.

    python3 controller.py <directory>

The directory holds ``model.urdf``. The controller writes ``status.json`` there
about ten times per second and reads ``request.json``.

take over   The first frame commands the measured posture with the torque the
            motors report, so nothing jumps whoever held the arms before. The
            feed-forward then blends to model gravity over TAKEOVER_S.
hold        Motor PD around the target, gravity feed-forward, bounded velocity
            damping and a slow bounded integral.
route       Quintic joint interpolation through the requested points.
relax       Arm stiffness and feed-forward fade to zero while the damping stays,
            so the arms sink slowly; the grippers keep their grip. The last
            frame (damping only) stays in the motors.
stop        On a close request, a signal or any fault the controller stops
            publishing. The motors keep the last frame: nothing retreats and
            nothing is relaxed.

The control law lives in ``Controller`` and is hardware free; ``DdsIo`` and
``GravityModel`` are the only parts that need the robot's software.
"""
# The robot runs Python 3.8: no zip(strict=) and percent formatting stays.
# ruff: noqa: UP031, B905

import json
import math
import os
import sys
import time

PERIOD_S = 0.004
STATUS_PERIOD_S = 0.1
MOTORS = 35
MODE_MACHINE = 11
WAIST, HEAD, GRIPPERS = (13,), (29, 30), (31, 33)
LEFT, RIGHT = tuple(range(15, 22)), tuple(range(22, 29))
ARMS = LEFT + RIGHT
MODELED = WAIST + ARMS + HEAD  # joints with a model, a target and tracking checks
CONTROLLED = MODELED + GRIPPERS  # grippers only keep the position found at take-over

# Motor PD (kp, kd) per slot.
GAINS = {13: (50.0, 3.0), 29: (50.0, 2.0), 30: (10.0, 0.1), 31: (5.0, 0.05), 33: (5.0, 0.05)}
# Per slot: integral gain, velocity damping, integral limit, gravity limit,
# feed-forward limit, total effort limit (PD + feed-forward).
LAW = {
    13: (2.0, 3.0, 0.5, 0.1, 0.5, 3.0),
    29: (10.0, 2.0, 1.0, 1.5, 1.5, 3.0),
    30: (1.0, 0.1, 0.1, 0.15, 0.2, 0.5),
}
for _first in (15, 22):
    for _j, (_kp, _effort) in enumerate(
        zip((50.0, 50.0, 40.0, 40.0, 30.0, 30.0, 30.0), (12.0, 12.0, 8.0, 8.0, 4.0, 4.0, 4.0))
    ):
        GAINS[_first + _j] = (_kp, 2.0)
        LAW[_first + _j] = (10.0, 6.0, 2.0, 6.0, 6.0, _effort)
VELOCITY_FILTER_S = 0.025
ERROR_DEADBAND_RAD = 0.0005

FEEDBACK_MAX_AGE_S = 0.25
PUBLISH_GAP_MAX_S = 0.08
TRACKING_MAX_RAD = 0.15
SPEED_MAX_RAD_S = 0.5
TEMPERATURE_START_MAX_C = 72.0
TEMPERATURE_MAX_C = 75.0
TAKEOVER_S = 3.0
TAKEOVER_DRIFT_MAX_RAD = 0.08
TAKEOVER_SPEED_MAX_RAD_S = 0.25
QUIET_S = 0.6
COMMAND_SPEED_MAX_RAD_S = 0.3
ROUTE_POINTS_MAX = 2000
# How a route point is timed: name -> (peak, start, end speed as multiples of its mean speed).
# "quintic" starts and ends at rest. A run "ease_in", "linear"..., "ease_out" passes through
# its inner points at speed, so a stroke is drawn in one motion instead of point by point.
PROFILES = {
    "quintic": (1.875, 0.0, 0.0),
    "ease_in": (2.0, 0.0, 2.0),
    "linear": (1.0, 1.0, 1.0),
    "ease_out": (2.0, 2.0, 0.0),
}
VELOCITY_STEP_MAX_RAD_S = 0.01  # joint speed may jump by this much between two points of a run
SETTLE_SPEED_RAD_S = 0.03
SETTLE_S = 0.3
SETTLE_TIMEOUT_S = 3.0
RELAX_S = 3.0  # arm stiffness and feed-forward fade to zero over this time
RELAX_HOLD_S = 0.6


def quintic(x):
    """Rest-to-rest fraction of a move; its peak speed is 1.875 times the mean."""
    x = min(1.0, max(0.0, x))
    return x * x * x * (10.0 - 15.0 * x + 6.0 * x * x)


def fraction(x, profile):
    """Fraction of a route point's move done at fraction x of its time."""
    x = min(1.0, max(0.0, x))
    if profile == "ease_in":
        return x * x * x * (2.0 - x)
    if profile == "linear":
        return x
    if profile == "ease_out":
        return 1.0 - (1.0 - x) ** 3 * (1.0 + x)
    return quintic(x)


def clip(value, low, high):
    return low if value < low else high if value > high else value


def checked_route(route, target, lower, upper):
    """Return [(goal35, duration_s, dwell_s, profile)] from ``target`` on, or raise ValueError."""
    points = route.get("points") if isinstance(route, dict) else None
    if not isinstance(points, list) or not 1 <= len(points) <= ROUTE_POINTS_MAX:
        raise ValueError("route.points must hold 1..%d points" % ROUTE_POINTS_MAX)
    result, current, moving = [], list(target), None  # moving: joint velocity carried over
    for number, point in enumerate(points):
        label = "route.points[%d]" % number
        joints = point.get("q") if isinstance(point, dict) else None
        if not isinstance(joints, dict) or not joints:
            raise ValueError(label + ".q must map motor slots to radians")
        goal = list(current)
        for key, value in joints.items():
            slot = int(key)
            if slot not in MODELED:
                raise ValueError("%s: slot %s cannot be commanded" % (label, key))
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError("%s: slot %s needs a finite angle" % (label, key))
            if not lower[slot] <= value <= upper[slot]:
                raise ValueError("%s: slot %s leaves the joint limits" % (label, key))
            goal[slot] = float(value)
        duration, dwell = point.get("duration_s"), point.get("dwell_s", 0.0)
        for name, number_value, minimum in (
            ("duration_s", duration, 1e-3),
            ("dwell_s", dwell, 0.0),
        ):
            if (
                type(number_value) not in (int, float)
                or not math.isfinite(number_value)
                or not minimum <= number_value <= 600.0
            ):
                raise ValueError("%s.%s is out of range" % (label, name))
        profile = point.get("profile", "quintic")
        if profile not in PROFILES:
            raise ValueError("%s.profile must be one of %s" % (label, sorted(PROFILES)))
        peak, start, end = PROFILES[profile]
        mean = [(b - a) / duration for a, b in zip(current, goal)]
        if peak * max(abs(v) for v in mean) > COMMAND_SPEED_MAX_RAD_S:
            raise ValueError(
                "%s: peak joint speed %.3f rad/s exceeds %.2f"
                % (label, peak * max(abs(v) for v in mean), COMMAND_SPEED_MAX_RAD_S)
            )
        if (start > 0.0) != (moving is not None):
            raise ValueError("%s: a run is ease_in, linear..., ease_out" % label)
        if moving is not None:
            jump = max(abs(start * v - w) for v, w in zip(mean, moving))
            if jump > VELOCITY_STEP_MAX_RAD_S + 1e-9:
                raise ValueError("%s: joint speed jumps by %.4f rad/s" % (label, jump))
        moving = [end * v for v in mean] if end > 0.0 else None
        if moving is not None and (dwell > 0.0 or number == len(points) - 1):
            raise ValueError("%s: a point passed at speed cannot dwell or end a route" % label)
        result.append((goal, float(duration), float(dwell), profile))
        current = goal
    return result


class Controller:
    """One control cycle per ``step``; all hardware access goes through ``io``."""

    def __init__(self, io, gravity, lower, upper, directory, session_id):
        self.io, self.gravity, self.lower, self.upper = io, gravity, lower, upper
        self.directory, self.session_id = directory, session_id
        self.state, self.error = "starting", None
        self.target = None
        self.frames_sent = 0
        self.last_tick = self.last_feedback = self.last_publish = None
        self.integral = dict.fromkeys(MODELED, 0.0)
        self.velocity = dict.fromkeys(MODELED, 0.0)
        self.takeover_torque = self.takeover_began = None
        self.relax_torque = self.relax_began = None
        self.takeover = {}
        self.route = None
        self.last_route = None
        self.last_request_id = self.request_error = None
        self.last_status = 0.0
        self.last_sample = None
        self.last_torque = [0.0] * MOTORS

    # ---- feedback ---------------------------------------------------------------------
    def read(self, now, temperature_max=TEMPERATURE_MAX_C, check_speed=True):
        sample = self.io.read()
        if sample is None or len(sample["q"]) != MOTORS or sample["mode_machine"] != MODE_MACHINE:
            raise RuntimeError("robot state is missing or its layout changed")
        if sample["tick"] != self.last_tick:
            if self.last_tick is not None and sample["tick"] < self.last_tick:
                raise RuntimeError("robot state tick went backwards")
            self.last_tick, self.last_feedback = sample["tick"], now
        if self.last_feedback is None or now - self.last_feedback > FEEDBACK_MAX_AGE_S:
            raise RuntimeError("robot state is stale")
        for slot in CONTROLLED:
            values = (sample["q"][slot], sample["dq"][slot], sample["tau"][slot])
            if not all(math.isfinite(v) for v in values):
                raise RuntimeError("non-finite feedback at motor %d" % slot)
            if sample["motor_mode"][slot] != 1 or sample["motor_state"][slot] != 0:
                raise RuntimeError("motor %d is disabled or reports a fault" % slot)
            if sample["temperature_c"][slot] >= temperature_max:
                raise RuntimeError(
                    "motor %d reached %.0f C" % (slot, sample["temperature_c"][slot])
                )
            if check_speed and abs(sample["dq"][slot]) > SPEED_MAX_RAD_S:
                raise RuntimeError("motor %d moves at %.3f rad/s" % (slot, sample["dq"][slot]))
        self.last_sample = sample
        return sample

    # ---- start ------------------------------------------------------------------------
    def start(self, now):
        """Check the robot, make sure nobody else commands it, publish the first frame."""
        sample = self.read(now, TEMPERATURE_START_MAX_C)
        if max(abs(sample["dq"][slot]) for slot in CONTROLLED) > 0.05:
            raise RuntimeError("the robot is moving; take over only when it is still")
        self.gravity.compute(sample["q"])  # the model must accept the posture before any command
        self.io.release_factory_controller()
        if self.io.foreign_commands(QUIET_S) > 0:
            raise RuntimeError("another program is publishing joint commands")
        sample = self.read(self.io.now())
        self.target = list(sample["q"])
        self.takeover_torque = [0.0] * MOTORS
        for slot in MODELED:
            limit = LAW[slot][4]
            self.takeover_torque[slot] = clip(sample["tau"][slot], -limit, limit)
        self.takeover_began = self.io.now()
        self.takeover = {
            "measured_torque": list(sample["tau"]),
            "max_drift_rad": 0.0,
            "max_speed_rad_s": 0.0,
            "completed": False,
        }
        self.state = "taking_over"
        self.io.open_publisher()
        self.publish(sample, self.takeover_began)

    # ---- one cycle --------------------------------------------------------------------
    def step(self, now):
        if self.state == "relaxing":
            return self.advance_relax(now)
        sample = self.read(now)
        if self.io.foreign_commands(0.0) > 0:
            raise RuntimeError("another program started publishing joint commands")
        request = self.next_request()
        if request == "close":
            self.state = "closed"
            self.write_status(now, force=True)
            return False
        if self.state == "taking_over":
            self.advance_takeover(sample, now)
        elif self.route is not None:
            self.advance_route(sample, now)
        error = max(abs(sample["q"][slot] - self.target[slot]) for slot in MODELED)
        if error > TRACKING_MAX_RAD:
            raise RuntimeError(
                "joint tracking error %.4f rad exceeds %.2f" % (error, TRACKING_MAX_RAD)
            )
        self.publish(sample, now)
        self.write_status(now)
        return True

    def advance_relax(self, now):
        """Fade arm stiffness and feed-forward to zero; the damping stays.

        The commanded angles and the feed-forward of the moment relax was asked
        stay fixed while they fade, so nothing pulls. The arms follow gravity,
        so speed is not a fault here.
        """
        sample = self.read(now, check_speed=False)
        x = min(1.0, (now - self.relax_began) / RELAX_S)
        gain = 1.0 - x * x * (3.0 - 2.0 * x)
        torque = list(self.relax_torque)
        for slot in ARMS:
            torque[slot] *= gain
            if gain == 0.0:
                self.target[slot] = sample["q"][slot]
        self.publish(sample, now, arm_gain=gain, torque=torque)
        if now - self.relax_began >= RELAX_S + RELAX_HOLD_S:
            self.state = "relaxed"
            self.write_status(now, force=True)
            return False
        self.write_status(now)
        return True

    def advance_takeover(self, sample, now):
        drift = max(abs(sample["q"][slot] - self.target[slot]) for slot in CONTROLLED)
        speed = max(abs(sample["dq"][slot]) for slot in CONTROLLED)
        self.takeover["max_drift_rad"] = max(self.takeover["max_drift_rad"], drift)
        self.takeover["max_speed_rad_s"] = max(self.takeover["max_speed_rad_s"], speed)
        if drift > TAKEOVER_DRIFT_MAX_RAD or speed > TAKEOVER_SPEED_MAX_RAD_S:
            raise RuntimeError(
                "take-over moved the robot: drift %.4f rad, speed %.4f rad/s" % (drift, speed)
            )
        if now - self.takeover_began >= TAKEOVER_S:
            self.takeover["completed"] = True
            self.state = "holding"

    def advance_route(self, sample, now):
        route = self.route
        goal, duration, dwell, profile = route["points"][route["index"]]
        elapsed = now - route["began"]
        while elapsed >= duration and PROFILES[profile][2] > 0.0:
            # Passed at speed: the next point starts at the instant this one ended.
            route.update(
                index=route["index"] + 1, start=list(goal), began=route["began"] + duration
            )
            goal, duration, dwell, profile = route["points"][route["index"]]
            elapsed = now - route["began"]
        done = fraction(elapsed / duration, profile)
        self.target = [a + (b - a) * done for a, b in zip(route["start"], goal)]
        if elapsed < duration + dwell:
            return
        self.target = list(goal)
        if route["index"] + 1 < len(route["points"]):
            route.update(index=route["index"] + 1, start=list(goal), began=now)
            return
        # Last point: wait until the arms are still, then report the measured error.
        # Raw joint speed is noisy at rest (up to 0.02 rad/s): use the filtered value.
        speed = max(abs(self.velocity[slot]) for slot in MODELED)
        if speed > SETTLE_SPEED_RAD_S:
            route["still_since"] = None
        elif route.get("still_since") is None:
            route["still_since"] = now
        settled = route["still_since"] is not None and now - route["still_since"] >= SETTLE_S
        waited = elapsed - duration - dwell
        if settled or waited >= SETTLE_TIMEOUT_S:
            self.last_route = {
                "id": route["id"],
                "status": "completed",
                "points": len(route["points"]),
                "settled": bool(settled),
                "end_error_rad": max(abs(sample["q"][slot] - goal[slot]) for slot in MODELED),
                "end_q": list(sample["q"]),
                "end_target": list(goal),
                "time_unix": time.time(),
            }
            self.route, self.state = None, "holding"

    # ---- requests ---------------------------------------------------------------------
    def next_request(self):
        request = self.io.read_request()
        if not isinstance(request, dict) or request.get("id") in (None, self.last_request_id):
            return None
        self.last_request_id, self.request_error = request["id"], None
        action = request.get("action")
        if action == "close":
            return "close"
        if action == "relax":
            if self.route is not None:
                self.last_route = {
                    "id": self.route["id"],
                    "status": "stopped",
                    "index": self.route["index"],
                    "error": "relaxed on request",
                }
                self.route = None
            self.state, self.relax_began = "relaxing", self.io.now()
            self.relax_torque = list(self.last_torque)
            return None
        if action == "stop":
            # Cancel the rest of the route and hold the posture commanded right now.
            if self.route is not None:
                self.last_route = {
                    "id": self.route["id"],
                    "status": "stopped",
                    "index": self.route["index"],
                    "error": "stopped on request",
                }
                self.route, self.state = None, "holding"
            return None
        try:
            if action != "move":
                raise ValueError("unknown action %r" % (action,))
            if self.state != "holding":
                raise ValueError("the controller is %s, not holding" % self.state)
            points = checked_route(request.get("route"), self.target, self.lower, self.upper)
        except ValueError as exc:
            self.request_error = str(exc)
            self.last_route = {"id": request["id"], "status": "rejected", "error": str(exc)}
            return None
        self.route = {
            "id": request["id"],
            "points": points,
            "index": 0,
            "start": list(self.target),
            "began": self.io.now(),
        }
        self.state = "moving"
        return None

    # ---- command ----------------------------------------------------------------------
    def torque(self, sample, now):
        """Feed-forward torque per slot for this cycle."""
        gravity = self.gravity.compute(sample["q"])
        dt = min(
            PUBLISH_GAP_MAX_S, PERIOD_S if self.last_publish is None else now - self.last_publish
        )
        blend = quintic((now - self.takeover_began) / TAKEOVER_S)
        integrate = self.state == "holding"
        alpha = dt / (VELOCITY_FILTER_S + dt)
        result = [0.0] * MOTORS
        for slot in MODELED:
            ki, damping, integral_limit, gravity_limit, torque_limit, effort = LAW[slot]
            kp, kd = GAINS[slot]
            error = self.target[slot] - sample["q"][slot]
            motor_pd = kp * error - kd * sample["dq"][slot]
            self.velocity[slot] += alpha * (sample["dq"][slot] - self.velocity[slot])
            if integrate and abs(error) > ERROR_DEADBAND_RAD:
                self.integral[slot] = clip(
                    self.integral[slot] + ki * error * dt, -integral_limit, integral_limit
                )
            law = (
                clip(gravity[slot], -gravity_limit, gravity_limit)
                + self.integral[slot]
                - damping * self.velocity[slot]
            )
            value = (1.0 - blend) * self.takeover_torque[slot] + blend * law
            # Keep PD + feed-forward inside the effort limit of the joint.
            low, high = max(-torque_limit, -effort - motor_pd), min(torque_limit, effort - motor_pd)
            result[slot] = clip(value, low, high)
        return result

    def publish(self, sample, now, arm_gain=1.0, torque=None):
        if self.last_publish is not None and now - self.last_publish > PUBLISH_GAP_MAX_S:
            raise RuntimeError(
                "command gap %.3f s exceeds %.2f" % (now - self.last_publish, PUBLISH_GAP_MAX_S)
            )
        if torque is None:
            torque = self.torque(sample, now)
        self.io.publish(self.target, torque, arm_gain)
        self.last_torque, self.last_publish = torque, now
        self.frames_sent += 1

    # ---- status -----------------------------------------------------------------------
    def write_status(self, now, force=False):
        if not force and now - self.last_status < STATUS_PERIOD_S:
            return
        self.last_status = now
        sample = self.last_sample
        self.io.write_status(
            {
                "session_id": self.session_id,
                "pid": os.getpid(),
                "state": self.state,
                "error": self.error,
                "time_unix": time.time(),
                "frames_sent": self.frames_sent,
                "target": self.target,
                "torque": self.last_torque,
                "q": None if sample is None else sample["q"],
                "dq": None if sample is None else sample["dq"],
                "temperature_c": None if sample is None else sample["temperature_c"],
                "tracking_error_rad": None
                if sample is None or self.target is None
                else max(abs(sample["q"][slot] - self.target[slot]) for slot in MODELED),
                "takeover": self.takeover,
                "last_route": self.last_route,
                "route": None
                if self.route is None
                else {
                    "id": self.route["id"],
                    "index": self.route["index"],
                    "points": len(self.route["points"]),
                },
                "last_request_id": self.last_request_id,
                "request_error": self.request_error,
            }
        )

    def run(self):
        """Control loop; returns when closed. Any exception leaves state ``fault``."""
        try:
            self.start(self.io.now())
            deadline = self.io.now()
            while self.step(self.io.now()):
                deadline += PERIOD_S
                delay = deadline - self.io.now()
                if delay > 0:
                    self.io.sleep(delay)
                else:
                    deadline = self.io.now()
        except Exception as exc:  # stop publishing; the motors keep the last frame
            self.state, self.error = "fault", "%s: %s" % (type(exc).__name__, exc)
            if self.route is not None:
                self.last_route = {
                    "id": self.route["id"],
                    "status": "fault",
                    "error": self.error,
                    "index": self.route["index"],
                }
                self.route = None
        finally:
            try:
                self.write_status(self.io.now(), force=True)
            finally:
                self.io.close()
        return self.state


class GravityModel:
    """Gravity torque of the 17 modeled joints from the user's URDF (Pinocchio)."""

    def __init__(self, urdf_text):
        import pinocchio as pin

        self.pin = pin
        names = {"waist_yaw_joint": 13, "head_pitch_joint": 29, "head_yaw_joint": 30}
        parts = (
            "shoulder_pitch",
            "shoulder_roll",
            "shoulder_yaw",
            "elbow",
            "wrist_roll",
            "wrist_pitch",
            "wrist_yaw",
        )
        for side, first in (("left", 15), ("right", 22)):
            names.update({"%s_%s_joint" % (side, part): first + j for j, part in enumerate(parts)})
        full = pin.buildModelFromXML(urdf_text)
        missing = [name for name in names if not full.existJointName(name)]
        if missing:
            raise ValueError("URDF lacks joints: %s" % ", ".join(sorted(missing)))
        locked = [i for i in range(1, full.njoints) if full.names[i] not in names]
        self.model = pin.buildReducedModel(full, locked, pin.neutral(full))
        self.data = self.model.createData()
        self.index, self.lower, self.upper = {}, [-math.inf] * MOTORS, [math.inf] * MOTORS
        for name, slot in names.items():
            joint = self.model.joints[self.model.getJointId(name)]
            if joint.nq != 1 or joint.nv != 1:
                raise ValueError("joint %s must have one degree of freedom" % name)
            self.index[slot] = (joint.idx_q, joint.idx_v)
            self.lower[slot] = float(self.model.lowerPositionLimit[joint.idx_q])
            self.upper[slot] = float(self.model.upperPositionLimit[joint.idx_q])
        self.q = pin.neutral(self.model)

    def compute(self, q35):
        for slot, (iq, _) in self.index.items():
            self.q[iq] = q35[slot]
        tau = self.pin.computeGeneralizedGravity(self.model, self.data, self.q)
        result = [0.0] * MOTORS
        for slot, (_, iv) in self.index.items():
            result[slot] = float(tau[iv])
            if not math.isfinite(result[slot]):
                raise RuntimeError("model gravity is not finite")
        return result


class DdsIo:
    """The robot's DDS, a lock against a second writer, and the session files."""

    LOCK_PATH = "/tmp/astra_paint_r1d_lowcmd_writer.lock"  # shared with the earlier R1-D tools
    LAYOUT = "<2B2x" + "B3x5fI" * MOTORS + "5I"

    def __init__(self, directory):
        import fcntl
        import struct
        from collections import deque

        from unitree_dds_wrapper.client import Client
        from unitree_dds_wrapper.idl import unitree_hg
        from unitree_dds_wrapper.robots import g1, r1
        from unitree_dds_wrapper.utils.crc import crc32

        self.directory, self.struct, self.crc32, self.hg = directory, struct, crc32, unitree_hg
        self.lock = open(self.LOCK_PATH, "a+")
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.state_reader = g1.sub.LowState(autospin=False)
        self.command_readers = (g1.sub.LowCmd(autospin=False), r1.sub.ArmSdk(autospin=False))
        self.robot_state = Client("robot_state")
        self.robot_state.timeout_s = 4.0
        self.own = deque(maxlen=8192)
        self.writer = None
        self.request_mtime = None
        check = unitree_hg.msg.dds_.LowCmd_()
        check.mode_machine = MODE_MACHINE
        for index, motor in enumerate(check.motor_cmd):
            motor.q = float(index)
        if self.checksum(check) != 3984972370:
            raise RuntimeError("command checksum self-test failed")
        deadline = time.monotonic() + 4.0
        while self.state_reader.msg is None and time.monotonic() < deadline:
            self.state_reader.take_one()
            time.sleep(0.01)

    @staticmethod
    def now():
        return time.monotonic()

    @staticmethod
    def sleep(seconds):
        time.sleep(seconds)

    def read(self):
        self.state_reader.take_one()
        message = self.state_reader.msg
        if message is None:
            return None
        motors = message.motor_state
        return {
            "tick": int(message.tick),
            "mode_machine": int(message.mode_machine),
            "q": [float(m.q) for m in motors],
            "dq": [float(m.dq) for m in motors],
            "tau": [float(m.tau_est) for m in motors],
            "temperature_c": [float(max(m.temperature)) for m in motors],
            "motor_mode": [int(m.mode) for m in motors],
            "motor_state": [int(m.motorstate) for m in motors],
        }

    def foreign_commands(self, seconds):
        """Count command frames that are not this controller's own echo."""
        count, end = 0, time.monotonic() + seconds
        while True:
            for index, reader in enumerate(self.command_readers):
                for _ in range(64):
                    before = getattr(reader, "_last_recv_time", None)
                    reader.take_one()
                    if getattr(reader, "_last_recv_time", None) == before:
                        break
                    if index == 1 or reader.msg.serialize() not in self.own:
                        count += 1
            if time.monotonic() >= end:
                return count
            time.sleep(0.004)

    def release_factory_controller(self):
        """Ask the vendor arm service to stop through its official interface."""
        status = self._service_status()
        if status == 1:
            return
        if status != 0:
            raise RuntimeError("vendor arm service is in an unknown state: %s" % status)
        self._call(1001, {"name": "ai_sport", "switch": 0})
        deadline = time.monotonic() + 12.0
        while self._service_status() != 1:
            if time.monotonic() >= deadline:
                raise RuntimeError("vendor arm service did not stop")
            time.sleep(0.1)

    def _call(self, api, payload):
        deadline = time.monotonic() + 4.0
        publisher = self.robot_state._pub
        while getattr(publisher, "_RequestPublisher__publication_matched_count", 0) < 1:
            if time.monotonic() >= deadline:
                raise RuntimeError("vendor state service is not reachable")
            time.sleep(0.02)
        code, data = self.robot_state.call(api, json.dumps(payload))
        if code != 0:
            raise RuntimeError("vendor state service call %d returned %d" % (api, code))
        return json.loads(data)

    def _service_status(self):
        error = None
        for _ in range(3):  # the first call after start-up often times out
            try:
                for service in self._call(1003, {}):
                    if service.get("name") == "ai_sport":
                        return int(service["status"])
                raise RuntimeError("vendor state service does not list the arm service")
            except RuntimeError as exc:
                error = exc
                time.sleep(0.25)
        raise RuntimeError("vendor state service did not answer: %s" % error)

    def checksum(self, message):
        values = [message.mode_pr, message.mode_machine]
        for motor in message.motor_cmd:
            values.extend(
                (motor.mode, motor.q, motor.dq, motor.tau, motor.kp, motor.kd, motor.reserve)
            )
        values.extend(message.reserve)
        values.append(0)
        packed = self.struct.pack(self.LAYOUT, *values)[:1000]
        return int(self.crc32(self.struct.unpack("<250I", packed)))

    def open_publisher(self):
        from unitree_dds_wrapper.publisher import Publisher

        self.writer = Publisher(self.hg.msg.dds_.LowCmd_, "rt/lowcmd")

    def publish(self, target, torque, arm_gain=1.0):
        message = self.hg.msg.dds_.LowCmd_()
        message.mode_pr, message.mode_machine = 0, MODE_MACHINE
        for slot, (kp, kd) in GAINS.items():
            motor = message.motor_cmd[slot]
            motor.mode, motor.q, motor.dq = 1, float(target[slot]), 0.0
            motor.tau, motor.kd = float(torque[slot]), kd
            motor.kp = kp * arm_gain if slot in ARMS else kp
        message.crc = self.checksum(message)
        self.own.append(message.serialize())
        self.writer.write(message)

    def read_request(self):
        path = os.path.join(self.directory, "request.json")
        try:
            mtime = os.stat(path).st_mtime_ns
            if mtime == self.request_mtime:
                return None
            self.request_mtime = mtime
            with open(path) as stream:
                return json.load(stream)
        except (OSError, ValueError):
            return None

    def write_status(self, status):
        # No fsync: a flush can block the control loop; the rename keeps readers consistent.
        path = os.path.join(self.directory, "status.json")
        with open(path + ".tmp", "w") as stream:
            json.dump(status, stream, allow_nan=False)
        os.replace(path + ".tmp", path)

    def close(self):
        self.lock.close()


def main(directory):
    import gc
    import signal

    directory = os.path.abspath(directory)
    with open(os.path.join(directory, "model.urdf")) as stream:
        gravity = GravityModel(stream.read())
    io = DdsIo(directory)
    controller = Controller(
        io, gravity, gravity.lower, gravity.upper, directory, os.path.basename(directory)
    )

    def stop(signum, frame):
        raise KeyboardInterrupt("signal %d" % signum)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    # A full garbage collection can stall the 250 Hz loop: collect once, then freeze.
    gc.collect()
    gc.freeze()
    try:
        state = controller.run()
    except KeyboardInterrupt as exc:
        controller.state, controller.error = "closed", "stopped by %s" % exc
        controller.write_status(io.now(), force=True)
        state = "closed"
    return 0 if state in ("closed", "relaxed") else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
