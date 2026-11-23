"""Persistent local trajectory/hold/home session. Call tick at 250 Hz."""

from copy import deepcopy
import math
import time
from .model import GROUPS, HOLD, finite, validate_targets
from .protocol import lift_command, low_command
from .client import FeedbackUnavailable


ARM_GAP_SECONDS = 2.0  # longest arm feedback gap that is waited out
ARMS = ("left_arm", "right_arm")
ARM_TOLERANCE = 0.15  # rad, largest arm tracking error before the session stops
# A trajectory waits while an arm joint is this far behind it (commands lost on the
# way hold the robot at the last one that arrived), for at most ARM_WAIT_SECONDS.
ARM_WAIT = 0.07  # rad
ARM_WAIT_SECONDS = 2.0
# Friction feedforward left on a stopped joint pushes it on as soon as a neighbour moves
# (measured: up to 0.3 deg on the shoulder pitch). The friction feedforward of a joint that
# is not being moved therefore ramps out over this time.
REST_FADE_SECONDS = 1.0


class ControlSession:
    """Single-owner 250 Hz control loop. Input thread never accesses the robot."""

    def __init__(self, robot, clock=time.monotonic, log=None, gravity=None, home_pose=None):
        """gravity: optional ArmGravity; adds feedforward at the commanded pose.
        home_pose: optional saved {"q", "height"} (sdk.model.load_home_pose); home moves
        straight to its arms, grippers and height. Without it home goes to the takeover pose."""
        self.robot, self.clock = robot, clock
        self.gravity = gravity
        self.feedforward_cache = (None, None)
        self.log = log or (lambda event: None)
        state = robot.arm_state()
        q = validate_targets(robot.measured_targets(state))
        if any(abs(state.motor_state[i].dq) > 0.15 for ids in GROUPS.values() for i in ids):
            raise RuntimeError("接管前双臂和夹爪必须静止")
        self.mode = state.mode_machine
        self.state = state  # latest arm feedback seen by tick()
        self.hold = {i: float(state.motor_state[i].q) for i in HOLD}
        # Waist/head feedforward torque (Nm) and largest allowed lag; set by the caller per tick.
        self.hold_tau = {i: 0.0 for i in HOLD}
        self.hold_tolerance = 0.15
        self.hold_gains = None  # (kp, kd) per waist/head joint; None = protocol defaults
        self.baseline = {"q": q, "height": robot.height()}
        self.last_height = self.baseline["height"]
        self.commanded = deepcopy(self.baseline)
        self.home_target = deepcopy(self.baseline)
        if home_pose is not None:
            self.home_target = {
                "q": {name: list(home_pose["q"][name]) for name in GROUPS},
                "height": float(home_pose["height"]),
            }
        self.motion = None
        self.last_tick = None
        self.last_lift_write = -math.inf
        self.lift_live = False
        self.height_pause_started = None
        self.height_recovered_since = None
        self.arm_pause_started = None
        self.arm_waited = 0.0
        self.arm_rate = 0.2  # rad/s, peak joint speed of arm trajectories
        self.friction_share = 1.0  # part of the URDF Coulomb friction that is fed forward
        # Per arm joint: way it moves (-1..1, fading to 0 at rest), and trajectory velocity.
        self.direction = {name: [0.0] * 7 for name in ARMS}
        self.velocity = {name: [0.0] * 7 for name in ARMS}
        self.log(
            {
                "event": "baseline",
                "baseline": self.baseline,
                "home": self.home_target,
                "hold": self.hold,
                "mode": self.mode,
            }
        )

    def feedforward(self):
        """Gravity and friction torques of the URDF at the commanded state."""
        if self.gravity is None:
            return None
        key = tuple(
            tuple(source[name]) for source in (self.commanded["q"], self.direction, self.velocity)
            for name in ARMS
        )
        if self.feedforward_cache[0] != key:
            self.feedforward_cache = (
                key,
                self.gravity.feedforward(
                    self.commanded["q"], self.direction, self.velocity, self.friction_share
                ),
            )
        return self.feedforward_cache[1]

    def begin(self, destination, label):
        # Peak derivative of the smoothstep is 1.5; arm rate is <= arm_rate.
        seconds = max(
            [2.0]
            + [
                1.5 * abs(a - b) / (self.arm_rate if name.endswith("arm") else 0.3)
                for name in GROUPS
                for a, b in zip(self.commanded["q"][name], destination["q"][name])
            ]
        )
        # Only used to budget the timeout. tick() requires fresh height before moving.
        height_now = self.last_height
        # During a feedback pause the motion starts at the pause, which is shifted out on resume.
        start = self.clock()
        for paused in (self.arm_pause_started, self.height_pause_started):
            if paused is not None:
                start = paused
        self.motion = {
            "from": deepcopy(self.commanded),
            "to": deepcopy(destination),
            "start": start,
            "duration": seconds,
            "label": label,
            "lift_deadline": start + abs(height_now - destination["height"]) / 0.003 + 10.0,
            "lift": abs(height_now - destination["height"]) > 0.002,
        }
        self.log({"event": "start", "label": label, "target": destination, "duration": seconds})

    def home(self):
        if self.motion is not None and self.motion["label"] == "home":
            return
        # One joint-space move straight to the home pose (no retrace of this session's moves).
        self.begin(deepcopy(self.home_target), "home")

    def stop(self):
        self.robot.stop_lift()
        self.lift_live = False
        if self.arm_pause_started is None:
            q = validate_targets(self.robot.measured_targets(self.robot.arm_state()))
        else:
            # No fresh arm feedback: keep the last sent target, which the robot is holding.
            q = deepcopy(self.commanded["q"])
        self.commanded = {"q": q, "height": self.last_height}
        self.motion = None
        self.log({"event": "stop", "holding": self.commanded})

    def move_to(self, target, label="move"):
        if self.height_pause_started is not None:
            raise ValueError("升降反馈恢复前仅允许停止或归位")
        if self.arm_pause_started is not None:
            raise ValueError("双臂反馈恢复前仅允许停止或归位")
        if self.motion is not None:
            raise ValueError("正在执行动作；可停止或归位")
        target = {"q": validate_targets(target["q"]), "height": finite(target["height"], "height")}
        if set(target["q"]) != set(GROUPS) or not 0 <= target["height"] <= 0.84:
            raise ValueError("需要完整的双臂/夹爪目标及 0..0.84 m 高度")
        self.begin(target, label)

    def shift_motion(self, seconds):
        """Continue the trajectory from where it paused."""
        if self.motion is not None:
            self.motion["start"] += seconds
            self.motion["lift_deadline"] += seconds

    def tick(self):
        now = self.clock()
        if self.last_tick is not None and now - self.last_tick > 0.2:
            raise RuntimeError("控制周期中断超过 0.2 秒，停止本次会话")
        dt = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        try:
            state = self.robot.arm_state()
        except FeedbackUnavailable as exc:
            # Nothing is sent without fresh arm feedback; the robot holds the last command.
            if self.arm_pause_started is None:
                self.arm_pause_started = now
                self.robot.stop_lift()
                self.lift_live = False
                self.log(
                    {
                        "event": "arm_feedback_paused",
                        "reason": str(exc),
                        "feedback": self.robot.feedback_status(),
                    }
                )
            if now - self.arm_pause_started > ARM_GAP_SECONDS:
                raise
            return
        if self.arm_pause_started is not None:
            paused = now - self.arm_pause_started
            self.arm_pause_started = None
            # A height pause that is still open shifts the trajectory itself when it ends.
            if self.height_pause_started is None:
                self.shift_motion(paused)
            self.height_recovered_since = None
            self.log(
                {
                    "event": "arm_feedback_resumed",
                    "pause_seconds": paused,
                    "feedback": self.robot.feedback_status(),
                }
            )
        if state.mode_machine != self.mode:
            raise RuntimeError("mode_machine 改变，停止本次会话")
        self.state = state
        try:
            h = self.robot.height()
        except FeedbackUnavailable as exc:
            if exc.topic not in ("rt/height_state", "rt/height_alarm"):
                raise
            self.height_recovered_since = None
            if self.height_pause_started is None:
                self.height_pause_started = now
                # Keep the strict height age threshold: never move the lift on old feedback.
                self.robot.stop_lift()
                self.lift_live = False
                self.log(
                    {
                        "event": "feedback_paused",
                        "reason": str(exc),
                        "feedback": self.robot.feedback_status(),
                    }
                )
            h = None
        if self.height_pause_started is not None and now - self.height_pause_started > 2.0:
            raise RuntimeError("升降反馈未在 2 秒内稳定恢复，已停止升降；退出本次控制")
        if h is not None:
            self.last_height = h
        if h is not None and self.height_pause_started is not None:
            if self.height_recovered_since is None:
                self.height_recovered_since = now
            if now - self.height_recovered_since >= 0.1:
                paused = now - self.height_pause_started
                self.shift_motion(paused)
                self.height_pause_started = self.height_recovered_since = None
                self.log(
                    {
                        "event": "feedback_resumed",
                        "pause_seconds": paused,
                        "feedback": self.robot.feedback_status(),
                    }
                )
        completed = False
        self.velocity = {name: [0.0] * 7 for name in ARMS}
        if self.motion is not None and self.height_pause_started is None:
            m = self.motion
            behind = max(
                abs(state.motor_state[i].q - q)
                for name in ARMS
                for i, q in zip(GROUPS[name], self.commanded["q"][name])
            )
            if behind > ARM_WAIT:
                if self.arm_waited == 0.0:
                    self.log({"event": "arm_wait", "behind_rad": behind})
                self.arm_waited += dt
                if self.arm_waited > ARM_WAIT_SECONDS:
                    raise RuntimeError(f"双臂落后目标 {behind:.3f} rad 超过 {ARM_WAIT_SECONDS} 秒")
                self.shift_motion(dt)
            else:
                self.arm_waited = 0.0
            t = min(1.0, (now - m["start"]) / m["duration"])
            alpha = t * t * (3.0 - 2.0 * t)
            self.commanded["q"] = {
                k: [a + (b - a) * alpha for a, b in zip(m["from"]["q"][k], m["to"]["q"][k])]
                for k in GROUPS
            }
            rate = 6.0 * t * (1.0 - t) / m["duration"]
            for name in ARMS:
                for j, (a, b) in enumerate(zip(m["from"]["q"][name], m["to"]["q"][name])):
                    if a == b:
                        continue
                    self.velocity[name][j] = (b - a) * rate
                    # The friction direction turns over while the joint speeds up to half
                    # its peak speed.
                    level = 2.0 * min(1.0, rate * m["duration"] / 0.75) - 1.0
                    if b > a:
                        self.direction[name][j] = max(self.direction[name][j], level)
                    else:
                        self.direction[name][j] = min(self.direction[name][j], -level)
            self.commanded["height"] = m["to"]["height"]
            height_error = m["to"]["height"] - h
            needs_lift = m["lift"] and abs(height_error) > 0.002
            if needs_lift and now > m["lift_deadline"]:
                raise TimeoutError("升降未在预期时间内到位")
            if needs_lift and now - self.last_lift_write >= 0.02:
                self.robot.publish("rt/cmd_vel", lift_command(math.copysign(0.1, height_error)))
                self.last_lift_write, self.lift_live = now, True
            elif not needs_lift and self.lift_live:
                self.robot.stop_lift()
                self.lift_live = False
            completed = now - m["start"] >= m["duration"] + 1.0 and not needs_lift
        for name in ARMS:
            for j, moving in enumerate(self.velocity[name]):
                if not moving:
                    level = self.direction[name][j]
                    self.direction[name][j] = math.copysign(
                        max(0.0, abs(level) - dt / REST_FADE_SECONDS), level
                    )
        for name, ids in GROUPS.items():
            tolerance = ARM_TOLERANCE if name.endswith("arm") else 0.6
            if any(
                abs(state.motor_state[i].q - q) > tolerance
                for i, q in zip(ids, self.commanded["q"][name])
            ):
                raise RuntimeError(f"{name} 跟踪误差超过 {tolerance}，停止会话")
        if any(abs(state.motor_state[i].q - self.hold[i]) > self.hold_tolerance for i in HOLD):
            raise RuntimeError("腰部或头部偏离保持位置")
        feedforward = self.feedforward()
        self.robot.publish(
            "rt/lowcmd",
            low_command(
                state, self.commanded["q"], self.hold, feedforward, self.velocity, self.hold_tau,
                self.hold_gains,
            ),
        )
        if completed:
            measured = self.robot.measured_targets(state)
            error = {
                name: max(abs(a - b) for a, b in zip(measured[name], self.commanded["q"][name]))
                for name in GROUPS
            }
            self.log(
                {
                    "event": "holding",
                    "label": self.motion["label"],
                    "measured": measured,
                    "height_m": h,
                    "tracking_error": error,
                    "tracking_error_degrees": {
                        name: math.degrees(value)
                        for name, value in error.items()
                        if name.endswith("arm")
                    },
                    "motor_effort_nm": {
                        name: [float(state.motor_state[i].tau_est) for i in ids]
                        for name, ids in GROUPS.items()
                        if name.endswith("arm")
                    },
                    "feedforward_nm": feedforward,
                    "feedback": self.robot.feedback_status(),
                    "within_tolerance": all(
                        value <= (0.05 if name.endswith("arm") else 0.1)
                        for name, value in error.items()
                    ),
                }
            )
            self.motion = None

    def wait(self, seconds=None, each=None, sleep=time.sleep):
        """Tick at 250 Hz until the motion (and a pending home) is complete, or for `seconds`.

        each() runs after every tick. The caller may have been busy since the last tick,
        so the cycle watchdog starts again here.
        """
        self.last_tick = None
        start = next_tick = self.clock()
        while True:
            self.tick()
            if each is not None:
                each()
            now = self.clock()
            if now - start >= seconds if seconds is not None else self.motion is None:
                return
            next_tick = max(next_tick + 0.004, now)
            sleep(max(0.0, next_tick - self.clock()))

    def status(self):
        actual = self.robot.measured_targets(self.robot.arm_state())
        return {
            "moving": self.motion["label"] if self.motion else None,
            "feedback_paused": self.height_pause_started is not None,
            "arm_feedback_paused": self.arm_pause_started is not None,
            "last_height_mm": self.last_height * 1000.0,
            "feedback": self.robot.feedback_status(),
            "target_height_mm": self.commanded["height"] * 1000.0,
            "arm_degrees": {
                k: [round(math.degrees(q), 2) for q in v]
                for k, v in actual.items()
                if k.endswith("arm")
            },
            "target_arm_degrees": {
                k: [round(math.degrees(q), 2) for q in v]
                for k, v in self.commanded["q"].items()
                if k.endswith("arm")
            },
            "gripper_percent": {
                k: round(v[0] / 5.4 * 100, 2) for k, v in actual.items() if k.endswith("gripper")
            },
        }
