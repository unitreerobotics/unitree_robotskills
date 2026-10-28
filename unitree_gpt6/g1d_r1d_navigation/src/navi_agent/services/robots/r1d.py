# -*- coding: utf-8 -*-
"""r1d.py -- R1D chassis velocity controller (rt/cmd_vel Twist normalised velocity + feedback guard).

Contract: rt/cmd_vel carries normalised [-1, 1] commands; if any of the three feedback streams
(rt/state_vel / rt/height_state / rt/height_alarm) is stale for 0.30 s or the alarm is abnormal,
zero speed is sent; close sends zero speed several times. Implementation notes:
  - Background publisher thread shaped like the G1D RobotController (instead of the caller
    driving move() at 50 Hz), so MotionExecutor / SequenceExecutor send_command streams and
    blending are reused as-is;
  - Physical -> normalised: forward = vx / mps_full, turn = turn_sign * vyaw / rps_full
    (Navi positive vyaw = left; R1D negative turn = left -> turn_sign=-1, to be verified on robot);
  - Fault latch: stale feedback / abnormal alarm / STALL while a non-zero command is active ->
    drop the command and latch a fault, keep sending zero speed. It unlatches only when feedback
    is healthy again AND the upper layer sends a zero command (stop); old motion never resumes
    by itself;
  - Lift is not exposed: lift is always 0 (NOT_HOMED/LOWER/UPPER are only recorded).
  - Odometry: command integration (same as G1D). Units/sign of rt/state_vel are unverified,
    so it is only passed through into health.
  - No SLAMWARE: acquire_navigation always refuses.
  - Heading hold (optional, active after attach_yaw_rate attaches a gyro, see
    core/heading_hold.py): each publisher tick corrects the turn component from the measured
    yaw rate, because wheels not starting aligned forward drag the heading (measured ~10 deg
    over 1 m straight). Odometry still integrates commands.
"""
from __future__ import annotations

import math
import sys
import threading
import time
from typing import Any

import config
from core.heading_hold import HeadingHold

try:
    from unitree_sdk2py.core.channel import (  # type: ignore[reportMissingImports]
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from unitree_sdk2py.idl.default import geometry_msgs_msg_dds__Twist_  # type: ignore[reportMissingImports]
    from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_, Twist_  # type: ignore[reportMissingImports]
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import Error_  # type: ignore[reportMissingImports]
    SDK_AVAILABLE = True
except ImportError:
    ChannelFactoryInitialize: Any = None
    ChannelPublisher: Any = None
    ChannelSubscriber: Any = None
    geometry_msgs_msg_dds__Twist_: Any = None
    Point32_: Any = None
    Twist_: Any = None
    Error_: Any = None
    SDK_AVAILABLE = False

T_CMD = "rt/cmd_vel"
T_VEL = "rt/state_vel"
T_HEIGHT = "rt/height_state"
T_ALARM = "rt/height_alarm"
ALARM_SOURCE = 200
NOT_HOMED, LOWER, UPPER, STALL = 1, 2, 4, 8
FEEDBACK_KEYS = ("velocity", "height", "alarm")
UNSUPPORTED = "unsupported_on_r1d: 无机载 SLAMWARE 导航"


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class R1DRobot:
    """R1D chassis with the same interface as services.robots.g1d.RobotController (send_command /
    dt / odom / connected / start_publisher / stop_publisher), so upper layers need not know the model.

    channels: test injection of two callables (publisher_write, subscribe) bypassing the SDK;
    publisher_write(forward, lift, turn) -> bool, subscribe(topic, cb) registers a feedback callback."""

    model = "r1d"

    def __init__(self, profile, net=None, domain_id=0, *, channels=None, now_fn=time.monotonic):
        self.profile = profile
        extra = profile.extra
        self.mps_full = float(extra.get("mps_full", 1.0))
        self.rps_full = float(extra.get("rps_full", 1.2))
        self.turn_sign = float(extra.get("turn_sign", -1.0))
        self.feedback_timeout_s = float(extra.get("feedback_timeout_s", 0.30))
        for name, v in (("mps_full", self.mps_full), ("rps_full", self.rps_full),
                        ("feedback_timeout_s", self.feedback_timeout_s)):
            if not (math.isfinite(v) and v > 0):
                raise ValueError(f"robot.r1d.{name} 须为正数")
        if self.turn_sign not in (-1.0, 1.0):
            raise ValueError("robot.r1d.turn_sign 须为 1 或 -1")
        self.max_vx = float(profile.max_vx)
        self.max_vyaw = float(profile.max_vyaw)
        self.dt = 1.0 / float(profile.rate_hz)
        self._now = now_fn

        self.cmd_lock = threading.Lock()
        self._fb_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self.current_cmd = None                    # (vx m/s, vyaw rad/s)
        self._last_cmd_time = None
        self._fault = None                         # latched fault reason
        self._notice = None                        # latest guard notice (NOT_HOMED etc., record only)
        self._values: dict[str, Any] = {}
        self._received: dict[str, float] = {}
        self.publish_running = False
        self.odom: dict[str, Any] = {"position": [0.0, 0.0, 0.0], "yaw": 0.0}
        self._odom_lock = threading.Lock()
        self._channels = []
        self._write = None
        self.heading: HeadingHold | None = None   # heading hold (set by attach_yaw_rate)
        self._yaw_rate = None                      # () -> (yaw rate rad/s, sample age s) | None
        self._heading_t = None                     # previous tick time: error integrates over the real tick interval (sleep can stretch the period)
        # Heading totals (accumulated per publisher tick): commanded yaw / gyro-measured yaw (rad) /
        # ticks in motion without a usable gyro. The difference of two readings gives "intended vs
        # actual turn" over that span (heading tracking in sequence results, see yaw_totals).
        self._yaw_cmd = self._yaw_gyro = 0.0
        self._yaw_gaps = 0

        if channels is not None:
            write, subscribe = channels
            self._write = write
            self._available = True
            for topic, cb in self._feedback_topics():
                subscribe(topic, cb)
            return
        if not SDK_AVAILABLE:
            self._available = False
            return
        self._available = True
        ChannelFactoryInitialize(domain_id, net or config.NET)
        try:
            pub = ChannelPublisher(T_CMD, Twist_)
            self._channels.append(pub)
            pub.Init()
            self._write = lambda f, lift, t: self._sdk_write(pub, f, lift, t)
            for (topic, cb), msg_type in zip(self._feedback_topics(), (Twist_, Point32_, Error_)):
                sub = ChannelSubscriber(topic, msg_type)
                self._channels.append(sub)
                sub.Init(cb)
        except BaseException:
            self._close_channels()
            raise

    # ---------- DDS ----------
    def _feedback_topics(self):
        return ((T_VEL, self._on_velocity), (T_HEIGHT, self._on_height), (T_ALARM, self._on_alarm))

    @staticmethod
    def _sdk_write(pub, forward, lift, turn) -> bool:
        msg = geometry_msgs_msg_dds__Twist_()
        msg.linear.x = forward
        msg.linear.z = lift
        msg.angular.z = turn
        return bool(pub.Write(msg))

    def _close_channels(self):
        for ch in reversed(self._channels):
            try:
                ch.Close()
            except Exception as exc:
                print(f"[r1d] DDS 关闭失败: {exc}", file=sys.stderr)
        self._channels.clear()

    def _update(self, name, value):
        with self._fb_lock:
            self._values[name] = value
            self._received[name] = self._now()

    def _on_velocity(self, m):
        self._update("velocity", (float(m.linear.x), float(m.linear.z), float(m.angular.z)))

    def _on_height(self, m):
        self._update("height", float(m.z))

    def _on_alarm(self, m):
        self._update("alarm", (int(m.source), int(m.state)))

    @property
    def available(self):
        return self._available

    @property
    def connected(self):
        return self._available and self._feedback_problem(self._now()) is None

    def wait_for_data(self, timeout_s=3.0):
        if not self._available:
            raise RuntimeError("unitree_sdk2py 不可用")
        deadline = time.monotonic() + timeout_s
        while True:
            with self._fb_lock:
                missing = [k for k in FEEDBACK_KEYS if k not in self._received]
            if not missing:
                return
            if time.monotonic() >= deadline:
                raise RuntimeError("等待 R1D 反馈超时: " + ", ".join(missing))
            time.sleep(0.05)

    def health(self) -> dict:
        now = self._now()
        with self._fb_lock:
            values = dict(self._values)
            ages = {k: round(now - t, 3) for k, t in self._received.items()}
        out = {"fault": self._fault, "feedback_problem": self._feedback_problem(now),
               "notice": self._notice, "feedback": values, "feedback_age_s": ages,
               "calibration": {"mps_full": self.mps_full, "rps_full": self.rps_full,
                               "turn_sign": self.turn_sign, "verified": False}}
        if self.heading is not None:
            out["heading_hold"] = self.heading.status()
        return out

    def attach_yaw_rate(self, source, log=None, **kw) -> None:
        """Attach a gyro (source() -> (yaw rate rad/s, + = left, sample age s) | None) and enable
        heading hold. kw is passed to HeadingHold (kp / max_corr / ...). Repeated calls only swap
        the source and keep the already estimated bias."""
        self._yaw_rate = source
        if self.heading is None:
            self.heading = HeadingHold(log=log, **kw)

    def measured_yaw_rate(self):
        """Gyro-measured yaw rate (rad/s, + = left, bias removed); None if no gyro is attached or
        it is untrustworthy right now. Discrete turns stop on the measured angle (core/motion.py)."""
        if self.heading is None or self._yaw_rate is None:
            return None
        try:
            return self.heading.measured_rate(self._yaw_rate())
        except Exception:
            return None

    def yaw_totals(self):
        """{"cmd": commanded yaw total, "gyro": measured yaw total (rad), "gaps": ticks in motion
        without a usable gyro}; None if no gyro is attached."""
        if self.heading is None:
            return None
        return {"cmd": self._yaw_cmd, "gyro": self._yaw_gyro, "gaps": self._yaw_gaps}

    # Feedback guard: stale feedback / abnormal alarm -> no motion allowed
    def _feedback_problem(self, now):
        """Reason feedback forbids motion; None = healthy. NOT_HOMED/LOWER/UPPER only affect the
        lift and are not blocked here."""
        with self._fb_lock:
            received = dict(self._received)
            values = dict(self._values)
        stale = [k for k in FEEDBACK_KEYS
                 if k not in received or now - received[k] >= self.feedback_timeout_s]
        if stale:
            return "feedback_stale: " + ", ".join(stale)
        source, state = values["alarm"]
        if source != ALARM_SOURCE:
            return f"alarm_source_invalid: {source}"
        if not all(math.isfinite(v) for v in (*values["velocity"], values["height"])):
            return "feedback_non_finite"
        if state & STALL:
            return "STALL"
        return None

    def _alarm_notice(self):
        with self._fb_lock:
            alarm = self._values.get("alarm")
        if not alarm:
            return None
        bits = [n for b, n in ((NOT_HOMED, "NotHomed"), (LOWER, "Lower"), (UPPER, "Upper"))
                if alarm[1] & b]
        return ("lift_disabled: " + ",".join(bits)) if bits else None

    def send_command(self, vx=0.0, vy=0.0, ang_z=0.0,
                     waist_yaw=0.0, waist_pitch=0.0, waist_roll=0.0, height=0.74):
        """Set chassis velocity (physical units). vy / waist_* / height exist for signature
        compatibility only and are ignored on R1D."""
        if not self._available:
            return
        vx, vyaw = float(vx), float(ang_z)
        if not (math.isfinite(vx) and math.isfinite(vyaw)):
            raise ValueError("非有限速度指令")
        vx = _clamp(vx, -self.max_vx, self.max_vx)
        vyaw = _clamp(vyaw, -self.max_vyaw, self.max_vyaw)
        with self.cmd_lock:
            if self._fault is not None:
                # While latched, drop non-zero commands; zero + healthy feedback = upper layer stopped, unlatch
                if vx == 0.0 and vyaw == 0.0 and self._feedback_problem(self._now()) is None:
                    self._fault = None
                else:
                    self.current_cmd = (0.0, 0.0)
                    self._last_cmd_time = self._now()
                    return
            self.current_cmd = (vx, vyaw)
            self._last_cmd_time = self._now()

    def _resolve_payload(self, now):
        """Same watchdog as G1D: zero after CMD_WATCHDOG_S, stop sending after a further
        ZERO_RELEASE_S (whether R1D holds the wheels is unverified). The feedback guard is then
        applied: unhealthy -> zero speed; with a non-zero command -> latch a fault."""
        with self.cmd_lock:
            cmd, last = self.current_cmd, self._last_cmd_time
        if cmd is None or last is None:
            return None
        idle = now - last
        if idle > config.CMD_WATCHDOG_S + config.ZERO_RELEASE_S:
            return None
        if idle > config.CMD_WATCHDOG_S or self._fault is not None:
            cmd = (0.0, 0.0)
        problem = self._feedback_problem(now)
        if problem is not None:
            if cmd != (0.0, 0.0):
                with self.cmd_lock:
                    self._fault = problem
                    self.current_cmd = (0.0, 0.0)
                print(f"[r1d] 反馈守卫锁存: {problem}", file=sys.stderr)
            return (0.0, 0.0)
        self._notice = self._alarm_notice()
        return cmd

    def normalize(self, vx, vyaw):
        """Physical units -> R1D normalised (forward, lift, turn), each clamped to [-1, 1]; lift is always 0."""
        return (_clamp(vx / self.mps_full, -1.0, 1.0), 0.0,
                _clamp(self.turn_sign * vyaw / self.rps_full, -1.0, 1.0))

    def _publish(self, vx, vyaw) -> bool:
        with self._write_lock:
            if self._write is None:
                return False
            return bool(self._write(*self.normalize(vx, vyaw)))

    def _integrate_odom(self, vx, vyaw, dt):
        with self._odom_lock:
            yaw = self.odom["yaw"] + vyaw * dt
            self.odom["yaw"] = yaw
            self.odom["position"][0] += vx * dt * math.cos(yaw)
            self.odom["position"][1] += vx * dt * math.sin(yaw)

    def _heading_payload(self, payload):
        """Heading-hold correction of the turn component. When stopped (None / zero speed) samples
        still feed the bias estimate, but no correction is ever output."""
        if self.heading is None:
            return payload
        try:
            sample = self._yaw_rate() if self._yaw_rate is not None else None
        except Exception:
            sample = None
        now = self._now()
        dt = self.dt if self._heading_t is None else min(max(now - self._heading_t, 0.0), 0.1)
        self._heading_t = now
        vx, vyaw = payload if payload is not None else (0.0, 0.0)
        rate = self.heading.measured_rate(sample)
        if rate is not None:
            self._yaw_cmd += vyaw * dt
            self._yaw_gyro += rate * dt
        elif vx != 0.0 or vyaw != 0.0:
            self._yaw_gaps += 1
        out = self.heading.step(vx, vyaw, sample, dt)
        if payload is None or payload == (0.0, 0.0):
            return payload
        return vx, _clamp(out, -self.max_vyaw, self.max_vyaw)

    def _tick(self):
        payload = self._resolve_payload(self._now())
        sent = self._heading_payload(payload)
        if payload is None:
            return
        try:
            if self._publish(*sent):
                self._integrate_odom(*payload, self.dt)
            elif payload != (0.0, 0.0):
                with self.cmd_lock:
                    self._fault = "dds_write_failed"
                    self.current_cmd = (0.0, 0.0)
        except Exception as exc:
            with self.cmd_lock:
                self._fault = f"dds_write_failed: {exc}"
                self.current_cmd = (0.0, 0.0)

    def _publish_loop(self):
        while self.publish_running:
            self._tick()
            time.sleep(self.dt)

    def acquire_navigation(self):
        raise RuntimeError(UNSUPPORTED)

    def navigation_zero(self):
        raise RuntimeError(UNSUPPORTED)

    def release_navigation(self):
        pass

    def start_publisher(self):
        if not self._available or self.publish_running:
            return
        self.publish_running = True
        self._publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._publish_thread.start()

    def stop_publisher(self):
        """Stop the publisher thread, send zero speed 5 times (DDS success does not mean physically
        stopped), then close channels."""
        self.publish_running = False
        if hasattr(self, "_publish_thread"):
            self._publish_thread.join(timeout=1.0)
        if self._available:
            for _ in range(5):
                try:
                    self._publish(0.0, 0.0)
                except Exception as exc:
                    print(f"[r1d] 停车发送失败: {exc}；请使用实体急停", file=sys.stderr)
                time.sleep(self.dt)
        with self._write_lock:
            self._write = None
        self._close_channels()
