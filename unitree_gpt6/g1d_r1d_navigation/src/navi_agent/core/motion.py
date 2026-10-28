# -*- coding: utf-8 -*-
"""motion.py -- discrete motion executor.

- Jog actions (forward/backward, turn in place) with ramp, duration and distance feedback.
- Single-flight: only one action at a time (a new one cancels the previous).
- Depends only on the robot controller (services/robots) and config.
- If the chassis provides measured_yaw_rate() (R1D with lidar IMU): turns stop on the
  gyro-measured angle and straight moves record gyro heading change. Without a gyro, turns
  stop on the integrated command, compensating for the start ramp (otherwise every turn
  falls ~6 deg short).

Usage:
    ex = MotionExecutor(robot)
    ex.move_linear_sync(1.0, 0.0, speed=0.3, distance=0.5)
    ex.turn_angle_sync(math.radians(90), turn_speed=0.5)
    ex.stop()          # cancel current action and send zero speed
"""

import math
import threading
import time

import config


class _GyroAngle:
    """Integrate gyro yaw angle during an action (rad, +left). value=None if the gyro is unavailable
    at start; also invalidated if it drops out midway, so a partial integral never poses as a measurement."""

    def __init__(self, read):
        self._read = read
        self._t = time.monotonic()
        self.rate = read()
        self.value = None if self.rate is None else 0.0

    def update(self):
        now = time.monotonic()
        dt, self._t = now - self._t, now
        if self.value is None:
            return None
        self.rate = self._read()
        if self.rate is None:
            self.value = None
        else:
            self.value += self.rate * dt
        return self.rate


class MotionExecutor:
    """Discrete motion: ramped start, timed stop, cancellable (single-flight)."""

    def __init__(self, robot, log=None):
        self.robot = robot
        self._log = log or (lambda *a, **k: None)   # optional log callback (msg, kind)
        # Each action owns its own cancel event: a cancelled action keeps reading its (still set)
        # event while the new one gets a fresh event. This avoids the race where clearing a shared
        # event after a join timeout revives the old thread and it resumes sending commands.
        self._cancel = threading.Event()
        self._thread = None
        self._last_moved = 0.0                      # command-integrated distance of the last action
        self._last_turned = 0.0                     # command-integrated angle of the last turn, rad (open loop)
        # Reflex hook (same as SequenceExecutor.speed_limiter): clamps vx every tick of linear
        # motion; blocked for longer than blocked_abort_s aborts the action with status "blocked".
        self.speed_limiter = None
        self.blocked_abort_s = 2.0
        self._last_blocked = False
        # Gyro (robot.measured_yaw_rate): gyro_turns=False forces command-integrated turn stop (A/B tests).
        # turn_stop_lead_s: when stopping on measured angle, stop early by rate x this lead to
        # absorb post-stop inertial coasting.
        self.gyro_turns = True
        self.turn_stop_lead_s = 0.1
        self._last_measured = None                  # gyro-measured angle of last turn, rad (incl. coasting); None without gyro
        self._last_turn_mode = None                 # gyro = stop on measured angle / command = stop on integrated command
        self._last_turn_stop = None                 # done / timeout (measured angle never reached)
        self._last_yaw_change = None                # gyro heading change during last straight move, rad; None without gyro

    def is_moving(self):
        return self._thread is not None and self._thread.is_alive()

    # Blocking API (CLI / MCP): the action runs on a background thread; we join and return
    # {"status": "done|cancelled|timeout|blocked", "moved_m": float}.
    def move_linear_sync(self, vx, vy, speed=None, distance=None, timeout_s=None) -> dict:
        """Blocking straight move until distance or timeout; returns {"status", "moved_m"}."""
        if self.robot is None:
            return {"status": "done", "moved_m": 0.0}
        speed = config.MOVE_SPEED if speed is None else speed
        distance = config.MOVE_DISTANCE if distance is None else distance
        res = self._run_blocking(self._run_linear,
                                 (vx, vy, "同步移动", speed, distance), timeout_s)
        res["yaw_change_rad"] = None if self._last_yaw_change is None else round(self._last_yaw_change, 4)
        return res

    def turn_angle_sync(self, angle_rad, turn_speed=None, timeout_s=None) -> dict:
        """Turn in place by angle; nominal duration = |angle_rad| / turn_speed. Returns {"status", "moved_m": 0.0, "turned_rad"}."""
        if self.robot is None:
            return {"status": "done", "moved_m": 0.0, "turned_rad": 0.0}
        turn_speed = config.TURN_ANGLE if turn_speed is None else turn_speed
        res = self._run_blocking(self._run_turn_angle,
                                 (angle_rad, "同步转向", turn_speed), timeout_s)
        res["turned_rad"] = round(self._last_turned, 4)
        res["measured_rad"] = None if self._last_measured is None else round(self._last_measured, 4)
        res["turn_mode"] = self._last_turn_mode
        if self._last_turn_stop == "timeout":
            res["turn_stop"] = "timeout"
        return res

    def stop(self):
        """Cancel the current action and send zero speed."""
        self._cancel.set()
        if self.robot is not None:
            self.robot.send_command(0.0)

    def _run_blocking(self, target, args, timeout_s=None) -> dict:
        """Run one action as a blocking call: join until done/timeout, return status and distance."""
        self._last_moved = 0.0
        self._last_turned = 0.0
        self._last_measured = self._last_yaw_change = None
        self._last_turn_mode = self._last_turn_stop = None
        # _last_blocked must be reset too: only _run_linear sets it, so otherwise a turn after a
        # blocked forward move would falsely report "blocked".
        self._last_blocked = False
        self._start(target, *args)
        t = self._thread
        cancel = self._cancel               # this action's own event (_start replaced it)
        t.join(timeout=timeout_s)
        if t.is_alive():
            cancel.set()           # stop the worker now (its finally still sends zero speed)
            return {"status": "timeout", "moved_m": round(self._last_moved, 3)}
        if self._last_blocked:
            return {"status": "blocked", "moved_m": round(self._last_moved, 3)}
        return {"status": "cancelled" if cancel.is_set() else "done",
                "moved_m": round(self._last_moved, 3)}

    def _start(self, target, *args):
        """Single-flight: cancel the previous action and start a new thread with its own cancel event."""
        if self._thread is not None and self._thread.is_alive():
            self._cancel.set()
            # wait for the old thread to exit; timeout covers its finally STOP_HOLD_TIME zero-speed hold
            self._thread.join(timeout=2.0)
        cancel = threading.Event()
        self._cancel = cancel
        self._thread = threading.Thread(target=target, args=(*args, cancel), daemon=True)
        self._thread.start()

    def _run_linear(self, vx, vy, description, speed, distance, cancel):
        move_time = distance / speed if speed > 0 else 0
        if speed <= 0:
            self._log(f"[{description}] 速度非正（speed={speed}），跳过运动", "error")
        self._log(f"下发指令 [{description}] vx={vx * speed:.2f} "
                  f"时长≈{move_time:.1f}s", "motion")
        move_start = time.monotonic()
        moved = 0.0
        gyro = _GyroAngle(self._gyro)
        self._last_blocked = False
        blocked_t = None
        paused = 0.0                               # time paused while blocked (excluded from travel time)
        try:
            while not cancel.is_set():
                elapsed = time.monotonic() - move_start - paused
                if elapsed >= move_time:
                    break
                factor = elapsed / config.RAMP_TIME if elapsed < config.RAMP_TIME else 1.0
                # vx is a unit direction (+-1.0): command = direction x ramp x speed (never amplified beyond nominal)
                cmd_vx = vx * factor * speed
                if self.speed_limiter is not None:
                    cmd_vx, _, info = self.speed_limiter(cmd_vx, 0.0)
                    if info.get("reason") in ("blocked", "no_data"):
                        blocked_t = blocked_t or time.monotonic()
                        paused += self.robot.dt
                        if time.monotonic() - blocked_t >= self.blocked_abort_s:
                            self._last_blocked = True
                            self._log(f"[{description}] 雷达反射：前方 {info.get('clear_m')} m 被挡 "
                                      f"{self.blocked_abort_s:.0f}s，中止", "motion")
                            break
                    else:
                        blocked_t = None
                # Wheeled chassis cannot strafe: vy is only a direction convention, always send 0
                sent_at = time.monotonic()
                self.robot.send_command(cmd_vx, 0.0)
                time.sleep(self.robot.dt)
                moved += abs(cmd_vx) * (time.monotonic() - sent_at)            # open-loop estimate (no measured odometry here)
                gyro.update()
        finally:
            self.robot.send_command(0.0)
            self._hold_stop(gyro)
            self._last_moved = moved
            self._last_yaw_change = gyro.value
            drift = "" if gyro.value is None else f"，陀螺测得航向变化 {math.degrees(gyro.value):+.1f}°"
            self._log(f"完成 [{description}]，里程 {moved:.3f}m（开环）{drift}", "motion")

    def _gyro(self):
        """Bias-corrected chassis gyro yaw rate (rad/s); None if there is no gyro or it is not trustworthy now."""
        meter = getattr(self.robot, "measured_yaw_rate", None)
        if meter is None:
            return None
        try:
            return meter()
        except Exception:
            return None

    def _hold_stop(self, gyro):
        """Hold STOP_HOLD_TIME after stopping (zero already sent) while still integrating the gyro, so post-stop coasting is measured."""
        end = time.monotonic() + config.STOP_HOLD_TIME
        while True:
            left = end - time.monotonic()
            if left <= 0:
                break
            time.sleep(min(self.robot.dt, left))
            if gyro is not None:
                gyro.update()

    def _run_turn_angle(self, ang_rad, description, turn_speed, cancel):
        """Turn by angle: with a usable gyro (and gyro_turns), stop when the measured angle is reached;
        otherwise stop on the integrated command (still compensating for the start ramp). If the measured
        angle is never reached (stuck / slipping), stop on timeout = expected duration x 1.5 + 1 s."""
        target = abs(ang_rad)
        speed = max(float(turn_speed or config.TURN_ANGLE), 1e-3)
        direction = 1.0 if ang_rad > 0 else -1.0
        gyro = _GyroAngle(self._gyro)
        mode = "gyro" if self.gyro_turns and gyro.value is not None else "command"
        expect = target / speed + config.RAMP_TIME / 2          # add back the half RAMP_TIME lost to the ramp
        limit = expect * 1.5 + 1.0
        self._log(f"下发指令 [{description}] 角度={math.degrees(ang_rad):.0f}° 时长≈{expect:.1f}s"
                  f"（{'按陀螺实测转角停' if mode == 'gyro' else '按指令积分停'}）", "motion")
        move_start = time.monotonic()
        turned = 0.0
        stop = "done"
        try:
            while not cancel.is_set():
                elapsed = time.monotonic() - move_start
                if elapsed >= limit:
                    stop = "timeout"
                    break
                rate_now = gyro.update()
                if mode == "gyro" and gyro.value is None:
                    mode = "command"                                    # gyro dropped out: finish on integrated command
                if mode == "gyro":
                    if abs(gyro.value) >= target - abs(rate_now or 0.0) * self.turn_stop_lead_s:
                        break
                elif turned >= target - 1e-6:
                    break
                factor = elapsed / config.RAMP_TIME if elapsed < config.RAMP_TIME else 1.0
                rate = speed * factor
                if mode == "command":
                    rate = min(rate, (target - turned) / self.robot.dt)   # last tick covers only the remainder, no overshoot
                sent_at = time.monotonic()
                self.robot.send_command(ang_z=direction * rate)
                time.sleep(self.robot.dt)
                turned += rate * (time.monotonic() - sent_at)            # command integral (gyro tracked separately)
        finally:
            self.robot.send_command(0.0)
            self._hold_stop(gyro)
            self._last_moved = 0.0
            self._last_turned = turned
            self._last_measured = gyro.value
            self._last_turn_mode = mode
            self._last_turn_stop = stop
            measured = ("（开环）" if gyro.value is None
                        else f"，陀螺实测 {math.degrees(gyro.value):+.0f}°{'（实测迟迟不到，超时停）' if stop == 'timeout' else ''}")
            self._log(f"完成 [{description}]，指令转角 {math.degrees(turned):.0f}°{measured}", "motion")
