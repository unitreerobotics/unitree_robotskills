# -*- coding: utf-8 -*-
"""heading_hold.py -- chassis heading closed loop: make actual yaw rate track the command
(pure logic, unit-testable offline).

Why: R1D wheels are not necessarily aligned forward at start. Measured: driving 1 m
straight at 0.5 m/s drifts the heading ~10 deg (lidar localization; mostly within the
first 0.5 m), and the open-loop timed turn does only ~84 deg for a 90 deg command.

How: integrate (commanded yaw rate - gyro yaw rate) into heading error err
(= commanded heading - actual heading) and output vyaw = cmd vyaw + kp*err (correction
clamped to max_corr, error clamped to max_err). Driving straight, err is the drift angle
and gets steered back; turning, the actual angle tracks the commanded one.

Safety rules (each prefers falling back to open loop over steering wrongly):
  - Zero command (stopped): never output a correction, reset err -- stop means stop.
  - Gyro bias is estimated only after still_s of stillness, and used only after min_bias_s
    accumulated; sudden rate jumps while still (pushed by hand) are excluded.
  - Direction is verified before enabling: in the first turn with |vyaw| >= verify_min_rate,
    gyro and command must agree for verify_s. Disagreement for verify_s disables permanently
    (with wrong mounting/sign, open loop at least does not steer further off).
  - IMU dropout (sample older than max_age_s): open loop this tick, err reset.
  - Error saturated at its limit for saturate_s (stuck / cannot keep up): suspended for the
    rest of this motion, resumes after stopping.

enabled=False disables only the correction: bias estimation and direction verification
continue, and the gyro can still measure turn angles (tools/heading_check.py compares on/off).
"""
from __future__ import annotations


def _clamp(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))


class HeadingHold:
    """step() is called once per tick by the chassis publisher thread; sample = (yaw rate rad/s, +left, sample age s) or None."""

    def __init__(self, *, kp: float = 1.5, max_corr: float = 0.25, max_err: float = 0.35,
                 still_s: float = 1.0, bias_tau_s: float = 2.0, min_bias_s: float = 1.0,
                 bias_jump: float = 0.03, max_age_s: float = 0.15, verify_min_rate: float = 0.15,
                 verify_s: float = 0.4, saturate_s: float = 1.5, log=None):
        self.kp, self.max_corr, self.max_err = float(kp), float(max_corr), float(max_err)
        self.still_s, self.bias_tau_s, self.min_bias_s = float(still_s), float(bias_tau_s), float(min_bias_s)
        self.bias_jump, self.max_age_s = float(bias_jump), float(max_age_s)
        self.verify_min_rate, self.verify_s, self.saturate_s = float(verify_min_rate), float(verify_s), float(saturate_s)
        self._log = log or (lambda *a, **k: None)
        self.bias: float | None = None
        self.bias_s = 0.0             # accumulated still time used for bias estimation
        self.err = 0.0                # commanded heading - actual heading (rad)
        self.verified = False
        self.disabled: str | None = None      # reason for permanent disable
        self.suspended = False                # suspended for this motion (error saturated); resumes after stop
        self.enabled = True                   # False = measure only, no correction (A/B testing)
        self.imu_ok = False
        self.last_corr = 0.0
        self._still_t = 0.0
        self._agree_t = self._oppose_t = self._sat_t = 0.0

    @property
    def state(self) -> str:
        if self.disabled:
            return "disabled"
        if not self.imu_ok:
            return "no_imu"
        if self.bias_s < self.min_bias_s:
            return "calibrating"
        if not self.verified:
            return "verifying"
        if not self.enabled:
            return "off"
        return "suspended" if self.suspended else "active"

    def status(self) -> dict:
        return {"state": self.state, "reason": self.disabled,
                "bias_rps": None if self.bias is None else round(self.bias, 4),
                "err_deg": round(self.err * 57.29578, 1), "corr_rps": round(self.last_corr, 3)}

    def measured_rate(self, sample):
        """Bias-corrected gyro yaw rate (rad/s, +left). None if bias not ready, direction not
        verified, permanently disabled, or sample stale (reading not trustworthy). Side-effect
        free; used for measuring turn angles and heading drift."""
        if self.disabled or not self.verified or self.bias is None or self.bias_s < self.min_bias_s:
            return None
        if sample is None or sample[1] is None or sample[1] > self.max_age_s:
            return None
        return float(sample[0]) - self.bias

    def step(self, vx: float, vyaw: float, sample, dt: float) -> float:
        """Return the yaw rate to send this tick (before chassis limits)."""
        self.last_corr = 0.0
        moving = abs(vx) > 1e-6 or abs(vyaw) > 1e-6
        if sample is None or sample[1] is None or sample[1] > self.max_age_s:
            self.imu_ok = False
            self.err = 0.0
            self._still_t = 0.0 if moving else self._still_t
            return vyaw
        self.imu_ok = True
        rate = float(sample[0])
        if not moving:
            self.err, self._sat_t, self.suspended = 0.0, 0.0, False
            self._still_t += dt
            if self._still_t >= self.still_s:
                if self.bias is None:
                    self.bias = rate
                elif abs(rate - self.bias) <= self.bias_jump:          # rotating while still (pushed): exclude from estimate
                    self.bias += (rate - self.bias) * min(1.0, dt / self.bias_tau_s)
                self.bias_s += dt
            return vyaw
        self._still_t = 0.0
        if self.disabled or self.bias is None or self.bias_s < self.min_bias_s:
            return vyaw
        omega = rate - self.bias
        if not self.verified:
            self._verify(vyaw, omega, dt)
            return vyaw
        if self.suspended or not self.enabled:
            return vyaw
        self.err = _clamp(self.err + (vyaw - omega) * dt, self.max_err)
        self._sat_t = self._sat_t + dt if abs(self.err) >= self.max_err else 0.0
        if self._sat_t >= self.saturate_s:
            self.suspended, self.err = True, 0.0
            self._log(f"航向闭环：误差顶在 ±{self.max_err * 57.3:.0f}° 超过 {self.saturate_s:.1f}s（卡住或跟不上），"
                      "本次运动改开环，停车后恢复", "motion")
            return vyaw
        self.last_corr = _clamp(self.kp * self.err, self.max_corr)
        return vyaw + self.last_corr

    def _verify(self, vyaw: float, omega: float, dt: float) -> None:
        """Verify gyro sign during open-loop turns: agree for verify_s -> enable; oppose for verify_s -> disable permanently."""
        if abs(vyaw) < self.verify_min_rate:
            return
        if omega * vyaw > 0 and abs(omega) >= 0.3 * abs(vyaw):
            self._agree_t += dt
        elif omega * vyaw < 0 and abs(omega) >= 0.3 * abs(vyaw):
            self._oppose_t += dt
        if self._agree_t >= self.verify_s:
            self.verified = True
            self._log("航向闭环：陀螺方向核对通过，启用", "motion")
        elif self._oppose_t >= self.verify_s:
            self.disabled = "陀螺实测转向与指令相反（安装外参或符号有误），航向闭环已关闭"
            self._log(self.disabled, "motion")
