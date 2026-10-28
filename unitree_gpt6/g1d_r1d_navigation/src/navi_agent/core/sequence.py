# -*- coding: utf-8 -*-
"""sequence.py -- sequence executor (lower-level controller for continuous motion mode).

Each inference of the upper layer (long-context agent) emits a time-parameterized velocity
sequence [(vx, vyaw, duration), ...] and immediately plans the next one from a new frame
without waiting for execution. This module:

- Samples sequences into a discrete timeline at control period dt and sends one command per tick.
- New sequences start at receipt by default (alignment=current), so a new correction is never
  trimmed by time the old plan spent driving straight. Only explicit continue, with old plan and
  new prefix being the same constant command, allows trimming the head; then merge:
    same direction -> linear blend over the overlap (new weight rises to 1), smooth speed change;
    conflict (reversal / old moving, new stopping) -> new sequence takes over immediately (no
    averaging: the averaged state neither stops nor turns).
- Watchdog: upper layer silent for watchdog_s while a sequence runs -> clear and ramp stop.
- Ramp limiting: approach the target at the accel limit each tick; stop and protective limits
  may drop to zero immediately.
- Two survival rules when the reflex layer (speed_limiter) reports blocked (from a real
  wall-collision post-mortem):
  a blocked command with a forward component also brakes the turn (a blocked arc must not
  degrade into turning in place against a wall -- the body sides have no sensors);
  blocked longer than blocked_abort_s -> remaining plan is dropped (the model takes tens of
  seconds to react; waiting for it means pushing against the obstacle for half a minute).
  Pure in-place turn segments are not clamped, matching the discrete turn tool, leaving an
  escape route to rotate out when trapped.

No MCP dependency; now_fn is injectable, and with start_thread=False tests call tick() manually.
"""
from __future__ import annotations

import threading
import time

import numpy as np

# Zero-speed threshold (m/s or rad/s): below this, ignored in direction conflict checks
ZERO_EPS = 0.02


def _sgn(v: float) -> int:
    return 0 if abs(v) < ZERO_EPS else (1 if v > 0 else -1)


def is_conflict(old_cmd, new_cmd) -> bool:
    """Whether the old timeline's current tick conflicts with the new sequence's first tick
    (conflict -> hard takeover, no blending).

    Conflict = linear reversal / angular reversal / old moving while new stops.
    Old still, new starting is not a conflict (the blend acts as a start ramp); old straight,
    new adding a turn is not either (fading the turn in is acceptable smoothing)."""
    ox, ow = old_cmd
    nx, nw = new_cmd
    if _sgn(ox) * _sgn(nx) < 0 or _sgn(ow) * _sgn(nw) < 0:
        return True
    old_moving = bool(_sgn(ox) or _sgn(ow))
    new_stop = not (_sgn(nx) or _sgn(nw))
    return old_moving and new_stop


def sample_segments(segments, dt: float) -> np.ndarray:
    """[(vx, vyaw, duration), ...] -> (N, 2) timeline sampled at dt; at least 1 tick per segment."""
    rows = []
    for vx, vyaw, dur in segments:
        n = max(1, int(round(float(dur) / dt)))
        rows.append(np.repeat([[float(vx), float(vyaw)]], n, axis=0))
    if not rows:
        return np.zeros((0, 2))
    return np.vstack(rows)


def merge_timelines(old: np.ndarray, new: np.ndarray, mode: str, n_blend: int) -> np.ndarray:
    """Merge by mode: fresh/replace use new as-is; blend cross-fades linearly over the first n ticks.

    Blend window = min(n_blend, len(old), len(new)): if old is shorter, blend only to its end and
    then follow new, so old "zeros" never slow the new sequence. Any old tail beyond new is
    dropped: the newer prediction supersedes the old plan and its end is the stop point."""
    out = new.copy()
    if mode != "blend" or len(old) == 0 or len(new) == 0:
        return out
    n = min(n_blend, len(old), len(new))
    if n <= 0:
        return out
    w = np.linspace(1.0 / n, 1.0, n)[:, None]
    out[:n] = (1.0 - w) * old[:n] + w * new[:n]
    return out


class SequenceExecutor:
    """Timeline executor: submit merges sequences; a control thread sends per tick (or tests call tick)."""

    def __init__(self, robot, *, dt=None, blend_s=1.0, watchdog_s=8.0,
                 ramp_time=0.5, max_vx=0.3, max_vyaw=0.5, max_accel=None,
                 now_fn=time.monotonic, log=None, start_thread=True,
                 speed_limiter=None):
        self.robot = robot
        # Reflex hook: (vx, vyaw) -> (vx', vyaw', info). info["reason"] == "blocked"/"no_data" with a
        # forward component marks this tick blocked: turn brakes with linear speed and the timeline
        # pauses (_t0 shifted by one tick), resuming once clear; blocked continuously longer than
        # blocked_abort_s drops the remaining plan.
        self.speed_limiter = speed_limiter
        self.blocked_abort_s = 2.0          # same semantics as MotionExecutor
        self.aborted = False                # last clear was caused by blocked auto-abort
        self.reflex = {"reason": "ok", "clear_m": None, "blocked_s": 0.0}
        self.dt = float(dt if dt is not None else getattr(robot, "dt", 0.02))
        self.now_fn = now_fn
        self._log = log or (lambda *a, **k: None)
        self._n_blend = max(1, int(round(blend_s / self.dt)))
        self._watchdog_s = float(watchdog_s)
        # Ramp limit: max change per tick = (cap / ramp_time) x dt, same as MotionExecutor's RAMP_TIME.
        # If max_accel (m/s^2) is given, linear speed uses it instead so raising max_vx to 1 m/s
        # does not double the start acceleration.
        rt = max(float(ramp_time), 1e-3)
        self._step_vx = (float(max_accel) if max_accel else max_vx / rt) * self.dt
        self._step_vyaw = max_vyaw / rt * self.dt

        self._lock = threading.Lock()
        self._timeline = np.zeros((0, 2))   # (vx, vyaw) sampled at dt from _t0
        self._t0 = 0.0
        self._nz_cum = np.zeros(1, int)     # prefix count of non-zero ticks (motion clock counts only these)
        self._motion_s = 0.0                # motion clock: executed seconds of finished/replaced timelines
        self._last_submit = None
        self._cur = [0.0, 0.0]              # last command actually sent (after ramp)
        self._idle_sent = True              # zero speed already sent after timeline ended
        self.starved = False                # last clear was caused by the watchdog
        self.submit_count = 0

        self._running = False
        self._thread = None
        if start_thread:
            self._running = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def motion_clock(self, now=None) -> float:
        """Accumulated seconds of planned non-zero commands; not measured motion or distance.

        Used by the restricted continue alignment; current never trims. Zero-speed ticks are not
        counted and lidar blocking pauses the plan. Do not infer frame freshness or actual
        robot position from it."""
        now = self.now_fn() if now is None else now
        with self._lock:
            return self._motion_s + self._consumed_locked(now)

    def _consumed_locked(self, now) -> float:
        """Consumed seconds of non-zero planned commands in the current timeline (zero ticks excluded)."""
        if len(self._timeline) == 0:
            return 0.0
        idx = min(int((now - self._t0) / self.dt + 1e-6), len(self._timeline))
        return float(self._nz_cum[max(0, idx)]) * self.dt

    def _fold_consumed_locked(self, now):
        """Before replacing/clearing the timeline, add its executed part to the motion clock."""
        self._motion_s += self._consumed_locked(now)

    def _set_timeline_locked(self, tl: np.ndarray, now: float):
        self._timeline = tl
        self._t0 = now
        nz = (np.abs(tl) > ZERO_EPS).any(axis=1) if len(tl) else np.zeros(0, bool)
        self._nz_cum = np.concatenate([[0], np.cumsum(nz)])

    def submit(self, segments, frame_ts=None, frame_motion_t=None, alignment="current") -> dict:
        """Submit a new sequence. segments: [(vx, vyaw, duration), ...].

        current (default) keeps the whole new plan. continue trims the head by the planned
        non-zero command time only when the constant-speed history and new prefix match;
        frame_motion_t is a command-time counter, not measured odometry.
        frame_ts is used for latency stats and to confirm no other plan was submitted after the frame."""
        now = self.now_fn()
        offset = max(0.0, now - frame_ts) if frame_ts is not None else 0.0
        new = sample_segments(segments, self.dt)
        if len(new) == 0:
            return {"ok": False, "error": "序列为空"}
        with self._lock:
            driven = (max(0.0, self._motion_s + self._consumed_locked(now) - frame_motion_t)
                      if frame_motion_t is not None else 0.0)
        if alignment not in ("current", "continue"):
            return {"ok": False, "error": "alignment 须为 current 或 continue"}
        # Default: correction plans start NOW. Never trim away a newly requested turn
        # merely because the OLD plan drove straight during inference.
        skip = int(round(driven / self.dt)) if alignment == "continue" else 0
        if skip:
            with self._lock:
                # Conservative continuation: only a single uniform command, identical
                # to this entire old timeline, and no intervening submit since the frame.
                # General path retiming needs actual localization, not a seconds counter.
                old_tl = self._timeline
                safe = (frame_ts is not None and self._last_submit is not None
                        and frame_ts >= self._last_submit and len(old_tl) > 0
                        and np.allclose(old_tl, old_tl[0])
                        and np.allclose(new[:min(skip, len(new))], old_tl[0]))
            if not safe:
                return {"ok": False, "error": "continue 无法确认历史动作一致；请按当前状态重新规划并用 current"}
        if skip >= len(new):
            # Since the frame, the old plan has run longer than the new sequence: it lies entirely in the past
            return {"ok": False, "error": "序列已过期：拍帧后非零计划已消耗 %.1fs ≥ 规划时长 %.1fs，"
                                          "请按 suggested_horizon_s 放长序列"
                                          % (driven, len(new) * self.dt),
                    "offset_s": round(offset, 2), "driven_s": round(driven, 2),
                    "planned_s": round(len(new) * self.dt, 2)}
        new = new[skip:]
        with self._lock:
            old = self._remaining_locked(now)
            self._fold_consumed_locked(now)
            if len(old) == 0:
                mode = "fresh"
            elif is_conflict(tuple(old[0]), tuple(new[0])):
                mode = "replace"
            else:
                mode = "blend"
            self._set_timeline_locked(merge_timelines(old, new, mode, self._n_blend), now)
            self._last_submit = now
            self._idle_sent = False
            self.starved = False
            self.aborted = False
            self.submit_count += 1
            total = len(self._timeline) * self.dt
        self._log(f"序列提交：{mode}，时延 {offset:.1f}s，裁头 {skip * self.dt:.2f}s，剩余 {total:.1f}s", "motion")
        return {"ok": True, "mode": mode, "offset_s": round(offset, 2),
                "driven_s": round(driven, 2), "dropped_s": round(skip * self.dt, 2),
                "remaining_s": round(total, 2), "alignment": alignment}

    def set_watchdog(self, seconds: float):
        """Adjust the watchdog at runtime (upper layer widens it per measured latency so slow models are not starved every step)."""
        with self._lock:
            self._watchdog_s = max(0.5, float(seconds))

    def stop(self):
        """Clear the timeline and send zero immediately (highest priority, no ramp)."""
        with self._lock:
            now = self.now_fn()
            self._fold_consumed_locked(now)
            self._set_timeline_locked(np.zeros((0, 2)), now)
            self._cur = [0.0, 0.0]
            self._idle_sent = True
            self._send(0.0, 0.0)

    def status(self) -> dict:
        now = self.now_fn()
        with self._lock:
            rem = self._remaining_locked(now)
            cur = list(self._cur)
            since = (now - self._last_submit) if self._last_submit is not None else None
        return {"active": len(rem) > 0,
                "remaining_s": round(len(rem) * self.dt, 2),
                "cmd": {"vx": round(cur[0], 3), "vyaw": round(cur[1], 3)},
                "since_submit_s": None if since is None else round(since, 2),
                "starved": self.starved,
                "aborted": self.aborted,
                "reflex": dict(self.reflex)}

    def close(self):
        self._running = False
        t = self._thread
        if t is not None and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=1.0)
        self.stop()

    def tick(self):
        """Advance one tick: watchdog -> target -> ramp limit -> send."""
        now = self.now_fn()
        with self._lock:
            rem = self._remaining_locked(now)
            if len(rem) > 0 and self._last_submit is not None \
                    and now - self._last_submit > self._watchdog_s:
                self._fold_consumed_locked(now)
                self._set_timeline_locked(np.zeros((0, 2)), now)
                rem = self._timeline
                self.starved = True
                self._log(f"序列看门狗：上层静默 {self._watchdog_s:.0f}s，斜坡停", "motion")
            target = (float(rem[0][0]), float(rem[0][1])) if len(rem) > 0 else (0.0, 0.0)
            active = len(rem) > 0
            if self.speed_limiter is not None and (active or abs(self._cur[0]) > 1e-6):
                tvx, tvyaw, info = self.speed_limiter(target[0], target[1])
                self.reflex = info
                if info.get("reason") in ("blocked", "no_data") and abs(target[0]) > ZERO_EPS:
                    tvyaw = 0.0          # blocked forward also brakes the turn: a blocked arc must not become a turn against the wall
                    if active:
                        self._t0 += self.dt      # pause the plan: this tick did not move, do not consume the timeline
                        if info.get("blocked_s", 0.0) >= self.blocked_abort_s:
                            # the model takes tens of seconds to react; do not keep pushing against the obstacle
                            self._fold_consumed_locked(now)
                            self._set_timeline_locked(np.zeros((0, 2)), now)
                            rem = self._timeline
                            active = False
                            self.aborted = True
                            self._log(f"序列被拦 {info['blocked_s']:.1f}s 未解除，"
                                      f"自动放弃剩余计划", "motion")
                target = (tvx, tvyaw)
            cur = self._cur
            cur[0] += max(-self._step_vx, min(self._step_vx, target[0] - cur[0]))
            cur[1] += max(-self._step_vyaw, min(self._step_vyaw, target[1] - cur[1]))
            # Protective limit is a final command envelope, not just a ramp target:
            # an absolute speed cap (info["cap_mps"], lidar headway envelope) must not lag
            # behind the deceleration ramp at high speed. A proportional slow factor is not
            # re-applied here (that would square it).
            # Do not probe zero a second time: LidarGuard would reset blocked_since.
            if self.speed_limiter is not None and abs(cur[0]) > 1e-6:
                safe_vx, safe_w, final_info = self.speed_limiter(cur[0], cur[1])
                cap = final_info.get("cap_mps")
                if cap is not None and abs(cur[0]) > cap:
                    cur[0] = cap if cur[0] > 0 else -cap
                if final_info.get("reason") in ("blocked", "no_data"):
                    cur[0], cur[1] = safe_vx, 0.0     # when blocked, zero angular speed immediately too (else it keeps turning while decelerating)
                    self.reflex = final_info
            settled_zero = not active and abs(cur[0]) < 1e-6 and abs(cur[1]) < 1e-6
            if settled_zero and self._idle_sent:
                return
            if settled_zero:
                cur[0] = cur[1] = 0.0
                self._idle_sent = True
            vx, vyaw = cur
            self._send(vx, vyaw)

    def _loop(self):
        while self._running:
            try:
                self.tick()
            except Exception as e:
                self._log(f"SequenceExecutor 异常: {type(e).__name__}: {e}", "error")
            time.sleep(self.dt)

    def _remaining_locked(self, now) -> np.ndarray:
        # +1e-6: avoid float edges like 4.999... reading one extra stale tick
        idx = int((now - self._t0) / self.dt + 1e-6)
        if idx < 0:
            idx = 0
        return self._timeline[idx:] if idx < len(self._timeline) else self._timeline[:0]

    def _send(self, vx, vyaw):
        if self.robot is not None and getattr(self.robot, "send_command", None):
            self.robot.send_command(vx, 0.0, vyaw)
