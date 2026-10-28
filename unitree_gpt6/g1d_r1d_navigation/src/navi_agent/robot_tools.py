# -*- coding: utf-8 -*-
"""Pure tool logic with no FastMCP dependency.

Tools are registered in navi_mcp_server.py; this module holds only testable decision and
transformation logic.
"""
from __future__ import annotations

import math
import time

import cv2
import numpy as np

JPEG_QUALITY = 85


def frame_to_jpeg(frame_bgr: np.ndarray, quality: int = JPEG_QUALITY) -> bytes:
    """Encode a BGR frame as JPEG bytes."""
    ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("JPEG 编码失败")
    return buf.tobytes()


def center_crop(frame: np.ndarray, ratio: float) -> np.ndarray:
    """Crop the centre of the frame by side-length ratio (ratio in (0, 1]; 1 = full frame)."""
    if not (isinstance(ratio, (int, float)) and math.isfinite(ratio) and 0 < ratio <= 1):
        raise ValueError(f"裁剪比例须在 (0, 1] 范围内，收到 {ratio}")
    h, w = frame.shape[:2]
    ch, cw = max(1, round(h * ratio)), max(1, round(w * ratio))
    top, left = (h - ch) // 2, (w - cw) // 2
    return frame[top:top + ch, left:left + cw]


def observe_frame(driver, eye: str | None, *, with_crop: bool = True, wait_fresh_s: float = 0.0) -> dict:
    """Latest camera frame -> decode -> split stereo half -> JPEG (+ optional centre-crop JPEG).

    eye=None falls back to driver.camera_cfg['eye']. Returns
    {"ok": True, "jpeg": bytes, "eye": str} or {"ok": False, "error": str}.
    Splitting matches core/driver.py::_get_frame_bgr: only when the camera is binocular and
    eye != "full"; monocular cameras keep the full frame.
    If profile.camera_crop_ratio < 1 (R1D), also adds "crop_jpeg" and "crop_ratio": a centre
    crop upscaled back to full size (a narrower "normal FOV" view; centre-cropping a pinhole
    image is equivalent to a longer focal length, so no reprojection is needed).
    "jpeg" is always the full frame (used for memory evidence and the observation view);
    with_crop=False skips the crop encode (high-frequency observation polling).
    wait_fresh_s > 0 waits that long for a fresh frame: after packet loss the R1D decoder
    drops frames until the next keyframe (~1 s apart measured), so without waiting a single
    loss would make the model think the camera is broken.
    """
    cam = getattr(driver, "camera", None)
    if cam is None or not getattr(cam, "connected", False):
        if getattr(getattr(driver, "profile", None), "camera_kind", None) == "r1d_rtp":
            return {"ok": False, "error": "R1D 相机未连接（UDP 5002 无左目画面；自动切换相机服务失败，"
                                          "详见 sidecar 日志，重启 Navi 会再试一次）"}
        return {"ok": False, "error": "相机未连接（observe 需要 image_server 在线）"}
    meta = {}
    if hasattr(cam, "snapshot"):
        snapshot = cam.snapshot()
        deadline = time.monotonic() + max(0.0, float(wait_fresh_s))
        while not snapshot["fresh"] and time.monotonic() < deadline:
            time.sleep(0.05)
            snapshot = cam.snapshot()
        raw = snapshot.pop("raw")
        meta = snapshot
        if not snapshot["fresh"]:
            return {"ok": False, "error": "相机帧过期或断流，请停止并恢复观测", **meta}
    else:  # legacy injected test cameras
        raw = cam.get_latest_frame()
    if raw is None:
        return {"ok": False, "error": "相机暂无帧（订阅建立后稍候重试）"}
    frame = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        return {"ok": False, "error": "帧解码失败（半截帧，稍候重试）"}
    eff_eye = eye or (driver.camera_cfg or {}).get("eye", "left")
    binocular = getattr(cam, "binocular", False)
    if binocular and eff_eye != "full":
        half = frame.shape[1] // 2
        frame = frame[:, :half] if eff_eye == "left" else frame[:, half:]
    out = {"ok": True, "jpeg": frame_to_jpeg(frame), "eye": eff_eye, **meta}
    ratio = getattr(getattr(driver, "profile", None), "camera_crop_ratio", 1.0)
    if with_crop and ratio < 1:
        h, w = frame.shape[:2]
        zoom = cv2.resize(center_crop(frame, ratio), (w, h), interpolation=cv2.INTER_CUBIC)
        out["crop_jpeg"] = frame_to_jpeg(zoom)
        out["crop_ratio"] = ratio
    return out


VALID_DIRECTIONS = ("forward", "backward")   # wheeled base cannot strafe; use turn to rotate


def guard_move(direction: str, distance_m: float, max_m: float) -> dict:
    """Validate a translation: known direction, positive distance, clamped to the hard cap.

    Raises ValueError on invalid input."""
    if direction not in VALID_DIRECTIONS:
        raise ValueError(f"未知方向 {direction}，合法值: {VALID_DIRECTIONS}")
    if not math.isfinite(distance_m) or distance_m <= 0:
        raise ValueError(f"距离必须为正数，收到 {distance_m}")
    clamped = distance_m > max_m
    return {"direction": direction,
            "distance_m": min(distance_m, max_m),
            "clamped": clamped}


# Smaller turns are useless for avoidance/alignment; on the real robot the model was seen
# creeping in 5/6/8 degree steps.
MIN_TURN_DEG = 10.0


def guard_turn(angle_deg: float, max_deg: float) -> dict:
    """Validate a turn: |angle| >= MIN_TURN_DEG, magnitude clamped to the hard cap (sign kept)."""
    if not math.isfinite(angle_deg):
        raise ValueError("角度必须为有限数")
    if abs(angle_deg) < MIN_TURN_DEG:
        raise ValueError(f"转向角度至少 {MIN_TURN_DEG:.0f}°（收到 {angle_deg}°）；"
                         f"微调没有意义，避障一次至少转 30°")
    clamped = abs(angle_deg) > max_deg
    return {"angle_deg": max(-max_deg, min(max_deg, angle_deg)),
            "clamped": clamped}


def guard_sequence(segments, limits: dict) -> dict:
    """Validate a velocity sequence: clamp per-segment speeds and truncate once cumulative
    distance/rotation/duration hits the hard cap. Raises ValueError on malformed input.

    segments: [{"vx": m/s, "vyaw": rad/s, "duration": s}, ...] (missing fields = 0).
    limits: max_vx / max_vyaw / max_duration_s / max_move_distance_m / max_turn_angle_deg.
    Trailing zero-velocity segments are stripped first (the executor stops on its own when the
    sequence ends; a zero tail only wastes duration budget and invites padding). An all-zero
    sequence collapses to a single 0.5 s segment (meaning "stop"). Interior zero segments
    (move-pause-move) are kept.
    Returns {"segments": [(vx, vyaw, dur), ...], "clamped", "total_s", "motion_s",
    "stripped_zero_s"}.
    """
    if not isinstance(segments, (list, tuple)) or not segments:
        raise ValueError("序列必须是非空列表")
    parsed = []
    for i, seg in enumerate(segments):
        if not isinstance(seg, dict):
            raise ValueError(f"第 {i} 段不是对象: {seg!r}")
        try:
            vx = float(seg.get("vx", 0.0))
            vyaw = float(seg.get("vyaw", 0.0))
            dur = float(seg.get("duration", 0.0))
        except (TypeError, ValueError):
            raise ValueError(f"第 {i} 段字段须为数值: {seg!r}") from None
        if not all(math.isfinite(v) for v in (vx, vyaw, dur)):
            raise ValueError(f"第 {i} 段字段须为有限数")
        if dur <= 0:
            raise ValueError(f"第 {i} 段 duration 必须为正数，收到 {dur}")
        parsed.append([vx, vyaw, dur])

    def _zero(p):
        return abs(p[0]) < 1e-6 and abs(p[1]) < 1e-6
    stripped = 0.0
    while len(parsed) > 1 and _zero(parsed[-1]):
        stripped += parsed.pop()[2]
    if _zero(parsed[-1]) and parsed[-1][2] > 0.5:
        stripped += parsed[-1][2] - 0.5
        parsed[-1][2] = 0.5

    max_vx = float(limits["max_vx"])
    max_vyaw = float(limits["max_vyaw"])
    max_s = float(limits["max_duration_s"])
    max_dist = float(limits["max_move_distance_m"])
    max_rad = math.radians(float(limits["max_turn_angle_deg"]))
    out, clamped = [], False
    acc_s = acc_dist = acc_rad = 0.0
    for i, (vx, vyaw, dur) in enumerate(parsed):
        if abs(vx) > max_vx:
            vx, clamped = math.copysign(max_vx, vx), True
        if abs(vyaw) > max_vyaw:
            vyaw, clamped = math.copysign(max_vyaw, vyaw), True
        # Cumulative budget: when duration, distance or rotation hits its cap, shorten this
        # segment to exactly reach it and drop the rest.
        budget = max_s - acc_s
        if abs(vx) > 0:
            budget = min(budget, (max_dist - acc_dist) / abs(vx))
        if abs(vyaw) > 0:
            budget = min(budget, (max_rad - acc_rad) / abs(vyaw))
        if budget <= 1e-6:
            clamped = True
            break
        if dur > budget:
            dur, clamped = budget, True
        out.append((vx, vyaw, dur))
        acc_s += dur
        acc_dist += abs(vx) * dur
        acc_rad += abs(vyaw) * dur
        if dur == budget and i < len(parsed) - 1:
            clamped = True
            break
    if not out:
        raise ValueError("序列被守门清空（首段即超预算）")
    motion_s = sum(d for vx, vy, d in out if abs(vx) >= 1e-6 or abs(vy) >= 1e-6)
    return {"segments": out, "clamped": clamped, "total_s": round(acc_s, 3),
            "motion_s": round(motion_s, 3), "stripped_zero_s": round(stripped, 3)}


TASK_STATUSES = ("todo", "doing", "done", "dropped")
# Accept common status synonyms: the board is display-only, and rejecting a misspelled status
# would cost another reasoning round (up to 20-30 s).
_TASK_STATUS_ALIASES = {"pending": "todo", "in_progress": "doing", "active": "doing", "running": "doing",
                        "completed": "done", "complete": "done", "finished": "done",
                        "failed": "dropped", "skipped": "dropped", "blocked": "dropped", "cancelled": "dropped"}
MAX_TASKS = 20


def normalize_tasks(tasks) -> list[dict]:
    """Validate the task board: [{"name", "status", "note"}, ...] -> normalised list
    (an empty list clears the board).

    name is required (truncated to 60 chars); status in TASK_STATUSES, default "todo", common
    synonyms normalised; note is optional (truncated to 80 chars).
    Raises ValueError on bad structure (not a list, item not a dict, empty or duplicate name,
    unknown status, more than MAX_TASKS items).
    """
    if not isinstance(tasks, (list, tuple)):
        raise ValueError("tasks 必须是列表")
    if len(tasks) > MAX_TASKS:
        raise ValueError(f"任务最多 {MAX_TASKS} 项，收到 {len(tasks)} 项")
    out, seen = [], set()
    for i, t in enumerate(tasks):
        if not isinstance(t, dict):
            raise ValueError(f"第 {i} 项不是对象: {t!r}")
        name = str(t.get("name") or "").strip()[:60]
        if not name:
            raise ValueError(f"第 {i} 项缺少 name")
        if name in seen:
            raise ValueError(f"任务名重复：{name}")
        seen.add(name)
        status = str(t.get("status") or "todo").strip().lower()
        status = _TASK_STATUS_ALIASES.get(status, status)
        if status not in TASK_STATUSES:
            raise ValueError(f"第 {i} 项 status 须为 {'/'.join(TASK_STATUSES)}，收到 {t.get('status')!r}")
        item = {"name": name, "status": status}
        note = str(t.get("note") or "").strip()[:80]
        if note:
            item["note"] = note
        out.append(item)
    return out


class SpinWatch:
    """Detect spinning in place: cumulative degrees turned since the robot last actually moved
    away. Pure logic; callers must invoke it serially.

    "Moved away" prefers measured pose (R1D lidar localisation): >= move_m from the anchor
    resets the count. Without pose, cumulative commanded translation >= move_m counts.
    record() returns {"turned_deg", "moved_m"} once turning reaches limit_deg, else None; no
    warning if the current step itself translates >= move_m (already leaving).
    Seen on the real robot: searching for a passage, the model turned 700+ degrees in ~4 min
    while moving only 2-3 m.
    """

    def __init__(self, limit_deg: float = 360.0, move_m: float = 1.0):
        self.limit_deg = float(limit_deg)
        self.move_m = float(move_m)
        self.reset()

    def reset(self, anchor=None):
        self.turned_deg = 0.0
        self.cmd_moved_m = 0.0
        self.anchor = anchor

    def record(self, turn_deg: float, move_m: float, pose=None):
        """Record one step: commanded turn (deg) and translation (m); pose is the post-step
        (x, y) or None."""
        if pose is not None:
            if self.anchor is None:
                self.anchor = pose
            elif math.hypot(pose[0] - self.anchor[0], pose[1] - self.anchor[1]) >= self.move_m:
                self.reset(anchor=pose)
        elif self.cmd_moved_m >= self.move_m:
            self.reset()
        self.turned_deg += abs(turn_deg)
        self.cmd_moved_m += abs(move_m)
        if self.turned_deg < self.limit_deg or abs(move_m) >= self.move_m:
            return None
        moved = (math.hypot(pose[0] - self.anchor[0], pose[1] - self.anchor[1])
                 if pose is not None and self.anchor is not None else self.cmd_moved_m)
        return {"turned_deg": round(self.turned_deg), "moved_m": round(moved, 1)}
