# -*- coding: utf-8 -*-
"""runlog.py -- run log: one-line summaries of each robot tool call's parameters and result
(ring buffer + optional JSONL file).

navi_mcp_server wraps every tool with RunLog.wrap. Things the MCP server cannot see (model
speech, user instructions, observer-layer messages) are extracted from session events by the host
plugin host/navi-observer, POSTed to the observer's /runlog and recorded via RunLog.note
into the same log. The camera page pulls /runlog incrementally and shows it below the image;
if NAVI_RUN_LOG is set (start.sh uses logs/run-<time>.jsonl) entries are also appended to
that file for later review. This module imports no other internal modules.
"""
from __future__ import annotations

import json
import math
import threading
import time
from collections import deque


def _clip(text, n: int = 160) -> str:
    s = " ".join(str(text).split())
    return s if len(s) <= n else s[:n] + "…"


def _num(v) -> str:
    return f"{v:g}" if isinstance(v, (int, float)) else str(v)


def call_text(name: str, params: dict) -> str:
    """One-line summary of a tool call."""
    p = {k: v for k, v in (params or {}).items() if v is not None}
    if name == "update_tasks":
        tasks = [t for t in p.get("tasks") or [] if isinstance(t, dict)]
        doing = "、".join(str(t.get("name")) for t in tasks if t.get("status") == "doing") or "—"
        done = sum(t.get("status") == "done" for t in tasks)
        return f"update_tasks 正在做：{_clip(doing, 80)}（已完成 {done}/{len(tasks)}）"
    if name == "submit_sequence":
        segs = [s for s in p.pop("segments", None) or [] if isinstance(s, dict)]
        body = " → ".join(f"{_num(s.get('vx', 0))}m/s 转{_num(s.get('vyaw', 0))} ×{_num(s.get('duration', 0))}s"
                          for s in segs)
        extra = f"，{_num(p['observe_after_s'])}s 后取帧" if p.get("observe_after_s") else ""
        return _clip(f"submit_sequence {body}{extra}", 200)
    args = " ".join(f"{k}={v if isinstance(v, str) else json.dumps(v, ensure_ascii=False)}" for k, v in p.items())
    return _clip(f"{name} {args}" if args else name, 200)


def result_text(result) -> tuple[str, bool]:
    """One-line summary of a tool result as (text, ok). `result` is the tool return value:
    a dict, or [dict, image, notes...]."""
    r = next((x for x in result if isinstance(x, dict)), None) if isinstance(result, (list, tuple)) else result
    if not isinstance(r, dict):
        return "完成", True
    if r.get("ok") is False:
        return _clip(r.get("error") or r.get("status") or "失败"), False
    parts = []
    if isinstance(r.get("status"), str) and r["status"] != "done":
        parts.append(r["status"])
    if isinstance(r.get("moved_m"), (int, float)):
        parts.append(f"走了 {r['moved_m']:.2f}m")
    if isinstance(r.get("turned_rad"), (int, float)):
        parts.append(f"转了 {round(math.degrees(r['turned_rad']))}°")
    if isinstance(r.get("measured_angle_rad"), (int, float)):
        parts.append(f"陀螺实测 {math.degrees(r['measured_angle_rad']):+.0f}°")
    if isinstance(r.get("measured_yaw_change_deg"), (int, float)):
        parts.append(f"航向变化 {r['measured_yaw_change_deg']:+.1f}°")
    track = r.get("heading_track")
    if isinstance(track, dict):
        parts.append(f"上段航向 指令 {track.get('cmd_deg'):+.0f}° / 实测 {track.get('gyro_deg'):+.0f}°")
    if r.get("turn_stop"):
        parts.append("⚠ 实测转角没到，超时停")
    if isinstance(r.get("place"), dict) and r["place"].get("name"):
        parts.append(f"已存「{r['place']['name']}」")
    if isinstance(r.get("state"), str):
        parts.append(r["state"])
    if isinstance(r.get("frame_id"), int):
        parts.append(f"帧 {r['frame_id']}")
    clearance = r.get("clearance")
    if isinstance(clearance, dict):
        ahead = clearance.get("ahead_m")
        parts.append("前方畅通" if ahead is None else f"前方 {ahead}m")
    if r.get("blocked_by_lidar") is not None:
        parts.append(f"雷达拦住（{r['blocked_by_lidar']}m）")
    if r.get("spin"):
        parts.append("⚠ 原地打转")
    return (_clip("，".join(parts)) if parts else "完成"), True


# Entry kinds supplied by the plugin: model speech / reasoning summary / user instruction /
# observer message / task goal / turn stopped / error
NOTE_KINDS = ("say", "think", "user", "observe", "goal", "stop", "error")


class RunLog:
    """Thread-safe run log. Entries are {"i": seq, "t": timestamp (s), "kind", "text", "ok"?};
    kind is call / result (tool calls, recorded here) or one of NOTE_KINDS (from the host plugin)."""

    def __init__(self, path: str | None = None, keep: int = 300, now_fn=time.time):
        self.path = path or None
        self.error: str | None = None
        self._entries: deque = deque(maxlen=keep)
        self._n = 0
        self._now = now_fn
        self._lock = threading.Lock()

    def record(self, kind: str, text: str, ok: bool | None = None) -> dict:
        with self._lock:
            entry = {"i": self._n, "t": round(self._now(), 3), "kind": kind, "text": text}
            if ok is not None:
                entry["ok"] = ok
            self._n += 1
            self._entries.append(entry)
            if self.path:
                try:
                    with open(self.path, "a", encoding="utf-8") as f:
                        f.write(json.dumps(entry, ensure_ascii=False) + "\n")
                except OSError as exc:                 # file write failure must not affect the tool; page still shows it
                    self.error = f"运行日志写不进 {self.path}: {exc}"
        return entry

    def note(self, body) -> dict:
        """Record an entry from the host plugin (JSON body {"kind", "text"} of POST /runlog)."""
        kind = body.get("kind") if isinstance(body, dict) else None
        text = body.get("text") if isinstance(body, dict) else None
        if kind not in NOTE_KINDS or not isinstance(text, str) or not text.strip():
            return {"ok": False, "error": f"kind 须为 {'/'.join(NOTE_KINDS)}，text 须为非空字符串"}
        return {"ok": True, "i": self.record(kind, _clip(text, 400))["i"]}

    def since(self, after: int = 0) -> dict:
        """Entries with seq >= after; n = total recorded (lets the page detect a server restart)."""
        with self._lock:
            return {"n": self._n, "entries": [e for e in self._entries if e["i"] >= after], "error": self.error}

    def wrap(self, name: str, params: dict, run):
        """Record one call: parameters first, result after; exceptions are re-raised to the caller."""
        self.record("call", call_text(name, params))
        try:
            result = run()
        except Exception as exc:
            self.record("result", _clip(f"{type(exc).__name__}: {exc}"), ok=False)
            raise
        text, ok = result_text(result)
        self.record("result", text, ok)
        return result
