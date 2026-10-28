# -*- coding: utf-8 -*-
"""stub_mcp_server.py -- offline robot stub: no real robot, observe returns synthetic frames.

Purpose: validate the full host -> MCP -> image-in-context chain and tune prompts while the
robot is offline. Tool names/arguments/result fields mirror navi_mcp_server.py, so switching
between stub and real robot needs no prompt changes.
Synthetic frames show a large incrementing number in black on white; if the model reads the
number correctly, the image really reached its context.
Odometry is a fake instantaneous integral; navigation is a fake state machine that "arrives"
after 2 s. Never connects to DDS and never sends motion commands.
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2
import numpy as np
from mcp.server.fastmcp import FastMCP, Image

import robot_tools

mcp = FastMCP("unitree-navi-stub", instructions="离线机器人桩：全部工具为假实现，仅用于链路验证。")

_state = {"x": 0.0, "y": 0.0, "yaw": 0.0, "_seq": 0,
          "_frame_ts": {}, "_frame_jpeg": {}, "_seq_active_until": 0.0, "_last_submit": None}


def _frame():
    """Synthesize a frame (incrementing number) and record frame_id -> timestamp, same contract as the real sidecar."""
    _state["_seq"] += 1
    fid = _state["_seq"]
    _state["_frame_ts"][fid] = time.monotonic()
    frame = np.full((480, 640, 3), 255, dtype=np.uint8)
    cv2.putText(frame, f"STUB {fid:04d}", (60, 260),
                cv2.FONT_HERSHEY_SIMPLEX, 3.0, (0, 0, 0), 12, cv2.LINE_AA)
    cv2.putText(frame, time.strftime("%H:%M:%S"), (60, 420),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        raise RuntimeError("stub 帧编码失败")
    cache = _state.setdefault("_frame_jpeg", {})    # tests replace _state wholesale
    cache[fid] = buf.tobytes()
    while len(cache) > 16:
        cache.pop(next(iter(cache)))
    return fid, Image(data=buf.tobytes(), format="jpeg")


def _busy():
    return navigation_status()["active"]


@mcp.tool()
def get_status() -> dict:
    """桩状态。lidar 字段与真机同形（离线桩无雷达：enabled=false）。"""
    return {"ok": True, "robot_connected": False, "camera_connected": True,
            "position": [_state["x"], _state["y"], 0.0], "yaw": _state["yaw"],
            "lidar": {"ok": False, "enabled": False, "state": "disabled", "error": None},
            "simulated": True, "navigation": navigation_status()}


@mcp.tool()
def get_odometry() -> dict:
    """桩里程计。"""
    return {"ok": True, "position": [_state["x"], _state["y"], 0.0],
            "yaw": _state["yaw"], "source": "command_integral_estimate", "measured": False}


@mcp.tool()
def observe(eye: str | None = None):
    """返回一帧合成图像（白底黑字递增数字）+ frame_id。"""
    fid, img = _frame()
    return [{"ok": True, "frame_id": fid, "eye": eye or "left"}, img]


@mcp.tool()
def move(direction: str, distance_m: float):
    """桩移动：按方向累加假里程计。"""
    if _busy():
        return {"ok": False, "error": "地图导航占用底盘", "simulated": True}
    try:
        g = robot_tools.guard_move(direction, distance_m, 5.)
    except ValueError as exc:
        return {"ok": False, "error": str(exc)}
    signed = g["distance_m"] * (1 if direction == "forward" else -1)
    _state["x"] += signed * math.cos(_state["yaw"])
    _state["y"] += signed * math.sin(_state["yaw"])
    fid, img = _frame()
    return [{"ok": True, "status": "done", "moved_m": g["distance_m"], "clamped": g["clamped"],
             "frame_id": fid, "simulated": True}, img]


@mcp.tool()
def turn(angle_deg: float):
    """桩转向：累加假朝向。"""
    if _busy():
        return {"ok": False, "error": "地图导航占用底盘", "simulated": True}
    try:
        g = robot_tools.guard_turn(angle_deg, 360.)
    except ValueError as exc:
        return {"ok": False, "error": str(exc)}
    _state["yaw"] = (_state["yaw"] + math.radians(g["angle_deg"])) % (2 * math.pi)
    fid, img = _frame()
    return [{"ok": True, "status": "done", "turned_rad": math.radians(g["angle_deg"]),
             "clamped": g["clamped"], "frame_id": fid, "simulated": True}, img]


@mcp.tool()
def submit_sequence(segments: list[dict], frame_id: int | None = None,
                    observe_after_s: float = 0., alignment: str = "current"):
    """桩序列：按段累加假里程计（瞬时完成），返回与真机同形的摘要 + 新帧。"""
    if _busy():
        return {"ok": False, "error": "地图导航占用底盘", "simulated": True}
    if alignment != "current":
        return {"ok": False, "error": "stub 仅模拟 alignment=current"}
    if not math.isfinite(observe_after_s):
        return {"ok": False, "error": "observe_after_s 须为有限数"}
    try:
        g = robot_tools.guard_sequence(segments, {"max_vx": .5, "max_vyaw": .5,
            "max_duration_s": 45., "max_move_distance_m": 5., "max_turn_angle_deg": 360.})
    except ValueError as exc:
        return {"ok": False, "error": str(exc)}
    now = time.monotonic()
    ts = _state["_frame_ts"].get(frame_id) if frame_id is not None else None
    offset = (now - ts) if ts is not None else 0.0
    for vx, vyaw, dur in g["segments"]:
        _state["yaw"] = (_state["yaw"] + vyaw * dur) % (2 * math.pi)
        _state["x"] += vx * dur * math.cos(_state["yaw"])
        _state["y"] += vx * dur * math.sin(_state["yaw"])
    mode = "fresh" if now >= _state["_seq_active_until"] else "blend"
    _state["_seq_active_until"] = now + g["total_s"]
    _state["_last_submit"] = now
    fid, img = _frame()
    return [{"ok": True, "mode": mode, "offset_s": round(offset, 2),
             "remaining_s": g["total_s"], "clamped": g["clamped"],
             "accepted_s": g["total_s"], "motion_s": g["motion_s"],
             "latency_est_s": round(offset, 2),
             "suggested_horizon_s": round(max(3.0, offset * 1.7), 1),
             "alignment": alignment, "dropped_s": 0., "simulated": True,
             "frame_id": fid}, img]


@mcp.tool()
def sequence_status() -> dict:
    """桩序列状态。"""
    rem = max(0.0, _state["_seq_active_until"] - time.monotonic())
    return {"ok": True, "active": rem > 0, "remaining_s": round(rem, 2),
            "cmd": {"vx": 0.0, "vyaw": 0.0}, "starved": False}


@mcp.tool()
def stop() -> dict:
    """桩停止。"""
    _state["_seq_active_until"] = 0.0
    if _nav["active"]:
        _nav.update(state="cancelled", active=False, ok=False)
    return {"ok": True, "status": "stopped"}


# Deliberately in-memory only, isolated from any real-robot places file.
_places = {}
_nav = {"ok": True, "active": False, "state": "idle"}


@mcp.tool()
def get_localization() -> dict:
    """仅返回模拟定位，不连接 DDS。"""
    return {"ok": True, "simulated": True,
            "pose": {k: _state[k] for k in ("x", "y", "yaw")} | {"frame": "stub-map"},
            "odom": {"speed": 0., "wz": 0.}, "slam_state": "simulated"}


@mcp.tool()
def list_places() -> dict:
    """列出此进程内的模拟点；不读取/写入真机标定文件。"""
    return {"ok": True, "simulated": True, "map_id": "stub-only", "frame": "stub-map",
            "places": [dict(name=n, **p) for n, p in _places.items()]}


@mcp.tool()
def save_place(name: str, overwrite: bool = True) -> dict:
    """模拟静止标定，重启后清空。与真机一致：重名默认覆盖，overwrite=false 时重名报错。"""
    if _busy() or sequence_status()["active"]:
        return {"ok": False, "error": "motion_busy", "simulated": True}
    name = name.strip()
    if not name or len(name) > 80 or any(ord(c) < 32 for c in name):
        return {"ok": False, "error": "无效点名"}
    replaced = name in _places
    if replaced and not overwrite:
        return {"ok": False, "error": "地点已存在；需 overwrite=true"}
    _places[name] = get_localization()["pose"]
    return {"ok": True, "simulated": True, "place": dict(name=name, **_places[name], replaced=replaced)}


@mcp.tool()
def update_tasks(tasks: list[dict]) -> dict:
    """桩任务看板：只校验并回显（没有相机页）。"""
    try:
        items = robot_tools.normalize_tasks(tasks)
    except ValueError as exc:
        return {"ok": False, "error": str(exc)}
    return {"ok": True, "doing": [t["name"] for t in items if t["status"] == "doing"],
            "done": sum(t["status"] == "done" for t in items), "total": len(items)}


@mcp.tool()
def navigate_to(name: str) -> dict:
    """模拟异步导航，两秒后在状态查询时完成；绝不下发运动。"""
    if _busy() or sequence_status()["active"]:
        return {"ok": False, "error": "motion_busy", "simulated": True}
    if name not in _places:
        return {"ok": False, "error": "未标定地点", "simulated": True}
    _nav.update(ok=True, active=True, state="running", place=name, goal=_places[name],
                deadline=time.monotonic() + 2.)
    return navigation_status()


@mcp.tool()
def navigation_status() -> dict:
    """模拟地图导航状态。"""
    if _nav["active"] and time.monotonic() >= _nav["deadline"]:
        for k in ("x", "y", "yaw"):
            _state[k] = _nav["goal"][k]
        _nav.update(ok=True, active=False, state="arrived")
    return {k: v for k, v in _nav.items() if k != "deadline"} | {
        "simulated": True, "map_id": "stub-only", "frame": "stub-map"}


@mcp.tool()
def navigation_preflight() -> dict:
    """桩预检：仅接口模拟，不证明真机已验收。"""
    return dict(ok=True, simulated=True, engine="stub", motion_started=False,
                active=navigation_status()["active"], errors=[])


@mcp.tool()
def mapping_capabilities() -> dict:
    """桩能力清单：不把假数据当作真实 SLAM 接入。"""
    return dict(ok=True, read_only=True, simulated=True, backend="stub",
                existing_map_navigation=True, online_map_building=False, map_save_load=False)


@mcp.tool()
def mapping_status() -> dict:
    """桩没有机载 SLAM；不伪造地图健康。"""
    return dict(ok=False, read_only=True, simulated=True, motion_started=False,
                error="stub_has_no_onboard_slam")


# Topological memory: real implementation backed by a temp-dir store (discarded on exit);
# evidence frames come from the synthetic frame cache.
from services.topo_memory import TopoMemory, register_memory_tools  # noqa: E402

_memory = None


def _stub_memory():
    global _memory
    if _memory is None:
        import tempfile
        _memory = TopoMemory(Path(tempfile.mkdtemp(prefix="navi-stub-memory-")), "stub", "stub")
    return _memory


def _stub_frame_jpeg(frame_id):
    cache = _state.setdefault("_frame_jpeg", {})
    if frame_id is None:
        if not cache:
            raise ValueError("还没有可用的帧：先 observe")
        fid = next(reversed(cache))
        return fid, cache[fid]
    if int(frame_id) not in cache:
        raise ValueError(f"frame_id {frame_id} 不在最近帧缓存里：重新 observe")
    return int(frame_id), cache[int(frame_id)]


register_memory_tools(mcp, _stub_memory, _stub_frame_jpeg)


if __name__ == "__main__":
    mcp.run()
