# src/tests/test_runlog.py
# -*- coding: utf-8 -*-
"""Run log: call/result summaries, incremental reads, persistence, and MCP tool registration that keeps parameter signatures and logs every call."""
import asyncio
import json
import math

import pytest

from services.runlog import RunLog, call_text, result_text


def test_call_text_summarizes_common_tools():
    assert call_text("turn", {"angle_deg": -90}) == "turn angle_deg=-90"
    seq = call_text("submit_sequence", {"segments": [{"vx": 0.3, "vyaw": 0, "duration": 4},
                                                     {"vx": 0.2, "vyaw": 0.25, "duration": 1.4}],
                                        "frame_id": 5, "observe_after_s": 4, "alignment": "current"})
    assert "0.3m/s 转0 ×4s → 0.2m/s 转0.25 ×1.4s" in seq and "4s 后取帧" in seq
    tasks = call_text("update_tasks", {"tasks": [{"name": "A1 卧室", "status": "done"},
                                                 {"name": "A2 厨房", "status": "doing"},
                                                 {"name": "A3 客厅"}]})
    assert tasks == "update_tasks 正在做：A2 厨房（已完成 1/3）"
    assert call_text("get_status", {}) == "get_status"
    assert call_text("observe", {"eye": None}) == "observe"          # default-valued args omitted


def test_result_text_picks_salient_fields():
    turn = [{"ok": True, "status": "done", "turned_rad": math.radians(84), "frame_id": 8,
             "clearance": {"ahead_m": 1.19, "front_m": 1.19}}, "图1 说明", object()]
    assert result_text(turn) == ("转了 84°，帧 8，前方 1.19m", True)
    assert result_text({"ok": False, "error": "motion_busy"}) == ("motion_busy", False)
    assert result_text({"ok": True, "place": {"name": "卧室"}})[0] == "已存「卧室」"
    assert result_text({"ok": True, "clearance": {"ahead_m": None}, "spin": "…"})[0] == "前方畅通，⚠ 原地打转"
    assert result_text({"ok": True, "doing": ["A2"]}) == ("完成", True)


def test_runlog_since_is_incremental_and_bounded():
    log = RunLog(keep=3, now_fn=lambda: 100.0)
    for i in range(5):
        log.record("call", f"c{i}")
    r = log.since(0)
    assert r["n"] == 5 and [e["i"] for e in r["entries"]] == [2, 3, 4]    # only the latest `keep` entries are retained
    assert [e["text"] for e in log.since(4)["entries"]] == ["c4"]
    assert log.since(5)["entries"] == []


def test_runlog_wrap_records_call_result_and_reraises(tmp_path):
    path = tmp_path / "run.jsonl"
    log = RunLog(str(path))
    assert log.wrap("move", {"direction": "forward", "distance_m": 0.5},
                    lambda: {"ok": True, "moved_m": 0.41}) == {"ok": True, "moved_m": 0.41}

    def boom():
        raise RuntimeError("driver gone")
    with pytest.raises(RuntimeError):
        log.wrap("observe", {}, boom)
    lines = [json.loads(x) for x in path.read_text(encoding="utf-8").splitlines()]
    assert [(e["kind"], e.get("ok")) for e in lines] == [("call", None), ("result", True),
                                                         ("call", None), ("result", False)]
    assert lines[1]["text"] == "走了 0.41m" and "driver gone" in lines[3]["text"]


def test_runlog_unwritable_path_keeps_entries_in_memory(tmp_path):
    log = RunLog(str(tmp_path / "missing-dir" / "run.jsonl"))
    log.record("call", "stop")
    r = log.since(0)
    assert r["n"] == 1 and "写不进" in r["error"]


def test_registered_tools_keep_schema_and_are_logged(monkeypatch):
    import navi_mcp_server as srv
    monkeypatch.setattr(srv, "_runlog", RunLog())
    tools = {t.name: t for t in asyncio.run(srv.mcp.list_tools())}
    assert "angle_deg" in tools["turn"].inputSchema["properties"]          # signature survives functools.wraps for FastMCP
    assert "segments" in tools["submit_sequence"].inputSchema["properties"]
    assert "memory_context" in tools
    asyncio.run(srv.mcp.call_tool("update_tasks", {"tasks": [{"name": "A1 卧室", "status": "doing"}]}))
    entries = srv._runlog.since(0)["entries"]
    assert [e["kind"] for e in entries] == ["call", "result"]
    assert entries[0]["text"] == "update_tasks 正在做：A1 卧室（已完成 0/1）" and entries[1]["ok"] is True
    assert srv._runlog_since({"after": "1"})["entries"] == entries[1:]
    assert srv._runlog_since({"after": "x"})["n"] == 2


def test_note_accepts_only_known_kinds_and_clips():
    log = RunLog()
    assert log.note({"kind": "say", "text": "已进入客厅，接下来找沙发右侧的通道"})["ok"]
    assert log.note({"kind": "user", "text": "长" * 1000})["ok"]
    assert not log.note({"kind": "call", "text": "伪造一条工具调用"})["ok"]       # tool calls may only be logged by the server itself
    assert not log.note({"kind": "say", "text": "  "})["ok"]
    assert not log.note(["say", "x"])["ok"]
    entries = log.since(0)["entries"]
    assert [e["kind"] for e in entries] == ["say", "user"] and len(entries[1]["text"]) == 401


def test_observer_post_runlog_records_note(monkeypatch):
    import urllib.request

    import navi_mcp_server as srv
    from services.observer import ObserverServer
    monkeypatch.setattr(srv, "_runlog", RunLog())
    obs = ObserverServer(lambda: {}, lambda: None, port=0, routes={"/runlog": srv._runlog_since},
                         post_routes={"/runlog": lambda body: srv._runlog.note(body)})
    obs.start()
    try:
        base = f"http://127.0.0.1:{obs._httpd.server_address[1]}"
        req = urllib.request.Request(f"{base}/runlog", method="POST", headers={"Content-Type": "application/json"},
                                     data=json.dumps({"kind": "say", "text": "床面中央有浅色衣物"}).encode())
        with urllib.request.urlopen(req, timeout=3) as r:
            assert json.loads(r.read())["ok"]
        with urllib.request.urlopen(f"{base}/runlog?after=0", timeout=3) as r:
            assert [e["text"] for e in json.loads(r.read())["entries"]] == ["床面中央有浅色衣物"]
    finally:
        obs.stop()


def test_result_text_shows_gyro_measurements():
    assert result_text({"ok": True, "turned_rad": math.radians(90), "measured_angle_rad": math.radians(-88)})[0] \
        == "转了 90°，陀螺实测 -88°"
    assert "航向变化 -3.2°" in result_text({"ok": True, "moved_m": 0.5, "measured_yaw_change_deg": -3.2})[0]
    assert "上段航向 指令 +0° / 实测 -4°" in result_text({"ok": True, "heading_track": {"cmd_deg": 0.0, "gyro_deg": -4.0}})[0]
    assert "超时停" in result_text({"ok": True, "turn_stop": "timeout: …"})[0]


def test_heading_track_between_submits(monkeypatch):
    from types import SimpleNamespace

    import navi_mcp_server as srv
    totals = {"cmd": 0.0, "gyro": 0.0, "gaps": 0}
    d = SimpleNamespace(robot=SimpleNamespace(yaw_totals=lambda: dict(totals)))
    monkeypatch.setattr(srv, "_yaw_mark", [None])
    assert srv._heading_since_last_submit(d) is None                  # first call: only records the baseline
    totals.update(cmd=math.radians(30), gyro=math.radians(24))
    assert srv._heading_since_last_submit(d) == {"cmd_deg": 30.0, "gyro_deg": 24.0}
    totals.update(gaps=3)
    assert srv._heading_since_last_submit(d) is None                  # gyro dropped out in this segment: not reported
    assert srv._heading_since_last_submit(SimpleNamespace(robot=None)) is None   # G1D / no gyro attached
