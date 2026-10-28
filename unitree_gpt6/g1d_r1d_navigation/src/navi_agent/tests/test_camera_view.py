# src/tests/test_camera_view.py
# -*- coding: utf-8 -*-
"""Camera preview page: frames from the sidecar observer (R1D), task board and run-log proxying."""
import socket
import time

import cv2
import numpy as np

from services.observer import ObserverServer
from tools import camera_view as cv


def _jpeg(h=448, w=544, value=100):
    ok, buf = cv2.imencode(".jpg", np.full((h, w, 3), value, np.uint8))
    return buf.tobytes()


def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class _Cam:
    connected = True
    binocular = False

    def __init__(self, raw):
        self.raw = raw

    def get_latest_frame(self):
        return self.raw


def test_monocular_frame_passes_raw_through(monkeypatch):
    raw = _jpeg()
    monkeypatch.setitem(cv._state, "cam", _Cam(raw))
    monkeypatch.setitem(cv._state, "binocular", False)
    assert cv._frame_jpeg("left") is raw                   # no green crop overlay any more: raw frame passed through


def test_explicit_source_skips_probe():
    assert cv._pick_source("zmq") == ("zmq", None)
    assert cv._pick_source("observer") == ("observer", None)


def test_observer_stream_pulls_sidecar_frames():
    frames = {"jpg": None}
    port = _free_port()
    srv = ObserverServer(lambda: {}, lambda: frames["jpg"], port=port)
    assert srv.start()
    try:
        cam = cv._ObserverStream(port, fps=20.0)
        deadline = time.monotonic() + 3
        while cam.error is None and time.monotonic() < deadline:
            time.sleep(0.05)
        assert not cam.connected and "等待 sidecar 相机" in cam.error   # 503: driver not created yet
        frames["jpg"] = _jpeg()
        deadline = time.monotonic() + 3
        while not cam.connected and time.monotonic() < deadline:
            time.sleep(0.05)
        assert cam.connected and cam.error is None
        assert cam.get_latest_frame() == frames["jpg"] and cam.n_recv >= 1
    finally:
        srv.stop()


def test_observer_stream_reports_unreachable():
    cam = cv._ObserverStream(_free_port(), fps=20.0)
    deadline = time.monotonic() + 3
    while cam.error is None and time.monotonic() < deadline:
        time.sleep(0.05)
    assert not cam.connected and "连不上 sidecar 观测面" in cam.error


def test_panel_relays_task_board_and_places(monkeypatch):
    st = {"task": {"tasks": [{"name": "A2 找到厨房并标记", "status": "doing"}], "updated_at": 1.0},
          "places": [{"name": "卧室", "taught_at": "2026-10-09T09:07:18+00:00"}], "lidar": {"ok": True}}
    port = _free_port()
    srv = ObserverServer(lambda: st, lambda: None, port=port)
    assert srv.start()
    try:
        monkeypatch.setitem(cv._state, "observer_port", port)
        p = cv._panel()
        assert p["ok"] and p["task"]["tasks"][0]["status"] == "doing"
        assert p["places"] == st["places"] and "lidar" not in p       # only the two fields the board needs are proxied
    finally:
        srv.stop()


def test_panel_reports_unreachable_sidecar(monkeypatch):
    monkeypatch.setitem(cv._state, "observer_port", _free_port())
    p = cv._panel()
    assert not p["ok"] and "连不上" in p["error"]
    monkeypatch.setitem(cv._state, "observer_port", 0)
    assert "port=0" in cv._panel()["error"]


def test_runlog_relays_sidecar_entries(monkeypatch):
    from services.runlog import RunLog
    log = RunLog()
    log.record("call", "turn angle_deg=-90")
    log.record("result", "转了 84°，前方 1.19m", ok=True)
    port = _free_port()
    srv = ObserverServer(lambda: {}, lambda: None, port=port,
                         routes={"/runlog": lambda q: log.since(int(q.get("after", 0)))})
    assert srv.start()
    try:
        monkeypatch.setitem(cv._state, "observer_port", port)
        r = cv._runlog(0)
        assert r["ok"] and r["n"] == 2 and [e["kind"] for e in r["entries"]] == ["call", "result"]
        assert [e["i"] for e in cv._runlog(1)["entries"]] == [1]          # incremental: only entries with index >= after
    finally:
        srv.stop()
    monkeypatch.setitem(cv._state, "observer_port", _free_port())
    assert "连不上" in cv._runlog(0)["error"]


def test_right_eye_comes_from_its_own_stream(monkeypatch):
    left, right = _jpeg(value=40), _jpeg(value=200)
    monkeypatch.setitem(cv._state, "cam", _Cam(left))
    monkeypatch.setitem(cv._state, "cam_right", _Cam(right))
    monkeypatch.setitem(cv._state, "binocular", False)
    assert cv._frame_jpeg("left") is left and cv._frame_jpeg("right") is right


def _page(monkeypatch, **state):
    import threading
    import urllib.request
    from http.server import ThreadingHTTPServer
    for k, v in state.items():
        monkeypatch.setitem(cv._state, k, v)
    srv = ThreadingHTTPServer(("127.0.0.1", 0), cv.Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{srv.server_address[1]}/", timeout=3) as r:
            return r.read().decode()
    finally:
        srv.shutdown()


def test_page_tiles_both_eyes_and_fills(monkeypatch):
    page = _page(monkeypatch, cam_right=_Cam(_jpeg()), binocular=False)
    assert "/stream?eye=left" in page and "/stream?eye=right" in page
    assert "repeat(2," in page and "object-fit:fill" in page and "%T" not in page


def test_page_single_tile_for_monocular_camera(monkeypatch):
    page = _page(monkeypatch, cam_right=None, binocular=False, eye="left")
    assert page.count('class="tile"') == 1 and "repeat(1," in page
