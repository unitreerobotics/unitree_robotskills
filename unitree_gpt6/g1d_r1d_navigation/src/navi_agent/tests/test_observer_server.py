# -*- coding: utf-8 -*-
"""Tests for services/observer.py: the three routes of the read-only status surface and error handling."""
import json
import urllib.error
import urllib.request

import pytest

from services.observer import ObserverServer


@pytest.fixture
def server():
    st = {"seq": {"active": True, "remaining_s": 3.0}}
    srv = ObserverServer(lambda: st, lambda: b"\xff\xd8jpg", port=0)
    srv.start()
    base = f"http://127.0.0.1:{srv._httpd.server_address[1]}"
    yield srv, base
    srv.stop()


def _get(url):
    with urllib.request.urlopen(url, timeout=3) as r:
        return r.status, r.headers.get("Content-Type"), r.read()


def test_state_route_returns_json(server):
    srv, base = server
    code, ctype, body = _get(f"{base}/state")
    assert code == 200 and ctype.startswith("application/json")
    assert json.loads(body)["seq"]["active"] is True


def test_frame_route_returns_jpeg(server):
    srv, base = server
    code, ctype, body = _get(f"{base}/frame.jpg")
    assert code == 200 and ctype == "image/jpeg" and body == b"\xff\xd8jpg"


def test_frame_route_503_when_no_frame():
    srv = ObserverServer(lambda: {}, lambda: None, port=0)
    srv.start()
    try:
        base = f"http://127.0.0.1:{srv._httpd.server_address[1]}"
        with pytest.raises(urllib.error.HTTPError) as ei:
            _get(f"{base}/frame.jpg")
        assert ei.value.code == 503
    finally:
        srv.stop()


def test_unknown_route_404(server):
    srv, base = server
    with pytest.raises(urllib.error.HTTPError) as ei:
        _get(f"{base}/nope")
    assert ei.value.code == 404


def test_state_fn_error_returns_500_not_crash():
    """A state-builder exception yields HTTP 500 instead of a dropped connection, so plugin polling keeps working."""
    def boom():
        raise RuntimeError("炸")
    srv = ObserverServer(boom, lambda: None, port=0)
    srv.start()
    try:
        base = f"http://127.0.0.1:{srv._httpd.server_address[1]}"
        with pytest.raises(urllib.error.HTTPError) as ei:
            _get(f"{base}/state")
        assert ei.value.code == 500
        assert srv.running is True               # server survives (ThreadingHTTPServer uses a thread per request)
    finally:
        srv.stop()


def test_stop_is_idempotent(server):
    srv, _ = server
    srv.stop()
    srv.stop()
    assert srv.running is False


def test_injected_json_route_gets_query_params():
    seen = {}

    def route(q):
        seen.update(q)
        return {"ok": True, "after": q.get("after")}
    srv = ObserverServer(lambda: {}, lambda: None, port=0, routes={"/runlog": route})
    srv.start()
    try:
        base = f"http://127.0.0.1:{srv._httpd.server_address[1]}"
        code, ctype, body = _get(f"{base}/runlog?after=7")
        assert code == 200 and ctype.startswith("application/json")
        assert json.loads(body) == {"ok": True, "after": "7"} and seen == {"after": "7"}
    finally:
        srv.stop()


def test_post_only_reaches_registered_routes():
    got = []
    srv = ObserverServer(lambda: {}, lambda: None, port=0, post_routes={"/runlog": lambda b: got.append(b) or {"ok": True}})
    srv.start()
    try:
        base = f"http://127.0.0.1:{srv._httpd.server_address[1]}"

        def post(path, data):
            req = urllib.request.Request(base + path, data=data, method="POST",
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=3) as r:
                return r.status, json.loads(r.read())
        assert post("/runlog", b'{"kind":"say","text":"hi"}') == (200, {"ok": True})
        assert got == [{"kind": "say", "text": "hi"}]
        with pytest.raises(urllib.error.HTTPError) as ei:
            post("/state", b'{}')                                  # unregistered paths are rejected
        assert ei.value.code == 404
        with pytest.raises(urllib.error.HTTPError) as ei:
            post("/runlog", b'not json')
        assert ei.value.code == 400
    finally:
        srv.stop()
