# -*- coding: utf-8 -*-
"""R1D camera service auto-switching: service-layer logic + driver connect flow (all fakes, no DDS/UDP)."""
from types import SimpleNamespace

from core.driver import HeadlessDriver
from services.r1d_camera_services import switch_camera_services
from services.robots.profile import build_profile


class FakeClient:
    def __init__(self, services, list_code=0, switch_codes=None):
        self.services, self.list_code = services, list_code
        self.switch_codes = switch_codes or {}
        self.apply_on_switch = set()
        self.calls = []

    def ServiceList(self):
        return self.list_code, self.services

    def ServiceSwitch(self, name, enabled):
        self.calls.append((name, enabled))
        if name in self.apply_on_switch:            # real robot: nonzero return code, yet the service did switch
            for s in self.services:
                if s.name == name:
                    s.status = 0 if enabled else 1
        return self.switch_codes.get(name, 0)


def _svc(name, protect=False, status=0):
    return SimpleNamespace(name=name, protect=protect, status=status)


def _run(client):
    return switch_camera_services(client_factory=lambda t: client, confirm_s=0.0)


def test_switch_order_and_targets():
    c = FakeClient([_svc("video_hub", status=0), _svc("stereo_patch_pc1", status=1)])
    res = _run(c)
    assert res["ok"] and res["error"] is None
    assert c.calls == [("video_hub", False), ("stereo_patch_pc1", True)]


def test_services_already_in_target_state_are_skipped():
    # Field case: video_hub already stopped (status=1); sending "stop" again only waits out an RPC timeout
    c = FakeClient([_svc("video_hub", status=1), _svc("stereo_patch_pc1", status=1)])
    res = _run(c)
    assert res["ok"] and c.calls == [("stereo_patch_pc1", True)] and res["switched"] == ["stereo_patch_pc1"]
    c = FakeClient([_svc("video_hub", status=1), _svc("stereo_patch_pc1", status=0)])
    res = _run(c)
    assert res["ok"] and c.calls == [] and res["switched"] == []


def test_disabled_video_hub_is_not_switched_off_again():
    # On some R1D units video_hub is listed with status=-1 (disabled); it must not be stopped
    c = FakeClient([_svc("video_hub", status=-1), _svc("stereo_patch_pc1", status=1)])
    res = _run(c)
    assert res["ok"] and c.calls == [("stereo_patch_pc1", True)] and res["switched"] == ["stereo_patch_pc1"]


def test_missing_service_refuses():
    c = FakeClient([_svc("video_hub")])
    res = _run(c)
    assert not res["ok"] and "stereo_patch_pc1" in res["error"] and c.calls == []


def test_protected_service_refuses():
    c = FakeClient([_svc("video_hub", protect=True), _svc("stereo_patch_pc1")])
    res = _run(c)
    assert not res["ok"] and "受保护" in res["error"] and c.calls == []


def test_service_list_failure():
    res = _run(FakeClient([], list_code=3104))
    assert not res["ok"] and "3104" in res["error"]


def test_partial_switch_failure_reported_but_continues():
    c = FakeClient([_svc("video_hub"), _svc("stereo_patch_pc1", status=1)], switch_codes={"video_hub": 7})
    res = _run(c)
    assert not res["ok"] and res["switched"] == ["stereo_patch_pc1"] and "code=7" in res["error"]


def test_nonzero_code_but_service_reached_target_counts_as_switched():
    # Real robot: starting stereo_patch_pc1 returns 5201 (SDK reports the transitional state as failure), then status becomes 0 and port 5002 streams
    c = FakeClient([_svc("video_hub", status=1), _svc("stereo_patch_pc1", status=1)],
                   switch_codes={"stereo_patch_pc1": 5201})
    c.apply_on_switch = {"stereo_patch_pc1"}
    res = _run(c)
    assert res["ok"] and res["error"] is None and res["switched"] == ["stereo_patch_pc1"]


def test_client_init_failure_does_not_raise():
    def boom(t):
        raise RuntimeError("no dds")
    res = switch_camera_services(client_factory=boom)
    assert not res["ok"] and "no dds" in res["error"]


# ---------------- driver connect flow ----------------
def _driver(monkeypatch, frames_after_switch=True, frames_initially=False, **extra):
    d = object.__new__(HeadlessDriver)
    d.profile = build_profile("r1d", extra)
    d.camera = None
    d.logs = []
    d._log_fn = lambda msg, kind="info": d.logs.append(msg)
    state = {"switched": 0, "waits": []}

    def fake_wait(cfg, timeout_s):
        state["waits"].append(timeout_s)
        ok = frames_initially or (state["switched"] and frames_after_switch)
        return (SimpleNamespace(host="0.0.0.0", port=5002), None) if ok else (None, "超时")

    def fake_switch(timeout_s, log=None):
        state["switched"] += 1
        return {"ok": True, "switched": ["video_hub", "stereo_patch_pc1"], "error": None}

    monkeypatch.setattr(d, "_wait_rtp_camera", fake_wait)
    import services.r1d_camera_services as mod
    monkeypatch.setattr(mod, "switch_camera_services", fake_switch)
    return d, state


def test_already_streaming_does_not_touch_services(monkeypatch):
    d, st = _driver(monkeypatch, frames_initially=True)
    assert d._connect_rtp_camera({}, 10.0) is True
    assert st["switched"] == 0 and st["waits"] == [3.0]


def test_no_stream_switches_then_connects(monkeypatch):
    d, st = _driver(monkeypatch)
    assert d._connect_rtp_camera({}, 10.0) is True
    assert st["switched"] == 1 and st["waits"] == [3.0, 15.0]


def test_switch_does_not_help_reports_failure(monkeypatch):
    d, st = _driver(monkeypatch, frames_after_switch=False)
    assert d._connect_rtp_camera({}, 10.0) is False
    assert d.camera is None and st["switched"] == 1


def test_auto_switch_disabled_is_passive(monkeypatch):
    d, st = _driver(monkeypatch, camera_auto_switch=False)
    assert d._connect_rtp_camera({}, 10.0) is False
    assert st["switched"] == 0 and st["waits"] == [10.0]
