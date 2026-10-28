# -*- coding: utf-8 -*-
"""Publish-decision tests for RobotController in services/robots/g1d.py (SDK-unavailable path, pure logic).

agv Move is a non-latching velocity servo: streaming Move(0,0,0) actively locks the chassis (cannot be pushed by hand).
After the watchdog zeroes the command, sending must stop within ZERO_RELEASE_S to release the chassis."""
import pytest

import config
import services.robots.g1d as robot_mod
from services.robots.g1d import RobotController


@pytest.fixture(autouse=True)
def _no_sdk(monkeypatch):
    """Take the SDK-unavailable branch: no DDS/RPC init, only the fields needed for publish decisions."""
    monkeypatch.setattr(robot_mod, "SDK_AVAILABLE", False)
    monkeypatch.setattr(config, "CMD_WATCHDOG_S", 0.5)
    monkeypatch.setattr(config, "ZERO_RELEASE_S", 1.0)


@pytest.fixture
def rc():
    r = RobotController()
    # send_command returns early without the SDK; write the buffer directly to simulate a command
    with r.cmd_lock:
        r.current_cmd = (0.3, 0.0)
        r._last_cmd_time = 100.0
    return r


def test_no_command_never_publishes():
    r = RobotController()
    assert r._resolve_payload(123.0) is None


def test_fresh_command_is_published(rc):
    assert rc._resolve_payload(100.3) == (0.3, 0.0)


def test_watchdog_degrades_to_zero(rc):
    assert rc._resolve_payload(100.6) == (0.0, 0.0)
    assert rc._resolve_payload(101.4) == (0.0, 0.0)


def test_zero_stream_released_after_window(rc):
    """0.5s watchdog + 1.0s zero window: sending stops after 101.5s and the chassis is released."""
    assert rc._resolve_payload(101.6) is None
    assert rc._resolve_payload(3600.0) is None


def test_new_command_resumes_publishing(rc):
    assert rc._resolve_payload(200.0) is None
    with rc.cmd_lock:
        rc.current_cmd = (0.0, 0.4)
        rc._last_cmd_time = 200.0
    assert rc._resolve_payload(200.1) == (0.0, 0.4)


def test_navigation_handoff_discards_manual_stream_and_blocks_commands(rc, monkeypatch):
    from types import SimpleNamespace
    sent = []
    rc._available = True  # Only fake RPC, never SDK initialization.
    rc.agv = SimpleNamespace(Move=lambda *args: (sent.append(args) or (0, {})))
    rc.acquire_navigation()
    assert rc._navigation_owned and rc._resolve_payload(100.) is None
    assert sent == [(0., 0., 0.)]
    with pytest.raises(RuntimeError): rc.send_command(.2)
    rc.publish_running = True
    monkeypatch.setattr(robot_mod.time, 'sleep', lambda _: setattr(rc, 'publish_running', False))
    rc._publish_loop()
    assert sent == [(0., 0., 0.)]  # No even-zero velocity stream while onboard planner owns motion.
    rc.release_navigation()
    assert rc._resolve_payload(100.) is None
    rc.send_command(.2)
    assert rc.current_cmd == (.2, 0.)
