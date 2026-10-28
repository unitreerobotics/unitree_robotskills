# -*- coding: utf-8 -*-
"""Process-wide HeadlessDriver singleton plus basic tool helpers decoupled from FastMCP.

Environment variables:
    NAVI_NO_CAMERA=1          skip camera connection (motion/status only)
    NAVI_CAMERA_HOST          camera host override (default: config camera.host)
    NAVI_CAMERA_REQUEST_PORT  camera config request port override
"""
import os
import threading

from config import load_camera_defaults, load_config
from core.driver import HeadlessDriver

_driver = None
_driver_lock = threading.Lock()  # guards lazy creation so concurrent tool calls build one driver


def _camera_cfg_from_env():
    if os.environ.get("NAVI_NO_CAMERA") in ("1", "true", "True"):
        return None
    cam = load_camera_defaults(load_config())
    if os.environ.get("NAVI_CAMERA_HOST"):
        cam["host"] = os.environ["NAVI_CAMERA_HOST"]
    if os.environ.get("NAVI_CAMERA_REQUEST_PORT"):
        cam["request_port"] = int(os.environ["NAVI_CAMERA_REQUEST_PORT"])
    return cam


def get_driver() -> HeadlessDriver:
    """Create and connect the driver (DDS init + camera) on first call; reuse it afterwards.

    The driver is cached even if connection fails and never rebuilt: DDS
    ChannelFactoryInitialize may only be called once per process (see
    services/robots/probe.py), and rebuilding would re-initialise it. Motion tools then
    report the error honestly; restart the service to retry."""
    global _driver
    with _driver_lock:
        if _driver is None:
            d = HeadlessDriver(cfg=load_config(), camera_cfg=_camera_cfg_from_env())
            d.connect()
            if d.camera_cfg:
                d.connect_camera(timeout_s=10.0)
            _driver = d
    return _driver


def current_driver():
    """Return the driver without creating it, for read-only paths such as observation polling
    (if no tool has run yet there is no robot connection to report)."""
    return _driver


def tool_status(driver) -> dict:
    return {"ok": True, **driver.status()}


def tool_odometry(driver) -> dict:
    s = driver.status()
    return {"ok": s["position"] is not None,
            "position": s["position"], "yaw": s["yaw"]}


def tool_stop(driver) -> dict:
    driver.stop()
    return {"ok": True, "status": "stopped"}
