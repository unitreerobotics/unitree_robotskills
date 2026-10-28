# -*- coding: utf-8 -*-
"""r1d_camera_services.py -- R1D left-eye camera service switching.

After reboot the R1D runs video_hub by default and the left-eye RTP-H264 stream is not
pushed to local UDP 5002. Port 5002 only carries video after
  video_hub -> off, stereo_patch_pc1 -> on
This module does that via RobotStateClient (ServiceList / ServiceSwitch).

Constraints:
  - Reuses the in-process DDS already initialised by R1DRobot (ChannelFactoryInitialize);
    this module never initialises DDS.
  - Refuses to switch when a service is missing or protect=True.
  - Does NOT enter damping mode (that is a chassis FSM switch, i.e. motion control,
    not a camera-layer concern).
  - Failures return an error description and never raise; the final verdict is whether
    fresh frames arrive on 5002.
  - Services already in the target state are skipped: sending "off" to a stopped service
    still waits out the full RPC timeout.
  - ServiceSwitch takes ~10 s to return server-side (measured on R1D: enabling
    stereo_patch_pc1 takes ~10 s and returns 5201 although the service is up and 5002
    streams right after), so the timeout must be well above 10 s or the client reports 3104 first.
"""
from __future__ import annotations

import time

# (service name, target enabled); applied in order: stop video_hub, then start stereo_patch_pc1
CAMERA_SWITCHES = (("video_hub", False), ("stereo_patch_pc1", True))

# ServiceList status: 0 = running, 1 = stopped (on R1D, resident services such as
# robot_state / master_service report 0)
STATUS_RUNNING, STATUS_STOPPED = 0, 1


def _default_client(timeout_s: float):
    from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
    client = RobotStateClient()
    client.SetTimeout(float(timeout_s))
    client.Init()
    return client


def _reached_target(client, name: str, enabled: bool, confirm_s: float, sleep_fn) -> bool:
    """Re-check the real state after ServiceSwitch returns non-zero. The SDK reports the
    service's starting transition (observed status=104) as 5201 although the service is
    coming up; reaching the target state within confirm_s counts as success."""
    target = STATUS_RUNNING if enabled else STATUS_STOPPED
    deadline = time.monotonic() + confirm_s
    while True:
        try:
            code, services = client.ServiceList()
        except Exception:
            return False
        if code == 0 and any(getattr(s, "name", None) == name and getattr(s, "status", None) == target
                             for s in services or []):
            return True
        if time.monotonic() >= deadline:
            return False
        sleep_fn(0.5)


def switch_camera_services(timeout_s: float = 20.0, *, client_factory=None, log=None,
                           confirm_s: float = 5.0, sleep_fn=time.sleep) -> dict:
    """Switch the R1D camera services. Returns {"ok": bool, "switched": [...], "error": str|None}."""
    log = log or (lambda *a, **k: None)
    try:
        client = (client_factory or _default_client)(timeout_s)
    except ImportError as exc:
        return {"ok": False, "switched": [], "error": f"SDK 缺少 RobotStateClient：{exc}"}
    except Exception as exc:
        return {"ok": False, "switched": [], "error": f"RobotStateClient 初始化失败：{type(exc).__name__}: {exc}"}
    try:
        code, services = client.ServiceList()
    except Exception as exc:
        return {"ok": False, "switched": [], "error": f"ServiceList 异常：{type(exc).__name__}: {exc}"}
    if code != 0:
        return {"ok": False, "switched": [], "error": f"ServiceList code={code}"}
    states = {getattr(s, "name", None): s for s in services or []}
    for name, _ in CAMERA_SWITCHES:
        if name not in states:
            return {"ok": False, "switched": [], "error": f"机器人未提供 {name} 服务（相机补丁未安装？）"}
        if getattr(states[name], "protect", False):
            return {"ok": False, "switched": [], "error": f"服务 {name} 受保护，未切换"}
    switched, errors = [], []
    for name, enabled in CAMERA_SWITCHES:
        # A service to stop counts as done if it is not running: some R1Ds report video_hub
        # status=-1 (disabled), and sending "off" would only wait out an RPC timeout and
        # record a spurious error.
        status = getattr(states[name], "status", None)
        if (status == STATUS_RUNNING) if enabled else (status != STATUS_RUNNING):
            log(f"R1D 相机服务 {name} 已{'运行' if enabled else '停止'}（status={status}），跳过", "camera")
            continue
        log(f"R1D 相机服务切换：{name} → {'开' if enabled else '关'}"
            f"（当前 status={getattr(states[name], 'status', '?')}）", "camera")
        try:
            rc = client.ServiceSwitch(name, enabled)
        except Exception as exc:
            errors.append(f"{name}: {type(exc).__name__}: {exc}")
            continue
        if rc == 0 or _reached_target(client, name, enabled, confirm_s, sleep_fn):
            switched.append(name)
        else:
            errors.append(f"{name}: code={rc}")
    # Partial failure is not fatal: the caller decides by whether frames arrive afterwards.
    return {"ok": not errors, "switched": switched, "error": "; ".join(errors) or None}
