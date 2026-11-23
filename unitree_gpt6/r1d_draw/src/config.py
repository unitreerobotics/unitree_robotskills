"""Device configuration: how to reach one R1-D, which model to use, where to keep state.

The robot is controlled by an onboard process started over wired SSH, so the
configuration names the SSH endpoint instead of a DDS network interface.
Task geometry (paper, paint, strokes) is not stored here.
"""

from __future__ import annotations

import copy
import ipaddress
import json
import re
from pathlib import Path

from .schema import array, number, text, vector

SECTIONS = ("device", "cameras", "model", "motion", "workspace", "storage")
MOTION_DEFAULTS = {
    # Peak joint speed with the brush in the air; the controller refuses more than 0.3 rad/s.
    "max_speed_rad_s": 0.09,
    # Peak joint speed while the brush touches paper, paint or water, and when it leaves them.
    "contact_speed_rad_s": 0.05,
    # Peak joint speed of the entry and park routes: the load on the arm changes a lot there.
    "posture_speed_rad_s": 0.05,
    # Brush-tip speed along a stroke, and in the air between the stations and the paper.
    "stroke_speed_m_s": 0.01,
    "travel_speed_m_s": 0.05,
    # Spacing of the joint samples used for offline path checks.
    "check_step_rad": 0.02,
    # Offline IK/FK agreement required when a job is compiled.
    "fk_position_tolerance_m": 0.003,
    "fk_orientation_tolerance_rad": 0.08,
    # Allowed difference between the measured arms and the first entry point of a job.
    "start_tolerance_rad": 0.04,
    "grip_tolerance": 0.18,
    "waist_tolerance_rad": 0.03,
}
_ROBOT_ID = re.compile(r"[A-Za-z0-9_-]+")
_MAC = re.compile(r"[0-9a-f]{2}(:[0-9a-f]{2}){5}")
_ENV_NAME = re.compile(r"[A-Z_][A-Z0-9_]*")
_SIZE = re.compile(r"[1-9][0-9]*x[1-9][0-9]*")


def load_config(path):
    path = Path(path).expanduser().resolve()
    return normalize_config(json.loads(path.read_text(encoding="utf-8")), path.parent)


def _address(value, label):
    try:
        return str(ipaddress.ip_address(text(value, label)))
    except ValueError as exc:
        raise ValueError(f"{label} must be an IP address") from exc


def _host(entry, label, defaults):
    """One SSH endpoint: address, login and the wired MAC that identifies the machine."""
    entry["host"] = _address(entry.get("host"), f"{label}.host")
    entry["user"] = text(entry.setdefault("user", defaults["user"]), f"{label}.user")
    entry["interface"] = text(
        entry.setdefault("interface", defaults["interface"]), f"{label}.interface"
    )
    # The MAC of that interface is checked after connecting, before anything else runs.
    mac = entry.get("mac")
    if not isinstance(mac, str) or not _MAC.fullmatch(mac.lower()):
        raise ValueError(f"{label}.mac must be a wired MAC address, aa:bb:cc:dd:ee:ff")
    entry["mac"] = mac.lower()
    if "password" in entry:
        raise ValueError(f"do not store a password in the configuration; use {label}.password_env")
    name = entry.setdefault("password_env", defaults["password_env"])
    if not isinstance(name, str) or not _ENV_NAME.fullmatch(name):
        raise ValueError(f"{label}.password_env must name an environment variable")


def _device(device):
    device.setdefault("robot_id", "r1d")
    if not isinstance(device["robot_id"], str) or not _ROBOT_ID.fullmatch(device["robot_id"]):
        raise ValueError("device.robot_id must use letters, digits, hyphens or underscores")
    _host(
        device,
        "device",
        {"user": "unitree", "interface": "eth0", "password_env": "R1D_SSH_PASSWORD"},
    )
    # Bind the SSH socket to the wired interface when the workstation has several.
    if device.setdefault("source_ip", None) is not None:
        device["source_ip"] = _address(device["source_ip"], "device.source_ip")


def _cameras(cameras, device):
    """Cameras are read with ffmpeg on the computer they are attached to."""
    for name, camera in cameras.items():
        label = f"cameras.{name}"
        if not _ROBOT_ID.fullmatch(name) or not isinstance(camera, dict):
            raise ValueError(f"{label} must be a named JSON object")
        _host(camera, label, device)
        if camera.get("kind") not in ("rtsp", "v4l2"):
            raise ValueError(f"{label}.kind must be rtsp or v4l2")
        text(camera.get("source"), f"{label}.source")
        if camera["kind"] == "v4l2":
            size = camera.setdefault("size", "1920x1080")
            if not isinstance(size, str) or not _SIZE.fullmatch(size):
                raise ValueError(f"{label}.size must look like 1920x1080")


def normalize_config(value, base=None):
    """Validate a configuration object and resolve its paths against ``base``."""
    if not isinstance(value, dict):
        raise ValueError("device configuration must be a JSON object")
    unknown = set(value) - set(SECTIONS)
    if unknown:
        raise ValueError(f"unknown configuration sections: {sorted(unknown)}")
    config = copy.deepcopy(value)
    for name in SECTIONS:
        if name in config and not isinstance(config[name], dict):
            raise ValueError(f"configuration.{name} must be a JSON object")
    base = Path.cwd() if base is None else Path(base)

    def resolve(path, label):
        path = Path(text(path, label)).expanduser()
        return str((path if path.is_absolute() else base / path).resolve())

    device = config.setdefault("device", {})
    _device(device)
    _cameras(config.setdefault("cameras", {}), device)

    # The URDF and its meshes are supplied by the user; nothing is bundled.
    if "model" in config:
        model = config["model"]
        model["urdf"] = resolve(model.get("urdf"), "model.urdf")
        meshes = model.setdefault("mesh_dirs", [])
        if not isinstance(meshes, list):
            raise ValueError("model.mesh_dirs must be a list of directories")
        model["mesh_dirs"] = [resolve(p, "model.mesh_dirs entry") for p in meshes]
        # Poses are expressed in base_frame, so the lift height does not enter them.
        text(model.setdefault("base_frame", "pelvis_link"), "model.base_frame")
        for hand in ("left", "right"):
            text(model.setdefault(f"{hand}_frame", f"{hand}_wrist_yaw_link"), f"model.{hand}_frame")
            # Rigid transform from that frame to the brush tip.
            key = f"{hand}_tool_transform"
            identity = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
            model[key] = array(model.get(key, identity), (4, 4), f"model.{key}").tolist()

    motion = config["motion"] = {**MOTION_DEFAULTS, **config.get("motion", {})}
    unknown = set(motion) - set(MOTION_DEFAULTS)
    if unknown:
        raise ValueError(f"unknown motion options: {sorted(unknown)}")
    for name, value in motion.items():
        motion[name] = number(value, f"motion.{name}", positive=True)
    for name in ("max_speed_rad_s", "contact_speed_rad_s", "posture_speed_rad_s"):
        if motion[name] > 0.3:
            raise ValueError(f"motion.{name} cannot exceed the controller limit of 0.3")
    if "workspace" in config:
        for name in ("lower", "upper"):
            config["workspace"][name] = vector(
                config["workspace"].get(name), 3, f"workspace.{name}"
            ).tolist()

    storage = config.setdefault("storage", {})
    default_root = Path.home() / ".local/state/r1d-draw" / device["robot_id"]
    storage["directory"] = resolve(storage.get("directory", str(default_root)), "storage.directory")
    storage["controller_lock"] = resolve(
        storage.get("controller_lock", str(Path(storage["directory"]) / "controller.lock")),
        "storage.controller_lock",
    )
    return config
