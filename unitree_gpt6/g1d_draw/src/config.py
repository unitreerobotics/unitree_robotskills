"""Deployment configuration, separate from task geometry and robot SDK imports."""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path

import numpy as np

MOTION_DEFAULTS = {
    "publish_hz": 250.0,
    "command_hz": 50.0,
    "state_timeout_s": 0.5,
    "connect_timeout_s": 10.0,
    "max_speed_rad_s": 0.5,
    "max_step_rad": 0.02,
    "tracking_tolerance_rad": 0.08,
    "tracking_window_s": 0.2,
    "arrival_tolerance_rad": 0.04,
    "settle_timeout_s": 5.0,
    "settle_time_s": 0.2,
    "stationary_speed_rad_s": 0.1,
    "grip_tolerance": 0.18,
    "waist_tolerance_rad": 0.03,
    "fk_position_tolerance_m": 0.003,
    "fk_orientation_tolerance_rad": 0.08,
    "contact_position_tolerance_m": 0.005,
    "runtime_orientation_tolerance_rad": 0.08,
}


def vector(value, count, label):
    result = np.asarray(value, dtype=float)
    if result.shape != (count,) or not np.isfinite(result).all():
        raise ValueError(f"{label} needs {count} finite numbers")
    return result


def waist_vector(value, label="waist_q3"):
    """Validate canonical G1-D [yaw, fixed-zero roll, pitch] model coordinates."""
    result = vector(value, 3, label)
    if result[1] != 0.0:
        raise ValueError(
            f"{label} must be [yaw, 0, pitch]: G1-D has no waist roll; "
            "re-measure and rebuild legacy tasks instead of reusing old slot-ordered values"
        )
    return result


def load_config(path):
    path = Path(path).expanduser().resolve()
    return normalize_config(json.loads(path.read_text(encoding="utf-8")), path.parent)


def normalize_config(value, base=None):
    config = copy.deepcopy(value)
    base = Path.cwd() if base is None else Path(base)
    if not isinstance(config, dict):
        raise ValueError("device configuration must be a JSON object")
    for name in ("device", "gains", "storage", "motion", "model"):
        if name in config and not isinstance(config[name], dict):
            raise ValueError(f"configuration.{name} must be a JSON object")
    device = config.setdefault("device", {})
    if not isinstance(device.get("interface"), str) or not device["interface"].strip():
        raise ValueError("device.interface must name the actual wired DDS interface")
    device.setdefault("robot_id", "g1d")
    if (
        not isinstance(device["robot_id"], str)
        or not device["robot_id"]
        or any(
            c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_"
            for c in device["robot_id"]
        )
    ):
        raise ValueError("device.robot_id must use letters, digits, hyphens or underscores")
    device.setdefault("domain_id", 0)
    if type(device["domain_id"]) is not int or device["domain_id"] < 0:
        raise ValueError("device.domain_id must be a nonnegative integer")
    device.setdefault("lowstate_topic", "rt/lowstate")
    device.setdefault("lowcmd_topic", "rt/lowcmd")
    device.setdefault("gripper_interface", "internal")
    if device["gripper_interface"] not in {"internal", "external"}:
        raise ValueError("gripper_interface must be internal or external")
    for key in ("lowstate_topic", "lowcmd_topic"):
        if not isinstance(device[key], str) or not device[key].strip():
            raise ValueError(f"device.{key} must be a topic name")
    if device["gripper_interface"] == "external":
        grippers = device.get("external_grippers", {})
        if not isinstance(grippers, dict):
            raise ValueError("device.external_grippers must be a JSON object")
        for side in ("left", "right"):
            topics = grippers.get(side, {})
            if not isinstance(topics, dict):
                raise ValueError(f"device.external_grippers.{side} must be a JSON object")
            if any(
                not isinstance(topics.get(key), str) or not topics[key].strip()
                for key in ("state", "command")
            ):
                raise ValueError(
                    "external grippers need explicit left/right state and command topics"
                )
    config["motion"] = {**MOTION_DEFAULTS, **config.get("motion", {})}
    # Older exports include these fields. Lifted position errors now only get
    # recorded, so accept the old configuration without restoring its gates.
    for name in ("lifted_xy_tolerance_m", "lifted_z_tolerance_m"):
        config["motion"].pop(name, None)
    unknown = set(config["motion"]) - set(MOTION_DEFAULTS)
    if unknown:
        raise ValueError(f"unknown motion options: {sorted(unknown)}")
    for name, number in config["motion"].items():
        if type(number) not in (float, int) or not math.isfinite(number) or number <= 0:
            raise ValueError(f"motion.{name} must be finite and positive")
    if config["motion"]["command_hz"] > config["motion"]["publish_hz"]:
        raise ValueError("command_hz cannot exceed publish_hz")
    gains = config.get("gains", {})
    for name, count in (
        ("arm_kp", 14),
        ("arm_kd", 14),
        ("waist_kp", 3),
        ("waist_kd", 3),
        ("gripper_kp", 2),
        ("gripper_kd", 2),
    ):
        numbers = vector(gains.get(name), count, f"gains.{name}")
        if np.any(numbers < 0) or (name.endswith("kp") and np.any(numbers <= 0)):
            raise ValueError(f"gains.{name} has invalid gains")
        gains[name] = numbers.tolist()
    config["gains"] = gains

    def resolve(value):
        p = Path(value).expanduser()
        return str((p if p.is_absolute() else base / p).resolve())

    state = config.setdefault("storage", {})
    # Keep the disk namespace stable so renaming cannot bypass locks or claims.
    default_root = Path.home() / ".local/state/astra-paint-execute" / device["robot_id"]
    state["directory"] = resolve(state.get("directory", str(default_root)))
    state["controller_lock"] = resolve(
        state.get("controller_lock", str(default_root / "controller.lock"))
    )
    state["additional_lock_files"] = [resolve(p) for p in state.get("additional_lock_files", [])]
    # Cooperate with the preserved legacy controller when run from this checkout.
    for ancestor in (base, *base.parents):
        if ancestor.name == "astra_paint" and (ancestor / "eef_session.py").is_file():
            legacy = str(ancestor / "runs/.eef_session.lock")
            if legacy not in state["additional_lock_files"]:
                state["additional_lock_files"].append(legacy)
            break
    if config.get("model"):
        model = config["model"]
        if not isinstance(model.get("urdf"), str) or not model["urdf"].strip():
            raise ValueError("model.urdf must point to the deployment's URDF")
        model["urdf"] = resolve(model["urdf"])
        model["package_dirs"] = [resolve(p) for p in model.get("package_dirs", [])]
    return config
