"""Explicit local hardware configuration and runtime paths; no device access."""

import hashlib
import json
import math
import os
from pathlib import Path

import yaml

_configured_path = None


def configure(path):
    """Select a config file for subsequent calls in this process, without I/O.

    Explicit load_config(path) arguments take precedence. Passing None clears
    this process default so GOMOKU_HARDWARE_CONFIG is consulted again.
    """
    global _configured_path
    if path is None:
        _configured_path = None
        return
    if not str(path).strip():
        raise ValueError("Hardware configuration path cannot be empty")
    _configured_path = Path(path).expanduser().resolve()


def load_config(path=None):
    """Read the explicitly selected YAML file, without connecting any devices."""
    if path is not None and not str(path).strip():
        raise ValueError("Hardware configuration path cannot be empty")
    selected = path if path is not None else _configured_path
    if selected is None:
        selected = os.environ.get("GOMOKU_HARDWARE_CONFIG")
    if not selected or not str(selected).strip():
        raise ValueError(
            "Hardware configuration is required: pass --config, call configure(path), "
            "or set GOMOKU_HARDWARE_CONFIG"
        )
    source = Path(selected).expanduser().resolve()
    with source.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("Configuration must be a YAML mapping")
    # Persist provenance in session metadata, and resolve paths independently of
    # the directory from which a CLI or background publisher was launched.
    config["_config_path"] = str(source)
    return config


def _runtime_setting(config, key):
    config = load_config() if config is None else config
    settings = config.get("runtime")
    if not isinstance(settings, dict):
        raise ValueError("Hardware configuration requires a runtime mapping")
    value = settings.get(key)
    if not isinstance(value, (str, os.PathLike)) or not str(value).strip():
        raise ValueError(f"Set runtime.{key} in the local hardware configuration")
    path = Path(value).expanduser()
    if not path.is_absolute():
        source = config.get("_config_path")
        if not source:
            raise ValueError("Relative runtime paths require a configuration loaded by load_config")
        path = Path(source).parent / path
    return path.resolve()


def runtime_dir(config=None):
    """Return runtime.directory relative to the config file; do not create it."""
    directory = _runtime_setting(config, "directory")
    if directory.exists() and not directory.is_dir():
        raise ValueError(f"runtime.directory must be a directory: {directory}")
    return directory


def controller_lock_path(config=None):
    """All motion tools must use this same lock for a given hardware deployment."""
    return runtime_dir(config) / ".arm_session.lock"


def reset_state_path(config=None):
    """Return an explicitly configured, existing robot-local reset-state file."""
    path = _runtime_setting(config, "reset_state")
    if not path.is_file():
        raise ValueError(f"runtime.reset_state must refer to an existing state file: {path}")
    return path


def vector(value, size, name):
    """Validate a numeric vector; NumPy is only needed by the hardware tools."""
    import numpy as np

    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result


def positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def scene_geometry(config=None):
    """Read explicit scene/tool calibration; no historical geometry fallbacks."""
    path = scene_calibration_path(config)
    raw = path.read_bytes()
    value = json.loads(raw)
    if not isinstance(value, dict) or value.get("frame") != "pelvis":
        raise ValueError("Scene calibration requires frame: pelvis")

    def finite_list(item, size):
        return (
            isinstance(item, list)
            and len(item) == size
            and all(type(v) in (int, float) and math.isfinite(v) for v in item)
        )

    if not finite_list(value.get("tool_tip_offset_m"), 3):
        raise ValueError("Measure scene tool_tip_offset_m as three finite numbers")
    for field in ("high_route_min_tcp_z_m", "high_route_min_tip_z_m"):
        if type(value.get(field)) not in (int, float) or not math.isfinite(value[field]):
            raise ValueError(f"Measure scene {field}")
    corridor = value.get("rest_corridor")
    if not isinstance(corridor, dict):
        raise ValueError("Scene calibration requires reviewed rest_corridor bounds")
    for side in ("left", "right"):
        bounds = corridor.get(side)
        if (
            not isinstance(bounds, dict)
            or not finite_list(bounds.get("min_m"), 3)
            or not finite_list(bounds.get("max_m"), 3)
            or any(a >= b for a, b in zip(bounds["min_m"], bounds["max_m"], strict=True))
        ):
            raise ValueError(f"Measure finite ordered {side} rest_corridor bounds")
    value["_sha256"] = hashlib.sha256(raw).hexdigest()
    return value


def scene_calibration_path(config=None):
    path = _runtime_setting(config, "scene_calibration")
    if not path.is_file():
        raise ValueError("Capture runtime.scene_calibration for the current scene before motion")
    return path
