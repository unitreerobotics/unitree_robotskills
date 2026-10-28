"""Load packaged YAML configuration and reject unsupported execution modes."""

import math
from importlib.resources import files
from pathlib import Path
from typing import Any

import yaml


def section(config: dict[str, Any], name: str) -> dict[str, Any]:
    value = config.get(name)
    if not isinstance(value, dict):
        raise ValueError(f"Configuration requires a {name} mapping")
    return value


def finite_number(value: Any, name: str, *, positive: bool = False) -> float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    if positive and value <= 0:
        raise ValueError(f"{name} must be positive")
    return float(value)


def load_config(skill: str, expected_robot: str, path: str | Path | None = None) -> dict[str, Any]:
    source = Path(path) if path is not None else files(f"unitree_gpt6.{skill}") / "config"
    if path is None:
        source = source / "default.yaml"
    config = yaml.safe_load(source.read_text(encoding="utf-8"))
    validate_config(config, expected_robot)
    return config


def validate_config(config: Any, expected_robot: str) -> None:
    if not isinstance(config, dict):
        raise ValueError("Configuration must be a YAML mapping")
    if config.get("robot_model") != expected_robot:
        raise ValueError(f"This skill requires robot_model: {expected_robot}")
    if config.get("mode") != "preview":
        raise ValueError("Only preview mode is implemented; hardware execution is unavailable")
    if config.get("model_provider") != "none":
        raise ValueError("No model provider is implemented; set model_provider: none")
    limits = config.get("limits")
    if not isinstance(limits, dict):
        raise ValueError("Configuration requires limits")
    maximum = limits.get("max_actions")
    if type(maximum) is not int or not 1 <= maximum <= 100:
        raise ValueError("limits.max_actions must be an integer between 1 and 100")
    timeout = limits.get("task_timeout_s")
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("limits.task_timeout_s must be finite and positive")
