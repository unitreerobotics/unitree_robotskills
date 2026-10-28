"""Explicit hardware tooling entrypoint with a connection-free preflight command."""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import json
import math
import sys
from pathlib import Path
from typing import Any

import yaml

from unitree_gpt6.g1d_gomoku.models.g1 import validate_assets

from .common import configure, load_config, reset_state_path, runtime_dir, scene_geometry

DEPENDENCIES = (
    "numpy",
    "scipy",
    "cv2",
    "zmq",
    "pinocchio",
    "casadi",
    "cyclonedds",
    "unitree_sdk2py",
)


def module_available(name: str) -> bool:
    """Locate dependencies without loading DDS; the SDK must be skill-local."""
    module_name = "unitree_gpt6.g1d_gomoku.src.unitree_sdk2py" if name == "unitree_sdk2py" else name
    return importlib.util.find_spec(module_name) is not None


def finite_vector(value: Any, length: int, name: str) -> list[float]:
    if (
        not isinstance(value, list)
        or len(value) != length
        or any(type(item) not in (int, float) or not math.isfinite(item) for item in value)
    ):
        raise ValueError(f"{name} must contain {length} finite numbers")
    return [float(item) for item in value]


def validate_reset_reference(value: Any) -> None:
    """Check the reference schema, without claiming model or reachability validation."""
    if not isinstance(value, dict):
        raise ValueError("Reset reference must be a JSON mapping")
    names = value.get("joint_names")
    if (
        not isinstance(names, list)
        or len(names) != 14
        or any(not isinstance(name, str) or not name.strip() for name in names)
        or len(set(names)) != 14
    ):
        raise ValueError("Reset reference requires 14 unique joint_names")
    finite_vector(value.get("joint_positions"), 14, "joint_positions")
    finite_vector(value.get("waist_q"), 3, "waist_q")
    grippers = value.get("grippers")
    if not isinstance(grippers, dict):
        raise ValueError("Reset reference requires grippers")
    for side in ("left", "right"):
        position = grippers.get(side)
        if type(position) not in (int, float) or not math.isfinite(position):
            raise ValueError(f"Reset reference requires a finite {side} gripper position")
        if not 0 <= position <= 5.45:
            raise ValueError(f"Reset reference {side} gripper is outside Dex1 limits")


def model_check() -> dict[str, Any]:
    """Import numerical bindings and build the bundled model, never a device."""
    modules = {}
    for name in ("numpy", "scipy", "pinocchio", "pinocchio.casadi", "casadi"):
        try:
            module = importlib.import_module(name)
            modules[name] = {"available": True, "version": getattr(module, "__version__", None)}
        except (ImportError, OSError, RuntimeError) as exc:
            modules[name] = {
                "available": False,
                "error": {"type": type(exc).__name__, "message": str(exc)},
            }
    report = {
        "task_status": "model_check",
        "python_executable": sys.executable,
        "numerical_modules": modules,
        "model_initialized": False,
        "hardware_commands_sent": 0,
        "connection_checked": False,
    }
    if all(item["available"] for item in modules.values()):
        try:
            from .kinematics import Kinematics

            kin = Kinematics()
            report.update(model_initialized=True, arm_joint_count=len(kin.names))
        except (ImportError, OSError, RuntimeError, ValueError) as exc:
            report["model_error"] = {"type": type(exc).__name__, "message": str(exc)}
    return report


def validate_hardware_mode(config: dict[str, Any]) -> None:
    if (
        not isinstance(config, dict)
        or config.get("robot_model") != "G1-D"
        or config.get("mode") != "hardware"
    ):
        raise ValueError("Hardware tools require robot_model: G1-D and mode: hardware")


def preflight(config: dict[str, Any]) -> dict[str, Any]:
    """Validate local files and dependency presence; do not instantiate a device."""
    validate_hardware_mode(config)
    robot = config.get("robot")
    camera = config.get("camera")
    if not isinstance(robot, dict) or robot.get("backend") != "dds":
        raise ValueError("robot.backend must be dds")
    if (
        not isinstance(robot.get("network_interface"), str)
        or not robot["network_interface"].strip()
    ):
        raise ValueError("Set robot.network_interface in the local configuration")
    if (
        not isinstance(camera, dict)
        or not isinstance(camera.get("host"), str)
        or not camera["host"].strip()
    ):
        raise ValueError("Set camera.host in the local configuration")
    for key in (
        "state_timeout",
        "connect_timeout",
        "rate_hz",
        "max_joint_speed",
        "joint_tolerance",
        "settle_timeout",
        "gripper_speed",
        "gripper_tolerance",
        "ik_position_tolerance",
        "ik_rotation_tolerance",
        "arrival_position_tolerance",
        "arrival_rotation_tolerance",
    ):
        value = robot.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"robot.{key} must be finite and positive")
    for key in ("topic_low_command", "topic_low_state"):
        if not isinstance(robot.get(key), str) or not robot[key].strip():
            raise ValueError(f"robot.{key} must be a nonempty topic name")
    if type(robot.get("domain_id")) is not int or robot["domain_id"] < 0:
        raise ValueError("robot.domain_id must be a nonnegative integer")
    grippers = robot.get("grippers")
    if not isinstance(grippers, dict):
        raise ValueError("robot.grippers must configure the left and right Dex1 grippers")
    for side in ("left", "right"):
        settings = grippers.get(side)
        if not isinstance(settings, dict):
            raise ValueError(f"robot.grippers.{side} must be a mapping")
        for key in ("state_topic", "command_topic"):
            if not isinstance(settings.get(key), str) or not settings[key].strip():
                raise ValueError(f"robot.grippers.{side}.{key} must be a nonempty topic name")
        for key in ("min", "max", "open", "close"):
            value = settings.get(key)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"robot.grippers.{side}.{key} must be finite")
        lower, upper = settings["min"], settings["max"]
        if not 0 <= lower < upper <= 5.45:
            raise ValueError(f"robot.grippers.{side} limits must lie within [0, 5.45]")
        if not all(lower <= settings[key] <= upper for key in ("open", "close")):
            raise ValueError(f"robot.grippers.{side} open/close must lie within its limits")
    enabled = camera.get("enabled")
    views = {"cam_left_high", "cam_right_high", "cam_left_wrist", "cam_right_wrist"}
    if (
        not isinstance(enabled, list)
        or not enabled
        or any(not isinstance(name, str) or name not in views for name in enabled)
        or len(set(enabled)) != len(enabled)
    ):
        raise ValueError("camera.enabled must contain unique supported camera names")
    ports = camera.get("ports", {})
    for source in ("head", "left_wrist", "right_wrist"):
        if (
            not isinstance(ports, dict)
            or type(ports.get(source)) is not int
            or not 1 <= ports[source] <= 65535
        ):
            raise ValueError(f"camera.ports.{source} must be an integer TCP port")
    for key in ("stale_after", "web_fps"):
        value = camera.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"camera.{key} must be finite and positive")
    if type(camera.get("jpeg_quality")) is not int or not 1 <= camera["jpeg_quality"] <= 100:
        raise ValueError("camera.jpeg_quality must be an integer between 1 and 100")
    if not isinstance(camera.get("web_host"), str) or not camera["web_host"].strip():
        raise ValueError("camera.web_host must be a nonempty bind address")
    if type(camera.get("web_port")) is not int or not 0 <= camera["web_port"] <= 65535:
        raise ValueError("camera.web_port must be an integer between 0 and 65535")
    runtime_dir(config)
    reference = reset_state_path(config)
    validate_reset_reference(json.loads(reference.read_text(encoding="utf-8")))
    scene_geometry(config)
    dependencies = {name: module_available(name) for name in DEPENDENCIES}
    model_assets = validate_assets()
    return {
        "task_status": "preflight",
        "configuration_valid": True,
        "dependencies": dependencies,
        "model_assets": model_assets,
        "hardware_commands_sent": 0,
        "connection_checked": False,
        "calibration_checked": False,
        "task_verified": False,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--config", required=True, type=Path, help="Local hardware configuration")
    parser.add_argument(
        "command",
        choices=(
            "preflight",
            "model-check",
            "state",
            "pose",
            "camera",
            "session",
            "control",
            "reset",
        ),
    )
    parser.add_argument(
        "arguments", nargs=argparse.REMAINDER, help="Arguments for the selected tool"
    )
    args = parser.parse_args(argv)
    try:
        for argument in args.arguments:
            option = argument.partition("=")[0]
            if option.startswith("--") and option != "--" and "--config".startswith(option):
                raise ValueError(
                    "Set --config once before the tool name; tool overrides are disabled"
                )
        configure(args.config)
        config = load_config()
        config_path = config["_config_path"]
        validate_hardware_mode(config)
        if args.command == "model-check":
            if args.arguments:
                raise ValueError("model-check does not accept tool arguments")
            report = model_check()
            print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
            return 0 if report["model_initialized"] else 1
        if args.command == "preflight":
            if args.arguments:
                raise ValueError("preflight does not accept tool arguments")
            report = preflight(config)
            print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
            return 0 if all(report["dependencies"].values()) else 1
        modules = {
            "state": "arm_pose",
            "pose": "arm_pose",
            "camera": "camera_web",
            "session": "arm_session",
            "control": "gomoku_live_control",
            "reset": "reset_right",
        }
        tool = importlib.import_module(f"{__package__}.{modules[args.command]}")
        arguments = list(args.arguments)
        if args.command in ("state", "pose", "camera", "reset"):
            arguments = ["--config", config_path, *arguments]
        if args.command == "state":
            # arm_pose's global options precede its subcommand. Output paths are
            # available via `pose --output PATH state --grippers`.
            arguments = ["--config", config_path, "state", *args.arguments]
        return int(tool.main(arguments) or 0)
    except (ImportError, ValueError, RuntimeError, OSError, KeyError, yaml.YAMLError) as exc:
        # Hardware commands may have been sent by an explicit tool. Never claim
        # zero commands or a final physical state on that error path.
        result: dict[str, Any] = {
            "task_status": "failed",
            "task_verified": False,
            "error_info": {"type": type(exc).__name__, "message": str(exc)},
        }
        if args.command == "preflight":
            result.update(configuration_valid=False, hardware_commands_sent=0)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
