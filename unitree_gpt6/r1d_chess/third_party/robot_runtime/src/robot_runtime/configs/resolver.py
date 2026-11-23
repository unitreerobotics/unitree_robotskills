"""Load and resolve experiment and robot configuration sources."""

from collections.abc import Mapping
from pathlib import Path

import yaml

from robot_runtime.configs.common import (
    reject_unknown_fields,
    require_non_empty_str,
)


PROJECT_ROOT = Path(__file__).resolve().parents[3]
EXPERIMENT_ROOT = PROJECT_ROOT / "configs/experiments"


def read_yaml(path: str | Path) -> dict[str, object]:
    yaml_path = Path(path).expanduser()
    if not yaml_path.is_absolute():
        yaml_path = PROJECT_ROOT / yaml_path
    with yaml_path.open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise TypeError(f"YAML config {yaml_path} must be a mapping.")
    return data


def relative_config_path(path: str | Path) -> str:
    target_path = Path(path).expanduser()
    if not target_path.is_absolute():
        target_path = PROJECT_ROOT / target_path
    try:
        relative_path = target_path.resolve().relative_to(PROJECT_ROOT.resolve())
    except ValueError as exc:
        raise ValueError(
            f"Config path {target_path} is outside the repository."
        ) from exc
    return relative_path.as_posix()


def _require_mapping(
    cfg: Mapping[str, object],
    key: str,
    *,
    path: str,
) -> dict[str, object]:
    try:
        value = cfg[key]
    except KeyError as exc:
        raise KeyError(f"{path}.{key} must be configured.") from exc
    if not isinstance(value, dict):
        raise TypeError(f"{path}.{key} must be a mapping.")
    return value


def _normalize_experiment_source(
    cfg: Mapping[str, object],
    *,
    source_path: str,
    expected_policy_id: str,
) -> dict[str, object]:
    reject_unknown_fields(
        cfg,
        frozenset(
            {
                "environment",
                "runtime",
                "robot",
                "policy",
                "recording",
            }
        ),
        label=source_path,
    )

    environment = cfg.get("environment")
    if environment not in {"real", "sim"}:
        raise ValueError(
            f"{source_path}.environment must be 'real' or 'sim', "
            f"got {environment!r}."
        )
    robot_cfg = _require_mapping(cfg, "robot", path=source_path)
    runtime_cfg = _require_mapping(cfg, "runtime", path=source_path)
    reject_unknown_fields(
        robot_cfg,
        frozenset({"spec", "drivers", "connections"}),
        label=f"{source_path}.robot",
    )
    robot_ref = robot_cfg.get("spec")
    if not isinstance(robot_ref, str) or not robot_ref:
        raise ValueError(f"{source_path}.robot.spec must be a non-empty ref.")

    driver_cfgs = robot_cfg.get("drivers")
    if not isinstance(driver_cfgs, list) or not driver_cfgs:
        raise TypeError(
            f"{source_path}.robot.drivers must be a non-empty list."
        )
    connections_cfg = robot_cfg.get("connections", {})
    if not isinstance(connections_cfg, Mapping):
        raise TypeError(f"{source_path}.robot.connections must be a mapping.")

    policy_cfg = dict(_require_mapping(cfg, "policy", path=source_path))
    declared_policy_id = policy_cfg.get("policy_id")
    if declared_policy_id != expected_policy_id:
        raise ValueError(
            f"{source_path}.policy.policy_id must match experiment directory "
            f"{expected_policy_id!r}, got {declared_policy_id!r}."
        )

    normalized = {
        "environment": environment,
        "runtime": runtime_cfg,
        "robot": {
            "spec": robot_ref,
            "drivers": driver_cfgs,
            "connections": connections_cfg,
        },
        "policy": policy_cfg,
    }
    if "recording" in cfg:
        normalized["recording"] = cfg["recording"]
    return normalized


def _config_ref_path(
    ref: str,
    *,
    root: Path,
    path: str,
) -> Path:
    if not isinstance(ref, str) or not ref:
        raise ValueError(f"{path} must be a non-empty repository-relative path.")

    ref_path = Path(ref)
    if ref_path.is_absolute():
        raise ValueError(f"{path} must be repository-relative, got {ref!r}.")
    if ref_path.suffix != ".yaml":
        raise ValueError(f"{path} must end with '.yaml', got {ref!r}.")
    if ".." in ref_path.parts:
        raise ValueError(f"{path} must not contain '..', got {ref!r}.")

    resolved_path = (PROJECT_ROOT / ref_path).resolve()
    try:
        resolved_path.relative_to(root.resolve())
    except ValueError as exc:
        expected_root = relative_config_path(root)
        raise ValueError(
            f"{path} must be under {expected_root}, got {ref!r}."
        ) from exc
    return resolved_path


def _component_ref(
    ref,
    *,
    path: str,
) -> dict[str, object]:
    component_root = PROJECT_ROOT / "configs/robots/components"
    component_file = _config_ref_path(
        ref,
        root=component_root,
        path=path,
    )
    source_path = relative_config_path(component_file)
    component_cfg = read_yaml(component_file)
    if "source_path" in component_cfg:
        raise KeyError(
            f"Robot component {source_path} must not define unsupported "
            "field 'source_path'."
        )
    require_non_empty_str(
        component_cfg.get("component_id"),
        label=f"Robot component {source_path}.component_id",
    )
    return component_cfg


def resolve_robot_spec(ref: str, *, path: str) -> dict[str, object]:
    composition_root = PROJECT_ROOT / "configs/robots/compositions"
    spec_file = _config_ref_path(
        ref,
        root=composition_root,
        path=path,
    )
    source_path = relative_config_path(spec_file)
    relative_spec_path = spec_file.relative_to(composition_root.resolve())
    if len(relative_spec_path.parts) != 2:
        raise ValueError(
            f"{path} must identify a robot composition YAML as "
            f"<body>/<variant>.yaml under "
            f"{relative_config_path(composition_root)}, got {ref!r}."
        )
    composition_body_key = relative_spec_path.parts[0]

    spec_cfg = read_yaml(spec_file)
    reject_unknown_fields(
        spec_cfg,
        frozenset(
            {
                "body",
                "end_effectors",
                "base_group",
                "kinematics",
                "sensors",
                "reset_pose",
            }
        ),
        label=f"Robot spec {source_path}",
    )
    try:
        body_ref = spec_cfg["body"]
    except KeyError as exc:
        raise KeyError(f"Robot spec {source_path} must define body.") from exc

    end_effector_refs = spec_cfg.get("end_effectors", [])
    if not isinstance(end_effector_refs, list):
        raise TypeError(f"Robot spec {source_path}.end_effectors must be a list.")

    sensors_cfg = spec_cfg.get("sensors", {})
    if not isinstance(sensors_cfg, Mapping):
        raise TypeError(f"Robot spec {source_path}.sensors must be a mapping.")

    resolved_body = _component_ref(
        body_ref,
        path=f"{source_path}.body",
    )
    expected_body_component_id = f"body.{composition_body_key}"
    if resolved_body["component_id"] != expected_body_component_id:
        raise ValueError(
            f"Robot spec {source_path} is under body directory "
            f"{composition_body_key!r}, but references body component "
            f"{resolved_body['component_id']!r}."
        )

    resolved: dict[str, object] = {
        "body": resolved_body,
        "end_effectors": [],
        "sensors": {},
    }
    if "kinematics" in spec_cfg:
        resolved["kinematics"] = spec_cfg["kinematics"]
    if "reset_pose" in spec_cfg:
        resolved["reset_pose"] = spec_cfg["reset_pose"]
    resolved_end_effectors = resolved["end_effectors"]
    for index, end_effector_ref in enumerate(end_effector_refs):
        resolved_end_effectors.append(
            _component_ref(
                end_effector_ref,
                path=f"{source_path}.end_effectors[{index}]",
            )
        )

    base_group_ref = spec_cfg.get("base_group")
    if base_group_ref is not None:
        resolved["base_group"] = _component_ref(
            base_group_ref,
            path=f"{source_path}.base_group",
        )

    resolved_sensors = resolved["sensors"]
    sensor_component_keys = frozenset({"imu", "color_camera"})
    for sensor_key, sensor_ref in sensors_cfg.items():
        sensor_name = require_non_empty_str(
            sensor_key,
            label=f"Robot spec {source_path}.sensors key",
        )
        if sensor_name not in sensor_component_keys:
            known = ", ".join(sorted(sensor_component_keys))
            raise KeyError(
                f"Robot spec {source_path}.sensors has unsupported key "
                f"{sensor_name!r}. Known: {known}."
            )
        resolved_sensors[sensor_name] = _component_ref(
            sensor_ref,
            path=f"{source_path}.sensors.{sensor_name}",
        )
    return resolved


def resolve_experiment(path: str | Path) -> dict[str, object]:
    source_file = Path(path).expanduser()
    if source_file.suffix != ".yaml":
        raise ValueError(
            f"Experiment path must end with '.yaml', got {str(path)!r}."
        )
    if not source_file.is_absolute():
        source_file = PROJECT_ROOT / source_file
    source_file = source_file.resolve()
    try:
        source_file.relative_to(EXPERIMENT_ROOT.resolve())
    except ValueError as exc:
        raise ValueError(
            "Experiment path must be under configs/experiments, got "
            f"{str(path)!r}."
        ) from exc
    source_path = relative_config_path(source_file)
    expected_policy_id = source_file.relative_to(
        EXPERIMENT_ROOT.resolve()
    ).parts[0]
    experiment_cfg = _normalize_experiment_source(
        read_yaml(source_file),
        source_path=source_path,
        expected_policy_id=expected_policy_id,
    )

    robot_cfg = experiment_cfg["robot"]
    robot_spec = resolve_robot_spec(
        robot_cfg["spec"],
        path=f"{source_path}.robot.spec",
    )
    resolved = {
        "source_path": source_path,
        "environment": experiment_cfg["environment"],
        "runtime": experiment_cfg["runtime"],
        "robot": {
            "spec": robot_spec,
            "drivers": robot_cfg["drivers"],
            "connections": robot_cfg["connections"],
        },
        "policy": experiment_cfg["policy"],
    }
    if "recording" in experiment_cfg:
        resolved["recording"] = experiment_cfg["recording"]
    return resolved
