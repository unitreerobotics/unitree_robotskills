"""Resolve robot driver declarations into canonical channels."""

from collections.abc import Collection

from robot_runtime.configs.common import (
    add_unique,
    reject_unknown_fields,
    require_mapping,
    require_non_empty_str,
    require_positive_float,
)
from robot_runtime.configs.sections.robot.components.base_group import BaseGroupSpec
from robot_runtime.configs.sections.robot.components.joint_group import JointGroupSpec
from robot_runtime.configs.sections.robot.components.sensor import SensorSpec
from robot_runtime.configs.sections.robot.composition import RobotSpec
from robot_runtime.configs.sections.robot.drivers.teleimager import (
    validate_teleimager_driver_config,
    validate_teleimager_robot_driver,
)
from robot_runtime.configs.sections.robot.drivers.unitree_sdk2 import (
    UNITREE_BASE_GROUP_DRIVERS,
    UNITREE_END_EFFECTOR_DRIVERS,
    validate_unitree_driver_config,
    validate_unitree_robot_driver,
)
from robot_runtime.configs.sections.runtime import (
    RuntimeControlMode,
    resolve_motion_schema_keys,
)


def build_robot_channel(
    channel_key: str,
    driver_config,
    *,
    components: tuple[dict[str, object], ...],
    label: str,
) -> dict[str, object]:
    """Build one validated robot Channel configuration."""
    driver_config = require_mapping(driver_config, label=label)
    driver_type = require_non_empty_str(
        driver_config.get("type"),
        label=f"{label}.type",
    )

    channel_config: dict[str, object] = {
        "driver": driver_config,
        "components": components,
    }
    start_timeout = driver_config.pop("start_timeout", None)
    if start_timeout is not None:
        channel_config["start_timeout"] = require_positive_float(
            start_timeout,
            label=f"{label}.start_timeout",
        )

    if driver_type.startswith("unitree_sdk2."):
        validate_unitree_robot_driver(channel_key, driver_config)
    elif driver_type == "teleimager.color_camera":
        validate_teleimager_robot_driver(channel_key, driver_config)
    else:
        raise KeyError(
            f"Unsupported robot driver for channel {channel_key!r}: "
            f"{driver_type!r}."
        )
    return channel_config


def index_driver_configs(
    driver_declarations,
    *,
    component_ids: Collection[str],
    label: str,
) -> dict[str, dict[str, object]]:
    """Normalize Driver declarations and index configs by component ID."""
    driver_configs_by_component_id = {}
    for index, driver_declaration in enumerate(driver_declarations):
        driver_label = f"{label}[{index}]"
        driver_config = require_mapping(driver_declaration, label=driver_label)
        component_id = require_non_empty_str(
            driver_config.pop("component_id", None),
            label=f"{driver_label}.component_id",
        )
        if component_id not in component_ids:
            raise KeyError(
                f"{driver_label}.component_id={component_id!r} is not declared by "
                "the robot specification."
            )
        add_unique(
            driver_configs_by_component_id,
            component_id,
            driver_config,
            label=label,
        )
    return driver_configs_by_component_id


def resolve_body_components(
    body_driver_config: dict[str, object],
    *,
    robot_spec: RobotSpec,
    control_mode: RuntimeControlMode,
    label: str,
) -> list[dict[str, object]]:
    """Validate the body Driver and build body-owned component bindings."""
    body_component = robot_spec.joint_group_specs[0]
    body_component_id = body_component.component_id
    base_group_component = robot_spec.base_group_spec

    body_driver_type = body_driver_config.get("type")
    expected_body_driver_type = {
        RuntimeControlMode.DUAL_ARM: "unitree_sdk2.low_level",
        RuntimeControlMode.UPPER_BODY: "unitree_sdk2.low_level",
        RuntimeControlMode.MOBILE_WBC: "unitree_sdk2.low_level",
        RuntimeControlMode.DECOUPLED_WBC: "unitree_sdk2.whole_body_sdk",
        RuntimeControlMode.HUMANOID_WBC: "unitree_sdk2.pose_82d",
        RuntimeControlMode.HYBRID_WBC: "unitree_sdk2.hybrid_wbc",
    }[control_mode]
    if body_driver_type != expected_body_driver_type:
        raise ValueError(
            f"{label}.type must be {expected_body_driver_type!r} for "
            "runtime.control_mode="
            f"{control_mode.value!r}, got {body_driver_type!r}."
        )
    body_state_schema_keys = body_component.schema_keys
    body_cmd_schema_bindings = resolve_motion_schema_keys(
        control_mode, body_state_schema_keys,
    )
    if (
        control_mode is RuntimeControlMode.MOBILE_WBC
        and "head" in body_state_schema_keys
    ):
        body_cmd_schema_bindings["joint_group"] = (
            *body_cmd_schema_bindings["joint_group"],
            "head",
        )
    if (
        control_mode in {RuntimeControlMode.DUAL_ARM, RuntimeControlMode.UPPER_BODY}
        and body_driver_config.get("cmd_topic") == "rt/lowcmd"
    ):
        hold_schema_keys = ("waist", "head", "left_leg", "right_leg")
        body_cmd_schema_bindings["joint_group"] += tuple(
            key for key in hold_schema_keys
            if key in body_state_schema_keys
            and key not in body_cmd_schema_bindings["joint_group"]
        )
    body_schema_bindings: dict[str, object] = {
        "state": {"joint_group": body_state_schema_keys},
        "cmd": body_cmd_schema_bindings,
    }
    required_joint_group_keys = body_cmd_schema_bindings.get("joint_group")
    if required_joint_group_keys is not None:
        missing_schema_keys = tuple(
            schema_key
            for schema_key in required_joint_group_keys
            if schema_key not in body_state_schema_keys
        )
        if missing_schema_keys:
            raise KeyError(
                f"{label} body component {body_component_id!r} does not "
                f"declare motion schema keys {missing_schema_keys!r}."
            )
        body_cmd_schema_bindings["joint_group"] = tuple(
            schema_key
            for schema_key in body_state_schema_keys
            if schema_key in required_joint_group_keys
        )

    if control_mode in {RuntimeControlMode.DECOUPLED_WBC, RuntimeControlMode.HYBRID_WBC}:
        if base_group_component is None:
            raise KeyError(f"{label} {control_mode.value} requires robot base_group.")

    body_components = [
        {
            "component_id": body_component_id,
            "schema_bindings": body_schema_bindings,
        }
    ]
    if control_mode in {RuntimeControlMode.DECOUPLED_WBC, RuntimeControlMode.HYBRID_WBC}:
        body_components.append(
            {
                "component_id": base_group_component.component_id,
                "interface_type": "g1_edu_pseudo_base",
                "schema_bindings": {
                    "state": {
                        "base_group": base_group_component.schema_keys,
                    } if control_mode is RuntimeControlMode.DECOUPLED_WBC else {},
                    "cmd": {
                        "base_group": base_group_component.schema_keys,
                    },
                },
            }
        )
    imu_spec = robot_spec.sensor_spec("imu")
    if imu_spec is not None:
        body_components.append(
            {
                "component_id": imu_spec.component_id,
                "interface_type": "imu",
                "schema_bindings": {
                    "state": {"imu": imu_spec.schema_keys}
                },
            }
        )
    return body_components


def resolve_internal_dex1_component(
    driver_config: dict[str, object],
    *,
    component_spec: JointGroupSpec,
    include_command_bindings: bool,
    label: str,
) -> dict[str, object]:
    """Build one internally wired Dex component in the primary body Channel."""
    internal_config = dict(driver_config)
    interface_type = internal_config.pop("type").removeprefix("unitree_sdk2.")
    reject_unknown_fields(
        internal_config,
        frozenset({"wiring", "shared_motor_indices"}),
        label=label,
    )
    shared_motor_indices = internal_config.get("shared_motor_indices")
    if shared_motor_indices is None:
        raise KeyError(
            f"{label}.shared_motor_indices must be configured for internal wiring."
        )
    if not isinstance(shared_motor_indices, list | tuple):
        raise TypeError(
            f"{label}.shared_motor_indices must be a list or tuple."
        )
    if not shared_motor_indices:
        raise ValueError(f"{label}.shared_motor_indices must not be empty.")
    if any(type(index) is not int for index in shared_motor_indices):
        raise TypeError(f"{label}.shared_motor_indices must contain ints.")
    internal_config["shared_motor_indices"] = tuple(shared_motor_indices)

    schema_bindings: dict[str, object] = {
        "state": {"joint_group": component_spec.schema_keys},
    }
    if include_command_bindings:
        schema_bindings["cmd"] = {
            "joint_group": component_spec.schema_keys,
        }
    return {
        "component_id": component_spec.component_id,
        "interface_type": interface_type,
        "schema_bindings": schema_bindings,
        **internal_config,
    }


def build_standalone_robot_channel(
    component_spec: JointGroupSpec | BaseGroupSpec | SensorSpec,
    driver_config: dict[str, object],
    *,
    label: str,
) -> tuple[str, dict[str, object]]:
    """Build a Channel for one component not embedded in the body Channel."""
    component_id = component_spec.component_id
    driver_config = dict(driver_config)

    if isinstance(component_spec, JointGroupSpec):
        schema_keys = component_spec.schema_keys
        if "side" in driver_config:
            raise KeyError(
                f"{label}.side is redundant; side is derived from "
                f"component_id {component_id!r}."
            )
        driver_type = driver_config.get("type")
        if driver_type not in UNITREE_END_EFFECTOR_DRIVERS:
            raise ValueError(
                f"{label}.type={driver_type!r} is not supported for "
                f"end-effector component {component_id!r}."
            )
        channel_key = (
            f"joint_group.{schema_keys[0]}"
            if len(schema_keys) == 1
            else component_id
        )
        schema_bindings = {
            "state": {"joint_group": schema_keys},
            "cmd": {"joint_group": schema_keys},
        }
    elif isinstance(component_spec, BaseGroupSpec):
        driver_type = driver_config.get("type")
        if driver_type not in UNITREE_BASE_GROUP_DRIVERS:
            raise ValueError(
                f"{label}.type={driver_type!r} is not supported for "
                f"base-group component {component_id!r}."
            )
        schema_keys = component_spec.schema_keys
        channel_key = f"base_group.{schema_keys[0]}"
        schema_bindings = {
            "state": {"base_group": schema_keys},
            "cmd": {"base_group": schema_keys},
        }
    elif isinstance(component_spec, SensorSpec):
        sensor_section = component_spec.sensor_kind
        driver_type = driver_config.get("type")
        if (
            sensor_section != "color_camera"
            or driver_type != "teleimager.color_camera"
        ):
            raise ValueError(
                f"{label}.type={driver_type!r} is not supported for "
                f"sensor section {sensor_section!r}."
            )
        streams = driver_config.pop("streams", None)
        if streams is None:
            raise KeyError(f"{label}.streams must be configured.")
        streams = require_mapping(streams, label=f"{label}.streams")
        if not streams:
            raise TypeError(f"{label}.streams must be a non-empty mapping.")
        stream_bindings = {
            require_non_empty_str(
                schema_key,
                label=f"{label}.streams key",
            ): require_non_empty_str(
                stream_key,
                label=f"{label}.streams.{schema_key}",
            )
            for schema_key, stream_key in streams.items()
        }
        unknown_schema_keys = tuple(
            sorted(set(stream_bindings) - set(component_spec.schema_keys))
        )
        if unknown_schema_keys:
            raise KeyError(
                f"{label}.streams configures schema keys "
                f"{unknown_schema_keys!r} not declared by component "
                f"{component_id!r}."
            )
        driver_config["stream_bindings"] = stream_bindings
        channel_key = f"sensor.{sensor_section}"
        schema_bindings = {
            "state": {sensor_section: tuple(stream_bindings)},
        }
    else:
        raise TypeError(
            f"{label} targets unsupported component spec "
            f"{type(component_spec).__name__}."
        )

    channel_config = build_robot_channel(
        channel_key,
        driver_config,
        components=(
            {
                "component_id": component_id,
                "schema_bindings": schema_bindings,
            },
        ),
        label=label,
    )
    return channel_key, channel_config


def _resolve_robot_channels(
    driver_declarations,
    *,
    robot_spec: RobotSpec,
    control_mode: RuntimeControlMode,
    label: str,
) -> dict[str, dict[str, object]]:
    body_component_id = robot_spec.joint_group_specs[0].component_id
    component_specs: list[
        JointGroupSpec | BaseGroupSpec | SensorSpec
    ] = list(robot_spec.joint_group_specs)
    if robot_spec.base_group_spec is not None:
        component_specs.append(robot_spec.base_group_spec)
    component_specs.extend(robot_spec.sensor_specs)
    component_specs_by_id = {
        component_spec.component_id: component_spec
        for component_spec in component_specs
    }

    # Index validated Driver configs by their composed component IDs.
    driver_configs_by_component_id = index_driver_configs(
        driver_declarations,
        component_ids=component_specs_by_id.keys(),
        label=label,
    )
    try:
        body_driver_config = driver_configs_by_component_id[
            body_component_id
        ]
    except KeyError as exc:
        raise ValueError(
            f"{label} must configure exactly one driver for body component "
            f"{body_component_id!r}."
        ) from exc
    body_driver_label = f"{label}[component_id={body_component_id!r}]"

    # Start the body Channel with its body and optional IMU components.
    body_components = resolve_body_components(
        body_driver_config,
        robot_spec=robot_spec,
        control_mode=control_mode,
        label=body_driver_label,
    )

    # Internal Dex hands share the body Channel's HG motor messages.
    standalone_channels = {}
    for component_id, driver_config in driver_configs_by_component_id.items():
        if component_id == body_component_id:
            continue
        component_spec = component_specs_by_id[component_id]
        driver_label = f"{label}[component_id={component_id!r}]"
        if (
            isinstance(component_spec, JointGroupSpec)
            and driver_config.get("type") in {
                "unitree_sdk2.dex1_1", "unitree_sdk2.dex2_5",
            }
            and driver_config.get("wiring") == "internal"
        ):
            body_components.append(
                resolve_internal_dex1_component(
                    driver_config,
                    component_spec=component_spec,
                    include_command_bindings=(
                        body_driver_config.get("type")
                        != "unitree_sdk2.whole_body_sdk"
                    ),
                    label=driver_label,
                )
            )
            continue

        channel_key, channel_config = build_standalone_robot_channel(
            component_spec,
            driver_config,
            label=driver_label,
        )
        standalone_channels[channel_key] = channel_config

    body_channel = build_robot_channel(
        "body",
        body_driver_config,
        components=tuple(body_components),
        label=body_driver_label,
    )
    return {"body": body_channel, **standalone_channels}


def resolve_robot_drivers(
    driver_declarations,
    connection_configs,
    *,
    robot_spec: RobotSpec,
    control_mode: RuntimeControlMode,
    label: str,
) -> tuple[dict[str, object], dict[str, dict[str, object]]]:
    channels = _resolve_robot_channels(
        driver_declarations,
        robot_spec=robot_spec,
        control_mode=control_mode,
        label=label,
    )
    driver_families = {
        channel_cfg["driver"]["type"].partition(".")[0]
        for channel_cfg in channels.values()
    }
    unknown_connections = tuple(
        sorted(set(connection_configs) - set(driver_families))
    )
    if unknown_connections:
        raise KeyError(
            "robot.connections configures unused driver families "
            f"{unknown_connections!r}."
        )

    normalized_connections = {}
    if "unitree_sdk2" in driver_families:
        normalized_connections["unitree_sdk2"] = validate_unitree_driver_config(
            connection_configs.get("unitree_sdk2", {}),
        )
    if "teleimager" in driver_families:
        normalized_connections["teleimager"] = validate_teleimager_driver_config(
            connection_configs.get("teleimager", {}),
        )
    return normalized_connections, channels
