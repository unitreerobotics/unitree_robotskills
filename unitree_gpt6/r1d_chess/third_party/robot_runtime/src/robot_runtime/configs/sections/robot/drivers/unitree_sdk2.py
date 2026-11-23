"""Validate Unitree SDK2 robot connection and component drivers."""

from robot_runtime.configs.common import (
    reject_unknown_fields,
    require_non_empty_str,
    require_choice,
    require_mapping,
)

UNITREE_END_EFFECTOR_DRIVERS = frozenset(
    {
        "unitree_sdk2.brainco_revo2",
        "unitree_sdk2.dex1_1",
        "unitree_sdk2.dex2_5",
        "unitree_sdk2.dex3_1",
        "unitree_sdk2.dex5_1",
        "unitree_sdk2.inspire_rh56dftp",
    }
)
UNITREE_BODY_DRIVERS = frozenset(
    {
        "unitree_sdk2.low_level",
        "unitree_sdk2.whole_body_sdk",
        "unitree_sdk2.pose_82d",
        "unitree_sdk2.hybrid_wbc",
    }
)
UNITREE_BASE_GROUP_DRIVERS = frozenset(
    {"unitree_sdk2.base_lift", "unitree_sdk2.cmd_vel"}
)

UNITREE_ROBOT_DRIVER_FIELDS = {
    "unitree_sdk2.hybrid_wbc": frozenset(
        {"type", "control_fsm_id", "restore_fsm_id"}
    ),
    "unitree_sdk2.low_level": frozenset(
        {
            "type",
            "cmd_topic",
            "control_weight_motor_index",
            "restore_motion_mode",
            "motor_topics",
        }
    ),
    "unitree_sdk2.whole_body_sdk": frozenset(
        {
            "type",
            "cmd_topic",
            "control_motion_mode",
            "restore_motion_mode",
        }
    ),
    "unitree_sdk2.pose_82d": frozenset(
        {
            "type",
            "cmd_topic",
            "control_fsm_id",
            "restore_fsm_id",
            "include_frame_id",
        }
    ),
    "unitree_sdk2.base_lift": frozenset(
        {
            "type",
            "height_cmd_topic",
            "height_state_topic",
            "move_cmd_topic",
            "move_state_topic",
        }
    ),
    "unitree_sdk2.cmd_vel": frozenset(
        {
            "type", "cmd_topic", "move_state_topic",
            "height_state_topic", "height_alarm_topic",
        }
    ),
    "unitree_sdk2.brainco_revo2": frozenset(
        {"type", "state_topic", "cmd_topic"}
    ),
    "unitree_sdk2.dex1_1": frozenset(
        {"type", "wiring", "state_topic", "cmd_topic"}
    ),
    "unitree_sdk2.dex2_5": frozenset({"type", "wiring"}),
    "unitree_sdk2.dex3_1": frozenset(
        {"type", "state_topic", "cmd_topic"}
    ),
    "unitree_sdk2.dex5_1": frozenset(
        {"type", "state_topic", "cmd_topic"}
    ),
    "unitree_sdk2.inspire_rh56dftp": frozenset(
        {"type", "state_topic", "cmd_topic"}
    ),
}


def validate_unitree_driver_config(cfg) -> dict[str, object]:
    if not isinstance(cfg, dict):
        raise TypeError("robot.connections.unitree_sdk2 must be a mapping.")

    unitree_cfg = dict(cfg)
    reject_unknown_fields(
        unitree_cfg,
        frozenset({"dds_domain_id", "network_interface"}),
        label="robot.connections.unitree_sdk2",
    )
    dds_domain_id = unitree_cfg.get("dds_domain_id", 0)
    if type(dds_domain_id) is not int:
        raise TypeError("robot.connections.unitree_sdk2.dds_domain_id must be an int.")
    if dds_domain_id < 0:
        raise ValueError(
            "robot.connections.unitree_sdk2.dds_domain_id must be non-negative."
        )

    network_interface = unitree_cfg.get("network_interface")
    if network_interface is not None:
        if not isinstance(network_interface, str) or not network_interface:
            raise ValueError(
                "robot.connections.unitree_sdk2.network_interface must be a non-empty "
                "string or null."
            )

    unitree_cfg["dds_domain_id"] = dds_domain_id
    unitree_cfg["network_interface"] = network_interface
    return unitree_cfg


def validate_unitree_robot_driver(
    channel_key: str,
    driver_config: dict,
) -> None:
    driver_type = driver_config["type"]
    try:
        allowed_fields = UNITREE_ROBOT_DRIVER_FIELDS[driver_type]
    except KeyError as exc:
        raise KeyError(
            f"Unsupported Unitree robot driver for channel "
            f"{channel_key!r}: {driver_type!r}."
        ) from exc
    reject_unknown_fields(
        driver_config,
        allowed_fields,
        label=f"Channel {channel_key!r} driver",
    )

    if driver_type in UNITREE_BODY_DRIVERS:
        if driver_type in {"unitree_sdk2.pose_82d", "unitree_sdk2.hybrid_wbc"}:
            for field_name in ("control_fsm_id", "restore_fsm_id"):
                if field_name not in driver_config:
                    raise KeyError(
                        f"Channel {channel_key!r} driver requires {field_name}."
                    )
                if type(driver_config[field_name]) is not int:
                    raise TypeError(
                        f"Channel {channel_key!r} driver.{field_name} must be "
                        "an int."
                    )
            if driver_type == "unitree_sdk2.hybrid_wbc" and (
                driver_config["control_fsm_id"], driver_config["restore_fsm_id"]
            ) != (505, 801):
                raise ValueError("hybrid_wbc requires FSM 505 and restore FSM 801.")
            if "cmd_topic" in driver_config:
                raise ValueError(
                    f"Channel {channel_key!r} pose_82d must not define "
                    "driver.cmd_topic; the protocol uses "
                    "'rt/fsm/teleop/cmd'."
                )
            if "include_frame_id" in driver_config and not isinstance(
                driver_config["include_frame_id"],
                bool,
            ):
                raise TypeError(
                    f"Channel {channel_key!r} driver.include_frame_id must "
                    "be a boolean."
                )
            return
        if driver_type == "unitree_sdk2.low_level":
            require_non_empty_str(
                driver_config.get("cmd_topic"),
                label=f"Channel {channel_key!r} driver.cmd_topic",
            )
            motor_topics = require_mapping(
                driver_config.get("motor_topics", {}),
                label=f"Channel {channel_key!r} driver.motor_topics",
            )
            for joint_name, topics in motor_topics.items():
                require_non_empty_str(joint_name, label="motor_topics joint name")
                label = f"Channel {channel_key!r} motor_topics.{joint_name}"
                topics = require_mapping(topics, label=label)
                reject_unknown_fields(
                    topics, frozenset({"state_topic", "cmd_topic"}), label=label,
                )
                for field in ("state_topic", "cmd_topic"):
                    require_non_empty_str(topics.get(field), label=f"{label}.{field}")
            if "restore_motion_mode" in driver_config:
                require_non_empty_str(
                    driver_config["restore_motion_mode"],
                    label=(
                        f"Channel {channel_key!r} "
                        "driver.restore_motion_mode"
                    ),
                )
                if driver_config["cmd_topic"] != "rt/lowcmd":
                    raise ValueError(
                        f"Channel {channel_key!r} low_level "
                        "driver.restore_motion_mode requires "
                        "driver.cmd_topic='rt/lowcmd'."
                    )
            if "control_weight_motor_index" in driver_config:
                control_weight_motor_index = driver_config[
                    "control_weight_motor_index"
                ]
                if type(control_weight_motor_index) is not int:
                    raise TypeError(
                        f"Channel {channel_key!r} "
                        "driver.control_weight_motor_index must be an int."
                    )
                if control_weight_motor_index < 0:
                    raise ValueError(
                        f"Channel {channel_key!r} "
                        "driver.control_weight_motor_index must be "
                        "non-negative."
                    )
        if driver_type == "unitree_sdk2.whole_body_sdk":
            if "cmd_topic" in driver_config:
                raise ValueError(
                    f"Channel {channel_key!r} whole_body_sdk must not define "
                    "driver.cmd_topic; the protocol uses "
                    "'rt/whole_body_sdk'."
                )
            for field_name in (
                "control_motion_mode",
                "restore_motion_mode",
            ):
                require_non_empty_str(
                    driver_config.get(field_name),
                    label=f"Channel {channel_key!r} driver.{field_name}",
                )
        return

    if driver_type == "unitree_sdk2.base_lift":
        for field_name in (
            "height_cmd_topic",
            "height_state_topic",
            "move_cmd_topic",
            "move_state_topic",
        ):
            require_non_empty_str(
                driver_config.get(field_name),
                label=f"Channel {channel_key!r} driver.{field_name}",
            )
        return

    if driver_type == "unitree_sdk2.cmd_vel":
        for field_name in (
            "cmd_topic",
            "move_state_topic",
            "height_state_topic",
            "height_alarm_topic",
        ):
            require_non_empty_str(
                driver_config.get(field_name),
                label=f"Channel {channel_key!r} driver.{field_name}",
            )
        return

    if driver_type in UNITREE_END_EFFECTOR_DRIVERS:
        if driver_type == "unitree_sdk2.dex2_5":
            raise ValueError("dex2_5 requires internal wiring in the body channel.")
        if driver_type == "unitree_sdk2.dex1_1":
            require_choice(
                driver_config.get("wiring"),
                choices={"external"},
                label=f"Channel {channel_key!r} driver.wiring",
            )
        require_non_empty_str(
            driver_config.get("state_topic"),
            label=f"Channel {channel_key!r} driver.state_topic",
        )
        require_non_empty_str(
            driver_config.get("cmd_topic"),
            label=f"Channel {channel_key!r} driver.cmd_topic",
        )
        return

    raise AssertionError(f"Unhandled Unitree robot driver {driver_type!r}.")
