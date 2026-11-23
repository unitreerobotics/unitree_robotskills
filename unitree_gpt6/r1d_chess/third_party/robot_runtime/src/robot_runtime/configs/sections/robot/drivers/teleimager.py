"""Validate TeleImager robot connection and component drivers."""

from robot_runtime.configs.common import (
    reject_unknown_fields,
    require_non_empty_str,
)


def validate_teleimager_driver_config(cfg) -> dict[str, object]:
    if not isinstance(cfg, dict):
        raise TypeError("robot.connections.teleimager must be a mapping.")

    reject_unknown_fields(
        cfg,
        frozenset({"host", "request_port"}),
        label="robot.connections.teleimager",
    )

    try:
        host = cfg["host"]
    except KeyError as exc:
        raise KeyError("robot.connections.teleimager.host must be configured.") from exc
    request_port = cfg.get("request_port", 60000)

    host = require_non_empty_str(
        host,
        label="robot.connections.teleimager.host",
    )
    if type(request_port) is not int:
        raise TypeError(
            "robot.connections.teleimager.request_port must be an integer."
        )
    if not 0 < request_port <= 65535:
        raise ValueError(
            "robot.connections.teleimager.request_port must be in range 1..65535."
        )

    normalized_cfg = dict(cfg)
    normalized_cfg["host"] = host
    normalized_cfg["request_port"] = request_port
    return normalized_cfg


def validate_teleimager_robot_driver(
    channel_key: str,
    driver_config: dict,
) -> None:
    driver_type = driver_config["type"]
    if driver_type != "teleimager.color_camera":
        raise KeyError(
            f"Unsupported TeleImager robot driver for channel "
            f"{channel_key!r}: {driver_type!r}."
        )
    reject_unknown_fields(
        driver_config,
        frozenset({"type", "request_bgr", "stream_bindings"}),
        label=f"Channel {channel_key!r} driver",
    )
    if type(driver_config.get("request_bgr")) is not bool:
        raise TypeError(
            f"Channel {channel_key!r} driver.request_bgr must be a bool."
        )
