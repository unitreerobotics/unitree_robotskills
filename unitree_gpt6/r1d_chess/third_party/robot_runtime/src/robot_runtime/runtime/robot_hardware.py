"""Own robot hardware channels and expose schema-keyed state and commands."""

from collections.abc import Mapping
from typing import TYPE_CHECKING

import logging_mp

from robot_runtime.drivers.unitree_sdk2.driver import UnitreeSDK2Driver
from robot_runtime.interfaces.robot.channel import RobotChannel
from robot_runtime.interfaces.robot.component import (
    ComponentCmds,
    ComponentStateValue,
)

if TYPE_CHECKING:  # the camera driver is not bundled here; it needs the external teleimager client
    from robot_runtime.drivers.teleimager.driver import TeleImagerDriver

logger_mp = logging_mp.getLogger(__name__)


class RobotHardware:
    """Own robot drivers, channels, lifecycle, and component routing."""

    def __init__(
        self,
        *,
        channel_configs: dict[str, dict[str, object]],
        drivers: "dict[str, UnitreeSDK2Driver | TeleImagerDriver]",
    ) -> None:
        self.channel_configs = channel_configs
        self.drivers = drivers
        self.channels: dict[str, RobotChannel] = {}
        self.component_channels: dict[str, RobotChannel] = {}
        self.state_routes: dict[str, dict[str, str]] = {}
        self.cmd_routes: dict[str, dict[str, str]] = {}
        self.connected = False

    def build_channel(self, channel_key: str) -> RobotChannel:
        channel_cfg = self.channel_configs[channel_key]
        driver_family = channel_cfg["driver"]["type"].partition(".")[0]
        driver = self.drivers[driver_family]
        channel = driver.build_channel(
            channel_key=channel_key,
            channel_cfg=channel_cfg,
        )
        component_configs = channel_cfg["components"]
        configured_components = {
            component_cfg["component_id"]
            for component_cfg in component_configs
        }
        if channel.components.keys() != configured_components:
            raise RuntimeError(
                f"Channel {channel_key!r} declares components "
                f"{tuple(sorted(channel.components))!r}, but its config "
                f"contains {tuple(sorted(configured_components))!r}."
            )
        for component_id in channel.components:
            existing_channel = self.component_channels.get(component_id)
            if existing_channel is not None and existing_channel is not channel:
                raise RuntimeError(
                    f"Component {component_id!r} belongs to both "
                    f"{type(existing_channel).__name__} and "
                    f"{type(channel).__name__}."
                )
            self.component_channels[component_id] = channel

        for component_cfg in component_configs:
            component_id = component_cfg["component_id"]
            schema_bindings = component_cfg.get("schema_bindings", {})
            for schema_section, schema_keys in schema_bindings.get(
                "state",
                {},
            ).items():
                section_routes = self.state_routes.setdefault(
                    schema_section,
                    {},
                )
                for schema_key in schema_keys:
                    section_routes[schema_key] = component_id

            cmd_bindings = schema_bindings.get("cmd", {})
            if cmd_bindings:
                if component_id not in channel.writable_components:
                    raise RuntimeError(
                        f"{type(channel).__name__} does not contain writable "
                        f"component {component_id!r}."
                    )
                for schema_section, schema_keys in cmd_bindings.items():
                    section_routes = self.cmd_routes.setdefault(
                        schema_section,
                        {},
                    )
                    for schema_key in schema_keys:
                        section_routes[schema_key] = component_id
        return channel

    def connect(self) -> None:
        if self.connected:
            return
        for robot_driver in self.drivers.values():
            robot_driver.initialize()
        try:
            for channel_key in self.channel_configs:
                channel = self.build_channel(channel_key)
                self.channels[channel_key] = channel
                channel.start()
        except Exception:
            self.disconnect()
            raise
        self.connected = True

    def require_connected(self) -> None:
        if not self.connected:
            raise RuntimeError(
                "Robot hardware is not connected. "
                "Call RuntimeServer.start() first."
            )

    def channel(self, channel_key: str) -> RobotChannel:
        self.require_connected()
        try:
            return self.channels[channel_key]
        except KeyError as exc:
            known = ", ".join(sorted(self.channels))
            raise RuntimeError(
                f"Channel key {channel_key!r} is not connected. Known: {known}"
            ) from exc

    def component_channel(self, component_id: str) -> RobotChannel:
        try:
            return self.component_channels[component_id]
        except KeyError as exc:
            known = ", ".join(sorted(self.component_channels))
            raise RuntimeError(
                f"Component {component_id!r} is not connected. Known: {known}"
            ) from exc

    def read_states(
        self,
        schema_section: str,
        schema_keys: tuple[str, ...],
    ) -> dict[str, ComponentStateValue]:
        self.require_connected()
        routes = self.state_routes[schema_section]
        states_by_channel: dict[
            RobotChannel,
            Mapping[str, Mapping[str, ComponentStateValue]],
        ] = {}
        states_by_schema_key: dict[str, ComponentStateValue] = {}
        for schema_key in schema_keys:
            component_id = routes[schema_key]
            channel = self.component_channel(component_id)
            if channel not in states_by_channel:
                states_by_channel[channel] = channel.read_component_states()
            states_by_schema_key[schema_key] = states_by_channel[channel][
                component_id
            ][schema_key]
        return states_by_schema_key

    def cmd_component_id(
        self,
        schema_section: str,
        schema_key: str,
    ) -> str:
        self.require_connected()
        cmd_routes = self.cmd_routes.get(schema_section, {})
        try:
            return cmd_routes[schema_key]
        except KeyError as exc:
            known = ", ".join(sorted(cmd_routes))
            raise RuntimeError(
                f"{schema_section} schema key {schema_key!r} has no command "
                f"component. "
                f"Known: {known}"
            ) from exc

    def write_component_cmds(
        self,
        component_cmds: Mapping[str, ComponentCmds],
    ) -> None:
        self.require_connected()
        cmds_by_channel: dict[RobotChannel, dict[str, ComponentCmds]] = {}
        for component_id, component_cmds_for_component in component_cmds.items():
            channel = self.component_channel(component_id)
            cmds_by_channel.setdefault(channel, {})[component_id] = (
                component_cmds_for_component
            )

        for channel, channel_cmds in cmds_by_channel.items():
            channel.write_component_cmds(channel_cmds)

    def color_camera_channel(self) -> RobotChannel | None:
        self.require_connected()
        color_camera_routes = self.state_routes.get("color_camera", {})
        component_id = next(iter(color_camera_routes.values()), None)
        if component_id is None:
            return None
        return self.component_channel(component_id)

    def disconnect(self) -> None:
        stop_error = None
        try:
            for channel_key, channel in reversed(self.channels.items()):
                try:
                    logger_mp.info("Stopping robot channel: %s", channel_key)
                    channel.stop()
                    logger_mp.info("Stopped robot channel: %s", channel_key)
                except Exception as exc:
                    logger_mp.exception(
                        "Failed to stop robot channel: %s",
                        channel_key,
                    )
                    if stop_error is None:
                        stop_error = exc
        finally:
            self.channels.clear()
            self.component_channels.clear()
            self.state_routes.clear()
            self.cmd_routes.clear()
            self.connected = False
        if stop_error is not None:
            raise stop_error
