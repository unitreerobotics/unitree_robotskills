"""Pose-82D command loop with timeout hold behavior."""

from __future__ import annotations

from collections.abc import Callable, Mapping

from robot_runtime.interfaces.robot.component import ComponentCmds
from robot_runtime.runtime.cmd_loop.base import RobotCmdLoop
from robot_runtime.runtime.safety.thermal import ThermalSafety


class Pose82dCmdLoop(RobotCmdLoop):
    """Publish Pose-82D body targets with optional joint-group commands."""

    def __init__(
        self,
        *,
        write_component_cmds: Callable[[Mapping[str, ComponentCmds]], None],
        frequency: float,
        cmd_timeout: float,
        qpos_limits: Mapping[tuple[str, str], tuple[tuple[float, float], ...]],
        thermal_safety: ThermalSafety | None = None,
        enter_control: Callable[[], None] | None = None,
        exit_control: Callable[[], None] | None = None,
    ) -> None:
        self.current_component_cmds: Mapping[str, ComponentCmds] | None = None

        super().__init__(
            write_component_cmds=write_component_cmds,
            frequency=frequency,
            cmd_timeout=cmd_timeout,
            qpos_limits=qpos_limits,
            thermal_safety=thermal_safety,
            enter_control=enter_control,
            exit_control=exit_control,
        )

    def build_component_cmds(
        self,
        component_cmds: Mapping[str, ComponentCmds],
        *,
        timed_out: bool,
    ) -> Mapping[str, ComponentCmds] | None:
        if not timed_out:
            self.current_component_cmds = component_cmds
        return self.current_component_cmds

    def clear_cmd_state(self) -> None:
        super().clear_cmd_state()
        self.current_component_cmds = None
