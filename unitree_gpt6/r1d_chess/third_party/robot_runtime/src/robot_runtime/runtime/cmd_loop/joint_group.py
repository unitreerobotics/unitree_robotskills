"""Joint command loop with rate limiting and timeout hold behavior."""

from __future__ import annotations

from collections.abc import Callable, Mapping
import threading
import time

import logging_mp
import numpy as np

from robot_runtime.interfaces.robot.cmd.base_group import BaseGroupCmd
from robot_runtime.interfaces.robot.cmd.joint_group import JointGroupCmd
from robot_runtime.interfaces.robot.component import (
    ComponentCmds,
    ComponentCmdValue,
)
from robot_runtime.runtime.cmd_loop.base import RobotCmdLoop
from robot_runtime.runtime.safety.thermal import ThermalSafety


logger_mp = logging_mp.getLogger(__name__)


class JointCmdLoop(RobotCmdLoop):
    """Publish joint targets with optional base-group commands."""

    def __init__(
        self,
        *,
        hold_component_cmds: Mapping[str, ComponentCmds],
        read_joint_group_qpos: Callable[[str], np.ndarray],
        joint_group_components: Mapping[str, str],
        write_component_cmds: Callable[[Mapping[str, ComponentCmds]], None],
        frequency: float,
        cmd_joint_group_order: tuple[str, ...],
        velocity_limit: float,
        cmd_timeout: float,
        qpos_limits: Mapping[tuple[str, str], tuple[tuple[float, float], ...]],
        body_height_rate_limit: float | None = None,
        thermal_safety: ThermalSafety | None = None,
        base_group_component: str | None = None,
        base_group_schema_key: str | None = None,
        zero_base_group_on_stop: bool = False,
        enter_control: Callable[[], None] | None = None,
        exit_control: Callable[[], None] | None = None,
        fixed_joint_group_schema_keys: tuple[str, ...] = (),
        compute_gravity_qtau: Callable[[], Mapping[str, np.ndarray]] | None = None,
        torque_rate_limits: Mapping[str, float] | None = None,
    ) -> None:
        self.read_joint_group_qpos = read_joint_group_qpos
        self.joint_group_components = dict(joint_group_components)
        self.hold_component_cmds = hold_component_cmds
        self.fixed_joint_group_schema_keys = fixed_joint_group_schema_keys
        self.compute_gravity_qtau = compute_gravity_qtau
        self.torque_rate_limits = dict(torque_rate_limits or {})
        self.cmd_joint_group_order = cmd_joint_group_order
        self.cmd_dof = sum(
            self.joint_group_cmd(
                self.hold_component_cmds,
                schema_key,
            ).qpos.size
            for schema_key in cmd_joint_group_order
        )
        self.velocity_limit = velocity_limit
        self.body_height_rate_limit = body_height_rate_limit
        self.base_group_component = base_group_component
        self.base_group_schema_key = base_group_schema_key
        self.zero_base_group_on_stop = zero_base_group_on_stop
        self.current_cmd_qpos: np.ndarray | None = None
        self.velocity_limit_active = False
        self.last_velocity_limit_warning_time = 0.0
        self.publish_lock = threading.Lock()

        super().__init__(
            write_component_cmds=write_component_cmds,
            frequency=frequency,
            cmd_timeout=cmd_timeout,
            qpos_limits=qpos_limits,
            thermal_safety=thermal_safety,
            enter_control=enter_control,
            exit_control=exit_control,
        )

    def publish_once(self, timestamp: float) -> None:
        # Serialize the exit hold with in-flight command publication.
        with self.publish_lock:
            try:
                super().publish_once(timestamp)
            except Exception as exc:
                self.cmd_error = exc
                raise

    def hold_position(self) -> None:
        """Hold body targets/feedforward; clear base speed and torque-only hands."""
        with self.publish_lock:
            if self.cmd_error is not None:
                raise RuntimeError(
                    "Cannot hold a failed command loop."
                ) from self.cmd_error
            if self.last_component_cmds is None:
                raise RuntimeError("Cannot hold before the first command output.")
            held = {
                component_id: dict(cmds)
                for component_id, cmds in self.last_component_cmds.items()
            }
            if self.base_group_component is not None:
                base_cmd = held[self.base_group_component][
                    self.base_group_schema_key
                ]
                held[self.base_group_component][self.base_group_schema_key] = (
                    self.zero_base_group_cmd(base_cmd)
                )
            self.update_cmd(self.zero_torque_cmds(held), timeout_enabled=False)
            super().publish_once(time.monotonic())

    def startup_cmd(self) -> Mapping[str, ComponentCmds]:
        startup_cmds = {
            component_id: dict(cmds)
            for component_id, cmds in self.hold_component_cmds.items()
        }
        for schema_key, component_id in self.joint_group_components.items():
            source_cmd = self.joint_group_cmd(
                self.hold_component_cmds,
                schema_key,
            )
            if source_cmd.qpos is None:
                startup_cmds[component_id][schema_key] = JointGroupCmd(
                    num_joints=source_cmd.qtau.size,
                    qtau=np.zeros_like(source_cmd.qtau),
                )
                continue
            qpos = (
                source_cmd.qpos.copy()
                if schema_key in self.fixed_joint_group_schema_keys
                else self.read_joint_group_qpos(schema_key)
            )
            startup_cmds.setdefault(component_id, {})[
                schema_key
            ] = JointGroupCmd(
                num_joints=qpos.size,
                qpos=qpos,
                qtau=source_cmd.qtau,
            )

        if self.base_group_component is not None:
            source_base_group_cmd = self.hold_component_cmds[
                self.base_group_component
            ][self.base_group_schema_key]
            if not isinstance(source_base_group_cmd, BaseGroupCmd):
                raise TypeError("Hold command has invalid base_group command.")
            startup_cmds.setdefault(self.base_group_component, {})[
                self.base_group_schema_key
            ] = self.zero_base_group_cmd(source_base_group_cmd)

        self.current_cmd_qpos = self.cmd_qpos(startup_cmds)
        return startup_cmds

    def update_cmd(
        self,
        component_cmds: Mapping[str, ComponentCmds],
        *,
        timeout_enabled: bool = True,
    ) -> None:
        if self.fixed_joint_group_schema_keys:
            component_cmds = {
                component_id: dict(cmds)
                for component_id, cmds in component_cmds.items()
            }
            for schema_key in self.fixed_joint_group_schema_keys:
                component_id = self.joint_group_components[schema_key]
                component_cmds.setdefault(component_id, {})[schema_key] = (
                    self.hold_component_cmds[component_id][schema_key]
                )
        # Fixed targets must pass through the same thermal protection as arms.
        super().update_cmd(component_cmds, timeout_enabled=timeout_enabled)

    def build_component_cmds(
        self,
        component_cmds: Mapping[str, ComponentCmds],
        *,
        timed_out: bool,
    ) -> Mapping[str, ComponentCmds]:
        if self.current_cmd_qpos is None:
            self.current_cmd_qpos = self.current_qpos()

        target_qpos = (
            self.current_cmd_qpos
            if timed_out
            else self.cmd_qpos(component_cmds)
        )
        if target_qpos.size == 0:
            qpos = target_qpos
        else:
            delta_qpos = target_qpos - self.current_cmd_qpos
            max_velocity = np.max(np.abs(delta_qpos)) / self.period
            if max_velocity > self.velocity_limit:
                now = time.monotonic()
                if (
                    not self.velocity_limit_active
                    or now - self.last_velocity_limit_warning_time >= 1.0
                ):
                    logger_mp.warning(
                        "Joint command velocity limited: requested=%.3f "
                        "rad/s applied=%.3f rad/s limit=%.3f rad/s "
                        "groups=%s",
                        max_velocity,
                        self.velocity_limit,
                        self.velocity_limit,
                        self.cmd_joint_group_order,
                    )
                    self.last_velocity_limit_warning_time = now
                self.velocity_limit_active = True
                qpos = self.current_cmd_qpos + delta_qpos * (
                    self.velocity_limit / max_velocity
                )
            else:
                self.velocity_limit_active = False
                qpos = target_qpos
        self.current_cmd_qpos = qpos

        publish_cmds = self.component_cmds_with_qpos(component_cmds, qpos)
        for schema_key, rate in self.torque_rate_limits.items():
            source = self.joint_group_cmd(component_cmds, schema_key)
            previous = self.joint_group_cmd(
                self.last_component_cmds or self.hold_component_cmds, schema_key,
            ).qtau
            qtau = np.where(
                timed_out | (source.qtau == 0.0),
                0.0,
                previous + np.clip(
                    source.qtau - previous, -rate * self.period, rate * self.period,
                ),
            )
            publish_cmds[self.joint_group_components[schema_key]][schema_key] = JointGroupCmd(
                num_joints=qtau.size, qtau=qtau,
            )
        if self.compute_gravity_qtau is not None and (
            self.thermal_safety is None or not self.thermal_safety.active
        ):
            # Replace upstream feedforward, including during startup and hold.
            # Thermal protection owns the complete command while active.
            for schema_key, qtau in self.compute_gravity_qtau().items():
                self.joint_group_cmd(publish_cmds, schema_key).qtau = qtau
        if self.base_group_component is not None:
            base_cmd = publish_cmds[self.base_group_component][self.base_group_schema_key]
            previous = (self.last_component_cmds or self.hold_component_cmds)[
                self.base_group_component
            ][self.base_group_schema_key]
            values = vars(base_cmd).copy()
            for name, limit in (
                ("waist_roll_qpos", self.velocity_limit),
                ("waist_pitch_qpos", self.velocity_limit),
                ("waist_yaw_qpos", self.velocity_limit),
                ("body_height", self.body_height_rate_limit),
            ):
                target, current = values[name], getattr(previous, name)
                if target is not None and current is not None and limit is not None:
                    values[name] = current if timed_out else float(
                        current + np.clip(
                            target - current, -limit * self.period, limit * self.period,
                        )
                    )
            publish_cmds[self.base_group_component][self.base_group_schema_key] = (
                BaseGroupCmd(**values)
            )
        if timed_out and self.base_group_component is not None:
            base_group_cmd = publish_cmds[self.base_group_component][
                self.base_group_schema_key
            ]
            if not isinstance(base_group_cmd, BaseGroupCmd):
                raise TypeError("Target has invalid base_group command.")
            publish_cmds[self.base_group_component][
                self.base_group_schema_key
            ] = self.zero_base_group_cmd(base_group_cmd)
        return publish_cmds

    def stop_component_cmds(self) -> Mapping[str, ComponentCmds] | None:
        if (
            not self.zero_base_group_on_stop
            or self.base_group_component is None
        ):
            if self.torque_rate_limits:
                return self.zero_torque_cmds(self.last_component_cmds or self.hold_component_cmds)
            return None
        if self.compute_gravity_qtau is not None and self.last_component_cmds is not None:
            # Keep the final supported pose when stopping the mobile base.
            stop_cmds = {
                component_id: dict(cmds)
                for component_id, cmds in self.last_component_cmds.items()
            }
            stop_cmds[self.base_group_component][self.base_group_schema_key] = (
                self.zero_base_group_cmd(
                    stop_cmds[self.base_group_component][self.base_group_schema_key]
                )
            )
            return self.zero_torque_cmds(stop_cmds)
        stop_cmds: dict[str, dict[str, ComponentCmdValue]] = {}
        for schema_key, component_id in self.joint_group_components.items():
            source = self.joint_group_cmd(self.hold_component_cmds, schema_key)
            if source.qpos is None:
                stop_cmds.setdefault(component_id, {})[schema_key] = source
                continue
            qpos = self.read_joint_group_qpos(schema_key)
            stop_cmds.setdefault(component_id, {})[
                schema_key
            ] = JointGroupCmd(
                num_joints=qpos.size,
                qpos=qpos,
                qtau=np.zeros_like(qpos),
            )
        stop_cmds.setdefault(self.base_group_component, {})[
            self.base_group_schema_key
        ] = BaseGroupCmd(
            base_linear_vel_x=0.0,
            base_linear_vel_y=0.0,
            base_angular_vel_z=0.0,
            lift_linear_vel_z=0.0,
        )
        return stop_cmds

    @staticmethod
    def zero_torque_cmds(
        component_cmds: Mapping[str, ComponentCmds],
    ) -> dict[str, dict[str, ComponentCmdValue]]:
        """Clear torque-only actuators while preserving body gravity feedforward."""
        cleared = {component: dict(cmds) for component, cmds in component_cmds.items()}
        for cmds in cleared.values():
            for key, cmd in cmds.items():
                if isinstance(cmd, JointGroupCmd) and cmd.qpos is None:
                    cmds[key] = JointGroupCmd(
                        num_joints=cmd.qtau.size, qtau=np.zeros_like(cmd.qtau),
                    )
        return cleared

    def clear_cmd_state(self) -> None:
        self.current_cmd_qpos = None
        super().clear_cmd_state()
        self.velocity_limit_active = False
        self.last_velocity_limit_warning_time = 0.0

    def cmd_qpos(
        self,
        component_cmds: Mapping[str, ComponentCmds],
    ) -> np.ndarray:
        qpos_values = []
        for schema_key in self.cmd_joint_group_order:
            joint_group_cmd = self.joint_group_cmd(
                component_cmds,
                schema_key,
            )
            qpos_values.append(joint_group_cmd.qpos)
        return np.concatenate(qpos_values) if qpos_values else np.empty(0, dtype=np.float32)

    def current_qpos(self) -> np.ndarray:
        if not self.cmd_joint_group_order:
            return np.empty(0, dtype=np.float32)
        qpos = np.concatenate(
            [
                self.read_joint_group_qpos(schema_key)
                for schema_key in self.cmd_joint_group_order
            ],
        )
        if qpos.size != self.cmd_dof:
            raise ValueError(
                f"{type(self).__name__} current qpos expected "
                f"{self.cmd_dof} values, received {qpos.size}."
            )
        return qpos

    def component_cmds_with_qpos(
        self,
        component_cmds: Mapping[str, ComponentCmds],
        qpos: np.ndarray,
    ) -> dict[str, dict[str, ComponentCmdValue]]:
        updated_cmds = {
            component_id: dict(cmds_for_component)
            for component_id, cmds_for_component in component_cmds.items()
        }
        offset = 0
        for schema_key in self.cmd_joint_group_order:
            source_cmd = self.joint_group_cmd(component_cmds, schema_key)
            num_joints = source_cmd.qpos.size
            component_id = self.joint_group_components[schema_key]
            updated_cmds[component_id][schema_key] = JointGroupCmd(
                num_joints=num_joints,
                qpos=qpos[offset : offset + num_joints],
                qtau=source_cmd.qtau,
            )
            offset += num_joints
        return updated_cmds

    def joint_group_cmd(
        self,
        component_cmds: Mapping[str, ComponentCmds],
        schema_key: str,
    ) -> JointGroupCmd:
        component_id = self.joint_group_components[schema_key]
        cmd = component_cmds[component_id][schema_key]
        if not isinstance(cmd, JointGroupCmd):
            raise TypeError(
                f"Command {schema_key!r} must be a JointGroupCmd."
            )
        return cmd

    @staticmethod
    def zero_base_group_cmd(cmd: BaseGroupCmd) -> BaseGroupCmd:
        values = vars(cmd).copy()
        for name in (
            "body_linear_vel_x", "body_linear_vel_y", "body_angular_vel_z",
            "base_linear_vel_x", "base_linear_vel_y", "base_angular_vel_z",
            "lift_linear_vel_z",
        ):
            if values[name] is not None:
                values[name] = 0.0
        return BaseGroupCmd(**values)
