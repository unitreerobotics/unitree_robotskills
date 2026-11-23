"""Mode-independent runtime thermal command protection."""

from collections.abc import Callable, Mapping
import time

import logging_mp
import numpy as np

from robot_runtime.interfaces.robot.cmd.base_group import BaseGroupCmd
from robot_runtime.interfaces.robot.cmd.joint_group import JointGroupCmd
from robot_runtime.interfaces.robot.cmd.pose_82d import Pose82dCmd
from robot_runtime.interfaces.robot.component import (
    ComponentCmds,
    ComponentCmdValue,
)


logger_mp = logging_mp.getLogger(__name__)

THERMAL_FIELDS = ("casing_temp", "winding_temp")
THERMAL_RECOVERY_SECONDS = 1.0


class ThermalSafety:
    """Hold targets on overheat, or only warn when protection is disabled."""

    def __init__(
        self,
        *,
        thresholds_by_schema_key: Mapping[
            str,
            Mapping[str, Mapping[str, float]],
        ],
        read_joint_states: Callable[[], Mapping[str, object]],
        protection_enabled: bool = True,
        waist_joint_names: tuple[str, ...] = (),
    ) -> None:
        self.thresholds_by_schema_key = {
            schema_key: {
                field_name: dict(thresholds)
                for field_name, thresholds in field_thresholds.items()
            }
            for schema_key, field_thresholds in thresholds_by_schema_key.items()
        }
        self.read_joint_states = read_joint_states
        self.protection_enabled = protection_enabled
        self.waist_joint_names = waist_joint_names
        self.active = False
        self.protected_cmds: Mapping[str, ComponentCmds] | None = None
        self.recovery_started_at: float | None = None
        self.last_warning_at: float | None = None

    def apply(
        self,
        component_cmds: Mapping[str, ComponentCmds],
        *,
        last_component_cmds: Mapping[str, ComponentCmds] | None = None,
    ) -> Mapping[str, ComponentCmds]:
        """Return the current target or the fixed commands captured on overheat."""
        joint_states = self.read_joint_states()
        if not self.protection_enabled:
            overheat = self.find_overheat(joint_states)
            if overheat is not None:
                now = time.monotonic()
                if self.last_warning_at is None or now - self.last_warning_at >= 1.0:
                    logger_mp.warning(
                        "Thermal warning only: %s.%s[%d]=%.1f exceeds %.1f; "
                        "commands continue.",
                        *overheat,
                    )
                    self.last_warning_at = now
            return component_cmds
        if self.active:
            if self.temperatures_recovered(joint_states):
                now = time.monotonic()
                if self.recovery_started_at is None:
                    self.recovery_started_at = now
                elif now - self.recovery_started_at >= THERMAL_RECOVERY_SECONDS:
                    logger_mp.info("Thermal command protection recovered.")
                    self.reset()
                    return component_cmds
            else:
                self.recovery_started_at = None
            if self.protected_cmds is None:
                raise RuntimeError("Active thermal protection has no held command.")
            return self.protected_cmds

        overheat = self.find_overheat(joint_states)
        if overheat is None:
            return component_cmds

        (
            schema_key,
            field_name,
            joint_index,
            temperature,
            trigger_threshold,
        ) = overheat
        self.protected_cmds = self.build_protected_cmds(
            component_cmds,
            joint_states,
            last_component_cmds=last_component_cmds,
        )
        self.active = True
        logger_mp.warning(
            "Thermal command protection active: %s.%s[%d]=%.1f exceeds %.1f.",
            schema_key,
            field_name,
            joint_index,
            temperature,
            trigger_threshold,
        )
        return self.protected_cmds

    def find_overheat(
        self,
        joint_states: Mapping[str, object],
    ) -> tuple[str, str, int, float, float] | None:
        """Return the first temperature above its configured trigger."""
        for schema_key, field_thresholds in self.thresholds_by_schema_key.items():
            state = joint_states[schema_key]
            for field_name in THERMAL_FIELDS:
                thresholds = field_thresholds.get(field_name)
                if thresholds is None:
                    continue
                values = getattr(state, field_name)
                if values is None:
                    continue
                temperatures = np.asarray(values, dtype=np.float32).reshape(-1)
                trigger_threshold = thresholds["trigger"]
                indices = np.flatnonzero(
                    np.isfinite(temperatures)
                    & (temperatures > trigger_threshold)
                )
                if indices.size:
                    joint_index = int(indices[0])
                    return (
                        schema_key,
                        field_name,
                        joint_index,
                        float(temperatures[joint_index]),
                        trigger_threshold,
                    )
        return None

    def temperatures_recovered(
        self,
        joint_states: Mapping[str, object],
    ) -> bool:
        """Return whether every available temperature reached release."""
        observed_temperature = False
        for schema_key, field_thresholds in self.thresholds_by_schema_key.items():
            state = joint_states[schema_key]
            for field_name in THERMAL_FIELDS:
                thresholds = field_thresholds.get(field_name)
                if thresholds is None:
                    continue
                values = getattr(state, field_name)
                if values is None:
                    continue
                temperatures = np.asarray(values, dtype=np.float32).reshape(-1)
                if temperatures.size == 0 or not np.all(
                    np.isfinite(temperatures)
                ):
                    return False
                observed_temperature = True
                release_threshold = thresholds["release"]
                if np.any(temperatures > release_threshold):
                    return False
        return observed_temperature

    def build_protected_cmds(
        self,
        component_cmds: Mapping[str, ComponentCmds],
        joint_states: Mapping[str, object],
        *,
        last_component_cmds: Mapping[str, ComponentCmds] | None = None,
    ) -> dict[str, dict[str, ComponentCmdValue]]:
        """Hold last sent joint targets; use feedback only before the first output."""
        protected_cmds: dict[str, dict[str, ComponentCmdValue]] = {}
        for component_id, cmds_for_component in component_cmds.items():
            protected_for_component = {}
            for schema_key, source_cmd in cmds_for_component.items():
                if isinstance(source_cmd, JointGroupCmd):
                    if source_cmd.qpos is None:
                        protected_for_component[schema_key] = JointGroupCmd(
                            num_joints=source_cmd.qtau.size,
                            qtau=np.zeros_like(source_cmd.qtau),
                        )
                        continue
                    state = joint_states.get(schema_key)
                    last_cmd = (
                        None if last_component_cmds is None
                        else last_component_cmds.get(component_id, {}).get(schema_key)
                    )
                    if last_cmd is not None and last_cmd.qpos is not None:
                        qpos = last_cmd.qpos.copy()
                    elif state is not None:
                        qpos = np.asarray(
                            state.qpos, dtype=np.float32,
                        ).reshape(-1).copy()
                    else:
                        qpos = source_cmd.qpos.copy()
                    protected_cmd = JointGroupCmd(
                        num_joints=qpos.size,
                        qpos=qpos,
                        qtau=np.zeros_like(qpos),
                    )
                elif isinstance(source_cmd, BaseGroupCmd):
                    held_cmd = (
                        source_cmd if last_component_cmds is None
                        else last_component_cmds[component_id][schema_key]
                    )
                    values = vars(held_cmd).copy()
                    for name in (
                        "body_linear_vel_x", "body_linear_vel_y", "body_angular_vel_z",
                        "base_linear_vel_x", "base_linear_vel_y", "base_angular_vel_z",
                        "lift_linear_vel_z",
                    ):
                        if values[name] is not None:
                            values[name] = 0.0
                    if self.waist_joint_names and "waist" in joint_states:
                        waist_qpos = dict(zip(
                            self.waist_joint_names, joint_states["waist"].qpos, strict=True,
                        ))
                        for axis in ("roll", "pitch", "yaw"):
                            name = f"waist_{axis}_qpos"
                            if values[name] is not None:
                                values[name] = float(waist_qpos[f"waist_{axis}_joint"])
                    protected_cmd = BaseGroupCmd(**values)
                elif isinstance(source_cmd, Pose82dCmd):
                    protected_cmd = Pose82dCmd(
                        smpl_joints=source_cmd.smpl_joints.copy(),
                        smpl_root_quat_wxyz=(
                            source_cmd.smpl_root_quat_wxyz.copy()
                        ),
                        wrist_qpos=source_cmd.wrist_qpos.copy(),
                    )
                else:
                    raise TypeError(
                        f"Unsupported component command {type(source_cmd).__name__}."
                    )
                protected_for_component[schema_key] = protected_cmd
            protected_cmds[component_id] = protected_for_component
        return protected_cmds

    def reset(self) -> None:
        self.active = False
        self.protected_cmds = None
        self.recovery_started_at = None
        self.last_warning_at = None
