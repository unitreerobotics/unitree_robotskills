"""Shared lifecycle and scheduling for runtime robot command loops."""

from __future__ import annotations

from collections.abc import Callable, Mapping
import threading
import time

import logging_mp
import numpy as np

from robot_runtime.interfaces.robot.cmd.joint_group import JointGroupCmd
from robot_runtime.interfaces.robot.component import ComponentCmds
from robot_runtime.runtime.safety.thermal import ThermalSafety


logger_mp = logging_mp.getLogger(__name__)


class RobotCmdLoop:
    """Run one command-specific output loop at a fixed frequency."""

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
        self.write_component_cmds = write_component_cmds
        self.period = 1.0 / frequency
        self.cmd_timeout = cmd_timeout
        self.qpos_bounds = {}
        for key, limits in qpos_limits.items():
            bounds = np.asarray(limits, dtype=np.float64)
            if (
                bounds.ndim != 2 or bounds.shape[1] != 2
                or not np.all(np.isfinite(bounds))
                or not np.all(bounds[:, 0] < bounds[:, 1])
            ):
                raise ValueError(f"{key!r} requires finite lower/upper qpos_limits.")
            # Allow only numerical rounding at the boundary, not motion margin.
            self.qpos_bounds[key] = (bounds[:, 0] - 1e-6, bounds[:, 1] + 1e-6)
        self.thermal_safety = thermal_safety
        self.display_delay = False

        self.enter_control = enter_control
        self.exit_control = exit_control
        self.control_active = False

        self.target_cmd_lock = threading.Lock()
        self.target_cmd: Mapping[str, ComponentCmds] | None = None
        self.target_cmd_time: float | None = None
        self.last_component_cmds: Mapping[str, ComponentCmds] | None = None

        self.stop_requested = threading.Event()
        self.cmd_thread: threading.Thread | None = None
        self.cmd_error: Exception | None = None

    def start(self, *, control_already_entered: bool = False) -> None:
        cmd_thread = self.cmd_thread
        if cmd_thread is not None:
            if cmd_thread.is_alive():
                return
            raise RuntimeError(
                f"{type(self).__name__} failed."
            ) from self.cmd_error

        self.cmd_error = None
        self.stop_requested.clear()
        try:
            if control_already_entered:
                self.control_active = True
            elif self.enter_control is not None:
                self.enter_control()
                self.control_active = True
            startup_cmd = self.startup_cmd()
            if startup_cmd is not None:
                self.update_cmd(startup_cmd, timeout_enabled=False)
                self.publish_once(time.monotonic())
            self.cmd_thread = threading.Thread(
                target=self.cmd_loop,
                name=type(self).__name__,
                daemon=True,
            )
            self.cmd_thread.start()
        except Exception:
            self.stop_requested.set()
            self.cmd_thread = None
            self.clear_target()
            self.clear_cmd_state()
            if self.thermal_safety is not None:
                self.thermal_safety.reset()
            if self.control_active and self.exit_control is not None:
                self.control_active = False
                self.exit_control()
            raise

    def stop(self, *, release_control: bool = True) -> None:
        cmd_thread = self.cmd_thread
        if cmd_thread is None:
            return
        self.stop_requested.set()
        cmd_thread.join(timeout=1.0)
        if cmd_thread.is_alive():
            raise RuntimeError(
                f"{type(self).__name__} command loop did not stop within "
                "1.0 seconds."
            )
        self.cmd_thread = None

        try:
            stop_component_cmds = self.stop_component_cmds()
            if stop_component_cmds is not None:
                self.validate_qpos(stop_component_cmds)
                self.write_component_cmds(stop_component_cmds)
        except Exception as exc:
            logger_mp.warning(
                "[%s] Failed to write stop command: %s",
                type(self).__name__,
                exc,
            )
        if self.control_active:
            self.control_active = False
            if release_control and self.exit_control is not None:
                self.exit_control()
        self.clear_target()
        self.clear_cmd_state()
        if self.thermal_safety is not None:
            self.thermal_safety.reset()

    def cmd_loop(self) -> None:
        previous_started_at = None
        delay_report_started_at = None
        cycle_delays = []
        publish_delays = []
        overrun_count = 0
        while not self.stop_requested.is_set():
            started_at = time.monotonic()
            if self.display_delay:
                if delay_report_started_at is None:
                    delay_report_started_at = started_at
                if previous_started_at is not None:
                    cycle_delays.append(started_at - previous_started_at)
                previous_started_at = started_at
            try:
                self.publish_once(started_at)
            except Exception as exc:
                self.cmd_error = exc
                self.stop_requested.set()
                logger_mp.exception(
                    "%s publish failed.",
                    type(self).__name__,
                )
                break

            elapsed = time.monotonic() - started_at
            if self.display_delay:
                publish_delays.append(elapsed)
                if elapsed > self.period:
                    overrun_count += 1
                if (
                    delay_report_started_at is not None
                    and started_at + elapsed - delay_report_started_at >= 1.0
                ):
                    logger_mp.info(
                        "%s delay ms avg/max samples=%d overrun=%d | "
                        "cycle=%.2f/%.2f publish=%.2f/%.2f",
                        type(self).__name__,
                        len(publish_delays),
                        overrun_count,
                        (
                            sum(cycle_delays) / len(cycle_delays) * 1000.0
                            if cycle_delays
                            else 0.0
                        ),
                        (
                            max(cycle_delays) * 1000.0
                            if cycle_delays
                            else 0.0
                        ),
                        sum(publish_delays) / len(publish_delays) * 1000.0,
                        max(publish_delays) * 1000.0,
                    )
                    cycle_delays.clear()
                    publish_delays.clear()
                    overrun_count = 0
                    delay_report_started_at = started_at + elapsed
            if self.stop_requested.wait(max(0.0, self.period - elapsed)):
                break

    def update_cmd(
        self,
        component_cmds: Mapping[str, ComponentCmds],
        *,
        timeout_enabled: bool = True,
    ) -> None:
        if self.cmd_error is not None:
            raise RuntimeError(
                f"{type(self).__name__} failed."
            ) from self.cmd_error
        try:
            self.validate_qpos(component_cmds)
        except ValueError as exc:
            with self.target_cmd_lock:
                # Reuse timeout holding: stop base motion and torque-only hands.
                self.target_cmd = self.last_component_cmds
                self.target_cmd_time = float("-inf")
                self.cmd_error = exc
            raise
        with self.target_cmd_lock:
            self.target_cmd = component_cmds
            self.target_cmd_time = (
                time.monotonic() if timeout_enabled else None
            )

    def publish_once(self, timestamp: float) -> None:
        with self.target_cmd_lock:
            target_cmd = self.target_cmd
            updated_at = self.target_cmd_time
        if target_cmd is None:
            return
        timed_out = (
            updated_at is not None
            and timestamp - updated_at > self.cmd_timeout
        )
        if timed_out and self.last_component_cmds is not None:
            # Hold the complete last output, including any pose commands.
            target_cmd = self.last_component_cmds
        if self.thermal_safety is not None:
            target_cmd = self.thermal_safety.apply(
                target_cmd, last_component_cmds=self.last_component_cmds,
            )
        component_cmds = self.build_component_cmds(
            target_cmd,
            timed_out=timed_out,
        )
        if component_cmds is not None:
            self.validate_qpos(component_cmds)
            self.write_component_cmds(component_cmds)
            self.last_component_cmds = component_cmds

    def validate_qpos(self, component_cmds: Mapping[str, ComponentCmds]) -> None:
        """Check cached bounds for joint targets, including Pose82D end effectors."""
        for component_id, cmds in component_cmds.items():
            for schema_key, cmd in cmds.items():
                if not isinstance(cmd, JointGroupCmd) or cmd.qpos is None:
                    continue
                key = (component_id, schema_key)
                bounds = self.qpos_bounds.get(key)
                if bounds is None:
                    raise ValueError(f"{key!r} has no configured qpos_limits.")
                lower, upper = bounds
                if cmd.qpos.shape != lower.shape:
                    raise ValueError(f"{key!r} qpos must match its joint count.")
                valid = (cmd.qpos >= lower) & (cmd.qpos <= upper)
                if not np.all(valid):
                    indices = np.flatnonzero(~valid).tolist()
                    raise ValueError(
                        f"{key!r} qpos at indices {indices} is non-finite "
                        "or outside qpos_limits."
                    )

    def clear_target(self) -> None:
        with self.target_cmd_lock:
            self.target_cmd = None
            self.target_cmd_time = None

    def startup_cmd(self) -> Mapping[str, ComponentCmds] | None:
        return None

    def stop_component_cmds(self) -> Mapping[str, ComponentCmds] | None:
        return None

    def clear_cmd_state(self) -> None:
        self.last_component_cmds = None

    def build_component_cmds(
        self,
        component_cmds: Mapping[str, ComponentCmds],
        *,
        timed_out: bool,
    ) -> Mapping[str, ComponentCmds] | None:
        """Build the driver-neutral component commands for one cycle."""
        raise NotImplementedError
