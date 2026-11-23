"""Validate runtime scheduling and motion-control configuration.

``RuntimeConfig`` is the configuration boundary shared by the runtime and
policy processes. ``resolve_motion_schema_keys`` resolves the required action
schema from the control mode and the body's declared joint groups.
"""

from collections.abc import Collection
from enum import Enum

from robot_runtime.configs.common import (
    reject_unknown_fields,
    require_choice,
    require_positive_float,
)


class RuntimeControlMode(Enum):
    DUAL_ARM = "dual_arm"
    UPPER_BODY = "upper_body"
    MOBILE_WBC = "mobile_wbc"
    DECOUPLED_WBC = "decoupled_wbc"
    HUMANOID_WBC = "humanoid_wbc"
    HYBRID_WBC = "hybrid_wbc"


MOTION_SCHEMA_KEYS = {
    RuntimeControlMode.HYBRID_WBC: {
        "joint_group": ("left_arm", "right_arm"),
        "pose_82d": ("pose_82d",),
    },
    RuntimeControlMode.DUAL_ARM: {
        "joint_group": ("left_arm", "right_arm")
    },
    RuntimeControlMode.UPPER_BODY: {
        "joint_group": ("left_arm", "right_arm")
    },
    RuntimeControlMode.MOBILE_WBC: {
        "joint_group": ("left_arm", "right_arm", "waist"),
    },
    RuntimeControlMode.DECOUPLED_WBC: {
        "joint_group": ("left_arm", "right_arm")
    },
    RuntimeControlMode.HUMANOID_WBC: {
        "pose_82d": ("pose_82d",)
    },
}


def resolve_motion_schema_keys(
    control_mode: RuntimeControlMode,
    body_schema_keys: Collection[str],
) -> dict[str, tuple[str, ...]]:
    """Resolve required actions; biped upper-body control never owns the waist.

    Head and waist are included only when the body declares them. Fixed
    LowCmd holding targets are driver requirements, not motion actions.
    Mobile WBC retains its existing optional head-control behavior.
    """
    schema_keys = dict(MOTION_SCHEMA_KEYS[control_mode])
    if control_mode is RuntimeControlMode.UPPER_BODY:
        is_biped = not {"left_leg", "right_leg"}.isdisjoint(body_schema_keys)
        extra_keys = ()
        if "waist" in body_schema_keys and not is_biped:
            extra_keys += ("waist",)
        if "head" in body_schema_keys:
            extra_keys += ("head",)
        schema_keys["joint_group"] += extra_keys
    return schema_keys


class RuntimeConfig:
    """Normalize runtime scheduling and motion-control values.

    Joint-command modes require ``cmd_joint_velocity_limit`` because the
    runtime interpolates their joint targets. ``humanoid_wbc`` forwards a
    ``pose_82d`` command instead, so that mode forbids the joint limit field
    and exposes ``None`` through the corresponding attribute.

    ``cmd_body_height_rate_limit`` bounds changes of the G1 EDU body-height
    target in m/s. It does not command or limit odometry z velocity.

    Args:
        runtime_config: Runtime section from a resolved experiment. It must
            contain the state publication rate, motion-control mode, command
            loop rate, command timeout, and the joint velocity limit when the
            selected mode uses joint commands.

    Attributes:
        state_frequency: Runtime-state publication frequency in Hz.
        control_mode: Selected mode and its required action schema.
        cmd_frequency: Robot command-loop frequency in Hz.
        cmd_timeout: Seconds after which an unrefreshed command is
            treated as stale.
        cmd_joint_velocity_limit: Sustained joint-target change-rate limit in
            rad/s, enforced on every command-loop update. This is not a motor
            velocity limit or a startup ramp. It is ``None`` for
            ``humanoid_wbc``.
        thermal_protection_enabled: Hold commands on overheat by default.
            When false, report temperature warnings without changing commands.

    Raises:
        KeyError: A required field is missing, an unsupported field is
            present, or ``humanoid_wbc`` declares a joint velocity limit.
        TypeError: A numeric field is not an integer or float.
        ValueError: A numeric field is not positive or the motion-control
            mode is unsupported.

    Example:
        runtime = RuntimeConfig(
            {
                "state_frequency": 30.0,
                "control_mode": "mobile_wbc",
                "cmd_frequency": 250.0,
                "cmd_timeout": 0.15,
                "cmd_joint_velocity_limit": 3.0,
            }
        )
        assert runtime.control_mode is RuntimeControlMode.MOBILE_WBC
        assert runtime.to_dict()["cmd_joint_velocity_limit"] == 3.0
    """

    def __init__(
        self,
        runtime_config: dict[str, object],
    ) -> None:
        reject_unknown_fields(
            runtime_config,
            frozenset(
                {
                    "state_frequency",
                    "control_mode",
                    "cmd_frequency",
                    "cmd_timeout",
                    "cmd_joint_velocity_limit",
                    "cmd_body_height_rate_limit",
                    "thermal_protection_enabled",
                }
            ),
            label="runtime",
        )
        self.state_frequency = require_positive_float(
            runtime_config["state_frequency"],
            label="runtime.state_frequency",
        )
        self.control_mode = RuntimeControlMode(
            require_choice(
                runtime_config["control_mode"],
                choices=frozenset(mode.value for mode in RuntimeControlMode),
                label="runtime.control_mode",
            )
        )
        self.cmd_frequency = require_positive_float(
            runtime_config["cmd_frequency"],
            label="runtime.cmd_frequency",
        )
        self.cmd_timeout = require_positive_float(
            runtime_config["cmd_timeout"],
            label="runtime.cmd_timeout",
        )
        self.thermal_protection_enabled = runtime_config.get(
            "thermal_protection_enabled", True,
        )
        if type(self.thermal_protection_enabled) is not bool:
            raise TypeError("runtime.thermal_protection_enabled must be a boolean.")
        if self.control_mode is RuntimeControlMode.HUMANOID_WBC:
            if "cmd_joint_velocity_limit" in runtime_config:
                raise KeyError(
                    "humanoid_wbc runtime must not configure "
                    "cmd_joint_velocity_limit."
                )
            self.cmd_joint_velocity_limit = None
        else:
            self.cmd_joint_velocity_limit = require_positive_float(
                runtime_config["cmd_joint_velocity_limit"],
                label="runtime.cmd_joint_velocity_limit",
            )

        self.cmd_body_height_rate_limit = None
        if self.control_mode is RuntimeControlMode.DECOUPLED_WBC:
            self.cmd_body_height_rate_limit = require_positive_float(
                runtime_config.get("cmd_body_height_rate_limit", 0.15),
                label="runtime.cmd_body_height_rate_limit",
            )
        elif "cmd_body_height_rate_limit" in runtime_config:
            raise ValueError("cmd_body_height_rate_limit requires decoupled_wbc.")

    def to_dict(self) -> dict[str, object]:
        """Return a new mapping containing normalized runtime values.

        The result omits ``cmd_joint_velocity_limit`` for ``humanoid_wbc`` so
        it remains valid input for another ``RuntimeConfig`` construction.

        Returns:
            Runtime configuration suitable for serialization.
        """
        runtime_config = {
            "state_frequency": self.state_frequency,
            "control_mode": self.control_mode.value,
            "cmd_frequency": self.cmd_frequency,
            "cmd_timeout": self.cmd_timeout,
        }
        if self.cmd_joint_velocity_limit is not None:
            runtime_config["cmd_joint_velocity_limit"] = (
                self.cmd_joint_velocity_limit
            )
        if self.cmd_body_height_rate_limit is not None:
            runtime_config["cmd_body_height_rate_limit"] = self.cmd_body_height_rate_limit
        if not self.thermal_protection_enabled:
            runtime_config["thermal_protection_enabled"] = False
        return runtime_config
