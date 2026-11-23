"""Validate teleoperation input and retargeting configuration."""

from robot_runtime.configs.sections.policy.config import PolicyConfig
from robot_runtime.configs.sections.policy.hmi.xr import XRConfig
from robot_runtime.configs.common import (
    reject_unknown_fields,
    require_choice,
    require_finite_float,
    require_float_range,
    require_mapping,
    require_non_empty_str,
    require_positive_float,
)


def require_filter_time_constant(value: object, *, label: str) -> float:
    """Validate an exponential-filter time constant in seconds."""
    time_constant = require_finite_float(value, label=label)
    if time_constant < 0.0:
        raise ValueError(f"{label} must be non-negative.")
    return time_constant


class TeleopConfig(PolicyConfig):
    """Normalize one resolved ``policy`` section for teleoperation.

    Args:
        policy_config: Teleoperation policy mapping containing an XR HMI source,
            update frequency, and retargeter declarations.

    Attributes:
        hmi_config: Validated single HMI input configuration.
        retargeter_configs: Retargeter configurations keyed by YAML name.
        display_delay: Whether to display Teleop loop delay statistics.
        reset_pose_on_start: Whether Policy requests the configured robot
            reset pose before accepting Teleop start input.

    Example:
        config = TeleopConfig(
            {
                "policy_id": "teleop",
                "update_frequency": 30.0,
                "hmi": {
                    "type": "xrobotoolkit.xr",
                    "input_mode": "controller",
                },
                "retargeters": {
                    "dual_arm": {
                        "type": "xr_body",
                    },
                },
            }
        )
        assert config.hmi_config.source_type == "xr"
    """

    def __init__(self, policy_config: dict[str, object]) -> None:
        policy_config = require_mapping(policy_config, label="policy")
        reject_unknown_fields(
            policy_config,
            frozenset(
                {
                    "policy_id",
                    "update_frequency",
                    "display_delay",
                    "reset_pose_on_start",
                    "hmi",
                    "retargeters",
                }
            ),
            label="policy",
        )
        super().__init__(policy_config)
        if self.policy_id != "teleop":
            raise ValueError(
                "policy.policy_id must be 'teleop' for TeleopConfig."
            )
        display_delay = policy_config.get("display_delay", False)
        if type(display_delay) is not bool:
            raise TypeError("policy.display_delay must be a boolean.")
        self.display_delay = display_delay
        reset_pose_on_start = policy_config.get(
            "reset_pose_on_start",
            False,
        )
        if type(reset_pose_on_start) is not bool:
            raise TypeError(
                "policy.reset_pose_on_start must be a boolean."
            )
        self.reset_pose_on_start = reset_pose_on_start

        hmi_config = require_mapping(
            policy_config["hmi"],
            label="policy.hmi",
        )
        driver_type = require_non_empty_str(
            hmi_config.get("type"),
            label="policy.hmi.type",
        )
        source_type = driver_type.rpartition(".")[2]
        if source_type != "xr":
            raise KeyError(
                f"Unsupported policy.hmi source type {source_type!r}. "
                "Known: xr."
            )
        self.hmi_config = XRConfig(hmi_config)
        retargeter_configs = require_mapping(
            policy_config["retargeters"],
            label="policy.retargeters",
        )
        if not retargeter_configs:
            raise KeyError("policy.retargeters must be a non-empty mapping.")

        normalized_retargeter_configs: dict[str, dict[str, object]] = {}
        for retargeter_name, retargeter_config in retargeter_configs.items():
            retargeter_name = require_non_empty_str(
                retargeter_name,
                label="policy.retargeters key",
            )
            retargeter_config = require_mapping(
                retargeter_config,
                label=f"policy.retargeters.{retargeter_name}",
            )
            if "name" in retargeter_config:
                raise KeyError(
                    f"policy.retargeters.{retargeter_name} must not define "
                    "name; the retargeter key is its name."
                )
            if "source" in retargeter_config:
                raise KeyError(
                    f"policy.retargeters.{retargeter_name} must not define "
                    "source; a Teleop policy has one HMI input."
                )
            try:
                raw_retargeter_type = retargeter_config["type"]
            except KeyError as exc:
                raise KeyError(
                    f"policy.retargeters.{retargeter_name} must define type."
                ) from exc
            retargeter_type = require_non_empty_str(
                raw_retargeter_type,
                label=f"policy.retargeters.{retargeter_name}.type",
            )
            input_type, separator, target_type = retargeter_type.partition("_")
            if not separator or not input_type or not target_type:
                raise ValueError(
                    f"policy.retargeters.{retargeter_name}.type="
                    f"{retargeter_type!r} must use <input>_<target>."
                )
            if input_type != self.hmi_config.source_type:
                raise ValueError(
                    f"policy.retargeters.{retargeter_name}.type="
                    f"{retargeter_type!r} requires HMI input type "
                    f"{input_type!r}, but policy.hmi.type="
                    f"{self.hmi_config.driver_type!r}."
                )
            retargeter_config["type"] = retargeter_type
            if retargeter_type == "xr_body":
                wrist_tracking_label = (
                    f"policy.retargeters.{retargeter_name}.wrist_tracking"
                )
                wrist_tracking = require_mapping(
                    retargeter_config.get("wrist_tracking", {}),
                    label=wrist_tracking_label,
                )
                reject_unknown_fields(
                    wrist_tracking,
                    frozenset(
                        {
                            "use_current_qpos_seed",
                            "filter_time_constant",
                            "regularization_reference",
                        }
                    ),
                    label=wrist_tracking_label,
                )
                use_current_qpos_seed = wrist_tracking.get(
                    "use_current_qpos_seed",
                    False,
                )
                if type(use_current_qpos_seed) is not bool:
                    raise TypeError(
                        f"{wrist_tracking_label}.use_current_qpos_seed "
                        "must be a boolean."
                    )
                wrist_tracking["use_current_qpos_seed"] = (
                    use_current_qpos_seed
                )
                wrist_tracking["filter_time_constant"] = (
                    require_filter_time_constant(
                        wrist_tracking.get("filter_time_constant", 0.0),
                        label=f"{wrist_tracking_label}.filter_time_constant",
                    )
                )
                wrist_tracking["regularization_reference"] = (
                    require_choice(
                        wrist_tracking.get("regularization_reference", "zero"),
                        choices=frozenset({"zero", "session_start"}),
                        label=f"{wrist_tracking_label}.regularization_reference",
                    )
                )
                retargeter_config["wrist_tracking"] = wrist_tracking
            elif retargeter_type == "xr_gripper":
                retargeter_label = f"policy.retargeters.{retargeter_name}"
                reject_unknown_fields(
                    retargeter_config,
                    frozenset({"type", "rate", "button_deadzone"}),
                    label=retargeter_label,
                )
                rate = require_finite_float(
                    retargeter_config.get("rate", 0.8),
                    label=f"{retargeter_label}.rate",
                )
                if rate <= 0.0:
                    raise ValueError(
                        f"{retargeter_label}.rate must be positive."
                    )
                retargeter_config["rate"] = rate
                button_deadzone = require_finite_float(
                    retargeter_config.get("button_deadzone", 0.15),
                    label=f"{retargeter_label}.button_deadzone",
                )
                if not 0.0 <= button_deadzone < 1.0:
                    raise ValueError(
                        f"{retargeter_label}.button_deadzone must be in "
                        "[0, 1)."
                    )
                retargeter_config["button_deadzone"] = button_deadzone
            elif retargeter_type == "xr_dexterous_hand":
                retargeter_label = f"policy.retargeters.{retargeter_name}"
                reject_unknown_fields(
                    retargeter_config,
                    frozenset({"type", "filter_time_constant"}),
                    label=retargeter_label,
                )
                retargeter_config["filter_time_constant"] = (
                    require_filter_time_constant(
                        retargeter_config.get("filter_time_constant", 0.1),
                        label=f"{retargeter_label}.filter_time_constant",
                    )
                )
            elif retargeter_type == "xr_base_lift":
                retargeter_label = f"policy.retargeters.{retargeter_name}"
                reject_unknown_fields(
                    retargeter_config,
                    frozenset(
                        {
                            "type",
                            "filter_time_constant",
                            "forward_velocity_range",
                            "yaw_velocity_range",
                            "lift_velocity_range",
                        }
                    ),
                    label=retargeter_label,
                )
                retargeter_config["filter_time_constant"] = (
                    require_filter_time_constant(
                        retargeter_config.get("filter_time_constant", 0.1),
                        label=f"{retargeter_label}.filter_time_constant",
                    )
                )
                for field_name, default_range in (
                    ("forward_velocity_range", (-0.2, 0.2)),
                    ("yaw_velocity_range", (-0.6, 0.6)),
                    ("lift_velocity_range", (-1.0, 1.0)),
                ):
                    velocity_range = require_float_range(
                        retargeter_config.get(field_name, default_range),
                        label=f"{retargeter_label}.{field_name}",
                    )
                    if not velocity_range[0] <= 0.0 <= velocity_range[1]:
                        raise ValueError(
                            f"{retargeter_label}.{field_name} must contain "
                            "zero."
                        )
                    retargeter_config[field_name] = velocity_range
            elif retargeter_type == "xr_pseudo_base_lift":
                retargeter_label = f"policy.retargeters.{retargeter_name}"
                reject_unknown_fields(
                    retargeter_config,
                    frozenset(
                        {
                            "type",
                            "filter_time_constant",
                            "forward_velocity_range",
                            "lateral_velocity_range",
                            "yaw_velocity_range",
                            "body_height_rate_range",
                            "waist_yaw_range",
                            "waist_roll_range",
                            "waist_pitch_range",
                        }
                    ),
                    label=retargeter_label,
                )
                retargeter_config["filter_time_constant"] = (
                    require_filter_time_constant(
                        retargeter_config.get("filter_time_constant", 0.1),
                        label=f"{retargeter_label}.filter_time_constant",
                    )
                )
                for field_name in (
                    "forward_velocity_range",
                    "lateral_velocity_range",
                    "yaw_velocity_range",
                    "body_height_rate_range",
                ):
                    velocity_range = require_float_range(
                        retargeter_config[field_name],
                        label=f"{retargeter_label}.{field_name}",
                    )
                    if not velocity_range[0] <= 0.0 <= velocity_range[1]:
                        raise ValueError(
                            f"{retargeter_label}.{field_name} must contain "
                            "zero."
                        )
                    retargeter_config[field_name] = velocity_range

                for field_name in (
                    "waist_yaw_range",
                    "waist_roll_range",
                    "waist_pitch_range",
                ):
                    retargeter_config[field_name] = require_float_range(
                        retargeter_config[field_name],
                        label=f"{retargeter_label}.{field_name}",
                    )

            elif retargeter_type == "xr_hybrid_wbc":
                label = f"policy.retargeters.{retargeter_name}"
                reject_unknown_fields(
                    retargeter_config,
                    frozenset({
                        "type", "velocity_scale", "use_cuda", "smpl_skeleton_path",
                        "root_orientation_mode", "elbow_swing_roll_gain",
                        "suppress_outliers", "wrist_jump_threshold", "arm_jump_threshold",
                        "pose_jump_threshold", "max_suppressed_samples",
                    }),
                    label=label,
                )
                velocity_scale = retargeter_config.get("velocity_scale", (0.8, 0.7, 1.0))
                if not isinstance(velocity_scale, (list, tuple)) or len(velocity_scale) != 3:
                    raise ValueError(f"{label}.velocity_scale requires 3 scales.")
                retargeter_config["velocity_scale"] = tuple(
                    require_positive_float(value, label=f"{label}.velocity_scale")
                    for value in velocity_scale
                )
            elif retargeter_type != "xr_pose_82d":
                raise KeyError(
                    f"Unsupported policy retargeter type {retargeter_type!r}."
                )
            normalized_retargeter_configs[retargeter_name] = retargeter_config

        self.retargeter_configs = normalized_retargeter_configs
