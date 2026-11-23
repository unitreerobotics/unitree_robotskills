"""Validate robot-composition dual-arm kinematics settings."""

from pathlib import PurePosixPath

from robot_runtime.configs.common import (
    reject_unknown_fields,
    require_finite_float,
    require_mapping,
    require_non_empty_str,
    require_positive_float,
)


class DualArmKinematicsSpec:
    """Expose validated dual-arm model and joint-layout settings."""

    def __init__(
        self,
        kinematics_config: object,
        *,
        left_arm_joint_names: tuple[str, ...],
        right_arm_joint_names: tuple[str, ...],
    ) -> None:
        label = "Robot specification kinematics.dual_arm"
        config = require_mapping(kinematics_config, label=label)
        required_fields = frozenset(
            {
                "urdf_path",
                "end_effector_translation",
                "rotation_cost_weight",
            }
        )
        reject_unknown_fields(config, required_fields, label=label)
        missing_fields = required_fields - set(config)
        if missing_fields:
            raise KeyError(
                f"{label} must define {tuple(sorted(missing_fields))!r}."
            )

        urdf_path = PurePosixPath(
            require_non_empty_str(
                config["urdf_path"],
                label=f"{label}.urdf_path",
            )
        )
        if urdf_path.is_absolute() or ".." in urdf_path.parts:
            raise ValueError(
                f"{label}.urdf_path must be robot-assets-relative."
            )
        if urdf_path.suffix != ".urdf":
            raise ValueError(f"{label}.urdf_path must end with '.urdf'.")

        translation = config["end_effector_translation"]
        if not isinstance(translation, list | tuple):
            raise TypeError(
                f"{label}.end_effector_translation must be a numeric list."
            )
        if len(translation) != 3:
            raise ValueError(
                f"{label}.end_effector_translation must contain 3 values."
            )

        if not left_arm_joint_names or not right_arm_joint_names:
            raise ValueError(f"{label} requires non-empty arm joint groups.")

        self.urdf_path = urdf_path.as_posix()
        self.end_effector_translation: tuple[float, float, float] = (
            require_finite_float(
                translation[0],
                label=f"{label}.end_effector_translation[0]",
            ),
            require_finite_float(
                translation[1],
                label=f"{label}.end_effector_translation[1]",
            ),
            require_finite_float(
                translation[2],
                label=f"{label}.end_effector_translation[2]",
            ),
        )
        self.rotation_cost_weight = require_positive_float(
            config["rotation_cost_weight"],
            label=f"{label}.rotation_cost_weight",
        )
        self.left_arm_joint_names = tuple(left_arm_joint_names)
        self.right_arm_joint_names = tuple(right_arm_joint_names)
