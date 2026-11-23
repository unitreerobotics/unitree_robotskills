"""Validate and expose a robot composition mapping.

``RobotSpec`` owns the composed component specs and resolves component or
schema lookups directly from those specs.

Example robot composition spec:
    ``configs/robots/compositions/g1_d_flagship/dex3_1.yaml``

Example YAML:
    body: configs/robots/components/joint_groups/bodies/g1_d_flagship.yaml
    end_effectors:
      - configs/robots/components/joint_groups/end_effectors/unitree/dex3_1_left.yaml
      - configs/robots/components/joint_groups/end_effectors/unitree/dex3_1_right.yaml
    base_group: configs/robots/components/base_groups/g1_d_base.yaml
    sensors:
      imu: configs/robots/components/sensors/imu/default.yaml
      color_camera: configs/robots/components/sensors/color_camera/teleimager.yaml
"""

from collections import Counter
from collections.abc import Mapping

from robot_runtime.configs.common import (
    reject_unknown_fields,
    require_finite_float,
    require_non_empty_str,
    require_positive_float,
)
from robot_runtime.configs.sections.robot.components.base_group import BaseGroupSpec
from robot_runtime.configs.sections.robot.components.joint_group import JointGroupSpec
from robot_runtime.configs.sections.robot.components.sensor import SensorSpec
from robot_runtime.configs.sections.robot.kinematics import DualArmKinematicsSpec


class RobotSpec:
    """Normalize and expose one robot composition.

    Robot composition specifications bind body, end-effector, base-group, and
    sensor component specs into one schema-key namespace used by config
    validation and cross-component lookups.

    Args:
        robot_spec_config: Robot specification mapping whose component
            references have already been resolved to component mappings. It
            declares one body, zero or more end effectors, an optional base
            group, optional sensors, and an optional reset pose.

    Attributes:
        joint_group_specs: Joint-group specs in public composition order. The
            first item is the body; remaining items are end effectors in YAML
            declaration order.
        base_group_spec: Optional base-group role spec.
        sensor_specs: Sensor component specs.
        dual_arm_kinematics: Optional validated dual-arm kinematics settings.
        reset_pose: Optional normalized robot-specific reset pose containing
            a positive joint velocity limit, complete joint-position targets,
            and an optional base height. Missing targets may come from a
            component's ``qpos_reset``.

    Raises:
        KeyError: If a required field is absent, an unsupported field is
            present, reset targets are incomplete, or a base-height target
            does not match the composed base group.
        TypeError: If a component or reset-pose value has the wrong type.
        ValueError: If component roles, identifiers, schema ownership, reset
            targets, or limits are invalid.
        IndexError: If a nested joint-group motor index is outside its motor
            slots.

    Example:
        spec = RobotSpec(
            {
                "body": {
                    "component_id": "body.example",
                    "motor_slots": 1,
                    "joint_groups": [
                        {
                            "schema_key": "waist",
                            "joint_names": ["waist_joint"],
                            "indices": [0],
                            "kp": [40.0],
                            "kd": [1.0],
                            "qpos_min": [-1.0],
                            "qpos_max": [1.0],
                        }
                    ],
                },
                "end_effectors": [],
                "sensors": {},
                "reset_pose": {
                    "joint_velocity_limit": 0.6,
                    "joint_groups": {"waist": [0.0]},
                },
            }
        )
        assert spec.joint_group_specs[0].component_id == "body.example"
        assert spec.schema_keys("joint_group") == ("waist",)
        assert spec.reset_pose["joint_groups"]["waist"] == (0.0,)
    """

    def __init__(
        self,
        robot_spec_config: dict[str, object],
    ) -> None:
        reject_unknown_fields(
            robot_spec_config,
            frozenset(
                {
                    "body",
                    "end_effectors",
                    "base_group",
                    "kinematics",
                    "sensors",
                    "reset_pose",
                }
            ),
            label="robot specification",
        )

        # Build the three supported component-spec categories.
        body_spec = JointGroupSpec(robot_spec_config["body"])
        robot_label = f"Robot specification {body_spec.component_id!r}"
        if body_spec.is_end_effector:
            raise ValueError(
                f"{robot_label} body must reference a body component."
            )
        end_effector_specs = tuple(
            JointGroupSpec(end_effector_config)
            for end_effector_config in robot_spec_config["end_effectors"]
        )
        for component_spec in end_effector_specs:
            if not component_spec.is_end_effector:
                raise ValueError(
                    f"{robot_label} end-effector component_id must start with "
                    f"'end_effector.', got {component_spec.component_id!r}."
                )
        self.joint_group_specs: tuple[JointGroupSpec, ...] = (
            body_spec,
            *end_effector_specs,
        )

        base_group_config = robot_spec_config.get("base_group")
        self.base_group_spec = None
        if base_group_config is not None:
            self.base_group_spec = BaseGroupSpec(base_group_config)

        sensor_specs = []
        for sensor_kind, sensor_config in robot_spec_config["sensors"].items():
            component_spec = SensorSpec(sensor_config)
            if component_spec.sensor_kind != sensor_kind:
                raise ValueError(
                    f"{robot_label} sensors.{sensor_kind} must reference a "
                    f"{sensor_kind} component, got "
                    f"{component_spec.component_id!r}."
                )
            sensor_specs.append(component_spec)
        self.sensor_specs = tuple(sensor_specs)

        self.validate_component_layout()
        self.dual_arm_kinematics = self.normalize_dual_arm_kinematics(
            robot_spec_config.get("kinematics")
        )
        self.reset_pose = self.normalize_reset_pose(
            robot_spec_config.get("reset_pose")
        )

    def normalize_dual_arm_kinematics(
        self,
        kinematics_config: object,
    ) -> DualArmKinematicsSpec | None:
        if kinematics_config is None:
            return None
        if not isinstance(kinematics_config, Mapping):
            raise TypeError("Robot specification kinematics must be a mapping.")
        reject_unknown_fields(
            kinematics_config,
            frozenset({"dual_arm"}),
            label="Robot specification kinematics",
        )
        if "dual_arm" not in kinematics_config:
            return None

        body_spec = self.joint_group_specs[0]
        missing_arm_groups = {
            "left_arm",
            "right_arm",
        } - set(body_spec.schema_keys)
        if missing_arm_groups:
            raise KeyError(
                "Robot specification kinematics.dual_arm requires body "
                "joint groups "
                f"{tuple(sorted(missing_arm_groups))!r}."
            )
        return DualArmKinematicsSpec(
            kinematics_config["dual_arm"],
            left_arm_joint_names=body_spec.joint_names["left_arm"],
            right_arm_joint_names=body_spec.joint_names["right_arm"],
        )

    # Reset-pose normalization.
    def normalize_reset_pose(
        self,
        reset_pose_config: object,
    ) -> dict[str, object] | None:
        """Normalize the complete reset target for this robot composition.

        Args:
            reset_pose_config: Optional reset-pose mapping. A configured pose
                must define a positive ``joint_velocity_limit`` and targets
                which, together with component ``qpos_reset`` defaults, cover
                every position-controlled joint group. Torque-only groups
                reset to zero effort. ``joint_tolerance`` optionally
                overrides the default 0.05 rad completion tolerance. A
                position-controlled base requires ``base_height``; a
                velocity-controlled lift may omit it to remain stopped.

        Returns:
            ``None`` when no reset pose is configured. Otherwise, a new
            mapping containing floating-point velocity and height values plus
            tuple-valued joint-position targets.

        Raises:
            KeyError: If fields or joint groups are missing or unsupported,
                or base-height presence does not match the composition.
            TypeError: If a mapping, target sequence, or numeric value has the
                wrong type.
            ValueError: If a schema key is invalid, a value is not finite or
                positive, target DOF is incorrect, or an end-effector target
                exceeds position limits.

        Example:
            reset_pose = spec.normalize_reset_pose(
                {
                    "joint_velocity_limit": 0.6,
                    "joint_groups": {"waist": [0.0]},
                }
            )
            assert reset_pose["joint_groups"]["waist"] == (0.0,)
        """
        if reset_pose_config is None:
            return None

        reset_pose_label = (
            "Robot specification "
            f"{self.joint_group_specs[0].component_id!r}.reset_pose"
        )

        # Validate the top-level reset-pose structure.
        if not isinstance(reset_pose_config, Mapping):
            raise TypeError(f"{reset_pose_label} must be a mapping.")
        reject_unknown_fields(
            reset_pose_config,
            frozenset(
                {
                    "joint_velocity_limit",
                    "joint_tolerance",
                    "joint_groups",
                    "base_height",
                }
            ),
            label=reset_pose_label,
        )
        missing_fields = {"joint_velocity_limit", "joint_groups"} - set(
            reset_pose_config
        )
        if missing_fields:
            raise KeyError(
                f"{reset_pose_label} must define "
                f"{tuple(sorted(missing_fields))!r}."
            )

        # Complete explicit targets with component-owned reset positions.
        joint_groups_label = f"{reset_pose_label}.joint_groups"
        joint_group_configs = reset_pose_config["joint_groups"]
        if not isinstance(joint_group_configs, Mapping):
            raise TypeError(f"{joint_groups_label} must be a mapping.")
        joint_group_targets = {}
        for schema_key_value, target_config in joint_group_configs.items():
            schema_key = require_non_empty_str(
                schema_key_value,
                label=f"{joint_groups_label} key",
            )
            joint_group_targets[schema_key] = target_config
        configured_joint_groups = set(joint_group_targets)
        expected_joint_groups = set(self.schema_keys("joint_group"))
        for component_spec in self.joint_group_specs:
            expected_joint_groups.difference_update(component_spec.torque_schema_keys)
        unknown = tuple(
            sorted(configured_joint_groups - expected_joint_groups)
        )
        for component_spec in self.joint_group_specs:
            for schema_key, target in component_spec.qpos_reset.items():
                joint_group_targets.setdefault(schema_key, target)
        missing = tuple(
            sorted(expected_joint_groups - set(joint_group_targets))
        )
        if missing or unknown:
            raise KeyError(
                f"{joint_groups_label} must cover the complete "
                "robot; "
                f"missing={missing!r}, unknown={unknown!r}."
            )

        # Normalize targets, then validate DOF and all joint position limits.
        joint_position_targets = {}
        for schema_key, target_config in joint_group_targets.items():
            target_label = f"{joint_groups_label}.{schema_key}"
            if not isinstance(target_config, list | tuple):
                raise TypeError(f"{target_label} must be a numeric list.")
            target = tuple(
                require_finite_float(
                    value,
                    label=f"{target_label}[{index}]",
                )
                for index, value in enumerate(target_config)
            )
            component_spec = self.joint_group_spec(schema_key)
            expected_dof = len(component_spec.motor_indices[schema_key])
            if len(target) != expected_dof:
                raise ValueError(
                    f"{target_label} expected {expected_dof} values, "
                    f"got {len(target)}."
                )
            qpos_limits = component_spec.qpos_limits[schema_key]
            outside_limits = tuple(
                index
                for index, (value, (lower, upper)) in enumerate(
                    zip(target, qpos_limits)
                )
                if not lower <= value <= upper
            )
            if outside_limits:
                raise ValueError(
                    f"{target_label} values at indices "
                    f"{outside_limits!r} are outside qpos_limits."
                )
            joint_position_targets[schema_key] = target

        # Match base-height presence and limits to the base-group component.
        base_height_config = reset_pose_config.get("base_height")
        base_height_label = f"{reset_pose_label}.base_height"
        if self.base_group_spec is None:
            if base_height_config is not None:
                raise KeyError(
                    f"{base_height_label} is configured but the "
                    "robot has no base group."
                )
            base_height = None
        elif base_height_config is None:
            if self.base_group_spec.height_control_mode == "position":
                raise KeyError(f"{base_height_label} must be configured.")
            base_height = None
        else:
            base_height = require_finite_float(
                base_height_config,
                label=base_height_label,
            )
            lower, upper = self.base_group_spec.height_limits
            if not lower <= base_height <= upper:
                raise ValueError(
                    f"{base_height_label} must be within [{lower}, {upper}]."
                )

        # Normalize the joint motion limit used during reset.
        joint_velocity_limit = require_positive_float(
            reset_pose_config["joint_velocity_limit"],
            label=f"{reset_pose_label}.joint_velocity_limit",
        )
        joint_tolerance = require_positive_float(
            reset_pose_config.get("joint_tolerance", 0.05),
            label=f"{reset_pose_label}.joint_tolerance",
        )
        return {
            "joint_velocity_limit": joint_velocity_limit,
            "joint_tolerance": joint_tolerance,
            "joint_groups": joint_position_targets,
            "base_height": base_height,
        }

    # Composition validation.
    def validate_component_layout(self) -> None:
        """Validate component IDs and joint-group schema-key ownership.

        Raises:
            ValueError: If component IDs are duplicated or more than one
                joint-group component owns the same schema key.

        Example:
            spec.validate_component_layout()
        """
        robot_label = (
            f"Robot specification {self.joint_group_specs[0].component_id!r}"
        )

        # Every component ID must identify one composed component.
        component_ids = [
            component_spec.component_id
            for component_spec in self.joint_group_specs
        ]
        if self.base_group_spec is not None:
            component_ids.append(self.base_group_spec.component_id)
        component_ids.extend(
            component_spec.component_id
            for component_spec in self.sensor_specs
        )
        duplicate_component_ids = tuple(
            sorted(
                component_id
                for component_id, count in Counter(component_ids).items()
                if count > 1
            )
        )
        if duplicate_component_ids:
            raise ValueError(
                f"{robot_label} repeats component_id "
                f"{duplicate_component_ids!r}."
            )

        # Every joint-group schema key must belong to one component.
        joint_group_schema_keys = [
            schema_key
            for component_spec in self.joint_group_specs
            for schema_key in component_spec.schema_keys
        ]
        joint_group_schema_key_counts = Counter(joint_group_schema_keys)
        duplicate_joint_group_schema_keys = tuple(
            sorted(
                schema_key
                for schema_key, count in joint_group_schema_key_counts.items()
                if count > 1
            )
        )
        if duplicate_joint_group_schema_keys:
            raise ValueError(
                f"{robot_label} has duplicate joint group schema keys "
                f"{duplicate_joint_group_schema_keys!r}."
            )

    # Component and schema queries.
    def joint_group_spec(self, schema_key: str) -> JointGroupSpec:
        """Return the joint-group component that owns one schema key.

        Args:
            schema_key: Body or end-effector joint-group schema key.

        Returns:
            The owning ``JointGroupSpec``.

        Raises:
            KeyError: If no composed joint-group component declares the key.

        Example:
            body_spec = spec.joint_group_spec("waist")
            assert body_spec.component_id == "body.example"
        """
        for component_spec in self.joint_group_specs:
            if schema_key in component_spec.schema_keys:
                return component_spec
        raise KeyError(
            "Robot specification "
            f"{self.joint_group_specs[0].component_id!r} does not "
            f"declare joint group {schema_key!r}."
        )

    def sensor_spec(self, sensor_kind: str) -> SensorSpec | None:
        """Return the optional sensor component of one kind.

        Args:
            sensor_kind: Sensor kind such as ``imu`` or ``color_camera``.

        Returns:
            The matching ``SensorSpec``, or ``None`` when that sensor kind is
            not part of the composition.

        Example:
            assert spec.sensor_spec("imu") is None
        """
        return next(
            (
                component_spec
                for component_spec in self.sensor_specs
                if component_spec.sensor_kind == sensor_kind
            ),
            None,
        )

    def schema_keys(self, schema_section: str) -> tuple[str, ...]:
        """Return schema keys exposed by one composition section.

        Args:
            schema_section: ``joint_group`` for all joint groups,
                ``end_effector`` for the end-effector subset, ``base_group``,
                or a sensor kind such as ``imu`` or ``color_camera``.

        Returns:
            Declared schema keys in composition order, or an empty tuple when
            the optional section is not configured.

        Example:
            assert spec.schema_keys("joint_group") == ("waist",)
            assert spec.schema_keys("end_effector") == ()
        """
        if schema_section == "joint_group":
            return tuple(
                schema_key
                for component_spec in self.joint_group_specs
                for schema_key in component_spec.schema_keys
            )
        if schema_section == "end_effector":
            return tuple(
                schema_key
                for component_spec in self.joint_group_specs[1:]
                for schema_key in component_spec.schema_keys
            )
        if schema_section == "base_group":
            return (
                ()
                if self.base_group_spec is None
                else self.base_group_spec.schema_keys
            )
        sensor_spec = self.sensor_spec(schema_section)
        return () if sensor_spec is None else sensor_spec.schema_keys
