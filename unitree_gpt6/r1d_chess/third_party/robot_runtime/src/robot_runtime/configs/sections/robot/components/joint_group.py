"""Validate robot body and end-effector joint-group configuration.

Joint-group specs declare static joint metadata for robot body and
end-effector components. Position-controlled groups require explicit limits;
``end_effector.*`` component IDs additionally enable command-conversion fields.

Example component files:
    ``configs/robots/components/joint_groups/bodies/g1_d_flagship.yaml``
    ``configs/robots/components/joint_groups/end_effectors/unitree/dex3_1_left.yaml``

Example YAML:
    component_id: end_effector.unitree.dex3_1.left
    motor_slots: 7

    joint_groups:
      - schema_key: left_ee
        joint_names: [
          "left_hand_thumb_0_joint",
        ]
        indices: [0]
        kp: [1.5]
        kd: [0.2]
        qpos_min: [-1.04719755]
        qpos_max: [1.04719755]
"""

from collections import Counter

import numpy as np

from robot_runtime.configs.common import (
    normalize_schema_resources,
    reject_unknown_fields,
    require_finite_float,
    require_float_range,
    require_non_empty_str,
    require_positive_float,
    require_positive_int,
)
from robot_runtime.schema.joint_group import JointGroupData


JOINT_GROUP_FIELDS = frozenset(
    {"schema_key", "joint_names", "indices", "kp", "kd", "qpos_min", "qpos_max"}
)
END_EFFECTOR_JOINT_GROUP_FIELDS = JOINT_GROUP_FIELDS | frozenset(
    {
        "invert",
        "scale",
        "qpos_reset",
        "qpos_open",
        "qpos_closed",
    }
)
TORQUE_JOINT_GROUP_FIELDS = frozenset({
    "schema_key", "joint_names", "indices", "qtau_open", "qtau_closed",
    "qtau_limits", "qtau_rate_limit",
})


class JointGroupSpec:
    """Validate and expose a joint-group component configuration.

    Args:
        component_config: Resolved body or end-effector component mapping
            containing motor layout, joint-group resources, and optional
            temperature thresholds.

    Attributes:
        component_id: Globally unique component-specification identifier.
        is_end_effector: Whether this component represents an end effector.
        motor_slots: Number of motor slots exposed by the component.
        temperature_thresholds: Trigger and release thresholds keyed by
            temperature field.
        joint_group_configs: Joint-group configurations keyed by schema key.
        joint_names: Joint-name tuples keyed by schema key.
        motor_indices: Local motor-index tuples keyed by schema key.
        schema_keys: Schema keys exposed by this spec.
        torque_schema_keys: Groups accepting qtau-only commands; empty for
            position-controlled components.
        qtau_open: Torque gripper opening efforts, keyed by schema key.
        qtau_closed: Torque gripper closing efforts, keyed by schema key.
        qtau_limits: Per-motor torque bounds, keyed by schema key.
        qtau_rate_limit: Maximum torque change in Nm/s, keyed by schema key.
        qpos_limits: Position bounds paired from qpos_min/qpos_max at startup;
            empty for torque-only groups.
        qpos_reset: Optional component-owned reset positions keyed by schema
            key. This is empty for body components.
        qpos_open: Optional gripper-open positions keyed by schema key. This
            attribute exists only for end-effector components.
        qpos_closed: Optional gripper-closed positions keyed by schema key.
            This attribute exists only for end-effector components.
        invert: End-effector command inversion keyed by schema key. This
            attribute exists only for end-effector components.
        scale: End-effector command scale keyed by schema key. This attribute
            exists only for end-effector components.

    Raises:
        KeyError: If a required field is absent.
        TypeError: If a field has the wrong type.
        ValueError: If names, gains, indices, group order, or end-effector
            fields are invalid.
        IndexError: If a motor index is outside ``motor_slots``.

    Example:
        spec = JointGroupSpec(
            {
                "component_id": "end_effector.example.gripper.left",
                "motor_slots": 1,
                "joint_groups": [
                    {
                        "schema_key": "left_ee",
                        "joint_names": ["left_gripper_joint"],
                        "indices": [0],
                        "kp": [1.5],
                        "kd": [0.2],
                        "qpos_min": [0.0],
                        "qpos_max": [1.0],
                        "qpos_open": [0.0],
                        "qpos_closed": [1.0],
                    }
                ],
            }
        )
        assert spec.motor_indices["left_ee"] == (0,)
        assert spec.qpos_limits["left_ee"] == ((0.0, 1.0),)
    """

    def __init__(
        self,
        component_config: dict[str, object],
    ) -> None:
        reject_unknown_fields(
            component_config,
            frozenset(
                {
                    "component_id",
                    "joint_groups",
                    "motor_slots",
                    "temperature_thresholds",
                }
            ),
            label="joint_group",
        )

        self.component_id = require_non_empty_str(
            component_config["component_id"],
            label="joint_group.component_id",
        )
        if self.component_id.startswith("body."):
            self.is_end_effector = False
        elif self.component_id.startswith("end_effector."):
            self.is_end_effector = True
        else:
            raise ValueError(
                "Joint-group component_id must start with 'body.' or "
                f"'end_effector.', got {self.component_id!r}."
            )

        self.motor_slots = require_positive_int(
            component_config["motor_slots"],
            label="joint_group.motor_slots",
        )
        thresholds_config = component_config.get(
            "temperature_thresholds",
            {},
        )
        if not isinstance(thresholds_config, dict):
            raise TypeError(
                f"Joint-group component {self.component_id!r}."
                "temperature_thresholds must be a mapping."
            )
        self.temperature_thresholds = self.normalize_temperature_thresholds(
            thresholds_config
        )

        self.joint_group_configs = self.normalize_joint_groups(
            component_config["joint_groups"]
        )
        self.schema_keys = tuple(self.joint_group_configs)
        self.joint_names = {
            schema_key: tuple(config["joint_names"])
            for schema_key, config in self.joint_group_configs.items()
        }
        self.motor_indices = {
            schema_key: tuple(config["indices"])
            for schema_key, config in self.joint_group_configs.items()
        }
        self.torque_schema_keys = tuple(
            key for key, config in self.joint_group_configs.items()
            if "qtau_open" in config
        )
        self.qtau_open = {
            key: tuple(self.joint_group_configs[key]["qtau_open"])
            for key in self.torque_schema_keys
        }
        self.qtau_closed = {
            key: tuple(self.joint_group_configs[key]["qtau_closed"])
            for key in self.torque_schema_keys
        }
        self.qtau_limits = {
            key: tuple(self.joint_group_configs[key]["qtau_limits"])
            for key in self.torque_schema_keys
        }
        self.qtau_rate_limit = {
            key: self.joint_group_configs[key]["qtau_rate_limit"]
            for key in self.torque_schema_keys
        }
        self.qpos_limits = self.normalize_qpos_limits(
            self.joint_group_configs
        )
        self.qpos_reset = self.normalize_qpos_reset(
            self.joint_group_configs
        )
        if self.is_end_effector:
            (
                self.qpos_open,
                self.qpos_closed,
            ) = self.normalize_gripper_qpos(self.joint_group_configs)
            (
                self.invert,
                self.scale,
            ) = self.normalize_end_effector_fields(
                self.joint_group_configs
            )
        self.validate_joint_group_layout()

    def normalize_temperature_thresholds(
        self,
        config: dict[str, object],
    ) -> dict[str, dict[str, float]]:
        """Normalize component-wide temperature protection thresholds.

        Args:
            config: Threshold mapping keyed by ``casing_temp`` or
                ``winding_temp``. Each value must define ``trigger`` and
                ``release``.

        Returns:
            Finite floating-point trigger and release values keyed by
            temperature field.

        Raises:
            KeyError: If a field is unsupported or a threshold pair is
                incomplete.
            TypeError: If a threshold pair is not a mapping or a value is not
                numeric.
            ValueError: If a value is not finite or ``release`` is not lower
                than ``trigger``.

        Example:
            thresholds = spec.normalize_temperature_thresholds(
                {"casing_temp": {"trigger": 85.0, "release": 80.0}}
            )
            assert thresholds["casing_temp"]["trigger"] == 85.0
        """
        thresholds_label = (
            f"Joint-group component {self.component_id!r}."
            "temperature_thresholds"
        )

        # Validate supported temperature fields.
        reject_unknown_fields(
            config,
            frozenset({"casing_temp", "winding_temp"}),
            label=thresholds_label,
        )

        # Normalize each trigger and release pair.
        temperature_thresholds: dict[str, dict[str, float]] = {}
        for field_name, thresholds_config in config.items():
            field_label = f"{thresholds_label}.{field_name}"
            if not isinstance(thresholds_config, dict):
                raise TypeError(f"{field_label} must be a mapping.")
            reject_unknown_fields(
                thresholds_config,
                frozenset({"trigger", "release"}),
                label=field_label,
            )
            missing_fields = {"trigger", "release"} - set(thresholds_config)
            if missing_fields:
                raise KeyError(
                    f"{field_label} must define "
                    f"{tuple(sorted(missing_fields))!r}."
                )
            trigger = require_finite_float(
                thresholds_config["trigger"],
                label=f"{field_label}.trigger",
            )
            release = require_finite_float(
                thresholds_config["release"],
                label=f"{field_label}.release",
            )
            if release >= trigger:
                raise ValueError(
                    f"{field_label}.release must be lower than trigger."
                )
            temperature_thresholds[field_name] = {
                "trigger": trigger,
                "release": release,
            }
        return temperature_thresholds

    def normalize_joint_groups(
        self,
        joint_groups_config: object,
    ) -> dict[str, dict[str, object]]:
        """Normalize schema-keyed joint metadata and controller gains.

        Args:
            joint_groups_config: List of joint-group resource mappings. Each
                resource declares a schema key, joint names, motor indices,
                and aligned ``kp`` and ``kd`` values.

        Returns:
            Normalized joint-group mappings keyed by schema key.

        Raises:
            KeyError: If a required resource field is missing or unsupported.
            TypeError: If a resource, list field, name, index, or gain has the
                wrong type.
            ValueError: If the collection is empty, values are invalid, or
                aligned fields have different lengths.

        Example:
            groups = spec.normalize_joint_groups(
                [
                    {
                        "schema_key": "left_ee",
                        "joint_names": ["left_gripper_joint"],
                        "indices": [0],
                        "kp": [1.5],
                        "kd": [0.2],
                        "qpos_min": [0.0],
                        "qpos_max": [1.0],
                    }
                ]
            )
            assert tuple(groups) == ("left_ee",)
        """
        component_label = f"Joint-group component {self.component_id!r}"
        allowed_fields = (
            END_EFFECTOR_JOINT_GROUP_FIELDS
            if self.is_end_effector
            else JOINT_GROUP_FIELDS
        )
        torque_only = (
            self.component_id.rsplit(".", 1)[0] == "end_effector.unitree.dex2_5"
        )
        if torque_only:
            allowed_fields = TORQUE_JOINT_GROUP_FIELDS

        # Index resources and require at least one joint group.
        joint_group_configs = normalize_schema_resources(
            joint_groups_config,
            label=f"{component_label}.joint_groups",
        )
        if not joint_group_configs:
            raise ValueError(
                f"{component_label}.joint_groups must not be empty."
            )

        for schema_key, joint_group_config in joint_group_configs.items():
            group_label = f"{component_label}.joint_groups.{schema_key}"

            # Validate required fields and list containers.
            reject_unknown_fields(
                joint_group_config,
                allowed_fields,
                label=group_label,
            )
            if torque_only:
                # dex2_5 has no position-control variant or configurable gains.
                indices = joint_group_config["indices"]
                for field in ("qtau_open", "qtau_closed", "qtau_limits"):
                    values = joint_group_config[field]
                    if not isinstance(values, list) or len(values) != len(indices):
                        raise ValueError(
                            f"{group_label}.{field} must match its joint count."
                        )
                limits = [
                    require_float_range(
                        value, label=f"{group_label}.qtau_limits[{i}]",
                    )
                    for i, value in enumerate(joint_group_config["qtau_limits"])
                ]
                if any(not -0.3 <= lo <= 0.0 <= hi <= 0.3 for lo, hi in limits):
                    raise ValueError(
                        f"{group_label}.qtau_limits must include zero "
                        "within [-0.3, 0.3] Nm."
                    )
                joint_group_config["qtau_limits"] = limits
                for field in ("qtau_open", "qtau_closed"):
                    values = [
                        require_finite_float(
                            value, label=f"{group_label}.{field}[{i}]",
                        )
                        for i, value in enumerate(joint_group_config[field])
                    ]
                    if any(
                        not lo <= value <= hi
                        for value, (lo, hi) in zip(values, limits)
                    ):
                        raise ValueError(f"{group_label}.{field} exceeds qtau_limits.")
                    joint_group_config[field] = values
                joint_group_config["qtau_rate_limit"] = require_positive_float(
                    joint_group_config["qtau_rate_limit"],
                    label=f"{group_label}.qtau_rate_limit",
                )
                # Protocol constants retained in schema metadata, never YAML inputs.
                joint_group_config["kp"] = [0.0] * len(indices)
                joint_group_config["kd"] = [0.0] * len(indices)
            try:
                joint_names = joint_group_config["joint_names"]
                indices = joint_group_config["indices"]
                kp = joint_group_config["kp"]
                kd = joint_group_config["kd"]
            except KeyError as exc:
                raise KeyError(
                    f"{group_label} "
                    "must define joint_names, indices, kp, and kd."
                ) from exc

            for field_name, value in (
                ("joint_names", joint_names),
                ("indices", indices),
                ("kp", kp),
                ("kd", kd),
            ):
                if not isinstance(value, list):
                    raise TypeError(
                        f"{group_label}.{field_name} must be a list."
                    )

            # Validate joint names, motor indices, and aligned lengths.
            if not joint_names:
                raise ValueError(
                    f"{group_label}.joint_names must not be empty."
                )
            if any(
                not isinstance(joint_name, str) or not joint_name
                for joint_name in joint_names
            ):
                raise TypeError(
                    f"{group_label}.joint_names must contain non-empty strings."
                )
            if any(type(motor_index) is not int for motor_index in indices):
                raise TypeError(f"{group_label}.indices must contain ints.")
            if any(motor_index < 0 for motor_index in indices):
                raise ValueError(
                    f"{group_label}.indices must be non-negative."
                )
            lengths = (len(joint_names), len(indices), len(kp), len(kd))
            if len(set(lengths)) != 1:
                raise ValueError(
                    f"{group_label} "
                    "joint_names, indices, kp, and kd must have equal length."
                )

            # Normalize position and velocity gains.
            for field_name, gains in (("kp", kp), ("kd", kd)):
                normalized_gains = []
                for index, gain in enumerate(gains):
                    gain = require_finite_float(
                        gain,
                        label=f"{group_label}.{field_name}[{index}]",
                    )
                    if gain < 0.0:
                        raise ValueError(
                            f"{group_label}.{field_name}[{index}] must be "
                            "non-negative."
                        )
                    normalized_gains.append(gain)
                joint_group_config[field_name] = normalized_gains
        return joint_group_configs

    def normalize_qpos_limits(
        self,
        joint_group_configs: dict[str, dict[str, object]],
    ) -> dict[str, tuple[tuple[float, float], ...]]:
        """Validate parallel lower/upper arrays and pair them for consumers."""
        qpos_limits = {}
        for schema_key, joint_group_config in joint_group_configs.items():
            if schema_key in self.torque_schema_keys:
                qpos_limits[schema_key] = ()
                continue
            group_label = (
                f"Joint-group component {self.component_id!r} "
                f"schema key {schema_key!r}"
            )
            bounds = {}
            for field_name in ("qpos_min", "qpos_max"):
                field_label = f"{group_label}.{field_name}"
                if field_name not in joint_group_config:
                    raise KeyError(f"{field_label} must be configured.")
                values = joint_group_config[field_name]
                if not isinstance(values, list | tuple):
                    raise TypeError(f"{field_label} must be a list or tuple.")
                if len(values) != len(joint_group_config["indices"]):
                    raise ValueError(f"{field_label} must match its joint count.")
                bounds[field_name] = tuple(
                    require_finite_float(value, label=f"{field_label}[{index}]")
                    for index, value in enumerate(values)
                )
            limits = tuple(zip(bounds["qpos_min"], bounds["qpos_max"], strict=True))
            for index, (lower, upper) in enumerate(limits):
                if lower >= upper:
                    raise ValueError(
                        f"{group_label}.qpos_min[{index}] must be below "
                        f"qpos_max[{index}]."
                    )
            qpos_limits[schema_key] = limits
        return qpos_limits

    def normalize_qpos_reset(
        self,
        joint_group_configs: dict[str, dict[str, object]],
    ) -> dict[str, tuple[float, ...]]:
        """Normalize optional component-owned reset positions."""
        qpos_reset = {}
        for schema_key, joint_group_config in joint_group_configs.items():
            if "qpos_reset" not in joint_group_config:
                continue
            group_label = (
                f"Joint-group component {self.component_id!r} "
                f"schema key {schema_key!r}"
            )
            qpos_reset_config = joint_group_config["qpos_reset"]
            if not isinstance(qpos_reset_config, list | tuple):
                raise TypeError(
                    f"{group_label}.qpos_reset must be a numeric list."
                )
            target = tuple(
                require_finite_float(
                    value,
                    label=f"{group_label}.qpos_reset[{index}]",
                )
                for index, value in enumerate(qpos_reset_config)
            )
            if len(target) != len(joint_group_config["indices"]):
                raise ValueError(
                    f"{group_label}.qpos_reset must match its joint count."
                )
            limits = self.qpos_limits[schema_key]
            if limits:
                outside_limits = tuple(
                    index
                    for index, (value, (lower, upper)) in enumerate(
                        zip(target, limits)
                    )
                    if not lower <= value <= upper
                )
                if outside_limits:
                    raise ValueError(
                        f"{group_label}.qpos_reset values at indices "
                        f"{outside_limits!r} are outside qpos_limits."
                    )
            qpos_reset[schema_key] = target
        return qpos_reset

    def normalize_gripper_qpos(
        self,
        joint_group_configs: dict[str, dict[str, object]],
    ) -> tuple[
        dict[str, tuple[float, ...]],
        dict[str, tuple[float, ...]],
    ]:
        """Normalize optional open and closed gripper positions."""
        qpos_open = {}
        qpos_closed = {}
        for schema_key, joint_group_config in joint_group_configs.items():
            configured_fields = {
                field_name
                for field_name in ("qpos_open", "qpos_closed")
                if field_name in joint_group_config
            }
            if not configured_fields:
                continue
            if len(configured_fields) != 2:
                raise KeyError(
                    f"End-effector component {self.component_id!r} schema "
                    f"key {schema_key!r} must define qpos_open and "
                    "qpos_closed together."
                )

            group_label = (
                f"End-effector component {self.component_id!r} schema key "
                f"{schema_key!r}"
            )
            targets: dict[str, tuple[float, ...]] = {}
            for field_name in ("qpos_open", "qpos_closed"):
                target_config = joint_group_config[field_name]
                if not isinstance(target_config, list | tuple):
                    raise TypeError(
                        f"{group_label}.{field_name} must be a numeric list."
                    )
                target = tuple(
                    require_finite_float(
                        value,
                        label=f"{group_label}.{field_name}[{index}]",
                    )
                    for index, value in enumerate(target_config)
                )
                if len(target) != len(joint_group_config["indices"]):
                    raise ValueError(
                        f"{group_label}.{field_name} must match its joint "
                        "count."
                    )
                limits = self.qpos_limits[schema_key]
                if limits:
                    outside_limits = tuple(
                        index
                        for index, (value, (lower, upper)) in enumerate(
                            zip(target, limits)
                        )
                        if not lower <= value <= upper
                    )
                    if outside_limits:
                        raise ValueError(
                            f"{group_label}.{field_name} values at indices "
                            f"{outside_limits!r} are outside qpos_limits."
                        )
                targets[field_name] = target

            if targets["qpos_open"] == targets["qpos_closed"]:
                raise ValueError(
                    f"{group_label}.qpos_open and qpos_closed must differ."
                )
            qpos_open[schema_key] = targets["qpos_open"]
            qpos_closed[schema_key] = targets["qpos_closed"]
        return qpos_open, qpos_closed

    def normalize_end_effector_fields(
        self,
        joint_group_configs: dict[str, dict[str, object]],
    ) -> tuple[
        dict[str, bool],
        dict[str, float],
    ]:
        """Normalize command conversion for one hand.

        Args:
            joint_group_configs: Normalized end-effector joint groups keyed
                by schema key.

        Returns:
            Schema-keyed mappings containing inversion flags and positive
            command scales.

        Raises:
            TypeError: If inversion flags have invalid types.
            ValueError: If the component side, schema-key side, or command
                scale is invalid.

        Example:
            invert, scale = spec.normalize_end_effector_fields(
                spec.joint_group_configs
            )
            assert invert["left_ee"] is False
            assert scale["left_ee"] == 1.0
        """
        component_label = f"End-effector component {self.component_id!r}"

        # Identify the configured end-effector side.
        side = self.component_id.rpartition(".")[2]
        if side not in {"left", "right"}:
            raise ValueError(
                f"End-effector component_id {self.component_id!r} must end "
                "in '.left' or '.right'."
            )

        # Require schema keys to belong to the same side.
        required_schema_prefix = f"{side}_"
        invalid_schema_keys = tuple(
            schema_key
            for schema_key in joint_group_configs
            if not schema_key.startswith(required_schema_prefix)
        )
        if invalid_schema_keys:
            raise ValueError(
                f"{component_label} schema keys must start with "
                f"{required_schema_prefix!r}, got "
                f"{invalid_schema_keys!r}."
            )

        # Normalize end-effector fields for each joint group.
        invert = {}
        scale = {}
        for schema_key, joint_group_config in joint_group_configs.items():
            group_label = f"{component_label} schema key {schema_key!r}"

            # Normalize command inversion and scale.
            invert_value = joint_group_config.get("invert", False)
            if type(invert_value) is not bool:
                raise TypeError(
                    f"{group_label} invert must be a boolean."
                )
            invert[schema_key] = invert_value
            scale[schema_key] = require_positive_float(
                joint_group_config.get("scale", 1.0),
                label=f"{group_label}.scale",
            )
        return invert, scale

    def validate_joint_group_layout(self) -> None:
        """Validate names, motor slots, and ordering across joint groups.

        Raises:
            ValueError: If joint names or motor indices are duplicated, or
                joint groups are not ordered by their first motor index.
            IndexError: If a motor index is outside ``motor_slots``.

        Example:
            spec.validate_joint_group_layout()
        """
        component_label = f"Joint-group component {self.component_id!r}"

        # Every schema entry must identify a unique physical joint.
        joint_names = [
            joint_name
            for group_joint_names in self.joint_names.values()
            for joint_name in group_joint_names
        ]
        duplicate_joint_names = tuple(
            sorted(
                joint_name
                for joint_name, count in Counter(joint_names).items()
                if count > 1
            )
        )
        if duplicate_joint_names:
            raise ValueError(
                f"{component_label} contains duplicate joint "
                f"names: {duplicate_joint_names!r}."
            )

        # Every motor slot must exist and belong to only one joint.
        motor_indices = [
            motor_index
            for group_motor_indices in self.motor_indices.values()
            for motor_index in group_motor_indices
        ]
        duplicate_indices = tuple(
            sorted(
                motor_index
                for motor_index, count in Counter(motor_indices).items()
                if count > 1
            )
        )
        if duplicate_indices:
            raise ValueError(
                f"{component_label} contains duplicate motor "
                f"indices: {duplicate_indices!r}."
            )
        invalid_indices = tuple(
            sorted(
                index
                for index in motor_indices
                if index >= self.motor_slots
            )
        )
        if invalid_indices:
            raise IndexError(
                f"{component_label} contains motor indices "
                f"outside [0, {self.motor_slots}): {invalid_indices!r}."
            )

        # Keep schema order aligned with the physical motor layout.
        group_first_indices = [
            min(group_motor_indices)
            for group_motor_indices in self.motor_indices.values()
        ]
        if group_first_indices != sorted(group_first_indices):
            raise ValueError(
                f"{component_label}.joint_groups must follow "
                "ascending motor-index order."
            )

    def build_schema(self, schema_key: str) -> JointGroupData:
        """Build one joint-group schema entry.

        Args:
            schema_key: Joint-group schema key declared by this component.

        Returns:
            A ``JointGroupData`` entry containing joint names and controller
            gains for ``schema_key``.

        Raises:
            KeyError: If this component does not declare ``schema_key``.

        Example:
            data = spec.build_schema("left_ee")
            assert data.joint_names == ["left_gripper_joint"]
        """
        joint_group_config = self.joint_group_configs[schema_key]
        return JointGroupData(
            schema_key=joint_group_config["schema_key"],
            joint_names=list(self.joint_names[schema_key]),
            kp=joint_group_config["kp"],
            kd=joint_group_config["kd"],
        )

    def build_dense_gains(self) -> tuple[np.ndarray, np.ndarray]:
        """Build dense controller-gain arrays indexed by motor slot.

        Returns:
            A pair of ``float32`` arrays containing ``kp`` and ``kd`` for all
            motor slots. Unassigned slots remain zero.

        Example:
            kp, kd = spec.build_dense_gains()
            assert kp.shape == (1,)
            assert kd.shape == (1,)
        """
        kp = np.zeros(self.motor_slots, dtype=np.float32)
        kd = np.zeros(self.motor_slots, dtype=np.float32)
        for schema_key, config in self.joint_group_configs.items():
            motor_indices = list(self.motor_indices[schema_key])
            kp[motor_indices] = config["kp"]
            kd[motor_indices] = config["kd"]
        return kp, kd
