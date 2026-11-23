"""Validate robot base-group component configuration.

Base-group specs declare one globally unique component and one base-motion
schema. Robot specs reference these component specs through ``base_group``.

``BaseGroup`` is the provider-neutral abstraction, analogous to ``JointGroup``.
``base_lift`` and ``pseudo_base_lift`` are schema-resource names rather than
component identities or alternative names for that abstraction.

Example component files:
    ``configs/robots/components/base_groups/g1_d_base.yaml``
    ``configs/robots/components/base_groups/r1_d_base.yaml``
    ``configs/robots/components/base_groups/g1_edu_pseudo_base.yaml``

Example base/lift YAML:
    component_id: base_group.g1_d_base
    schema_key: base_lift
    height_limits: [0.0, 0.4]
    height_control_mode: velocity
"""

import math

from robot_runtime.configs.common import (
    reject_unknown_fields,
    require_choice,
    require_float_range,
    require_non_empty_str,
)
from robot_runtime.schema.base_group import BaseGroupData


class BaseGroupSpec:
    """Validate and expose one base-group component configuration.

    Args:
        component_config: Resolved base-group component mapping containing
            its identity, schema key, supported height interval, and command
            mode.

    Attributes:
        component_id: Globally unique component-specification identifier.
        schema_keys: Schema keys exposed by this spec.
        height_limits: Inclusive supported height interval.
        height_control_mode: Whether the driver accepts height position or
            height velocity commands.

    Raises:
        KeyError: If a required field is absent or an unsupported field is
            present.
        TypeError: If a field has the wrong type.
        ValueError: If height limits or the schema key are invalid.

    Example:
        spec = BaseGroupSpec(
            {
                "component_id": "base_group.example",
                "schema_key": "base_lift",
                "height_limits": [0.0, 0.4],
                "height_control_mode": "velocity",
            }
        )
        assert spec.height_limits == (0.0, 0.4)
        assert spec.build_schema("base_lift").schema_key == "base_lift"
    """

    def __init__(self, component_config: dict[str, object]) -> None:
        reject_unknown_fields(
            component_config,
            frozenset(
                {
                    "component_id",
                    "schema_key",
                    "height_limits",
                    "height_control_mode",
                }
            ),
            label="base_group",
        )
        self.component_id = require_non_empty_str(
            component_config["component_id"],
            label="base_group.component_id",
        )
        schema_key = require_non_empty_str(
            component_config["schema_key"],
            label="base_group.schema_key",
        )
        self.schema_keys = (schema_key,)
        self.height_limits = require_float_range(
            component_config["height_limits"],
            label="base_group.height_limits",
        )
        self.height_control_mode = require_choice(
            component_config["height_control_mode"],
            choices=frozenset({"position", "velocity"}),
            label="base_group.height_control_mode",
        )

    def command_values(
        self, action: BaseGroupData, *, planar_only: bool = False,
    ) -> dict[str, float]:
        """Validate the actual component's command fields at data boundaries."""
        body = self.component_id == "base_group.g1_edu_pseudo_base"
        prefix = "body" if body else "base"
        required = (
            f"{prefix}_linear_vel_x", f"{prefix}_linear_vel_y",
            f"{prefix}_angular_vel_z",
        )
        if not planar_only:
            required += (
                (
                    "body_height", "waist_roll_qpos",
                    "waist_pitch_qpos", "waist_yaw_qpos",
                )
                if body else ("lift_linear_vel_z",)
            )
        values = {
            name: getattr(action, name)
            for name in action.__struct_fields__
            if name != "schema_key" and getattr(action, name) is not None
        }
        missing = set(required) - values.keys()
        unsupported = values.keys() - set(required)
        if missing or unsupported:
            raise ValueError(
                f"Base group {action.schema_key!r} command fields: "
                f"missing={sorted(missing)}, unsupported={sorted(unsupported)}."
            )
        for name, value in values.items():
            if not math.isfinite(value):
                raise ValueError(f"Base group {name} must be finite.")
        if body and not planar_only:
            lower, upper = self.height_limits
            if not lower <= values["body_height"] <= upper:
                raise ValueError("body_height target is outside component height_limits.")
        return values

    def build_schema(self, schema_key: str) -> BaseGroupData:
        """Build the base-group schema entry for one declared key.

        Args:
            schema_key: Base-group schema key requested by a runtime or
                recording binding.

        Returns:
            A ``BaseGroupData`` entry carrying ``schema_key``.

        Raises:
            KeyError: If this component does not declare ``schema_key``.

        Example:
            data = spec.build_schema("base_lift")
            assert data.schema_key == "base_lift"
        """
        (known_schema_key,) = self.schema_keys
        if schema_key != known_schema_key:
            raise KeyError(
                f"Base-group component {self.component_id!r} does not declare "
                f"schema key {schema_key!r}. Known: {known_schema_key!r}."
            )
        return BaseGroupData(schema_key=schema_key)
