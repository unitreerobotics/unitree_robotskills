"""Validate robot IMU and color-camera component configuration.

Sensor specs declare schema-keyed metadata for IMUs or color cameras. Robot
specs reference them through the corresponding key in ``sensors``.

Example component files:
    ``configs/robots/components/sensors/imu/default.yaml``
    ``configs/robots/components/sensors/color_camera/teleimager.yaml``

Example IMU YAML:
    component_id: sensor.imu.default
    imus:
      - schema_key: pelvis
"""

from robot_runtime.configs.common import (
    normalize_schema_resources,
    reject_unknown_fields,
    require_non_empty_str,
)
from robot_runtime.schema.sensor import IMUData, ImageData


class SensorSpec:
    """Validate and expose one sensor component configuration.

    The component ID selects the supported sensor kind and therefore its
    resource collection and schema type.

    Args:
        component_config: Resolved IMU or color-camera component mapping. The
            component ID selects the resource collection and schema type.

    Attributes:
        component_id: Globally unique component-specification identifier.
        sensor_kind: ``imu`` or ``color_camera``.
        sensor_configs: Sensor resources keyed by schema key.
        schema_keys: Schema keys exposed by this spec.

    Raises:
        KeyError: If a required field is missing or unsupported.
        TypeError: If a collection or resource has the wrong type.
        ValueError: If the component kind, resource, or image shape is invalid.

    Example:
        spec = SensorSpec(
            {
                "component_id": "sensor.imu.example",
                "imus": [{"schema_key": "pelvis"}],
            }
        )
        assert spec.schema_keys == ("pelvis",)
        assert spec.build_schema("pelvis").schema_key == "pelvis"
    """

    def __init__(self, component_config: dict[str, object]) -> None:
        self.component_id = require_non_empty_str(
            component_config["component_id"],
            label="sensor.component_id",
        )
        component_label = f"Sensor component {self.component_id!r}"
        if self.component_id.startswith("sensor.imu."):
            self.sensor_kind = "imu"
            collection_key = "imus"
        elif self.component_id.startswith("sensor.color_camera."):
            self.sensor_kind = "color_camera"
            collection_key = "color_cameras"
        else:
            raise ValueError(
                "Sensor component_id must start with 'sensor.imu.' or "
                f"'sensor.color_camera.', got {self.component_id!r}."
            )

        reject_unknown_fields(
            component_config,
            frozenset({"component_id", collection_key}),
            label=component_label,
        )
        sensor_configs = normalize_schema_resources(
            component_config[collection_key],
            label=f"{component_label}.{collection_key}",
        )
        if not sensor_configs:
            raise ValueError(
                f"{component_label}.{collection_key} must not be empty."
            )

        for schema_key, sensor_config in sensor_configs.items():
            sensor_label = f"{component_label}.{collection_key}.{schema_key}"
            self.validate_sensor_config(sensor_config, label=sensor_label)

        self.sensor_configs = sensor_configs
        self.schema_keys = tuple(sensor_configs)

    def validate_sensor_config(
        self,
        sensor_config: dict[str, object],
        *,
        label: str,
    ) -> None:
        """Validate one schema-keyed sensor resource.

        Args:
            sensor_config: IMU or color-camera resource mapping selected by
                this component's ``sensor_kind``.
            label: Fully qualified configuration label used in errors.

        Raises:
            KeyError: If a color camera omits ``format`` or ``shape``, or a
                resource declares an unsupported field.
            TypeError: If a format or image-shape value has the wrong type.
            ValueError: If a format is empty, an image shape is not two
                positive dimensions, or ``sensor_kind`` is unsupported.

        Example:
            spec.validate_sensor_config(
                {"schema_key": "pelvis"},
                label="sensor.imu.example.imus.pelvis",
            )
        """
        if self.sensor_kind == "imu":
            reject_unknown_fields(
                sensor_config,
                frozenset({"schema_key"}),
                label=label,
            )
        elif self.sensor_kind == "color_camera":
            reject_unknown_fields(
                sensor_config,
                frozenset({"schema_key", "format", "shape"}),
                label=label,
            )
            try:
                image_format = sensor_config["format"]
                shape = sensor_config["shape"]
            except KeyError as exc:
                raise KeyError(
                    f"{label} must define format and shape."
                ) from exc
            require_non_empty_str(
                image_format,
                label=f"{label}.format",
            )
            if not isinstance(shape, list):
                raise TypeError(f"{label}.shape must be a list.")
            if len(shape) != 2:
                raise ValueError(
                    f"{label}.shape must contain height and width."
                )
            if any(type(dimension) is not int for dimension in shape):
                raise TypeError(f"{label}.shape must contain ints.")
            if any(dimension <= 0 for dimension in shape):
                raise ValueError(
                    f"{label}.shape dimensions must be positive."
                )
        else:
            raise ValueError(f"Unsupported sensor kind {self.sensor_kind!r}.")

    def build_schema(self, schema_key: str) -> IMUData | ImageData:
        """Build one IMU or color-image schema entry.

        Args:
            schema_key: Sensor schema key declared by this component.

        Returns:
            ``IMUData`` for an IMU component, otherwise ``ImageData`` with
            configured format and image shape.

        Raises:
            KeyError: If this component does not declare ``schema_key``.

        Example:
            data = spec.build_schema("pelvis")
            assert isinstance(data, IMUData)
        """
        try:
            sensor_config = self.sensor_configs[schema_key]
        except KeyError as exc:
            known = ", ".join(self.schema_keys)
            raise KeyError(
                f"Sensor component {self.component_id!r} does not declare "
                f"schema key {schema_key!r}. Known: {known}"
            ) from exc
        if self.sensor_kind == "imu":
            return IMUData(schema_key=schema_key)
        return ImageData(
            schema_key=schema_key,
            format=sensor_config["format"],
            shape=sensor_config["shape"],
        )
