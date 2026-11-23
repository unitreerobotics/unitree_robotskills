from typing import Annotated, Any, Optional

import msgspec

from .structs import SchemaEntry, require_optional_lengths


# Sensor stream payloads: large data produced by sensors and dehydrated to files.


class SensorStreamDataStruct(
    SchemaEntry,
):
    raw: Annotated[
        Any,
        msgspec.Meta(
            description="Raw sensor stream payload held in memory before dehydration."
        ),
    ] = None
    format: Annotated[
        Optional[str],
        msgspec.Meta(description="Sensor stream encoding or storage format."),
    ] = None
    ref: Annotated[
        Optional[str],
        msgspec.Meta(
            description="Reference path to dehydrated sensor stream data on disk."
        ),
    ] = None

    def storage_name(self) -> str:
        return self.schema_key

    def dehydrated(self, ref: str | None) -> "SensorStreamDataStruct":
        return msgspec.structs.replace(self, raw=None, format=None, ref=ref)


class ImageData(
    SensorStreamDataStruct,
):
    list_fields = frozenset({"shape"})

    shape: Annotated[
        Optional[list[int]],
        msgspec.Meta(description="Image shape as [height, width]."),
    ] = None
    is_padded: Annotated[
        bool,
        msgspec.Meta(
            description="Whether this sample reuses the last valid image."
        ),
    ] = False

    def __post_init__(self) -> None:
        super().__post_init__()
        require_optional_lengths(self, ("shape", self.shape, 2))


# Sensor state payloads: small structured values stored directly in JSON.


class IMUData(
    SchemaEntry,
):
    list_fields = frozenset(
        {
            "linear_acceleration",
            "angular_velocity",
            "orientation_quat_wxyz",
        }
    )

    linear_acceleration: Annotated[
        Optional[list[float]],
        msgspec.Meta(
            description="Linear acceleration as [ax, ay, az].",
            extra={"unit": "m/s²"},
        ),
    ] = None
    angular_velocity: Annotated[
        Optional[list[float]],
        msgspec.Meta(
            description="Angular velocity as [wx, wy, wz].",
            extra={"unit": "rad/s"},
        ),
    ] = None
    orientation_quat_wxyz: Annotated[
        Optional[list[float]],
        msgspec.Meta(description="Orientation quaternion as [qw, qx, qy, qz]."),
    ] = None

    def __post_init__(self) -> None:
        super().__post_init__()
        require_optional_lengths(
            self,
            ("linear_acceleration", self.linear_acceleration, 3),
            ("angular_velocity", self.angular_velocity, 3),
            ("orientation_quat_wxyz", self.orientation_quat_wxyz, 4),
        )
