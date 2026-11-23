from typing import Annotated, Optional

import msgspec

from .structs import SchemaEntry, require_optional_lengths


# Joint group payloads.

class JointGroupData(
    SchemaEntry,
):
    list_fields = frozenset({
        "joint_names",
        "kp",
        "kd",
        "qpos",
        "qvel",
        "qtau",
        "casing_temp",
        "winding_temp",
        "cartesian_pose",
        "cartesian_wrench",
    })

    # Static joint configuration.
    joint_names: Annotated[
        Optional[list[str]],
        msgspec.Meta(description="Corresponding joint names in URDF order."),
    ] = None
    kp: Annotated[
        Optional[list[float]],
        msgspec.Meta(description="Stiffness gain."),
    ] = None
    kd: Annotated[
        Optional[list[float]],
        msgspec.Meta(description="Damping gain."),
    ] = None

    # Per-joint runtime state.
    qpos: Annotated[
        Optional[list[float]],
        msgspec.Meta(description="Joint position.", extra={"unit": "rad"}),
    ] = None
    qvel: Annotated[
        Optional[list[float]],
        msgspec.Meta(description="Joint velocity.", extra={"unit": "rad/s"}),
    ] = None
    qtau: Annotated[
        Optional[list[float]],
        msgspec.Meta(description="Joint torque.", extra={"unit": "N·m"}),
    ] = None
    # Optional motor temperature readings.
    casing_temp: Annotated[
        Optional[list[float]],
        msgspec.Meta(
            description="Per-motor casing temperature.",
            extra={"unit": "°C"},
        ),
    ] = None
    winding_temp: Annotated[
        Optional[list[float]],
        msgspec.Meta(
            description="Per-motor winding temperature.",
            extra={"unit": "°C"},
        ),
    ] = None

    # Group-level Cartesian state.
    cartesian_pose: Annotated[
        Optional[list[float]],
        msgspec.Meta(description="Cartesian pose as [x, y, z, qw, qx, qy, qz]."),
    ] = None
    cartesian_wrench: Annotated[
        Optional[list[float]],
        msgspec.Meta(
            description="Cartesian wrench as [fx, fy, fz, tx, ty, tz].",
            extra={"unit": "N for force, N·m for torque"},
        ),
    ] = None

    def __post_init__(self) -> None:
        super().__post_init__()
        expected_name = None
        expected_length = None

        for field_name, value in (
            ("joint_names", self.joint_names),
            ("kp", self.kp),
            ("kd", self.kd),
            ("qpos", self.qpos),
            ("qvel", self.qvel),
            ("qtau", self.qtau),
            ("casing_temp", self.casing_temp),
            ("winding_temp", self.winding_temp),
        ):
            if value is None:
                continue

            length = len(value)
            if expected_length is None:
                expected_name = field_name
                expected_length = length
            elif length != expected_length:
                raise ValueError(
                    f"{type(self).__name__} per-joint field lengths must match, got "
                    f"{expected_name}={expected_length}, {field_name}={length}."
                )
        require_optional_lengths(
            self,
            ("cartesian_pose", self.cartesian_pose, 7),
            ("cartesian_wrench", self.cartesian_wrench, 6),
        )
