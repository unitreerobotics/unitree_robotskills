from typing import Annotated, Optional

import msgspec

from .structs import SchemaEntry


class BaseGroupData(SchemaEntry):
    """Scalar motion targets or feedback; unavailable fields stay None.

    Body velocities follow the odometry/body convention. Body height is
    distinct from world position z. Waist feedback lives in joint_group.
    """

    body_linear_vel_x: Annotated[
        Optional[float],
        msgspec.Meta(
            description="Body forward velocity.",
            extra={"unit": "m/s"},
        ),
    ] = None
    body_linear_vel_y: Annotated[
        Optional[float],
        msgspec.Meta(
            description="Body lateral velocity.",
            extra={"unit": "m/s"},
        ),
    ] = None
    body_linear_vel_z: Annotated[
        Optional[float],
        msgspec.Meta(
            description="Observed odometry velocity[2]; not a height command.",
            extra={"unit": "m/s"},
        ),
    ] = None
    body_angular_vel_z: Annotated[
        Optional[float],
        msgspec.Meta(
            description="Body yaw velocity.",
            extra={"unit": "rad/s"},
        ),
    ] = None
    body_height: Annotated[
        Optional[float],
        msgspec.Meta(
            description="Controller body-height target or feedback; not world position z.",
            extra={"unit": "m"},
        ),
    ] = None
    base_linear_vel_x: Annotated[
        Optional[float],
        msgspec.Meta(
            description="Physical base forward velocity; native units for R1-D.",
        ),
    ] = None
    base_linear_vel_y: Annotated[
        Optional[float],
        msgspec.Meta(
            description="Physical base lateral velocity; support depends on the driver.",
        ),
    ] = None
    base_angular_vel_z: Annotated[
        Optional[float],
        msgspec.Meta(
            description="Physical base yaw velocity; native units for R1-D.",
        ),
    ] = None
    lift_height: Annotated[
        Optional[float],
        msgspec.Meta(
            description="Reported lift height; reference is defined by the device.",
            extra={"unit": "m"},
        ),
    ] = None
    lift_linear_vel_z: Annotated[
        Optional[float],
        msgspec.Meta(
            description="Lift speed; preserve native DDS units until calibrated.",
        ),
    ] = None
    waist_roll_qpos: Annotated[
        Optional[float],
        msgspec.Meta(
            description="Waist roll joint target; feedback is in joint_group.waist.",
            extra={"unit": "rad"},
        ),
    ] = None
    waist_pitch_qpos: Annotated[
        Optional[float],
        msgspec.Meta(
            description="Waist pitch joint target; feedback is in joint_group.waist.",
            extra={"unit": "rad"},
        ),
    ] = None
    waist_yaw_qpos: Annotated[
        Optional[float],
        msgspec.Meta(
            description="Waist yaw joint target; feedback is in joint_group.waist.",
            extra={"unit": "rad"},
        ),
    ] = None
