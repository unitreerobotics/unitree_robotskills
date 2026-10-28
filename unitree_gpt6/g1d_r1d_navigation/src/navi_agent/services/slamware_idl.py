"""CycloneDDS IDL for the SLAMWARE lidar topic (not shipped by unitree_sdk2py).

Python dataclasses matching the SLAMWARE ROS SDK message layouts. This module must NOT
use ``from __future__ import annotations``: cyclonedds imports the dotted prefix of string
annotations, so ``types.uint8`` would resolve to the stdlib ``types`` module.

LaserScan_ is verified on the real robot at rt/slamware_ros_sdk_server_node/scan (360 beams
at 1 deg). Importable only where cyclonedds + unitree_sdk2py are installed; services/lidar.py
imports it lazily.
"""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types
from unitree_sdk2py.idl.std_msgs.msg.dds_ import Header_


@dataclass
@annotate.final
@annotate.autoid("sequential")
class LaserScan_(idl.IdlStruct, typename="sensor_msgs.msg.dds_.LaserScan_"):
    header: Header_
    angle_min: types.float32
    angle_max: types.float32
    angle_increment: types.float32
    time_increment: types.float32
    scan_time: types.float32
    range_min: types.float32
    range_max: types.float32
    ranges: types.sequence[types.float32]
    intensities: types.sequence[types.float32]


@dataclass
@annotate.final
@annotate.autoid("sequential")
class CancelActionRequest_(idl.IdlStruct,
                           typename="slamware_ros_sdk.msg.dds_.CancelActionRequest_"):
    structure_needs_at_least_one_member: types.uint8


# Matches the SLAMWARE ROS SDK RobotBasicState message layout (checked against a live
# ROS2 publisher). These types are subscriptions only (no lifecycle writers).
@dataclass
@annotate.final
@annotate.autoid("sequential")
class RobotBasicState_(idl.IdlStruct, typename="slamware_ros_sdk.msg.dds_.RobotBasicState_"):
    is_map_building_enabled: bool
    is_localization_enabled: bool
    localization_quality: types.int32
    board_temperature: types.int32
    battery_percentage: types.int32
    is_dc_in: bool
    is_charging: bool


from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Quaternion_, Vector3_  # noqa: E402
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import MapMetaData_  # noqa: E402


@dataclass
@annotate.final
@annotate.autoid("sequential")
class OccupancyGrid_(idl.IdlStruct, typename="nav_msgs.msg.dds_.OccupancyGrid_"):
    header: Header_
    info: MapMetaData_
    data: types.sequence[types.int8]  # SDK's uint8 declaration loses unknown=-1.


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Transform_(idl.IdlStruct, typename="geometry_msgs.msg.dds_.Transform_"):
    translation: Vector3_
    rotation: Quaternion_


@dataclass
@annotate.final
@annotate.autoid("sequential")
class TransformStamped_(idl.IdlStruct, typename="geometry_msgs.msg.dds_.TransformStamped_"):
    header: Header_
    child_frame_id: str
    transform: Transform_


@dataclass
@annotate.final
@annotate.autoid("sequential")
class TFMessage_(idl.IdlStruct, typename="tf2_msgs.msg.dds_.TFMessage_"):
    transforms: types.sequence[TransformStamped_]
