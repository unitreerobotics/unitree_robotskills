"""G1 arm motor mapping and gains retained from unitree_deploy 0.0.5.

See README.md, manifest.json and licenses/ for provenance and notices.
Only the fields consumed by the Gomoku DDS controller are included.
"""

from dataclasses import dataclass
from enum import IntEnum


class G1_29_JointArmIndex(IntEnum):
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristyaw = 21
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28


ARM_JOINT_NAMES = tuple(
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in (
        "shoulder_pitch",
        "shoulder_roll",
        "shoulder_yaw",
        "elbow",
        "wrist_roll",
        "wrist_pitch",
        "wrist_yaw",
    )
)


@dataclass(frozen=True)
class G1ArmConfig:
    kp_high: float = 300.0
    kd_high: float = 3.0
    kp_low: float = 80.0
    kd_low: float = 3.0
    kp_wrist: float = 40.0
    kd_wrist: float = 1.5
