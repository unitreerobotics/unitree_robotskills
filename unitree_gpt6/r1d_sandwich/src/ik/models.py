# Vendored from r1-ik r1ik/models.py (5b867e6, 2026-11-19): only the a7d / a7d_cal variants, and the IK URDFs
# are the files r1ik.models.build_ik_urdf generated (copied to robot_devices/assets/r1d, sha256 in SHA256SUMS)
# instead of being regenerated at run time. Their mesh paths point at the vendored assets/r1d/meshes (r1-ik's
# assets_official/r1d_description/meshes, 36 STL), so the collision model needs no r1-ik checkout either.
"""R1-D IK model variants (dual arm + waist + merged lift joint, TCP at the Dex1 finger pads)."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

ASSETS = Path(__file__).resolve().parents[2] / "assets" / "r1d"

ARM_JOINTS_7 = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"]
LIFT_JOINT = "lift_stage1_joint"  # merged: the joint value is the measured total height (m)
WAIST_JOINT = "waist_yaw_joint"


@dataclass(frozen=True)
class Variant:
    name: str
    arm_joints: list[str]
    ee_parent: str  # per-side link name template, "{side}" is replaced
    ee_offset: tuple[float, float, float]  # TCP in the ee_parent frame
    has_lift: bool = False
    modes: dict[str, list[str]] = field(default_factory=dict)

    def arm_joint_names(self, side: str) -> list[str]:
        return [f"{side}_{j}_joint" for j in self.arm_joints]

    @property
    def ee_links(self) -> list[str]:
        return ["left_ee", "right_ee"]

    @property
    def source_urdf(self) -> Path:
        """Official R1-D URDF. Its meshes are in assets/r1d/meshes, so the collision check works."""
        return ASSETS / "r1d.urdf"

    @property
    def urdf_path(self) -> Path:
        return ASSETS / f"{self.name}_ik.urdf"

    def mode_joints(self, mode: str) -> list[str]:
        return self.modes[mode]


def _dual_arm(arm_joints: list[str]) -> list[str]:
    return [f"{s}_{j}_joint" for s in ("left", "right") for j in arm_joints]


_MODES = {
    "dual_arm": _dual_arm(ARM_JOINTS_7),
    "arm_waist": [*_dual_arm(ARM_JOINTS_7), WAIST_JOINT],
    "arm_lift": [*_dual_arm(ARM_JOINTS_7), LIFT_JOINT],
    "wbc": [*_dual_arm(ARM_JOINTS_7), WAIST_JOINT, LIFT_JOINT],
}
# Dex1 base at (0.0485,-0.003,0.02) + finger pad centre 0.12 ahead of it
VARIANTS: dict[str, Variant] = {
    name: Variant(
        name=name,
        arm_joints=ARM_JOINTS_7,
        ee_parent="{side}_wrist_yaw_link",
        ee_offset=(0.169, -0.003, 0.02),
        has_lift=True,
        modes=dict(_MODES),
    )
    for name in ("a7d", "a7d_cal")  # a7d_cal: same kinematics, real-robot-calibrated arm inertials
}


def build_ik_urdf(variant: Variant, force: bool = False) -> Path:
    """The vendored, pre-generated IK URDF (r1-ik generates it; here it is a fixed asset)."""
    del force
    path = variant.urdf_path
    if not path.is_file():
        raise FileNotFoundError(f"vendored IK URDF missing: {path}")
    return path
