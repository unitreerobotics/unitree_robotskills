from typing import Annotated, Optional

import msgspec

from .structs import SchemaEntry, require_optional_lengths


class Pose82dData(
    SchemaEntry,
):
    list_fields = frozenset({
        "smpl_joints",
        "smpl_root_quat_wxyz",
        "wrist_qpos",
    })

    smpl_joints: Annotated[
        Optional[list[list[float]]],
        msgspec.Meta(
            description="SMPL Cartesian joint positions in the root frame.",
            extra={"unit": "m"},
        ),
    ] = None
    smpl_root_quat_wxyz: Annotated[
        Optional[list[float]],
        msgspec.Meta(description="Human SMPL root orientation as [w, x, y, z]."),
    ] = None
    wrist_qpos: Annotated[
        Optional[list[float]],
        msgspec.Meta(
            description=(
                "Robot wrist qpos ordered as left/right [roll, pitch, yaw]."
            ),
            extra={"unit": "rad"},
        ),
    ] = None

    def __post_init__(self) -> None:
        super().__post_init__()
        require_optional_lengths(
            self,
            ("smpl_joints", self.smpl_joints, 24),
            ("smpl_root_quat_wxyz", self.smpl_root_quat_wxyz, 4),
            ("wrist_qpos", self.wrist_qpos, 6),
        )
        if self.smpl_joints is not None:
            require_optional_lengths(
                self,
                *(
                    (f"smpl_joints[{index}]", joint, 3)
                    for index, joint in enumerate(self.smpl_joints)
                ),
            )
