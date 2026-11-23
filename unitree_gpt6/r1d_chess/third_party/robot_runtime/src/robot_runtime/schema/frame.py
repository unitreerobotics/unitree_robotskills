from typing import Annotated, Any, Literal, Optional

import msgspec
import numpy as np

from .base_group import BaseGroupData
from .joint_group import JointGroupData
from .pose_82d import Pose82dData
from .sensor import (
    IMUData,
    ImageData,
)
from .structs import SchemaStruct


# Per-frame recording payloads.

class ObservationData(
    SchemaStruct,
):
    list_fields = frozenset({
        "joint_groups",
        "base_group",
        "color",
        "imu",
    })

    # Robot state.
    joint_groups: Optional[list[JointGroupData]] = None
    base_group: Optional[list[BaseGroupData]] = None

    # Sensor streams.
    color: Optional[list[ImageData]] = None

    # Sensor states.
    imu: Optional[list[IMUData]] = None

    # Task context.
    instruction: Annotated[
        Optional[str],
        msgspec.Meta(description="Frame-level instruction text."),
    ] = None

    def joint_group_data(self, schema_key: str) -> JointGroupData | None:
        for joint_group in self.joint_groups or ():
            if joint_group.schema_key == schema_key:
                return joint_group
        return None

    def joint_group_qpos(self, schema_key: str) -> np.ndarray:
        joint_group = self.joint_group_data(schema_key)
        if joint_group is None or joint_group.qpos is None:
            raise RuntimeError(f"Observation is missing {schema_key!r} qpos.")
        qpos = np.asarray(joint_group.qpos, dtype=np.float32).reshape(-1)
        if qpos.size == 0:
            raise RuntimeError(f"Observation {schema_key!r} qpos is empty.")
        return qpos.copy()

    def base_group_data(self, schema_key: str) -> BaseGroupData | None:
        for base_group in self.base_group or ():
            if base_group.schema_key == schema_key:
                return base_group
        return None

    def merge(self, patch: "ObservationData | None") -> "ObservationData":
        if patch is None:
            return self

        for field_name in (
            "joint_groups",
            "base_group",
            "color",
            "imu",
        ):
            patch_items = getattr(patch, field_name)
            if not patch_items:
                continue

            base_items = getattr(self, field_name)
            if not base_items:
                self.update(**{field_name: list(patch_items)})
                continue

            merged_items = list(base_items)
            positions_by_schema_key = {
                item.schema_key: position
                for position, item in enumerate(merged_items)
            }
            for patch_item in patch_items:
                position = positions_by_schema_key.get(patch_item.schema_key)
                if position is None:
                    positions_by_schema_key[patch_item.schema_key] = len(merged_items)
                    merged_items.append(patch_item)
                    continue

                base_item = merged_items[position]
                if type(base_item) is not type(patch_item):
                    raise TypeError(
                        f"Cannot merge {type(patch_item).__name__} "
                        f"{patch_item.schema_key!r} into "
                        f"{type(base_item).__name__}."
                    )
                fields = {
                    field: getattr(patch_item, field)
                    for field in patch_item.__struct_fields__
                    if field != "schema_key"
                    and getattr(patch_item, field) is not None
                }
                if fields:
                    base_item.update(**fields)
            self.update(**{field_name: merged_items})

        if patch.instruction is not None:
            self.update(instruction=patch.instruction)
        return self


class ActionData(
    SchemaStruct,
):
    list_fields = frozenset({"joint_groups", "base_group", "pose_82d"})

    joint_groups: Optional[list[JointGroupData]] = None
    base_group: Optional[list[BaseGroupData]] = None
    pose_82d: Optional[list[Pose82dData]] = None
    hybrid_mode: Annotated[
        Literal[0, 1] | None,
        msgspec.Meta(description=(
            "Hybrid selection: 0 decoupled, 1 full-body; None for other modes."
        )),
    ] = None

    def __post_init__(self) -> None:
        if self.hybrid_mode is not None and (
            type(self.hybrid_mode) is not int or self.hybrid_mode not in (0, 1)
        ):
            raise ValueError("hybrid_mode must be 0, 1, or None.")


class UniFrame(
    SchemaStruct,
):
    id: Annotated[
        Optional[int],
        msgspec.Meta(description="Frame index."),
    ] = None

    task: Annotated[
        Optional[str],
        msgspec.Meta(description="Natural-language task for this frame."),
    ] = None

    # Agent input and output.
    observation: ObservationData = msgspec.field(default_factory=ObservationData)
    action: ActionData = msgspec.field(default_factory=ActionData)

    # Episode signals.
    reward: Annotated[
        Optional[float],
        msgspec.Meta(description="Environment reward."),
    ] = None
    terminated: Annotated[
        Optional[bool],
        msgspec.Meta(description="Episode termination flag."),
    ] = None
    truncated: Annotated[
        Optional[bool],
        msgspec.Meta(description="Episode truncation flag."),
    ] = None

    # Runtime extras.
    simulation_state: Annotated[
        Optional[dict[str, Any]],
        msgspec.Meta(description="Simulation state for recovery."),
    ] = None
    extras: Annotated[
        Optional[dict[str, Any]],
        msgspec.Meta(description="Additional data."),
    ] = None
