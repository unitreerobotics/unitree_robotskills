"""Inspire RH56DFTP joint-group mappings.

``InspireRH56DFTPInterface`` converts Inspire hand feedback and commands
between SDK-specific integer positions and driver-neutral joint positions.
Importing this module does not require ``inspire_sdkpy``; constructing the
interface does, so other Unitree components remain usable without that SDK.
"""

from collections.abc import Mapping

import logging_mp
import numpy as np

try:
    import inspire_sdkpy.inspire_hand_defaut as inspire_hand_default
    from inspire_sdkpy import inspire_dds
except ModuleNotFoundError as exc:
    inspire_dds = None
    inspire_hand_default = None
    INSPIRE_SDK_IMPORT_ERROR = exc

from robot_runtime.interfaces.robot.component import (
    ComponentCmds,
    ComponentInterface,
    ComponentStates,
)
from robot_runtime.interfaces.robot.cmd.joint_group import JointGroupCmd
from robot_runtime.interfaces.robot.state.joint_group import JointGroupState


logger_mp = logging_mp.getLogger(__name__)


class InspireRH56DFTPInterface(ComponentInterface):
    """Map Inspire RH56DFTP states and position commands.

    ``qpos_limits`` defines physical joint ranges. ``scale`` defines the raw
    command range for each schema key, while ``invert`` selects its direction.
    State velocity and effort are reported as zero because this protocol only
    exposes measured positions through the fields used here.

    Args:
        component_id: Stable ID of the mapped Inspire hand component.
        motor_slots: Total number of motors in the hand message.
        motor_indices: Local motor indices for every joint-group schema key.
        qpos_limits: Per-joint physical position limits by schema key.
        invert: Whether each schema key reverses normalized motor direction.
        scale: Raw integer position range for each schema key.

    Attributes:
        motor_slots: Total local motor count.
        motor_indices: Schema-keyed local motor mappings.
        qpos_limits: Schema-keyed physical position limits.
        invert: Schema-keyed direction reversal flags.
        scale: Schema-keyed raw position scales.

    Raises:
        ModuleNotFoundError: ``inspire_sdkpy`` is not installed.

    Example:
        interface = InspireRH56DFTPInterface(
            component_id="end_effector.inspire.rh56dftp.left",
            motor_slots=6,
            motor_indices={"left_ee": (5, 4, 3, 2, 1, 0)},
            qpos_limits={
                "left_ee": (
                    (-0.1, 1.3),
                    (0.0, 0.5),
                    (0.0, 1.7),
                    (0.0, 1.7),
                    (0.0, 1.7),
                    (0.0, 1.7),
                )
            },
            invert={"left_ee": True},
            scale={"left_ee": 1000.0},
        )
        cmd_msg = interface.CMD_MSG_FACTORY()
        interface.write_cmds_to_msgs(
            (cmd_msg,),
            {
                "left_ee": JointGroupCmd(
                    num_joints=6,
                    qpos=np.zeros(6, dtype=np.float32),
                )
            },
        )
    """

    HAND_CMD_MODE = 0b0001
    CMD_MSG_TYPE = None if inspire_dds is None else inspire_dds.inspire_hand_ctrl
    STATE_MSG_TYPE = None if inspire_dds is None else inspire_dds.inspire_hand_state
    CMD_MSG_FACTORY = (
        None
        if inspire_hand_default is None
        else staticmethod(inspire_hand_default.get_inspire_hand_ctrl)
    )

    def __init__(
        self,
        *,
        component_id: str,
        motor_slots: int,
        motor_indices: Mapping[str, tuple[int, ...]],
        qpos_limits: Mapping[str, tuple[tuple[float, float], ...]],
        invert: Mapping[str, bool],
        scale: Mapping[str, float],
    ) -> None:
        if inspire_dds is None or inspire_hand_default is None:
            raise ModuleNotFoundError(
                "inspire_sdkpy is required to use "
                "unitree_sdk2.inspire_rh56dftp."
            ) from INSPIRE_SDK_IMPORT_ERROR

        super().__init__(component_id=component_id)
        self.motor_slots = motor_slots
        # Local motor indices keyed by schema key.
        self.motor_indices = motor_indices
        self.qpos_limits = qpos_limits
        self.invert = invert
        self.scale = scale

    def read_states_from_msgs(
        self,
        state_msgs: tuple[object],
    ) -> ComponentStates:
        """Decode raw Inspire motor positions into physical joint states."""
        (state_msg,) = state_msgs
        component_states = {}
        for schema_key, motor_indices in self.motor_indices.items():
            qpos = np.empty(len(motor_indices), dtype=np.float32)
            for state_index, (motor_index, (minimum, maximum)) in enumerate(
                zip(
                    motor_indices,
                    self.qpos_limits[schema_key],
                    strict=True,
                )
            ):
                normalized = (
                    state_msg.angle_act[motor_index] / self.scale[schema_key]
                )
                if self.invert[schema_key]:
                    normalized = 1.0 - normalized
                qpos[state_index] = minimum + normalized * (
                    maximum - minimum
                )
            component_states[schema_key] = JointGroupState(
                num_joints=len(motor_indices),
                qpos=qpos,
                qvel=np.zeros(len(motor_indices), dtype=np.float32),
                qtau=np.zeros(len(motor_indices), dtype=np.float32),
            )
        return component_states

    def cmd_qpos(self, schema_key: str, joint_index: int, qpos: float) -> int:
        """Convert one physical joint position to an Inspire integer command.

        Args:
            schema_key: Joint-group schema key selecting limits and scaling.
            joint_index: Joint offset within the schema-defined group.
            qpos: Physical joint position to convert.

        Returns:
            Clipped integer position in the configured Inspire command range.
        """
        minimum, maximum = self.qpos_limits[schema_key][joint_index]
        normalized = (float(qpos) - minimum) / (maximum - minimum)
        if self.invert[schema_key]:
            normalized = 1.0 - normalized
        return int(
            min(max(float(normalized), 0.0), 1.0) * self.scale[schema_key]
        )

    def write_cmds_to_msgs(
        self,
        cmd_msgs: tuple[object],
        component_cmds: ComponentCmds,
    ) -> None:
        """Encode physical joint positions into an Inspire control message."""
        (cmd_msg,) = cmd_msgs
        received_schema_keys = tuple(component_cmds)
        expected_schema_keys = tuple(self.motor_indices)
        if set(received_schema_keys) != set(expected_schema_keys):
            raise RuntimeError(
                f"{type(self).__name__} {self.component_id!r} requires "
                f"joint groups {expected_schema_keys!r}, got "
                f"{received_schema_keys!r}."
            )

        # FTP mode 1 makes angle_set interpreted as angle-position cmds.
        cmd_msg.mode = self.HAND_CMD_MODE
        for schema_key, motor_indices in self.motor_indices.items():
            cmd = component_cmds[schema_key]
            if not isinstance(cmd, JointGroupCmd):
                raise TypeError(
                    f"{type(self).__name__} requires JointGroupCmd values."
                )
            qpos_limits = np.asarray(
                self.qpos_limits[schema_key],
                dtype=np.float64,
            )
            clipped_qpos = np.clip(
                cmd.qpos,
                qpos_limits[:, 0],
                qpos_limits[:, 1],
            )
            if np.any(cmd.qpos != clipped_qpos):
                logger_mp.warning(
                    "[%s] %s qpos command clipped: requested=%s "
                    "clipped=%s limits=%s",
                    self.component_id,
                    schema_key,
                    cmd.qpos.tolist(),
                    clipped_qpos.tolist(),
                    qpos_limits.tolist(),
                )
            for joint_index, (motor_index, qpos) in enumerate(
                zip(motor_indices, clipped_qpos, strict=True)
            ):
                cmd_msg.angle_set[motor_index] = self.cmd_qpos(
                    schema_key,
                    joint_index,
                    qpos,
                )
