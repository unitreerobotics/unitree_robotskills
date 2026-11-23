"""Driver-neutral command values for the 82-dimensional human pose resource.

``Pose82dCmd`` is carried under the ``pose_82d`` schema key in a component's
``ComponentCmds`` mapping. The owning ``RobotChannel`` and ``ComponentInterface``
encode it into the concrete whole-body-control protocol.
"""

import numpy as np

POSE_82D_SMPL_JOINTS_SHAPE = (24, 3)
POSE_82D_SMPL_ROOT_QUAT_WXYZ_SHAPE = (4,)
POSE_82D_WRIST_QPOS_SHAPE = (6,)


class Pose82dCmd:
    """Human pose command consumed by humanoid whole-body control.

    Input arrays are retained without copying.

    Args:
        smpl_joints: SMPL joint positions in the root frame, shape ``(24, 3)``
            and unit meters.
        smpl_root_quat_wxyz: SMPL root orientation quaternion in ``[w, x, y,
            z]`` order, shape ``(4,)``.
        wrist_qpos: Left then right wrist angles, each ordered as ``[roll,
            pitch, yaw]`` in radians, shape ``(6,)``.

    Attributes:
        smpl_joints: Stored SMPL joint positions.
        smpl_root_quat_wxyz: Stored SMPL root quaternion.
        wrist_qpos: Stored wrist joint positions.

    Raises:
        ValueError: An input array has the wrong shape or non-finite values.

    Example:
        cmd = Pose82dCmd(
            smpl_joints=np.zeros((24, 3), dtype=np.float32),
            smpl_root_quat_wxyz=np.array(
                [1.0, 0.0, 0.0, 0.0],
                dtype=np.float32,
            ),
            wrist_qpos=np.zeros(6, dtype=np.float32),
        )
        component_cmds = {"pose_82d": cmd}
    """

    def __init__(
        self,
        *,
        smpl_joints: np.ndarray,
        smpl_root_quat_wxyz: np.ndarray,
        wrist_qpos: np.ndarray,
    ) -> None:
        for field_name, value, expected_shape in (
            (
                "smpl_joints",
                smpl_joints,
                POSE_82D_SMPL_JOINTS_SHAPE,
            ),
            (
                "smpl_root_quat_wxyz",
                smpl_root_quat_wxyz,
                POSE_82D_SMPL_ROOT_QUAT_WXYZ_SHAPE,
            ),
            (
                "wrist_qpos",
                wrist_qpos,
                POSE_82D_WRIST_QPOS_SHAPE,
            ),
        ):
            if value.shape != expected_shape:
                raise ValueError(
                    f"Pose82dCmd.{field_name} expected shape "
                    f"{expected_shape}, got {value.shape}."
                )
            if not np.all(np.isfinite(value)):
                raise ValueError(
                    f"Pose82dCmd.{field_name} values must be finite."
                )

        self.smpl_joints = smpl_joints
        self.smpl_root_quat_wxyz = smpl_root_quat_wxyz
        self.wrist_qpos = wrist_qpos
