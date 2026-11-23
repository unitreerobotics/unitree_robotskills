"""Driver-neutral command values for a named joint group."""

import numpy as np


class JointGroupCmd:
    """Command for an ordered joint group.

    Every provided array has shape ``(num_joints,)`` and is retained without
    copying. Values follow the joint order defined by the schema.

    Args:
        num_joints: Expected number of values in every provided array.
        qpos: Optional target positions, in radians or meters.
        qtau: Effort targets in N m or N; feedforward when qpos is present.

    Attributes:
        qpos: Stored optional joint position targets.
        qtau: Stored optional joint effort targets.

    Raises:
        ValueError: A provided array has the wrong shape or non-finite values.

    Example:
        cmd = JointGroupCmd(
            num_joints=2,
            qpos=np.zeros(2, dtype=np.float32),
        )
        component_cmds = {"left_arm": cmd}
    """

    def __init__(
        self,
        *,
        num_joints: int,
        qpos: np.ndarray | None = None,
        qtau: np.ndarray | None = None,
    ) -> None:
        if qpos is None and qtau is None:
            raise ValueError("JointGroupCmd requires qpos or qtau.")
        expected_shape = (num_joints,)
        for field_name, value in (
            ("qpos", qpos),
            ("qtau", qtau),
        ):
            if value is not None and value.shape != expected_shape:
                raise ValueError(
                    f"JointGroupCmd.{field_name} expected shape "
                    f"{expected_shape}, got {value.shape}."
                )
            if value is not None and not np.all(np.isfinite(value)):
                raise ValueError(
                    f"JointGroupCmd.{field_name} values must be finite."
                )

        self.qpos = qpos
        self.qtau = qtau
