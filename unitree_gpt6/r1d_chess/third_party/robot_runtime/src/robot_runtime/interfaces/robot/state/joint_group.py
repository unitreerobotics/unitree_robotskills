"""Driver-neutral state values for a named joint group.

``ComponentInterface`` implementations expose these values through
schema-keyed ``ComponentStates`` mappings owned by a ``RobotChannel``. Each
schema key identifies the joint group represented by a value.
"""

import numpy as np


class JointGroupState:
    """Observed state for an ordered joint group.

    Every array has shape ``(num_joints,)`` and is retained without copying.
    Position and velocity units are radians/meters and rad/s or m/s according
    to joint type; effort uses N m for revolute joints or N for prismatic
    joints. Temperatures are in degrees Celsius.

    Args:
        num_joints: Expected number of values in every provided array.
        qpos: Observed joint positions in schema order.
        qvel: Observed joint velocities in schema order.
        qtau: Observed joint efforts in schema order.
        casing_temp: Optional motor casing temperatures in schema order.
        winding_temp: Optional motor winding temperatures in schema order.

    Attributes:
        qpos: Stored joint positions.
        qvel: Stored joint velocities.
        qtau: Stored joint efforts.
        casing_temp: Stored optional casing temperatures.
        winding_temp: Stored optional winding temperatures.

    Raises:
        ValueError: A provided array does not have shape ``(num_joints,)``.

    Example:
        state = JointGroupState(
            num_joints=2,
            qpos=np.zeros(2, dtype=np.float32),
            qvel=np.zeros(2, dtype=np.float32),
            qtau=np.zeros(2, dtype=np.float32),
        )
        component_states = {"left_arm": state}
    """

    def __init__(
        self,
        *,
        num_joints: int,
        qpos: np.ndarray,
        qvel: np.ndarray,
        qtau: np.ndarray,
        casing_temp: np.ndarray | None = None,
        winding_temp: np.ndarray | None = None,
    ) -> None:
        expected_shape = (num_joints,)
        for field_name, value in (
            ("qpos", qpos),
            ("qvel", qvel),
            ("qtau", qtau),
            ("casing_temp", casing_temp),
            ("winding_temp", winding_temp),
        ):
            if value is not None and value.shape != expected_shape:
                raise ValueError(
                    f"JointGroupState.{field_name} expected shape "
                    f"{expected_shape}, got {value.shape}."
                )

        self.qpos = qpos
        self.qvel = qvel
        self.qtau = qtau
        self.casing_temp = casing_temp
        self.winding_temp = winding_temp
