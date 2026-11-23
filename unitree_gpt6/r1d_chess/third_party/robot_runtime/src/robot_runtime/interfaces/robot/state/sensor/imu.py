"""Driver-neutral state values for inertial-measurement components.

Robot drivers construct these values directly or through a
``ComponentInterface``. A ``RobotChannel`` exposes them inside
``ComponentStates``,
keyed first by ``component_id`` and then by schema key.
"""

import numpy as np


class IMUState:
    """Linear, angular, and optional orientation measurements from one IMU.

    Arrays use the sensor frame selected by the driver and are retained without
    copying.

    Args:
        linear_acceleration: Acceleration ``[x, y, z]`` in m/s², shape ``(3,)``.
        angular_velocity: Angular velocity ``[x, y, z]`` in rad/s, shape
            ``(3,)``.
        orientation_quat_wxyz: Optional orientation quaternion
            ``[w, x, y, z]``, shape ``(4,)``.

    Attributes:
        linear_acceleration: Stored linear acceleration.
        angular_velocity: Stored angular velocity.
        orientation_quat_wxyz: Stored optional orientation quaternion.

    Raises:
        ValueError: A provided array does not have its required shape.

    Example:
        state = IMUState(
            linear_acceleration=np.zeros(3, dtype=np.float32),
            angular_velocity=np.zeros(3, dtype=np.float32),
            orientation_quat_wxyz=np.array(
                [1.0, 0.0, 0.0, 0.0],
                dtype=np.float32,
            ),
        )
        component_states = {"pelvis_imu": state}
    """

    def __init__(
        self,
        *,
        linear_acceleration: np.ndarray,
        angular_velocity: np.ndarray,
        orientation_quat_wxyz: np.ndarray | None = None,
    ) -> None:
        for field_name, value, expected_shape in (
            (
                "linear_acceleration",
                linear_acceleration,
                (3,),
            ),
            (
                "angular_velocity",
                angular_velocity,
                (3,),
            ),
        ):
            if value.shape != expected_shape:
                raise ValueError(
                    f"IMUState.{field_name} expected shape "
                    f"{expected_shape}, got {value.shape}."
                )
        if (
            orientation_quat_wxyz is not None
            and orientation_quat_wxyz.shape != (4,)
        ):
            raise ValueError(
                "IMUState.orientation_quat_wxyz expected shape (4,), "
                f"got {orientation_quat_wxyz.shape}."
            )

        self.linear_acceleration = linear_acceleration
        self.angular_velocity = angular_velocity
        self.orientation_quat_wxyz = orientation_quat_wxyz
