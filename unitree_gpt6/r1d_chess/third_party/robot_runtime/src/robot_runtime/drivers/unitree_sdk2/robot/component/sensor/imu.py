"""Unitree HG IMU message mapping.

``IMUInterface`` converts a raw HG ``IMUState_`` message into the
driver-neutral ``IMUState`` used by robot runtime. It does not depend on a specific
transport envelope: a body ``RobotChannel`` may pass ``LowState_.imu_state``,
while a standalone IMU ``RobotChannel`` may pass a directly received
``IMUState_`` message.
"""

import numpy as np
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import IMUState_ as HGIMUState

from robot_runtime.interfaces.robot.component import (
    ComponentInterface,
    ComponentStates,
)
from robot_runtime.interfaces.robot.state.sensor.imu import IMUState


class IMUInterface(ComponentInterface):
    """Map one Unitree HG IMU message to a schema-keyed state.

    Accelerometer, gyroscope, and quaternion values are copied into detached
    ``float32`` arrays so the returned state does not alias a DDS message
    buffer. Quaternion order follows the SDK's ``[w, x, y, z]`` convention.

    Args:
        component_id: Stable ID of the mapped IMU component.
        schema_key: IMU schema key exposed by the owning robot channel.

    Attributes:
        schema_key: Schema key used in the returned component-state mapping.

    Example:
        interface = IMUInterface(
            component_id="sensor.imu.default",
            schema_key="pelvis",
        )
        states = interface.read_states_from_msgs((imu_msg,))
        pelvis_imu = states["pelvis"]

        # A body robot channel passes the same payload extracted from LowState_.
        states = interface.read_states_from_msgs(
            (lowstate_msg.imu_state,),
        )
    """

    def __init__(self, *, component_id: str, schema_key: str) -> None:
        super().__init__(component_id=component_id)
        self.schema_key = schema_key

    def read_states_from_msgs(
        self,
        state_msgs: tuple[HGIMUState],
    ) -> ComponentStates:
        """Convert one raw HG IMU message into a driver-neutral state."""
        (imu_msg,) = state_msgs
        return {
            self.schema_key: IMUState(
                linear_acceleration=np.asarray(
                    imu_msg.accelerometer,
                    dtype=np.float32,
                ).copy(),
                angular_velocity=np.asarray(
                    imu_msg.gyroscope,
                    dtype=np.float32,
                ).copy(),
                orientation_quat_wxyz=np.asarray(
                    imu_msg.quaternion,
                    dtype=np.float32,
                ).copy(),
            ),
        }
