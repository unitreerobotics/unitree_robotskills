"""G1-D Base message mappings for lift height and planar motion.

``G1DBaseInterface`` converts the base component's driver-neutral
``BaseGroupState`` and ``BaseGroupCmd`` values to four Unitree SDK2 DDS
messages. The owning ``RobotChannel`` supplies the messages and owns their topics,
lifecycle, synchronization, and storage.
"""

from unitree_sdk2py.idl.default import (
    geometry_msgs_msg_dds__Point32_,
    geometry_msgs_msg_dds__Twist_,
)
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_, Twist_
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_

from robot_runtime.interfaces.robot.component import (
    ComponentCmds,
    ComponentInterface,
    ComponentStates,
)
from robot_runtime.interfaces.robot.cmd.base_group import BaseGroupCmd
from robot_runtime.interfaces.robot.state.base_group import BaseGroupState


class G1DBaseInterface(ComponentInterface):
    """Map G1-D Base height and planar-motion DDS messages.

    Height state and command use ``Point32_`` messages. Planar-motion state
    uses ``Odometry_`` and commands use ``Twist_``. The schema key identifies
    the single ``BaseGroupState`` or ``BaseGroupCmd`` value represented by the
    four messages.

    Args:
        component_id: Stable ID of the mapped base component.
        schema_key: Base-group schema key exposed by the owning robot channel.

    Attributes:
        schema_key: Schema key used in returned states and accepted commands.

    Example:
        interface = G1DBaseInterface(
            component_id="base_group.g1_d_base",
            schema_key="base_lift",
        )
        states = interface.read_states_from_msgs(
            (height_state_msg, odometry_msg),
        )
        base_state = states["base_lift"]
    """

    HEIGHT_CMD_MSG_TYPE = Point32_
    HEIGHT_STATE_MSG_TYPE = Point32_
    MOVE_CMD_MSG_TYPE = Twist_
    MOVE_STATE_MSG_TYPE = Odometry_
    HEIGHT_CMD_MSG_FACTORY = staticmethod(geometry_msgs_msg_dds__Point32_)
    MOVE_CMD_MSG_FACTORY = staticmethod(geometry_msgs_msg_dds__Twist_)

    def __init__(self, *, component_id: str, schema_key: str) -> None:
        super().__init__(component_id=component_id)
        self.schema_key = schema_key

    def read_states_from_msgs(
        self,
        state_msgs: tuple[Point32_, Odometry_],
    ) -> ComponentStates:
        """Decode height and odometry messages into one base-group state."""
        height_state_msg, move_state_msg = state_msgs
        return {
            self.schema_key: BaseGroupState(
                base_linear_vel_x=float(move_state_msg.twist.twist.linear.x),
                base_linear_vel_y=float(move_state_msg.twist.twist.linear.y),
                base_angular_vel_z=float(move_state_msg.twist.twist.angular.z),
                lift_height=float(height_state_msg.y),
                lift_linear_vel_z=float(height_state_msg.z),
            ),
        }

    def write_cmds_to_msgs(
        self,
        cmd_msgs: tuple[Point32_, Twist_],
        component_cmds: ComponentCmds,
    ) -> None:
        """Encode one base-group command into height-speed and motion messages."""
        received_schema_keys = tuple(component_cmds)
        if received_schema_keys != (self.schema_key,):
            raise RuntimeError(
                f"{type(self).__name__} {self.component_id!r} requires "
                f"base group {self.schema_key!r}, got "
                f"{received_schema_keys!r}."
            )
        cmd = component_cmds[self.schema_key]
        if not isinstance(cmd, BaseGroupCmd):
            raise TypeError(
                f"{type(self).__name__} requires BaseGroupCmd values."
            )

        height_cmd_msg, move_cmd_msg = cmd_msgs
        height_cmd_msg.z = float(cmd.lift_linear_vel_z)
        move_cmd_msg.linear.x = float(cmd.base_linear_vel_x)
        move_cmd_msg.linear.y = float(cmd.base_linear_vel_y)
        move_cmd_msg.angular.z = float(cmd.base_angular_vel_z)
