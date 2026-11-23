"""R1-D Base commands and measured velocity/column-height feedback.

Velocity commands and feedback retain the protocol's normalized [-1, 1]
units. Column height is converted from millimetres to metres, relative to
the homed zero (5 mm above the mechanical lower limit).
"""

import logging_mp
import numpy as np
from unitree_sdk2py.idl.default import geometry_msgs_msg_dds__Twist_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_, Twist_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import Error_

from robot_runtime.interfaces.robot.component import (
    ComponentCmds,
    ComponentInterface,
    ComponentStates,
)
from robot_runtime.interfaces.robot.cmd.base_group import BaseGroupCmd
from robot_runtime.interfaces.robot.state.base_group import BaseGroupState


logger_mp = logging_mp.getLogger(__name__)


class R1DBaseInterface(ComponentInterface):
    """Map R1-D Base ``cmd_vel`` and its three feedback topics.

    Linear x velocity, height velocity, and angular z velocity are clipped to
    ``[-1, 1]`` before being written to ``Twist_``. Height is unavailable
    while the column is not homed; a homed height of zero remains valid.

    Args:
        component_id: Stable ID of the mapped base component.
        schema_key: Base-group schema key accepted for commands.

    Attributes:
        schema_key: Schema key used to select the command value.

    Example:
        interface = R1DBaseInterface(
            component_id="base_group.r1_d_base",
            schema_key="base_lift",
        )
        cmd_msg = interface.CMD_MSG_FACTORY()
        interface.write_cmds_to_msgs(
            (cmd_msg,),
            {
                "base_lift": BaseGroupCmd(
                    base_linear_vel_x=0.2,
                    base_linear_vel_y=0.0,
                    base_angular_vel_z=0.1,
                    lift_linear_vel_z=0.0,
                )
            },
        )
    """

    CMD_MSG_TYPE = Twist_
    CMD_MSG_FACTORY = staticmethod(geometry_msgs_msg_dds__Twist_)
    MOVE_STATE_MSG_TYPE = Twist_
    HEIGHT_STATE_MSG_TYPE = Point32_
    HEIGHT_ALARM_MSG_TYPE = Error_
    COMMAND_LIMITS = np.asarray(((-1.0, 1.0),) * 3, dtype=np.float64)

    def __init__(self, *, component_id: str, schema_key: str) -> None:
        super().__init__(component_id=component_id)
        self.schema_key = schema_key

    def read_states_from_msgs(
        self,
        state_msgs: tuple[Twist_, Point32_, Error_],
    ) -> ComponentStates:
        """Decode motor feedback without integrating or echoing commands."""
        move_state, height_state, height_alarm = state_msgs
        height_valid = (
            height_alarm.source == 200
            and not (height_alarm.state & 1)
            and np.isfinite(height_state.z)
        )
        return {
            self.schema_key: BaseGroupState(
                base_linear_vel_x=float(move_state.linear.x),
                base_linear_vel_y=0.0,
                base_angular_vel_z=float(move_state.angular.z),
                lift_height=(
                    float(height_state.z) / 1000.0 if height_valid else None
                ),
                lift_linear_vel_z=float(move_state.linear.z),
            ),
        }

    def write_cmds_to_msgs(
        self,
        cmd_msgs: tuple[Twist_],
        component_cmds: ComponentCmds,
    ) -> None:
        """Encode one clipped base-group command into a ``Twist_`` message."""
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

        (cmd_msg,) = cmd_msgs
        requested = np.asarray(
            (cmd.base_linear_vel_x, cmd.lift_linear_vel_z, cmd.base_angular_vel_z),
            dtype=np.float64,
        )
        clipped = np.clip(
            requested,
            self.COMMAND_LIMITS[:, 0],
            self.COMMAND_LIMITS[:, 1],
        )
        if np.any(requested != clipped):
            logger_mp.warning(
                "[%s] Base command clipped: requested=%s clipped=%s "
                "limits=%s",
                self.component_id,
                requested.tolist(),
                clipped.tolist(),
                self.COMMAND_LIMITS.tolist(),
            )
        cmd_msg.linear.x = float(clipped[0])
        cmd_msg.linear.z = float(clipped[1])
        cmd_msg.angular.z = float(clipped[2])
