"""G1 EDU virtual base state mapped from whole-body controller topics."""

from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HGLowState

from robot_runtime.interfaces.robot.component import (
    ComponentInterface,
    ComponentStates,
)
from robot_runtime.interfaces.robot.state.base_group import BaseGroupState


class G1EduPseudoBaseInterface(ComponentInterface):
    """Decode body odometry and height; waist feedback belongs to the body joints."""

    def __init__(
        self,
        *,
        component_id: str,
        schema_key: str,
    ) -> None:
        super().__init__(component_id=component_id)
        self.schema_key = schema_key

    def read_states_from_msgs(
        self,
        state_msgs: tuple[HGLowState, SportModeState_],
    ) -> ComponentStates:
        _, odom_state = state_msgs
        return {
            self.schema_key: BaseGroupState(
                body_linear_vel_x=float(odom_state.velocity[0]),
                body_linear_vel_y=float(odom_state.velocity[1]),
                body_angular_vel_z=float(odom_state.yaw_speed),
                body_height=float(odom_state.body_height),
                body_linear_vel_z=float(odom_state.velocity[2]),
            )
        }
