"""Driver-neutral robot component values and driver-message mappings.

A concrete ``RobotChannel`` owns transport and composes ``ComponentInterface``
objects. Each interface maps one physical component's schema-keyed states and
commands to one or more driver messages. The dependency flow is
``RobotChannel -> ComponentInterface -> driver messages``; this module does
not depend on ``RobotChannel``.

``ComponentCmds`` and ``ComponentStates`` are mappings for one component,
keyed by schema key. A command mapping is complete before encoding;
``RobotChannel`` adds ``component_id`` as the outer mapping key.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Mapping

from robot_runtime.interfaces.robot.cmd.base_group import BaseGroupCmd
from robot_runtime.interfaces.robot.cmd.joint_group import JointGroupCmd
from robot_runtime.interfaces.robot.cmd.pose_82d import Pose82dCmd
from robot_runtime.interfaces.robot.state.base_group import BaseGroupState
from robot_runtime.interfaces.robot.state.joint_group import JointGroupState
from robot_runtime.interfaces.robot.state.sensor.color_camera import (
    ColorCameraState,
)
from robot_runtime.interfaces.robot.state.sensor.imu import IMUState


ComponentCmdValue = (
    JointGroupCmd
    | BaseGroupCmd
    | Pose82dCmd
)

ComponentStateValue = (
    JointGroupState
    | BaseGroupState
    | IMUState
    | ColorCameraState
)

# Commands and states for one physical component, keyed by schema key.
ComponentCmds = Mapping[str, ComponentCmdValue]
ComponentStates = Mapping[str, ComponentStateValue]


class ComponentInterface(ABC):
    """Stateless driver-message mapping for one physical component.

    Component interfaces convert schema-keyed driver-neutral values without
    owning transport, lifecycle, synchronization, or state storage.

    Args:
        component_id: Stable identifier of the mapped physical component.

    Attributes:
        component_id: Outer key used by robot channel state and command mappings.

    Example:
        interface.write_cmds_to_msgs(
            (low_cmd_msg,),
            {"left_arm": left_arm_cmd, "right_arm": right_arm_cmd},
        )
    """

    def __init__(self, *, component_id: str) -> None:
        self.component_id = component_id

    @abstractmethod
    def read_states_from_msgs(
        self,
        state_msgs: tuple[object, ...],
    ) -> ComponentStates:
        """Decode schema-keyed states from the owning robot channel's messages.

        Args:
            state_msgs: Driver state messages received by the owning robot channel.

        Returns:
            States for this physical component, keyed by schema key.
        """
        raise NotImplementedError

    def write_cmds_to_msgs(
        self,
        cmd_msgs: tuple[object, ...],
        component_cmds: ComponentCmds,
    ) -> None:
        """Encode this component's commands into driver messages.

        Args:
            cmd_msgs: Mutable driver command messages owned by the robot channel.
            component_cmds: Complete commands for this component, keyed by
                schema key.

        Raises:
            RuntimeError: The component interface is read-only.
        """
        raise RuntimeError(f"{type(self).__name__} is not writable.")
