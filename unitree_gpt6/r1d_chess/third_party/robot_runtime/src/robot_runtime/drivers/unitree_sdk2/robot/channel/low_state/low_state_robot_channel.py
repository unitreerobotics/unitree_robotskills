"""Shared Unitree HG LowState channel support.

``LowStateRobotChannel`` fixes the state endpoint to ``rt/lowstate`` and decodes
the shared HG envelope into body, embedded IMU, and optional embedded
component states. Command-protocol subclasses provide their own output topics
and encoding behavior.
"""

from collections.abc import Callable, Iterable, Mapping

from typing_extensions import override
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HGLowState

from robot_runtime.drivers.unitree_sdk2.robot.channel.dds_robot_channel import (
    DDSRobotChannel,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.body.hg_body import (
    HGBodyInterface,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.sensor.imu import (
    IMUInterface,
)
from robot_runtime.interfaces.robot.component import (
    ComponentInterface,
    ComponentStates,
)

LOW_STATE_TOPIC = "rt/lowstate"


class LowStateRobotChannel(DDSRobotChannel):
    """Decode one Unitree HG LowState stream for composed components.

    The body and other embedded interfaces receive the complete LowState
    message. The IMU interface receives its nested ``imu_state`` payload.

    Args:
        body_interface: Interface mapping HG motor slots to body states.
        imu_interface: Interface mapping the embedded HG IMU state.
        cmd_topics: Command topics supplied by the concrete protocol.
        writable_components: Component IDs accepted by command writes.
        embedded_interfaces: Additional interfaces sharing the LowState
            envelope, such as an internally wired end effector.
        additional_interfaces: Interfaces decoded by a protocol subclass from
            additional state topics.
        additional_state_topics: Extra state topics owned by the subclass.
        start_timeout: Seconds to wait for the first LowState message.
        close_timeout: Optional maximum seconds spent closing each DDS handle.

    Attributes:
        state_topic: Fixed Unitree HG LowState topic.
        body_interface: Body interface receiving the complete LowState message.
        imu_interface: IMU interface receiving ``LowState_.imu_state``.

    Example:
        channel = LowStateRobotChannel(
            body_interface=body_interface,
            imu_interface=imu_interface,
            cmd_topics={},
            writable_components=(),
        )
        states = channel.decode_component_states({
            LOW_STATE_TOPIC: low_state_msg,
        })
        body_states = states[body_interface.component_id]
    """

    def __init__(
        self,
        *,
        body_interface: HGBodyInterface,
        imu_interface: IMUInterface,
        cmd_topics: Mapping[str, tuple[type, Callable[[], object]]],
        writable_components: Iterable[str],
        embedded_interfaces: Iterable[ComponentInterface] = (),
        additional_interfaces: Iterable[ComponentInterface] = (),
        additional_state_topics: Mapping[str, type] | None = None,
        start_timeout: float = 10.0,
        close_timeout: float | None = None,
    ) -> None:
        self.state_topic = LOW_STATE_TOPIC
        self.body_interface = body_interface
        self.imu_interface = imu_interface
        embedded_interfaces = tuple(embedded_interfaces)
        self.low_state_interfaces = (body_interface, *embedded_interfaces)
        components = (
            *self.low_state_interfaces,
            imu_interface,
            *additional_interfaces,
        )
        super().__init__(
            state_topics={
                self.state_topic: HGLowState,
                **(additional_state_topics or {}),
            },
            cmd_topics=cmd_topics,
            components={
                component_interface.component_id: component_interface
                for component_interface in components
            },
            writable_components=writable_components,
            start_timeout=start_timeout,
            close_timeout=close_timeout,
        )

    @override
    def decode_component_states(
        self,
        state_msgs: Mapping[str, object],
    ) -> dict[str, ComponentStates]:
        """Decode body, IMU, and embedded states from one LowState message."""
        state_msg = state_msgs[self.state_topic]
        component_states = {
            component_interface.component_id: component_interface.read_states_from_msgs(
                (state_msg,)
            )
            for component_interface in self.low_state_interfaces
        }
        component_states[self.imu_interface.component_id] = (
            self.imu_interface.read_states_from_msgs((state_msg.imu_state,))
        )
        return component_states
