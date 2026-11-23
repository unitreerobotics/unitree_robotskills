"""Driver-owned communication boundaries for robot components.

A concrete ``RobotChannel`` owns transport I/O and lifecycle, and composes
``ComponentInterface`` objects to convert driver messages. The runtime
dependency flow is ``RobotHardware -> RobotChannel ->
ComponentInterface -> driver message``.

Robot channel mappings use ``component_id`` as the outer key and schema key as
the inner key. ``ComponentStates`` and ``ComponentCmds`` describe the inner
mapping for one physical component.
"""

from abc import ABC, abstractmethod
from collections.abc import Iterable, Mapping

from robot_runtime.interfaces.robot.component import (
    ComponentCmds,
    ComponentInterface,
    ComponentStates,
)


class RobotChannel(ABC):
    """Communication boundary for a set of robot components.

    Args:
        components: Component interfaces keyed by component ID.
        writable_components: Component IDs that accept commands.

    Attributes:
        components: ``component_id -> ComponentInterface`` mapping.
        writable_components: Immutable set of writable component IDs.

    Example:
        states = channel.read_component_states()
        left_arm_state = states["body.g1_edu"]["left_arm"]

        channel.write_component_cmds({
            "body.g1_edu": {
                "left_arm": left_arm_cmd,
                "right_arm": right_arm_cmd,
            }
        })
    """

    def __init__(
        self,
        *,
        components: Mapping[str, ComponentInterface],
        writable_components: Iterable[str] = (),
    ) -> None:
        self.components = dict(components)
        self.writable_components = frozenset(writable_components)

    @abstractmethod
    def start(self) -> None:
        """Start the transport resources required by this robot channel."""
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        """Stop the robot channel and release its transport resources."""
        raise NotImplementedError

    @abstractmethod
    def is_connected(self) -> bool:
        """Return whether the robot channel is ready for its supported I/O."""
        raise NotImplementedError

    @abstractmethod
    def read_component_states(self) -> Mapping[str, ComponentStates]:
        """Return the latest states by component ID.

        Returns:
            ``Mapping[component_id, ComponentStates]`` whose inner mappings are
            keyed by schema key.
        """
        raise NotImplementedError

    def write_component_cmds(
        self,
        component_cmds: Mapping[str, ComponentCmds],
    ) -> None:
        """Write commands by component ID.

        Args:
            component_cmds: ``Mapping[component_id, ComponentCmds]`` whose inner
                mappings contain every command schema key required by the
                component.

        Raises:
            RuntimeError: The robot channel does not support command output.
        """
        raise RuntimeError(f"{type(self).__name__} is not writable.")
