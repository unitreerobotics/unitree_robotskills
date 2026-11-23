"""Driver-neutral communication boundary for simulation environments."""

from abc import ABC, abstractmethod
from collections.abc import Mapping
from enum import Enum


class SimulationResetScope(Enum):
    OBJECTS = "objects"
    SCENE = "scene"


class SimulationChannel(ABC):
    """Lifecycle, state, and reset contract for simulation channels."""

    @abstractmethod
    def start(self) -> None:
        """Start the transport resources required by this channel."""
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        """Stop the channel and release its transport resources."""
        raise NotImplementedError

    @abstractmethod
    def is_connected(self) -> bool:
        """Return whether the channel is ready for its supported I/O."""
        raise NotImplementedError

    @abstractmethod
    def read_state(self) -> Mapping[str, object] | None:
        """Return the latest driver-neutral simulation snapshot."""
        raise NotImplementedError

    @abstractmethod
    def request_reset(self, scope: SimulationResetScope) -> None:
        """Request a reset with driver-neutral simulation semantics."""
        raise NotImplementedError
