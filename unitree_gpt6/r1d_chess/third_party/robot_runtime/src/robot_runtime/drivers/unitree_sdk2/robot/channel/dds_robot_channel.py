"""Shared DDS transport for Unitree SDK2 component channels.

``DDSRobotChannel`` owns DDS publishers, subscribers, synchronization,
lifecycle, and the latest decoded component-state snapshot. Concrete channels
provide topic mappings and override message decoding or encoding when a
protocol does not map directly through its ``ComponentInterface`` objects.
"""

import threading
import time
from collections.abc import Callable, Iterable, Mapping

import logging_mp
from typing_extensions import override
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber

from robot_runtime.interfaces.robot.channel import RobotChannel
from robot_runtime.interfaces.robot.component import (
    ComponentCmds,
    ComponentInterface,
    ComponentStates,
)

logger_mp = logging_mp.getLogger(__name__)

STATE_READ_PERIOD = 1.0 / 250.0
STATE_READ_TIMEOUT = 0.1


class DDSRobotChannel(RobotChannel):
    """DDS transport for components using zero or more topics.

    The channel becomes ready after every configured state topic has produced
    a message. Readers retain the latest message from each topic; consumers
    decode that snapshot on demand. A command write creates one message for
    every configured command topic.

    Topic mapping order defines the message tuple order received by the
    default component-interface decoder and encoder.

    Args:
        state_topics: DDS state topics mapped to their message classes.
        cmd_topics: DDS command topics mapped to message classes and factories.
        components: Component interfaces keyed by component ID.
        writable_components: Component IDs accepted by command writes.
        start_timeout: Seconds to wait for all configured state topics.
        close_timeout: Optional maximum seconds spent closing each DDS handle.

    Attributes:
        state_topics: Configured state-topic message classes.
        cmd_topics: Configured command-topic message classes and factories.
        components: Component interfaces keyed by component ID.
        writable_components: Immutable set of writable component IDs.
        start_timeout: Seconds allowed for the initial state snapshot.
        close_timeout: Optional maximum seconds allowed per DDS handle close.
        running: Whether the DDS channel lifecycle is active.
        state_msgs: Latest raw DDS message cached by state topic. Consumers
            decode only after every configured state topic is represented.
        component_states: Latest decoded states keyed by component ID.

    Example:
        channel = DDSRobotChannel(
            state_topics={"rt/example/state": StateMsg},
            cmd_topics={"rt/example/cmd": (CmdMsg, CmdMsg)},
            components={component.component_id: component},
            writable_components=(component.component_id,),
        )
        states = channel.decode_component_states({
            "rt/example/state": state_msg,
        })
        cmd_msg = CmdMsg()
        channel.encode_component_cmds(
            {
                component.component_id: component_cmds,
            },
            {"rt/example/cmd": cmd_msg},
        )
    """

    def __init__(
        self,
        *,
        state_topics: Mapping[str, type],
        cmd_topics: Mapping[
            str,
            tuple[type, Callable[[], object]],
        ],
        components: Mapping[str, ComponentInterface],
        writable_components: Iterable[str],
        start_timeout: float = 10.0,
        close_timeout: float | None = None,
    ) -> None:
        self.state_topics = dict(state_topics)
        self.cmd_topics = dict(cmd_topics)
        self.start_timeout = start_timeout
        self.close_timeout = close_timeout

        super().__init__(
            components=components,
            writable_components=writable_components,
        )

        self.dds_io_lock = threading.Lock()
        self.running = False
        self.state_subscribers: dict[str, ChannelSubscriber] = {}
        self.cmd_publishers: dict[str, ChannelPublisher] = {}

        self.state_lock = threading.Lock()
        self.decode_lock = threading.Lock()
        self.state_ready = threading.Event()
        self.state_read_stop = threading.Event()
        self.state_threads: list[threading.Thread] = []
        self.state_msgs: dict[str, object] = {}
        self.component_states: dict[str, ComponentStates] = {}
        self.state_version = 0
        self.decoded_state_version = -1

    @override
    def start(self) -> None:
        """Open DDS handles and wait for every configured state topic."""
        with self.dds_io_lock:
            if self.running:
                return
            with self.state_lock:
                self.state_msgs = {}
                self.component_states = {}
                self.state_version = 0
                self.decoded_state_version = -1
            self.state_ready.clear()
            self.state_read_stop.clear()
            self.running = True
            state_subscribers = {}
            try:
                for state_topic, state_msg_type in self.state_topics.items():
                    state_subscriber = ChannelSubscriber(
                        state_topic,
                        state_msg_type,
                    )
                    state_subscribers[state_topic] = state_subscriber
                    # Pull from DDS instead of deserializing every arrival
                    # into the SDK's FIFO, which drops new samples when full.
                    state_subscriber.Init()
                self.state_subscribers = state_subscribers
            except Exception:
                self.running = False
                self.state_ready.clear()
                for state_topic, state_subscriber in state_subscribers.items():
                    self.close_dds_handle(
                        f"state subscriber {state_topic!r}",
                        state_subscriber,
                    )
                raise
            if not self.state_topics:
                self.state_ready.set()

        try:
            for state_topic, subscriber in self.state_subscribers.items():
                thread = threading.Thread(
                    target=self.read_state_loop,
                    args=(state_topic, subscriber),
                    name=f"{type(self).__name__}.{state_topic}",
                    daemon=True,
                )
                thread.start()
                self.state_threads.append(thread)
            if self.state_topics:
                logger_mp.info(
                    "[%s] Waiting for DDS state from %r for up to %.1fs.",
                    type(self).__name__,
                    tuple(self.state_topics),
                    self.start_timeout,
                )
                if not self.state_ready.wait(self.start_timeout):
                    with self.state_lock:
                        missing_topics = tuple(
                            topic
                            for topic in self.state_topics
                            if topic not in self.state_msgs
                        )
                    raise TimeoutError(
                        f"[{type(self).__name__}] Timed out waiting for DDS state "
                        f"from {missing_topics!r} after {self.start_timeout:.1f}s."
                    )
                self.read_component_states()
        except Exception:
            self.stop()
            raise

        cmd_publishers = {}
        try:
            for cmd_topic, (cmd_msg_type, _) in self.cmd_topics.items():
                cmd_publisher = ChannelPublisher(cmd_topic, cmd_msg_type)
                cmd_publishers[cmd_topic] = cmd_publisher
                cmd_publisher.Init()
        except Exception:
            self.stop()
            for cmd_topic, cmd_publisher in cmd_publishers.items():
                self.close_dds_handle(
                    f"cmd publisher {cmd_topic!r}",
                    cmd_publisher,
                )
            raise
        with self.dds_io_lock:
            if not self.running:
                for cmd_topic, cmd_publisher in cmd_publishers.items():
                    self.close_dds_handle(
                        f"cmd publisher {cmd_topic!r}",
                        cmd_publisher,
                    )
                return
            self.cmd_publishers = cmd_publishers
        logger_mp.info("[%s] DDS ready.", type(self).__name__)

    @override
    def stop(self) -> None:
        """Stop I/O, clear raw DDS messages, and close all DDS handles."""
        with self.dds_io_lock:
            if (
                not self.running
                and not self.state_subscribers
                and not self.cmd_publishers
                and not self.state_threads
            ):
                return
            self.running = False
            self.state_read_stop.set()
            self.state_ready.clear()
        # A reader may still be in Read(); do not close its DDS handle first.
        for thread in self.state_threads:
            thread.join(timeout=1.0)
            if thread.is_alive():
                raise RuntimeError(f"DDS state reader {thread.name!r} did not stop.")
        self.state_threads.clear()
        with self.dds_io_lock:
            handles = (
                *(
                    (f"cmd publisher {topic!r}", publisher)
                    for topic, publisher in self.cmd_publishers.items()
                ),
                *(
                    (f"state subscriber {topic!r}", subscriber)
                    for topic, subscriber in self.state_subscribers.items()
                ),
            )
            self.state_subscribers = {}
            self.cmd_publishers = {}
        with self.decode_lock:
            with self.state_lock:
                self.state_msgs = {}
                self.component_states = {}
                self.state_version = 0
                self.decoded_state_version = -1
                self.state_ready.clear()
        for handle_name, handle in handles:
            self.close_dds_handle(handle_name, handle)
        logger_mp.info("[%s] DDS stopped.", type(self).__name__)

    def close_dds_handle(self, handle_name: str, handle: object) -> None:
        """Close one DDS handle, optionally enforcing ``close_timeout``."""
        if self.close_timeout is None:
            handle.Close()
            return

        close_errors = []

        def close_handle() -> None:
            try:
                handle.Close()
            except Exception as exc:
                close_errors.append(exc)

        close_thread = threading.Thread(
            target=close_handle,
            name=f"{type(self).__name__}.{handle_name}.close",
            daemon=True,
        )
        close_thread.start()
        close_thread.join(self.close_timeout)
        if close_thread.is_alive():
            logger_mp.warning(
                "[%s] Timed out closing %s after %.1fs.",
                type(self).__name__,
                handle_name,
                self.close_timeout,
            )
            return
        if close_errors:
            raise close_errors[0]

    @override
    def is_connected(self) -> bool:
        """Return whether every configured publisher and subscriber is ready."""
        return bool(
            self.running
            and self.state_ready.is_set()
            and len(self.state_subscribers) == len(self.state_topics)
            and len(self.cmd_publishers) == len(self.cmd_topics)
        )

    @override
    def read_component_states(self) -> Mapping[str, ComponentStates]:
        """Return the latest complete decoded component-state snapshot."""
        with self.decode_lock:
            with self.state_lock:
                if (
                    not self.state_msgs
                    or len(self.state_msgs) != len(self.state_topics)
                    or self.state_version == self.decoded_state_version
                ):
                    return self.component_states
                state_msgs = self.state_msgs.copy()
                version = self.state_version
            self.component_states = self.decode_component_states(state_msgs)
            self.decoded_state_version = version
            return self.component_states

    @override
    def write_component_cmds(
        self,
        component_cmds: Mapping[str, ComponentCmds],
    ) -> None:
        """Encode and publish one synchronized command message per topic."""
        if set(component_cmds) != self.writable_components:
            raise RuntimeError(
                f"{type(self).__name__} requires component commands for "
                f"{tuple(sorted(self.writable_components))!r}."
            )
        with self.decode_lock:
            cmd_msgs = {
                cmd_topic: cmd_msg_factory()
                for cmd_topic, (_, cmd_msg_factory) in self.cmd_topics.items()
            }
            self.encode_component_cmds(component_cmds, cmd_msgs)

        with self.dds_io_lock:
            if not self.is_connected():
                raise RuntimeError(
                    f"{type(self).__name__} is not connected."
                )
            for cmd_topic, cmd_msg in cmd_msgs.items():
                self.cmd_publishers[cmd_topic].Write(cmd_msg)

    def read_state_loop(
        self,
        state_topic: str,
        subscriber: ChannelSubscriber,
    ) -> None:
        """Read each topic independently, at most 250 times per second."""
        while not self.state_read_stop.is_set():
            started_at = time.monotonic()
            state_msg = subscriber.Read(timeout=STATE_READ_TIMEOUT)
            if state_msg is not None:
                self.handle_state_msg(state_topic, state_msg)
            self.state_read_stop.wait(
                max(0.0, STATE_READ_PERIOD - (time.monotonic() - started_at))
            )

    def handle_state_msg(
        self,
        state_topic: str,
        state_msg: object,
    ) -> None:
        """Replace a topic's raw sample without decoding other components."""
        with self.state_lock:
            if not self.running:
                return
            self.state_msgs[state_topic] = state_msg
            self.state_version += 1
            if len(self.state_msgs) < len(self.state_topics):
                return
            self.state_ready.set()

    def decode_component_states(
        self,
        state_msgs: Mapping[str, object],
    ) -> dict[str, ComponentStates]:
        """Decode an ordered state-message snapshot through every component."""
        ordered_state_msgs = tuple(
            state_msgs[state_topic] for state_topic in self.state_topics
        )
        return {
            component_id: component_interface.read_states_from_msgs(
                ordered_state_msgs
            )
            for component_id, component_interface in self.components.items()
        }

    def encode_component_cmds(
        self,
        component_cmds: Mapping[str, ComponentCmds],
        cmd_msgs: Mapping[str, object],
    ) -> None:
        """Fill ordered command messages through all writable components."""
        ordered_cmd_msgs = tuple(
            cmd_msgs[cmd_topic] for cmd_topic in self.cmd_topics
        )
        for component_id in self.writable_components:
            component_interface = self.components[component_id]
            component_interface.write_cmds_to_msgs(
                ordered_cmd_msgs,
                component_cmds[component_id],
            )
