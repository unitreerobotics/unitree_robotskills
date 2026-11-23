"""Runtime command message transport."""

from __future__ import annotations

import msgspec
import zmq

from robot_runtime.schema.frame import ActionData


class RuntimeCmd(msgspec.Struct, kw_only=True, forbid_unknown_fields=True):
    """One runtime IPC command and its command-session metadata.

    ``timestamp`` is the publisher process's monotonic time in seconds. ``action``
    contains the selected component targets; Hybrid pose input is required only
    when ``action.hybrid_mode == 1``. Runtime applies its
    configured hard joint-rate limit to accepted body-joint targets.
    """

    cmd_session_id: str
    cmd_sequence: int
    timestamp: float
    action: ActionData


class RuntimeCmdSender:
    """Publish runtime commands through one PUB socket."""

    def __init__(self, *, endpoint: str) -> None:
        self.endpoint = endpoint
        self.context: zmq.Context | None = None
        self.socket = None

    @property
    def is_running(self) -> bool:
        return self.socket is not None

    def start(self) -> None:
        if self.is_running:
            return
        try:
            self.context = zmq.Context()
            self.socket = self.context.socket(zmq.PUB)
            self.socket.setsockopt(zmq.CONFLATE, 1)
            self.socket.setsockopt(zmq.IMMEDIATE, 1)
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.connect(self.endpoint)
        except Exception:
            self.stop()
            raise

    def stop(self) -> None:
        if self.socket is not None:
            self.socket.close(linger=0)
            self.socket = None
        if self.context is not None:
            self.context.term()
            self.context = None

    def send(self, cmd: RuntimeCmd) -> None:
        if self.socket is None:
            raise RuntimeError("Runtime command sender has not been started.")
        self.socket.send(
            msgspec.msgpack.encode(cmd),
            flags=zmq.DONTWAIT,
        )


class RuntimeCmdReceiver:
    """Receive runtime commands through one SUB socket."""

    def __init__(self, *, endpoint: str) -> None:
        self.endpoint = endpoint
        self.context: zmq.Context | None = None
        self.socket = None
        self.decoder = msgspec.msgpack.Decoder(RuntimeCmd)

    @property
    def is_running(self) -> bool:
        return self.socket is not None

    def start(self) -> None:
        if self.is_running:
            return
        try:
            self.context = zmq.Context()
            self.socket = self.context.socket(zmq.SUB)
            self.socket.setsockopt(zmq.SUBSCRIBE, b"")
            self.socket.setsockopt(zmq.CONFLATE, 1)
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.bind(self.endpoint)
        except Exception:
            self.stop()
            raise

    def stop(self) -> None:
        if self.socket is not None:
            self.socket.close(linger=0)
            self.socket = None
        if self.context is not None:
            self.context.term()
            self.context = None

    def receive(self) -> RuntimeCmd:
        if self.socket is None:
            raise RuntimeError("Runtime command receiver has not been started.")
        return self.decoder.decode(self.socket.recv())
