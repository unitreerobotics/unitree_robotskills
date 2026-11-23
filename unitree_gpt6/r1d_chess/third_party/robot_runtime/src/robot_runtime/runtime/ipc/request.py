"""Runtime request message transport."""

from __future__ import annotations

from collections.abc import Mapping
from enum import Enum

import msgspec
import zmq


class RuntimeRequestOperation(Enum):
    PREPARE_CONTROL = "prepare_control"
    START_CONTROL = "start_control"
    STOP_CONTROL = "stop_control"
    RESET_POSE = "reset_pose"
    STOP_RESET_POSE = "stop_reset_pose"
    RESET_SIMULATION_OBJECTS = "reset_simulation_objects"
    RESET_SIMULATION_SCENE = "reset_simulation_scene"


class RuntimeRequest(msgspec.Struct, kw_only=True, forbid_unknown_fields=True):
    operation: RuntimeRequestOperation
    cmd_session_id: str | None = None
    cmd_components: dict[str, list[str]] | None = None
    renew_only: bool = False
    restore_standing: bool = False


class RuntimeRequestSender:
    """Send synchronous requests through one REQ socket."""

    def __init__(self, *, endpoint: str) -> None:
        self.endpoint = endpoint

        self.context: zmq.Context | None = None
        self.socket = None

    def start(self) -> None:
        if self.socket is not None:
            return
        if self.context is None:
            self.context = zmq.Context()
        try:
            self.socket = self.context.socket(zmq.REQ)
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

    def send(
        self,
        request: RuntimeRequest,
        *,
        timeout: float,
    ) -> None:
        self.start()
        self.socket.send(msgspec.msgpack.encode(request))
        if not self.socket.poll(int(timeout * 1000), zmq.POLLIN):
            self.socket.close(linger=0)
            self.socket = None
            raise TimeoutError(
                f"Runtime request {request.operation.value!r} at "
                f"{self.endpoint!r} timed out after {timeout:.1f}s."
            )
        error_message = msgspec.msgpack.decode(
            self.socket.recv(),
            type=str | None,
        )
        if error_message is not None:
            raise RuntimeError(error_message)

    def prepare_control(
        self,
        *,
        cmd_session_id: str,
        renew_only: bool = False,
        timeout: float = 60.0,
    ) -> None:
        self.send(
            RuntimeRequest(
                operation=RuntimeRequestOperation.PREPARE_CONTROL,
                cmd_session_id=cmd_session_id,
                renew_only=renew_only,
            ),
            timeout=timeout,
        )

    def start_control(
        self,
        *,
        cmd_session_id: str,
        cmd_components: Mapping[str, tuple[str, ...]],
        timeout: float = 60.0,
    ) -> None:
        self.send(
            RuntimeRequest(
                operation=RuntimeRequestOperation.START_CONTROL,
                cmd_session_id=cmd_session_id,
                cmd_components={
                    section: list(component_names)
                    for section, component_names in cmd_components.items()
                },
            ),
            timeout=timeout,
        )

    def stop_control(
        self,
        *,
        cmd_session_id: str,
        restore_standing: bool = False,
        timeout: float = 60.0,
    ) -> None:
        self.send(
            RuntimeRequest(
                operation=RuntimeRequestOperation.STOP_CONTROL,
                cmd_session_id=cmd_session_id,
                restore_standing=restore_standing,
            ),
            timeout=timeout,
        )

    def reset_pose(
        self,
        *,
        cmd_session_id: str | None = None,
        timeout: float = 12.0,
    ) -> None:
        self.send(
            RuntimeRequest(
                operation=RuntimeRequestOperation.RESET_POSE,
                cmd_session_id=cmd_session_id,
            ),
            timeout=timeout,
        )

    def stop_reset_pose(self, *, timeout: float = 12.0) -> None:
        self.send(
            RuntimeRequest(operation=RuntimeRequestOperation.STOP_RESET_POSE),
            timeout=timeout,
        )

    def reset_simulation_objects(self, *, timeout: float = 2.0) -> None:
        self.send(
            RuntimeRequest(
                operation=RuntimeRequestOperation.RESET_SIMULATION_OBJECTS
            ),
            timeout=timeout,
        )

    def reset_simulation_scene(self, *, timeout: float = 2.0) -> None:
        self.send(
            RuntimeRequest(
                operation=RuntimeRequestOperation.RESET_SIMULATION_SCENE
            ),
            timeout=timeout,
        )


class RuntimeRequestReceiver:
    """Receive requests and send replies through one REP socket."""

    def __init__(self, *, endpoint: str) -> None:
        self.endpoint = endpoint
        self.decoder = msgspec.msgpack.Decoder(RuntimeRequest)

        self.context: zmq.Context | None = None
        self.socket = None

    def start(self) -> None:
        if self.socket is not None:
            return
        try:
            self.context = zmq.Context()
            self.socket = self.context.socket(zmq.REP)
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

    def receive(self) -> RuntimeRequest:
        if self.socket is None:
            raise RuntimeError("Runtime request receiver has not been started.")
        return self.decoder.decode(self.socket.recv())

    def reply(self, error_message: str | None) -> None:
        if self.socket is None:
            raise RuntimeError("Runtime request receiver has not been started.")
        self.socket.send(msgspec.msgpack.encode(error_message))
