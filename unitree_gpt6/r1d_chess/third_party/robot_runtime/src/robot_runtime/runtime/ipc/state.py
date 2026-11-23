"""Runtime state message transport."""

from __future__ import annotations

import threading
import time
from typing import Any

import msgspec
import numpy as np
import zmq

from robot_runtime.runtime.ipc.shared_image import ImageFrameRef, SharedImageReader
from robot_runtime.schema.frame import ActionData, ObservationData


class RuntimeState(msgspec.Struct, kw_only=True, forbid_unknown_fields=True):
    """Latest observation, available resources, and execution status."""

    state_bindings: dict[str, list[str]]
    available_cmd_components: dict[str, list[str]]
    sequence: int
    observation: ObservationData
    display_streams: dict[str, Any] = msgspec.field(default_factory=dict)
    image_refs: list[ImageFrameRef] = msgspec.field(default_factory=list)
    accepted_cmd: ActionData | None = None
    control_active: bool = False
    reset_pose_active: bool = False
    reset_pose_generation: int = 0
    thermal_protection_active: bool = False
    simulation_state: dict[str, Any] | None = None
    runtime_error: str | None = None


class RuntimeStateStaleError(RuntimeError):
    """The latest Runtime state is older than the accepted age."""


class RuntimeStateSender:
    """Publish the latest runtime snapshot through one PUB socket."""

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

    def send(self, state: RuntimeState) -> None:
        if self.socket is None:
            raise RuntimeError("Runtime state sender has not been started.")
        self.socket.send(msgspec.msgpack.encode(state))


class RuntimeStateReceiver:
    """Receive the latest runtime snapshot and hydrate shared images."""

    def __init__(
        self,
        *,
        endpoint: str,
        max_state_age: float | None = None,
        hydrate_images: bool = True,
    ) -> None:
        self.endpoint = endpoint
        self.max_state_age = max_state_age
        self.hydrate_images_enabled = hydrate_images
        self.context: zmq.Context | None = None
        self.socket = None
        self.thread = None
        self.stop_requested = threading.Event()
        self.ready = threading.Event()
        self.state_lock = threading.Lock()
        self.state: RuntimeState | None = None
        self.state_received_at = 0.0
        self.bgr_frames: dict[str, np.ndarray] = {}
        self.image_reader = SharedImageReader()
        self.error: Exception | None = None

    def start(self, *, timeout: float = 10.0) -> None:
        if self.thread is not None:
            return
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.connect(self.endpoint)
        self.stop_requested.clear()
        self.ready.clear()
        self.error = None
        self.thread = threading.Thread(
            target=self.receive_loop,
            name="robot-runtime-state-receiver",
            daemon=True,
        )
        self.thread.start()
        if not self.ready.wait(timeout):
            self.stop()
            raise TimeoutError(
                f"No runtime state received from {self.endpoint!r} "
                f"within {timeout:.1f}s."
            )
        if self.error is not None:
            error = self.error
            self.stop()
            raise RuntimeError("Runtime state receiver failed.") from error

    def stop(self) -> None:
        self.stop_requested.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        self.thread = None
        if self.socket is not None:
            self.socket.close(linger=0)
            self.socket = None
        if self.context is not None:
            self.context.term()
            self.context = None
        self.image_reader.close()
        with self.state_lock:
            self.state = None
            self.bgr_frames = {}
            self.state_received_at = 0.0
        self.ready.clear()

    def receive_loop(self) -> None:
        socket = self.socket
        if socket is None:
            return
        state_decoder = msgspec.msgpack.Decoder(RuntimeState)
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)
        while not self.stop_requested.is_set():
            try:
                events = dict(poller.poll(20))
                if socket not in events:
                    continue
                state = state_decoder.decode(socket.recv())
                bgr_frames = (
                    self.hydrate_images(state)
                    if self.hydrate_images_enabled
                    else {}
                )
                with self.state_lock:
                    self.state = state
                    self.bgr_frames = bgr_frames
                    self.state_received_at = time.monotonic()
                self.ready.set()
            except zmq.error.ContextTerminated:
                break
            except Exception as exc:
                self.error = exc
                self.ready.set()
                break

    def hydrate_images(
        self,
        state: RuntimeState,
    ) -> dict[str, np.ndarray]:
        with self.state_lock:
            previous_state = self.state
            bgr_frames = dict(self.bgr_frames)
        image_data = {
            item.schema_key: item
            for item in state.observation.color or ()
        }
        previous_image_data = {
            item.schema_key: item
            for item in (
                ()
                if previous_state is None
                else previous_state.observation.color or ()
            )
        }
        for frame_ref in state.image_refs:
            try:
                raw = self.image_reader.read(frame_ref)
            except FileNotFoundError:
                raw = None
            if raw is None:
                if frame_ref.format == "bgr":
                    continue
                image = image_data.get(frame_ref.schema_key)
                previous_image = previous_image_data.get(
                    frame_ref.schema_key
                )
                if (
                    image is None
                    or previous_image is None
                    or previous_image.raw is None
                ):
                    continue
                image.update(
                    raw=previous_image.raw,
                    shape=frame_ref.shape[:2],
                    format=frame_ref.format,
                    is_padded=True,
                )
                continue
            if frame_ref.format == "bgr":
                frame = np.frombuffer(raw, dtype=np.uint8).reshape(
                    tuple(frame_ref.shape)
                )
                bgr_frames[frame_ref.schema_key] = frame.copy()
                continue
            image = image_data.get(frame_ref.schema_key)
            if image is None:
                continue
            image.update(
                raw=raw,
                shape=frame_ref.shape[:2],
                format=frame_ref.format,
                is_padded=frame_ref.is_padded,
            )
        return bgr_frames

    def latest_state(
        self,
        *,
        max_age: float | None = None,
    ) -> RuntimeState:
        if self.error is not None:
            raise RuntimeError("Runtime state receiver failed.") from self.error
        with self.state_lock:
            state = self.state
            received_at = self.state_received_at
        if state is None:
            raise RuntimeError("Runtime state is not available.")
        max_age = self.max_state_age if max_age is None else max_age
        age = time.monotonic() - received_at
        if max_age is not None and age > max_age:
            raise RuntimeStateStaleError(
                f"Runtime state is stale (age={age:.3f}s, "
                f"max_age={max_age:.3f}s)."
            )
        return state

    def read_bgr_frame(self, schema_key: str) -> np.ndarray:
        if not self.hydrate_images_enabled:
            raise RuntimeError("Runtime state image hydration is disabled.")
        with self.state_lock:
            frame = self.bgr_frames.get(schema_key)
        if frame is None:
            raise RuntimeError(
                f"Runtime does not provide a BGR frame for {schema_key!r}."
            )
        return frame.copy()

    def wait_for_jpg_frames(
        self,
        *,
        schema_keys: tuple[str, ...],
        timeout: float,
    ) -> tuple[str, ...]:
        if not self.hydrate_images_enabled:
            raise RuntimeError("Runtime state image hydration is disabled.")
        deadline = time.monotonic() + max(timeout, 0.0)
        while True:
            state = self.latest_state()
            images = {
                image.schema_key: image
                for image in state.observation.color or ()
            }
            missing = tuple(
                schema_key
                for schema_key in schema_keys
                if images.get(schema_key) is None
                or images[schema_key].raw is None
            )
            if not missing or time.monotonic() >= deadline:
                return missing
            time.sleep(0.02)

    def display_stream_config(self, stream_key: str) -> dict[str, object]:
        display_streams = self.latest_state().display_streams
        try:
            stream_config = display_streams[stream_key]
        except KeyError as exc:
            known = ", ".join(sorted(display_streams))
            raise KeyError(
                f"Runtime display stream {stream_key!r} is unknown. Known: {known}"
            ) from exc
        return dict(stream_config)
