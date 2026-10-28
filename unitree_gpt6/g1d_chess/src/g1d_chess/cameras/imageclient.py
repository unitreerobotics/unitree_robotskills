# ------------------------------------------------------------------------------
# Copyright 2025 Unitree Robotics
# Licensed under the Apache License 2.0.
#
# Adapted (made self-contained) from unitree-deploy:
#     unitree_deploy/robot_devices/cameras/imageclient.py
#
# Only depends on: pyzmq, opencv-python, numpy, pyyaml.
# ------------------------------------------------------------------------------
"""Self-contained ZMQ image client for the Unitree G1 internal cameras.

Connects to the image server running on the G1 board (default
configured explicitly for the current robot).  It first requests the camera configuration over a
REQ/REP socket (``request_port``, default 60000) and then subscribes to the
ZMQ publish streams of the head (binocular) camera and the left/right wrist
cameras, exposing BGR frames.
"""

from __future__ import annotations

import logging
import os
import queue
import threading
import time
from collections import deque
from typing import Any, Dict, Optional

import cv2
import numpy as np
import yaml
import zmq

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------
class TripleRingBuffer:
    """Latest-value ring buffer shared across the subscriber thread."""

    def __init__(self):
        self.buffer = [None, None, None]
        self.write_index = 0
        self.latest_index = -1
        self.read_index = -1
        self.lock = threading.Lock()

    def write(self, data):
        with self.lock:
            self.buffer[self.write_index] = data
            self.latest_index = self.write_index
            self.write_index = (self.write_index + 1) % 3
            if self.write_index == self.read_index:
                self.write_index = (self.write_index + 1) % 3

    def read(self):
        with self.lock:
            if self.latest_index == -1:
                return None
            self.read_index = self.latest_index
        return self.buffer[self.read_index]


class SimpleFPSMonitor:
    def __init__(self, window_size: int):
        self._times = deque(maxlen=window_size)
        self._last_tick = None
        self._fps = 0.0

    def tick(self):
        now = time.perf_counter_ns()
        if self._last_tick is not None:
            interval_ns = now - self._last_tick
            if interval_ns < 100_000:
                return
            self._times.append(interval_ns)
            if len(self._times) == self._times.maxlen:
                rolling_sum = sum(self._times)
                if rolling_sum > 0:
                    self._fps = (len(self._times) * 1_000_000_000.0) / rolling_sum
            else:
                self._fps = 0.0
        self._last_tick = now

    def reset(self):
        self._times.clear()
        self._last_tick = None
        self._fps = 0.0

    @property
    def fps(self) -> float:
        return self._fps


class TeleImage:
    """A single camera frame: JPEG bytes and (optionally) a decoded BGR array."""

    _NOT_SET = object()
    __slots__ = ["jpg", "_bgr", "fps"]

    def __init__(self, fps: float, jpg: Optional[bytes], bgr: Any = _NOT_SET):
        self.fps = fps
        self.jpg = jpg
        self._bgr = bgr

    @property
    def bgr(self) -> Optional[np.ndarray]:
        if self._bgr is TeleImage._NOT_SET or self._bgr is None:
            return None
        return self._bgr

    def __bool__(self):
        return bool(self.jpg)


# ---------------------------------------------------------------------------
# ZMQ subscribe
# ---------------------------------------------------------------------------
class ZMQ_SubscriberThread(threading.Thread):
    """Thread that owns a SUB socket and keeps the latest received frame."""

    def __init__(self, host: str, port: int, context: Optional[zmq.Context] = None, request_bgr: bool = False):
        super().__init__(daemon=True)
        self._host = host
        self._port = port
        self._context = context or zmq.Context.instance()
        self._request_bgr = request_bgr

        self._socket = None
        self._running = True
        self._started = threading.Event()

        self._jpg_3ring_buffer = TripleRingBuffer()
        self._fps_monitor = SimpleFPSMonitor(window_size=10)
        if self._request_bgr:
            self._bgr_3ring_buffer = TripleRingBuffer()
            self._bgr_decode_queue = queue.Queue(maxsize=1)
            self._decoder_thread = threading.Thread(target=self._decoder_loop, daemon=True)
            self._decoder_thread.start()
        else:
            self._bgr_3ring_buffer = None
            self._bgr_decode_queue = None
            self._decoder_thread = None

    def _decode_image(self, jpg_bytes):
        if jpg_bytes is None:
            return None
        try:
            np_img = np.frombuffer(jpg_bytes, dtype=np.uint8)
            return cv2.imdecode(np_img, cv2.IMREAD_COLOR)
        except Exception as e:  # noqa: BLE001
            logger.warning("Failed to decode image: %s", e)
            return None

    def _decoder_loop(self):
        while self._running:
            try:
                jpg_bytes = self._bgr_decode_queue.get(timeout=0.1)
                if jpg_bytes is None:
                    continue
                img_numpy = self._decode_image(jpg_bytes)
                self._bgr_3ring_buffer.write(img_numpy)
                self._bgr_decode_queue.task_done()
            except queue.Empty:
                continue

    def _wait_for_start(self, timeout: float = 1.0) -> bool:
        return self._started.wait(timeout=timeout)

    def recv(self) -> TeleImage:
        current_fps = self._fps_monitor.fps
        jpg_data = self._jpg_3ring_buffer.read()
        if not self._request_bgr:
            return TeleImage(fps=current_fps, jpg=jpg_data)
        bgr_data = self._bgr_3ring_buffer.read()
        return TeleImage(fps=current_fps, jpg=jpg_data, bgr=bgr_data)

    def stop(self) -> None:
        self._running = False
        self.join(timeout=1.0)
        if self._decoder_thread is not None:
            self._decoder_thread.join(timeout=1.0)
        if self.is_alive():
            logger.warning("Subscriber thread did not stop gracefully")

    def run(self) -> None:
        try:
            self._socket = self._context.socket(zmq.SUB)
            self._socket.setsockopt(zmq.RCVHWM, 1)
            self._socket.setsockopt(zmq.LINGER, 0)
            self._socket.connect(f"tcp://{self._host}:{self._port}")
            self._socket.setsockopt_string(zmq.SUBSCRIBE, "")

            poller = zmq.Poller()
            poller.register(self._socket, zmq.POLLIN)

            self._started.set()
            while self._running:
                events = dict(poller.poll(timeout=100))
                if self._socket in events:
                    try:
                        img_bytes = self._socket.recv()
                        self._jpg_3ring_buffer.write(img_bytes)
                        if self._request_bgr:
                            try:
                                if self._bgr_decode_queue.full():
                                    self._bgr_decode_queue.get_nowait()
                                self._bgr_decode_queue.put_nowait(img_bytes)
                            except queue.Full:
                                pass
                        self._fps_monitor.tick()
                    except Exception as e:  # noqa: BLE001
                        if self._running:
                            logger.error("Error in subscriber loop: %s", e)
                        break
                else:
                    self._jpg_3ring_buffer.write(None)
                    if self._request_bgr:
                        try:
                            if self._bgr_decode_queue.full():
                                self._bgr_decode_queue.get_nowait()
                            self._bgr_decode_queue.put_nowait(None)
                        except queue.Full:
                            pass
                    self._fps_monitor.reset()
        except Exception as e:  # noqa: BLE001
            logger.error("Failed to initialize subscriber socket: %s", e)
        finally:
            if self._socket:
                try:
                    self._socket.close()
                except Exception:  # noqa: BLE001
                    pass
                self._socket = None


class ZMQ_SubscriberManager:
    """Centralized management of ZMQ subscribers."""

    _instance: Optional["ZMQ_SubscriberManager"] = None
    _subscriber_threads: Dict[tuple, ZMQ_SubscriberThread] = {}
    _lock = threading.Lock()
    _running = True

    def __init__(self):
        # Shared context: never call term() on it, otherwise sockets from the
        # daemon threads can abort at interpreter shutdown.
        self._context = zmq.Context.instance()

    def _create_subscriber_thread(self, host: str, port: int, request_bgr: bool = False) -> ZMQ_SubscriberThread:
        subscriber_thread = ZMQ_SubscriberThread(host, port, self._context, request_bgr)
        subscriber_thread.start()
        if not subscriber_thread._wait_for_start(timeout=1.0):
            raise ConnectionError(f"Subscriber thread failed to start for {host}:{port}")
        return subscriber_thread

    def _get_subscriber_thread(self, host: str, port: int, request_bgr: bool = False) -> ZMQ_SubscriberThread:
        key = (host, port)
        with self._lock:
            if key not in self._subscriber_threads:
                self._subscriber_threads[key] = self._create_subscriber_thread(host, port, request_bgr)
            return self._subscriber_threads[key]

    @classmethod
    def get_instance(cls) -> "ZMQ_SubscriberManager":
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance

    def subscribe(self, host: str, port: int, request_bgr: bool = False) -> TeleImage:
        if not self._running:
            raise RuntimeError("SubscriberManager is closed.")
        subscriber_thread = self._get_subscriber_thread(host, port, request_bgr=request_bgr)
        return subscriber_thread.recv()

    def close(self) -> None:
        self._running = False
        with self._lock:
            for key, subscriber in self._subscriber_threads.items():
                try:
                    subscriber.stop()
                except Exception as e:  # noqa: BLE001
                    logger.error("Error stopping subscriber at %s:%s: %s", key[0], key[1], e)
            self._subscriber_threads.clear()


# ---------------------------------------------------------------------------
# ZMQ request (camera configuration)
# ---------------------------------------------------------------------------
class ZMQ_Requester:
    """ZMQ REQ socket to request the camera configuration from the server."""

    def __init__(self, host: str, port: int, config_dir: Optional[str] = None):
        self._host = host
        self._port = port
        self._context = zmq.Context.instance()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(f"tcp://{self._host}:{self._port}")

        self._poller = zmq.Poller()
        self._poller.register(self._socket, zmq.POLLIN)

        from g1d_chess.paths import ROOT
        config_dir = config_dir or str(ROOT / "config")
        self._config_client_path = os.path.join(config_dir, "cam_config_client.yaml")
        self._config_server_path = os.path.join(config_dir, "cam_config_server.yaml")

    def _load_local(self) -> Optional[Dict[str, Any]]:
        for path in (self._config_client_path, self._config_server_path):
            if os.path.exists(path):
                try:
                    with open(path, "r") as f:
                        return yaml.safe_load(f)
                except Exception as e:  # noqa: BLE001
                    logger.warning("Failed to load local camera config %s: %s", path, e)
        return None

    def request(self) -> Optional[Dict[str, Any]]:
        cam_config = None
        try:
            self._socket.send(b"GET_DATA")
            socks = dict(self._poller.poll(timeout=1000))
            if self._socket in socks and socks[self._socket] == zmq.POLLIN:
                cam_config = self._socket.recv_json()
                if cam_config is not None:
                    logger.info("Received camera config from %s:%s", self._host, self._port)
                    with open(self._config_client_path, "w") as f:
                        yaml.safe_dump(cam_config, f, sort_keys=False, allow_unicode=True)
            else:
                logger.warning("No response from %s:%s, using local config.", self._host, self._port)
                cam_config = self._load_local()
        except Exception as e:  # noqa: BLE001
            logger.error("Error requesting camera config: %s", e)
            cam_config = self._load_local()
        return cam_config

    def close(self) -> None:
        try:
            self._socket.close()
        except Exception:  # noqa: BLE001
            pass


# ---------------------------------------------------------------------------
# Image client
# ---------------------------------------------------------------------------
class ImageClient:
    def __init__(self, host: str, request_port: int = 60000,
                 request_bgr: bool = True, config_dir: Optional[str] = None):
        self._host = host
        self._request_port = request_port
        self._request_bgr = request_bgr

        self._subscriber_manager = ZMQ_SubscriberManager.get_instance()
        self._requester = ZMQ_Requester(self._host, self._request_port, config_dir=config_dir)
        self._cam_config = self._requester.request()

        if self._cam_config is None:
            raise RuntimeError("Failed to get camera configuration.")

        # Subscribe lazily in the selected get_*_frame method. In particular,
        # creating a client for head + right wrist must not open the left stream.

    def get_cam_config(self):
        return self._cam_config

    def get_head_frame(self):
        return self._subscriber_manager.subscribe(
            self._host, self._cam_config["head_camera"]["zmq_port"], request_bgr=self._request_bgr
        )

    def get_left_wrist_frame(self):
        return self._subscriber_manager.subscribe(
            self._host, self._cam_config["left_wrist_camera"]["zmq_port"], request_bgr=self._request_bgr
        )

    def get_right_wrist_frame(self):
        return self._subscriber_manager.subscribe(
            self._host, self._cam_config["right_wrist_camera"]["zmq_port"], request_bgr=self._request_bgr
        )

    def close(self):
        self._subscriber_manager.close()
        self._requester.close()
        # Explicitly terminate the shared context now that every socket is closed
        # and every thread is joined, instead of letting pyzmq finalize it during
        # interpreter shutdown (which intermittently aborts with
        # "terminate called without an active exception").
        try:
            zmq.Context.instance().term()
        except Exception:
            pass
        logger.info("Image client has been closed.")


class ImageClientCamera:
    """Wrapper that mirrors unitree-deploy's ImageClientCamera.

    ``async_read()`` returns ``{"cam_left_high", "cam_right_high",
    "cam_left_wrist", "cam_right_wrist"}`` as BGR numpy arrays, splitting the
    binocular head frame into left/right at the midpoint.
    """

    def __init__(self, host_ip: str, config_dir: Optional[str] = None):
        self.host_ip = host_ip
        self.config_dir = config_dir
        self.img_client: Optional[ImageClient] = None
        self.camera_config = None

    def connect(self):
        self.img_client = ImageClient(self.host_ip, request_bgr=True, config_dir=self.config_dir)
        self.camera_config = self.img_client.get_cam_config()

    def async_read(self, cameras=None) -> Dict[str, np.ndarray]:
        if self.img_client is None:
            raise RuntimeError("ImageClient is not connected. Call connect() first.")

        selected = set(cameras) if cameras is not None else {
            "cam_left_high", "cam_right_high", "cam_left_wrist", "cam_right_wrist"
        }
        colors: Dict[str, np.ndarray] = {}
        head_cfg = self.camera_config.get("head_camera") or {}
        if selected.intersection({"cam_left_high", "cam_right_high"}) and head_cfg.get("enable_zmq"):
            head_img = self.img_client.get_head_frame()
            if head_img.bgr is not None:
                width = head_cfg["image_shape"][1]
                if "cam_left_high" in selected:
                    colors["cam_left_high"] = head_img.bgr[:, : width // 2]
                if "cam_right_high" in selected:
                    colors["cam_right_high"] = head_img.bgr[:, width // 2 :]

        left_cfg = self.camera_config.get("left_wrist_camera") or {}
        if "cam_left_wrist" in selected and left_cfg.get("enable_zmq"):
            left_wrist_img = self.img_client.get_left_wrist_frame()
            if left_wrist_img.bgr is not None:
                colors["cam_left_wrist"] = left_wrist_img.bgr

        right_cfg = self.camera_config.get("right_wrist_camera") or {}
        if "cam_right_wrist" in selected and right_cfg.get("enable_zmq"):
            right_wrist_img = self.img_client.get_right_wrist_frame()
            if right_wrist_img.bgr is not None:
                colors["cam_right_wrist"] = right_wrist_img.bgr

        return colors

    def disconnect(self):
        if self.img_client is not None:
            self.img_client.close()
            self.img_client = None


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default=os.environ.get("CHESS_CAMERA_HOST"), help="IP address of image server")
    args = parser.parse_args()
    if not args.host:
        parser.error("pass --host or set CHESS_CAMERA_HOST")

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    client = ImageClient(host=args.host, request_bgr=True)
    cam_config = client.get_cam_config()
    print("Camera config:")
    for key in ("head_camera", "left_wrist_camera", "right_wrist_camera"):
        cfg = cam_config.get(key) or {}
        print(f"  {key}: enable_zmq={cfg.get('enable_zmq')} "
              f"zmq_port={cfg.get('zmq_port')} shape={cfg.get('image_shape')}")

    try:
        while True:
            camera = ImageClientCamera(host_ip=args.host)
            camera.img_client = client
            camera.camera_config = cam_config
            colors = camera.async_read()
            print({k: (None if v is None else v.shape) for k, v in colors.items()})
            time.sleep(0.5)
    except KeyboardInterrupt:
        client.close()


if __name__ == "__main__":
    main()
