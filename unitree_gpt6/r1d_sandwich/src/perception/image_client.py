"""Read-only client of the robot's image server (teleimager on the R1-D development unit, 192.168.123.164).

Wire protocol of the teleimager image server:
  REQ  host:60000   b"GET_DATA" -> JSON camera config: {"head_camera": {"enable_zmq", "zmq_port", "binocular",
                    "image_shape": [h, w], ...}, "left_wrist_camera": {...}, "right_wrist_camera": {...}},
                    sometimes wrapped as {"camera": {...}}
  SUB  host:<port>  one JPEG per message; the head image is the stereo pair side by side (left eye on the left)

Needs pyzmq, OpenCV and numpy only. Each stream is read by a daemon thread that keeps the newest JPEG; decoding
happens when a frame is asked for. Frames are BGR, as cv2.imdecode returns them.
"""
from __future__ import annotations

import threading
import time

CONFIG_PORT = 60000
DEFAULT_PORTS = {"head_camera": 55555, "left_wrist_camera": 55556, "right_wrist_camera": 55557}
# our view name -> (stream, which half of a side-by-side pair: 0 left, 1 right, None = the whole image)
VIEWS = {"head_left": ("head_camera", 0), "head_right": ("head_camera", 1),
         "left_wrist": ("left_wrist_camera", None), "right_wrist": ("right_wrist_camera", None)}


def request_config(host: str, port: int = CONFIG_PORT, timeout_ms: int = 1000) -> dict | None:
    """The image server's camera config, or None when it does not answer within the timeout."""
    import zmq

    sock = zmq.Context.instance().socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    try:
        sock.connect(f"tcp://{host}:{port}")
        sock.send(b"GET_DATA")
        if not sock.poll(timeout_ms):
            return None
        cfg = sock.recv_json()
    except (zmq.ZMQError, ValueError):
        return None
    finally:
        sock.close()
    if isinstance(cfg, dict) and isinstance(cfg.get("camera"), dict):
        cfg = cfg["camera"]
    return cfg if isinstance(cfg, dict) else None


class ImageClient:
    """Newest frame of the head pair and the two wrist cameras.

    The config is asked for once; a server that does not answer leaves the default ports in use.
    A frame's time is when its JPEG last CHANGED: a stream that died, or one that keeps repeating its last
    picture (a camera unplugged behind a running server), goes stale instead of looking fresh.
    """

    def __init__(self, host: str = "192.168.123.164", config_port: int = CONFIG_PORT):
        self.host = host
        self.config = request_config(host, config_port)
        self._lock = threading.Lock()
        self._jpg: dict[str, bytes] = {}
        self._stamp: dict[str, float] = {}
        self._decoded: dict[str, tuple] = {}        # stream -> (jpg it was decoded from, image)
        self.ports = {}
        for stream, default in DEFAULT_PORTS.items():
            cam = (self.config or {}).get(stream) or {}
            if cam.get("enable_zmq", True):
                self.ports[stream] = int(cam.get("zmq_port", default))
        for stream, port in self.ports.items():
            threading.Thread(target=self._reader, args=(stream, port), daemon=True, name=f"image-{stream}").start()

    def _reader(self, stream: str, port: int) -> None:
        import zmq

        sock = zmq.Context.instance().socket(zmq.SUB)
        sock.setsockopt(zmq.RCVHWM, 2)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        sock.connect(f"tcp://{self.host}:{port}")
        while True:
            try:
                jpg = sock.recv()
            except zmq.ZMQError:
                time.sleep(0.1)
                continue
            with self._lock:
                if jpg != self._jpg.get(stream):
                    self._jpg[stream] = jpg
                    self._stamp[stream] = time.time()

    def stamps(self) -> dict[str, float]:
        """{view name: time its newest frame arrived} for the views that have one."""
        with self._lock:
            return {name: self._stamp[stream] for name, (stream, _) in VIEWS.items() if stream in self._stamp}

    def frame(self, name: str):
        """(BGR image, arrival time) of a view, or (None, 0.0) when nothing decodable has arrived."""
        import cv2
        import numpy as np

        stream, half = VIEWS[name]
        with self._lock:
            jpg, stamp = self._jpg.get(stream), self._stamp.get(stream, 0.0)
            cached = self._decoded.get(stream)
        if jpg is None:
            return None, 0.0
        if cached is None or cached[0] is not jpg:
            cached = (jpg, cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR))
            with self._lock:
                self._decoded[stream] = cached
        img = cached[1]
        if img is None:
            return None, 0.0
        if half is not None:
            cam = (self.config or {}).get(stream) or {}
            if cam.get("binocular", True):
                shape = cam.get("image_shape")
                w = (int(shape[1]) if shape else img.shape[1]) // 2
                img = img[:, :w] if half == 0 else img[:, w:]
            elif half == 1:
                return None, 0.0                    # a single-eye head camera has no right image
        return img, stamp

    def frames(self) -> dict:
        """{view name: BGR image} for every view that has a frame."""
        out = {}
        for name in VIEWS:
            img, _ = self.frame(name)
            if img is not None:
                out[name] = img
        return out
