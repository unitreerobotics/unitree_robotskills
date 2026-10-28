# -*- coding: utf-8 -*-
"""camera.py -- real-robot camera ZMQ stream reader (no Tk dependency).

Shared by the UI and HeadlessDriver (CLI / MCP):
- CameraStream: ZMQ SUB background thread that keeps only the latest frame;
- request_camera_config: REQ/REP camera config request (port negotiation).
"""

import math
import sys
import threading
import time

try:
    import zmq
except ImportError:
    zmq = None


class CameraStream:
    """ZMQ SUB camera stream reader.

    A background thread receives frames and keeps the latest one for polling.
    Thread safety: _lock guards _latest_frame and its receive metadata."""

    def __init__(self, host: str, port: int, binocular: bool,
                 first_frame_timeout_s: float = 5.0, max_frame_age_s: float = 1.0):
        self.host = host
        self.port = port
        self.binocular = binocular
        self.first_frame_timeout_s = first_frame_timeout_s
        if not math.isfinite(max_frame_age_s) or max_frame_age_s <= 0:
            raise ValueError("max_frame_age_s 须为有限正数")
        self.max_frame_age_s = max_frame_age_s
        self._received_at = None

        self._lock = threading.Lock()
        self._latest_frame = None
        self._running = False
        self._connected = False
        self._error = None
        self._n_recv = 0
        self._thread = None

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def error(self):
        return self._error

    @property
    def n_recv(self) -> int:
        return self._n_recv

    def get_latest_frame(self):
        with self._lock:
            return self._latest_frame

    def snapshot(self):
        """Bytes and receive metadata captured under one lock (not capture time)."""
        with self._lock:
            age = None if self._received_at is None else time.monotonic() - self._received_at
            return {"raw": self._latest_frame, "source_frame_seq": self._n_recv,
                    "received_at": self._received_at, "frame_age_ms": None if age is None else round(age * 1000, 1),
                    "fresh": age is not None and 0 <= age <= self.max_frame_age_s}

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _run(self):
        if zmq is None:
            self._error = "相机流不可用（缺少 pyzmq）"
            self._running = False
            return
        ctx = zmq.Context.instance()
        sub = ctx.socket(zmq.SUB)
        sub.setsockopt(zmq.RCVHWM, 1)
        sub.setsockopt(zmq.LINGER, 0)
        try:
            sub.connect(f"tcp://{self.host}:{self.port}")
        except Exception as e:
            self._error = f"无法连接相机: {e}"
            self._running = False
            return

        sub.setsockopt_string(zmq.SUBSCRIBE, "")
        poller = zmq.Poller()
        poller.register(sub, zmq.POLLIN)

        t_wait = time.monotonic()
        first = None
        while self._running and time.monotonic() - t_wait < self.first_frame_timeout_s:
            if sub in dict(poller.poll(timeout=200)):
                first = sub.recv()
                self._n_recv = 1
                break
        if first is None:
            self._error = f"等待首帧超时 ({self.first_frame_timeout_s:.0f}s)"
            self._running = False
            sub.close(0)
            return

        self._connected = True
        with self._lock:
            self._latest_frame = first
            self._received_at = time.monotonic()

        while self._running:
            if sub in dict(poller.poll(timeout=50)):
                try:
                    data = sub.recv()
                    with self._lock:
                        self._n_recv += 1
                        self._latest_frame = data
                        self._received_at = time.monotonic()
                except Exception as e:
                    print(f"   收帧异常: {e}", file=sys.stderr)

        sub.close(0)


def request_camera_config(host: str, request_port: int, timeout_ms: int = 2000):
    """Request the camera config over REQ/REP (port negotiation); None on failure."""
    if zmq is None:
        print("   相机协议不可用（缺少 pyzmq）", file=sys.stderr)
        return None
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(f"tcp://{host}:{request_port}")
    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)
    try:
        sock.send(b"GET_DATA")
        if sock in dict(poller.poll(timeout=timeout_ms)):
            return sock.recv_json()
    except Exception as e:
        print(f"   请求相机配置失败: {e}", file=sys.stderr)
    finally:
        sock.close(0)
    return None
