# -*- coding: utf-8 -*-
"""rtp_camera.py -- R1D left-eye RTP-H264 receiver (receive only; never switches robot services).

gst-launch-1.0 receives RTP-H264 over UDP -> decode -> jpegenc -> stdout; frames are split
on the JPEG EOI marker FFD9.
Notes:
  - Same interface as services/camera.py::CameraStream (connected / error / n_recv / binocular /
    get_latest_frame / snapshot / start / stop), so driver and observe need not know the model;
  - No ServiceSwitch (video_hub/stereo_patch_pc1) and no FSM damping switch: those change robot
    state. The operator enables the stereo service in the phone app (or
    services/r1d_camera_services.py switches it); this module only receives passively;
  - An identical JPEG does not refresh freshness (a repeated image is not a new frame);
    the gst process is restarted with a 1 s backoff after it exits.
"""
from __future__ import annotations

import math
import os
import select
import subprocess
import sys
import threading
import time

MAX_PENDING = 16 * 1024 * 1024
UDP_BUFFER_BYTES = 2 * 1024 * 1024     # I-frame bursts overflow the default 208 KB rmem (effective cap is net.core.rmem_max)


def gst_command(host: str, port: int, quality: int = 85) -> list[str]:
    """Never pass corrupted frames upstream on packet loss: output-corrupt=false makes the
    decoder drop damaged frames (better a missing frame than a mosaic for the model / visual
    reflex layer). In real-robot replays ~20% of frames were corrupted, mostly near obstacles
    where image motion is largest. The jitterbuffer only absorbs slight reordering (30 ms cap)."""
    return ["gst-launch-1.0", "-q", "udpsrc", f"address={host}", f"port={int(port)}",
            f"buffer-size={UDP_BUFFER_BYTES}", "!",
            "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96", "!",
            "rtpjitterbuffer", "latency=30", "drop-on-latency=true", "!",
            "rtph264depay", "!", "avdec_h264", "output-corrupt=false", "!",
            "queue", "max-size-buffers=1", "max-size-bytes=0", "max-size-time=0", "leaky=downstream", "!",
            "videoconvert", "!", "jpegenc", f"quality={int(quality)}", "!", "fdsink", "fd=1", "sync=false"]


class RtpCameraStream:
    """R1D monocular RTP stream (port 5002 is the left eye; no half-frame cropping).
    `command` is injectable for tests."""

    binocular = False

    def __init__(self, host: str = "0.0.0.0", port: int = 5002, *,
                 first_frame_timeout_s: float = 15.0, max_frame_age_s: float = 1.0,
                 command: list[str] | None = None, restart_backoff_s: float = 1.0):
        if not 1 <= int(port) <= 65535:
            raise ValueError("相机端口须为 1–65535")
        if not math.isfinite(max_frame_age_s) or max_frame_age_s <= 0:
            raise ValueError("max_frame_age_s 须为有限正数")
        self.host, self.port = host, int(port)
        self.first_frame_timeout_s = first_frame_timeout_s
        self.max_frame_age_s = max_frame_age_s
        self.command = command or gst_command(host, self.port)
        self.restart_backoff_s = restart_backoff_s
        self._lock = threading.Lock()
        self._latest_frame = None
        self._received_at = None
        self._n_recv = 0
        self._connected = False
        self._error = None
        self._running = False
        self._thread = None
        self._proc = None

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
        with self._lock:
            age = None if self._received_at is None else time.monotonic() - self._received_at
            return {"raw": self._latest_frame, "source_frame_seq": self._n_recv,
                    "received_at": self._received_at,
                    "frame_age_ms": None if age is None else round(age * 1000, 1),
                    "fresh": age is not None and 0 <= age <= self.max_frame_age_s}

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, name="rtp-camera", daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        self._kill()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _kill(self):
        p = self._proc
        if p is not None and p.poll() is None:
            p.terminate()
            try:
                p.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                p.kill()

    def _publish(self, jpeg: bytes):
        with self._lock:
            if jpeg == self._latest_frame:
                return
            self._latest_frame = jpeg
            self._received_at = time.monotonic()
            self._n_recv += 1
        self._connected = True
        self._error = None

    def _run(self):
        started = time.monotonic()
        last = None
        while self._running:
            try:
                self._read_once(started)
            except Exception as exc:
                last = f"{type(exc).__name__}: {exc}"
                print(f"[rtp_camera] {last}", file=sys.stderr)
                if self._connected:          # before the first frame only log; driver treats a set error as connect failure
                    self._error = last
            if not self._running:
                break
            if not self._connected and time.monotonic() - started >= self.first_frame_timeout_s:
                self._error = (f"等待首帧超时 ({self.first_frame_timeout_s:.0f}s)：UDP {self.host}:{self.port} 无 RTP-H264；"
                               f"确认机器人已关 video_hub、开 stereo_patch_pc1，且推流目标是本机"
                               + (f"；最近错误 {last}" if last else ""))
                self._running = False
                break
            time.sleep(self.restart_backoff_s)

    def _read_once(self, started):
        try:
            p = subprocess.Popen(self.command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, bufsize=0)
        except FileNotFoundError as exc:
            raise RuntimeError("找不到 gst-launch-1.0（需 GStreamer tools / plugins-good / libav）") from exc
        self._proc = p
        pending = bytearray()
        diag = b""
        streams = [p.stdout, p.stderr]
        try:
            while self._running and p.stdout in streams:
                if not self._connected and time.monotonic() - started >= self.first_frame_timeout_s:
                    return
                ready, _, _ = select.select(streams, [], [], 0.1)
                for s in ready:
                    chunk = os.read(s.fileno(), 65536)
                    if not chunk:
                        streams.remove(s)
                        continue
                    if s is p.stderr:
                        diag = (diag + chunk)[-2048:]
                        continue
                    pending.extend(chunk)
                    latest = None
                    while True:
                        end = pending.find(b"\xff\xd9")
                        if end < 0:
                            break
                        latest = bytes(pending[:end + 2])
                        del pending[:end + 2]
                    if len(pending) > MAX_PENDING:
                        raise RuntimeError("JPEG 输出损坏（缓冲超过 16 MiB）")
                    if latest is not None:
                        start = latest.find(b"\xff\xd8")
                        if start >= 0:
                            self._publish(latest[start:])
            if self._running:
                detail = diag.decode("utf-8", "replace").strip()
                raise RuntimeError(f"gst 接收进程退出：{detail or '无视频输出'}")
        finally:
            self._kill()
            p.stdout.close()
            p.stderr.close()
            self._proc = None
