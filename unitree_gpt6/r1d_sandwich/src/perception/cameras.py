"""Latest-frame camera hub over the robot's image server (read-only, see image_client.py).

Names: head_left / head_right (the two halves of the head stereo stream, 544x448 each), left_wrist /
right_wrist (640x480).
Frames are RGB uint8 (the image server delivers JPEG, decoded as BGR and converted here). In --sim without
real cameras, frames come from SimWorld.render().

Receiving 3 JPEG streams at 30 fps in background threads would compete for the GIL with the 250 Hz control
loop in the server process (a real run tripped on a 41 ms cycle gap, 2026-11-16), so the client lives in its
OWN process (spawned, no DDS inside) and the server fetches frames on demand over a pipe.
"""
from __future__ import annotations

import multiprocessing as mp
import threading
import time

import numpy as np

CAMS = ("head_left", "head_right", "left_wrist", "right_wrist")


def _camera_process(host, conn):
    """Child process: answer ("get", name) with (RGB frame, arrival time) and ("status", None) with frame ages."""
    try:
        from perception.image_client import ImageClient
        cam, error = ImageClient(host), None
    except Exception as e:
        cam, error = None, f"camera connect failed: {type(e).__name__}: {e}"
    while True:
        if not conn.poll(0.05):
            continue
        try:
            msg = conn.recv()
        except EOFError:                    # the server is gone
            return
        if msg == "quit":
            return
        kind, arg = msg
        if kind == "get":
            img, stamp = None, 0.0
            if cam is not None and arg in CAMS:
                try:
                    img, stamp = cam.frame(arg)
                    error = None
                except Exception as e:
                    error = f"{type(e).__name__}: {e}"
            # cv2.imdecode gives BGR. Everything downstream is RGB.
            conn.send((None if img is None else np.ascontiguousarray(img[..., ::-1]), stamp))
        elif kind == "status":
            now, seen = time.time(), ({} if cam is None else cam.stamps())
            conn.send(({n: (round(now - seen[n], 2) if n in seen else None) for n in CAMS}, error))


class CameraHub:
    def __init__(self, host="192.168.123.164"):
        ctx = mp.get_context("spawn")
        self._conn, child = ctx.Pipe()
        self._proc = ctx.Process(target=_camera_process, args=(host, child), daemon=True, name="cameras")
        self._proc.start()
        self._lock = threading.Lock()

    def close(self):
        """Ask the camera process to exit; it also exits by itself when this process dies."""
        with self._lock:
            if self._proc.is_alive():
                try:
                    self._conn.send("quit")
                except OSError:
                    pass
                self._proc.join(timeout=2.0)

    def _ask(self, msg, timeout=3.0):
        with self._lock:
            if not self._proc.is_alive():
                return None
            self._conn.send(msg)
            if not self._conn.poll(timeout):
                return None
            return self._conn.recv()

    def get(self, name, max_age=2.0):
        r = self._ask(("get", name))
        if r is None:
            return None
        f, t = r
        if f is None or time.time() - t > max_age:
            return None
        return f

    def all(self, max_age=2.0):
        return {n: self.get(n, max_age) for n in CAMS}

    def status(self):
        r = self._ask(("status", None))
        if r is None:
            return {n: None for n in CAMS} | {"error": "camera process not responding"}
        ages, err = r
        return ages | {"error": err}


class SimCameras:
    def __init__(self, world):
        self.world = world

    def close(self):
        pass

    def get(self, name, max_age=None):
        return self.world.render() if name in CAMS else None

    def all(self, max_age=None):
        img = self.world.render()
        return {n: img for n in CAMS}

    def status(self):
        return {n: 0.0 for n in CAMS} | {"error": None, "sim": True}
