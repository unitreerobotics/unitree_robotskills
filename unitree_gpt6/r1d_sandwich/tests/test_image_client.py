from __future__ import annotations

import importlib.util
import socket
import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

HAVE_DEPS = all(importlib.util.find_spec(m) for m in ("zmq", "cv2", "numpy"))


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@unittest.skipUnless(HAVE_DEPS, "needs pyzmq, OpenCV and numpy")
class ImageClientTest(unittest.TestCase):
    """The client against a local stand-in for the robot's image server (config REP + JPEG PUB sockets)."""

    def setUp(self):
        import cv2
        import numpy as np
        import zmq

        self.zmq, self.cv2, self.np = zmq, cv2, np
        self.ctx = zmq.Context()
        self.ports = {"config": free_port(), "head": free_port(), "left": free_port()}
        self.config = {"camera": {
            "head_camera": {"enable_zmq": True, "zmq_port": self.ports["head"], "binocular": True,
                            "image_shape": [48, 128]},
            "left_wrist_camera": {"enable_zmq": True, "zmq_port": self.ports["left"], "binocular": False,
                                  "image_shape": [48, 64]},
            "right_wrist_camera": {"enable_zmq": False, "zmq_port": 1},
        }}
        self.rep = self.ctx.socket(zmq.REP)
        self.rep.bind(f"tcp://127.0.0.1:{self.ports['config']}")
        self.pubs = {k: self.ctx.socket(zmq.PUB) for k in ("head", "left")}
        for k, s in self.pubs.items():
            s.bind(f"tcp://127.0.0.1:{self.ports[k]}")
        self.running = True
        self.requests = []
        threading.Thread(target=self._serve_config, daemon=True).start()

    def tearDown(self):
        self.running = False
        time.sleep(0.15)
        for s in (self.rep, *self.pubs.values()):
            s.close(linger=0)
        self.ctx.term()

    def _serve_config(self):
        while self.running:
            if self.rep.poll(50):
                self.requests.append(self.rep.recv())
                self.rep.send_json(self.config)

    def jpeg(self, left, right=None, size=(48, 64)):
        np = self.np
        img = np.full((size[0], size[1], 3), left, np.uint8)
        if right is not None:
            img = np.hstack([img, np.full((size[0], size[1], 3), right, np.uint8)])
        return self.cv2.imencode(".jpg", img)[1].tobytes()

    def publish_until(self, client, name, send, timeout=3.0):
        end = time.time() + timeout
        while time.time() < end:
            send()
            time.sleep(0.03)
            img, stamp = client.frame(name)
            if img is not None:
                return img, stamp
        self.fail(f"no frame for {name}")

    def test_reads_config_splits_the_head_pair_and_skips_disabled_streams(self):
        from perception.image_client import ImageClient

        client = ImageClient("127.0.0.1", self.ports["config"])
        self.assertEqual(self.requests, [b"GET_DATA"])
        self.assertEqual(client.ports, {"head_camera": self.ports["head"], "left_wrist_camera": self.ports["left"]})

        head = self.jpeg(40, 200)
        left, _ = self.publish_until(client, "head_left", lambda: self.pubs["head"].send(head))
        right, _ = client.frame("head_right")
        self.assertEqual(left.shape, (48, 64, 3))
        self.assertEqual(right.shape, (48, 64, 3))
        self.assertLess(abs(float(left.mean()) - 40), 3)        # left eye is the left half
        self.assertLess(abs(float(right.mean()) - 200), 3)

        wrist = self.jpeg(120)
        img, _ = self.publish_until(client, "left_wrist", lambda: self.pubs["left"].send(wrist))
        self.assertEqual(img.shape, (48, 64, 3))
        self.assertEqual(client.frame("right_wrist"), (None, 0.0))
        self.assertEqual(sorted(client.frames()), ["head_left", "head_right", "left_wrist"])

    def test_a_repeated_picture_does_not_look_fresh(self):
        from perception.image_client import ImageClient

        client = ImageClient("127.0.0.1", self.ports["config"])
        frozen = self.jpeg(90)
        _, first = self.publish_until(client, "left_wrist", lambda: self.pubs["left"].send(frozen))
        for _ in range(10):                                     # the server keeps sending the same picture
            self.pubs["left"].send(frozen)
            time.sleep(0.02)
        self.assertEqual(client.frame("left_wrist")[1], first)
        self.assertEqual(client.stamps()["left_wrist"], first)
        changed = self.jpeg(91)
        end = time.time() + 3.0
        while client.frame("left_wrist")[1] == first and time.time() < end:
            self.pubs["left"].send(changed)
            time.sleep(0.03)
        self.assertGreater(client.frame("left_wrist")[1], first)

    def test_no_server_falls_back_to_the_default_ports(self):
        from perception import image_client

        self.assertIsNone(image_client.request_config("127.0.0.1", free_port(), timeout_ms=100))
        self.assertEqual(image_client.DEFAULT_PORTS,
                         {"head_camera": 55555, "left_wrist_camera": 55556, "right_wrist_camera": 55557})


if __name__ == "__main__":
    unittest.main()
