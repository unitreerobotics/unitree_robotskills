"""Device configuration: defaults, path resolution and rejected inputs."""

import copy
import json
import tempfile
import unittest
from pathlib import Path

from unitree_gpt6.r1d_draw.src.config import load_config, normalize_config

EXAMPLE = Path(__file__).resolve().parents[1] / "config/device.example.json"
MINIMAL = {"device": {"host": "192.168.123.161", "mac": "AA:BB:CC:DD:EE:0F"}}


class DefaultsTests(unittest.TestCase):
    def test_minimal_configuration_gets_defaults(self):
        config = normalize_config(MINIMAL, "/tmp/task")
        device = config["device"]
        self.assertEqual(device["robot_id"], "r1d")
        self.assertEqual(device["user"], "unitree")
        self.assertIsNone(device["source_ip"])
        self.assertEqual(device["mac"], "aa:bb:cc:dd:ee:0f")
        self.assertEqual(device["password_env"], "R1D_SSH_PASSWORD")
        self.assertEqual(device["interface"], "eth0")
        self.assertEqual(config["cameras"], {})
        self.assertNotIn("model", config)
        root = Path.home() / ".local/state/r1d-draw/r1d"
        self.assertEqual(config["storage"]["directory"], str(root))
        self.assertEqual(config["storage"]["controller_lock"], str(root / "controller.lock"))

    def test_input_is_not_modified(self):
        original = copy.deepcopy(MINIMAL)
        normalize_config(MINIMAL, "/tmp/task")
        self.assertEqual(MINIMAL, original)

    def test_normalizing_twice_changes_nothing(self):
        once = normalize_config(MINIMAL, "/tmp/task")
        self.assertEqual(normalize_config(once, "/somewhere/else"), once)

    def test_lock_follows_a_custom_directory(self):
        value = {**MINIMAL, "storage": {"directory": "state"}}
        storage = normalize_config(value, "/tmp/task")["storage"]
        self.assertEqual(storage["directory"], "/tmp/task/state")
        self.assertEqual(storage["controller_lock"], "/tmp/task/state/controller.lock")


class PathTests(unittest.TestCase):
    def test_relative_paths_resolve_against_the_configuration_file(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder).resolve() / "task" / "device.json"
            path.parent.mkdir()
            path.write_text(EXAMPLE.read_text(encoding="utf-8"), encoding="utf-8")
            config = load_config(path)
            self.assertEqual(config["model"]["urdf"], str(path.parent / "models/r1d.urdf"))
            self.assertEqual(config["model"]["mesh_dirs"], [str(path.parent / "models/meshes")])
            self.assertEqual(config["storage"]["directory"], str(path.parent.parent / "runtime"))

    def test_absolute_paths_are_kept(self):
        value = {**MINIMAL, "model": {"urdf": "/opt/r1d/r1d.urdf"}}
        model = normalize_config(value, "/tmp/task")["model"]
        self.assertEqual(model["urdf"], "/opt/r1d/r1d.urdf")
        self.assertEqual(model["mesh_dirs"], [])


class CameraTests(unittest.TestCase):
    CAMERA = {
        "host": "192.168.123.164",
        "interface": "eth10",
        "mac": "11:22:33:44:55:66",
        "kind": "v4l2",
        "source": "/dev/video0",
    }

    def cameras(self, **changes):
        camera = {k: v for k, v in {**self.CAMERA, **changes}.items() if v is not None}
        return normalize_config({**MINIMAL, "cameras": {"wrist": camera}}, "/tmp/task")["cameras"]

    def test_login_defaults_come_from_the_device(self):
        value = {
            "device": {**MINIMAL["device"], "user": "robot", "password_env": "MY_PASSWORD"},
            "cameras": {"wrist": dict(self.CAMERA)},
        }
        camera = normalize_config(value, "/tmp/task")["cameras"]["wrist"]
        self.assertEqual((camera["user"], camera["password_env"]), ("robot", "MY_PASSWORD"))
        self.assertEqual(camera["size"], "1920x1080")
        self.assertEqual(camera["interface"], "eth10")

    def test_rtsp_camera_has_no_size(self):
        camera = self.cameras(kind="rtsp", source="rtsp://127.0.0.1:8551/video")["wrist"]
        self.assertNotIn("size", camera)

    def test_rejected_cameras(self):
        for changes in (
            {"kind": "usb"},
            {"kind": None},
            {"source": ""},
            {"source": None},
            {"mac": "zz:22:33:44:55:66"},
            {"mac": None},
            {"host": "camera-pc"},
            {"size": "big"},
            {"size": "0x480"},
            {"password": "123"},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.cameras(**changes)
        with self.assertRaises(ValueError):
            normalize_config({**MINIMAL, "cameras": {"wrist": "video0"}}, "/tmp/task")
        with self.assertRaises(ValueError):
            normalize_config({**MINIMAL, "cameras": {"wrist cam": dict(self.CAMERA)}}, "/tmp/task")


class RejectionTests(unittest.TestCase):
    def reject(self, **device):
        value = {"device": {**MINIMAL["device"], **device}}
        with self.assertRaises(ValueError):
            normalize_config(value, "/tmp/task")

    def test_device_fields(self):
        self.reject(host="robot.local")
        self.reject(host="")
        self.reject(host=None)
        self.reject(source_ip="enp4s0")
        self.reject(mac="aa:bb:cc:dd:ee")
        self.reject(mac="aabbccddeeff")
        self.reject(mac=None)
        self.reject(robot_id="r1d/../x")
        self.reject(robot_id="")
        self.reject(user="")
        self.reject(password_env="my password")
        self.reject(interface="")

    def test_password_must_not_be_stored(self):
        self.reject(password="not-stored-here")

    def test_structure(self):
        for bad in ([], "x", None):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                normalize_config(bad)
        with self.assertRaises(ValueError):
            normalize_config({**MINIMAL, "gains": {}})
        with self.assertRaises(ValueError):
            normalize_config({**MINIMAL, "storage": []})
        with self.assertRaises(ValueError):
            normalize_config({**MINIMAL, "model": {}})
        with self.assertRaises(ValueError):
            normalize_config({**MINIMAL, "model": {"urdf": "a.urdf", "mesh_dirs": "meshes"}})

    def test_example_file_is_valid_json_with_only_known_sections(self):
        value = json.loads(EXAMPLE.read_text(encoding="utf-8"))
        self.assertEqual(set(value), {"device", "cameras", "model", "storage"})
        normalize_config(value, EXAMPLE.parent)


if __name__ == "__main__":
    unittest.main()
