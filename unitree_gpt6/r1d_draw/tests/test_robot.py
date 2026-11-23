"""Robot interface against a fake connection: no network, no hardware."""

import json
import struct
import tempfile
import unittest
from pathlib import Path

from unitree_gpt6.r1d_draw.src import schema
from unitree_gpt6.r1d_draw.src.config import normalize_config
from unitree_gpt6.r1d_draw.src.robot import Robot, WiredRobot, png_size

RAW = {
    "device": {
        "host": "192.168.123.161",
        "mac": "aa:bb:cc:dd:ee:01",
        "source_ip": "192.168.123.200",
    },
    "cameras": {
        "head": {
            "host": "192.168.123.161",
            "mac": "aa:bb:cc:dd:ee:01",
            "kind": "rtsp",
            "source": "rtsp://127.0.0.1:8551/video",
        },
        "wrist": {
            "host": "192.168.123.164",
            "interface": "eth10",
            "mac": "aa:bb:cc:dd:ee:02",
            "kind": "v4l2",
            "source": "/dev/v4l/by-id/cam 1",
            "size": "640x480",
        },
    },
}
CONFIG = normalize_config(RAW, "/tmp/task")
PNG = (
    b"\x89PNG\r\n\x1a\n" + struct.pack(">I", 13) + b"IHDR" + struct.pack(">II", 640, 480) + b"rest"
)


def sample(**changes):
    value = {
        "time_unix": 1000.5,
        "tick": 42,
        "ticks_seen": 80,
        "window_s": 0.4,
        "mode_machine": 11,
        "q": [slot / 100 for slot in range(35)],
        "dq": [0.0] * 35,
        "tau": [0.0] * 35,
        "temperature_c": [40.0] * 35,
        "motor_mode": [1] * 35,
        "motor_state": [0] * 35,
        "command_frames": {"lowcmd": 0, "arm_sdk": 0},
    }
    value.update(changes)
    return value


class FakeLink:
    """Records what would run on a host and returns a prepared answer."""

    opened = []

    def __init__(self, **settings):
        self.settings, self.commands, self.closed = settings, [], False
        self.answer = (0, b"", "")
        FakeLink.opened.append(self)

    def run(self, command, *, stdin=None, timeout=20.0):
        self.commands.append((command, stdin))
        return self.answer

    def close(self):
        self.closed = True


class WiredRobotTests(unittest.TestCase):
    def setUp(self):
        FakeLink.opened = []
        self.robot = WiredRobot(CONFIG, link_factory=FakeLink)

    def answer_state(self, value, code=0, noise="some warning\n"):
        link = self.robot._robot_link()
        link.answer = (code, (noise + "R1D_STATE=" + json.dumps(value) + "\n").encode(), "")
        return link

    def test_state_names_the_slots(self):
        link = self.answer_state(sample())
        state = self.robot.state()
        self.assertEqual(state["arms"]["left"], [slot / 100 for slot in range(15, 22)])
        self.assertEqual(state["arms"]["right"], [slot / 100 for slot in range(22, 29)])
        self.assertEqual(state["waist_yaw"], 0.13)
        self.assertEqual(state["head"], [0.29, 0.30])
        self.assertEqual(state["grip"], [0.31, 0.33])
        self.assertEqual(state["lift"], 0.12)
        self.assertEqual((state["tick"], state["time_unix"]), (42, 1000.5))
        self.assertEqual(len(state["q"]), schema.MOTOR_COUNT)
        self.assertFalse(state["commanded"])
        self.assertEqual(state["motor_faults"], [])
        command, source = link.commands[-1]
        self.assertTrue(command.endswith("python3 -B - 0.4"))
        # A shell only treats NAME=value as environment when it precedes the command.
        self.assertTrue(command.startswith("CYCLONEDDS_URI="))
        words = command.split()
        self.assertTrue(all("=" in word for word in words[: words.index("timeout")]))
        self.assertIn("unitree_dds_wrapper", source)

    def test_state_reports_active_commands_and_motor_faults(self):
        faulty = [0] * 35
        faulty[24] = 3
        self.answer_state(sample(command_frames={"lowcmd": 90, "arm_sdk": 0}, motor_state=faulty))
        state = self.robot.state()
        self.assertTrue(state["commanded"])
        self.assertEqual(state["motor_faults"], [24])

    def test_state_rejects_bad_feedback(self):
        for changes in (
            {"mode_machine": 5},
            {"ticks_seen": 1},
            {"q": [0.0] * 29},
            {"q": [float("nan")] + [0.0] * 34},
            {"temperature_c": None},
        ):
            with self.subTest(changes=changes):
                link = self.robot._robot_link()
                text = "R1D_STATE=" + json.dumps(sample(**changes)).replace("NaN", "NaN")
                link.answer = (0, text.encode(), "")
                with self.assertRaises((RuntimeError, ValueError)):
                    self.robot.state()

    def test_state_fails_when_the_onboard_reader_fails(self):
        link = self.robot._robot_link()
        link.answer = (1, b"", "RuntimeError: no LowState received on the robot")
        with self.assertRaisesRegex(RuntimeError, "no LowState"):
            self.robot.state()
        link.answer = (0, b"nothing useful\n", "")
        with self.assertRaises(RuntimeError):
            self.robot.state()

    def test_one_connection_per_host_with_that_hosts_identity(self):
        self.answer_state(sample())
        self.robot.state()
        self.robot.state()
        self.robot._robot_link().answer = (0, PNG, "")
        self.robot.photo("head")
        self.assertEqual(len(FakeLink.opened), 1)
        robot_link = FakeLink.opened[0]
        self.assertEqual(robot_link.settings["mac"], "aa:bb:cc:dd:ee:01")
        self.assertEqual(robot_link.settings["source_ip"], "192.168.123.200")
        with self.assertRaises(RuntimeError):
            self.robot.photo("wrist")  # the new link answers with no image
        self.assertEqual(len(FakeLink.opened), 2)
        wrist_link = FakeLink.opened[1]
        self.assertEqual(
            (
                wrist_link.settings["host"],
                wrist_link.settings["interface"],
                wrist_link.settings["mac"],
            ),
            ("192.168.123.164", "eth10", "aa:bb:cc:dd:ee:02"),
        )

    def test_photo_commands(self):
        self.robot._robot_link().answer = (0, PNG, "")
        self.assertEqual(self.robot.photo("head"), PNG)
        head = self.robot._robot_link().commands[-1][0]
        self.assertTrue(head.startswith("ionice -c3 nice -n 19 timeout"))
        self.assertIn("-rtsp_transport tcp -i rtsp://127.0.0.1:8551/video", head)
        with self.assertRaises(RuntimeError):
            self.robot.photo("wrist")
        wrist = FakeLink.opened[1].commands[-1][0]
        self.assertFalse(wrist.startswith("ionice"))
        self.assertIn("-video_size 640x480 -i '/dev/v4l/by-id/cam 1'", wrist)
        self.assertEqual(png_size(PNG), (640, 480))

    def test_photo_rejects_unknown_camera_and_non_png(self):
        with self.assertRaises(ValueError):
            self.robot.photo("overhead")
        self.assertEqual(FakeLink.opened, [])
        self.robot._robot_link().answer = (0, b"JFIF" * 10, "")
        with self.assertRaises(RuntimeError):
            self.robot.photo("head")
        with self.assertRaises(ValueError):
            png_size(b"JFIF" * 10)

    def test_close_closes_every_connection(self):
        self.robot._robot_link()
        with self.assertRaises(RuntimeError):
            self.robot.photo("wrist")
        self.robot.close()
        self.assertTrue(all(link.closed for link in FakeLink.opened))
        self.assertEqual(len(FakeLink.opened), 2)

    def test_motion_needs_a_model_a_quiet_robot_and_a_session(self):
        with self.assertRaisesRegex(ValueError, "model.urdf"):
            self.robot.take_over()
        with self.assertRaisesRegex(RuntimeError, "take_over first"):
            self.robot.move({"points": []})
        self.robot.release()  # nothing to release: no error, nothing sent
        self.assertEqual(FakeLink.opened, [])
        with tempfile.TemporaryDirectory() as folder:
            urdf = Path(folder) / "r1d.urdf"
            urdf.write_text("<robot/>")
            config = normalize_config(
                {**RAW, "model": {"urdf": str(urdf)}, "storage": {"directory": folder}}, folder
            )
            robot = WiredRobot(config, link_factory=FakeLink)
            link = robot._robot_link()
            busy = sample(command_frames={"lowcmd": 40, "arm_sdk": 0})
            link.answer = (0, ("R1D_STATE=" + json.dumps(busy)).encode(), "")
            with self.assertRaisesRegex(RuntimeError, "another program"):
                robot.take_over()
            # Only the read-only state reader ran: nothing was copied or started.
            self.assertTrue(all("controller.py" not in command for command, _ in link.commands))
            self.assertFalse((Path(folder) / "session.json").exists())

    def test_status_parses_the_controller_file_and_liveness(self):
        with tempfile.TemporaryDirectory() as folder:
            config = normalize_config({**RAW, "storage": {"directory": folder}}, folder)
            robot = WiredRobot(config, link_factory=FakeLink)
            self.assertIsNone(robot.status())
            (Path(folder) / "session.json").write_text('{"directory": "/tmp/r1d_draw/s1"}')
            link = robot._robot_link()
            link.answer = (0, b'{"state": "holding", "error": null}\nR1D_ALIVE\n', "")
            self.assertEqual(robot.status(), {"state": "holding", "error": None, "alive": True})
            self.assertIn("/tmp/r1d_draw/s1/status.json", link.commands[-1][0])
            link.answer = (1, b'{"state": "fault", "error": "x"}\n', "")
            self.assertEqual(robot.status(), {"state": "fault", "error": "x", "alive": False})
            link.answer = (1, b"", "No such file")
            self.assertIsNone(robot.status())

    def test_every_interface_function_exists(self):
        for name in ("state", "photo", "take_over", "move", "release", "close"):
            self.assertTrue(callable(getattr(Robot, name)))


if __name__ == "__main__":
    unittest.main()
