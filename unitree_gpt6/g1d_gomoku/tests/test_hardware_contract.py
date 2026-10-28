"""Connection-free contract tests for the optional G1 Gomoku hardware tools.

The tests in this module deliberately do not construct a robot model, create a
DDS participant, open a camera, or send a command.  The larger hardware
dependencies are optional; tests which exercise those adapters are skipped
when their explicit optional dependencies are unavailable.
"""

from __future__ import annotations

import contextlib
import hashlib
import importlib
import importlib.util
import io
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

import yaml

HARDWARE_PACKAGE = "unitree_gpt6.g1d_gomoku.src.hardware"
OPTIONAL_DEPENDENCIES = {
    "camera_web": ("cv2", "numpy", "zmq"),
    "dds_robot": ("numpy",),
    "kinematics": ("numpy", "scipy"),
    "arm_session": ("numpy", "scipy", "cv2", "zmq"),
    "continuous_gomoku_session": ("numpy", "scipy", "cv2", "zmq"),
    "gomoku_live_control": ("numpy", "scipy", "cv2", "zmq"),
    "reset_right": ("numpy", "scipy"),
}


def optional_module(name: str):
    """Import one optional hardware module, returning ``None`` when unavailable."""

    module_name = f"{HARDWARE_PACKAGE}.{name}"
    if any(importlib.util.find_spec(dep) is None for dep in OPTIONAL_DEPENDENCIES.get(name, ())):
        return None
    # Once the module and its optional dependencies exist, let import errors
    # surface: a broken migrated module must fail the contract tests.
    return importlib.import_module(module_name)


def valid_reference() -> dict:
    return {
        "joint_names": [f"joint_{i}" for i in range(14)],
        "joint_positions": [0.0] * 14,
        "waist_q": [0.0, 0.0, 0.0],
        "grippers": {"left": 1.0, "right": 2.0},
    }


class CommonConfigurationTests(unittest.TestCase):
    def test_runtime_paths_are_resolved_from_config_and_reset_is_existing(self):
        common = optional_module("common")
        if common is None:
            self.skipTest("hardware common module is not available")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            reset = root / "reset.json"
            reset.write_text(json.dumps(valid_reference()), encoding="utf-8")
            config_path = root / "local.yaml"
            config_path.write_text(
                "runtime:\n  directory: ./session\n  reset_state: ./reset.json\n",
                encoding="utf-8",
            )
            config = common.load_config(config_path)
            self.assertEqual(common.runtime_dir(config), root / "session")
            self.assertEqual(
                common.controller_lock_path(config), root / "session" / ".arm_session.lock"
            )
            self.assertEqual(common.reset_state_path(config), reset)
            self.assertEqual(config["_config_path"], str(config_path.resolve()))

    def test_missing_reset_reference_is_rejected_without_device_access(self):
        common = optional_module("common")
        if common is None:
            self.skipTest("hardware common module is not available")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_path = root / "local.yaml"
            config_path.write_text(
                "runtime:\n  directory: ./session\n  reset_state: ./missing.json\n",
                encoding="utf-8",
            )
            config = common.load_config(config_path)
            with self.assertRaises(ValueError):
                common.reset_state_path(config)


class PreflightTests(unittest.TestCase):
    def _config(self, root: Path, reference: Path) -> dict:
        source = Path(__file__).resolve().parents[1] / "config" / "hardware.example.yaml"
        config = yaml.safe_load(source.read_text(encoding="utf-8"))
        config["_config_path"] = str((root / "local.yaml").resolve())
        config["runtime"]["directory"] = "./session"
        config["runtime"]["reset_state"] = str(reference)
        geometry = {
            "frame": "pelvis",
            "tool_tip_offset_m": [0.1, 0.0, 0.0],
            "high_route_min_tcp_z_m": 0.3,
            "high_route_min_tip_z_m": 0.2,
            "rest_corridor": {
                side: {"min_m": [-1, -1, 0], "max_m": [1, 1, 1]} for side in ("left", "right")
            },
        }
        scene = root / "scene.json"
        scene.write_text(json.dumps(geometry))
        config["runtime"]["scene_calibration"] = str(scene)
        config["robot"]["network_interface"] = "eth0"
        config["robot"]["robot_model"] = "G1-D"
        config["robot"]["mode"] = "hardware"
        config["camera"]["host"] = "camera.local"
        return config

    def test_preflight_validates_reset_schema_and_reports_zero_commands(self):
        main = optional_module("main")
        if main is None:
            self.skipTest("hardware main module is not available")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            reference = root / "reset.json"
            reference.write_text(json.dumps(valid_reference()), encoding="utf-8")
            config = self._config(root, reference)
            with mock.patch.object(main, "module_available", return_value=True):
                result = main.preflight(config)
            self.assertEqual(result["task_status"], "preflight")
            self.assertTrue(result["configuration_valid"])
            self.assertEqual(result["hardware_commands_sent"], 0)
            self.assertFalse(result["task_verified"])
            self.assertTrue(all(result["dependencies"].values()))
            self.assertNotIn("unitree_deploy", result["dependencies"])
            self.assertTrue(result["model_assets"]["available"])
            self.assertEqual(result["model_assets"]["mesh_count"], 49)

    def test_preflight_rejects_invalid_reset_schema(self):
        main = optional_module("main")
        if main is None:
            self.skipTest("hardware main module is not available")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            reference = root / "reset.json"
            bad = valid_reference()
            bad["joint_positions"] = [0.0] * 13
            reference.write_text(json.dumps(bad), encoding="utf-8")
            with self.assertRaises(ValueError):
                main.preflight(self._config(root, reference))

    def test_preflight_missing_dependencies_never_loads_a_hardware_tool(self):
        main = optional_module("main")
        common = optional_module("common")
        self.addCleanup(common.configure, None)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            reference = root / "reset.json"
            reference.write_text(json.dumps(valid_reference()), encoding="utf-8")
            config_path = root / "local.yaml"
            config_path.write_text(yaml.safe_dump(self._config(root, reference)), encoding="utf-8")
            output = io.StringIO()
            with (
                mock.patch.object(main, "module_available", return_value=False),
                mock.patch.object(
                    main.importlib,
                    "import_module",
                    side_effect=AssertionError("preflight must not load device tools"),
                ),
                contextlib.redirect_stdout(output),
            ):
                code = main.main(["--config", str(config_path), "preflight"])
            report = json.loads(output.getvalue())
            self.assertEqual(code, 1)
            self.assertEqual(report["task_status"], "preflight")
            self.assertTrue(report["configuration_valid"])
            self.assertEqual(report["hardware_commands_sent"], 0)
            self.assertFalse(report["task_verified"])
            self.assertFalse(any(report["dependencies"].values()))

    def test_cli_preflight_reports_a_structured_failure_without_importing_sdk(self):
        main = optional_module("main")
        if main is None:
            self.skipTest("hardware main module is not available")
        self.addCleanup(optional_module("common").configure, None)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "local.yaml"
            config.write_text(
                "runtime:\n  directory: ./session\n  reset_state: ./reset.json\n"
                "robot:\n  backend: dds\n  network_interface: eth0\n"
                "camera:\n  host: camera.local\n",
                encoding="utf-8",
            )
            # A missing reset state is a configuration error, and must still
            # produce JSON and zero commands rather than touching a device.
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                code = main.main(["--config", str(config), "preflight"])
            self.assertEqual(code, 1)
            report = json.loads(output.getvalue())
            self.assertEqual(report["task_status"], "failed")
            self.assertEqual(report["hardware_commands_sent"], 0)
            self.assertFalse(report["task_verified"])


class DDSStateFreshnessTests(unittest.TestCase):
    def test_stale_state_is_rejected(self):
        dds = optional_module("dds_robot")
        if dds is None:
            self.skipTest("DDS adapter or NumPy is not available")
        robot = dds.DDSRobot.__new__(dds.DDSRobot)
        robot._lock = threading.Lock()
        robot._receivers = {}
        robot._state = {"arm": (object(), time.monotonic() - 10.0)}
        robot.timeout = 0.01
        with self.assertRaises(TimeoutError):
            robot._get("arm")

    def test_fresh_state_is_returned_without_connecting(self):
        dds = optional_module("dds_robot")
        if dds is None:
            self.skipTest("DDS adapter or NumPy is not available")
        value = object()
        robot = dds.DDSRobot.__new__(dds.DDSRobot)
        robot._lock = threading.Lock()
        robot._receivers = {}
        robot._state = {"arm": (value, time.monotonic())}
        robot.timeout = 1.0
        self.assertIs(robot._get("arm"), value)


class LiveControlAuthorizationTests(unittest.TestCase):
    def test_read_rejects_controller_from_a_different_configuration(self):
        control = optional_module("gomoku_live_control")
        if control is None:
            self.skipTest("optional numerical or camera dependencies are not installed")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_path = str(root / "local.yaml")
            status = {
                "config_path": str(root / "other.yaml"),
                "phase": "holding",
                "publishing": True,
                "fault": None,
                "feedback": {"at_unix": 100.0},
            }
            with (
                mock.patch.object(
                    control, "load_config", return_value={"_config_path": config_path}
                ),
                mock.patch.object(control.time, "time", return_value=100.0),
                mock.patch.object(Path, "read_text", return_value=json.dumps(status)),
            ):
                with self.assertRaises(ValueError):
                    control.read(root)
                status["config_path"] = config_path
            with (
                mock.patch.object(
                    control, "load_config", return_value={"_config_path": config_path}
                ),
                mock.patch.object(control.time, "time", return_value=100.0),
                mock.patch.object(Path, "read_text", return_value=json.dumps(status)),
            ):
                self.assertEqual(control.read(root), status)

    def test_wait_release_does_not_finish_before_return_record_and_holding(self):
        control = optional_module("gomoku_live_control")
        if control is None:
            self.skipTest("optional numerical or camera dependencies are not installed")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            request = root / "inbox" / "request.json"
            return_directory = root / "released"
            base = {"fault": None, "publishing": True, "feedback": {"at_unix": 100.0}}
            statuses = [
                {**base, "phase": "holding", "stage": "release"},
                {**base, "phase": "returning", "stage": None},
                {**base, "phase": "holding", "stage": None},
            ]

            def exists(path):
                return path.suffix == ".done"

            with (
                mock.patch.object(control.time, "time", return_value=100.0),
                mock.patch.object(control.time, "monotonic", side_effect=[0.0, 0.1, 0.2, 0.3]),
                mock.patch.object(control.time, "sleep") as sleep,
                mock.patch.object(Path, "exists", autospec=True, side_effect=exists),
                mock.patch.object(Path, "is_file", side_effect=[False, True, True]),
                mock.patch.object(Path, "read_text", side_effect=[json.dumps(s) for s in statuses]),
            ):
                result = control.wait_request(
                    root, request, "release", return_directory=return_directory
                )
            self.assertEqual(result, statuses[-1])
            self.assertEqual(sleep.call_count, 2)

    def test_owner_without_execute_is_rejected_before_device_construction(self):
        control = optional_module("gomoku_live_control")
        if control is None:
            self.skipTest("optional numerical or camera dependencies are not installed")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = {
                "_config_path": str(root / "local.yaml"),
                "runtime": {"directory": str(root / "runtime")},
            }
            with (
                mock.patch.object(control, "load_config", return_value=config),
                mock.patch.object(control, "ContinuousSession") as session,
                mock.patch.object(control, "DDSRobot") as robot,
                mock.patch.object(control, "Kinematics") as model,
            ):
                with self.assertRaisesRegex(ValueError, "require --execute"):
                    control.main(["--root", str(root / "round"), "owner"])
            session.assert_not_called()
            robot.assert_not_called()
            model.assert_not_called()


class ArmPlanContractTests(unittest.TestCase):
    def test_pending_inbox_is_rejected_before_model_or_device_construction(self):
        arm_session = optional_module("arm_session")
        if arm_session is None:
            self.skipTest("optional numerical or camera dependencies are not installed")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            inbox = root / "session" / "inbox"
            inbox.mkdir(parents=True)
            pending = inbox / "unhandled.json"
            pending.write_text(json.dumps({"op": "stage"}), encoding="utf-8")
            config = {
                "_config_path": str(root / "local.yaml"),
                "runtime": {"directory": str(root / "runtime")},
            }
            with (
                mock.patch.object(arm_session, "Kinematics") as model,
                mock.patch.object(arm_session, "DDSRobot") as robot,
                mock.patch.object(arm_session, "CameraCapture") as camera,
            ):
                with self.assertRaisesRegex(ValueError, "pending JSON requests"):
                    arm_session.Session(root / "session", execute=True, config=config)
            model.assert_not_called()
            robot.assert_not_called()
            camera.assert_not_called()
            self.assertTrue(pending.is_file())

    @staticmethod
    def _fake_kinematics():
        import numpy as np

        class FakeKinematics:
            names = [f"joint_{i}" for i in range(14)]

            def check_joints(self, q):
                result = np.asarray(q, dtype=float)
                if result.shape != (14,):
                    raise ValueError("bad joints")
                return result

        return FakeKinematics()

    def test_validate_plan_holds_left_arm_and_rejects_speed(self):
        arm_session = optional_module("arm_session")
        if arm_session is None:
            self.skipTest("migrated arm_session is not available")
        import numpy as np

        kin = self._fake_kinematics()
        config = {"joint_tolerance": 0.1, "max_joint_speed": 1.0}
        valid = {
            "seed": [0.0] * 14,
            "clearance_review": {"passed": True},
            "steps": [
                {
                    "joint_positions": [0.0] * 14,
                    "duration": 1.0,
                    "targets": {"right": [0.0] * 6},
                }
            ],
        }
        with mock.patch.object(arm_session, "arrival_report", return_value={"reached": True}):
            arm_session.validate_plan(valid, kin, np.zeros(14), config, np.zeros(7))

            left_change = json.loads(json.dumps(valid))
            left_change["steps"][0]["joint_positions"][0] = 0.2
            with self.assertRaises(ValueError, msg="left arm must remain held"):
                arm_session.validate_plan(left_change, kin, np.zeros(14), config, np.zeros(7))

            too_fast = json.loads(json.dumps(valid))
            too_fast["steps"][0]["joint_positions"][7] = 2.0
            too_fast["steps"][0]["duration"] = 1.0
            with self.assertRaises(ValueError, msg="joint speed limit must be enforced"):
                arm_session.validate_plan(too_fast, kin, np.zeros(14), config, np.zeros(7))


class ReleaseAndResetContractTests(unittest.TestCase):
    def _release_case(self, module, root):
        """A held command and two JSON plans; no hardware constructors run."""
        import numpy as np

        session = object.__new__(module.ContinuousSession)
        session.phase, session.fault = "holding", None
        session.started, session.execute = True, True
        session.q = np.zeros(14)
        session.left = np.zeros(7)
        session.initial_waist = np.zeros(3)
        session.kin = self._fake_kinematics()
        session.kin.set_waist = mock.Mock()
        session.robot = mock.Mock()
        session.robot.arm_q.return_value = session.q.copy()
        session.cfg = {
            "joint_tolerance": 0.1,
            "max_joint_speed": 1.0,
            "grippers": {"right": {"min": 0, "max": 5.45, "open": 5.45, "close": 0}},
        }
        session.grippers = {"right": 0.85}
        reference = root / "reset.json"
        saved = valid_reference()
        saved["joint_positions"][7:] = [0.1] * 7
        reference.write_text(json.dumps(saved), encoding="utf-8")
        session.config = {"runtime": {"reset_state": str(reference)}}
        session.full_cfg = session.config
        session.reset_state = reference
        release = {
            "seed": session.q.tolist(),
            "clearance_review": {"passed": True},
            "steps": [
                {
                    "joint_positions": session.q.tolist(),
                    "duration": 1.0,
                    "targets": {"right": [0.0] * 6},
                    "grippers": {"right": 1.25},
                }
            ],
        }
        returned = {
            "seed": session.q.tolist(),
            "locked_waist_q": session.initial_waist.tolist(),
            "clearance_review": {"passed": True},
            "steps": [
                {
                    "joint_positions": saved["joint_positions"],
                    "duration": 1.0,
                    "targets": {"right": [0.0] * 6},
                    "grippers": {},
                }
            ],
        }
        request = {
            "op": "release_and_return",
            "plan": str(root / "release.json"),
            "return_plan": str(root / "return.json"),
            "directory": str(root / "out"),
            "release_authorization": str(root / "out_release_authorization.json"),
        }
        return session, release, returned, reference, request

    def _write_release_case(self, request, release, returned):
        Path(request["plan"]).write_text(json.dumps(release), encoding="utf-8")
        Path(request["return_plan"]).write_text(json.dumps(returned), encoding="utf-8")
        authorization = {
            "approved": True,
            "directory": str(Path(request["directory"]).resolve()),
            "release_plan_sha256": hashlib.sha256(Path(request["plan"]).read_bytes()).hexdigest(),
            "return_plan_sha256": hashlib.sha256(
                Path(request["return_plan"]).read_bytes()
            ).hexdigest(),
        }
        Path(request["release_authorization"]).write_text(
            json.dumps(authorization), encoding="utf-8"
        )

    def test_release_and_return_accepts_only_fixed_release_and_invariant_return(self):
        module = optional_module("continuous_gomoku_session")
        if module is None:
            self.skipTest("optional numerical or camera dependencies are not installed")
        with tempfile.TemporaryDirectory() as directory:
            session, release, returned, reference, request = self._release_case(
                module, Path(directory)
            )
            release["steps"][0]["grippers"]["right"] = 1.7
            self._write_release_case(request, release, returned)
            with (
                mock.patch.object(module, "reset_state_path", return_value=reference),
                mock.patch.object(module, "validate_plan"),
                mock.patch.object(module.Session, "command") as stage,
            ):
                session.command(request)
            stage.assert_called_once()
            stage_request = stage.call_args.args[0]
            self.assertEqual(stage_request["op"], "stage")
            self.assertFalse(stage_request["capture_images"])
            self.assertEqual(session.finish_pending["plan"], returned)
            self.assertEqual(release["steps"][0]["grippers"], {"right": 1.7})
            self.assertTrue(all(not step.get("grippers") for step in returned["steps"]))

    def test_release_rejects_arm_motion_even_with_correct_gripper_goal(self):
        module = optional_module("continuous_gomoku_session")
        if module is None:
            self.skipTest("optional numerical or camera dependencies are not installed")
        with tempfile.TemporaryDirectory() as directory:
            session, release, returned, reference, request = self._release_case(
                module, Path(directory)
            )
            release["steps"][0]["joint_positions"][7] = 0.02
            self._write_release_case(request, release, returned)
            with (
                mock.patch.object(module, "reset_state_path", return_value=reference),
                mock.patch.object(module, "validate_plan"),
                mock.patch.object(module.Session, "command") as stage,
            ):
                with self.assertRaises(ValueError):
                    session.command(request)
            stage.assert_not_called()

    def test_release_authorization_is_bound_to_stage_and_unchanged_plan_files(self):
        module = optional_module("continuous_gomoku_session")
        if module is None:
            self.skipTest("optional numerical or camera dependencies are not installed")
        for invalid in ("approval", "directory", "release_hash", "return_hash", "path"):
            with self.subTest(invalid=invalid), tempfile.TemporaryDirectory() as directory:
                session, release, returned, reference, request = self._release_case(
                    module, Path(directory)
                )
                self._write_release_case(request, release, returned)
                authorization_path = Path(request["release_authorization"])
                authorization = json.loads(authorization_path.read_text())
                if invalid == "approval":
                    authorization["approved"] = False
                elif invalid == "directory":
                    authorization["directory"] = str(Path(directory) / "another_stage")
                elif invalid == "release_hash":
                    authorization["release_plan_sha256"] = "old_release_plan"
                elif invalid == "return_hash":
                    authorization["return_plan_sha256"] = "old_return_plan"
                else:
                    request["release_authorization"] = str(Path(directory) / "other.json")
                    authorization_path = Path(request["release_authorization"])
                authorization_path.write_text(json.dumps(authorization), encoding="utf-8")
                with (
                    mock.patch.object(module, "reset_state_path", return_value=reference),
                    mock.patch.object(module, "validate_plan"),
                    mock.patch.object(module.Session, "command") as stage,
                ):
                    with self.assertRaises(ValueError):
                        session.command(request)
                stage.assert_not_called()

    def test_return_rejects_wrong_seed_endpoint_or_gripper_change(self):
        module = optional_module("continuous_gomoku_session")
        if module is None:
            self.skipTest("optional numerical or camera dependencies are not installed")
        for invalid in ("seed", "waist", "endpoint", "gripper"):
            with self.subTest(invalid=invalid), tempfile.TemporaryDirectory() as directory:
                session, release, returned, reference, request = self._release_case(
                    module, Path(directory)
                )
                if invalid == "seed":
                    returned["seed"][7] = 0.02
                elif invalid == "waist":
                    returned["locked_waist_q"][0] = 0.02
                elif invalid == "endpoint":
                    returned["steps"][-1]["joint_positions"][7] = 0.2
                else:
                    returned["steps"][0]["grippers"] = {"right": 1.0}
                self._write_release_case(request, release, returned)
                with (
                    mock.patch.object(module, "reset_state_path", return_value=reference),
                    mock.patch.object(module, "validate_plan"),
                    mock.patch.object(module.Session, "command") as stage,
                ):
                    with self.assertRaises(ValueError):
                        session.command(request)
                stage.assert_not_called()

    def test_release_holding_starts_return_immediately_without_a_second_gripper_gate(self):
        module = optional_module("continuous_gomoku_session")
        if module is None:
            self.skipTest("optional numerical or camera dependencies are not installed")
        import numpy as np

        with tempfile.TemporaryDirectory() as directory:
            session, _release, returned, _reference, _request = self._release_case(
                module, Path(directory)
            )
            session.restore_left_target = None
            session.finish_pending = {"plan": returned, "directory": directory}
            session.release_settled_at = None
            session.camera = mock.Mock()
            session.cameras_enabled = True
            session.robot.gripper_q.return_value = 1.20
            session.robot.waist_q.return_value = np.zeros(3)
            session.grippers = {"right": 1.25}
            session.kin.tau = mock.Mock(return_value=np.zeros(14))
            session.feedback = mock.Mock(return_value={"feedback": "synthetic"})
            with (
                mock.patch.object(module.Session, "advance"),
                mock.patch.object(module.time, "monotonic", return_value=2.0),
                mock.patch.object(module, "snapshot") as snapshot,
                mock.patch.object(module, "write_json") as write_json,
            ):
                session.advance()
                self.assertEqual(session.phase, "returning")
                self.assertIsNone(session.finish_pending)
                snapshot.assert_called_once_with(
                    session.camera, Path(directory) / "released_images"
                )
                session.camera.close.assert_called_once()
                # A complete return interpolation does not issue a second
                # gripper command and does not inspect the saved image.
                session.begin = 0.0
                session.advance()
                self.assertEqual(session.phase, "holding")
                self.assertEqual(session.grippers, {"right": 1.25})
                self.assertIsNone(session.return_plan)
                snapshot.assert_called_once()
                write_json.assert_called_once()

    def test_release_requires_fixed_arm_gripper_command_and_healthy_hold(self):
        session_module = optional_module("continuous_gomoku_session")
        if session_module is None:
            self.skipTest("migrated continuous Gomoku session is not available")
        session = object.__new__(session_module.ContinuousSession)
        session.phase, session.fault = "holding", None
        session.started, session.execute = True, True
        session.cfg = {"grippers": {"right": {"min": 0, "max": 5.45, "open": 5.45, "close": 0}}}
        session.grippers = {"right": 0.85}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            release = root / "release.json"
            release.write_text(
                json.dumps({"seed": [0.0] * 14, "steps": [{"grippers": {"right": 0.5}}]}),
                encoding="utf-8",
            )
            returned = root / "return.json"
            returned.write_text(json.dumps({"seed": [0.0] * 14, "steps": []}), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "fixed-arm release"):
                session.command(
                    {
                        "op": "release_and_return",
                        "plan": str(release),
                        "return_plan": str(returned),
                        "directory": str(root / "out"),
                    }
                )
        session.phase, session.fault = "paused_holding", None
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            release = root / "release.json"
            release.write_text(
                json.dumps({"seed": [0.0] * 14, "steps": [{"grippers": {"right": 1.25}}]}),
                encoding="utf-8",
            )
            returned = root / "return.json"
            returned.write_text(json.dumps({"seed": [0.0] * 14, "steps": []}), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "healthy hold"):
                session.command(
                    {
                        "op": "release_and_return",
                        "plan": str(release),
                        "return_plan": str(returned),
                        "directory": str(root / "out"),
                    }
                )

    def test_reset_plan_keeps_left_arm_when_no_live_grippers_are_supplied(self):
        reset = optional_module("reset_right")
        if reset is None:
            self.skipTest("migrated reset_right is not available")
        import numpy as np

        kin = self._fake_kinematics()
        saved = valid_reference()
        saved["joint_positions"] = [float(i) for i in range(14)]
        config = {
            "max_joint_speed": 1.0,
            "gripper_speed": 1.0,
            "grippers": {
                "left": {"min": 0.0, "max": 5.45},
                "right": {"min": 0.0, "max": 5.45},
            },
        }
        # build_plan only needs these FK values to produce recorded targets;
        # no URDF or robot is constructed.
        kin.set_waist = lambda waist: setattr(kin, "waist", np.asarray(waist, dtype=float))
        kin.fk = lambda q: {"left": np.eye(4), "right": np.eye(4)}
        with mock.patch.object(reset, "matrix_pose", return_value=[0.0] * 6):
            plan = reset.build_plan(saved, np.zeros(14), [0.0, 0.0, 0.0], kin, config, 1.0)
        self.assertEqual(plan["steps"][0]["joint_positions"][:7], [0.0] * 7)
        self.assertEqual(plan["steps"][0]["joint_positions"][7:], [float(i) for i in range(7, 14)])
        self.assertEqual(plan["steps"][0]["grippers"], {})

    def _fake_kinematics(self):
        import numpy as np

        class FakeKinematics:
            names = [f"joint_{i}" for i in range(14)]

            def check_joints(self, q):
                result = np.asarray(q, dtype=float)
                if result.shape != (14,):
                    raise ValueError("bad joints")
                return result

        return FakeKinematics()


class NumericalDiagnosticTests(unittest.TestCase):
    def test_missing_casadi_binding_reports_the_real_cause_without_loading_devices(self):
        main = optional_module("main")

        def fake_import(name):
            if name == "pinocchio.casadi":
                raise ModuleNotFoundError("No module named 'pinocchio.casadi'")
            if name not in ("numpy", "scipy", "pinocchio", "casadi"):
                raise AssertionError(f"Unexpected device/tool import: {name}")
            return type("NumericalModule", (), {"__version__": "test"})()

        with mock.patch.object(main.importlib, "import_module", side_effect=fake_import):
            report = main.model_check()
        self.assertFalse(report["model_initialized"])
        self.assertEqual(report["hardware_commands_sent"], 0)
        self.assertIn(
            "pinocchio.casadi", report["numerical_modules"]["pinocchio.casadi"]["error"]["message"]
        )
        self.assertTrue(report["python_executable"])

    def test_kinematics_preserves_interpreter_and_original_import_error(self):
        kin = optional_module("kinematics")
        if kin is None:
            self.skipTest("numerical dependencies are not installed")
        import builtins
        import sys

        original = builtins.__import__

        def missing_binding(name, *args, **kwargs):
            if name == "unitree_gpt6.g1d_gomoku.models.g1.arm_model":
                raise ImportError("cannot import name 'casadi' from 'pinocchio'")
            return original(name, *args, **kwargs)

        with mock.patch("builtins.__import__", side_effect=missing_binding):
            with self.assertRaises(RuntimeError) as error:
                kin.Kinematics()
        self.assertIn(sys.executable, str(error.exception))
        self.assertIn("cannot import name 'casadi'", str(error.exception))
        self.assertIn("model-check", str(error.exception))


if __name__ == "__main__":
    unittest.main()
