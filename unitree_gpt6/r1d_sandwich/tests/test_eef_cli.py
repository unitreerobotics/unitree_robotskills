from __future__ import annotations

import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

SKILL_ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(SKILL_ROOT / "src"), str(SKILL_ROOT / "scripts")]

import eef_cli  # noqa: E402


VALID_PAYLOAD = {
    "note": "lettuce",
    "segments": [
        {
            "side": "left",
            "speed": 0.15,
            "waypoints": [
                {"pos": [0.4, 0.1, 0.7], "rpy_deg": [0, 35, -45], "linear": True}
            ],
        }
    ],
}


class FakeClient:
    instances = []

    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.moves = []
        FakeClient.instances.append(self)

    grasp_opening = 0.18          # what a grasp reports; class attribute so a test can set it before the CLI runs
    refuse_move = None            # index of the move to refuse

    fixed_lift = 0.522

    def health(self):
        return {"mode": "sim", "fixed_lift": self.fixed_lift}

    def move(self, side, pos, **kwargs):
        self.moves.append((side, pos, kwargs))
        if self.refuse_move == len(self.moves) - 1:
            return {"ok": False, "executed": False, "code": "unreachable", "reason": "pose out of reach"}
        return {"ok": True, "executed": not kwargs.get("check_only", False)}

    def gripper(self, side, opening=None, *, action=None):
        self.moves.append((side, "gripper", {"opening": opening, "action": action}))
        return {"ok": True, "executed": True, "opening": self.grasp_opening if action == "grasp" else opening}

    def state(self):
        return {"tcp": {s: {"pos": [0.4, 0.0, 0.7]} for s in ("left", "right")},
                "gripper": {s: {"opening": 0.02, "holding": False} for s in ("left", "right")},
                "lift_m": FakeClient.lift_m, "head_pitch": FakeClient.head_pitch, "head_yaw": 0.0, "waist_rad": 0.0}

    def move_both(self, left, right, **kwargs):
        self.moves.append(("both", (left, right), kwargs))
        return {"ok": True, "executed": True}

    lift_m, head_pitch = 0.573, 0.003     # where the fake robot starts; a test may change them before the CLI runs

    def poses(self):
        return {"poses": {}, "scene": {"table_z": 0.595}}

    def lift(self, height_m):
        self.moves.append(("lift", height_m, {}))
        FakeClient.lift_m = height_m
        return {"ok": True, "executed": True}

    def head(self, pitch=None, yaw=None):
        self.moves.append(("head", pitch, {"yaw": yaw}))
        FakeClient.head_pitch = pitch + 0.025        # the real head settles a little past the command
        return {"ok": True, "executed": True}

    def project(self, cam, pixels, z=None, method="auto"):
        self.moves.append((cam, "project", {"pixels": pixels, "z": z, "method": method}))
        return {"ok": True, "points": [{"pixel": p, "xyz": [0.4, 0.0, z], "method": f"plane z={z:.3f}",
                                        "reason": None} for p in pixels]}


def run_cli(args):
    stdout = io.StringIO()
    stderr = io.StringIO()
    with patch.object(sys, "argv", ["eef_cli.py", *args]), contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
        eef_cli.main()
    return stdout.getvalue(), stderr.getvalue()


@contextlib.contextmanager
def fake_run():
    """A temporary runs directory with a current run and one earlier picture in latest/."""
    with tempfile.TemporaryDirectory() as tmp:
        runs = Path(tmp)
        (runs / "run").mkdir()
        (runs / "latest").mkdir()
        (runs / "latest" / "head_left.jpg").write_bytes(b"old picture")
        (runs / "current_run").write_text(str(runs / "run"), encoding="utf-8")
        with patch.object(eef_cli, "CUR", runs / "current_run"), patch.object(eef_cli, "R1DClient", FakeClient):
            yield runs


def grasp_payload(min_opening=None):
    """Descend and grasp, pull up, then carry; the carry waypoint may require a held object."""
    carry = {"pos": [0.36, -0.07, 0.716], "rpy_deg": [90, -10, 20], "linear": False}
    if min_opening is not None:
        carry["min_opening"] = min_opening
    return {"note": "slice 1", "segments": [{"side": "right", "speed": 0.2, "waypoints": [
        {"pos": [0.55, -0.277, 0.79], "rpy_deg": [0, 8, 0], "linear": False, "gripper": "grasp"},
        {"pos": [0.55, -0.277, 0.92], "rpy_deg": [0, 8, 0]},
        carry,
    ]}]}


class EefCliTest(unittest.TestCase):
    def setUp(self):
        FakeClient.instances.clear()
        FakeClient.grasp_opening = 0.18
        FakeClient.refuse_move = None
        FakeClient.lift_m, FakeClient.head_pitch, FakeClient.fixed_lift = 0.573, 0.003, 0.522

    def test_validate_accepts_grasp_and_numeric_gripper(self):
        payload = json.loads(json.dumps(VALID_PAYLOAD))
        payload["segments"][0]["waypoints"][0]["gripper"] = "grasp"
        self.assertIs(eef_cli.validate_eef_input(payload), payload)
        payload["segments"][0]["waypoints"][0]["gripper"] = 0.45
        eef_cli.validate_eef_input(payload)

    def test_validate_rejects_missing_or_malformed_fields(self):
        cases = [
            [],
            {"segments": []},
            {"segments": [{"side": "middle", "waypoints": []}]},
            {"segments": [{"side": "left", "waypoints": [{"pos": [0, 0, 0]}]}]},
            {"segments": [{"side": "left", "waypoints": [{"pos": [0, float("nan"), 0], "rpy_deg": [0, 0, 0]}]}]},
            {"segments": [{"side": "left", "waypoints": [{"pos": [0, 0, 0], "rpy_deg": [0, 0, 0], "linear": 1}]}]},
            {"segments": [{"side": "left", "waypoints": [{"pos": [0, 0, 0], "rpy_deg": [0, 0, 0], "gripper": 1.2}]}]},
        ]
        for payload in cases:
            with self.subTest(payload=payload), self.assertRaises(eef_cli.InputError):
                eef_cli.validate_eef_input(payload)

    def test_check_is_read_only_and_keeps_linear_value(self):
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "run"
            run_dir.mkdir()
            current = Path(tmp) / "current_run"
            current.write_text(str(run_dir), encoding="utf-8")
            with (
                patch.object(eef_cli, "CUR", current),
                patch.object(eef_cli, "R1DClient", FakeClient),
            ):
                stdout, _ = run_cli(["check", json.dumps(VALID_PAYLOAD)])

        self.assertIn("seg0 wp0 OK", stdout)
        client = FakeClient.instances[0]
        self.assertEqual(len(client.moves), 1)
        _, _, kwargs = client.moves[0]
        self.assertTrue(kwargs["check_only"])
        self.assertFalse(kwargs["auto_lift"])
        self.assertTrue(kwargs["linear"])

    def test_validate_rejects_bad_min_opening(self):
        for bad in (1.5, -0.1, True, "0.1"):
            with self.subTest(min_opening=bad), self.assertRaises(eef_cli.InputError):
                eef_cli.validate_eef_input(grasp_payload(bad))
        eef_cli.validate_eef_input(grasp_payload(0.1))

    def test_exec_sends_waypoints_in_order_with_fixed_lift(self):
        with fake_run() as runs:
            stdout, _ = run_cli(["exec", json.dumps(grasp_payload())])
            log = [json.loads(line) for line in (runs / "run" / "eef_log.jsonl").read_text().splitlines()]
        calls = FakeClient.instances[0].moves
        self.assertEqual([c[1] for c in calls],
                         [[0.55, -0.277, 0.79], "gripper", [0.55, -0.277, 0.92], [0.36, -0.07, 0.716]])
        self.assertEqual(calls[1][2], {"opening": None, "action": "grasp"})
        for _, target, kwargs in calls:
            if target != "gripper":
                self.assertFalse(kwargs["auto_lift"])
                self.assertEqual(kwargs["speed"], 0.2)
        self.assertEqual([c[2].get("linear") for c in calls if c[1] != "gripper"], [False, True, False])
        self.assertIn("ok=True", stdout)
        self.assertIn("gripper grasp -> opening 0.18", stdout)
        self.assertEqual(log[0]["kind"], "exec")
        self.assertTrue(log[0]["result"]["ok"])

    def test_exec_stops_at_the_first_refused_waypoint(self):
        FakeClient.refuse_move = 2          # the pull after the grasp
        with fake_run(), self.assertRaises(SystemExit) as raised:
            run_cli(["exec", json.dumps(grasp_payload())])
        self.assertEqual(raised.exception.code, 1)
        self.assertEqual(len(FakeClient.instances[0].moves), 3)

    def test_min_opening_stops_the_carry_after_a_missed_grasp(self):
        FakeClient.grasp_opening = 0.03
        with fake_run() as runs, self.assertRaises(SystemExit) as raised:
            run_cli(["exec", json.dumps(grasp_payload(0.1))])
        self.assertEqual(raised.exception.code, 1)
        calls = FakeClient.instances[0].moves
        self.assertEqual(len(calls), 3)     # descend, grasp, pull; the carry was not sent
        self.assertEqual(calls[-1][1], [0.55, -0.277, 0.92])

    def test_min_opening_lets_a_held_object_through(self):
        with fake_run():
            run_cli(["exec", json.dumps(grasp_payload(0.1))])
        self.assertEqual(len(FakeClient.instances[0].moves), 4)

    def test_min_opening_without_a_grasp_reads_the_measured_opening(self):
        payload = grasp_payload(0.1)
        payload["segments"][0]["waypoints"] = payload["segments"][0]["waypoints"][2:]
        with fake_run(), self.assertRaises(SystemExit):
            run_cli(["exec", json.dumps(payload)])     # the fake state reports opening 0.02
        self.assertEqual(FakeClient.instances[0].moves, [])

    def test_motion_deletes_earlier_pictures_and_check_keeps_them(self):
        with fake_run() as runs:
            run_cli(["check", json.dumps(VALID_PAYLOAD)])
            self.assertTrue((runs / "latest" / "head_left.jpg").exists())
            run_cli(["exec", json.dumps(VALID_PAYLOAD)])
            self.assertFalse((runs / "latest" / "head_left.jpg").exists())

    def test_both_moves_both_arms_and_both_check_only_checks(self):
        payload = {"note": "push plate", "speed": 0.1, "linear": False,
                   "left": {"pos": [0.2, 0.03, 0.66], "rpy_deg": [0, 15, -90]},
                   "right": {"pos": [0.2, -0.05, 0.66], "rpy_deg": [0, 15, 90]}}
        with fake_run() as runs:
            run_cli(["both-check", json.dumps(payload)])
            self.assertTrue((runs / "latest" / "head_left.jpg").exists())
            run_cli(["both", json.dumps(payload)])
            self.assertFalse((runs / "latest" / "head_left.jpg").exists())
        check, move = (c.moves[0][2] for c in FakeClient.instances)
        self.assertEqual(check, {"speed": 0.1, "check_only": True, "linear": False})
        self.assertEqual(move, {"speed": 0.1, "check_only": False, "linear": False})

    def test_both_rejects_a_missing_arm_before_opening_client(self):
        constructor = unittest.mock.Mock(side_effect=AssertionError("client opened"))
        with (
            patch.object(eef_cli, "R1DClient", constructor),
            patch.object(sys, "argv", ["eef_cli.py", "both", '{"left": {"pos": [0, 0, 0], "rpy_deg": [0, 0, 0]}}']),
            contextlib.redirect_stderr(io.StringIO()),
            self.assertRaises(SystemExit),
        ):
            eef_cli.main()
        constructor.assert_not_called()

    def test_project_accepts_options_before_the_pixels(self):
        with fake_run():
            stdout, _ = run_cli(["project", "--cams", "head_left", "--z", "0.61", "261,226;296.7,224"])
        _, _, sent = FakeClient.instances[0].moves[0]
        self.assertEqual(sent, {"pixels": [[261.0, 226.0], [296.7, 224.0]], "z": 0.61, "method": "plane"})
        self.assertIn("#1 pixel [296.7, 224.0] -> xyz [0.4, 0.0, 0.61]", stdout)

    def test_missing_camera_frame_is_reported_without_a_traceback(self):
        def no_frame(self, tag):            # what Session.keyframe raises when the server has no fresh frame
            raise eef_cli.R1DError("no image from head_left: no fresh frame from head_left")

        with fake_run(), patch.object(eef_cli.Session, "keyframe", no_frame), \
                self.assertRaises(SystemExit) as raised:
            run_cli(["keyframe", "final"])
        self.assertEqual(str(raised.exception), "keyframe failed: no image from head_left: no fresh frame from head_left")

    def test_prepare_sets_the_lift_and_the_head_and_clears_old_pictures(self):
        with fake_run() as runs:
            stdout, _ = run_cli(["prepare"])
            self.assertFalse((runs / "latest" / "head_left.jpg").exists())
            log = [json.loads(line) for line in (runs / "run" / "eef_log.jsonl").read_text().splitlines()]
        self.assertEqual(FakeClient.instances[0].moves, [("lift", 0.522, {}), ("head", 0.486, {"yaw": 0.0})])
        out = json.loads(stdout)
        self.assertTrue(out["ok"])
        self.assertEqual((out["lift_m"], out["head_pitch"]), (0.522, 0.511))
        self.assertEqual(log[0]["kind"], "prepare")

    def test_prepare_moves_nothing_when_the_robot_is_already_at_the_start_pose(self):
        FakeClient.lift_m, FakeClient.head_pitch = 0.5222, 0.5115
        with fake_run():
            stdout, _ = run_cli(["prepare"])
        self.assertEqual(FakeClient.instances[0].moves, [])
        self.assertTrue(json.loads(stdout)["ok"])

    def test_prepare_needs_a_fixed_lift_service(self):
        FakeClient.fixed_lift = None
        with fake_run(), self.assertRaises(SystemExit) as raised:
            run_cli(["prepare"])
        self.assertIn("--fixed-lift", str(raised.exception))
        self.assertEqual(FakeClient.instances[0].moves, [])

    def test_invalid_json_fails_before_opening_client(self):
        constructor = unittest.mock.Mock(side_effect=AssertionError("client opened"))
        stderr = io.StringIO()
        with (
            patch.object(eef_cli, "R1DClient", constructor),
            patch.object(sys, "argv", ["eef_cli.py", "check", "not-json"]),
            contextlib.redirect_stderr(stderr),
            self.assertRaises(SystemExit),
        ):
            eef_cli.main()
        constructor.assert_not_called()
        self.assertIn("invalid JSON", stderr.getvalue())


class LazyImportTest(unittest.TestCase):
    def test_help_does_not_load_heavy_dependencies(self):
        scripts = {
            "eef_cli.py",
            "eef_eval.py",
            "arm_posture.py",
            "check_environment.py",
            "collect_metrics.py",
        }
        probe = (
            "import json, runpy, sys\n"
            "path = sys.argv[1]\n"
            "sys.argv = [path, '--help']\n"
            "try:\n"
            "    runpy.run_path(path, run_name='__main__')\n"
            "except SystemExit:\n"
            "    pass\n"
            "print(json.dumps(sorted(k for k in ('numpy', 'cv2', 'pinocchio', 'ik.kin') if k in sys.modules)))\n"
        )
        env = dict(os.environ, PYTHONPATH=str(SKILL_ROOT / "src"))
        for name in sorted(scripts):
            with self.subTest(script=name):
                proc = subprocess.run(
                    [sys.executable, "-B", "-c", probe, str(SKILL_ROOT / "scripts" / name)],
                    capture_output=True,
                    text=True,
                    env=env,
                    check=False,
                )
                self.assertEqual(proc.returncode, 0, proc.stderr)
                self.assertEqual(json.loads(proc.stdout.strip().splitlines()[-1]), [])


if __name__ == "__main__":
    unittest.main()
