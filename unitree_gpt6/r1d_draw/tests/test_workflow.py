"""Plan -> compile -> execute with the simulated robot, including interruption and resume."""

import json
import tempfile
import unittest
from pathlib import Path

from unitree_gpt6.r1d_draw.src.cli import main
from unitree_gpt6.r1d_draw.src.config import load_config
from unitree_gpt6.r1d_draw.src.job import validate_job
from unitree_gpt6.r1d_draw.src.runner import Executor
from unitree_gpt6.r1d_draw.src.simulation import SimulatedKinematics, SimulatedRobot
from unitree_gpt6.r1d_draw.src.storage import read_json
from unitree_gpt6.r1d_draw.src.task import verify_task
from unitree_gpt6.r1d_draw.src.workflow import initialize_task, prepare_task, run_demo_workflow


class PreparedTask(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.root = Path(self.folder.name)
        initialize_task(self.root / "inputs", simulation=True)
        self.prepare("job-1")

    def tearDown(self):
        self.folder.cleanup()

    def prepare(self, job_id, edit=None):
        inputs = self.root / "inputs"
        if edit:
            strokes = read_json(inputs / "strokes.json")
            edit(strokes)
            (inputs / "strokes.json").write_text(json.dumps(strokes))
        self.ready = self.root / job_id
        prepare_task(
            config_path=inputs / "device.json",
            strokes_path=inputs / "strokes.json",
            measurements_path=inputs / "measurements.json",
            job_id=job_id,
            output=self.ready,
            simulation=True,
        )
        self.job = read_json(self.ready / "job.json")
        self.config = load_config(self.ready / "device.json")

    def robot(self):
        return SimulatedRobot(
            self.job["entry_q14"][0], waist_yaw=self.job["waist_yaw"], grip_q2=self.job["grip_q2"]
        )

    def run_job(self, robot, **options):
        return Executor(self.config, robot, SimulatedKinematics()).run(self.job, **options)

    def phases(self):
        return [(b["phase"], b["color"]) for b in self.job["batches"]]


class PlanningTests(PreparedTask):
    def test_reload_and_wash_follow_the_plan_inputs(self):
        expected = (
            "LOAD blue, PAINT blue, LOAD blue, PAINT blue, WASH water, LOAD green, PAINT green"
        )
        self.assertEqual(
            self.phases(), [tuple(x.split()) for x in (expected + ", WASH water").split(", ")]
        )
        # Strokes of one load share a batch; reload_every=2 splits the three blue strokes 2 + 1.
        self.assertEqual([len(b["strokes"]) for b in self.job["batches"]], [0, 2, 0, 1, 0, 0, 1, 0])

    def test_without_those_inputs_each_color_is_loaded_once_and_nothing_is_washed(self):
        def edit(strokes):
            del strokes["reload_every"], strokes["wash_station"]

        self.prepare("job-plain", edit)
        expected = "LOAD blue, PAINT blue, LOAD green, PAINT green"
        self.assertEqual(self.phases(), [tuple(x.split()) for x in expected.split(", ")])
        self.assertEqual([len(b["strokes"]) for b in self.job["batches"]], [0, 3, 0, 1])
        validate_job(self.job)

    def test_task_directory_is_complete_and_detects_changes(self):
        names = {p.name for p in self.ready.iterdir()}
        self.assertTrue(
            {
                "job.json",
                "plan.json",
                "preview.svg",
                "run.sh",
                "manifest.json",
                "calibration.json",
                "device.json",
                "report.json",
            }
            <= names
        )
        self.assertIn("r1d-draw run-task", (self.ready / "run.sh").read_text())
        verify_task(self.ready)
        job = read_json(self.ready / "job.json")
        job["batches"][1]["action"][2]["x"] += 0.001
        (self.ready / "job.json").write_text(json.dumps(job))
        with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
            verify_task(self.ready)


class ExecutionTests(PreparedTask):
    def test_whole_job_ends_parked_at_the_rest_posture(self):
        robot = self.robot()
        result = self.run_job(robot)
        self.assertEqual(
            (result["state"], result["completed_batches"], result["completed_strokes"]),
            ("job_completed", 8, 4),
        )
        self.assertTrue(result["parked"])
        self.assertTrue(robot.holding)
        self.assertEqual(robot.state()["arms"]["right"], self.job["park_q14"][-1][7:])
        self.assertEqual(robot.state()["arms"]["left"], self.job["entry_q14"][0][:7])
        # entry + two routes per batch (to its first point, then the rest) + park
        self.assertEqual(len(robot.routes), 1 + 2 * 8 + 1)

    def test_material_dwell_is_sent_at_the_contact_point(self):
        robot = self.robot()
        self.run_job(robot)
        wash = robot.routes[1 + 2 * 4 + 1]["points"]  # second route of batch 4 (WASH)
        dwells = [number for number, point in enumerate(wash) if point.get("dwell_s")]
        self.assertEqual([wash[number]["dwell_s"] for number in dwells], [0.5])
        heights = [point["q"]["24"] for point in wash]
        self.assertEqual(heights[dwells[0]], min(heights))  # it waits at the lowest point

    def test_strokes_are_drawn_in_one_motion_and_travel_follows_a_straight_line(self):
        robot = self.robot()
        self.run_job(robot)
        paint = robot.routes[1 + 2 * 3 + 1]["points"]  # second route of batch 3 (one stroke)
        on_paper = [point for point in paint if point["q"]["24"] < 0.03]
        profiles = [point.get("profile", "quintic") for point in on_paper]
        self.assertEqual(profiles[0], "quintic")  # touching down is a move of its own
        self.assertEqual((profiles[1], profiles[-1]), ("ease_in", "ease_out"))
        self.assertEqual(set(profiles[2:-1]), {"linear"})
        # The move to the next batch is cut into short steps along the line between the two.
        travel = robot.routes[1 + 2 * 4]["points"]
        self.assertGreater(len(travel), 3)
        start = [paint[-1]["q"][slot] for slot in ("22", "23", "24")]
        end = [travel[-1]["q"][slot] for slot in ("22", "23", "24")]
        for number, point in enumerate(travel[:-1], 1):
            for slot, a, b in zip(("22", "23", "24"), start, end, strict=True):
                self.assertAlmostEqual(point["q"][slot], a + (b - a) * number / len(travel))

    def test_refuses_wrong_start_posture_waist_or_grip(self):
        cases = {
            "arms": (lambda r: r.q.__setitem__(24, 0.3), "first entry point"),
            "waist": (lambda r: r.q.__setitem__(13, 0.2), "waist yaw"),
            "grip": (lambda r: r.q.__setitem__(33, 1.0), "grip"),
        }
        for number, (name, (change, message)) in enumerate(cases.items()):
            with self.subTest(name=name):
                self.prepare(f"job-refuse-{number}")
                robot = self.robot()
                change(robot)
                result = self.run_job(robot)
                self.assertEqual(result["state"], "interrupted")
                self.assertIn(message, result["error"])
                self.assertEqual(robot.routes, [])

    def test_a_started_job_id_cannot_be_started_again(self):
        self.run_job(self.robot())
        with self.assertRaisesRegex(RuntimeError, "already started"):
            self.run_job(self.robot())
        again = self.run_job(self.robot(), resume=True)  # finished: nothing runs again
        self.assertEqual(again["state"], "job_completed")
        with self.assertRaisesRegex(RuntimeError, "never started"):
            self.prepare("job-2")
            self.run_job(self.robot(), resume=True)


class ResumeTests(PreparedTask):
    def interrupt(self, route_number, point=None):
        robot = self.robot()
        robot.fail_on_route, robot.fail_at_point = route_number, point
        result = self.run_job(robot)
        self.assertEqual(result["state"], "interrupted")
        self.assertIn("simulated", result["error"])
        return robot, result

    def test_resume_continues_at_the_unfinished_batch_without_replaying_finished_ones(self):
        robot, result = self.interrupt(1 + 2 * 3 + 0)  # travelling to the first point of batch 3
        self.assertEqual(result["completed_batches"], 3)
        sent = len(robot.routes)
        robot.fail_on_route = None
        final = self.run_job(robot, resume=True)
        self.assertEqual(
            (final["state"], final["completed_batches"], final["completed_strokes"]),
            ("job_completed", 8, 4),
        )
        # the five remaining batches and the park route; the entry route is not repeated
        self.assertEqual(len(robot.routes) - sent, 2 * 5 + 1)
        self.assertEqual(robot.state()["arms"]["right"], self.job["park_q14"][-1][7:])

    def test_resume_refuses_a_low_brush_unless_asked_to_lift(self):
        probe = self.robot()
        self.run_job(probe)
        heights = [point["q"]["24"] for point in probe.routes[1 + 2 * 1 + 1]["points"]]
        low = next(number for number, height in enumerate(heights) if height < 0.03)
        self.prepare("job-low")
        # stop inside the first stroke of batch 1, with the brush on the paper
        robot, result = self.interrupt(1 + 2 * 1 + 1, point=low + 2)
        self.assertEqual(result["completed_batches"], 1)
        height = robot.state()["arms"]["right"][2]
        self.assertLess(height, 0.05)
        robot.fail_on_route = None
        refused = self.run_job(robot, resume=True)
        self.assertEqual(refused["state"], "interrupted")
        self.assertIn("resume with lift", refused["error"])
        self.assertEqual(robot.state()["arms"]["right"][2], height)  # it did not move
        sent = len(robot.routes)
        final = self.run_job(robot, resume=True, lift=True)
        self.assertEqual((final["state"], final["completed_batches"]), ("job_completed", 8))
        lift = robot.routes[sent]["points"]
        self.assertEqual(len(lift), 1)
        self.assertAlmostEqual(lift[0]["q"]["24"], 0.15)  # straight up to the transfer height

    def test_a_changed_job_cannot_be_resumed(self):
        robot, _ = self.interrupt(5)
        self.job["batches"][-1]["dwell_s"] = 0.9
        with self.assertRaisesRegex(RuntimeError, "changed"):
            self.run_job(robot, resume=True)


class CommandLineTests(unittest.TestCase):
    def test_demo_workflow_and_run_task_from_the_command_line(self):
        with tempfile.TemporaryDirectory() as folder:
            summary = run_demo_workflow(Path(folder) / "demo")
            self.assertEqual((summary["state"], summary["completed_strokes"]), ("job_completed", 4))
            self.assertEqual(
                main(
                    [
                        "prepare",
                        "--simulation",
                        "--config",
                        f"{folder}/demo/inputs/device.json",
                        "--strokes",
                        f"{folder}/demo/inputs/strokes.json",
                        "--measurements",
                        f"{folder}/demo/inputs/measurements.json",
                        "--job-id",
                        "cli-1",
                        "--output",
                        f"{folder}/cli",
                    ]
                ),
                0,
            )
            self.assertEqual(main(["run-task", "--task", f"{folder}/cli"]), 0)
            self.assertEqual(main(["run-task", "--task", f"{folder}/cli"]), 1)  # already started
            self.assertEqual(main(["run-task", "--task", f"{folder}/cli", "--resume"]), 0)


if __name__ == "__main__":
    unittest.main()
