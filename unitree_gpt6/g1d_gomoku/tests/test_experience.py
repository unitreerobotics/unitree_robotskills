"""Cold start, success gating and deployment isolation for learned experience."""

import copy
import json
import tempfile
import unittest
from pathlib import Path

from unitree_gpt6.g1d_gomoku.src import experience as exp


class ExperienceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.config = self.root / "hardware.yaml"
        self.config.write_text(
            "runtime:\n  reset_state: reset.json\n  scene_calibration: scene.json\n"
        )
        (self.root / "reset.json").write_text('{"reference": "test capture"}')
        self.calibration = self.root / "scene.json"
        self.calibration.write_text(
            json.dumps(
                {
                    "frame": "pelvis",
                    "tool_tip_offset_m": [0.1, 0, 0],
                    "high_route_min_tcp_z_m": 0.3,
                    "high_route_min_tip_z_m": 0.2,
                    "rest_corridor": {
                        side: {"min_m": [-1, -1, 0], "max_m": [1, 1, 1]}
                        for side in ("left", "right")
                    },
                }
            )
        )
        exp.initialize(self.root)
        exp.make_context(self.root, "test-robot", "test-scene", self.config, self.calibration)
        self.context = exp.read(self.root / "experience/context.json")
        self.round = self.root / "runtime/round_01"
        self.round.mkdir(parents=True)
        self.state = {
            "round_id": "synthetic-round-id",
            "task_status": "returned",
            "reset_joint_verified": True,
            "physical_placement_verified": False,
            "config_path": str(self.config),
            "config_sha256": exp.sha256(self.config),
        }
        self.candidate = {
            "context": self.context,
            "observations": [
                {
                    "kind": "grasp",
                    "target": {"gripper_rad": 0.7},
                    "measured": {"gripper_rad": 0.72},
                    "verification": {
                        "single_stone_held": True,
                        "method": "Synthetic visual review",
                    },
                    "evidence": ["grasp_feedback.json"],
                }
            ],
        }
        (self.round / "grasp_feedback.json").write_text('{"gripper_rad": 0.72}')
        self.write_inputs()

    def write_inputs(self):
        (self.round / "round.json").write_text(json.dumps(self.state))
        self.input = self.round / "experience_candidate.json"
        self.input.write_text(json.dumps(self.candidate))

    def test_cold_start_is_empty_and_initialize_never_overwrites(self):
        self.assertEqual(exp.matching(self.root)["records"], [])
        exp.record(self.root, self.round, self.input)
        before = (self.root / "experience/records.json").read_bytes()
        exp.initialize(self.root)
        self.assertEqual((self.root / "experience/records.json").read_bytes(), before)
        result = exp.matching(self.root)["records"]
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["measured"], {"gripper_rad": 0.72})
        self.assertNotEqual(result[0]["target"], result[0]["measured"])
        self.assertTrue(all(not Path(e["path"]).is_absolute() for e in result[0]["evidence"]))
        with self.assertRaisesRegex(ValueError, "already published"):
            exp.record(self.root, self.round, self.input)

    def test_failure_unverified_and_plan_only_never_publish(self):
        for status in ("execution_failed", "return_unverified", "awaiting_release_confirmation"):
            with self.subTest(status=status):
                self.state["task_status"] = status
                self.write_inputs()
                with self.assertRaisesRegex(ValueError, "successfully returned"):
                    exp.record(self.root, self.round, self.input)
        self.assertEqual(exp.matching(self.root)["records"], [])

    def test_return_does_not_claim_visual_placement_success(self):
        self.candidate["observations"][0].update(
            kind="placement",
            verification={"physical_placement_verified": True, "method": "Unsupported claim"},
        )
        self.write_inputs()
        with self.assertRaisesRegex(ValueError, "does not verify physical placement"):
            exp.record(self.root, self.round, self.input)

    def test_missing_review_nonfinite_unknown_and_empty_values_are_rejected(self):
        original = copy.deepcopy(self.candidate)
        for changes in (
            {"verification": {"single_stone_held": False, "method": "not checked"}},
            {"measured": {"gripper_rad": float("nan")}},
            {"measured": {"gripper_rad": None}},
            {"measured": {}},
            {"evidence": ["missing.jpg"]},
            {"evidence": ["../../scene.json"]},
        ):
            with self.subTest(changes=changes):
                self.candidate = copy.deepcopy(original)
                self.candidate["observations"][0].update(changes)
                self.write_inputs()
                with self.assertRaises(ValueError):
                    exp.record(self.root, self.round, self.input)
        self.assertEqual(exp.matching(self.root)["records"], [])

    def test_changed_config_calibration_reset_and_scene_do_not_reuse_records(self):
        exp.record(self.root, self.round, self.input)
        for path in (self.config, self.calibration, self.root / "reset.json"):
            original = path.read_bytes()
            path.write_bytes(original + b"\n")
            with self.assertRaisesRegex(ValueError, "Deployment changed"):
                exp.matching(self.root)
            path.write_bytes(original)
        exp.make_context(self.root, "test-robot", "different-scene", self.config, self.calibration)
        self.assertEqual(exp.matching(self.root)["records"], [])

    def test_changed_evidence_is_not_reused(self):
        exp.record(self.root, self.round, self.input)
        (self.round / "grasp_feedback.json").write_text('{"gripper_rad": 1.4}')
        self.assertEqual(exp.matching(self.root)["records"], [])

    def test_batch_validation_never_publishes_partial_success(self):
        invalid = copy.deepcopy(self.candidate["observations"][0])
        invalid["kind"] = "placement"
        invalid["verification"] = {"physical_placement_verified": True, "method": "Invalid claim"}
        self.candidate["observations"].append(invalid)
        self.write_inputs()
        with self.assertRaises(ValueError):
            exp.record(self.root, self.round, self.input)
        self.assertEqual(exp.matching(self.root)["records"], [])

    def test_missing_evidence_does_not_reuse_an_old_record(self):
        exp.record(self.root, self.round, self.input)
        (self.round / "grasp_feedback.json").unlink()
        self.assertEqual(exp.matching(self.root)["records"], [])

    def test_scene_requires_measured_values_without_fallback(self):
        from unitree_gpt6.g1d_gomoku.src.hardware.common import load_config, scene_geometry

        self.calibration.write_text('{"frame": "pelvis"}')
        with self.assertRaisesRegex(ValueError, "tool_tip_offset_m"):
            scene_geometry(load_config(self.config))
        self.calibration.unlink()
        with self.assertRaisesRegex(ValueError, "Capture runtime.scene_calibration"):
            scene_geometry(load_config(self.config))

    def test_missing_store_is_empty_and_source_writes_are_rejected(self):
        self.assertEqual(exp.matching(self.root / "new-workspace")["records"], [])
        package = Path(exp.__file__).resolve().parents[1]
        with self.assertRaisesRegex(ValueError, "source directory"):
            exp.initialize(package)
        outside = self.root.parent / "outside-workspace.json"
        self.candidate["observations"][0]["evidence"] = [str(outside)]
        self.write_inputs()
        with self.assertRaisesRegex(ValueError, "relative paths"):
            exp.record(self.root, self.round, self.input)

    def test_symlink_cannot_escape_workspace(self):
        with tempfile.TemporaryDirectory() as other:
            link = self.round / "external"
            link.symlink_to(other, target_is_directory=True)
            self.candidate["observations"][0]["evidence"] = ["external/result.json"]
            self.write_inputs()
            with self.assertRaisesRegex(ValueError, "inside the workspace"):
                exp.record(self.root, self.round, self.input)
