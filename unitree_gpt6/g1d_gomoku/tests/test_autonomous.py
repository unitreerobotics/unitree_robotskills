"""Behavior tests for automatic choice and the Agent/hardware round boundary."""

import contextlib
import copy
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from unitree_gpt6.g1d_gomoku.src.agent import GomokuAgent, save
from unitree_gpt6.g1d_gomoku.src.board_vision import VIEWS, digest, project_intersection
from unitree_gpt6.g1d_gomoku.src.gomoku_engine import choose_move
from unitree_gpt6.g1d_gomoku.src.main import main


def empty_board():
    return [[0] * 19 for _ in range(19)]


def position(black, white):
    board = empty_board()
    for color, points in ((1, black), (2, white)):
        for row, column in points:
            board[row - 1][column - 1] = color
    return board


class StrategyTests(unittest.TestCase):
    def test_opening_is_chosen_without_target(self):
        move = choose_move(empty_board())
        self.assertEqual((move.row, move.column), (10, 10))
        self.assertEqual(move.to_dict()["selection_source"], "autonomous_engine")

    def test_black_immediate_win_precedes_white_threat(self):
        board = position([(8, c) for c in range(5, 9)], [(12, c) for c in range(5, 9)])
        move = choose_move(board)
        self.assertEqual(move.row, 8)
        self.assertIn(move.column, (4, 9))

    def test_blocks_white_broken_four(self):
        board = position([(1, 1), (3, 1), (5, 1), (7, 1)], [(10, 5), (10, 6), (10, 8), (10, 9)])
        move = choose_move(board)
        self.assertEqual((move.row, move.column), (10, 7))

    def test_diagonal_and_edge_wins(self):
        for black in ([(i, i) for i in range(1, 5)], [(1, i) for i in range(1, 5)]):
            board = position(black, [(19, i) for i in (1, 3, 5, 7)])
            move = choose_move(board)
            self.assertEqual((move.row, move.column), (5, 5) if black[1] == (2, 2) else (1, 5))

    def test_rejects_wrong_turn_terminal_and_unknown_cells(self):
        terminal = position([(1, i) for i in range(1, 6)], [(3, i) for i in (1, 3, 5, 7, 9)])
        unknown = empty_board()
        unknown[0][0] = -1
        for board in (position([(10, 10)], []), terminal, unknown, [[0]], None):
            with self.subTest(board=board), self.assertRaises(ValueError):
                choose_move(board)

    def test_does_not_mutate_observed_board(self):
        board = position([(10, 10)], [(10, 11)])
        before = copy.deepcopy(board)
        move = choose_move(board)
        self.assertEqual(board, before)
        self.assertEqual(board[move.row - 1][move.column - 1], 0)

    def test_blocked_four_is_not_a_win(self):
        board = position([(10, c) for c in range(5, 9)], [(10, 4), (10, 9), (1, 1), (3, 1)])
        move = choose_move(board)
        self.assertNotIn((move.row, move.column), ((10, 4), (10, 9)))
        self.assertNotIn("取胜", move.reason)


class FakeAdapter:
    def __init__(self):
        self.config = {
            "robot": {"joint_tolerance": 0.1, "grippers": {"right": {"min": 0, "max": 5.45}}}
        }
        self.current = {
            "pid": 123,
            "commanded_q": [0.0] * 14,
            "initial_waist": [0.0] * 3,
            "stage": "aligned",
            "feedback": {"joint_positions": [0.0] * 14},
        }
        self.release_calls = 0

    def provenance(self):
        return {
            "config_path": "/private/local.yaml",
            "config_sha256": "config",
            "owner_root": "/private/owner",
        }

    def capture(self, root):
        folder = root / "board_capture"
        folder.mkdir()
        for name in VIEWS:
            (folder / f"{name}.jpg").write_bytes(
                b"fake encoded image; never sent to a vision model"
            )
        save(
            folder / "metadata.json",
            {name: {"available": True, "shape": [600, 800]} for name in VIEWS},
        )
        return copy.deepcopy(self.current)

    def reference(self):
        return {"joint_positions": [0.0] * 7 + [0.1] * 7}

    def status(self):
        return copy.deepcopy(self.current)

    def release(self, root):
        self.release_calls += 1
        self.current["feedback"]["joint_positions"] = self.reference()["joint_positions"]
        return self.status()


class AutonomousRoundTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "turn"
        self.agent = GomokuAgent(self.root)
        self.adapter = FakeAdapter()
        self.agent.start(self.adapter)

    def recognize(self, **changes):
        observation = {
            "capture_id": self.agent.state()["capture_id"],
            "recognition_source": "agent_visual_review",
            "board": empty_board(),
            "views_agree": True,
            "uncertain_intersections": [],
            "grid_corners_px": [[100, 100], [500, 100], [500, 500], [100, 500]],
            "notes": "Both head images agree; rows down, columns right.",
        }
        observation.update(changes)
        save(self.root / "board_observation.json", observation)

    def prepared(self):
        self.recognize()
        self.agent.decide()
        folder = self.root / "aligned"
        (folder / "after").mkdir(parents=True)
        save(
            folder / "result.json",
            {"outcome": "motion_complete_unverified", "status": self.adapter.status()},
        )
        save(folder / "after/metadata.json", {name: {"available": True} for name in VIEWS})
        for name in VIEWS:
            (folder / "after" / f"{name}.jpg").write_bytes(b"low position view")
        save(
            self.root / "alignment_observation.json",
            {
                "decision_sha256": digest(self.root / "position_and_move.json"),
                "matches_selected_intersection": True,
                "target_empty_before_descent": True,
                "single_black_stone_held": True,
                "grid_neighbor_review": "Cross checked both grid axes.",
                "stage_directory": str(folder),
            },
        )
        save(
            self.root / "return_plan.json",
            {
                "seed": [0.0] * 14,
                "locked_waist_q": [0.0] * 3,
                "clearance_review": {"passed": True},
                "steps": [
                    {
                        "joint_positions": self.adapter.reference()["joint_positions"],
                        "grippers": {},
                    }
                ],
            },
        )
        save(
            self.root / "release_parameters.json",
            {"right_gripper_rad": 1.3, "review": "Synthetic release target"},
        )
        return self.agent.prepare_release(self.adapter)

    def test_missing_perception_never_assumes_empty(self):
        with self.assertRaisesRegex(ValueError, "inspect"):
            self.agent.decide()
        self.assertFalse((self.root / "position_and_move.json").exists())

    def test_decides_and_marks_only_after_evidence_bound_recognition(self):
        self.recognize()
        result = self.agent.decide()
        self.assertEqual(result["task_status"], "target_selected")
        decision = json.loads((self.root / "position_and_move.json").read_text())
        self.assertEqual((decision["row"], decision["column"]), (10, 10))
        self.assertEqual(decision["target_pixel_left_high"], [300.0, 300.0])
        self.assertTrue((self.root / "suggested_black_move.svg").is_file())
        self.assertFalse(decision["physical_placement_verified"])

    def test_rejects_target_override_uncertainty_disagreement_and_old_capture(self):
        for change in (
            {"target": [1, 1]},
            {"uncertain_intersections": [[1, 1]]},
            {"views_agree": False},
            {"capture_id": "old"},
            {"grid_corners_px": [[0, 0]] * 4},
        ):
            with self.subTest(change=change):
                self.recognize(**change)
                with self.assertRaises(ValueError):
                    self.agent.decide()
                self.assertEqual(self.agent.state()["task_status"], "awaiting_board_review")

    def test_rejects_changed_original_image(self):
        self.recognize()
        (self.root / "board_capture/cam_left_high.jpg").write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "images changed"):
            self.agent.decide()

    def test_release_requires_alignment_and_confirmation(self):
        self.assertEqual(self.prepared()["task_status"], "awaiting_release_confirmation")
        with self.assertRaises(ValueError):
            self.agent.release(self.adapter, "")
        self.assertEqual(self.adapter.release_calls, 0)
        result = self.agent.release(self.adapter, "确认放子")
        self.assertEqual(result["task_status"], "returned")
        self.assertTrue(result["reset_joint_verified"])
        self.assertFalse(result["physical_placement_verified"])
        with self.assertRaises(ValueError):
            self.agent.release(self.adapter, "确认放子")
        self.assertEqual(self.adapter.release_calls, 1)

    def test_no_release_after_plan_command_or_controller_change(self):
        self.prepared()
        self.adapter.current["commanded_q"][7] = 0.2
        with self.assertRaisesRegex(ValueError, "Held command changed"):
            self.agent.release(self.adapter, "确认放子")
        self.adapter.current["commanded_q"][7] = 0.0
        self.adapter.current["pid"] = 456
        with self.assertRaisesRegex(ValueError, "restarted"):
            self.agent.release(self.adapter, "确认放子")
        self.adapter.current["pid"] = 123
        with (self.root / "return_plan.json").open("a") as stream:
            stream.write(" ")
        with self.assertRaisesRegex(ValueError, "changed after review"):
            self.agent.release(self.adapter, "确认放子")
        self.assertEqual(self.adapter.release_calls, 0)

    def test_changed_release_opening_invalidates_confirmation(self):
        self.prepared()
        save(
            self.root / "release_parameters.json",
            {
                "right_gripper_rad": 1.7,
                "review": "Changed synthetic target",
            },
        )
        with self.assertRaisesRegex(ValueError, "changed after review"):
            self.agent.release(self.adapter, "确认放子")
        self.assertEqual(self.adapter.release_calls, 0)

    def test_release_has_no_default_opening(self):
        from unitree_gpt6.g1d_gomoku.src.robot_adapter import release_opening

        with self.assertRaises(FileNotFoundError):
            release_opening(self.root, self.adapter.config)
        for value in (None, float("nan"), 6.0):
            save(
                self.root / "release_parameters.json",
                {
                    "right_gripper_rad": value if value is None or value == 6.0 else "NaN",
                    "review": "Synthetic invalid value",
                },
            )
            with self.assertRaises(ValueError):
                release_opening(self.root, self.adapter.config)

    def test_fault_does_not_allow_automatic_replay(self):
        self.prepared()
        with mock.patch.object(self.adapter, "release", side_effect=TimeoutError("feedback lost")):
            with self.assertRaises(TimeoutError):
                self.agent.release(self.adapter, "确认放子")
        self.assertEqual(self.agent.state()["task_status"], "execution_failed")
        with self.assertRaises(ValueError):
            self.agent.release(self.adapter, "确认放子")

    def test_manual_coordinate_cli_removed(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            main(["--instruction", "R12C14"])
        self.assertEqual(error.exception.code, 2)
        with contextlib.redirect_stdout(io.StringIO()) as output:
            code = main(["decide", "--round", str(self.root)])
        self.assertEqual(code, 1)
        self.assertEqual(json.loads(output.getvalue())["task_status"], "failed")

    def test_projective_grid_uses_intersections_not_outer_border(self):
        corners = [[120, 80], [600, 100], [560, 500], [100, 460]]
        for r, c, expected in (
            (1, 1, corners[0]),
            (1, 19, corners[1]),
            (19, 19, corners[2]),
            (19, 1, corners[3]),
        ):
            actual = project_intersection(corners, r, c)
            self.assertAlmostEqual(actual[0], expected[0])
            self.assertAlmostEqual(actual[1], expected[1])


if __name__ == "__main__":
    unittest.main()
