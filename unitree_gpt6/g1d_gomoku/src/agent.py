"""Persistent single-round workflow for Agent-led autonomous physical Gomoku."""

from __future__ import annotations

import contextlib
import fcntl
import json
import time
import uuid
from importlib.resources import files
from pathlib import Path
from typing import Any

import yaml

from .board_vision import VIEWS, annotate, digest, read_observation
from .gomoku_engine import choose_move
from .robot_adapter import HardwareAdapter, close_vector, release_opening


def load_settings() -> dict:
    value = yaml.safe_load((files("unitree_gpt6.g1d_gomoku") / "config/default.yaml").read_text())
    if (
        not isinstance(value, dict)
        or value.get("mode") != "autonomous"
        or value.get("robot_model") != "G1-D"
        or value.get("agent_provider") != "host_agent"
    ):
        raise ValueError("Gomoku only supports autonomous G1-D play with a host Agent")
    settings = value.get("gomoku", {})
    rules = {"win_length": 5, "overline_wins": True, "forbidden_moves": False}
    if (
        settings.get("board_size") != 19
        or settings.get("color") != "black"
        or settings.get("coordinate_notation") != "row_column"
        or settings.get("rules") != rules
    ):
        raise ValueError("Only black, 19x19 freestyle Gomoku is supported")
    if any(key in settings for key in ("target", "demo_target", "occupied", "initial_occupied")):
        raise ValueError("Targets and positions must come from the current observed board")
    return value


def save(path: Path, value: Any) -> None:
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        temporary.write_text(
            json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


class GomokuAgent:
    """Coordinate perception, automatic choice and confirmed continuous return.

    The hosting Agent supplies visual recognition and mechanical stage plans.
    User-facing input is simply to play a turn; no target can be provided here.
    """

    def __init__(self, round_dir: Path):
        self.root = Path(round_dir).expanduser().resolve()
        self.settings = load_settings()

    @contextlib.contextmanager
    def lock(self):
        """Serialize CLI requests for one round, including the entire release."""
        self.root.parent.mkdir(parents=True, exist_ok=True)
        path = self.root.parent / ("." + self.root.name + ".gomoku.lock")
        with path.open("a+") as stream:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            yield

    def state(self) -> dict:
        return json.loads((self.root / "round.json").read_text())

    def _state(self, expected: str | tuple[str, ...]) -> dict:
        state = self.state()
        allowed = (expected,) if isinstance(expected, str) else expected
        if state["task_status"] not in allowed:
            raise ValueError(f"Round is {state['task_status']}; expected {expected}; do not replay")
        return state

    def _write_state(self, state: dict, status: str, **fields) -> dict:
        state.update(task_status=status, **fields)
        save(self.root / "round.json", state)
        return state

    def _owner(self, adapter: HardwareAdapter, state: dict) -> dict:
        provenance = adapter.provenance()
        if any(provenance[key] != state[key] for key in provenance):
            raise ValueError(
                "Configuration or controller root changed; do not continue an old round"
            )
        current = adapter.status()
        if current["pid"] != state["owner_pid"]:
            raise ValueError("Controller restarted; re-observe and re-plan from current state")
        return current

    def start(self, adapter: HardwareAdapter) -> dict:
        self.root.mkdir(parents=True, exist_ok=False)
        initial = {
            "task_status": "observing",
            "round_id": uuid.uuid4().hex,
            "created_at_unix": time.time(),
            **adapter.provenance(),
        }
        save(self.root / "round.json", initial)
        try:
            status = adapter.capture(self.root)
            images = {
                name: {
                    "path": str(self.root / "board_capture" / f"{name}.jpg"),
                    "sha256": digest(self.root / "board_capture" / f"{name}.jpg"),
                }
                for name in VIEWS
            }
            evidence = {
                "capture_id": uuid.uuid4().hex,
                "images": images,
                "controller": status,
                "captured_at_unix": time.time(),
            }
            save(self.root / "capture.json", evidence)
            return self._write_state(
                initial,
                "awaiting_board_review",
                owner_pid=status["pid"],
                capture_id=evidence["capture_id"],
                images=images,
                next_action="Agent inspects four images and writes board_observation.json",
            )
        except Exception as exc:
            self._write_state(initial, "observation_failed", error=str(exc))
            raise

    def decide(self) -> dict:
        state = self._state("awaiting_board_review")
        evidence = json.loads((self.root / "capture.json").read_text())
        observation = read_observation(self.root, evidence)
        decision = choose_move(observation["board"])
        pixel = annotate(self.root, decision, observation["grid_corners_px"])
        board = observation["board"]
        record = {
            **decision.to_dict(),
            "board_size": 19,
            "capture_id": evidence["capture_id"],
            "coordinate_convention": "1-based; left-head view rows down, columns right",
            "black": [[r + 1, c + 1] for r in range(19) for c in range(19) if board[r][c] == 1],
            "white": [[r + 1, c + 1] for r in range(19) for c in range(19) if board[r][c] == 2],
            "target_pixel_left_high": pixel,
            "images": evidence["images"],
            "observation_sha256": digest(self.root / "board_observation.json"),
            "physical_placement_verified": False,
        }
        save(self.root / "position_and_move.json", record)
        return self._write_state(
            state,
            "target_selected",
            selected_move=decision.to_dict(),
            decision_sha256=digest(self.root / "position_and_move.json"),
            next_action="Agent picks one stone, aligns to selected point and prepares return",
        )

    def prepare_release(self, adapter: HardwareAdapter) -> dict:
        state = self._state(("target_selected", "awaiting_release_confirmation"))
        current = self._owner(adapter, state)
        decision = self.root / "position_and_move.json"
        observation = self.root / "board_observation.json"
        if digest(decision) != state["decision_sha256"]:
            raise ValueError("Selected move changed; do not replace the autonomous decision")
        if digest(observation) != json.loads(decision.read_text())["observation_sha256"]:
            raise ValueError("Recognized position changed; observe the board again")
        alignment = json.loads((self.root / "alignment_observation.json").read_text())
        if (
            alignment.get("decision_sha256") != state["decision_sha256"]
            or alignment.get("matches_selected_intersection") is not True
            or alignment.get("target_empty_before_descent") is not True
            or alignment.get("single_black_stone_held") is not True
            or not isinstance(alignment.get("grid_neighbor_review"), str)
            or not alignment["grid_neighbor_review"].strip()
        ):
            raise ValueError(
                "Agent must verify held stone, empty target and neighbors against selected move"
            )
        stage = Path(alignment["stage_directory"]).resolve()
        if stage.parent != self.root or stage.name != current.get("stage"):
            raise ValueError("Alignment images must belong to this round")
        result = json.loads((stage / "result.json").read_text())
        held = current["commanded_q"]
        if (
            result.get("outcome") not in ("motion_complete_unverified", "reached")
            or result.get("status", {}).get("stage") != current.get("stage")
            or not close_vector(result["status"]["commanded_q"], held, 1e-8)
        ):
            raise ValueError(
                "Alignment review must refer to the current held command and completed stage"
            )
        metadata = json.loads((stage / "after/metadata.json").read_text())
        if set(metadata) != set(VIEWS) or not all(v.get("available") for v in metadata.values()):
            raise ValueError("All four alignment images must be available")
        image_digests = {name: digest(stage / "after" / f"{name}.jpg") for name in VIEWS}
        plan = json.loads((self.root / "return_plan.json").read_text())
        if (
            not close_vector(plan.get("seed"), held, 1e-8)
            or not plan.get("steps")
            or not close_vector(plan.get("locked_waist_q"), current["initial_waist"], 1e-8)
            or not plan.get("clearance_review", {}).get("passed")
            or any(step.get("grippers") for step in plan["steps"])
            or not close_vector(
                plan["steps"][-1]["joint_positions"][7:],
                adapter.reference()["joint_positions"][7:],
                1e-8,
            )
        ):
            raise ValueError(
                "Return must join current hold, preserve waist/gripper "
                "and finish at reset reference"
            )
        release_opening(self.root, adapter.config)
        checkpoint = {
            "release_parameters_sha256": digest(self.root / "release_parameters.json"),
            "held_command": held,
            "decision_sha256": digest(decision),
            "return_sha256": digest(self.root / "return_plan.json"),
            "alignment_sha256": digest(self.root / "alignment_observation.json"),
            "alignment_images": image_digests,
            "prepared_at_unix": time.time(),
        }
        save(self.root / "release_checkpoint.json", checkpoint)
        return self._write_state(
            state,
            "awaiting_release_confirmation",
            next_action="Ask the user to confirm release and the prepared return",
        )

    def release(self, adapter: HardwareAdapter, confirmation: str) -> dict:
        state = self._state("awaiting_release_confirmation")
        if not isinstance(confirmation, str) or not confirmation.strip():
            raise ValueError("Record the user confirmation before release")
        checkpoint = json.loads((self.root / "release_checkpoint.json").read_text())
        current = self._owner(adapter, state)
        for name, key in [
            ("release_parameters.json", "release_parameters_sha256"),
            ("position_and_move.json", "decision_sha256"),
            ("return_plan.json", "return_sha256"),
            ("alignment_observation.json", "alignment_sha256"),
        ]:
            if digest(self.root / name) != checkpoint[key]:
                raise ValueError(
                    "Prepared release changed after review; prepare again before confirmation"
                )
        if not close_vector(current["commanded_q"], checkpoint["held_command"], 1e-8):
            raise ValueError("Held command changed; do not reuse previous release approval")
        save(
            self.root / "release_authorization.json",
            {"approved": True, "confirmation": confirmation, "recorded_at_unix": time.time()},
        )
        self._write_state(state, "releasing_and_returning")
        try:
            final = adapter.release(self.root)
            joint_ok = close_vector(
                final["feedback"]["joint_positions"][7:],
                adapter.reference()["joint_positions"][7:],
                adapter.config["robot"]["joint_tolerance"],
            )
            status = "returned" if joint_ok else "return_unverified"
            return self._write_state(
                state,
                status,
                reset_joint_verified=joint_ok,
                final_feedback=final["feedback"],
                physical_placement_verified=False,
                next_action="Wait for the opponent and the user to start the next round",
            )
        except Exception as exc:
            self._write_state(
                state,
                "execution_failed",
                error=str(exc),
                next_action="Inspect current controller state; do not replay",
            )
            raise
