"""Live tools used by the image-capable host Agent; no coordinate-mode adapter."""

from __future__ import annotations

import contextlib
import json
import math
import time
from pathlib import Path

from .board_vision import VIEWS, digest


def close_vector(a, b, tolerance: float) -> bool:
    return (
        isinstance(a, list)
        and isinstance(b, list)
        and len(a) == len(b)
        and len(a) > 0
        and all(
            type(x) in (int, float)
            and type(y) in (int, float)
            and math.isfinite(x)
            and math.isfinite(y)
            and abs(x - y) <= tolerance
            for x, y in zip(a, b, strict=True)
        )
    )


def release_opening(round_dir, config):
    """Require a locally planned opening; never supply an empirical fallback."""
    parameters = json.loads((Path(round_dir) / "release_parameters.json").read_text())
    value = parameters.get("right_gripper_rad")
    limits = config["robot"]["grippers"]["right"]
    if (
        type(value) not in (int, float)
        or not math.isfinite(value)
        or not limits["min"] <= value <= limits["max"]
        or not isinstance(parameters.get("review"), str)
        or not parameters["review"].strip()
    ):
        raise ValueError("Plan and review this round's release opening within configured limits")
    return float(value)


class HardwareAdapter:
    """Observe and release through the existing single continuous controller.

    The host Agent runs the low-level movement tools between these calls using
    current images, FK/IK and the operating guide. There is no blind pixel-to-TCP
    conversion or historical trajectory replay hidden in this adapter.
    """

    def __init__(self, config_path: Path, owner_root: Path):
        from .hardware.common import configure, load_config, runtime_dir
        from .hardware.main import validate_hardware_mode

        configure(config_path)
        self.config = load_config()
        validate_hardware_mode(self.config)
        self.config_path = Path(self.config["_config_path"])
        self.owner_root = (
            owner_root if owner_root.is_absolute() else runtime_dir(self.config) / owner_root
        ).resolve()

    def status(self) -> dict:
        from .hardware.gomoku_live_control import read

        return read(self.owner_root)

    def reference(self) -> dict:
        from .hardware.common import reset_state_path
        from .hardware.main import validate_reset_reference

        reference = json.loads(reset_state_path(self.config).read_text())
        validate_reset_reference(reference)
        return reference

    def call(self, round_dir: Path, *arguments: str) -> None:
        from .hardware.gomoku_live_control import main

        with (round_dir / "tool_output.log").open("a", encoding="utf-8") as stream:
            with contextlib.redirect_stdout(stream):
                code = main(
                    [
                        "--config",
                        str(self.config_path),
                        "--root",
                        str(round_dir),
                        "--owner-root",
                        str(self.owner_root),
                        "--quiet",
                        *arguments,
                    ]
                )
        if code:
            raise RuntimeError(
                "Hardware tool failed; inspect round log and current controller state"
            )

    def capture(self, round_dir: Path) -> dict:
        before = self.status()
        right = before["feedback"]["joint_positions"][7:]
        if not close_vector(
            right, self.reference()["joint_positions"][7:], self.config["robot"]["joint_tolerance"]
        ):
            raise ValueError(
                "Observe the board from reset_right rest; reset before starting this round"
            )
        started = time.time()
        self.call(round_dir, "new_round")
        deadline = time.monotonic() + 5
        while True:
            cameras = self.status().get("cameras", {})
            if set(cameras) == set(VIEWS) and all(
                item.get("available") and item.get("received_at", 0) >= started
                for item in cameras.values()
            ):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("Four fresh camera views did not become available")
            time.sleep(0.1)
        self.call(round_dir, "observe", "board_capture")
        metadata = json.loads((round_dir / "board_capture/metadata.json").read_text())
        if set(metadata) != set(VIEWS) or not all(v.get("available") for v in metadata.values()):
            raise ValueError("A fresh capture from all four cameras is required")
        for name in VIEWS:
            if not (round_dir / "board_capture" / f"{name}.jpg").is_file():
                raise ValueError("Camera capture is missing an original image")
        after = self.status()
        if before["pid"] != after["pid"]:
            raise ValueError("Controller changed during observation")
        return after

    def release(self, round_dir: Path) -> dict:
        self.call(
            round_dir,
            "--execute",
            "release",
            "release",
            "--gripper",
            str(release_opening(round_dir, self.config)),
            "--return-plan",
            str(round_dir / "return_plan.json"),
            "--review",
            "Agent verified selected intersection and prepared return before confirmation",
        )
        return self.status()

    def provenance(self) -> dict:
        return {
            "config_path": str(self.config_path),
            "config_sha256": digest(self.config_path),
            "owner_root": str(self.owner_root),
        }
