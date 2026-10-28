"""Validate the host Agent's visual recognition against this round's raw images.

Vision belongs to the image-capable Agent running SKILL.md. This module does
not pretend that reading a JSON matrix is camera recognition or calibration.
"""

from __future__ import annotations

import base64
import hashlib
import json
import math
from pathlib import Path
from typing import Any
from xml.sax.saxutils import escape

from .gomoku_engine import Decision, validate_board

VIEWS = ("cam_left_high", "cam_right_high", "cam_left_wrist", "cam_right_wrist")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_observation(round_dir: Path, evidence: dict[str, Any]) -> dict[str, Any]:
    path = round_dir / "board_observation.json"
    if not path.is_file():
        raise ValueError(
            "Agent must inspect this round's four images and write board_observation.json"
        )
    value = json.loads(path.read_text(encoding="utf-8"))
    allowed = {
        "capture_id",
        "recognition_source",
        "board",
        "views_agree",
        "uncertain_intersections",
        "grid_corners_px",
        "notes",
    }
    if not isinstance(value, dict) or set(value) != allowed:
        raise ValueError(
            "Observation must contain exactly the documented perception fields; no target"
        )
    if value["capture_id"] != evidence["capture_id"]:
        raise ValueError("Observation belongs to a different image capture")
    if value["recognition_source"] != "agent_visual_review":
        raise ValueError("An image-capable Agent must review the original camera images")
    if value["views_agree"] is not True or value["uncertain_intersections"] != []:
        raise ValueError("Resolve uncertain intersections and cross-check both head views first")
    if not isinstance(value["notes"], str) or not value["notes"].strip():
        raise ValueError("Record the Agent's visual review, including grid orientation")
    for name in VIEWS:
        image = round_dir / "board_capture" / (name + ".jpg")
        if digest(image) != evidence["images"][name]["sha256"]:
            raise ValueError("Captured images changed; observe and recognize the board again")
    value["board"] = validate_board(value["board"])
    corners = value["grid_corners_px"]
    if (
        not isinstance(corners, list)
        or len(corners) != 4
        or any(
            not isinstance(p, list)
            or len(p) != 2
            or any(type(x) not in (int, float) or not math.isfinite(x) or x < 0 for x in p)
            for p in corners
        )
    ):
        raise ValueError("grid_corners_px requires four finite image points: TL, TR, BR, BL")
    metadata = json.loads((round_dir / "board_capture/metadata.json").read_text())
    height, width = metadata["cam_left_high"]["shape"]
    if any(x >= width or y >= height for x, y in corners):
        raise ValueError("Grid corner falls outside the original left-head image")
    crosses = []
    for i in range(4):
        a, b, c = corners[i], corners[(i + 1) % 4], corners[(i + 2) % 4]
        crosses.append((b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]))
    if not all(x > 1 for x in crosses):
        raise ValueError("Grid corners must form a nondegenerate clockwise quadrilateral")
    return value


def project_intersection(corners: list[list[float]], row: int, column: int) -> list[float]:
    """Fit the grid plane's projective map, without NumPy or a robot transform."""
    equations = []
    for (u, v), (x, y) in zip(((0, 0), (1, 0), (1, 1), (0, 1)), corners, strict=True):
        equations.append([u, v, 1, 0, 0, 0, -x * u, -x * v, x])
        equations.append([0, 0, 0, u, v, 1, -y * u, -y * v, y])
    for i in range(8):
        pivot = max(range(i, 8), key=lambda j: abs(equations[j][i]))
        equations[i], equations[pivot] = equations[pivot], equations[i]
        divisor = equations[i][i]
        if abs(divisor) < 1e-10:
            raise ValueError("Grid projection is singular")
        equations[i] = [x / divisor for x in equations[i]]
        for j in range(8):
            if j != i:
                factor = equations[j][i]
                equations[j] = [
                    a - factor * b for a, b in zip(equations[j], equations[i], strict=True)
                ]
    h = [equations[i][8] for i in range(8)]
    u, v = (column - 1) / 18, (row - 1) / 18
    scale = h[6] * u + h[7] * v + 1
    if abs(scale) < 1e-10:
        raise ValueError("Intersection projects to infinity")
    return [(h[0] * u + h[1] * v + h[2]) / scale, (h[3] * u + h[4] * v + h[5]) / scale]


def annotate(round_dir: Path, decision: Decision, corners: list[list[float]]) -> list[float]:
    """Mark one grid intersection on a copy of the left-head image as self-contained SVG."""
    x, y = project_intersection(corners, decision.row, decision.column)
    # SVG embeds the unmodified camera image; it can be opened without external files.
    image = base64.b64encode((round_dir / "board_capture/cam_left_high.jpg").read_bytes()).decode()
    metadata = json.loads((round_dir / "board_capture/metadata.json").read_text())
    height, width = metadata["cam_left_high"]["shape"]
    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">'
        f'<image width="{width}" height="{height}" href="data:image/jpeg;base64,{image}"/>'
        f'<circle cx="{x}" cy="{y}" r="10" stroke="red" stroke-width="3" fill="none"/>'
        f'<text x="{x + 14}" y="{y - 14}" fill="red" font-size="18">'
        f"黑落点：{decision.row}行{decision.column}列</text>"
        f'<text x="12" y="24" fill="red" font-size="18">'
        f"{escape(decision.reason)}</text></svg>"
    )
    (round_dir / "suggested_black_move.svg").write_text(svg, encoding="utf-8")
    return [x, y]
