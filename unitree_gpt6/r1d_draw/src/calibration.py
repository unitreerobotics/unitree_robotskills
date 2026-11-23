"""Offline paper/material calibration and read-only robot snapshots.

Coordinates are brush-tip positions in the configured base frame, in meters;
paper coordinates are physical meters, not pixels or normalized fractions.
Contact flags and sources are supplied observations, never inferred from FK.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import time
from datetime import datetime, timezone

import numpy as np

MEASUREMENTS_FORMAT = "r1d-draw-measurements/v1"
CALIBRATION_FORMAT = "r1d-draw-calibration/v1"
OBSERVATION_FORMAT = "r1d-draw-observation/v1"
POSE_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")
UNITS = {"length": "m", "angle": "rad"}
# Poses are expressed in the model's configured base frame (the pelvis by default).
FRAME = "configured_base_frame"


def _array(value, shape, label):
    try:
        raw = np.asarray(value, dtype=object)
        if raw.shape != shape or any(
            type(x) not in (int, float, np.float64, np.int64) for x in raw.flat
        ):
            raise ValueError
        result = np.asarray(value, dtype=float)
        if not np.isfinite(result).all():
            raise ValueError
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain finite numbers with shape {shape}") from exc
    return result.copy()


def _number(value, label, *, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(f"{label} must be a finite {'positive ' if positive else ''}number")
    return float(value)


def _purpose(value):
    purpose = value.get("purpose")
    if purpose not in ("painting", "simulation"):
        raise ValueError("purpose must be painting or simulation")
    return purpose


def _evidence(value, purpose, label):
    if purpose == "painting" and (
        value.get("contact_verified") is not True
        or not isinstance(value.get("source"), str)
        or not value["source"].strip()
    ):
        raise ValueError(f"{label} requires contact_verified=true and a nonempty source")


def _digest(value):
    try:
        raw = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    except (TypeError, ValueError) as exc:
        raise ValueError("measurements must be finite JSON data") from exc
    return hashlib.sha256(raw).hexdigest()


def _paper_size(paper):
    if not isinstance(paper, dict):
        raise ValueError("paper must be an object")
    size = _array(paper.get("size_m"), (2,), "paper.size_m")
    if np.any(size <= 0):
        raise ValueError("paper.size_m must be positive")
    return size


def _uv(value, size, label):
    uv = _array(value, (2,), label)
    if np.any(uv < 0) or np.any(uv > size):
        raise ValueError(f"{label} leaves the paper's physical meter bounds")
    return uv


def _fit_paper(paper, purpose):
    if not isinstance(paper, dict):
        raise ValueError("paper must be an object")
    size = _paper_size(paper)
    tolerance = _number(paper.get("tolerance_m"), "paper.tolerance_m", positive=True)
    records = {}
    for group, minimum in (("anchors", 3), ("checks", 1)):
        points = paper.get(group)
        if not isinstance(points, list) or len(points) < minimum:
            raise ValueError(f"paper.{group} requires at least {minimum} points")
        records[group] = []
        for index, point in enumerate(points):
            label = f"paper.{group}[{index}]"
            if not isinstance(point, dict):
                raise ValueError(f"{label} must be an object")
            _evidence(point, purpose, label)
            uv = _uv(point.get("uv_m"), size, f"{label}.uv_m")
            xyz = _array(point.get("xyz_m"), (3,), f"{label}.xyz_m")
            records[group].append(
                {**copy.deepcopy(point), "uv_m": uv.tolist(), "xyz_m": xyz.tolist()}
            )
    anchor_uv = np.asarray([p["uv_m"] for p in records["anchors"]])
    check_uv = np.asarray([p["uv_m"] for p in records["checks"]])
    normalized = anchor_uv / size
    if any(
        np.linalg.norm(a - b) <= 1e-9 for i, a in enumerate(normalized) for b in normalized[i + 1 :]
    ):
        raise ValueError("paper anchors must have distinct coordinates")
    if any(np.linalg.norm(a - b / size) <= 1e-9 for a in normalized for b in check_uv):
        raise ValueError("independent paper checks must not reuse training coordinates")
    if any(
        np.linalg.norm(a - b) <= 1e-9
        for i, a in enumerate(check_uv / size)
        for b in (check_uv / size)[i + 1 :]
    ):
        raise ValueError("independent paper checks must have distinct coordinates")
    design_normalized = np.c_[normalized, np.ones(len(normalized))]
    rank = int(np.linalg.matrix_rank(design_normalized))
    condition = float(np.linalg.cond(design_normalized))
    if rank != 3 or not np.isfinite(condition) or condition > 1e6:
        raise ValueError(
            "paper anchors must span two dimensions without near-collinear distribution"
        )
    xyz = np.asarray([p["xyz_m"] for p in records["anchors"]])
    # Row-vector least squares yields coefficients.T @ [u,v,1]. Z is fitted too.
    coefficients = np.linalg.lstsq(design_normalized, xyz, rcond=None)[0]
    matrix = coefficients.T @ np.diag([1 / size[0], 1 / size[1], 1.0])
    if np.linalg.matrix_rank(matrix[:, :2]) != 2:
        raise ValueError("paper anchors describe a collapsed physical surface")
    maxima = {}
    for group, points in records.items():
        errors = []
        for point in points:
            predicted = matrix @ np.r_[point["uv_m"], 1.0]
            error = float(np.linalg.norm(predicted - point["xyz_m"]))
            if not np.isfinite(error) or error > tolerance:
                raise ValueError(
                    f"paper.{group} error {error:.6g}m exceeds tolerance {tolerance:.6g}m"
                )
            errors.append(error)
        maxima[group] = max(errors)
    report = {
        "status": "validated",
        "simulation": purpose == "simulation",
        "physical_contact_verified_by_tool": False,
        "contact_evidence": "source declarations" if purpose == "painting" else "synthetic",
        "normalized_design_rank": rank,
        "normalized_design_condition": condition,
        "anchor_uv_min_m": anchor_uv.min(axis=0).tolist(),
        "anchor_uv_max_m": anchor_uv.max(axis=0).tolist(),
        "anchor_max_error_m": maxima["anchors"],
        "check_max_error_m": maxima["checks"],
        "anchors": records["anchors"],
        "checks": records["checks"],
    }
    return {
        "size_m": size.tolist(),
        "matrix_uv1_to_xyz": matrix.tolist(),
        "tolerance_m": tolerance,
        "report": report,
    }


def _components(value, purpose):
    brush, materials, robot = (value.get(key) for key in ("brush", "materials", "robot"))
    if (
        not isinstance(brush, dict)
        or not isinstance(materials, dict)
        or not isinstance(robot, dict)
    ):
        raise ValueError("brush, materials and robot must be objects")
    _evidence(brush, purpose, "brush")
    _array(brush.get("rpy_rad"), (3,), "brush.rpy_rad")
    for name in ("width_m", "max_stroke_length_m", "clearance_m"):
        _number(brush.get(name), f"brush.{name}", positive=True)
    if robot.get("hand") not in ("left", "right"):
        raise ValueError("robot.hand must be left or right")
    grips = _array(robot.get("grip_q2"), (2,), "robot.grip_q2")
    if np.any(grips < 0) or np.any(grips > 5.45):
        raise ValueError("robot.grip_q2 must use Dex1 units within 0..5.45")
    _number(robot.get("waist_yaw"), "robot.waist_yaw")
    entry, park = robot.get("entry_q14"), robot.get("park_q14")
    if not isinstance(entry, list) or not entry:
        raise ValueError("robot.entry_q14 requires a reviewed joint route")
    for q in entry:
        _array(q, (14,), "robot.entry_q14 waypoint")
    if not isinstance(park, list) or len(park) < 2 or park[0] is not None:
        raise ValueError("robot.park_q14 must start with null followed by a reviewed route")
    for q in park[1:]:
        _array(q, (14,), "robot.park_q14 waypoint")
    if not isinstance(robot.get("clearance_review"), str) or not robot["clearance_review"].strip():
        raise ValueError("robot.clearance_review is required")
    if "transfer_z_m" in robot:
        _number(robot["transfer_z_m"], "robot.transfer_z_m")
    if not materials:
        raise ValueError("materials requires at least one pigment station")
    for color, station in materials.items():
        if not isinstance(color, str) or not color.strip() or not isinstance(station, dict):
            raise ValueError("each material needs a name and station object")
        _evidence(station, purpose, f"materials.{color}")
        action, index = station.get("action"), station.get("contact_index")
        if not isinstance(action, list) or len(action) < 2:
            raise ValueError(f"materials.{color}.action needs contact and withdrawal poses")
        for point in action:
            if not isinstance(point, dict):
                raise ValueError("material action poses must be xyz/rpy objects")
            _array([point.get(k) for k in POSE_KEYS], (6,), f"materials.{color} pose")
        if type(index) is not int or not 0 <= index < len(action) - 1:
            raise ValueError(f"materials.{color}.contact_index needs a later withdrawal pose")
        clearance = _number(
            station.get("clearance_m"), f"materials.{color}.clearance_m", positive=True
        )
        if action[-1]["z"] < action[index]["z"] + clearance - 1e-10:
            raise ValueError(f"materials.{color} has insufficient withdrawal clearance")
        if _number(station.get("dwell_s"), f"materials.{color}.dwell_s") < 0:
            raise ValueError("material dwell must be nonnegative")


def calibrate_measurements(value: dict) -> dict:
    """Fit a tilted affine paper surface and independently verify every sample."""
    if not isinstance(value, dict) or value.get("format") != MEASUREMENTS_FORMAT:
        raise ValueError(f"expected format={MEASUREMENTS_FORMAT}")
    purpose = _purpose(value)
    _components(value, purpose)
    paper = _fit_paper(value.get("paper"), purpose)
    result = {
        "format": CALIBRATION_FORMAT,
        "purpose": purpose,
        "units": dict(UNITS),
        "coordinate_frame": FRAME,
        "paper": paper,
        **{key: copy.deepcopy(value[key]) for key in ("brush", "materials", "robot")},
        "provenance": {
            "measurements_format": MEASUREMENTS_FORMAT,
            "measurements_sha256": _digest(value),
            "physical_contact_verified_by_tool": False,
            "simulation": purpose == "simulation",
        },
    }
    return result


def validate_calibration(value: dict) -> dict:
    """Validate a saved artifact using its embedded, self-contained observations."""
    if not isinstance(value, dict) or value.get("format") != CALIBRATION_FORMAT:
        raise ValueError(f"expected format={CALIBRATION_FORMAT}")
    purpose = _purpose(value)
    if value.get("units") != UNITS or value.get("coordinate_frame") != FRAME:
        raise ValueError("calibration must use meters/radians in the configured base frame")
    _components(value, purpose)
    paper = value.get("paper")
    if not isinstance(paper, dict) or not isinstance(paper.get("report"), dict):
        raise ValueError("calibration paper requires its self-contained validation report")
    report = paper["report"]
    fitted = _fit_paper(
        {
            "size_m": paper.get("size_m"),
            "tolerance_m": paper.get("tolerance_m"),
            "anchors": report.get("anchors"),
            "checks": report.get("checks"),
        },
        purpose,
    )
    matrix = _array(paper.get("matrix_uv1_to_xyz"), (3, 3), "paper.matrix_uv1_to_xyz")
    if not np.allclose(matrix, fitted["matrix_uv1_to_xyz"], atol=1e-10, rtol=1e-10):
        raise ValueError("paper matrix no longer matches the embedded measurements")
    if (
        report.get("status") != "validated"
        or report.get("simulation") is not (purpose == "simulation")
        or report.get("physical_contact_verified_by_tool") is not False
    ):
        raise ValueError(
            "paper report has inconsistent simulation/contact verification declarations"
        )
    provenance = value.get("provenance", {})
    digest = provenance.get("measurements_sha256")
    if (
        not isinstance(digest, str)
        or len(digest) != 64
        or any(c not in "0123456789abcdef" for c in digest)
        or provenance.get("physical_contact_verified_by_tool") is not False
        or provenance.get("simulation") is not (purpose == "simulation")
    ):
        raise ValueError("calibration requires consistent measurement provenance")
    _digest(value)
    return copy.deepcopy(value)


def map_paper(calibration, uv_m) -> np.ndarray:
    """Map one in-bounds paper point in meters to configured task-frame XYZ."""
    if not isinstance(calibration, dict) or calibration.get("format") != CALIBRATION_FORMAT:
        raise ValueError(f"expected format={CALIBRATION_FORMAT}")
    _purpose(calibration)
    if calibration.get("units") != UNITS or calibration.get("coordinate_frame") != FRAME:
        raise ValueError("paper mapping requires meters/radians in the configured base frame")
    paper = calibration.get("paper", {})
    size = _paper_size(paper)
    uv = _uv(uv_m, size, "uv_m")
    matrix = _array(paper.get("matrix_uv1_to_xyz"), (3, 3), "paper.matrix_uv1_to_xyz")
    if np.linalg.matrix_rank(matrix[:, :2]) != 2:
        raise ValueError("paper matrix describes a collapsed surface")
    return matrix @ np.r_[uv, 1.0]


def _rpy(rotation):
    # XYZ Euler extraction uses the same Rz(yaw) Ry(pitch) Rx(roll) convention
    # as the kinematics module, including a deterministic gimbal-lock branch.
    pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))
    if math.hypot(rotation[0, 0], rotation[1, 0]) > 1e-9:
        return [
            math.atan2(rotation[2, 1], rotation[2, 2]),
            pitch,
            math.atan2(rotation[1, 0], rotation[0, 0]),
        ]
    return [math.atan2(-rotation[1, 2], rotation[1, 1]), pitch, 0.0]


def observe(config, robot, kinematics, label) -> dict:
    """One read-only snapshot: measured joints and the brush-tip pose of both hands.

    Calls only ``robot.state()``. FK describes the configured tool frames; it does
    not establish that a brush touches anything.
    """
    if not isinstance(label, str) or not label.strip():
        raise ValueError("observation label is required")
    state = robot.state()
    timestamp = time.time()
    q14 = _array(state["arms"]["left"] + state["arms"]["right"], (14,), "observed arms")
    tools = {}
    for side in ("left", "right"):
        transform = _array(kinematics.fk(q14, side), (4, 4), f"{side} FK")
        tools[side] = {
            "xyz_m": transform[:3, 3].tolist(),
            "rpy_rad": _rpy(transform[:3, :3]),
            "transform": transform.tolist(),
        }
    model = config.get("model", {})
    return {
        "format": OBSERVATION_FORMAT,
        "label": label,
        "timestamp_unix": timestamp,
        "timestamp_utc": datetime.fromtimestamp(timestamp, timezone.utc).isoformat(),
        "units": dict(UNITS),
        "coordinate_frame": FRAME,
        "model": {
            key: model.get(key) for key in ("urdf", "base_frame", "left_frame", "right_frame")
        },
        "q14": q14.tolist(),
        "grip_q2": list(state["grip"]),
        "waist_yaw": state["waist_yaw"],
        "head_q2": list(state["head"]),
        "lift": state["lift"],
        "commanded": state["commanded"],
        "tools": tools,
        "contact_verified": False,
    }
