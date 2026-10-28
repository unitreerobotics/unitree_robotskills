"""Offline paper/material calibration and explicitly read-only device snapshots.

Coordinates are configured task-frame positions in the URDF root, in meters;
paper coordinates are physical meters, not pixels or normalized fractions.
Contact flags and sources are supplied observations, never inferred from FK.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from .config import waist_vector

MEASUREMENTS_FORMAT = "astra-paint-measurements/v1"
CALIBRATION_FORMAT = "astra-paint-calibration/v1"
OBSERVATION_FORMAT = "astra-paint-observation/v1"
POSE_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")
UNITS = {"length": "m", "angle": "rad"}


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
    waist_vector(_array(robot.get("waist_q3"), (3,), "robot.waist_q3"), "robot.waist_q3")
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
    if "water" not in materials or len(materials) < 2:
        raise ValueError("materials requires water and at least one pigment")
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
        dwell = _number(station.get("dwell_s"), f"materials.{color}.dwell_s")
        if dwell < 0 or (color == "water" and dwell < 3):
            raise ValueError(
                "material dwell must be nonnegative; water requires at least 3 seconds"
            )
        if color == "water":
            lowest = min(point["z"] for point in action)
            if index != max(i for i, point in enumerate(action) if point["z"] == lowest):
                raise ValueError("water contact_index must be its final lowest point")


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
        "coordinate_frame": "configured_urdf_root",
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
    if value.get("units") != UNITS or value.get("coordinate_frame") != "configured_urdf_root":
        raise ValueError("calibration must use meters/radians in the configured URDF root frame")
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
    if (
        calibration.get("units") != UNITS
        or calibration.get("coordinate_frame") != "configured_urdf_root"
    ):
        raise ValueError("paper mapping requires meters/radians in the configured URDF root frame")
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


def _model_summary(config, simulated):
    settings = config.get("model", {})
    summary = {
        "simulation": simulated,
        "urdf": None,
        "frames": {hand: settings.get(f"{hand}_frame") for hand in ("left", "right")},
        "tool_transforms": {
            hand: settings.get(f"{hand}_tool_transform", np.eye(4).tolist())
            for hand in ("left", "right")
        },
        "reference_positions": copy.deepcopy(settings.get("reference_positions", {})),
    }
    if not simulated or settings.get("urdf"):
        if not isinstance(settings.get("urdf"), str) or not settings["urdf"].strip():
            raise ValueError("physical observation requires model.urdf")
        path = Path(settings["urdf"]).expanduser().resolve()
        raw = path.read_bytes()
        root = ET.fromstring(raw)
        roots = {link.attrib["name"] for link in root.findall("link")} - {
            child.attrib["link"] for child in root.findall("joint/child")
        }
        if len(roots) != 1:
            raise ValueError("URDF must identify one root coordinate frame")
        summary["urdf"] = {
            "path": str(path),
            "sha256": hashlib.sha256(raw).hexdigest(),
            "root_link": next(iter(roots)),
        }
    return summary


def observe_device(config, backend, kinematics, label) -> dict:
    """Connect a fresh backend for one feedback/FK snapshot, then close it.

    Never calls acquire, command, or hold. FK positions describe the configured
    task frames; they do not establish brush contact or camera observations.
    """
    if not isinstance(label, str) or not label.strip():
        raise ValueError("observation label is required")
    hand = config.get("hand", "right")
    if hand not in ("left", "right"):
        raise ValueError("hand must be left or right")
    simulated = getattr(backend, "is_simulation", False) is True
    if simulated != (getattr(kinematics, "is_simulation", False) is True):
        raise ValueError("synthetic kinematics and physical feedback cannot be mixed")
    # Reject a borrowed connection before the finally block can close it. In
    # particular, a failed reconnect must not release an active controller.
    if any(
        getattr(backend, name, False) is True
        for name in ("connected", "acquired", "_connected", "_acquired", "_acquiring", "_closed")
    ):
        raise ValueError("observation requires a fresh, unconnected backend")
    summary = _model_summary(config, simulated)
    try:
        backend.connect()
        sample = backend.read()
        timestamp = time.time()
        q14 = _array(sample.get("q14"), (14,), "observed q14")
        grips = _array(sample.get("grip_q2"), (2,), "observed grip_q2")
        waist = waist_vector(
            _array(sample.get("waist_q3"), (3,), "observed waist_q3"), "observed waist_q3"
        )
        if not simulated:
            references = summary["reference_positions"]
            required = _array(
                [references.get(f"waist_{axis}_joint") for axis in ("yaw", "roll", "pitch")],
                (3,),
                "model waist reference_positions",
            )
            tolerance = _number(
                config.get("motion", {}).get("waist_tolerance_rad", 0.03),
                "waist_tolerance_rad",
                positive=True,
            )
            if np.max(np.abs(waist - required)) > tolerance:
                raise ValueError("observed waist differs from the FK model reference frame")
        tools = {}
        for side in ("left", "right"):
            transform = _array(kinematics.fk(q14, side), (4, 4), f"{side} FK")
            rotation = transform[:3, :3]
            if (
                not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7)
                or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-7)
            ):
                raise ValueError("observed FK must be a rigid homogeneous transform")
            tools[side] = {
                "xyz_m": transform[:3, 3].tolist(),
                "rpy_rad": _rpy(rotation),
                "transform": transform.tolist(),
            }
        return {
            "format": OBSERVATION_FORMAT,
            "purpose": "simulation" if simulated else "painting",
            "label": label,
            "hand": hand,
            "timestamp_unix": timestamp,
            "timestamp_utc": datetime.fromtimestamp(timestamp, timezone.utc).isoformat(),
            "units": dict(UNITS),
            "coordinate_frame": "configured_urdf_root",
            "model": summary,
            "q14": q14.tolist(),
            "grip_q2": grips.tolist(),
            "waist_q3": waist.tolist(),
            "tools": tools,
            "contact_verified": False,
            "physical_execution": False,
            "timestamp_policy": "host time after feedback read; not hardware-synchronized",
        }
    finally:
        backend.close()
