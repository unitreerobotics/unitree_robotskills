"""Hardware-free job validation and explicit import of precomputed legacy paths."""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from .config import vector, waist_vector

FORMAT = "astra-paint-execute/v1"
POSE_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


def matrix(value, label):
    result = np.asarray(value, dtype=float)
    if (
        result.ndim != 2
        or result.shape[1] != 14
        or len(result) < 1
        or not np.isfinite(result).all()
    ):
        raise ValueError(f"{label} needs a nonempty N by 14 finite joint path")
    return result


def fingerprint(job):
    raw = json.dumps(job, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(raw).hexdigest()


def load_job(path):
    return validate_job(json.loads(Path(path).read_text(encoding="utf-8")))


def contact_index(batch):
    if batch["phase"] == "WASH":
        lowest = min(point["z"] for point in batch["action"])
        index = max(i for i, point in enumerate(batch["action"]) if abs(point["z"] - lowest) < 1e-9)
        if batch.get("contact_index", index) != index:
            raise ValueError("water dwell must use the last lowest contact point")
        return index
    if batch["phase"] == "LOAD":
        index = batch.get("contact_index")
        if type(index) is not int or not 0 <= index < len(batch["action"]):
            raise ValueError("LOAD requires an explicit contact_index")
        return index
    return None


def lift_floor(batch):
    if batch["phase"] == "PAINT":
        indices = [
            i for stroke in batch["strokes"] for i in range(stroke["start"], stroke["end"] + 1)
        ]
        contact_z = max(batch["action"][i]["z"] for i in indices)
    else:
        contact_z = batch["action"][contact_index(batch)]["z"]
    return contact_z + batch["clearance_m"]


def validate_job(value):
    job = copy.deepcopy(value)
    if not isinstance(job, dict) or job.get("format") != FORMAT:
        raise ValueError(f"expected format={FORMAT}; import legacy jobs explicitly")
    if not isinstance(job.get("job_id"), str) or not job["job_id"].strip():
        raise ValueError("job_id is required")
    if job.get("purpose", "painting") not in {"painting", "simulation"}:
        raise ValueError("purpose must be painting or simulation")
    hand = job.setdefault("hand", "right")
    if hand not in {"left", "right"}:
        raise ValueError("hand must be left or right")
    cadence = job.setdefault("reload_every", 3)
    if type(cadence) is not int or not 1 <= cadence <= 64:
        raise ValueError("reload_every must be an integer from 1 to 64")
    grips = vector(job.get("grip_q2"), 2, "grip_q2")
    if np.any(grips < 0) or np.any(grips > 5.45):
        raise ValueError("Dex1 grip_q2 must use device units within 0..5.45")
    waist_vector(job.get("waist_q3"))
    entry = matrix(job.get("entry_q14"), "entry_q14")
    park = matrix(job.get("park_q14"), "park_q14")
    if not isinstance(job.get("clearance_review"), str) or not job["clearance_review"].strip():
        raise ValueError("clearance_review must describe the reviewed whole-arm/tool paths")
    if job.get("initial_brush", "clean") != "clean":
        raise ValueError("start with a clean brush; initial loaded-color state is unsupported")
    batches = job.get("batches")
    if not isinstance(batches, list) or not batches:
        raise ValueError("batches must be a nonempty list")
    frozen = slice(0, 7) if hand == "right" else slice(7, 14)
    active_grip = grips[1 if hand == "right" else 0]
    loaded, used, painted = None, 0, 0
    batch_ids, stroke_ids = set(), set()
    for batch_number, batch in enumerate(batches):
        if not isinstance(batch, dict):
            raise ValueError(f"batches[{batch_number}] must be a JSON object")
        batch_id = batch.get("id")
        if not isinstance(batch_id, str) or not batch_id or batch_id in batch_ids:
            raise ValueError("batch IDs must be nonempty and unique")
        batch_ids.add(batch_id)
        phase = batch.get("phase")
        if phase not in {"LOAD", "PAINT", "WASH"}:
            raise ValueError("only LOAD, PAINT and water WASH are supported")
        actions = batch.get("action")
        if not isinstance(actions, list) or not 1 <= len(actions) <= 4096:
            raise ValueError("batch.action needs 1..4096 EEF poses")
        for point_number, point in enumerate(actions):
            if not isinstance(point, dict):
                raise ValueError(
                    f"batches[{batch_number}].action[{point_number}] must be a JSON object"
                )
            vector([point.get(k) for k in POSE_KEYS], 6, "EEF pose")
            if "gripper" in point and abs(point["gripper"] - active_grip) > 1e-8:
                raise ValueError("batch gripper differs from the fixed job grip")
        path = matrix(batch.get("joint_path_q14"), "joint_path_q14")
        if len(path) != len(actions):
            raise ValueError("every EEF action needs a precomputed q14")
        if np.max(np.abs(path[:, frozen] - entry[-1, frozen])) > 1e-7:
            raise ValueError("nonpainting arm must remain frozen during batches")
        if batch.get("end_clear") is not True:
            raise ValueError("each batch needs a reviewed end_clear=true withdrawal")
        strokes = batch.setdefault("strokes", [])
        if not isinstance(strokes, list):
            raise ValueError("strokes must be a list")
        previous_end = -1
        for stroke_number, stroke in enumerate(strokes):
            if not isinstance(stroke, dict):
                raise ValueError(
                    f"batches[{batch_number}].strokes[{stroke_number}] must be a JSON object"
                )
            sid = stroke.get("id")
            start, end = stroke.get("start"), stroke.get("end")
            if not isinstance(sid, str) or not sid or sid in stroke_ids:
                raise ValueError("stroke IDs must be nonempty and unique")
            if (
                type(start) is not int
                or type(end) is not int
                or not previous_end + 1 < start <= end < len(actions) - 1
            ):
                raise ValueError("ordered contact ranges must have a later lifted action")
            if actions[start - 1]["z"] <= actions[start]["z"] or actions[end + 1]["z"] <= max(
                p["z"] for p in actions[start : end + 1]
            ):
                raise ValueError("each contact stroke requires approach and withdrawal clearance")
            previous_end = end
            stroke_ids.add(sid)
        color = batch.get("color")
        if phase == "PAINT":
            if not strokes or not color or color != loaded:
                raise ValueError("PAINT requires contact strokes and the loaded color")
            used += len(strokes)
            painted += len(strokes)
            if used > cadence:
                raise ValueError(f"batch {batch_number} exceeds reload_every={cadence}")
        else:
            if strokes:
                raise ValueError("material actions are not painting strokes")
            index = contact_index(batch)
            if index >= len(actions) - 1 or actions[-1]["z"] <= actions[index]["z"]:
                raise ValueError("material contact needs a later lifted endpoint")
            if phase == "LOAD":
                if not isinstance(color, str) or not color or color == "water":
                    raise ValueError("LOAD needs a pigment color")
                if loaded is not None and loaded != color:
                    raise ValueError("wash before changing color")
                loaded, used = color, 0
            else:
                if color != "water" or batch.get("wash_step", "water") != "water":
                    raise ValueError("only water washing is implemented")
                loaded, used = None, 0
        dwell = batch.setdefault("dwell_s", 3.0 if phase == "WASH" else 0.6)
        if type(dwell) not in (int, float) or not math.isfinite(dwell) or dwell < 0:
            raise ValueError("dwell_s must be finite and nonnegative")
        if phase == "WASH" and dwell < 3:
            raise ValueError("water washing needs at least 3 settled seconds")
        settle = batch.setdefault("contact_settle_s", 0.0)
        if type(settle) not in (int, float) or not math.isfinite(settle) or not 0 <= settle <= 1:
            raise ValueError("contact_settle_s must be finite and within 0..1 seconds")
        clearance = batch.setdefault("clearance_m", 0.05 if phase == "WASH" else 0.025)
        if type(clearance) not in (int, float) or not math.isfinite(clearance) or clearance <= 0:
            raise ValueError("clearance_m must be finite and positive")
        if actions[-1]["z"] < lift_floor(batch) - 1e-9:
            raise ValueError("batch endpoint does not provide its declared withdrawal clearance")
    if not painted or batches[-1]["phase"] != "WASH":
        raise ValueError("painting must contain strokes and finish with a water wash")
    if np.max(np.abs(park[0] - matrix(batches[-1]["joint_path_q14"], "last batch")[-1])) > 1e-8:
        raise ValueError("park route must start at the last lifted command")
    job["initial_brush"] = "clean"
    return job


def import_legacy_job(value, *, entry_q14, grip_q2, job_id):
    """Preserve numeric paths; does not transfer physical review or replay rights."""
    original = copy.deepcopy(value)
    execution = original["execution"]
    if execution.get("mode") != "continuous_precomputed":
        raise ValueError("only continuous_precomputed legacy jobs can be imported")
    entry = matrix(entry_q14, "reviewed entry route")
    if not np.allclose(entry[-1], execution["start_q14"], rtol=0, atol=1e-8):
        raise ValueError("entry route must end at the original compiled start")
    batches = copy.deepcopy(original["batches"])
    for batch in batches:
        if batch["phase"] == "LOAD":
            endpoint = batch.get("material_endpoint", {})
            if endpoint.get("canonical_key") != batch["color"]:
                raise ValueError("legacy material endpoint does not match color")
            batch["contact_index"] = endpoint["action_index"]
        elif batch["phase"] == "WASH":
            if batch.get("wash_step") != "water":
                raise ValueError("legacy blot/drying actions need a separate adapter")
            batch["color"] = "water"
    return validate_job(
        {
            "format": FORMAT,
            "job_id": job_id,
            "purpose": "painting",
            "hand": "right",
            "reload_every": original["reload_every"],
            "initial_brush": "clean",
            "grip_q2": list(grip_q2),
            "waist_q3": execution["required_waist_q3"],
            "entry_q14": entry.tolist(),
            "park_q14": execution["after_job_rest"]["outbound_q14"],
            "clearance_review": execution["clearance_evidence"],
            "batches": batches,
            "provenance": {
                "legacy_job_id": original["job_id"],
                "legacy_content_sha256": fingerprint(original),
                "physical_review_transferred": False,
                "replay_authorized": False,
            },
        }
    )
