"""Hardware-free validation of a compiled painting job."""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path

import numpy as np

from .schema import POSE_KEYS, digest, number, vector

FORMAT = "r1d-draw-job/v1"
TRAVEL_SPACING_M = 0.01  # spacing of the targets along a straight move through the air


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
    return digest(job)


def load_job(path):
    return validate_job(json.loads(Path(path).read_text(encoding="utf-8")))


def contact_index(batch):
    """Index of the pose where a LOAD or WASH batch touches its material, else None."""
    if batch["phase"] in ("LOAD", "WASH"):
        index = batch.get("contact_index")
        if type(index) is not int or not 0 <= index < len(batch["action"]):
            raise ValueError(f"{batch['phase']} requires an explicit contact_index")
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
        raise ValueError(f"expected format={FORMAT}")
    if not isinstance(job.get("job_id"), str) or not job["job_id"].strip():
        raise ValueError("job_id is required")
    if job.get("purpose", "painting") not in {"painting", "simulation"}:
        raise ValueError("purpose must be painting or simulation")
    hand = job.setdefault("hand", "right")
    if hand not in {"left", "right"}:
        raise ValueError("hand must be left or right")
    grips = vector(job.get("grip_q2"), 2, "grip_q2")
    if np.any(grips < 0) or np.any(grips > 5.45):
        raise ValueError("Dex1 grip_q2 must use device units within 0..5.45")
    number(job.get("waist_yaw"), "waist_yaw")
    entry = matrix(job.get("entry_q14"), "entry_q14")
    park = matrix(job.get("park_q14"), "park_q14")
    if not isinstance(job.get("clearance_review"), str) or not job["clearance_review"].strip():
        raise ValueError("clearance_review must describe the reviewed whole-arm/tool paths")
    batches = job.get("batches")
    if not isinstance(batches, list) or not batches:
        raise ValueError("batches must be a nonempty list")
    frozen = slice(0, 7) if hand == "right" else slice(7, 14)
    active_grip = grips[1 if hand == "right" else 0]
    painted = 0
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
            raise ValueError("phase must be LOAD, PAINT or WASH")
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
        travel = batch.setdefault("travel_q14", [])
        for q in [*path, *(matrix(travel, "travel_q14") if travel else [])]:
            if np.max(np.abs(q[frozen] - entry[-1, frozen])) > 1e-7:
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
        if not isinstance(color, str) or not color:
            raise ValueError("every batch names its color; a wash uses its water station")
        if phase == "PAINT":
            if not strokes:
                raise ValueError("PAINT requires contact strokes")
            painted += len(strokes)
        else:
            if strokes:
                raise ValueError("material actions are not painting strokes")
            index = contact_index(batch)
            if index >= len(actions) - 1 or actions[-1]["z"] <= actions[index]["z"]:
                raise ValueError("material contact needs a later lifted endpoint")
        dwell = batch.setdefault("dwell_s", 0.0)
        if type(dwell) not in (int, float) or not math.isfinite(dwell) or dwell < 0:
            raise ValueError("dwell_s must be finite and nonnegative")
        settle = batch.setdefault("contact_settle_s", 0.0)
        if type(settle) not in (int, float) or not math.isfinite(settle) or not 0 <= settle <= 1:
            raise ValueError("contact_settle_s must be finite and within 0..1 seconds")
        clearance = batch.get("clearance_m")
        if type(clearance) not in (int, float) or not math.isfinite(clearance) or clearance <= 0:
            raise ValueError("clearance_m must be finite and positive")
        if actions[-1]["z"] < lift_floor(batch) - 1e-9:
            raise ValueError("batch endpoint does not provide its declared withdrawal clearance")
    if not painted:
        raise ValueError("a painting job must contain strokes")
    if np.max(np.abs(park[0] - matrix(batches[-1]["joint_path_q14"], "last batch")[-1])) > 1e-8:
        raise ValueError("park route must start at the last lifted command")
    return job
