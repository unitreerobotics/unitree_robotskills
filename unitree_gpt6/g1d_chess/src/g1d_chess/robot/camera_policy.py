"""Chess image selection by physical actor, independent of logical arm roles."""
import json
from functools import lru_cache
from pathlib import Path

HEAD_CAMERA = "cam_left_high"
AVAILABLE_CAMERAS = (HEAD_CAMERA, "cam_left_wrist", "cam_right_wrist")
from g1d_chess.paths import ROOT
POLICY_PATH = ROOT / "config/chess_camera_policy.json"


def validate_policy(policy):
    if (policy.get("schema_version") != 1 or policy.get("head_camera") != HEAD_CAMERA
            or policy.get("planning_camera") != HEAD_CAMERA):
        raise ValueError("chess planning/head camera must be cam_left_high")
    mapping = policy.get("wrist_stream_by_physical_arm", {})
    if (set(mapping) != {"left", "right"}
            or set(mapping.values()) != {"cam_left_wrist", "cam_right_wrist"}):
        raise ValueError("configure config/chess_camera_policy.json: physical left/right must map to distinct wrist streams; see docs/first_game_setup.md")
    return policy


@lru_cache(maxsize=1)
def _policy():
    # Loaded once per process, never reread during a held-piece checkpoint.
    return validate_policy(json.loads(POLICY_PATH.read_text()))


def planning_cameras():
    return [HEAD_CAMERA]


def camera_selection(active_arm):
    if active_arm not in ("left", "right"):
        raise ValueError("unknown physical execution arm")
    mapping = dict(_policy()["wrist_stream_by_physical_arm"])
    return {"schema_version": 1, "physical_actor": active_arm,
            "planning_cameras": planning_cameras(),
            "execution_cameras": [HEAD_CAMERA, mapping[active_arm]],
            "wrist_stream_by_physical_arm": mapping}


def execution_cameras(active_arm, selection=None):
    selection = camera_selection(active_arm) if selection is None else selection
    mapping = selection.get("wrist_stream_by_physical_arm", {})
    if (active_arm not in ("left", "right") or selection.get("physical_actor") != active_arm
            or set(mapping) != {"left", "right"}
            or set(mapping.values()) != {"cam_left_wrist", "cam_right_wrist"}):
        raise ValueError("camera selection belongs to a different or invalid physical actor")
    expected = [HEAD_CAMERA, mapping[active_arm]]
    if selection.get("execution_cameras") != expected:
        raise ValueError("execution requires head + physical actor wrist only")
    return expected


def checkpoint_cameras(checkpoint):
    observation = checkpoint["observation"]
    state = observation.get("state", {})
    actor = checkpoint.get("physical_actor", state.get("active_arm", "right"))
    selection = checkpoint.get("camera_selection", state.get("camera_selection"))
    expected = execution_cameras(actor, selection)
    order = checkpoint.get("camera_order", observation.get("camera_order", expected))
    if order != expected:
        raise ValueError("checkpoint must contain head + execution wrist in that order")
    if any(key not in observation["paths"] for key in expected):
        raise ValueError("checkpoint is missing its head or execution wrist image")
    return expected
