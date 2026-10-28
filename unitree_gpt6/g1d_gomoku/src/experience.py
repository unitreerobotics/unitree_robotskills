"""Workspace-only, evidence-bound experience storage; no robot access or motion defaults."""

from __future__ import annotations

import argparse
import contextlib
import fcntl
import hashlib
import json
import math
import uuid
from datetime import datetime, timezone
from pathlib import Path

from .agent import save

CONTEXT_KEYS = {
    "robot_id",
    "scene_id",
    "config_sha256",
    "calibration_sha256",
    "model_sha256",
    "reset_sha256",
}
CHECKS = {
    "geometry": "calibration_verified",
    "grasp": "single_stone_held",
    "alignment": "selected_intersection_verified",
    "release": "release_completed",
    "return": "reset_joint_verified",
    "placement": "physical_placement_verified",
}


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def inside(path, root):
    path = Path(path).expanduser().resolve()
    if not path.is_relative_to(root):
        raise ValueError("Experience files and evidence must stay inside the workspace")
    return path


def workspace(path):
    root = Path(path).expanduser().resolve()
    package = Path(__file__).resolve().parents[1]
    repository = package.parents[1]
    if root == repository or root.is_relative_to(repository / "unitree_gpt6"):
        raise ValueError("Use a runtime workspace, not the source directory")
    if root.is_relative_to(repository / ".git"):
        raise ValueError("Use a runtime workspace, not a package or Git directory")
    inside(root / "experience", root)
    return root


def validate_context(value):
    if not isinstance(value, dict) or set(value) != CONTEXT_KEYS:
        raise ValueError("Experience context requires robot, scene, config, calibration and model")
    for key, item in value.items():
        if not isinstance(item, str) or not item.strip():
            raise ValueError(f"Missing experience context: {key}")
        if key.endswith("sha256") and (
            len(item) != 64 or any(c not in "0123456789abcdef" for c in item)
        ):
            raise ValueError(f"Invalid digest: {key}")
    return value


def finite_data(value):
    if value is None:
        raise ValueError("Do not store unknown values as experience")
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("Experience values must be finite")
    if isinstance(value, dict):
        for item in value.values():
            finite_data(item)
    elif isinstance(value, list):
        for item in value:
            finite_data(item)


@contextlib.contextmanager
def store_lock(root):
    directory = inside(root / "experience", root)
    directory.mkdir(parents=True, exist_ok=True)
    with inside(directory / ".lock", root).open("a+") as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        yield directory


def initialize(root):
    root = workspace(root)
    with store_lock(root) as directory:
        path = inside(directory / "records.json", root)
        if not path.exists():
            save(path, {"schema_version": 1, "records": []})
    return {"task_status": "experience_initialized"}


def make_context(root, robot_id, scene_id, config, calibration):
    root = workspace(root)
    config = inside(config, root)
    calibration = inside(calibration, root)
    from .hardware.common import (
        load_config,
        reset_state_path,
        scene_calibration_path,
        scene_geometry,
    )

    settings = load_config(config)
    reset = inside(reset_state_path(settings), root)
    if inside(scene_calibration_path(settings), root) != calibration:
        raise ValueError("Bind the scene calibration actually used by the motion tools")
    scene_geometry(settings)
    from unitree_gpt6.g1d_gomoku.models.g1.resources import asset_directory

    context = validate_context(
        {
            "robot_id": robot_id,
            "scene_id": scene_id,
            "config_sha256": sha256(config),
            "calibration_sha256": sha256(calibration),
            "reset_sha256": sha256(reset),
            "model_sha256": sha256(asset_directory().parent / "manifest.json"),
        }
    )
    with store_lock(root) as directory:
        save(inside(directory / "context.json", root), context)
        save(
            inside(directory / "binding.json", root),
            {
                "config": str(config.relative_to(root)),
                "calibration": str(calibration.relative_to(root)),
                "reset": str(reset.relative_to(root)),
            },
        )
    return {"task_status": "experience_context_saved"}


def active_context(root):
    from unitree_gpt6.g1d_gomoku.models.g1.resources import asset_directory

    context = validate_context(read(inside(root / "experience/context.json", root)))
    binding = read(inside(root / "experience/binding.json", root))
    for field in ("config", "calibration", "reset"):
        if sha256(inside(root / binding[field], root)) != context[field + "_sha256"]:
            raise ValueError("Deployment changed; rebuild experience context before reuse")
    if sha256(asset_directory().parent / "manifest.json") != context["model_sha256"]:
        raise ValueError("Robot model changed; rebuild experience context before reuse")
    return context


def load_store(root):
    path = inside(root / "experience/records.json", root)
    data = read(path) if path.exists() else {"schema_version": 1, "records": []}
    if data.get("schema_version") != 1 or not isinstance(data.get("records"), list):
        raise ValueError("Unsupported experience store")
    finite_data(data)
    return data


def record(root, round_dir, candidate_path):
    root = workspace(root)
    round_dir = inside(round_dir, root)
    state_path = inside(round_dir / "round.json", root)
    state = read(state_path)
    if state.get("task_status") != "returned" or state.get("reset_joint_verified") is not True:
        raise ValueError("Only a successfully returned round can publish experience")
    if not isinstance(state.get("round_id"), str) or not state["round_id"].strip():
        raise ValueError("Missing round identity")
    candidate = read(inside(candidate_path, round_dir))
    finite_data(candidate)
    context = validate_context(candidate.get("context"))
    active = active_context(root)
    if context != active or state.get("config_sha256") != context["config_sha256"]:
        raise ValueError("Experience context does not match this round and active deployment")
    if sha256(inside(state["config_path"], root)) != context["config_sha256"]:
        raise ValueError("Configuration changed since the round")
    observations = candidate.get("observations")
    if not isinstance(observations, list) or not observations:
        raise ValueError("Provide verified observations, never placeholder defaults")
    prepared = []
    for item in observations:
        kind = item.get("kind")
        if kind not in CHECKS:
            raise ValueError("Unknown experience kind")
        review = item.get("verification")
        if not isinstance(review, dict) or review.get(CHECKS[kind]) is not True:
            raise ValueError(f"Missing verification for {kind}")
        if not isinstance(review.get("method"), str) or not review["method"].strip():
            raise ValueError("Verification requires a method")
        if kind == "placement" and state.get("physical_placement_verified") is not True:
            raise ValueError("Returned does not verify physical placement")
        for field in ("target", "measured"):
            if not isinstance(item.get(field), dict) or (field == "measured" and not item[field]):
                raise ValueError("Keep target and nonempty measured values separate")
        evidence = item.get("evidence")
        if not isinstance(evidence, list) or not evidence:
            raise ValueError("Experience requires evidence files from this round")
        digests = [{"path": str(state_path.relative_to(root)), "sha256": sha256(state_path)}]
        for entry in evidence:
            if not isinstance(entry, str) or Path(entry).is_absolute():
                raise ValueError("Evidence uses round-relative paths")
            path = inside(round_dir / entry, round_dir)
            if not path.is_file():
                raise ValueError("Experience evidence file is missing")
            digests.append({"path": str(path.relative_to(root)), "sha256": sha256(path)})
        prepared.append(
            {
                "id": uuid.uuid4().hex,
                "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
                "round_id": state["round_id"],
                "round_state_sha256": sha256(state_path),
                "context": context,
                "kind": kind,
                "target": item["target"],
                "measured": item["measured"],
                "verification": review,
                "evidence": digests,
            }
        )
    with store_lock(root) as directory:
        if active_context(root) != context:
            raise ValueError("Deployment changed during experience publication")
        data = load_store(root)
        if any(item["round_id"] == state["round_id"] for item in data["records"]):
            raise ValueError("This round already published experience; do not duplicate it")
        data["records"].extend(prepared)
        save(inside(directory / "records.json", root), data)
    return {"task_status": "experience_recorded", "records_added": len(prepared)}


def matching(root):
    root = workspace(root)
    context_path = inside(root / "experience/context.json", root)
    if not context_path.exists():
        return {"task_status": "experience_empty", "records": []}
    context = active_context(root)
    records = []
    for item in load_store(root)["records"]:
        if item["context"] != context:
            continue
        try:
            intact = all(
                sha256(inside(root / e["path"], root)) == e["sha256"] for e in item["evidence"]
            )
        except OSError:
            intact = False
        if intact:
            records.append(item)
    return {
        "task_status": "experience_available" if records else "experience_empty",
        "records": records,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("command", choices=("init", "context", "record", "show"))
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--round", type=Path)
    parser.add_argument("--input", type=Path)
    parser.add_argument("--robot-id")
    parser.add_argument("--scene-id")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--calibration", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "init":
            result = initialize(args.workspace)
        elif args.command == "context":
            if not all((args.config, args.calibration, args.robot_id, args.scene_id)):
                raise ValueError("context requires config, calibration, robot-id and scene-id")
            result = make_context(
                args.workspace, args.robot_id, args.scene_id, args.config, args.calibration
            )
        elif args.command == "record":
            if not args.round or not args.input:
                raise ValueError("record requires --round and --input")
            result = record(args.workspace, args.round, args.input)
        else:
            result = matching(args.workspace)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
        print(json.dumps({"task_status": "failed", "error": str(exc)}, ensure_ascii=False))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
