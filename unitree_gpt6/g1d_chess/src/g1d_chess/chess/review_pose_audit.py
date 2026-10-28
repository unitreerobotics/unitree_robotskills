#!/usr/bin/env python3
"""Post-execution FK review from archived feedback. Never connects to hardware."""
import argparse
import contextlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from g1d_chess.paths import ROOT
KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


def pose_matrix(pose):
    values = np.array([pose[k] for k in KEYS], dtype=float)
    if not np.isfinite(values).all():
        raise ValueError("non-finite target pose")
    result = np.eye(4)
    result[:3, 3] = values[:3]
    result[:3, :3] = Rotation.from_euler("xyz", values[3:]).as_matrix()
    return result


def pose_json(T):
    return dict(zip(KEYS, map(float, [*T[:3, 3],
                    *Rotation.from_matrix(T[:3, :3]).as_euler("xyz")])))


def error(actual, reference):
    delta = actual[:3, 3] - reference[:3, 3]
    angle = Rotation.from_matrix(actual[:3, :3] @ reference[:3, :3].T).magnitude()
    return {"delta_xyz_mm": (delta * 1000).tolist(),
            "position_norm_mm": float(np.linalg.norm(delta) * 1000),
            "rotation_error_rad": float(angle), "rotation_error_deg": float(np.degrees(angle))}


def actor_fk(model, q, arm):
    q = np.asarray(q, dtype=float)
    if q.shape != (14,) or not np.isfinite(q).all():
        raise ValueError("invalid archived joint vector")
    if arm == "left":
        q = np.concatenate((q[7:], q[:7]))
        index = 0
    elif arm == "right":
        index = 1
    else:
        raise ValueError("invalid physical actor")
    return model.fk(q)[index].copy()


def review(result, model):
    if not result.get("token") or result.get("outcome") in (None, "not_started"):
        raise ValueError("review requires a finalized execution result, not a prepared plan")
    audit = result.get("pose_audit")
    if not isinstance(audit, dict):
        raise ValueError("this result predates pose snapshot recording")
    rows = []
    seen = set()
    for snapshot in audit["records"]:
        event = snapshot["moment"]
        if (snapshot.get("schema_version") != 1 or snapshot["token"] != result["token"]
                or event not in ("grasp_success", "release_start") or event in seen
                or snapshot["joint_order"] != "observer_7_then_actor_7"):
            raise ValueError("invalid, duplicate or mismatched snapshot")
        seen.add(event)
        base = np.asarray(snapshot["fk_base"], dtype=float)
        if base.shape != (4, 4) or not np.isfinite(base).all():
            raise ValueError("invalid archived FK base")
        model.base = base.copy()
        arm = snapshot["physical_actor"]
        actual = actor_fk(model, snapshot["arm_q"], arm)
        comparisons = {name: error(actual, pose_matrix(snapshot[key]))
                       if snapshot.get(key) is not None else None
                       for name, key in (("vs_original_plan", "original_target"),
                                         ("vs_effective_target", "effective_target"))}
        commands = snapshot.get("commands")
        command = actor_fk(model, commands["arm_target"], arm) if commands else None
        comparisons["vs_command_fk"] = error(actual, command) if command is not None else None
        rows.append({"moment": event, "physical_actor": arm,
                     "event_stage": snapshot.get("event_stage", "before_low_open_command" if event == "release_start"
                                                  else "source_contact_success"),
                     "placement_target": snapshot.get("placement_target"),
                     "approval_reason": snapshot.get("approval_reason"),
                     "grasp_attempt_index": snapshot["grasp_attempt_index"],
                     "feedback_tick": snapshot["feedback_tick"],
                     "feedback_age_s": snapshot["feedback_age_s"],
                     "feedback_read_monotonic_s": snapshot.get("feedback_read_monotonic_s"),
                     "event_monotonic_s": snapshot["event_monotonic_s"],
                     "actual_fk": pose_json(actual),
                     "command_fk": pose_json(command) if command is not None else None,
                     "last_sent_fk": pose_json(actor_fk(model, commands["arm_sent"], arm)) if commands else None,
                     "original_target": snapshot.get("original_target"),
                     "effective_target": snapshot.get("effective_target"),
                     "errors": comparisons})
    return {"schema_version": 1, "token": result["token"], "outcome": result["outcome"],
            "records": rows, "missing_events": sorted({"grasp_success", "release_start"} - seen),
            "capture_errors": audit.get("errors", []),
            "notes": ["FK uses measured joints and the local model; it is not a visual piece position.",
                      "Use the same robot model/dependencies as execution.",
                      "vs_command_fk uses arm_target; last_sent_fk also exposes the last publisher command.",
                      "Missing command snapshots remain null; no commanded pose substitutes for feedback."]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", required=True, type=Path,
                        help="final archived TOKEN_result.json (after plan execution)")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = json.loads(args.result.read_text())
    # Load the offline kinematics only. Do not instantiate RobotService/InternalDDS.
    with contextlib.redirect_stdout(sys.stderr):
        from g1d_chess.robot.motion_harness import MotionHarness
        report = review(result, MotionHarness())
    report['result_path'] = str(args.result.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"report": str(args.output), "records": len(report["records"]),
                      "missing_events": report["missing_events"]}))


if __name__ == "__main__":
    main()
