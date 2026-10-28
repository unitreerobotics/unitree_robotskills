#!/usr/bin/env python3
"""Restore captured arms, waist, and left gripper; hold the live right gripper."""

import argparse
import fcntl
import json
import signal
import sys
import time
from pathlib import Path

import numpy as np

from .arm_pose import execute_plan, validate_execution_config
from .common import (
    controller_lock_path,
    load_config,
    positive,
    reset_state_path,
    runtime_dir,
    vector,
)
from .dds_robot import DDSRobot
from .kinematics import Kinematics, matrix_pose


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
    tmp.replace(path)


def build_plan(saved, current, waist, kin, config, duration, current_grippers=None):
    """Direct reset restores both arms; legacy session callers restore right only."""
    if saved["joint_names"] != kin.names:
        raise ValueError("Saved joint names/order do not match this robot model")
    recorded = vector(saved["joint_positions"], 14, "saved joints")
    seed = kin.check_joints(current).copy()
    target = seed.copy()
    target[7:] = recorded[7:]
    if current_grippers is not None:
        target[:7] = recorded[:7]
    kin.check_joints(target)
    kin.set_waist(waist)
    fk = kin.fk(target)
    sides = ("left", "right") if current_grippers is not None else ("right",)
    poses = {side: matrix_pose(fk[side]) for side in sides}
    grippers = {}
    if current_grippers is not None:
        left = float(saved["grippers"]["left"])
        right = float(current_grippers["right"])
        for side, value in (("left", left), ("right", right)):
            limits = config["grippers"][side]
            if not np.isfinite(value) or not limits["min"] <= value <= limits["max"]:
                raise ValueError(f"{side} gripper target outside configured limits: {value}")
            grippers[side] = value
        current_left = float(current_grippers["left"])
        if not np.isfinite(current_left):
            raise ValueError("Nonfinite live left gripper position")
    duration = max(
        positive(duration, "duration"),
        1.5
        * float(np.max(np.abs(target - seed)))
        / positive(config["max_joint_speed"], "max_joint_speed"),
    )
    if grippers:
        duration = max(
            duration,
            abs(grippers["left"] - current_left)
            / positive(config["gripper_speed"], "gripper_speed")
            + 1.0,
        )
    return {
        "seed": seed.tolist(),
        "locked_waist_q": kin.waist.tolist(),
        "steps": [
            {
                "joint_positions": target.tolist(),
                "duration": duration,
                "targets": poses,
                "grippers": grippers,
            }
        ],
        "path": "Direct cubic interpolation in joint space; no collision avoidance",
        "gripper_policy": (
            "Left: saved target; right: live opening latched at start"
            if current_grippers is not None
            else "No gripper commands"
        ),
        "recorded_tcp": saved.get("poses", {}).get("right"),
        "recorded_poses": saved.get("poses"),
        "recorded_waist_q": saved.get("waist_q"),
    }


def check_publishers(robot, config, seconds=2.0):
    """Discover other command writers before creating any local writer."""
    from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication

    reader = BuiltinDataReader(robot._participant, BuiltinTopicDcpsPublication)
    topics = {
        config["topic_low_command"],
        config["grippers"]["left"]["command_topic"],
        config["grippers"]["right"]["command_topic"],
    }
    found = {}
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        for sample in reader.take(100):
            if getattr(sample, "topic_name", None) in topics:
                found[str(sample.key)] = sample.topic_name
        robot.check_motor_health()
        robot.arm_q()
        robot.gripper_q("right")
        time.sleep(0.02)
    if found:
        raise RuntimeError(f"Other command publishers detected: {found}")
    return {"discovery_seconds": seconds, "external_writers": found}


def feedback(robot, kin):
    q, waist = robot.arm_q(), robot.waist_q()
    kin.set_waist(waist)
    poses = {side: matrix_pose(matrix) for side, matrix in kin.fk(q).items()}
    return {
        "at_unix": time.time(),
        "joint_positions": q.tolist(),
        "waist_q": waist.tolist(),
        "poses": poses,
        "right_pose": poses["right"],
        "grippers": {side: robot.gripper_q(side) for side in ("left", "right")},
        "motor_health": robot.motor_health(),
    }


def run(args):
    full_config = load_config(args.config)
    reference = (
        Path(args.state).expanduser().resolve() if args.state else reset_state_path(full_config)
    )
    saved = json.loads(reference.read_text())
    output = Path(
        args.output or runtime_dir(full_config) / f"reset_right_{time.time_ns()}.json"
    ).resolve()
    if output == reference:
        raise ValueError("Output must not overwrite the saved reference")
    config = full_config["robot"]
    if args.interface:
        config["network_interface"] = args.interface
    validate_execution_config(config)
    positive(args.duration, "duration")
    if args.hold_seconds is not None and (
        not np.isfinite(args.hold_seconds) or args.hold_seconds < 0
    ):
        raise ValueError("hold-seconds must be finite and nonnegative")
    waist = vector(saved["waist_q"], 3, "recorded waist yaw/roll/pitch")
    waist_source = str(reference)
    if getattr(args, "waist_state", None):
        waist_path = Path(args.waist_state).resolve()
        waist_state = json.loads(waist_path.read_text())
        waist = vector(waist_state["waist_q"], 3, "task startup waist")
        waist_source = str(waist_path)
    kin = Kinematics(config["ik_position_tolerance"], config["ik_rotation_tolerance"])
    robot, lock = None, None
    result = {
        "reference": str(reference),
        "started_at_unix": time.time(),
        "waist_reference_source": waist_source,
        "execute_requested": args.execute,
        "status": "preflight",
        "publishing": False,
    }
    exit_code = 0
    try:
        if args.execute:
            lock_path = controller_lock_path(full_config)
            lock_path.parent.mkdir(parents=True, exist_ok=True)
            lock = lock_path.open("a+")
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        robot = DDSRobot(config)
        robot.connect(("left", "right"))
        robot.check_motor_health()
        if args.execute:
            result["publisher_check"] = check_publishers(robot, config)
        before = feedback(robot, kin)
        result["before"] = before
        # Use the same reference for target FK and the actual waist lock.
        plan = build_plan(
            saved, before["joint_positions"], waist, kin, config, args.duration, before["grippers"]
        )
        result["plan"] = plan
        result["status"] = "planned"
        write_json(output, result)
        print(
            json.dumps(
                {
                    "output": str(output),
                    "reset_target": plan["steps"][0],
                    "locked_waist_q": plan["locked_waist_q"],
                    "waist_reference_source": waist_source,
                    "execute": args.execute,
                },
                indent=2,
            ),
            flush=True,
        )
        if not args.execute:
            return 0
        robot.lock_waist(plan["locked_waist_q"])

        def on_pause(event):
            result.update(
                status="paused_holding", pause=event, publishing=True, after=feedback(robot, kin)
            )
            write_json(output, result)

        result["status"] = "executing"
        write_json(output, result)
        # Return from the shared executor without closing DDS, so the reached
        # evidence is saved immediately, before entering the final hold loop.
        execution = execute_plan(
            robot,
            kin,
            plan["steps"],
            np.asarray(plan["seed"]),
            config,
            hold_seconds=0,
            on_pause=on_pause,
            initial_grippers=plan["steps"][0]["grippers"],
        )
        result.update(
            status="completed_holding",
            execution=execution,
            publishing=True,
            after=feedback(robot, kin),
        )
        write_json(output, result)
        print(f"Reset reached; holding. Result: {output}. Ctrl+C stops publishing.", flush=True)
        deadline = None if args.hold_seconds is None else time.monotonic() + args.hold_seconds
        goal = np.asarray(plan["steps"][0]["joint_positions"])
        while deadline is None or time.monotonic() < deadline:
            tick = time.monotonic()
            robot.arm_q()
            kin.set_waist(robot.waist_q())
            robot.send(goal, kin.tau(goal), plan["steps"][0]["grippers"])
            time.sleep(max(0, 1 / config["rate_hz"] - (time.monotonic() - tick)))
        result["status"] = "completed"
    except KeyboardInterrupt:
        result["status_before_stop"] = result["status"]
        result["status"] = "stopped_by_user"
        exit_code = 130
    except Exception as exc:
        result.update(status="failed", error=str(exc))
        print(f"Reset failed: {exc}", file=sys.stderr, flush=True)
        exit_code = 1
    finally:
        if robot is not None:
            try:
                result["final_feedback"] = feedback(robot, kin)
            except Exception as exc:
                result["final_feedback"] = {"unavailable": str(exc)}
            robot.close()
        if lock is not None:
            lock.close()
        result.update(publishing=False, finished_at_unix=time.time())
        write_json(output, result)
    return exit_code


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--state", help="Captured reference JSON; defaults to runtime.reset_state from config"
    )
    parser.add_argument("--config")
    parser.add_argument(
        "--waist-state", help="Override the saved waist with waist_q from this task state JSON"
    )
    parser.add_argument("--interface")
    parser.add_argument("--output", help="Result JSON; defaults to a new timestamped artifact")
    parser.add_argument(
        "--duration", type=float, default=4.0, help="Minimum motion duration in seconds"
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Restore saved arms, waist and left gripper; hold live right gripper",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        help="Stop publishing after this successful hold; default holds until interrupted",
    )
    args = parser.parse_args(argv)

    def stop(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        return run(args)
    except Exception as exc:
        print(f"Reset preflight failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
