#!/usr/bin/env python3
"""G1 xyz+rpy IK, read-only state and explicit DDS motion with Dex1 control."""

import argparse
import fcntl
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .common import controller_lock_path, load_config, positive, runtime_dir, vector
from .dds_robot import DDSRobot
from .kinematics import FIELDS, SIDES, Kinematics, matrix_pose, pose_matrix


def gripper_value(value, config, side):
    cfg = config["grippers"][side]
    if isinstance(value, str):
        if value not in ("open", "close"):
            raise ValueError("gripper must be open, close, or a raw Dex1 position")
        value = cfg[value]
    value = float(value)
    if not np.isfinite(value) or not cfg["min"] <= value <= cfg["max"]:
        raise ValueError(f"{side} gripper position outside configured limits")
    return value


def parse_plan(data, config, duration):
    if not isinstance(data, dict) or not data or set(data) - {"left_arm", "right_arm"}:
        raise ValueError("Plan must contain left_arm and/or right_arm waypoint arrays")
    points = {side: data.get(f"{side}_arm", []) for side in SIDES}
    for side, sequence in points.items():
        if not isinstance(sequence, list):
            raise ValueError(f"{side}_arm must be an array")
        for point in sequence:
            if not isinstance(point, dict) or set(point) - set(FIELDS) - {"gripper", "duration"}:
                raise ValueError("Waypoint requires xyz+rpy, optional gripper and duration")
            pose_matrix(point)
            positive(point.get("duration", duration), "waypoint duration")
            if "gripper" in point:
                gripper_value(point["gripper"], config, side)
    count = max(map(len, points.values()))
    if not count:
        raise ValueError("Plan contains no waypoints")
    steps = []
    for i in range(count):
        targets, grippers, durations = {}, {}, []
        for side in SIDES:
            if i >= len(points[side]):
                continue
            point = points[side][i]
            targets[side] = [point[key] for key in FIELDS]
            durations.append(positive(point.get("duration", duration), "duration"))
            if "gripper" in point:
                grippers[side] = gripper_value(point["gripper"], config, side)
        steps.append({"targets": targets, "grippers": grippers, "duration": max(durations)})
    return steps


def plan_motion(kin, steps, seed, config):
    q = kin.check_joints(seed)
    speed = positive(config["max_joint_speed"], "max_joint_speed")
    result = []
    for step in steps:
        solved = kin.solve(step["targets"], q)
        goal = np.asarray(solved["joint_positions"])
        # Cubic smoothstep has max ds/dt = 1.5 / duration.
        duration = max(step["duration"], 1.5 * float(np.max(np.abs(goal - q))) / speed)
        result.append(
            {
                **solved,
                "duration": duration,
                "grippers": step["grippers"],
                "targets": step["targets"],
            }
        )
        q = goal
    return result


def validate_execution_config(config):
    for key in (
        "rate_hz",
        "max_joint_speed",
        "joint_tolerance",
        "settle_timeout",
        "gripper_tolerance",
        "gripper_speed",
        "state_timeout",
        "connect_timeout",
    ):
        positive(config[key], key)
    if config["rate_hz"] > 500:
        raise ValueError("rate_hz must not exceed 500")
    for key in ("arrival_position_tolerance", "arrival_rotation_tolerance"):
        if key in config:
            positive(config[key], key)


def arrival_report(kin, step, measured_q, config):
    """Evaluate every arrival gate from one joint sample, including the held arm."""
    measured_q = vector(measured_q, 14, "measured arm q")
    target = vector(step["joint_positions"], 14, "target arm q")
    errors = np.abs(measured_q - target)
    poses = kin.fk(measured_q)
    limits = {
        "joint_rad": config["joint_tolerance"],
        "position_m": config.get("arrival_position_tolerance", kin.position_tolerance),
        "rotation_rad": config.get("arrival_rotation_tolerance", kin.rotation_tolerance),
    }
    arms = {}
    for i, side in enumerate(SIDES):
        sl = slice(i * 7, (i + 1) * 7)
        worst = i * 7 + int(np.argmax(errors[sl]))
        entry = {
            "commanded": side in step["targets"],
            "joint_error_rad": float(errors[worst]),
            "worst_joint": kin.names[worst],
            "joint_ok": bool(errors[worst] <= limits["joint_rad"]),
            "actual_pose": matrix_pose(poses[side]),
        }
        if side in step["targets"]:
            desired = pose_matrix(step["targets"][side])
            entry["target_pose"] = matrix_pose(desired)
            entry["position_error_m"] = float(np.linalg.norm(poses[side][:3, 3] - desired[:3, 3]))
            entry["rotation_error_rad"] = float(
                Rotation.from_matrix(poses[side][:3, :3] @ desired[:3, :3].T).magnitude()
            )
            entry["position_ok"] = entry["position_error_m"] <= limits["position_m"]
            entry["rotation_ok"] = entry["rotation_error_rad"] <= limits["rotation_rad"]
        arms[side] = entry
    reached = all(
        a["joint_ok"] and a.get("position_ok", True) and a.get("rotation_ok", True)
        for a in arms.values()
    )
    return {
        "reached": reached,
        "limits": limits,
        "arms": arms,
        "measured_joint_positions": measured_q.tolist(),
        "target_joint_positions": target.tolist(),
    }


def arrival_summary(report):
    limits = report["limits"]
    parts = []
    for side, arm in report["arms"].items():
        role = "target" if arm["commanded"] else "hold"
        text = (
            f"{side}({role}): joint={arm['joint_error_rad']:.4f}/{limits['joint_rad']:.4f} rad"
            f" [{arm['worst_joint']}]"
        )
        if arm["commanded"]:
            text += (
                f", position={arm['position_error_m'] * 1000:.1f}/"
                f"{limits['position_m'] * 1000:.1f} mm"
                f", rotation={arm['rotation_error_rad']:.4f}/{limits['rotation_rad']:.4f} rad"
            )
        parts.append(text)
    return "; ".join(parts)


def execute_plan(
    robot, kin, plan, seed, config, hold_seconds=None, on_pause=None, initial_grippers=None
):
    validate_execution_config(config)
    if hold_seconds is not None and (not np.isfinite(hold_seconds) or hold_seconds < 0):
        raise ValueError("hold_seconds must be finite and nonnegative")
    sides = sorted({side for step in plan for side in step["grippers"]})
    robot.connect(sides)
    locked_waist = robot.lock_waist()
    planned_waist = kin.waist.copy()
    if np.max(np.abs(robot.arm_q() - seed)) > config["joint_tolerance"]:
        raise ValueError("Robot moved since planning; read state and plan again")
    robot.begin_motion(sides)
    q = np.asarray(seed).copy()
    held_grippers = {side: robot.gripper_q(side) for side in sides}
    if initial_grippers is not None:
        if set(initial_grippers) != set(sides):
            raise ValueError("Initial gripper targets must match the planned grippers")
        held_grippers.update(initial_grippers)
    period = 1 / config["rate_hz"]
    completed = []

    def send(command):
        robot.arm_q()  # Still require fresh, finite feedback.
        kin.set_waist(robot.waist_q())
        robot.send(command, kin.tau(command), held_grippers)

    def pause_and_hold(reason, report, command):
        event = {
            "status": "paused_holding",
            "reason": reason,
            "arrival": report,
            "commanded_q": command.tolist(),
            "locked_waist_q": locked_waist.tolist(),
            "held_grippers": dict(held_grippers),
        }
        print(json.dumps(event, allow_nan=False), flush=True)
        if on_pause is not None:
            try:
                on_pause(event)
            except Exception as exc:
                print(f"Could not save pause diagnostics: {exc}", file=sys.stderr, flush=True)
        print(
            "Plan paused; continuing position hold. Ctrl+C stops publishing. "
            "Use arm_session for online correction.",
            file=sys.stderr,
            flush=True,
        )
        # A task timeout must never fall through to main()'s publisher cleanup.
        # Finite hold_seconds applies only to successful completion.
        while True:
            tick = time.monotonic()
            send(command)
            time.sleep(max(0, period - (time.monotonic() - tick)))

    for step_index, step in enumerate(plan):
        target = np.asarray(step["joint_positions"])
        start = q.copy()
        begin = time.monotonic()
        while True:
            tick = time.monotonic()
            fraction = min(1.0, (tick - begin) / step["duration"])
            q = start + (target - start) * (3 * fraction**2 - 2 * fraction**3)
            send(q)
            if fraction >= 1:
                break
            time.sleep(max(0, period - (time.monotonic() - tick)))
        deadline = time.monotonic() + config["settle_timeout"]
        settle_start = time.monotonic()
        next_log = settle_start
        history = []
        while True:
            tick = time.monotonic()
            send(target)
            report = arrival_report(kin, step, robot.arm_q(), config)
            if report["reached"]:
                break
            if tick >= next_log:
                print(
                    f"Step {step_index + 1} settling: {arrival_summary(report)}",
                    file=sys.stderr,
                    flush=True,
                )
                history.append({"elapsed_s": tick - settle_start, "arms": report["arms"]})
                next_log = tick + 1.0
            if time.monotonic() >= deadline:
                report.update(
                    step=step_index + 1,
                    settle_seconds=time.monotonic() - settle_start,
                    planned_waist_q=planned_waist.tolist(),
                    measured_waist_q=robot.waist_q().tolist(),
                    settle_history=history,
                    gripper_change_issued=False,
                )
                pause_and_hold("arrival_timeout", report, target)
            time.sleep(max(0, period - (time.monotonic() - tick)))
        held_grippers.update(step["grippers"])
        deadline = time.monotonic() + config["settle_timeout"]
        while step["grippers"]:
            tick = time.monotonic()
            send(target)
            if all(
                abs(robot.gripper_q(side) - value) <= config["gripper_tolerance"]
                for side, value in step["grippers"].items()
            ):
                break
            if time.monotonic() >= deadline:
                held_grippers.update({side: robot.gripper_q(side) for side in step["grippers"]})
                pause_and_hold("gripper_timeout_contact_unconfirmed", report, target)
            time.sleep(max(0, period - (time.monotonic() - tick)))
        q = target
        completed.append(
            {
                "step": step_index + 1,
                "arrival": report,
                "grippers": {side: robot.gripper_q(side) for side in sides},
            }
        )
    print(
        "Waypoints reached. Holding final pose; Ctrl+C stops publishing.",
        file=sys.stderr,
        flush=True,
    )
    deadline = None if hold_seconds is None else time.monotonic() + hold_seconds
    hold_start = time.monotonic()
    next_sample = hold_start
    hold_samples = []
    while deadline is None or time.monotonic() < deadline:
        tick = time.monotonic()
        send(q)
        if tick >= next_sample:
            hold_samples.append(
                {
                    "elapsed_s": tick - hold_start,
                    **arrival_report(kin, plan[-1], robot.arm_q(), config),
                }
            )
            hold_samples = hold_samples[-100:]
            next_sample = tick + 0.1
        time.sleep(max(0, period - (time.monotonic() - tick)))
    return {
        "status": "completed",
        "steps": completed,
        "hold_samples": hold_samples,
        "final": arrival_report(kin, plan[-1], robot.arm_q(), config),
        "locked_waist_q": locked_waist.tolist(),
        "measured_waist_q": robot.waist_q().tolist(),
        "grippers": {side: robot.gripper_q(side) for side in sides},
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config")
    parser.add_argument("--interface", help="DDS network interface override")
    parser.add_argument("--output", help="Also write JSON result to this path")
    sub = parser.add_subparsers(dest="command", required=True)
    state = sub.add_parser("state", help="Read real arm joints and FK; no commands")
    state.add_argument("--grippers", action="store_true")
    for command in ("ik", "move"):
        cmd = sub.add_parser(command)
        cmd.add_argument(
            "--left", nargs=6, type=float, metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW")
        )
        cmd.add_argument(
            "--right", nargs=6, type=float, metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW")
        )
        cmd.add_argument("--left-gripper")
        cmd.add_argument("--right-gripper")
        cmd.add_argument("--plan", help="JSON waypoint file")
        cmd.add_argument("--duration", type=float, default=3.0)
        if command == "ik":
            cmd.add_argument(
                "--seed", nargs=14, type=float, help="Offline seed; omit to read real DDS state"
            )
            cmd.add_argument(
                "--waist",
                nargs=3,
                type=float,
                default=[0, 0, 0],
                help="Offline waist yaw/roll/pitch with --seed; default zero",
            )
        else:
            cmd.add_argument(
                "--execute", action="store_true", help="Publish DDS commands; otherwise plan only"
            )
            cmd.add_argument(
                "--hold-seconds", type=float, help="Hold after motion; default holds until Ctrl+C"
            )
            cmd.add_argument(
                "--position-tolerance", type=float, help="Measured arrival tolerance in meters"
            )
            cmd.add_argument(
                "--rotation-tolerance", type=float, help="Measured arrival tolerance in radians"
            )
            cmd.add_argument(
                "--joint-tolerance", type=float, help="Joint arrival tolerance in radians"
            )
            cmd.add_argument(
                "--settle-timeout", type=float, help="Seconds to wait for arm/gripper arrival"
            )
    args = parser.parse_args(argv)
    full_config = load_config(args.config)
    config = full_config["robot"]
    if args.interface:
        config["network_interface"] = args.interface
    robot = None
    lock = None
    try:
        if getattr(args, "execute", False):
            lock_path = controller_lock_path(full_config)
            lock_path.parent.mkdir(parents=True, exist_ok=True)
            lock = lock_path.open("a+")
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if config["backend"] != "dds":
            raise ValueError("This tool supports the pure upper-body DDS backend")
        if args.command == "move":
            for arg, key in (
                ("position_tolerance", "arrival_position_tolerance"),
                ("rotation_tolerance", "arrival_rotation_tolerance"),
                ("joint_tolerance", "joint_tolerance"),
                ("settle_timeout", "settle_timeout"),
            ):
                value = getattr(args, arg)
                if value is not None:
                    config[key] = positive(value, arg)
            validate_execution_config(config)
        steps = None
        if args.command != "state":
            if args.plan:
                if any(
                    v is not None
                    for v in (args.left, args.right, args.left_gripper, args.right_gripper)
                ):
                    raise ValueError("--plan cannot be combined with individual targets")
                with open(args.plan, encoding="utf-8") as stream:
                    data = json.load(stream)
            else:
                data = {}
                for side in SIDES:
                    pose, grip = getattr(args, side), getattr(args, f"{side}_gripper")
                    if grip is not None and pose is None:
                        raise ValueError(f"--{side}-gripper requires --{side} pose")
                    if pose is not None:
                        point = dict(zip(FIELDS, pose, strict=True))
                        if grip is not None:
                            point["gripper"] = grip if grip in ("open", "close") else float(grip)
                        data[f"{side}_arm"] = [point]
            steps = parse_plan(data, config, positive(args.duration, "duration"))
        kin = Kinematics(config["ik_position_tolerance"], config["ik_rotation_tolerance"])
        if args.command == "ik" and args.seed is not None:
            seed = vector(args.seed, 14, "seed")
            kin.set_waist(args.waist)
        else:
            robot = DDSRobot(config)
            robot.connect(SIDES if args.command == "state" and args.grippers else ())
            robot.lock_waist()
            seed = robot.arm_q()
            kin.set_waist(robot.waist_q())
        if args.command == "state":
            result = {
                "joint_names": kin.names,
                "joint_positions": seed.tolist(),
                "poses": {s: matrix_pose(m) for s, m in kin.fk(seed).items()},
                "waist_q": robot.waist_q().tolist(),
                "frame": "G1 URDF pelvis, measured waist; L_ee/R_ee",
            }
            if args.grippers:
                result["grippers"] = {s: robot.gripper_q(s) for s in SIDES}
        else:
            plan = plan_motion(kin, steps, seed, config)
            result = {
                "mode": "execute" if getattr(args, "execute", False) else "plan_only",
                "seed": seed.tolist(),
                "waist_q": kin.waist.tolist(),
                "frame": "G1 URDF pelvis; L_ee/R_ee",
                "steps": plan,
            }
        encoded = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
        print(encoded, flush=True)
        if args.output:
            Path(args.output).write_text(encoded + "\n", encoding="utf-8")
        if getattr(args, "execute", False):

            def save_pause(event):
                result["execution"] = event
                path = runtime_dir(full_config) / f"{event['reason']}_{time.time_ns()}.json"
                path.parent.mkdir(parents=True, exist_ok=True)
                encoded_pause = json.dumps(result, indent=2, allow_nan=False) + "\n"
                path.write_text(encoded_pause, encoding="utf-8")
                print(f"Pause diagnostics: {path}", file=sys.stderr, flush=True)
                if args.output:
                    Path(args.output).write_text(encoded_pause, encoding="utf-8")

            result["execution"] = execute_plan(
                robot, kin, plan, seed, config, args.hold_seconds, on_pause=save_pause
            )
            print(
                "Final feedback: " + arrival_summary(result["execution"]["final"]),
                file=sys.stderr,
                flush=True,
            )
            if args.output:
                Path(args.output).write_text(
                    json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8"
                )
    except KeyboardInterrupt:
        print("Stopped publishing; no return-to-home command sent.", file=sys.stderr)
    except (ValueError, RuntimeError, TimeoutError, KeyError, OSError) as exc:
        parser.exit(1, f"Error: {exc}\n")
    finally:
        if robot is not None:
            robot.close()
        if lock is not None:
            lock.close()


if __name__ == "__main__":
    main()
