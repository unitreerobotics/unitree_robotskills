"""One persistent gomoku owner and offline request/plan client."""

import argparse
import hashlib
import json
import signal
import time
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

from . import arm_session
from .arm_pose import plan_motion
from .arm_session import validate_plan, write_json
from .common import configure, load_config, reset_state_path, runtime_dir, scene_geometry
from .continuous_gomoku_session import ContinuousSession
from .dds_robot import DDSRobot
from .kinematics import Kinematics, matrix_pose, pose_matrix
from .reset_right import build_plan


class StartupRobot(DDSRobot):
    def begin_motion(self, grippers=()):
        self.startup_waist = self.waist_q().copy()
        self.startup_time = time.monotonic()
        super().begin_motion(grippers)

    def _apply_waist_lock(self, message):
        super()._apply_waist_lock(message)
        u = min(1.0, max(0.0, (time.monotonic() - self.startup_time) / 4.0))
        waist = self.startup_waist + (self._waist_reference - self.startup_waist) * (
            3 * u * u - 2 * u * u * u
        )
        for index, angle in zip(self.WAIST_INDICES, waist, strict=True):
            message.motor_cmd[index].q = float(angle)


def read(root, allow_observing=False):
    s = json.loads((root / "session/status.json").read_text())
    configured_path = load_config()["_config_path"]
    if s.get("config_path") != configured_path:
        raise ValueError("Client and controller must use the same local configuration")
    if time.time() - s["feedback"].get("at_unix", 0) > 2:
        raise RuntimeError("Stale controller feedback")
    ready = s["publishing"] and s["phase"] == "holding"
    starting = allow_observing and not s["publishing"] and s["phase"] == "observing"
    if s["fault"] or not (ready or starting):
        raise RuntimeError(f"Not healthy holding: {s['phase']} {s['fault']}")
    return s


def summary(s):
    return {k: s.get(k) for k in ("pid", "phase", "publishing", "fault", "stage")} | {
        "actual_tcp": s["feedback"].get("poses", {}).get("right"),
        "waist": s["feedback"].get("waist_q"),
        "gripper": s["feedback"].get("grippers", {}).get("right"),
    }


def request(root, data):
    p = root / "session/inbox" / f"{time.time_ns()}.json"
    write_json(p, data)
    return p


def wait_request(root, path, name=None, seconds=90, return_directory=None):
    """Wait for the whole requested operation, including continuous return."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if path.with_suffix(".rejected").exists():
            raise RuntimeError(path.with_suffix(".error").read_text())
        s = json.loads((root / "session/status.json").read_text())
        if time.time() - s["feedback"].get("at_unix", 0) > 2:
            raise RuntimeError("Stale controller feedback; do not restart or replay")
        if s["fault"] or not s["publishing"]:
            raise RuntimeError(str(summary(s)))
        done = path.with_suffix(".done").exists()
        if return_directory is not None:
            returned = Path(return_directory) / "return_result.json"
            if done and returned.is_file() and s["phase"] == "holding" and s["stage"] is None:
                return s
        elif done and (name is None or s["stage"] == name and s["phase"] == "holding"):
            return s
        time.sleep(0.1)
    raise TimeoutError("Owner may still be active; do not restart or replay")


def model(s):
    cfg = load_config()["robot"]
    kin = Kinematics(cfg["ik_position_tolerance"], cfg["ik_rotation_tolerance"])
    kin.set_waist(s["initial_waist"])
    return cfg, kin


def numerical_plan(kin, steps, seed, cfg):
    """Bounded FK fallback for the vertical-gripper Euler singularity."""
    result = []
    q = seed.copy()
    lo = kin.model.lowerPositionLimit[7:] + 0.02
    hi = kin.model.upperPositionLimit[7:] - 0.02
    for step in steps:
        desired = pose_matrix(step["targets"]["right"])

        def residual(right, q=q, desired=desired):
            trial = q.copy()
            trial[7:] = right
            actual = kin.fk(trial)["right"]
            position = (actual[:3, 3] - desired[:3, 3]) * 1000
            rotation = Rotation.from_matrix(actual[:3, :3] @ desired[:3, :3].T).as_rotvec() * 1000
            return np.r_[position, rotation, (right - q[7:]) * 0.1]

        solved = least_squares(
            residual,
            np.clip(q[7:], lo + 1e-8, hi - 1e-8),
            bounds=(lo, hi),
            max_nfev=100,
            ftol=1e-10,
            xtol=1e-10,
            gtol=1e-10,
        )
        goal = q.copy()
        goal[7:] = solved.x
        kin.check_joints(goal)
        final = kin.fk(goal)["right"]
        pos_error = np.linalg.norm(final[:3, 3] - desired[:3, 3])
        rot_error = Rotation.from_matrix(final[:3, :3] @ desired[:3, :3].T).magnitude()
        if pos_error > cfg["ik_position_tolerance"] or rot_error > cfg["ik_rotation_tolerance"]:
            raise ValueError(f"Numerical IK misses target: {pos_error}m, {rot_error}rad")
        duration = max(step["duration"], 1.5 * np.max(np.abs(goal - q)) / cfg["max_joint_speed"])
        result.append(
            {
                "joint_positions": goal.tolist(),
                "duration": float(duration),
                "grippers": step["grippers"],
                "targets": step["targets"],
                "solver": "bounded_least_squares",
                "errors": {
                    "right": {"position_m": float(pos_error), "rotation_rad": float(rot_error)}
                },
            }
        )
        q = goal
    return result


def plan_move(root, s, target, duration, review):
    geometry = scene_geometry()
    cfg, kin = model(s)
    q = np.asarray(s["commanded_q"])
    a = kin.fk(q)["right"]
    b = pose_matrix(target)
    rotations = Slerp([0, 1], Rotation.from_matrix([a[:3, :3], b[:3, :3]]))
    distance = np.linalg.norm(b[:3, 3] - a[:3, 3])
    angle = Rotation.from_matrix(b[:3, :3] @ a[:3, :3].T).magnitude()
    count = max(1, int(np.ceil(max(distance / 0.018, angle / 0.16))))
    steps = []
    for u in np.linspace(1 / count, 1, count):
        m = np.eye(4)
        m[:3, 3] = a[:3, 3] * (1 - u) + b[:3, 3] * u
        m[:3, :3] = rotations(float(u)).as_matrix()
        steps.append(
            {
                "targets": {"right": matrix_pose(m)},
                "grippers": {},
                "duration": max(0.4, duration / count),
            }
        )
    try:
        solved = plan_motion(kin, steps, q, cfg)
    except ValueError as exc:
        if "IK did not converge" not in str(exc):
            raise
        solved = numerical_plan(kin, steps, q, cfg)
    lower, upper = kin.model.lowerPositionLimit, kin.model.upperPositionLimit

    def limit_issue(plan):
        for index, step in enumerate(plan, 1):
            joints = np.asarray(step["joint_positions"])
            margins = np.minimum(joints[7:] - lower[7:], upper[7:] - joints[7:])
            worst = int(np.argmin(margins)) + 7
            if margins[worst - 7] < 0.015:
                return (
                    f"Waypoint {index} {kin.names[worst]} has only "
                    f"{margins[worst - 7]:.4f} rad URDF margin"
                )
        return None

    issue = limit_issue(solved)
    if issue:
        solved = numerical_plan(kin, steps, q, cfg)
        issue = limit_issue(solved)
    if issue:
        raise ValueError(issue)
    plan = {"seed": q.tolist(), "locked_waist_q": s["initial_waist"], "steps": solved}
    xyz = []
    tips = []
    previous = q
    for step in solved:
        goal = np.asarray(step["joint_positions"])
        for u in np.linspace(0, 1, 15):
            sample = previous + (goal - previous) * (3 * u * u - 2 * u * u * u)
            m = kin.fk(sample)["right"]
            xyz.append(m[:3, 3].tolist())
            tips.append((m[:3, 3] + m[:3, :3] @ np.asarray(geometry["tool_tip_offset_m"])).tolist())
        previous = goal
    plan["clearance_review"] = {
        "passed": True,
        "scope": review,
        "evidence_directory": str(root),
        "joint_limits_checked": True,
        "sampled_tcp_min": np.min(xyz, axis=0).tolist(),
        "sampled_tcp_max": np.max(xyz, axis=0).tolist(),
        "tool_tip_offset_m": geometry["tool_tip_offset_m"],
        "scene_sha256": geometry["_sha256"],
        "sampled_tip_min": np.min(tips, axis=0).tolist(),
        "sampled_tip_max": np.max(tips, axis=0).tolist(),
        "limitation": "Current scene calibration and sampled FK; not complete collision detection.",
    }
    validate_plan(
        plan, kin, np.asarray(s["feedback"]["joint_positions"]), cfg, np.asarray(s["held_left_q"])
    )
    return plan


def reverse_executed_high_route(root, source_path, start_q, waist, held_left, kin, cfg):
    geometry = scene_geometry()
    source_path = source_path.resolve()
    if source_path.parent != root.resolve() or not source_path.name.endswith("_plan.json"):
        raise ValueError("High route must belong to the current round")
    name = source_path.name[: -len("_plan.json")]
    source = json.loads(source_path.read_text())
    result = json.loads((root / name / "result.json").read_text())
    if (
        result.get("outcome") != "motion_complete_unverified"
        or result.get("status", {}).get("stage") != name
    ):
        raise ValueError("High route has no completed execution record")
    if (
        not source.get("clearance_review", {}).get("passed")
        or not source.get("steps")
        or not np.allclose(source.get("locked_waist_q", []), waist, atol=1e-8, rtol=0)
    ):
        raise ValueError("High route review or waist does not match")
    review = source["clearance_review"]
    if review.get("scene_sha256") != geometry["_sha256"]:
        raise ValueError("High route scene calibration changed")
    if (
        review["sampled_tcp_min"][2] < geometry["high_route_min_tcp_z_m"]
        or review["sampled_tip_min"][2] < geometry["high_route_min_tip_z_m"]
    ):
        raise ValueError("Source route is not in the reviewed high corridor")
    points = [np.asarray(source["seed"], dtype=float)] + [
        np.asarray(step["joint_positions"], dtype=float) for step in source["steps"]
    ]
    validate_plan(source, kin, points[0], cfg, np.asarray(held_left))
    if not np.allclose(result["status"]["commanded_q"], points[-1], atol=1e-7, rtol=0):
        raise ValueError("Execution record does not finish at the high-route anchor")
    if np.max(np.abs(points[-1] - start_q)) > 0.03:
        raise ValueError("Return does not join high route at the same arm configuration")
    if any(step.get("grippers") for step in source["steps"]):
        raise ValueError("High route contains a gripper action")
    steps = []
    for index in range(len(source["steps"]) - 1, -1, -1):
        q = points[index]
        if not np.allclose(q[:7], held_left, atol=1e-8, rtol=0):
            raise ValueError("High route changes held left arm")
        steps.append(
            {
                "joint_positions": q.tolist(),
                "duration": float(source["steps"][index]["duration"]),
                "targets": {"right": matrix_pose(kin.fk(q)["right"])},
                "grippers": {},
            }
        )
    plan = {
        "seed": start_q.tolist(),
        "locked_waist_q": list(waist),
        "steps": steps,
        "clearance_review": {
            "passed": True,
            "scope": "Reverse of this round completed, reviewed high-clearance route",
            "source_review": source["clearance_review"],
        },
    }
    previous = start_q
    minimum = float("inf")
    tip_minimum = float("inf")
    for step in steps:
        goal = np.asarray(step["joint_positions"])
        for u in np.linspace(0, 1, 15):
            q = previous + (goal - previous) * (3 * u * u - 2 * u * u * u)
            pose = kin.fk(q)["right"]
            minimum = min(minimum, float(pose[2, 3]))
            tip_minimum = min(
                tip_minimum,
                float(pose[2, 3] + pose[2, :3] @ np.asarray(geometry["tool_tip_offset_m"])),
            )
        previous = goal
    if (
        minimum < review["sampled_tcp_min"][2] - 0.005
        or tip_minimum < review["sampled_tip_min"][2] - 0.005
    ):
        raise ValueError("Reversed route falls below reviewed high corridor")
    plan["clearance_review"]["sampled_tcp_min_z"] = minimum
    plan["clearance_review"]["sampled_tip_min_z"] = tip_minimum
    validate_plan(plan, kin, start_q, cfg, np.asarray(held_left))
    return plan


def submit(root, name, plan, op="stage", extra=None, owner_root=None, quiet=False):
    path = root / (name + "_plan.json")
    write_json(path, plan)
    target = plan["steps"][-1]["targets"]["right"]
    print(
        json.dumps(
            {
                "stage": name,
                "target_tcp": target,
                "duration": sum(x["duration"] for x in plan["steps"]),
            }
        ),
        flush=True,
    )
    req = {
        "op": op,
        "plan": str(path),
        "directory": str(root / name),
        "target": {"right_tcp": target},
    }
    req.update(extra or {})
    if op == "release_and_return":
        # Consume the human confirmation exactly once and bind it to these files.
        approval = root / "release_authorization.json"
        authorization = json.loads(approval.read_text())
        if authorization.get("approved") is not True:
            raise ValueError("This round needs explicit release approval")
        returned = Path(req["return_plan"])
        if approval.stat().st_mtime_ns < returned.stat().st_mtime_ns:
            raise ValueError("Release approval must follow the final return plan")
        consumed = root / (name + "_release_authorization.json")
        if consumed.exists():
            raise ValueError("This release stage already has an authorization record")
        approval.replace(consumed)
        authorization.update(
            directory=str((root / name).resolve()),
            release_plan_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            return_plan_sha256=hashlib.sha256(returned.read_bytes()).hexdigest(),
        )
        write_json(consumed, authorization)
        req["release_authorization"] = str(consumed.resolve())
    session_root = owner_root or root
    p = request(session_root, req)
    return_directory = root / name if op == "release_and_return" else None
    seconds = max(90, sum(step["duration"] for step in plan["steps"]) + 30)
    if return_directory is not None:
        returned = json.loads(Path(req["return_plan"]).read_text())
        seconds += sum(step["duration"] for step in returned["steps"])
    return wait_request(session_root, p, name, seconds, return_directory)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    p.add_argument("--config", help="Local hardware YAML configuration")
    p.add_argument(
        "--execute", action="store_true", help="Authorize owner publishing or a motion request"
    )
    p.add_argument("--root", required=True, type=Path)
    p.add_argument(
        "--owner-root",
        type=Path,
        help="Existing controller root when recording a new round separately",
    )
    p.add_argument(
        "op",
        choices=[
            "owner",
            "new_round",
            "status",
            "observe",
            "move",
            "grip",
            "reset",
            "prepare_return",
            "release",
            "restore_initial",
        ],
    )
    p.add_argument("name", nargs="?")
    p.add_argument("--pose", nargs=6, type=float)
    p.add_argument(
        "--via-pose",
        nargs=6,
        type=float,
        help="High-clearance exit waypoint after the vertical lift",
    )
    p.add_argument(
        "--outbound-plan",
        type=Path,
        help="Completed, reviewed high route from this round to reverse after the lift",
    )
    p.add_argument("--gripper", type=float)
    p.add_argument("--duration", type=float, default=4)
    p.add_argument("--review")
    p.add_argument("--waist-state", type=Path)
    p.add_argument("--initial-state", type=Path)
    p.add_argument(
        "--return-plan", type=Path, help="Reviewed return plan for release and continuous reset"
    )
    p.add_argument(
        "--quiet",
        action="store_true",
        help="Print target poses without artifact paths or reference details",
    )
    args = p.parse_args(argv)
    if args.config:
        configure(args.config)
    full_cfg = load_config()

    def local_root(path):
        return (path if path.is_absolute() else runtime_dir(full_cfg) / path).resolve()

    root = local_root(args.root)
    if (
        args.op in ("owner", "move", "grip", "reset", "release", "restore_initial")
        and not args.execute
    ):
        raise ValueError("Motion and owner publishing require --execute")
    if args.op in (
        "observe",
        "move",
        "grip",
        "reset",
        "prepare_return",
        "release",
        "restore_initial",
    ):
        if not args.name or args.name in (".", "..") or Path(args.name).name != args.name:
            raise ValueError("A single-component stage name is required")
    if not np.isfinite(args.duration) or args.duration <= 0:
        raise ValueError("Duration must be finite and positive")
    if args.op in ("move", "prepare_return") and (
        args.pose is None or not np.isfinite(args.pose).all()
    ):
        raise ValueError("A finite six-value target --pose is required")
    if args.outbound_plan and args.via_pose:
        raise ValueError("Choose one high-exit route")

    def report(data):
        if not args.quiet:
            print(json.dumps(data, indent=2))

    owner_root = local_root(args.owner_root) if args.owner_root else root
    if args.op == "owner":
        if owner_root != root:
            raise ValueError("Owner root must be its artifact root")
        arm_session.DDSRobot = StartupRobot
        s = ContinuousSession(
            root / "session",
            args.execute,
            waist_state=args.waist_state,
            arrival_mode="observe",
            config=full_cfg,
        )
        write_json(
            root / "runtime_parameters.json",
            {
                "config_path": full_cfg["_config_path"],
                "gripper_speed": s.cfg["gripper_speed"],
                "waist_reference": s.initial_waist.tolist(),
                "release_requires_user_confirmation": True,
            },
        )
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: setattr(s, "stopping", True))
        return s.run()
    s = read(owner_root, allow_observing=args.op == "reset")
    root.mkdir(parents=True, exist_ok=True)
    if args.op == "status":
        report(summary(s))
        return
    if args.op == "new_round":
        req = request(owner_root, {"op": "new_round"})
        report(summary(wait_request(owner_root, req)))
        return
    if args.op == "observe":
        req = request(owner_root, {"op": "observe", "directory": str(root / args.name)})
        report(summary(wait_request(owner_root, req)))
        return
    if args.op == "restore_initial":
        if not args.initial_state or not args.name:
            raise ValueError("Initial state and stage name required")
        req = request(
            owner_root,
            {
                "op": "restore_initial",
                "state": str(args.initial_state.resolve()),
                "directory": str(root / args.name),
            },
        )
        report(summary(wait_request(owner_root, req, args.name)))
        return
    if not args.review:
        raise ValueError("Current path/visual review required")
    if args.op in ("move", "prepare_return"):
        if args.op == "prepare_return":
            _, kin = model(s)
            held = kin.fk(np.asarray(s["commanded_q"]))["right"]
            lift = pose_matrix(args.pose)
            if (
                np.linalg.norm(lift[:2, 3] - held[:2, 3]) > 0.002
                or lift[2, 3] <= held[2, 3]
                or Rotation.from_matrix(lift[:3, :3] @ held[:3, :3].T).magnitude() > 0.01
            ):
                raise ValueError(
                    "Return must begin with a vertical lift at held XY and orientation"
                )
        plan = plan_move(root, s, args.pose, args.duration, args.review)
        if args.op == "prepare_return":
            cfg, kin = model(s)
            q = np.asarray(plan["steps"][-1]["joint_positions"])
            if args.outbound_plan:
                source = json.loads(args.outbound_plan.read_text())
                anchor = np.asarray(source["steps"][-1]["joint_positions"])
                anchor_pose = matrix_pose(kin.fk(anchor)["right"])
                lift_pose = matrix_pose(kin.fk(q)["right"])
                if (
                    abs(lift_pose[2] - anchor_pose[2]) > 0.08
                    or np.linalg.norm(np.asarray(lift_pose[:2]) - anchor_pose[:2]) > 0.03
                ):
                    raise ValueError("Lift is too far from the high-route anchor")
                virtual = {
                    **s,
                    "commanded_q": q.tolist(),
                    "feedback": {**s["feedback"], "joint_positions": q.tolist()},
                }
                connector = plan_move(
                    root,
                    virtual,
                    anchor_pose,
                    args.duration,
                    args.review + "; high anchor connection",
                )
                connector_review = connector["clearance_review"]
                geometry = scene_geometry(full_cfg)
                if (
                    connector_review["sampled_tcp_min"][2] < geometry["high_route_min_tcp_z_m"]
                    or connector_review["sampled_tip_min"][2] < geometry["high_route_min_tip_z_m"]
                ):
                    raise ValueError("High-anchor connection lacks board clearance")
                plan["steps"].extend(connector["steps"])
                q = np.asarray(plan["steps"][-1]["joint_positions"])
                reverse = reverse_executed_high_route(
                    root, args.outbound_plan, q, s["initial_waist"], s["held_left_q"], kin, cfg
                )
                plan["steps"].extend(reverse["steps"])
                q = np.asarray(plan["steps"][-1]["joint_positions"])
                plan["clearance_review"]["scope"] += "; " + reverse["clearance_review"]["scope"]
                plan["clearance_review"]["reverse_high_route"] = reverse["clearance_review"]
            elif args.via_pose:
                virtual = {
                    **s,
                    "commanded_q": q.tolist(),
                    "feedback": {**s["feedback"], "joint_positions": q.tolist()},
                }
                exit_plan = plan_move(
                    root,
                    virtual,
                    args.via_pose,
                    args.duration,
                    args.review + "; high-clearance board exit",
                )
                plan["steps"].extend(exit_plan["steps"])
                q = np.asarray(plan["steps"][-1]["joint_positions"])
                plan["clearance_review"]["scope"] += "; high-clearance board exit"
            saved = json.loads(reset_state_path(full_cfg).read_text())
            rest = build_plan(saved, q, s["initial_waist"], kin, cfg, 8)
            plan["steps"].extend(rest["steps"])
            plan["clearance_review"]["scope"] += "; followed by reviewed direct reset joint path"
            validate_plan(
                plan,
                kin,
                np.asarray(s["feedback"]["joint_positions"]),
                cfg,
                np.asarray(s["held_left_q"]),
            )
            write_json(root / (args.name + "_plan.json"), plan)
            if args.quiet:
                print(
                    json.dumps(
                        {
                            "stage": args.name,
                            "lift_target_tcp": args.pose,
                            "exit_target_tcp": args.via_pose,
                            "rest_target_tcp": plan["steps"][-1]["targets"]["right"],
                            "duration": sum(step["duration"] for step in plan["steps"]),
                        }
                    )
                )
            else:
                report({"return_plan": str(root / (args.name + "_plan.json"))})
            return
    else:
        cfg, kin = model(s)
        q = np.asarray(s["commanded_q"])
        if args.op == "reset":
            saved = json.loads(reset_state_path(full_cfg).read_text())
            plan = build_plan(saved, q, s["initial_waist"], kin, cfg, args.duration)
        else:
            value = args.gripper
            if value is None or not np.isfinite(value) or not 0 <= value <= 5.45:
                raise ValueError("Invalid gripper")
            plan = {
                "seed": q.tolist(),
                "locked_waist_q": s["initial_waist"],
                "steps": [
                    {
                        "joint_positions": q.tolist(),
                        "targets": {"right": matrix_pose(kin.fk(q)["right"])},
                        "grippers": {"right": value},
                        "duration": 0.4,
                    }
                ],
            }
        plan["clearance_review"] = {
            "passed": True,
            "scope": args.review,
            "evidence_directory": str(root),
        }
    if args.op == "release":
        authorization = json.loads((root / "release_authorization.json").read_text())
        if authorization.get("approved") is not True:
            raise ValueError("This round needs explicit release approval")
        return_plan = (args.return_plan or root / "return_plan.json").resolve()
        result = submit(
            root,
            args.name,
            plan,
            "release_and_return",
            {"return_plan": str(return_plan)},
            owner_root,
            quiet=args.quiet,
        )
    else:
        result = submit(
            root,
            args.name,
            plan,
            "rest" if args.op == "reset" else "stage",
            owner_root=owner_root,
            quiet=args.quiet,
        )
    report(summary(result))


if __name__ == "__main__":
    raise SystemExit(main())
