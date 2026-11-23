"""Command line of the R1-D painting skill: ``python -m unitree_gpt6.r1d_draw.src.main``.

Commands that reach the robot say so in their help text; every other command is offline.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .config import load_config
from .storage import atomic_json, read_json


def emit(value):
    print(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), flush=True)


def _new_file(path):
    path = Path(path)
    if path.exists():
        raise ValueError(f"{path} already exists; keep earlier records and choose a new name")
    return path


def init_task(args):
    from .workflow import initialize_task

    emit(initialize_task(args.output))


def demo_workflow(args):
    from .workflow import run_demo_workflow

    result = run_demo_workflow(args.output)
    emit(result)
    return 0 if result["state"] == "job_completed" else 1


def observe(args):
    from .calibration import observe as take
    from .kinematics import PinocchioKinematics
    from .robot import connect

    config = load_config(args.config)
    robot = connect(config)
    try:
        model = PinocchioKinematics(config, robot.state()["waist_yaw"])
        atomic_json(_new_file(args.output), take(config, robot, model, args.label))
    finally:
        robot.close()
    emit({"output": args.output, "robot_commands_sent": 0})


def photo(args):
    from .robot import connect, png_size

    robot = connect(load_config(args.config))
    try:
        png = robot.photo(args.camera)
    finally:
        robot.close()
    _new_file(args.output).write_bytes(png)
    emit({"output": args.output, "size": list(png_size(png)), "robot_commands_sent": 0})


def calibrate(args):
    from .calibration import calibrate_measurements

    calibration = calibrate_measurements(read_json(args.measurements))
    atomic_json(args.output, calibration)
    emit({"output": args.output, "paper": calibration["paper"]["report"]})


def preview(args):
    from .task import render_preview

    target = Path(args.output)
    if target.suffix.lower() != ".svg":
        raise ValueError("preview output must use .svg")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        render_preview(read_json(args.strokes), read_json(args.calibration)), encoding="utf-8"
    )
    emit({"output": args.output, "preview": "planned strokes, not a photograph"})


def prepare(args):
    from .workflow import prepare_task

    emit(
        prepare_task(
            config_path=args.config,
            strokes_path=args.strokes,
            measurements_path=args.measurements,
            job_id=args.job_id,
            output=args.output,
            simulation=args.simulation,
        )
    )


def run_task(args):
    from .runner import Executor
    from .task import verify_task

    bundle = verify_task(args.task)
    config, job = bundle["config"], bundle["job"]
    if job["purpose"] == "simulation":
        from .simulation import SimulatedKinematics, SimulatedRobot

        robot = SimulatedRobot(
            job["entry_q14"][0], waist_yaw=job["waist_yaw"], grip_q2=job["grip_q2"]
        )
        model = SimulatedKinematics()
    else:
        from .kinematics import PinocchioKinematics
        from .robot import connect

        robot, model = connect(config), PinocchioKinematics(config, job["waist_yaw"])
    try:
        executor = Executor(config, robot, model, tip_correction=not args.no_correction)
        result = executor.run(job, resume=args.resume, lift=args.lift)
    finally:
        robot.close()
    emit(result)
    return 0 if result["state"] == "job_completed" else 1


def status(args):
    from .robot import connect

    config = load_config(args.config)
    path = Path(config["storage"]["directory"]) / "status.json"
    robot = connect(config)
    try:
        controller = robot.status()
    finally:
        robot.close()
    keys = ("state", "error", "alive", "frames_sent", "tracking_error_rad", "last_route", "route")
    emit(
        {
            "job": read_json(path) if path.exists() else None,
            "controller": None if controller is None else {k: controller.get(k) for k in keys},
        }
    )


def _robot_call(name):
    def call(args):
        from .robot import connect

        robot = connect(load_config(args.config))
        try:
            result = getattr(robot, name)()
        finally:
            robot.close()
        emit({"done": name, "state": result.get("state") if isinstance(result, dict) else None})

    return call


COMMANDS = {
    "init-task": (init_task, "write blank task forms", ["output"]),
    "demo-workflow": (demo_workflow, "run a synthetic task with the simulated robot", ["output"]),
    "observe": (
        observe,
        "ROBOT, read-only: save joints and brush-tip poses",
        ["config", "label", "output"],
    ),
    "photo": (photo, "ROBOT, read-only: save one camera frame", ["config", "camera", "output"]),
    "calibrate": (calibrate, "fit the paper plane from measurements", ["measurements", "output"]),
    "preview": (preview, "draw the planned strokes as SVG", ["strokes", "calibration", "output"]),
    "prepare": (
        prepare,
        "compile a complete task directory",
        ["config", "strokes", "measurements", "job-id", "output"],
    ),
    "run-task": (run_task, "ROBOT, MOVES: run or resume a prepared task", ["task"]),
    "status": (status, "ROBOT, read-only: job progress and controller state", ["config"]),
    "take-over": (_robot_call("take_over"), "ROBOT: hold the arms where they are", ["config"]),
    "stop": (_robot_call("stop"), "ROBOT: cancel the running route, keep holding", ["config"]),
    "relax": (
        _robot_call("relax"),
        "ROBOT: arms go limp slowly; grippers keep holding",
        ["config"],
    ),
    "release": (_robot_call("release"), "ROBOT: stop commanding; arms are not relaxed", ["config"]),
}


def main(argv=None):
    parser = argparse.ArgumentParser(prog="r1d-draw", description="R1-D painting")
    subs = parser.add_subparsers(dest="command", required=True)
    for name, (_, text, options) in COMMANDS.items():
        sub = subs.add_parser(name, help=text)
        for option in options:
            sub.add_argument("--" + option, required=True)
    subs.choices["prepare"].add_argument("--simulation", action="store_true")
    run = subs.choices["run-task"]
    run.add_argument("--resume", action="store_true", help="continue an interrupted job")
    run.add_argument(
        "--lift", action="store_true", help="when resuming, first raise the brush straight up"
    )
    run.add_argument(
        "--no-correction", action="store_true", help="do not correct the measured tip offset"
    )
    args = parser.parse_args(argv)
    try:
        return COMMANDS[args.command][0](args) or 0
    except (
        ValueError,
        RuntimeError,
        OSError,
        ImportError,
        KeyError,
        TypeError,
        TimeoutError,
    ) as exc:
        emit({"ok": False, "error": f"{type(exc).__name__}: {exc}"})
        return 1
