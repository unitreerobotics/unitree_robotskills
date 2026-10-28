"""Installable workflow CLI with explicit hardware-only observation and motion."""

from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
import time
import uuid
from pathlib import Path

from .config import load_config
from .job import import_legacy_job, load_job, validate_job
from .runner import Executor, preflight
from .storage import atomic_json


def emit(value):
    print(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), flush=True)


def serve(executor, job):
    """Retain ownership; SIGINT stops motion or releases an already terminal run."""
    running = True
    release = False
    monitor_done = threading.Event()

    def monitor():
        nonlocal release
        seen = None
        while not monitor_done.wait(0.1):
            try:
                control = json.loads((executor.root_directory / "control.json").read_text())
                if control.get("job_id") != job["job_id"] or control.get("id") == seen:
                    continue
                if control.get("action") == "stop":
                    seen = control["id"]
                    executor.stop()
                elif control.get("action") == "release" and not running:
                    seen = control["id"]
                    release = True
            except (OSError, ValueError, KeyError):
                continue

    def stop_or_release(signum, _frame):
        nonlocal release
        if running and not executor.stop_event.is_set():
            executor.stop()
        else:
            release = True

    handlers = {sig: signal.signal(sig, stop_or_release) for sig in (signal.SIGINT, signal.SIGTERM)}
    worker = threading.Thread(target=monitor, name="paint-execute-controls", daemon=True)
    worker.start()
    try:
        result = executor.run(job)
        running = False
        try:
            emit(result)
            print(
                "Controller retains hold. Press Ctrl-C to explicitly release DDS.",
                file=sys.stderr,
                flush=True,
            )
        except OSError:
            # A closed output pipe must not relinquish an acquired controller.
            pass
        while not release:
            try:
                executor._save(health=executor.backend.health())
            except OSError as exc:
                executor.progress["storage_error"] = str(exc)
            time.sleep(0.5)
        return 0 if result["state"] == "job_completed" else 1
    finally:
        monitor_done.set()
        worker.join(timeout=1)
        executor.close()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Standalone G1-D painting execution")
    subs = parser.add_subparsers(dest="command", required=True)
    for name in ("validate", "compile", "run", "simulate"):
        sub = subs.add_parser(name)
        sub.add_argument("--config", required=True)
        sub.add_argument("--job", required=True)
        if name == "compile":
            sub.add_argument("--output", required=True)
        if name == "validate":
            sub.add_argument("--model-check", action="store_true")
    for name in ("compile-probe", "probe"):
        sub = subs.add_parser(name)
        sub.add_argument("--config", required=True)
        sub.add_argument("--probe", required=True)
        if name == "compile-probe":
            sub.add_argument("--output", required=True)
    observe = subs.add_parser("observe")
    observe.add_argument("--config", required=True)
    observe.add_argument("--label", required=True)
    observe.add_argument("--output", required=True)
    calibrate = subs.add_parser("calibrate")
    calibrate.add_argument("--measurements", required=True)
    calibrate.add_argument("--output", required=True)
    for name in ("build-plan", "preview"):
        sub = subs.add_parser(name)
        sub.add_argument("--strokes", required=True)
        sub.add_argument("--calibration", required=True)
        sub.add_argument("--output", required=True)
        if name == "build-plan":
            sub.add_argument("--job-id", required=True)
    prepare = subs.add_parser("prepare")
    for name in ("config", "strokes", "measurements", "job-id", "output"):
        prepare.add_argument("--" + name, required=True)
    prepare.add_argument("--simulation", action="store_true")
    for name in ("init-task", "demo-workflow"):
        sub = subs.add_parser(name)
        sub.add_argument("--output", required=True)
    run_task = subs.add_parser("run-task")
    run_task.add_argument("--task", required=True)
    for name in ("status", "stop", "release"):
        status = subs.add_parser(name)
        status.add_argument("--config", required=True)
    demo = subs.add_parser("demo-job")
    demo.add_argument("--output", required=True)
    model_info = subs.add_parser("model-info")
    model_info.add_argument("--urdf", required=True)
    model_info.add_argument("--output", required=True)
    legacy = subs.add_parser("import-legacy")
    legacy.add_argument("--job", required=True)
    legacy.add_argument(
        "--entry-route",
        required=True,
        help="JSON N x 14 reviewed route, starting at deployment rest",
    )
    legacy.add_argument("--grips", nargs=2, type=float, required=True)
    legacy.add_argument("--job-id", required=True)
    legacy.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    try:
        from .workflow import read_json

        if args.command == "run-task":
            from .task import verify_task

            bundle = verify_task(args.task)
            config, job = bundle["config"], bundle["job"]
            if job["purpose"] == "simulation":
                from .simulation import SimulatedBackend, SimulatedKinematics

                executor = Executor(config, SimulatedBackend(job), SimulatedKinematics())
                try:
                    result = executor.run(job)
                    emit(result)
                    return 0 if result["state"] == "job_completed" else 1
                finally:
                    executor.close()
            from .g1 import G1DDSBackend
            from .kinematics import PinocchioKinematics

            return serve(Executor(config, G1DDSBackend(config), PinocchioKinematics(config)), job)
        if args.command == "init-task":
            from .workflow import initialize_task

            emit(initialize_task(args.output))
            return 0
        if args.command == "demo-workflow":
            from .workflow import run_demo_workflow

            result = run_demo_workflow(args.output)
            emit(result)
            return 0 if result["painting_state"] == "job_completed" else 1
        if args.command == "calibrate":
            from .calibration import calibrate_measurements

            calibration = calibrate_measurements(read_json(args.measurements))
            atomic_json(args.output, calibration)
            emit(
                {
                    "output": args.output,
                    "physical_execution": False,
                    "purpose": calibration["purpose"],
                    "paper": calibration["paper"]["report"],
                }
            )
            return 0
        if args.command in {"build-plan", "preview"}:
            from .task import build_plan, render_preview

            strokes, calibration = read_json(args.strokes), read_json(args.calibration)
            if args.command == "build-plan":
                plan = build_plan(strokes, calibration, args.job_id)
                atomic_json(args.output, plan)
                emit(
                    {
                        "output": args.output,
                        "job_id": plan["job_id"],
                        "batches": len(plan["batches"]),
                        "physical_execution": False,
                    }
                )
            else:
                target = Path(args.output)
                if target.suffix.lower() != ".svg":
                    raise ValueError("preview output must use .svg")
                svg = render_preview(strokes, calibration)
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(svg, encoding="utf-8")
                emit(
                    {
                        "output": args.output,
                        "physical_execution": False,
                        "preview": "planned strokes",
                    }
                )
            return 0
        if args.command == "prepare":
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
            return 0
        if args.command == "model-info":
            import pinocchio as pin

            from .kinematics import ARM_JOINT_NAMES

            urdf = Path(args.urdf).expanduser().resolve()
            if not urdf.is_file():
                raise ValueError("URDF file does not exist")
            model = pin.buildModelFromUrdf(str(urdf))
            template = {
                "urdf": str(urdf),
                "left_frame": "left_wrist_yaw_joint",
                "right_frame": "right_wrist_yaw_joint",
                "reference_positions": {
                    name: None for name in model.names[1:] if name not in ARM_JOINT_NAMES
                },
            }
            atomic_json(args.output, template)
            emit(
                {
                    "output": args.output,
                    "required_reference_joints": len(template["reference_positions"]),
                    "note": "fill actual fixed joint angles and calibrated task frames before use",
                }
            )
            return 0
        if args.command == "demo-job":
            from .simulation import demo_job

            atomic_json(args.output, validate_job(demo_job()))
            emit({"output": str(Path(args.output).resolve()), "physical_execution": False})
            return 0
        if args.command == "import-legacy":
            job = import_legacy_job(
                json.loads(Path(args.job).read_text()),
                entry_q14=json.loads(Path(args.entry_route).read_text()),
                grip_q2=args.grips,
                job_id=args.job_id,
            )
            atomic_json(args.output, job)
            emit({"output": args.output, "physical_review_transferred": False})
            return 0
        config = load_config(args.config)
        if args.command == "observe":
            from .calibration import observe_device
            from .g1 import G1DDSBackend
            from .kinematics import PinocchioKinematics

            if Path(args.output).exists():
                raise ValueError("observation output already exists; preserve earlier measurements")
            record = observe_device(
                config, G1DDSBackend(config), PinocchioKinematics(config), args.label
            )
            atomic_json(args.output, record)
            emit({"output": args.output, "robot_commands_sent": 0, "contact_verified": False})
            return 0
        if args.command in {"compile-probe", "probe"}:
            from .kinematics import PinocchioKinematics
            from .probe import ProbeExecutor, compile_probe, preflight_probe

            model = PinocchioKinematics(config)
            if args.command == "compile-probe":
                probe_job = compile_probe(read_json(args.probe), model)
                report = preflight_probe(probe_job, config, model)
                atomic_json(args.output, probe_job)
                emit({**report, "output": args.output})
                return 0
            from .g1 import G1DDSBackend

            return serve(ProbeExecutor(config, G1DDSBackend(config), model), read_json(args.probe))
        if args.command in {"status", "stop", "release"}:
            directory = Path(config["storage"]["directory"])
            status = json.loads((directory / "status.json").read_text())
            if args.command == "status":
                emit(status)
                return 0
            if status.get("released"):
                raise ValueError("controller is already released")
            if args.command == "release" and status["state"] in {"running", "connecting"}:
                raise ValueError("stop the job and wait for its terminal status before release")
            atomic_json(
                directory / "control.json",
                {"job_id": status["job_id"], "id": uuid.uuid4().hex, "action": args.command},
            )
            emit(
                {
                    "control_requested": args.command,
                    "job_id": status["job_id"],
                    "note": "request written; query status for confirmation",
                }
            )
            return 0
        if args.command == "compile":
            from .kinematics import PinocchioKinematics, compile_job

            model = PinocchioKinematics(config)
            job = validate_job(compile_job(json.loads(Path(args.job).read_text()), model))
            report = preflight(job, config, model)
            atomic_json(args.output, job)
            emit({**report, "output": args.output})
            return 0
        job = load_job(args.job)
        if args.command == "validate":
            report = {
                "job_id": job["job_id"],
                "batches": len(job["batches"]),
                "reload_every": job["reload_every"],
                "model_checked": False,
            }
            if args.model_check:
                from .kinematics import PinocchioKinematics

                report.update(
                    preflight(job, config, PinocchioKinematics(config)), model_checked=True
                )
            emit(report)
            return 0
        if args.command == "simulate":
            from .simulation import SimulatedBackend, SimulatedKinematics

            if job.get("purpose") != "simulation":
                raise ValueError("synthetic simulation only accepts purpose=simulation fixtures")
            executor = Executor(config, SimulatedBackend(job), SimulatedKinematics())
            try:
                result = executor.run(job)
                emit(result)
                return 0 if result["state"] == "job_completed" else 1
            finally:
                executor.close()
        from .g1 import G1DDSBackend
        from .kinematics import PinocchioKinematics

        return serve(Executor(config, G1DDSBackend(config), PinocchioKinematics(config)), job)
    except (ValueError, RuntimeError, OSError, ImportError, KeyError, TypeError) as exc:
        emit({"ok": False, "error": f"{type(exc).__name__}: {exc}"})
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
