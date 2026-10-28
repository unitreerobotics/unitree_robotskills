#!/usr/bin/env python3
"""One persistent, guarded DDS owner accepting precomputed stage plans.

Starts with subscribers only. A stage starts publishing only with --execute.
Plans are produced offline; observations and planning never stop an active hold.
"""

import argparse
import fcntl
import json
import os
import signal
import tempfile
import time
from pathlib import Path

import numpy as np

from .arm_pose import arrival_report, validate_execution_config
from .camera_web import CameraCapture
from .common import controller_lock_path, load_config, reset_state_path, runtime_dir
from .dds_robot import DDSRobot
from .kinematics import Kinematics, matrix_pose


def write_json(path, value):
    path = Path(path)
    payload = json.dumps(value, indent=2, allow_nan=False) + "\n"
    # Each writer owns its temporary file, even for destinations sharing a stem.
    # Rename in the same directory keeps readers from seeing partial JSON.
    tmp = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=path.name + ".",
            suffix=".tmp",
            delete=False,
        ) as stream:
            tmp = Path(stream.name)
            stream.write(payload)
        tmp.replace(path)
    finally:
        if tmp is not None:
            tmp.unlink(missing_ok=True)


def snapshot(camera, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    with camera._lock:
        frames = dict(camera._frames)
    result = {}
    for name in camera.enabled:
        frame = frames.get(name)
        age = None if frame is None else time.monotonic() - frame[1]
        available = frame is not None and age <= camera.stale_after
        item = {
            "available": available,
            "received_at_unix": None if frame is None else frame[2],
            "age_seconds": age,
            "timestamp_source": "local reception, exposure timestamp unavailable",
        }
        if available:
            (directory / (name + ".jpg")).write_bytes(frame[0])
            item.update(path=name + ".jpg", shape=list(frame[3]))
        result[name] = item
    write_json(directory / "metadata.json", result)
    return result


def validate_plan(plan, kin, current, config, held_left):
    seed = np.asarray(plan["seed"], dtype=float)
    kin.check_joints(seed)
    if np.max(np.abs(seed - current)) > config["joint_tolerance"]:
        raise ValueError("State changed since planning")
    if not plan["steps"]:
        raise ValueError("Empty stage")
    if not plan.get("clearance_review", {}).get("passed", False):
        raise ValueError("Stage requires recorded complete-path clearance review")
    previous = seed
    for step in plan["steps"]:
        goal = kin.check_joints(step["joint_positions"])
        if not np.allclose(goal[:7], held_left, atol=1e-8, rtol=0):
            raise ValueError("Plan changes held left arm")
        duration = float(step["duration"])
        if not np.isfinite(duration) or duration <= 0:
            raise ValueError("Invalid duration")
        if 1.5 * np.max(np.abs(goal - previous)) / duration > config["max_joint_speed"] + 1e-8:
            raise ValueError("Plan exceeds joint speed")
        if set(step["targets"]) != {"right"} or set(step.get("grippers", {})) - {"right"}:
            raise ValueError("Only right arm/gripper can be commanded")
        report = arrival_report(kin, step, goal, config)
        if not report["reached"]:
            raise ValueError("Planned FK misses target")
        for value in step.get("grippers", {}).values():
            if not np.isfinite(value) or not 0 <= value <= 5.45:
                raise ValueError("Invalid gripper goal")
        previous = goal


RECOVERABLE = {"arrival_timeout", "gripper_timeout_contact_unconfirmed", "camera_unavailable"}
ARRIVAL_MODES = ("guarded", "observe")
OBSERVE_SETTLE_SECONDS = 0.3


def validate_observe_gripper_plan(plan, held_command):
    """Visual acceptance must precede a separate, fixed-arm gripper stage."""
    if not any(step.get("grippers") for step in plan["steps"]):
        return
    if len(plan["steps"]) != 1:
        raise ValueError("Observe mode requires a separate single-step fixed-arm gripper plan")
    goal = np.asarray(plan["steps"][0]["joint_positions"], dtype=float)
    if not np.allclose(goal, plan["seed"], atol=1e-8, rtol=0) or not np.allclose(
        goal, held_command, atol=1e-8, rtol=0
    ):
        raise ValueError("Observe mode cannot combine arm motion and gripper commands")


def validate_recovery(request, status, config):
    """Recovery never overrides control protection or substitutes a new task goal."""
    if status["phase"] != "paused_holding" or status["fault"] not in RECOVERABLE:
        raise ValueError("Recovery requires a recoverable pause, not control protection")
    review = request.get("recovery_review", {})
    if review.get("previous_stage") != status["stage"] or not review.get("diagnosis"):
        raise ValueError("Recovery requires diagnosis linked to the paused segment")
    if not review.get("fresh_visual_review") or not review.get("path_replanned"):
        raise ValueError("Recovery requires fresh visual review and replanned path")
    fb = status["feedback"]
    if "unavailable" in fb or not all(v["available"] for v in status["cameras"].values()):
        raise ValueError("Recovery requires fresh feedback and both cameras")


class Session:
    def __init__(
        self, root, execute, resume=None, waist_state=None, arrival_mode="guarded", config=None
    ):
        if arrival_mode not in ARRIVAL_MODES:
            raise ValueError(f"Unknown arrival mode: {arrival_mode}")
        self.arrival_mode = arrival_mode
        cfg = config if isinstance(config, dict) else load_config(config)
        self.full_cfg = cfg
        self.config_path = cfg.get("_config_path")
        selected_root = Path(root) if root is not None else Path("session")
        self.root = (
            selected_root if selected_root.is_absolute() else runtime_dir(cfg) / selected_root
        )
        self.execute = execute
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / "inbox").mkdir(exist_ok=True)
        lock_path = controller_lock_path(cfg)
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = lock_path.open("a+")
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if any((self.root / "inbox").glob("*.json")):
            self.lock.close()
            raise ValueError(
                "Session inbox contains pending JSON requests; inspect them before starting "
                "a new controller. Requests are never cleared or replayed automatically."
            )
        self.cfg = cfg["robot"]
        validate_execution_config(self.cfg)
        self.kin = Kinematics(self.cfg["ik_position_tolerance"], self.cfg["ik_rotation_tolerance"])
        self.robot = DDSRobot(self.cfg).connect(("left", "right"))
        self.initial_waist = self.robot.waist_q().copy()
        if waist_state:
            self.initial_waist = np.asarray(
                json.loads(Path(waist_state).read_text())["waist_q"], dtype=float
            )
        self.kin.set_waist(self.initial_waist)
        self.q = self.robot.arm_q()
        self.left = self.q[:7].copy()
        self.grippers = {"right": self.robot.gripper_q("right")}
        cfg["camera"]["enabled"] = [
            "cam_left_high",
            "cam_right_high",
            "cam_left_wrist",
            "cam_right_wrist",
        ]
        self.camera = CameraCapture(cfg["camera"]).start()
        from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication

        self.discovery = BuiltinDataReader(self.robot._participant, BuiltinTopicDcpsPublication)
        self.external_writers = {}
        self.started, self.stopping, self.fault = False, False, None
        self.active, self.phase, self.index = None, "observing", 0
        self.next_status, self.next_inbox = 0, 0
        self.tau = self.kin.tau(self.q)
        self.started_at = time.monotonic()
        self.resume_pending = bool(resume)
        if resume:
            saved = json.loads(Path(resume).read_text())
            if saved.get("publishing") or not saved.get("process_exited"):
                raise ValueError("Handover requires previous publisher confirmed stopped")
            if saved.get("fault") or saved.get("phase") != "holding":
                raise ValueError("Only healthy hold may be handed over")
            self.q = self.kin.check_joints(saved["commanded_q"])
            self.left = np.asarray(saved["held_left_q"])
            self.initial_waist = np.asarray(saved["initial_waist"])
            self.grippers = {"right": saved["feedback"]["grippers"]["right"]}
            self.check_health()
            self.tau = self.kin.tau(self.q)
            if not self.execute:
                raise ValueError("Resume hold requires --execute")
        self.history = []
        # Handover must retain the old reference, not the new measured angle.
        self.initial_waist = self.robot.lock_waist(self.initial_waist)

    def start_hold(self):
        self.discover()
        if time.monotonic() - self.started_at < 2 or self.external_writers:
            raise RuntimeError(f"Command publisher check failed: {self.external_writers}")
        if not all(v["available"] for v in self.camera.status().values()):
            raise RuntimeError("Fresh cameras required before holding")
        self.check_health()
        self.robot.lock_waist(self.initial_waist)
        self.robot.begin_motion(("right",))
        self.started, self.resume_pending, self.phase = True, False, "holding"

    def check_health(self):
        self.robot.check_motor_health()
        self.robot.arm_q()
        self.robot.waist_q()
        self.robot.gripper_q("right")

    def feedback(self):
        q = self.robot.arm_q()
        waist = self.robot.waist_q()
        self.kin.set_waist(waist)
        result = {
            "at_unix": time.time(),
            "joint_positions": q.tolist(),
            "waist_q": waist.tolist(),
            "poses": {s: matrix_pose(m) for s, m in self.kin.fk(q).items()},
            "grippers": {s: self.robot.gripper_q(s) for s in ("left", "right")},
        }
        result["motor_health"] = self.robot.motor_health()
        result["tracking_error_rad"] = (q - self.q).tolist()
        result["waist_error_rad"] = (waist - self.initial_waist).tolist()
        if self.active:
            result["arrival"] = arrival_report(
                self.kin, self.active["steps"][self.index], q, self.cfg
            )
            final_step = self.active["steps"][-1]
            if "task_target" in self.active:
                final_step = {**final_step, "targets": {"right": self.active["task_target"]}}
            result["stage_arrival"] = arrival_report(self.kin, final_step, q, self.cfg)
        return result

    def status(self):
        try:
            feedback = self.feedback()
        except Exception as exc:
            feedback = {"unavailable": str(exc)}
        return {
            "pid": os.getpid(),
            "config_path": self.config_path,
            "publishing": self.started,
            "phase": self.phase,
            "arrival_mode": self.arrival_mode,
            "fault": self.fault,
            "stage": None if self.active is None else self.active["id"],
            "step": self.index,
            "commanded_q": self.q.tolist(),
            "held_left_q": self.left.tolist(),
            "initial_waist": self.initial_waist.tolist(),
            "feedback": feedback,
            "external_writers": self.external_writers,
            "cameras": self.camera.status(),
        }

    def save_end(self, outcome):
        if self.active is None or self.active.get("recorded"):
            return
        self.active["recorded"] = True
        folder = Path(self.active["directory"])
        images = (
            snapshot(self.camera, folder / "after")
            if self.active.get("capture_images", True)
            else {}
        )
        write_json(
            folder / "result.json",
            {
                "outcome": outcome,
                "status": self.status(),
                "images": images,
                "feedback_history": self.history,
            },
        )
        print(
            json.dumps({"event": outcome, "directory": str(folder), "feedback": self.feedback()}),
            flush=True,
        )

    def discover(self):
        for sample in self.discovery.take(100):
            if not hasattr(sample, "topic_name"):
                continue
            if sample.topic_name not in (
                self.cfg["topic_low_command"],
                self.cfg["grippers"]["right"]["command_topic"],
            ):
                continue
            # Participant created by the SDK publisher differs from subscriber participant.
            # Before begin_motion every discovered writer is necessarily external.
            if not self.started:
                self.external_writers[str(sample.key)] = sample.topic_name

    def command(self, request):
        op = request["op"]
        if op == "stop":
            self.stopping = True
            return
        if op == "observe":
            images = snapshot(self.camera, request["directory"])
            write_json(Path(request["directory"]) / "state.json", self.feedback())
            print(json.dumps({"event": "observed", "images": images}), flush=True)
            return
        resting = op == "rest"
        recovering = op == "recover" or (resting and self.phase == "paused_holding")
        if recovering:
            validate_recovery(request, self.status(), self.cfg)
            self.check_health()
        elif (
            op not in ("stage", "rest") or self.fault or self.phase not in ("observing", "holding")
        ):
            raise ValueError("Stage rejected: invalid operation, fault, or busy")
        if not self.execute:
            raise ValueError("Motion requires --execute")
        if not all(v["available"] for v in self.camera.status().values()):
            raise ValueError("Fresh images required before a stage")
        if not self.started:
            self.discover()
            if time.monotonic() - self.started_at < 2 or self.external_writers:
                raise ValueError(f"Command publisher check failed: {self.external_writers}")
        plan = json.loads(Path(request["plan"]).read_text())
        if resting:
            saved = json.loads(reset_state_path(self.full_cfg).read_text())
            if not plan.get("steps") or not np.allclose(
                plan["steps"][-1]["joint_positions"][7:],
                saved["joint_positions"][7:],
                atol=1e-8,
                rtol=0,
            ):
                raise ValueError("Rest must finish at reset_right reference joints")
            if any(step.get("grippers") for step in plan["steps"]) or "task_target" in plan:
                raise ValueError(
                    "Rest must preserve the held gripper and cannot mark the task complete"
                )
            plan["rest_context"] = {
                "previous_stage": None if self.active is None else self.active["id"],
                "previous_fault": self.fault,
                "task_complete": False,
            }
        if recovering and not resting:
            original = self.active.get("task_target", self.active["steps"][-1]["targets"]["right"])
            if not np.allclose(plan.get("task_target", []), original, atol=1e-10, rtol=0):
                raise ValueError("Correction must preserve original task target")
        self.kin.set_waist(self.robot.waist_q())
        validate_plan(plan, self.kin, self.robot.arm_q(), self.cfg, self.left)
        if self.arrival_mode == "observe":
            validate_observe_gripper_plan(plan, self.q if self.started else plan["seed"])
        if self.started and np.max(np.abs(np.asarray(plan["seed"]) - self.q)) > 0.03:
            raise ValueError("Plan seed does not join continuously to held command")
        directory = Path(request["directory"])
        directory.mkdir(parents=True, exist_ok=False)
        capture_images = request.get("capture_images", True)
        if not isinstance(capture_images, bool):
            raise ValueError("capture_images must be a boolean")
        images = snapshot(self.camera, directory / "before") if capture_images else {}
        if capture_images and not all(v["available"] for v in images.values()):
            raise ValueError("Stage snapshot missing")
        write_json(directory / "plan.json", plan)
        write_json(directory / "target.json", request["target"])
        if recovering:
            write_json(directory / "recovery_review.json", request["recovery_review"])
        before = directory / "before"
        before.mkdir(exist_ok=True)
        write_json(before / "state.json", self.feedback())
        self.active = {
            **plan,
            "directory": str(directory),
            "id": directory.name,
            "capture_images": capture_images,
        }
        self.index = 0
        self.history = []
        self.fault = None
        self.start_q = self.q.copy() if self.started else np.asarray(plan["seed"]).copy()
        if not self.started:
            self.start_hold()
        self.phase, self.begin = "moving", time.monotonic()
        print(json.dumps({"event": "stage_started", "id": directory.name}), flush=True)

    def advance(self):
        if self.phase not in ("moving", "settling", "gripping"):
            return
        step = self.active["steps"][self.index]
        goal = np.asarray(step["joint_positions"])
        now = time.monotonic()
        if self.phase == "moving":
            u = min(1.0, (now - self.begin) / step["duration"])
            self.q = self.start_q + (goal - self.start_q) * (3 * u * u - 2 * u * u * u)
            self.tau = self.kin.tau(self.q)
            if u >= 1:
                self.phase, self.begin = "settling", now
        elif self.phase == "settling":
            report = arrival_report(self.kin, step, self.robot.arm_q(), self.cfg)
            if self.arrival_mode == "observe":
                # Errors remain diagnostic in feedback(); a completed interpolation
                # does not establish actual arrival or authorize a mixed gripper step.
                if now - self.begin >= OBSERVE_SETTLE_SECONDS:
                    self.grippers.update(step.get("grippers", {}))
                    self.phase, self.begin = "gripping", now
                return
            # Rest restores reviewed joint configurations, not an independently
            # specified Cartesian task pose. Preserve all TCP diagnostics while
            # using the existing joint gate for this gripper-invariant operation.
            if "rest_context" in self.active:
                report["reached"] = all(arm["joint_ok"] for arm in report["arms"].values())
            if "task_target" in self.active and self.index == len(self.active["steps"]) - 1:
                task_report = arrival_report(
                    self.kin,
                    {**step, "targets": {"right": self.active["task_target"]}},
                    self.robot.arm_q(),
                    self.cfg,
                )
                report["reached"] = report["reached"] and task_report["reached"]
            if report["reached"]:
                self.grippers.update(step.get("grippers", {}))
                self.phase, self.begin = "gripping", now
            elif now - self.begin > self.cfg["settle_timeout"]:
                self.fault, self.phase = "arrival_timeout", "paused_holding"
                self.save_end("arrival_timeout")
        elif self.phase == "gripping":
            reached = all(
                abs(self.robot.gripper_q(s) - g) <= self.cfg["gripper_tolerance"]
                for s, g in step.get("grippers", {}).items()
            )
            if reached:
                if self.index == len(self.active["steps"]) - 1:
                    self.phase = "holding"
                    self.save_end(
                        "motion_complete_unverified"
                        if self.arrival_mode == "observe"
                        else "reached"
                    )
                else:
                    self.index += 1
                    self.start_q = self.q.copy()
                    self.phase, self.begin = "moving", now
            elif now - self.begin > self.cfg["settle_timeout"]:
                self.grippers = {"right": self.robot.gripper_q("right")}
                self.fault, self.phase = "gripper_timeout_contact_unconfirmed", "paused_holding"
                self.save_end(self.fault)

    def run(self):
        try:
            while not self.stopping:
                tick = time.monotonic()
                if self.execute and not self.started:
                    self.discover()
                    if tick - self.started_at >= 2:
                        self.start_hold()
                if self.started:
                    self.check_health()
                    if self.phase in ("moving", "settling", "gripping") and not all(
                        v["available"] for v in self.camera.status().values()
                    ):
                        self.fault, self.phase = "camera_unavailable", "paused_holding"
                        self.save_end(self.fault)
                    self.advance()
                    self.robot.send(self.q, self.tau, self.grippers)
                if tick >= self.next_inbox:
                    for path in sorted((self.root / "inbox").glob("*.json")):
                        try:
                            self.command(json.loads(path.read_text()))
                            path.rename(path.with_suffix(".done"))
                        except Exception as exc:
                            write_json(path.with_suffix(".error"), {"error": str(exc)})
                            path.rename(path.with_suffix(".rejected"))
                            print(
                                json.dumps({"event": "request_rejected", "error": str(exc)}),
                                flush=True,
                            )
                    self.next_inbox = tick + 0.2
                if tick >= self.next_status:
                    write_json(self.root / "status.json", self.status())
                    if self.phase in ("moving", "settling", "gripping"):
                        self.history.append(self.feedback())
                    self.next_status = tick + 0.25
                time.sleep(max(0, 1 / self.cfg["rate_hz"] - (time.monotonic() - tick)))
        except Exception as exc:
            self.fault, self.phase = str(exc), "protection_stopped"
            print(json.dumps({"event": "protection_stopped", "error": str(exc)}), flush=True)
            try:
                self.save_end("protection_stopped")
                folder = self.root / ("protection_" + str(time.time_ns()))
                images = snapshot(self.camera, folder)
                write_json(folder / "status.json", {"status": self.status(), "images": images})
            except Exception as save_error:
                write_json(self.root / "save_error.json", {"error": str(save_error)})
        finally:
            final = self.status()
            self.robot.close()
            self.started = False
            self.camera.close()
            final.update(publishing=False, process_exited=True)
            write_json(self.root / "status.json", final)
            print("Command publishing stopped.", flush=True)
        return 1 if self.fault else 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", help="Explicit local hardware YAML configuration")
    p.add_argument("--root", help="Session directory; relative paths use runtime.directory")
    p.add_argument("--execute", action="store_true")
    p.add_argument("--resume", help="Stopped healthy session status for guarded tool handover")
    p.add_argument(
        "--waist-state",
        help="Task startup state JSON; retains original waist reference across reset handover",
    )
    p.add_argument(
        "--arrival-mode",
        choices=ARRIVAL_MODES,
        default="guarded",
        help=(
            "guarded: pause on joint/TCP arrival timeout; observe: record errors, "
            "wait 0.3 s after interpolation, require visual acceptance"
        ),
    )
    args = p.parse_args(argv)
    session = Session(
        args.root,
        args.execute,
        args.resume,
        args.waist_state,
        args.arrival_mode,
        config=args.config,
    )
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: setattr(session, "stopping", True))
    return session.run()


if __name__ == "__main__":
    raise SystemExit(main())
