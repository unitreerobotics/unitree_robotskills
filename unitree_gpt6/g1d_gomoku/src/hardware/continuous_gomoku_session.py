"""Persistent gomoku controller; release snapshot immediately followed by return."""

import argparse
import hashlib
import json
import signal
import time
from pathlib import Path

import numpy as np

from .arm_session import Session, snapshot, validate_plan, write_json
from .common import configure, reset_state_path, scene_geometry
from .kinematics import matrix_pose


class ContinuousSession(Session):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.return_plan = None
        self.finish_pending = None
        self.release_settled_at = None
        self.cameras_enabled = True
        self.restore_left_target = None
        self.handover_target = self.q.copy()
        self.q = self.robot.arm_q().copy()
        if np.max(np.abs(self.q - self.handover_target)) > 0.05:
            raise ValueError("Handover state changed; do not restore old command")
        self.left = self.q[:7].copy()
        self.tau = self.kin.tau(self.q)

    def start_hold(self):
        super().start_hold()

    def command(self, request):
        if request["op"] == "restore_initial":
            if self.phase != "holding" or self.fault or not self.started or not self.execute:
                raise ValueError("Initial restoration requires healthy hold")
            state = json.loads(Path(request["state"]).read_text())
            target = np.asarray(state["joint_positions"], dtype=float)
            measured = self.robot.arm_q()
            self.kin.check_joints(target)
            if not np.allclose(state["waist_q"], self.initial_waist, atol=1e-8, rtol=0):
                raise ValueError("Initial waist does not match controller lock")
            if np.max(np.abs(self.robot.waist_q() - self.initial_waist)) > 0.01:
                raise ValueError("Initial waist has not settled before arm restoration")
            if np.max(np.abs(measured - self.q)) > 0.08 or np.max(np.abs(target - measured)) > 1.2:
                raise ValueError("Measured-to-initial joint gap needs fresh review")
            lower, upper = self.kin.model.lowerPositionLimit, self.kin.model.upperPositionLimit
            if np.min(np.minimum(target - lower, upper - target)) < 0.01:
                raise ValueError("Initial target lacks joint-limit margin")
            self.kin.set_waist(self.initial_waist)
            geometry = scene_geometry(self.full_cfg)
            for u in np.linspace(0, 1, 21):
                q = measured + (target - measured) * u
                poses = self.kin.fk(q)
                for side in ("left", "right"):
                    point = poses[side][:3, 3]
                    bounds = geometry["rest_corridor"][side]
                    if np.any(point < bounds["min_m"]) or np.any(point > bounds["max_m"]):
                        raise ValueError("Restoration leaves calibrated rest corridor")
            duration = max(6.0, 1.5 * np.max(np.abs(target - self.q)) / self.cfg["max_joint_speed"])
            plan = {
                "seed": self.q.tolist(),
                "locked_waist_q": self.initial_waist.tolist(),
                "steps": [
                    {
                        "joint_positions": target.tolist(),
                        "duration": float(duration),
                        "targets": {
                            side: matrix_pose(pose) for side, pose in self.kin.fk(target).items()
                        },
                        "grippers": {"right": float(state["grippers"]["right"])},
                    }
                ],
                "clearance_review": {
                    "passed": True,
                    "scope": "21-sample FK within calibrated rest corridor; joint margins checked",
                    "scene_sha256": geometry["_sha256"],
                },
            }
            directory = Path(request["directory"])
            directory.mkdir(parents=True, exist_ok=False)
            images = snapshot(self.camera, directory / "before")
            if not all(v["available"] for v in images.values()):
                raise ValueError("Fresh images required for restoration")
            write_json(directory / "plan.json", plan)
            write_json(directory / "before/state.json", self.feedback())
            self.active = {
                **plan,
                "directory": str(directory),
                "id": directory.name,
                "capture_images": True,
            }
            self.restore_left_target = target[:7].copy()
            self.index, self.history = 0, []
            self.start_q = self.q.copy()
            self.phase, self.begin = "moving", time.monotonic()
            return
        if request["op"] == "new_round":
            if self.phase != "holding" or self.fault:
                raise ValueError("New round needs healthy rest hold")
            if not self.cameras_enabled:
                self.camera.start()
                self.cameras_enabled = True
            return
        if request["op"] == "release_and_return":
            plan = json.loads(Path(request["return_plan"]).read_text())
            release = json.loads(Path(request["plan"]).read_text())
            if self.phase != "holding" or self.fault or not self.started or not self.execute:
                raise ValueError("Release requires healthy hold with --execute")
            grippers = release["steps"][0].get("grippers", {}) if len(release["steps"]) == 1 else {}
            value = grippers.get("right")
            limits = self.cfg["grippers"]["right"]
            if (
                set(grippers) != {"right"}
                or type(value) not in (int, float)
                or not np.isfinite(value)
                or not limits["min"] <= value <= limits["max"]
                or (value - self.grippers["right"]) * (limits["open"] - limits["close"]) <= 0
            ):
                raise ValueError("Require fixed-arm release with an explicit opening target")
            for joints in (release["seed"], release["steps"][0]["joint_positions"]):
                goal = self.kin.check_joints(joints)
                if not np.allclose(goal, self.q, atol=1e-8, rtol=0):
                    raise ValueError("Require fixed-arm release at the held command")
            if not np.allclose(plan["seed"], self.q, atol=1e-7, rtol=0):
                raise ValueError("Return must begin at held release command")
            if not np.allclose(
                plan.get("locked_waist_q", []), self.initial_waist, atol=1e-8, rtol=0
            ):
                raise ValueError("Return must preserve the held waist reference")
            self.kin.set_waist(self.initial_waist)
            validate_plan(plan, self.kin, self.robot.arm_q(), self.cfg, self.left)
            rest = json.loads(reset_state_path(self.full_cfg).read_text())
            if not np.allclose(
                plan["steps"][-1]["joint_positions"][7:],
                rest["joint_positions"][7:],
                atol=1e-7,
                rtol=0,
            ):
                raise ValueError("Return must end at reset reference")
            if any(step.get("grippers") for step in plan["steps"]):
                raise ValueError("Return preserves release command")
            directory = Path(request["directory"]).resolve()
            authorization_path = Path(request["release_authorization"]).resolve()
            if (
                authorization_path.parent != directory.parent
                or authorization_path.name != directory.name + "_release_authorization.json"
            ):
                raise ValueError("Release authorization must belong to this stage")
            authorization = json.loads(authorization_path.read_text())
            if (
                authorization.get("approved") is not True
                or authorization.get("directory") != str(directory)
                or authorization.get("release_plan_sha256")
                != hashlib.sha256(Path(request["plan"]).read_bytes()).hexdigest()
                or authorization.get("return_plan_sha256")
                != hashlib.sha256(Path(request["return_plan"]).read_bytes()).hexdigest()
            ):
                raise ValueError("Release authorization does not match the reviewed plans")
            super().command({**request, "op": "stage", "capture_images": False})
            self.finish_pending = {"plan": plan, "directory": request["directory"]}
            self.release_settled_at = None
            return
        super().command(request)

    def advance(self):
        if self.phase == "returning":
            step = self.return_plan["steps"][self.index]
            u = min(1.0, (time.monotonic() - self.begin) / step["duration"])
            goal = np.asarray(step["joint_positions"])
            self.q = self.start_q + (goal - self.start_q) * (3 * u * u - 2 * u * u * u)
            self.kin.set_waist(self.robot.waist_q())
            self.tau = self.kin.tau(self.q)
            if u >= 1:
                self.return_history.append(
                    {"label": step.get("label", str(self.index)), "feedback": self.feedback()}
                )
                self.index += 1
                if self.index == len(self.return_plan["steps"]):
                    self.phase = "holding"
                    write_json(
                        self.finish_directory / "return_result.json",
                        {
                            "status": "reset_holding",
                            "images_saved_without_analysis": True,
                            "history": self.return_history,
                        },
                    )
                    self.return_plan = None
                else:
                    self.start_q = self.q.copy()
                    self.begin = time.monotonic()
            return
        super().advance()
        if self.restore_left_target is not None and self.phase == "holding":
            self.left = self.restore_left_target
            self.restore_left_target = None
        if self.finish_pending and self.phase == "holding" and not self.fault:
            # The base gripper stage already enforces the configured tolerance
            # and timeout. Start the prepared return immediately after it settles.
            pending = self.finish_pending
            self.finish_directory = Path(pending["directory"])
            # Save exactly once; no visual inference or human/model gate follows.
            snapshot(self.camera, self.finish_directory / "released_images")
            self.camera.close()
            self.cameras_enabled = False
            self.return_plan = pending["plan"]
            self.finish_pending = None
            self.return_history = []
            self.index = 0
            self.start_q = self.q.copy()
            self.active = None
            self.phase = "returning"
            self.begin = time.monotonic()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    p.add_argument("--config", help="Local hardware YAML configuration")
    p.add_argument("--root", default="session")
    p.add_argument("--execute", action="store_true")
    p.add_argument("--resume", help="Stopped healthy session handover")
    p.add_argument("--waist-state", help="Optional reviewed task startup state")
    a = p.parse_args(argv)
    if a.config:
        configure(a.config)
    s = ContinuousSession(
        a.root, a.execute, resume=a.resume, waist_state=a.waist_state, arrival_mode="observe"
    )
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: setattr(s, "stopping", True))
    return s.run()


if __name__ == "__main__":
    raise SystemExit(main())
