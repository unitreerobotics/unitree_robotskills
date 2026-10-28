"""Independent, reviewed measurement paths; no material or painting semantics.

Compiling and preflighting are hardware free. Captures contain measured joint
feedback and model FK only: physical contact always needs operator confirmation.
"""

from __future__ import annotations

import copy
import hashlib
import time

import numpy as np

from .config import vector, waist_vector
from .job import POSE_KEYS, fingerprint, matrix
from .runner import Executor, samples
from .storage import ControllerLease, atomic_json, claim_attempt

FORMAT = "astra-paint-probe/v1"


def _validate_probe(value, *, compiled=True):
    probe = copy.deepcopy(value)
    if not isinstance(probe, dict) or probe.get("format") != FORMAT:
        raise ValueError(f"expected format={FORMAT}")
    if not isinstance(probe.get("job_id"), str) or not probe["job_id"].strip():
        raise ValueError("probe.job_id is required")
    if probe.get("purpose") not in {"measurement", "simulation"}:
        raise ValueError("probe.purpose must be measurement or simulation")
    if probe.get("hand") not in {"left", "right"}:
        raise ValueError("probe.hand must be left or right")
    grips = vector(probe.get("grip_q2"), 2, "grip_q2")
    if np.any(grips < 0) or np.any(grips > 5.45):
        raise ValueError("Dex1 grip_q2 must use device units within 0..5.45")
    waist_vector(probe.get("waist_q3"))
    entry = matrix(probe.get("entry_q14"), "entry_q14")
    if not isinstance(probe.get("clearance_review"), str) or not probe["clearance_review"].strip():
        raise ValueError("clearance_review must describe the reviewed whole-arm/tool paths")
    if probe.get("end_clear") is not True:
        raise ValueError("probe needs a reviewed final withdrawal and end_clear=true")
    action = probe.get("action")
    if not isinstance(action, list) or not 1 <= len(action) <= 4096:
        raise ValueError("probe.action needs 1..4096 EEF poses")
    for point in action:
        if not isinstance(point, dict):
            raise ValueError("each probe action must be an EEF pose object")
        vector([point.get(key) for key in POSE_KEYS], 6, "EEF pose")
        if "gripper" in point:
            grip = vector([point["gripper"]], 1, "EEF gripper")[0]
            if abs(grip - grips[1 if probe["hand"] == "right" else 0]) > 1e-8:
                raise ValueError("probe gripper differs from the fixed probe grip")
    indices = probe.setdefault("capture_indices", [])
    if (
        not isinstance(indices, list)
        or any(type(i) is not int or not 0 <= i < len(action) for i in indices)
        or len(set(indices)) != len(indices)
    ):
        raise ValueError("capture_indices must contain unique, in-range action integers")
    lifted = probe.setdefault("lifted_capture_indices", [])
    if (
        not isinstance(lifted, list)
        or any(type(i) is not int or i not in indices for i in lifted)
        or len(set(lifted)) != len(lifted)
    ):
        raise ValueError("lifted_capture_indices must contain unique indices from capture_indices")
    park_value = probe.get("park_q14")
    if not compiled and isinstance(park_value, list) and park_value and park_value[0] is None:
        if len(park_value) < 2:
            raise ValueError("park_q14 needs a reviewed route after its null start")
        matrix(park_value[1:], "park_q14 reviewed route")
    else:
        matrix(park_value, "park_q14")
    path_value = probe.get("joint_path_q14")
    if path_value is None and not compiled:
        return probe
    path = matrix(path_value, "joint_path_q14")
    if len(path) != len(action):
        raise ValueError("every probe EEF action needs a precomputed q14")
    frozen = slice(0, 7) if probe["hand"] == "right" else slice(7, 14)
    if not np.allclose(path[:, frozen], entry[-1, frozen], atol=1e-10, rtol=0):
        raise ValueError("nonmeasurement arm must remain frozen during probe actions")
    if compiled and not np.allclose(np.asarray(park_value)[0], path[-1], atol=1e-8, rtol=0):
        raise ValueError("park route must start at the last reviewed lifted action")
    return probe


def _limits(kinematics):
    lower = vector(kinematics.lower, 14, "model lower limits")
    upper = vector(kinematics.upper, 14, "model upper limits")
    if np.any(lower >= upper):
        raise ValueError("invalid model joint limits")
    return lower, upper


def _checked_q(value, lower, upper):
    q = vector(value, 14, "probe joint configuration")
    if np.any(q < lower) or np.any(q > upper):
        raise ValueError("probe joint path exceeds model limits")
    return q


def _fk(kinematics, q, hand):
    result = np.asarray(kinematics.fk(q, hand), dtype=float)
    if result.shape != (4, 4) or not np.isfinite(result).all():
        raise ValueError("nonfinite or invalid FK during probe path checking")
    return result


def compile_probe(value, kinematics):
    """Fill missing IK and park start, checking all targets without devices.

    ``preflight_probe`` adds configuration-dependent interpolation/workspace and
    waist-reference checks. Neither function infers a physical contact depth.
    """
    plan = copy.deepcopy(value)
    if isinstance(plan, dict):
        plan.setdefault("format", FORMAT)
    probe = _validate_probe(plan, compiled=False)
    if probe["purpose"] != "simulation" and getattr(kinematics, "is_simulation", False):
        raise ValueError("synthetic kinematics cannot validate a physical probe")
    lower, upper = _limits(kinematics)
    for q in matrix(probe["entry_q14"], "entry_q14"):
        _checked_q(q, lower, upper)
        for hand in ("left", "right"):
            _fk(kinematics, q, hand)
    seed = np.asarray(probe["entry_q14"][-1], dtype=float)
    frozen = slice(0, 7) if probe["hand"] == "right" else slice(7, 14)
    frozen_q7 = seed[frozen].copy()
    supplied = probe.get("joint_path_q14")
    path = []
    for index, pose in enumerate(probe["action"]):
        candidate = (
            kinematics.solve(pose, seed.copy(), probe["hand"])
            if supplied is None
            else supplied[index]
        )
        target = _checked_q(candidate, lower, upper)
        if not np.allclose(target[frozen], frozen_q7, atol=1e-10, rtol=0):
            raise ValueError("compiled probe trajectory changes the frozen arm")
        kinematics.check_pose(target, pose, probe["hand"])
        for hand in ("left", "right"):
            _fk(kinematics, target, hand)
        path.append(target.tolist())
        seed = target
    probe["joint_path_q14"] = path
    if probe["park_q14"][0] is None:
        probe["park_q14"][0] = seed.tolist()
    probe = _validate_probe(probe)
    for q in matrix(probe["park_q14"], "park_q14"):
        _checked_q(q, lower, upper)
        for hand in ("left", "right"):
            _fk(kinematics, q, hand)
    return probe


def preflight_probe(value, config, kinematics):
    """Check the entire compiled route without claims, SDKs or hardware I/O."""
    probe = _validate_probe(value)
    if probe["purpose"] != "simulation" and getattr(kinematics, "is_simulation", False):
        raise ValueError("synthetic kinematics cannot validate a physical probe")
    lower, upper = _limits(kinematics)
    if probe["purpose"] != "simulation":
        reference = config.get("model", {}).get("reference_positions", {})
        names = ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint")
        if any(name not in reference for name in names):
            raise ValueError("model waist reference must match probe.waist_q3")
        model_waist = vector([reference[name] for name in names], 3, "model waist reference")
        if not np.allclose(model_waist, probe["waist_q3"], atol=1e-8, rtol=0):
            raise ValueError("model waist reference must match probe.waist_q3")
    paths = [matrix(probe[key], key) for key in ("entry_q14", "joint_path_q14", "park_q14")]
    for target, pose in zip(paths[1], probe["action"], strict=False):
        kinematics.check_pose(target, pose, probe["hand"])
    workspace = config.get("workspace")
    if workspace:
        workspace_lower = vector(workspace["lower"], 3, "workspace.lower")
        workspace_upper = vector(workspace["upper"], 3, "workspace.upper")
        if np.any(workspace_lower > workspace_upper):
            raise ValueError("invalid workspace bounds")
    previous, checked = paths[0][0], 0
    for path in paths:
        for target in path:
            _checked_q(target, lower, upper)
            for q in samples(previous, target, config["motion"]):
                _checked_q(q, lower, upper)
                transform = _fk(kinematics, q, probe["hand"])
                _fk(kinematics, q, "left" if probe["hand"] == "right" else "right")
                if workspace and (
                    np.any(transform[:3, 3] < workspace_lower)
                    or np.any(transform[:3, 3] > workspace_upper)
                ):
                    raise ValueError(
                        "interpolated probe end-effector path leaves the configured workspace"
                    )
                checked += 1
            previous = target
    return {
        "job_id": probe["job_id"],
        "sha256": fingerprint(probe),
        "purpose": probe["purpose"],
        "actions": len(probe["action"]),
        "capture_indices": list(probe["capture_indices"]),
        "lifted_capture_indices": list(probe["lifted_capture_indices"]),
        "checked_joint_samples": checked,
        "physical_execution": False,
        "contact_verified": False,
        "collision_detection": False,
    }


class ProbeExecutor(Executor):
    """One persistently claimed measurement attempt; explicit close releases."""

    def __init__(self, config, backend, kinematics):
        super().__init__(config, backend, kinematics)
        self.progress = {
            "state": "created",
            "physical_execution": False,
            "completed_actions": 0,
            "completed_captures": 0,
            "captures": [],
            "parked": False,
            "contact_verified": False,
        }

    def _capture(self, probe, index, target):
        mode = "lifted" if index in probe["lifted_capture_indices"] else "contact"
        evidence = self._settle(
            target, pose=probe["action"][index], hand=probe["hand"], arrival_mode=mode
        )
        measured = vector(evidence["measured_q14"], 14, "captured measured_q14")
        evidence.update(
            action_index=index,
            timestamp_unix=time.time(),
            left_fk=_fk(self.kinematics, measured, "left").tolist(),
            right_fk=_fk(self.kinematics, measured, "right").tolist(),
            contact_verified=False,
            physical_contact_verified=False,
            requested_pose=copy.deepcopy(probe["action"][index]),
        )
        atomic_json(self.directory / "captures" / f"{index:05d}.json", evidence)
        captures = [*self.progress["captures"], index]
        self._save(captures=captures, completed_captures=len(captures))

    def run(self, value):
        if self._used or self._closed:
            raise RuntimeError(
                "an Executor runs one attempt; query its result instead of replaying"
            )
        probe = _validate_probe(value)
        simulated = getattr(self.backend, "is_simulation", False) is True
        if (probe["purpose"] == "simulation") != simulated:
            raise ValueError("simulation probes and hardware backends cannot be mixed")
        report = preflight_probe(probe, self.config, self.kinematics)
        self._used = True
        storage = self.config["storage"]
        self._lease = ControllerLease(
            [storage["controller_lock"], *storage["additional_lock_files"]]
        )
        self._lease.__enter__()
        try:
            claim_attempt(self.root_directory, probe["job_id"], report["sha256"])
        except BaseException:
            self._lease.__exit__(None, None, None)
            self._lease = None
            raise
        self.directory = (
            self.root_directory / "attempts" / hashlib.sha256(probe["job_id"].encode()).hexdigest()
        )
        self._claimed = True
        try:
            atomic_json(self.directory / "probe.json", probe)
            atomic_json(self.directory / "config.json", self.config)
            self._save(
                state="connecting",
                job_id=probe["job_id"],
                purpose=probe["purpose"],
                preflight=report,
                physical_execution=not simulated,
            )
            self.backend.connect()
            measured = vector(self.backend.read()["q14"], 14, "measured entry q14")
            start = np.asarray(probe["entry_q14"][0])
            if np.max(np.abs(measured - start)) > self.config["motion"]["arrival_tolerance_rad"]:
                raise ValueError(
                    "current arms differ from reviewed entry-route start; prepare a new route"
                )
            self._check()
            self.backend.acquire(
                np.asarray(probe["grip_q2"]),
                np.asarray(probe["waist_q3"]),
                self.kinematics.gravity(measured),
            )
            acquired = vector(self.backend.read()["q14"], 14, "acquired entry q14")
            self._prepare_entry_bridge(measured, acquired, start, probe["hand"])
            self._save(state="running", route="entry")
            previous = acquired.copy()
            for target in np.asarray(probe["entry_q14"]):
                self._move(previous, target)
                previous = target
            self._settle(previous, arrival_mode="lifted", wait_for_stationary=False)
            self._save(route="action")
            for index, target in enumerate(np.asarray(probe["joint_path_q14"])):
                self._save(action_index=index, dispatch_uncertain=True)
                self._move(previous, target)
                previous = target
                if index in probe["capture_indices"]:
                    self._capture(probe, index, target)
                self._save(completed_actions=index + 1, dispatch_uncertain=False)
            # The planner's end_clear declaration and clearance_review describe
            # this final withdrawal; no unknown contact depth is inferred here.
            evidence = self._settle(
                previous,
                pose=probe["action"][-1],
                hand=probe["hand"],
                arrival_mode="lifted",
                wait_for_stationary=False,
            )
            self._save(last_arrival=evidence)
            self._save(route="park")
            for target in np.asarray(probe["park_q14"]):
                self._move(previous, target)
                previous = target
            evidence = self._settle(previous, arrival_mode="rest")
            self._check()
            self._save(state="job_completed", job_completed=True, parked=True, final_pose=evidence)
        except BaseException as exc:
            hold_error = None
            try:
                self.backend.hold()
            except Exception as hold_exc:
                hold_error = str(hold_exc)
            try:
                health = self.backend.health()
            except Exception as health_exc:
                health = {"holding": False, "error": str(health_exc)}
            interrupted = dict(
                state="interrupted",
                parked=False,
                error=f"{type(exc).__name__}: {exc}",
                health=health,
                holding=health.get("holding") is True,
                hold_error=hold_error,
            )
            try:
                self._save(**interrupted)
            except OSError as storage_exc:
                self.progress.update(**interrupted, storage_error=str(storage_exc))
        return dict(self.progress)
