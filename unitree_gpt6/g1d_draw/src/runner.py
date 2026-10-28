"""Whole-job preflight and playback; SDK ownership remains until explicit close."""

from __future__ import annotations

import hashlib
import threading
import time
from pathlib import Path

import numpy as np

from .config import vector
from .job import fingerprint, matrix, validate_job
from .storage import ControllerLease, atomic_json, claim_attempt


def samples(start, target, motion):
    """Bound every published waypoint, including transfers between batches."""
    delta = np.asarray(target) - np.asarray(start)
    step = min(motion["max_step_rad"], motion["max_speed_rad_s"] / motion["command_hz"])
    count = max(1, int(np.ceil(np.max(np.abs(delta)) / step)))
    for index in range(1, count + 1):
        yield np.asarray(start) + delta * (index / count)


def preflight(job, config, kinematics):
    """No SDK imports, file claims or hardware activity."""
    job = validate_job(job)
    if job.get("purpose", "painting") != "simulation" and getattr(
        kinematics, "is_simulation", False
    ):
        raise ValueError("synthetic kinematics cannot validate a physical job")
    lower = vector(kinematics.lower, 14, "model lower limits")
    upper = vector(kinematics.upper, 14, "model upper limits")
    if np.any(lower >= upper):
        raise ValueError("invalid model joint limits")
    model = config.get("model", {})
    reference = model.get("reference_positions", {})
    if job.get("purpose", "painting") != "simulation":
        for name, value in zip(
            ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"),
            job["waist_q3"],
            strict=False,
        ):
            if name not in reference or abs(reference[name] - value) > 1e-8:
                raise ValueError("model waist reference must match job.waist_q3")
    paths = [matrix(job["entry_q14"], "entry")]
    for batch in job["batches"]:
        path = matrix(batch["joint_path_q14"], "batch")
        for q, pose in zip(path, batch["action"], strict=False):
            kinematics.check_pose(q, pose, job["hand"])
        paths.append(path)
    paths.append(matrix(job["park_q14"], "park"))
    previous = paths[0][0]
    checked = 0
    workspace = config.get("workspace")
    for path in paths:
        for target in path:
            if np.any(target < lower) or np.any(target > upper):
                raise ValueError("joint path exceeds model limits")
            for q in samples(previous, target, config["motion"]):
                transform = kinematics.fk(q, job["hand"])
                if not np.isfinite(transform).all():
                    raise ValueError("nonfinite FK during path checking")
                if workspace:
                    if np.any(
                        transform[:3, 3] < vector(workspace["lower"], 3, "workspace.lower")
                    ) or np.any(
                        transform[:3, 3] > vector(workspace["upper"], 3, "workspace.upper")
                    ):
                        raise ValueError(
                            "interpolated end-effector path leaves the configured workspace"
                        )
                checked += 1
            previous = target
    return {
        "job_id": job["job_id"],
        "sha256": fingerprint(job),
        "batches": len(job["batches"]),
        "strokes": sum(len(batch["strokes"]) for batch in job["batches"]),
        "reload_every": job["reload_every"],
        "checked_joint_samples": checked,
        "physical_execution": False,
        "collision_detection": False,
    }


class Executor:
    """Backend contract: connect/read/acquire/command/hold/health/close.

    ``run`` returns with the publisher holding, on completion or interruption.
    The caller explicitly calls ``close`` to release it; close never parks.
    No failed job is automatically resumed or replayed.
    """

    def __init__(self, config, backend, kinematics):
        self.config, self.backend, self.kinematics = config, backend, kinematics
        self.stop_event = threading.Event()
        self._lease = None
        self._used = False
        self._closed = False
        self._claimed = False
        self._progress_lock = threading.RLock()
        self.directory = Path(config["storage"]["directory"])
        self.root_directory = self.directory
        self.progress = {
            "state": "created",
            "physical_execution": False,
            "completed_batches": 0,
            "completed_strokes": 0,
            "parked": False,
        }

    def _save(self, **updates):
        with self._progress_lock:
            self.progress.update(updates, updated_unix=time.time())
            if not self._claimed:
                return
            atomic_json(self.directory / "status.json", self.progress)
            if self.directory != self.root_directory:
                atomic_json(self.root_directory / "status.json", self.progress)

    def stop(self):
        self.stop_event.set()
        try:
            self.backend.hold()
        finally:
            try:
                self._save(stop_requested=True)
            except OSError as exc:
                self.progress.update(stop_requested=True, storage_error=str(exc))

    def _check(self):
        if self.stop_event.is_set():
            raise InterruptedError("job stopped; remaining motion discarded")
        health = self.backend.health()
        if health.get("fault"):
            raise RuntimeError(f"device fault: {health['fault']}")
        if health.get("stopped"):
            raise InterruptedError("backend stop is latched")

    def _move(self, start, target):
        for q in samples(start, target, self.config["motion"]):
            self._check()
            tau = vector(self.kinematics.gravity(q), 14, "gravity compensation")
            self.backend.command(q, tau)
            if self.stop_event.wait(1 / self.config["motion"]["command_hz"]):
                self._check()
        # Do not replace a corner/material waypoint that was never published.
        deadline = time.monotonic() + self.config["motion"]["settle_timeout_s"]
        while not self.backend.health().get("published_target"):
            self._check()
            if time.monotonic() >= deadline:
                raise RuntimeError("command target publication was not confirmed")
            self.stop_event.wait(min(0.01, 1 / self.config["motion"]["publish_hz"]))

    def _pose_arrival(self, q, pose, hand, *, arrival_mode="contact", minimum_z=None):
        """Check contact accuracy; lifted target errors are evidence only."""
        from .kinematics import _pose_transform

        if arrival_mode not in {"contact", "lifted"}:
            raise ValueError("arrival_mode must be contact or lifted")
        motion = self.config["motion"]
        desired = _pose_transform(pose)
        actual = np.asarray(self.kinematics.fk(q, hand), dtype=float)
        if actual.shape != (4, 4) or not np.isfinite(actual).all():
            raise ValueError("nonfinite or invalid FK during arrival checking")
        delta = actual[:3, 3] - desired[:3, 3]
        rotation = desired[:3, :3].T @ actual[:3, :3]
        angle = float(np.arccos(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)))
        errors = {
            "position_error_xyz_m": delta.tolist(),
            "position_error_m": float(np.linalg.norm(delta)),
            "xy_error_m": float(np.linalg.norm(delta[:2])),
            "z_error_m": float(abs(delta[2])),
            "orientation_error_rad": angle,
            "actual_z_m": float(actual[2, 3]),
        }
        limits = {}
        if arrival_mode == "lifted":
            pose_ok = True
        else:
            limits["position_error_m"] = motion["contact_position_tolerance_m"]
            limits["orientation_error_rad"] = motion["runtime_orientation_tolerance_rad"]
            pose_ok = (
                errors["position_error_m"] <= limits["position_error_m"]
                and angle <= limits["orientation_error_rad"]
            )
        if minimum_z is not None:
            limits["minimum_z_m"] = float(minimum_z)
        clearance_ok = minimum_z is None or errors["actual_z_m"] >= minimum_z
        arrived = pose_ok and clearance_ok
        return arrived, errors, limits

    def _arrival_status(
        self,
        measured,
        target,
        *,
        pose=None,
        hand=None,
        arrival_mode="contact",
        minimum_z=None,
        wait_for_stationary=True,
    ):
        """One arrival policy shared by motion and evidence capture callers."""
        if arrival_mode not in {"contact", "lifted", "rest"}:
            raise ValueError("arrival_mode must be contact, lifted or rest")
        if arrival_mode == "rest" and (pose is not None or minimum_z is not None):
            raise ValueError(
                "rest arrival uses a joint target without a Cartesian pose or clearance"
            )
        motion = self.config["motion"]
        q = vector(measured["q14"], 14, "measured arrival q14")
        mode = arrival_mode if pose is not None or arrival_mode in {"lifted", "rest"} else "joints"
        errors = {
            "joint_error_rad": float(np.max(np.abs(q - target))),
            "speed_rad_s": float(np.max(np.abs(vector(measured["dq14"], 14, "arm velocity")))),
            "waist_speed_rad_s": float(
                np.max(np.abs(vector(measured.get("waist_dq3", np.zeros(3)), 3, "waist velocity")))
            ),
            "gripper_speed_units_s": float(
                np.max(np.abs(vector(measured.get("grip_dq2", np.zeros(2)), 2, "gripper velocity")))
            ),
        }
        limits = {}
        arrived = True
        if mode not in {"lifted", "rest"}:
            limits["joint_error_rad"] = motion["arrival_tolerance_rad"]
            arrived = errors["joint_error_rad"] <= limits["joint_error_rad"]
        if wait_for_stationary:
            limits["speed_rad_s"] = motion["stationary_speed_rad_s"]
            limits["waist_speed_rad_s"] = motion["stationary_speed_rad_s"]
            limits["gripper_speed_units_s"] = motion["stationary_speed_rad_s"]
            arrived = arrived and all(
                errors[key] <= limits[key]
                for key in ("speed_rad_s", "waist_speed_rad_s", "gripper_speed_units_s")
            )
        if pose is not None:
            pose_ok, pose_errors, pose_limits = self._pose_arrival(
                q, pose, hand, arrival_mode=arrival_mode, minimum_z=minimum_z
            )
            errors.update(pose_errors)
            limits.update(pose_limits)
            arrived = arrived and pose_ok
        return arrived, errors, limits

    def _settle(
        self,
        target,
        *,
        pose=None,
        hand=None,
        dwell=0.0,
        minimum_z=None,
        arrival_mode="contact",
        wait_for_stationary=True,
    ):
        """Use runtime tolerances; compilation residuals do not gate feedback.

        Captures, contact dwell and parking wait for stationary feedback. Parking
        records joint error without a separate precision gate. Lifted transfers
        only wait for healthy publication and explicit clearance. Commands stay fixed.
        """
        if arrival_mode not in {"contact", "lifted", "rest"}:
            raise ValueError("arrival_mode must be contact, lifted or rest")
        if arrival_mode == "rest" and (pose is not None or minimum_z is not None):
            raise ValueError(
                "rest arrival uses a joint target without a Cartesian pose or clearance"
            )
        motion = self.config["motion"]
        target = vector(target, 14, "settle target")
        mode = arrival_mode if pose is not None or arrival_mode in {"lifted", "rest"} else "joints"
        if not wait_for_stationary and (mode != "lifted" or dwell > 0):
            raise ValueError("only lifted transfers without dwell may skip stationary arrival")
        limits = {}
        duration = max(float(dwell), motion["settle_time_s"]) if wait_for_stationary else 0.0
        deadline = time.monotonic() + motion["settle_timeout_s"] + duration
        stationary_since = None
        last_check = {}
        while time.monotonic() < deadline:
            self._check()
            measured = self.backend.read()
            health = self.backend.health()
            q = vector(measured["q14"], 14, "measured settle q14")
            pose_ok, errors, limits = self._arrival_status(
                measured,
                target,
                pose=pose,
                hand=hand,
                arrival_mode=arrival_mode,
                minimum_z=minimum_z,
                wait_for_stationary=wait_for_stationary,
            )
            arrived = (
                health.get("published_target") is True and health.get("holding") is True and pose_ok
            )
            last_check = {
                **errors,
                "published_target": health.get("published_target"),
                "holding": health.get("holding"),
            }
            now = time.monotonic()
            stationary_since = (
                (now if stationary_since is None else stationary_since) if arrived else None
            )
            if stationary_since is not None and now - stationary_since >= duration:
                return {
                    "settled_dwell_s": now - stationary_since,
                    "measured_q14": q.tolist(),
                    "command_q14": target.tolist(),
                    "arrival_mode": mode,
                    **errors,
                    "tolerances": limits,
                    "stationary_required": wait_for_stationary,
                    "physical_contact_verified": False,
                }
            if self.stop_event.wait(min(0.02, 1 / motion["command_hz"])):
                self._check()
        raise RuntimeError(
            f"arrival not confirmed ({mode}): measured={last_check}, limits={limits}"
        )

    def _prepare_entry_bridge(self, measured, acquired, start, hand):
        """Rebase a small acquisition drift and validate its full entry bridge."""
        measured = vector(measured, 14, "pre-acquisition q14")
        acquired = vector(acquired, 14, "acquired q14")
        start = vector(start, 14, "reviewed entry q14")
        delta = float(np.max(np.abs(acquired - measured)))
        limit = self.config["motion"]["tracking_tolerance_rad"]
        if delta > limit:
            raise RuntimeError(
                f"arms moved beyond tracking tolerance during acquisition: {delta:.6f} > {limit}"
            )
        workspace = self.config.get("workspace")
        checked = 0

        def check(q):
            nonlocal checked
            if np.any(q < self.kinematics.lower) or np.any(q > self.kinematics.upper):
                raise ValueError("live entry bridge exceeds model limits")
            for side in ("left", "right"):
                tf = np.asarray(self.kinematics.fk(q, side), dtype=float)
                if tf.shape != (4, 4) or not np.isfinite(tf).all():
                    raise ValueError("nonfinite or invalid FK during live entry bridge checking")
                if (
                    side == hand
                    and workspace
                    and (
                        np.any(tf[:3, 3] < np.asarray(workspace["lower"]))
                        or np.any(tf[:3, 3] > np.asarray(workspace["upper"]))
                    )
                ):
                    raise ValueError("live entry bridge leaves reviewed workspace")
            checked += 1

        check(acquired)
        for q in samples(acquired, start, self.config["motion"]):
            self._check()
            check(q)
        self._save(
            acquisition={
                "delta_rad": delta,
                "limit_rad": limit,
                "bridge_samples": checked,
                "measured_q14": measured.tolist(),
                "acquired_q14": acquired.tolist(),
            }
        )

    def run(self, value):
        from .job import contact_index, lift_floor

        if self._used or self._closed:
            raise RuntimeError(
                "an Executor runs one attempt; query its result instead of replaying"
            )
        job = validate_job(value)
        simulated = getattr(self.backend, "is_simulation", False) is True
        if (job.get("purpose", "painting") == "simulation") != simulated:
            raise ValueError("simulation jobs and hardware backends cannot be mixed")
        report = preflight(job, self.config, self.kinematics)
        self._used = True
        storage = self.config["storage"]
        self._lease = ControllerLease(
            [storage["controller_lock"], *storage["additional_lock_files"]]
        )
        self._lease.__enter__()
        try:
            claim_attempt(self.root_directory, job["job_id"], report["sha256"])
        except BaseException:
            self._lease.__exit__(None, None, None)
            self._lease = None
            raise
        self.directory = (
            self.root_directory / "attempts" / hashlib.sha256(job["job_id"].encode()).hexdigest()
        )
        self._claimed = True
        try:
            atomic_json(self.directory / "job.json", job)
            atomic_json(self.directory / "config.json", self.config)
            self._save(
                state="connecting",
                job_id=job["job_id"],
                preflight=report,
                physical_execution=not simulated,
            )
            self.backend.connect()
            measured = self.backend.read()
            start = np.asarray(job["entry_q14"][0])
            if (
                np.max(np.abs(measured["q14"] - start))
                > self.config["motion"]["arrival_tolerance_rad"]
            ):
                raise ValueError(
                    "current arms differ from reviewed entry-route start; prepare a new route"
                )
            self._check()
            self.backend.acquire(
                np.asarray(job["grip_q2"]),
                np.asarray(job["waist_q3"]),
                self.kinematics.gravity(measured["q14"]),
            )
            acquired = self.backend.read()
            self._prepare_entry_bridge(measured["q14"], acquired["q14"], start, job["hand"])
            self._save(state="running", route="entry")
            # The first step uses current feedback once; subsequent targets
            # retain the original commands rather than accumulating servo bias.
            previous = acquired["q14"].copy()
            for target in np.asarray(job["entry_q14"]):
                self._move(previous, target)
                previous = target
            self._settle(previous, arrival_mode="lifted", wait_for_stationary=False)
            for batch_index, batch in enumerate(job["batches"]):
                self._save(
                    batch_index=batch_index,
                    batch_id=batch["id"],
                    phase=batch["phase"],
                    color=batch["color"],
                    route="batch",
                )
                index = contact_index(batch)
                starts = {stroke["start"] for stroke in batch["strokes"]}
                for point_index, target in enumerate(np.asarray(batch["joint_path_q14"])):
                    self._save(action_index=point_index, dispatch_uncertain=True)
                    self._move(previous, target)
                    previous = target
                    if point_index == index:
                        evidence = self._settle(
                            target,
                            pose=batch["action"][point_index],
                            hand=job["hand"],
                            dwell=batch["dwell_s"],
                        )
                        atomic_json(
                            self.directory / "contacts" / f"{batch_index:05d}.json", evidence
                        )
                    elif point_index in starts:
                        self._settle(
                            target,
                            pose=batch["action"][point_index],
                            hand=job["hand"],
                            dwell=batch.get("contact_settle_s", 0.0),
                        )
                    self._save(dispatch_uncertain=False)
                evidence = self._settle(
                    previous,
                    pose=batch["action"][-1],
                    hand=job["hand"],
                    minimum_z=lift_floor(batch),
                    arrival_mode="lifted",
                    wait_for_stationary=False,
                )
                self._save(
                    completed_batches=batch_index + 1,
                    completed_strokes=self.progress["completed_strokes"] + len(batch["strokes"]),
                    last_arrival=evidence,
                )
            self._save(route="park")
            for target in np.asarray(job["park_q14"]):
                self._move(previous, target)
                previous = target
            evidence = self._settle(previous, arrival_mode="rest")
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

    def close(self):
        """Explicit release only; never invent a retreat after an interruption."""
        if self._closed:
            return
        self._closed = True
        try:
            self.backend.close()
        finally:
            try:
                if self._claimed:
                    self._save(released=True)
            finally:
                # Publish the terminal record before another owner can acquire
                # the lease and replace the shared status with its new run.
                if self._lease is not None:
                    self._lease.__exit__(None, None, None)
