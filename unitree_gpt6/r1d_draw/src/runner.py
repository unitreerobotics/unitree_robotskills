"""Whole-job checks and execution through the robot interface.

``preflight`` is offline. ``Executor.run`` plays a compiled job batch by batch,
records progress after every batch and can resume an interrupted job at the
batch that did not finish. Nothing is replayed that was recorded as done.
"""

from __future__ import annotations

import hashlib
import time
from pathlib import Path

import numpy as np

from .job import contact_index, fingerprint, lift_floor, matrix, validate_job
from .kinematics import pose_transform, transform_pose, travel_path
from .schema import LEFT_ARM_SLOTS, RIGHT_ARM_SLOTS, vector
from .storage import ControllerLease, atomic_json, read_json
from .timing import constant_speed, rest_to_rest

ARM_SLOTS = LEFT_ARM_SLOTS + RIGHT_ARM_SLOTS
TIP_ERROR_MAX_M = 0.03  # a larger measured tip error means something else is wrong
STROKE_RAMP_M = 0.001  # a stroke reaches its speed within this distance
TRAVEL_SPEED_SHARE = 0.75  # joints stay this far below max_speed_rad_s on moves made in one go
ROUTE_POINTS = 1000  # longer batches are sent as several routes, split where the arm rests


def samples(start, target, step):
    """Evenly spaced joint samples from start (excluded) to target (included)."""
    delta = np.asarray(target) - np.asarray(start)
    count = max(1, int(np.ceil(np.max(np.abs(delta)) / step)))
    for index in range(1, count + 1):
        yield np.asarray(start) + delta * (index / count)


def preflight(job, config, kinematics):
    """Check every joint path of a compiled job. No hardware, no files."""
    job = validate_job(job)
    if job["purpose"] != "simulation" and kinematics.is_simulation:
        raise ValueError("synthetic kinematics cannot validate a physical job")
    lower, upper = vector(kinematics.lower, 14, "lower"), vector(kinematics.upper, 14, "upper")
    paths = [matrix(job["entry_q14"], "entry")]
    for batch in job["batches"]:
        if batch["travel_q14"]:
            paths.append(matrix(batch["travel_q14"], "travel"))
        path = matrix(batch["joint_path_q14"], "batch")
        for q, target_pose in zip(path, batch["action"], strict=True):
            kinematics.check_pose(q, target_pose, job["hand"])
        paths.append(path)
    paths.append(matrix(job["park_q14"], "park"))
    workspace = config.get("workspace")
    previous, checked = paths[0][0], 0
    for path in paths:
        for target in path:
            if np.any(target < lower) or np.any(target > upper):
                raise ValueError("joint path exceeds model limits")
            for q in samples(previous, target, config["motion"]["check_step_rad"]):
                tip = kinematics.fk(q, job["hand"])[:3, 3]
                if not np.isfinite(tip).all():
                    raise ValueError("nonfinite FK during path checking")
                if workspace and (
                    np.any(tip < workspace["lower"]) or np.any(tip > workspace["upper"])
                ):
                    raise ValueError("interpolated brush-tip path leaves the configured workspace")
                checked += 1
            previous = target
    return {
        "job_id": job["job_id"],
        "sha256": fingerprint(job),
        "batches": len(job["batches"]),
        "strokes": sum(len(batch["strokes"]) for batch in job["batches"]),
        "checked_joint_samples": checked,
        "physical_execution": False,
        "collision_detection": False,
    }


class Executor:
    """Runs one job on a robot object that offers state/take_over/move/stop.

    ``tip_correction``: after reaching the first (lifted) point of a batch the
    measured brush-tip offset is subtracted from the remaining targets of that
    batch, so a steady servo error does not shift the strokes.
    """

    def __init__(self, config, robot, kinematics, *, tip_correction=True):
        self.config, self.robot, self.kinematics = config, robot, kinematics
        self.tip_correction = tip_correction
        self.root = Path(config["storage"]["directory"])
        self.directory = None
        self.progress = {}

    # ---- progress ---------------------------------------------------------------------
    def _save(self, **updates):
        self.progress.update(updates, updated_unix=time.time())
        atomic_json(self.directory / "journal.json", self.progress)
        atomic_json(self.root / "status.json", self.progress)

    # ---- routes -----------------------------------------------------------------------
    def _measured(self):
        state = self.robot.state()
        return np.asarray(state["arms"]["left"] + state["arms"]["right"], dtype=float), state

    def _timed(self, kind, start, targets, hand):
        """[(joint target, duration, profile)] for one piece of a batch; it ends at rest."""
        motion = self.config["motion"]
        if kind == "contact":  # touching down, leaving, dipping: one slow move per target
            return rest_to_rest(start, targets, motion["contact_speed_rad_s"])
        tips = [self.kinematics.fk(q, hand, check_limits=False)[:3, 3] for q in [start, *targets]]
        if kind == "stroke":
            return constant_speed(
                start,
                targets,
                tips,
                tip_speed=motion["stroke_speed_m_s"],
                joint_speed=motion["contact_speed_rad_s"],
                ramp_m=STROKE_RAMP_M,
            )
        return constant_speed(
            start,
            targets,
            tips,
            tip_speed=motion["travel_speed_m_s"],
            joint_speed=motion["max_speed_rad_s"] * TRAVEL_SPEED_SHARE,
        )

    def _send(self, timed, dwell_s=0.0):
        """Send timed targets as routes; the dwell applies to the last one."""
        points = []
        for target, duration, profile in timed:
            point = {
                "q": {str(slot): float(v) for slot, v in zip(ARM_SLOTS, target, strict=True)},
                "duration_s": float(duration),
            }
            if profile != "quintic":
                point["profile"] = profile
            points.append(point)
        if dwell_s:
            points[-1]["dwell_s"] = float(dwell_s)
        return points

    def _move(self, points):
        result, route = None, []
        for number, point in enumerate(points):
            route.append(point)
            resting = point.get("profile", "quintic") in ("quintic", "ease_out")
            if number == len(points) - 1 or (len(route) >= ROUTE_POINTS and resting):
                result = self.robot.move({"points": route})
                route = []
        return result

    def _joint_move(self, start, targets, speed):
        targets = np.atleast_2d(np.asarray(targets, dtype=float))
        self._move(self._send(rest_to_rest(start, targets, speed)))
        return targets[-1]

    # ---- one batch --------------------------------------------------------------------
    @staticmethod
    def _pieces(batch):
        """[(kind, first index, last index, dwell)] covering the batch after its first pose."""
        last, pieces, cursor = len(batch["action"]) - 1, [], 1

        def add(kind, first, final, dwell=0.0):
            if first <= final:
                pieces.append((kind, first, final, dwell))

        if batch["phase"] == "PAINT":
            for stroke in batch["strokes"]:
                add("travel", cursor, stroke["start"] - 1)
                add("contact", stroke["start"], stroke["start"])  # touch down
                add("stroke", stroke["start"] + 1, stroke["end"])
                add("contact", stroke["end"] + 1, stroke["end"] + 1)  # lift off
                cursor = stroke["end"] + 2
        else:
            index = contact_index(batch)
            add("travel", cursor, index - 1)
            add("contact", index, index, batch["dwell_s"])
            add("contact", index + 1, index + 1)  # leave the material slowly
            cursor = index + 2
        add("travel", cursor, last)
        return pieces

    def _corrected(self, batch, hand, measured, first):
        """Joint path of a batch after subtracting the measured tip offset; plus the offset."""
        path = np.asarray(batch["joint_path_q14"], dtype=float)
        offset = self.kinematics.fk(measured, hand, check_limits=False)[:3, 3]
        offset = offset - self.kinematics.fk(first, hand)[:3, 3]
        if np.linalg.norm(offset) > TIP_ERROR_MAX_M:
            raise RuntimeError(f"measured tip offset {np.linalg.norm(offset):.4f} m is implausible")
        if not self.tip_correction or len(path) < 2:
            return path, offset
        seed, corrected = first.copy(), [first]
        for target_pose in batch["action"][1:]:
            shifted = pose_transform(target_pose)
            shifted[:3, 3] -= offset
            seed = self.kinematics.solve(transform_pose(shifted), seed, hand)
            corrected.append(seed)
        return np.asarray(corrected), offset

    def _run_batch(self, job, number, previous, *, replan_travel=False):
        batch, hand = job["batches"][number], job["hand"]
        first = np.asarray(batch["joint_path_q14"][0], dtype=float)
        # Straight through the air to the point above the batch, then measure where the tip is.
        travel = (
            travel_path(self.kinematics, previous, batch["action"][0], hand)
            if replan_travel
            else batch["travel_q14"]
        )
        self._move(self._send(self._timed("travel", previous, [*travel, first], hand)))
        measured, _ = self._measured()
        path, offset = self._corrected(batch, hand, measured, first)
        points, previous = [], first
        for kind, begin, end, dwell in self._pieces(batch):
            targets = path[begin : end + 1]
            points += self._send(self._timed(kind, previous, targets, hand), dwell)
            previous = targets[-1]
        evidence = {
            "batch": batch["id"],
            "phase": batch["phase"],
            "tip_offset_m": offset.tolist(),
            "tip_correction": bool(self.tip_correction),
        }
        if points:
            evidence["end_error_rad"] = self._move(points).get("end_error_rad")
        atomic_json(self.directory / "batches" / f"{number:05d}.json", evidence)
        return previous

    # ---- start and resume -------------------------------------------------------------
    def _check_fixed_joints(self, job, state):
        motion = self.config["motion"]
        if abs(state["waist_yaw"] - job["waist_yaw"]) > motion["waist_tolerance_rad"]:
            raise RuntimeError(
                f"waist yaw is {state['waist_yaw']:.3f} rad, the job was built for "
                f"{job['waist_yaw']:.3f}"
            )
        if np.max(np.abs(np.asarray(state["grip"]) - job["grip_q2"])) > motion["grip_tolerance"]:
            raise RuntimeError("the grippers differ from the job's grip_q2")

    def _lift(self, job, measured):
        """Raise the brush straight up to the transfer height from wherever it is."""
        hand, height = job["hand"], job["planning"]["transfer_z_m"]
        tip = self.kinematics.fk(measured, hand, check_limits=False)
        if tip[2, 3] >= height:
            return measured
        tip[2, 3] = height
        seed = np.clip(measured, self.kinematics.lower, self.kinematics.upper)
        target = self.kinematics.solve(transform_pose(tip), seed, hand)
        return self._joint_move(measured, target, self.config["motion"]["contact_speed_rad_s"])

    def run(self, value, *, resume=False, lift=False):
        """Run or resume a job. Returns the progress record; never releases the robot."""
        job = validate_job(value)
        if (job["purpose"] == "simulation") != bool(getattr(self.robot, "is_simulation", False)):
            raise ValueError("simulation jobs and real robots cannot be mixed")
        report = preflight(job, self.config, self.kinematics)
        key = hashlib.sha256(job["job_id"].encode()).hexdigest()
        self.directory = self.root / "attempts" / key
        journal = self.directory / "journal.json"
        if journal.exists():
            existing = read_json(journal)
            if not resume:
                raise RuntimeError("this job_id was already started; resume it or use a new id")
            if existing["sha256"] != report["sha256"]:
                raise RuntimeError("the job changed since it was started; it cannot be resumed")
            if existing["state"] == "job_completed":
                return existing
            self.progress = existing
        elif resume:
            raise RuntimeError("nothing to resume: this job_id was never started")
        else:
            self.progress = {
                "job_id": job["job_id"],
                "sha256": report["sha256"],
                "completed_batches": 0,
                "completed_strokes": 0,
                "parked": False,
            }
        lease = ControllerLease([self.config["storage"]["controller_lock"]])
        lease.__enter__()
        try:
            if not resume:
                atomic_json(self.directory / "job.json", job)
            self._save(state="connecting", error=None, preflight=report)
            self._check_fixed_joints(job, self.robot.state())
            self.robot.take_over()
            measured, _ = self._measured()
            done = self.progress["completed_batches"]
            if resume:
                if lift:
                    measured = self._lift(job, measured)
                if done < len(job["batches"]):
                    floor = lift_floor(job["batches"][done])
                    height = self.kinematics.fk(measured, job["hand"], check_limits=False)[2, 3]
                    if height < floor - 0.002:
                        raise RuntimeError(
                            f"the brush tip is at z={height:.4f} m, below the lift height "
                            f"{floor:.4f} m of the interrupted batch; resume with lift"
                        )
                previous = measured
            else:
                tolerance = self.config["motion"]["start_tolerance_rad"]
                if np.max(np.abs(measured - np.asarray(job["entry_q14"][0]))) > tolerance:
                    raise RuntimeError("the arms are not at the first entry point of the job")
                self._save(state="running", route="entry")
                previous = self._joint_move(
                    measured, job["entry_q14"], self.config["motion"]["posture_speed_rad_s"]
                )
            for number in range(done, len(job["batches"])):
                batch = job["batches"][number]
                self._save(
                    state="running",
                    route="batch",
                    batch_index=number,
                    batch_id=batch["id"],
                    phase=batch["phase"],
                    color=batch["color"],
                )
                previous = self._run_batch(
                    job, number, previous, replan_travel=resume and number == done
                )
                self._save(
                    completed_batches=number + 1,
                    completed_strokes=self.progress["completed_strokes"] + len(batch["strokes"]),
                )
            self._save(route="park")
            self._joint_move(
                previous, job["park_q14"], self.config["motion"]["posture_speed_rad_s"]
            )
            self._save(state="job_completed", job_completed=True, parked=True)
        except BaseException as exc:
            stop_error = None
            if isinstance(exc, KeyboardInterrupt):
                try:
                    self.robot.stop()
                except Exception as stop_exc:
                    stop_error = str(stop_exc)
            interrupted = dict(
                state="interrupted",
                parked=False,
                stop_error=stop_error,
                error=f"{type(exc).__name__}: {exc}",
            )
            try:
                self._save(**interrupted)
            except OSError as storage_exc:
                self.progress.update(**interrupted, storage_error=str(storage_exc))
        finally:
            lease.__exit__(None, None, None)
        return dict(self.progress)
