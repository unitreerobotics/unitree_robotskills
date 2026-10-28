"""Review after descent; correct wrist-observed landings off the target intersection."""
import copy
import json
import secrets
import time

import numpy as np

from g1d_chess.robot.dex1_internal import ARM, GRIP
from g1d_chess.robot.motion_harness import matrix, waypoint
from g1d_chess.robot.camera_policy import camera_selection
from g1d_chess.robot.board_guard import GuardViolation
from scipy.spatial.transform import Rotation

POSE_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


class ReleaseXYGuard:
    """Scoped planning exception for bounded, fixed-height release XY only."""
    def __init__(self, guard, origin, radius):
        self.guard, self.origin, self.radius = guard, origin, radius

    def __getattr__(self, name):
        return getattr(self.guard, name)

    def check_segment(self, start, end):
        try:
            return self.guard.check_segment(start, end)
        except GuardViolation:
            # Only the actor's bounded local neighborhood can use the exception.
            # Surface collision, changes of height/attitude and other motions
            # still fail. IK may deviate slightly from a fixed Cartesian target.
            for pose in (start, end):
                if (self.guard.clearance(pose) < self.guard.minimum_clearance
                        or np.linalg.norm(pose[:2, 3]-self.origin[:2, 3]) > self.radius+.002
                        or abs(pose[2, 3]-self.origin[2, 3]) > .002
                        or Rotation.from_matrix(self.origin[:3, :3].T @ pose[:3, :3]).magnitude() > .025):
                    raise
            if not np.isfinite(start).all() or not np.isfinite(end).all():
                raise GuardViolation("non-finite release XY segment")


def configure_release_gate(request, guard):
    """Validate/mark high -> descend/review -> open -> rise topology offline."""
    if "pre_release_adjustment" not in request and "grasp_retry" not in request:
        return None
    config = copy.deepcopy(request.get("pre_release_adjustment", {}))
    if not isinstance(config, dict):
        raise ValueError("pre_release_adjustment must be an object")
    if request.get("decision_channel", {}).get("mode") != "serial":
        raise ValueError("pre-release review requires the main-agent serial channel")
    config.setdefault("max_corrections", 3)
    config.pop("max_offset_m", None)
    config.pop("max_total_offset_m", None)
    config.setdefault("decision_timeout_s", 300.)
    config["on_correction_limit"] = "release_without_another_photo"
    config["review_pose"] = "after_descent_before_open"
    config["adjustment_policy"] = "obvious_off_target_intersection"
    config.setdefault("alignment_criterion", "actor_wrist_predicted_landing_near_target_intersection")
    if config["alignment_criterion"] not in ("actor_wrist_predicted_landing_near_target_intersection",
                                            "actor_wrist_predicted_landing_near_offboard_target",
                                            "actor_wrist_predicted_landing_inside_same_side_tray"):
        raise ValueError("invalid release alignment criterion")
    config["correction_motion"] = "direct_low_xy"
    if type(config["max_corrections"]) is not int or not 1 <= config["max_corrections"] <= 3:
        raise ValueError("release max_corrections must be 1..3")
    if (type(config["decision_timeout_s"]) not in (int, float)
            or not np.isfinite(config["decision_timeout_s"])
            or not 1 <= config["decision_timeout_s"] <= 300):
        raise ValueError("release decision timeout must be 1..300 seconds")
    plan = request["success_plan"]
    points = plan["right_arm"]
    if len(plan["left_arm"]) != 1 or plan["left_arm"][0].get("gripper", "hold") != "hold":
        raise ValueError("release gate requires a stationary observer in the success path")
    for point in points:
        point.pop("pre_release_gate", None)
    opens = [i for i, p in enumerate(points) if p.get("gripper") == "open"]
    if len(opens) != 1 or opens[0] < 2 or opens[0] + 2 >= len(points):
        raise ValueError("release gate requires one placement opening and a high return connector")
    i = opens[0]
    high, low, opened, rise = points[i-2:i+2]
    if [p.get("gripper", "hold") for p in (high, low, opened, rise)] != ["hold", "hold", "open", "hold"]:
        raise ValueError("release gate must precede a separate descent/open/rise")
    if any(abs(low[k]-opened[k]) > 1e-9 for k in POSE_KEYS):
        raise ValueError("release open must be stationary")
    if any(abs(p[k]-low[k]) > 1e-9 for p in (high, rise)
           for k in ("x", "y", "roll", "pitch", "yaw")):
        raise ValueError("release high/descent/rise must share XY and RPY")
    if high["z"] <= low["z"] or rise["z"] < high["z"]:
        raise ValueError("release gate and retreat must be above the release pose")
    if min(guard.clearance(matrix(p)) for p in (high, rise)) < guard.travel_clearance:
        raise ValueError("release gate must have transport clearance")
    connector = i+2
    if points[connector].get("gripper") == "close":
        if any(abs(points[connector][k]-rise[k]) > 1e-9 for k in POSE_KEYS):
            raise ValueError("post-release high close must be stationary")
        connector += 1
    if connector >= len(points) or points[connector].get("gripper", "hold") != "hold":
        raise ValueError("release gate needs a held high connector to the frozen return")
    if guard.clearance(matrix(points[connector])) < guard.travel_clearance:
        raise ValueError("release return connector must be at transport clearance")
    guard.check_gripper(matrix(low))
    low["pre_release_gate"] = True
    config.update(original_target={k: low[k] for k in POSE_KEYS},
                  original_high={k: high[k] for k in POSE_KEYS},
                  speed_scale=plan.get("speed_scale", 1.))
    return config


def validate_release_decision(checkpoint, decision):
    """Never accept a source decision, stale photo or a fourth correction."""
    identity = ("token", "worker_id", "checkpoint_id", "stage", "correction_index", "grasp_attempt_index")
    if not isinstance(decision, dict) or any(decision.get(k) != checkpoint[k] for k in identity):
        raise ValueError("wrong release checkpoint identity")
    if type(decision.get("correction_index")) is not int or type(decision.get("grasp_attempt_index")) is not int:
        raise ValueError("invalid release checkpoint index")
    action = decision.get("decision")
    if action not in ("success", "adjust", "failure"):
        raise ValueError("release accepts success/adjust/failure only")
    if any(k in decision for k in ("grasp_z_m", "target_z_m", "rpy_rad", "yaw")):
        raise ValueError("release review cannot change Z or RPY")
    if action != "failure" and decision.get("target_identified") is not True:
        raise ValueError("release target not identified")
    if action == "success" and decision.get("release_alignment_verified") is not True:
        raise ValueError("release at current low pose not visually verified")
    if type(decision.get("obvious_large_offset")) is not bool:
        raise ValueError("release review requires explicit obvious_large_offset judgment")
    # Keep the wire field: off-target, unclear landing or collision risk needs correction.
    if action == "success" and decision["obvious_large_offset"]:
        raise ValueError("release requiring correction cannot be approved as aligned")
    if action == "adjust":
        if not decision["obvious_large_offset"]:
            raise ValueError("release adjustment requires an explicit need for correction")
        if checkpoint["corrections_remaining"] <= 0:
            raise ValueError("release correction budget exhausted")
        offset = decision.get("offset_xy_m")
        if (not isinstance(offset, list) or len(offset) != 2
                or any(type(v) not in (int, float) or not np.isfinite(v) for v in offset)
                or not np.isfinite(np.linalg.norm(offset))
                or np.linalg.norm(offset) < 1e-6):
            raise ValueError("invalid release XY correction")
    elif decision.get("offset_xy_m", [0, 0]) != [0, 0]:
        raise ValueError("non-adjust release decision has an offset")
    return action


class ReleaseAlignmentMixin:
    def _release_checkpoint(self, bundle, reference, correction_index):
        config = bundle["pre_release_adjustment"]
        token, directory = bundle["token"], bundle["directory"]
        suffix = "_release_" + str(correction_index)
        checkpoint_path = directory / (token + suffix + "_checkpoint.json")
        decision_path = directory / (token + suffix + "_decision.json")
        receipt_path = directory / (token + suffix + "_decision_receipt.json")
        if any(p.exists() for p in (checkpoint_path, decision_path, receipt_path)):
            raise RuntimeError("release checkpoint files already exist")
        # capture() already returns the state for these new images. Do not run
        # replanning, archive searches or independent evaluations after publish.
        observation = self.capture(self._execution_cameras(bundle), self._cached_image_directory(bundle, "release_"+str(correction_index)))
        state = observation["state"]
        actual = state["right_arm"]
        self.guard.check_gripper(matrix(actual))
        created = time.time()
        checkpoint = {
            "token": token, "worker_id": bundle["decision_channel"]["worker_id"],
            "checkpoint_id": secrets.token_hex(16), "stage": "pre_release",
            "review_pose": config["review_pose"],
            "adjustment_policy": config["adjustment_policy"],
            "alignment_criterion": config["alignment_criterion"],
            "correction_motion": config["correction_motion"],
            "physical_actor": self.active_arm,
            "camera_order": self._execution_cameras(bundle),
            "camera_selection": bundle.get("camera_selection", camera_selection(self.active_arm)),
            "grasp_attempt_index": bundle.get("grasp_attempt_index", 0),
            "correction_index": correction_index,
            "corrections_remaining": config["max_corrections"]-correction_index,
            "original_target": config["original_target"],
            "target_description": config.get("target_description", "planned target"),
            "target_evidence": config.get("target_evidence", ""),
            "planned_right_arm": reference, "actual_right_arm": actual,
            "adjustment_reference_right_arm": actual,
            "cumulative_offset_xy_m": [actual[k]-config["original_target"][k] for k in ("x", "y")],
            "observation": observation, "actual_state": state,
            "created_at_unix_s": created, "decision_timeout_s": config["decision_timeout_s"],
            "deadline_unix_s": created+config["decision_timeout_s"], "wait_indefinitely": False,
            "decision_target_s": 5, "checkpoint_path": str(checkpoint_path),
            "on_correction_limit": config["on_correction_limit"],
            "decision_path": str(decision_path), "receipt_path": str(receipt_path),
        }
        self._atomic_json(checkpoint_path, checkpoint)
        self._atomic_json(directory / (token+"_checkpoint_latest.json"), checkpoint)
        deadline = time.monotonic()+config["decision_timeout_s"]
        decision, action, accepted, reason = {}, "failure", False, "release_decision_timeout"
        while time.monotonic() < deadline:
            self._cached_health()
            if self.cancelled.is_set():
                raise RuntimeError("release checkpoint cancelled")
            if decision_path.exists():
                try:
                    decision = json.loads(decision_path.read_text())
                    action = validate_release_decision(checkpoint, decision)
                    accepted, reason = True, decision.get("reason", "external_release_review")
                except (ValueError, OSError) as exc:
                    action, reason = "failure", str(exc)
                break
            time.sleep(.05)
        receipt = {k: checkpoint[k] for k in ("token", "worker_id", "checkpoint_id", "stage",
                                              "correction_index", "grasp_attempt_index")}
        receipt.update(accepted=accepted, decision=action, reason=reason,
                       received_at_unix_s=time.time(), motion_executed=False)
        self._atomic_json(receipt_path, receipt)
        self.log({"event": "release_decision_received", **receipt})
        return action, decision, checkpoint, receipt

    def _checked_release_local(self, points, seed, grip, bundle):
        """Compile only changed target motions with the normal (non-source) guard."""
        observer = waypoint(self.motion.fk(seed)[0])
        plan = {"left_arm": [observer], "right_arm": points,
                "speed_scale": bundle["pre_release_adjustment"]["speed_scale"], "skip_preflight": False}
        compiled = self._compile_cached(plan, seed, grip, bundle["gripper_open_q"])
        # Physically hold the observation arm; verify after fixing numerical drift.
        for step in compiled["steps"]:
            step["seed"][:7] = seed[:7]
            previous = step["seed"]
            for q, _ in step["samples"]:
                q[:7] = seed[:7]
                self._check_joint_segment(previous, q)
                previous = q
            errors = self.motion.errors(self.motion.fk(previous), step["targets"])
            if any(p > .005 or r > .025 for p, r in errors):
                raise ValueError("release local IK endpoint exceeds 5 mm / .025 rad")
        compiled["end_q"][:7] = seed[:7]
        return compiled

    def _release_tail(self, bundle, tail, seed, grip, actual):
        """Open at the reviewed low pose, rise vertically, rejoin frozen return."""
        config = bundle["pre_release_adjustment"]
        opened = dict(actual, gripper="open", pose_audit_release=config["original_target"])
        rise = dict(actual, z=max(actual["z"], config["original_high"]["z"]), gripper="hold")
        points = [opened, rise]
        next_index = 2
        if tail[next_index]["waypoints"][1].get("gripper") == "close":
            points.append(dict(rise, gripper="close"))
            next_index += 1
        local = self._checked_release_local(points, seed, grip, bundle)
        # A bounded joint bridge reaches exactly the prechecked high return
        # endpoint. No IK of staging/init and no accumulated target re-solving.
        frozen = tail[next_index]
        start, end = local["end_q"].copy(), frozen["samples"][-1][0].copy()
        n = max(1, int(np.ceil(np.max(np.abs(end-start))/.1)))
        duration = max(.12, 1.6*float(np.max(np.abs(end-start)))/.20/n)/config["speed_scale"]
        samples, previous = [], start
        for alpha in np.linspace(0., 1., n+1)[1:]:
            q = start+(end-start)*alpha
            self._check_joint_segment(previous, q)
            for beta in np.linspace(0., 1., 5):
                if self.guard.clearance(self.motion.fk(previous+(q-previous)*beta)[1]) < self.guard.travel_clearance:
                    raise ValueError("release return bridge falls below transport clearance")
            samples.append((q.copy(), duration))
            previous = q
        bridge = dict(frozen, seed=start, samples=samples)
        return local["steps"]+[bridge]+tail[next_index+1:]

    def _release_correction(self, bundle, actual, target, seed, grip):
        """Direct XY at the reviewed low height; never lift to correct."""
        config = bundle["pre_release_adjustment"]
        if any(abs(target[k]-actual[k]) > 1e-9 for k in ("z", "roll", "pitch", "yaw")):
            raise ValueError("release XY correction must preserve actual Z/RPY")
        radius = np.linalg.norm([target[k]-actual[k] for k in ("x", "y")])
        if not np.isfinite(radius) or radius < 1e-6:
            raise ValueError("release XY correction must be finite and nonzero")
        guard = self.guard
        guard.check_gripper(matrix(actual))
        guard.check_gripper(matrix(target))
        # This service executes requests serially. Restore the normal guard even
        # on IK failure; source, return and other transactions never inherit it.
        self.guard = ReleaseXYGuard(guard, matrix(actual), radius)
        try:
            return self._checked_release_local([target], seed, grip, bundle)
        finally:
            self.guard = guard

    def _run_placement_steps(self, steps, bundle, result, context, cursor):
        if "pre_release_adjustment" not in bundle:
            return self._run_cached_steps(steps, context, cursor)
        gates = [i for i, step in enumerate(steps) if step["waypoints"][1].get("pre_release_gate")]
        if len(gates) != 1:
            raise RuntimeError("placement tail has no unique after-descent gate")
        index = gates[0]
        completed = self._run_cached_steps(steps[:index+1], context, cursor)
        tail = steps[index+1:]
        config = bundle["pre_release_adjustment"]
        reference = {k: steps[index]["waypoints"][1][k] for k in POSE_KEYS}
        result["release_alignment"] = {"checks": [], "corrections": []}
        context["branch"] = "release_alignment"
        for count in range(config["max_corrections"]):
            feedback = self._cached_health()
            self.guard.check_gripper(self.motion.fk(feedback[0][ARM])[1])
            action, decision, checkpoint, receipt = self._release_checkpoint(bundle, reference, count)
            result["release_alignment"]["checks"].append(receipt)
            if action == "failure":
                raise RuntimeError("release alignment refused; holding piece: "+receipt["reason"])
            state = checkpoint["actual_state"]
            seed = np.asarray(state["arm_q"], dtype=float)
            grip = np.array([state["grippers"][side]["q"] for side in ("left", "right")])
            # The photo must still describe the held configuration. No wait/retry.
            fresh_feedback = self._verify_cached_start(seed, grip)
            actual = {k: checkpoint["actual_right_arm"][k] for k in POSE_KEYS}
            if action == "success":
                release_actual = waypoint(self.motion.fk(fresh_feedback[0][ARM])[1])
                completed.extend(self._approved_release(bundle, tail, result, context,
                                                        fresh_feedback, release_actual, reference,
                                                        "visual_success"))
                return completed
            reference = dict(actual, gripper="hold")
            for axis, delta in zip(("x", "y"), decision["offset_xy_m"]):
                reference[axis] += float(delta)
            correction = self._release_correction(bundle, actual, reference, seed, grip)
            self._run_cached_steps(correction["steps"], context, {})
            result["release_alignment"]["corrections"].append({
                "correction_index": count+1, "offset_xy_m": decision["offset_xy_m"],
                "target": copy.deepcopy(reference)})
            # First/second adjustment: immediately recapture. Third: release.
        feedback = self._cached_health()
        actual = waypoint(self.motion.fk(feedback[0][ARM])[1])
        completed.extend(self._approved_release(bundle, tail, result, context, feedback,
                                                actual, reference, "xy_budget_exhausted_auto_release"))
        return completed

    def _approved_release(self, bundle, tail, result, context, feedback, actual, reference, reason):
        config = bundle["pre_release_adjustment"]
        actual = {k: actual[k] for k in POSE_KEYS}
        self.guard.check_gripper(matrix(actual))
        audit = getattr(self, "_pose_audit", None)
        if audit is not None:
            audit.capture("release_start", feedback, self.dds,
                          effective_target=reference, original_target=config["original_target"],
                          grasp_attempt_index=bundle.get("grasp_attempt_index", 0),
                          feedback_read_monotonic_s=time.monotonic(),
                          event_stage="target_low_release_approval", approval_reason=reason,
                          placement_target=actual)
        context["release_approved_before_open"] = True
        context["require_release_open_before_continuation"] = bundle.get(
            "require_release_open_before_continuation", False)
        result["release_alignment"]["approval_reason"] = reason
        adjusted = self._release_tail(bundle, tail, feedback[0][ARM].copy(),
                                      feedback[0][GRIP].copy(), actual)
        context["branch"] = "success"
        result["release_alignment"]["effective_target"] = {
            k: adjusted[0]["waypoints"][1][k] for k in POSE_KEYS}
        return self._run_cached_steps(adjusted, context, {})
