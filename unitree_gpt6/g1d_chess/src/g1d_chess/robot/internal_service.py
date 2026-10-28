"""Persistent, initially read-only G1/dex1_internal control and observation service."""
import argparse
import copy
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import secrets
import signal
import sys
import threading
import time

import cv2
import numpy as np
import zmq

from g1d_chess.robot.dex1_internal import InternalDDS, ARM, GRIP, WAIST
from g1d_chess.robot.motion_harness import MotionHarness, matrix, waypoint
from g1d_chess.robot.board_guard import BoardGuard, GuardViolation, LOW_ROTATION_DRIFT_TOLERANCE_RAD
from g1d_chess.robot.grasp_vision import prepare_vision, evaluate_image
from g1d_chess.robot.async_event_log import AsyncEventLog
from g1d_chess.robot.fast_correction import RuntimeSourcePaths
from g1d_chess.robot.pose_audit import PoseAudit
from g1d_chess.robot.release_alignment import ReleaseAlignmentMixin, configure_release_gate
from g1d_chess.robot.camera_policy import AVAILABLE_CAMERAS, planning_cameras, camera_selection, execution_cameras

CAMERAS = AVAILABLE_CAMERAS
from g1d_chess.paths import ROOT
WAYPOINT_POSITION_TOLERANCE_M = .010


class CachedHardwareFault(RuntimeError):
    """A hardware status fault that must never trigger autonomous recovery."""


class RobotService(ReleaseAlignmentMixin):
    active_arm = "right"

    def __init__(self, interface, camera_host=None, log_dir=None, gripper_open_sign=1):
        self.motion = MotionHarness()
        self.dds = InternalDDS(interface, gripper_open_sign=gripper_open_sign)
        self.camera_host = camera_host
        self.camera = None
        self.log_dir = Path(log_dir or ROOT / "runs" / datetime.now().strftime("%Y%m%d_%H%M%S_harness"))
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = self.log_dir / "events.jsonl"
        self.event_log = AsyncEventLog(self.log_path)
        self.cancelled = threading.Event()
        self.guard = None
        self.cached_plans = {}
        self.cached_sequences = {}

    def state(self):
        q,dq,msg,age = self.dds.state()
        poses = self.motion.fk(q[ARM])
        with self.dds.lock:
            enabled = self.dds.enabled
            waist_enabled = getattr(self.dds,"waist_enabled",False)
            commands = {key:getattr(self.dds,key).copy().tolist() if enabled else None
                        for key in ("arm_target", "arm_sent", "tau")}
        return {"timestamp":datetime.now().isoformat(), "state_age_s":age,
                "active_arm":self.active_arm, "camera_selection":camera_selection(self.active_arm),
                "tick":int(msg.tick), "mode_machine":int(msg.mode_machine),
                "control_enabled":enabled, "fault":self.dds.fault,
                "fault_hold_active":getattr(self.dds,"fault_hold_active",False),
                "fault_hold_publish_error":getattr(self.dds,"fault_hold_publish_error",None),
                "fault_hold_last_published_at":getattr(self.dds,"fault_hold_last_published_at",None),
                "waist_hold_enabled":waist_enabled,
                "waist_target":self.dds.waist_target.tolist() if waist_enabled else None,
                "waist_sent":self.dds.waist_sent.tolist() if waist_enabled else None,
                "waist_zero_verified":bool(waist_enabled and np.max(np.abs(q[WAIST]))<=.01
                                            and np.max(np.abs(dq[WAIST]))<=.03),
                "published_commands":self.dds.published,
                "audit_log":self.event_log.status() if hasattr(self,"event_log") else None,
                **commands,
                "gripper_open_sign":self.dds.gripper_open_sign,
                "grasp_contact_hold_supported":True,
                "grasp_descent_actual_xy_rpy_locked":True,
                "grasp_z_selection_supported":True,
                "preclose_visual_gate_supported":True,
                "source_tool_envelope_override_supported":True,
                "cached_stationary_gripper_preserves_arm_commands":True,
                "post_release_actual_vertical_departure":True,
                "vertical_departure_clearance_before_connector":True,
                "waypoint_position_tolerance_m":WAYPOINT_POSITION_TOLERANCE_M,
                "runtime_error_checks_enabled":False,
                "runtime_low_height_motion_rejection_enabled":False,
                "runtime_surface_collision_checks_enabled":True,
                "runtime_joint_branch_rejection_enabled":False,
                "runtime_gripper_arrival_rejection_enabled":False,
                "runtime_correction_preflight_enabled":False,
                "automatic_fault_release_enabled":False,
                "low_rotation_drift_tolerance_rad":LOW_ROTATION_DRIFT_TOLERANCE_RAD,
                "coordinate_frame":"arm model with locked zero waist; x forward, y left, z up",
                "controlled_motors":ARM + GRIP + (WAIST if waist_enabled else []),
                "motor_diagnostics":{
                    str(i):{"temperature_raw":list(getattr(msg.motor_state[i],"temperature",[])),
                            "motorstate":int(msg.motor_state[i].motorstate),
                            "tau_est":float(msg.motor_state[i].tau_est)}
                    for i in ARM + GRIP + (WAIST if waist_enabled else [])},
                "scene_guard":vars(self.guard) if self.guard is not None else None,
                "arm_q":q[ARM].tolist(), "arm_dq":dq[ARM].tolist(),
                "full_q":q.tolist(), "waist_q":q[12:15].tolist(),
                "imu_rpy":list(msg.imu_state.rpy),
                "left_arm":waypoint(poses[0]), "right_arm":waypoint(poses[1]),
                "grippers":{side:{"motor_index":i,"q":float(q[i]),"dq":float(dq[i]),
                                  "tau_est":float(msg.motor_state[i].tau_est),
                                  "motorstate":int(msg.motor_state[i].motorstate)}
                            for side,i in zip(("left","right"),GRIP)}}

    def log(self, event):
        self.event_log.append(event)

    def capture(self, cameras=None, output_dir=None):
        cameras = list(planning_cameras() if cameras is None else cameras)
        if not cameras or not set(cameras).issubset(CAMERAS):
            raise ValueError("unsupported camera; use cam_left_high, cam_left_wrist or cam_right_wrist")
        if self.camera is None:
            from g1d_chess.cameras.imageclient import ImageClientCamera
            self.camera = ImageClientCamera(self.camera_host)
            self.camera.connect()
        # Only subscribe to selected streams; wait for frames to refresh.
        self.camera.async_read(cameras)
        time.sleep(.2)
        deadline = time.monotonic()+5
        while time.monotonic()<deadline:
            images = self.camera.async_read(cameras)
            if all(images.get(key) is not None for key in cameras):
                break
            time.sleep(.05)
        else:
            raise RuntimeError("missing camera frame")
        ts=datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        paths={}
        root=Path(output_dir or ROOT / "images").resolve()
        for key in cameras:
            path=root / ("observation.images."+key) / (ts+".jpg")
            path.parent.mkdir(parents=True,exist_ok=True)
            if not cv2.imwrite(str(path),images[key]):
                raise RuntimeError("image write failed")
            paths[key]=str(path)
        result={"timestamp":ts,"paths":paths,"state":self.state(),
                "camera_order":cameras,"physical_actor":self.active_arm}
        self.log({"event":"observation", **result})
        return result

    def _execution_cameras(self, bundle):
        return execution_cameras(self.active_arm, bundle.get("camera_selection"))

    def enable(self):
        q,_,_,_=self.dds.state()
        self.cancelled.clear()
        self.dds.enable()
        self.dds.command_arm(q[ARM],self.motion.tau(q[ARM]))
        time.sleep(.3)
        return self.state()

    def prepare(self,plan):
        q,_,_,_=self.dds.state()
        if self.guard is None:
            raise ValueError("set_scene required before motion")
        return self.motion.prepare(plan,q[ARM], self.guard)

    def _check_feedback(self, origin, wanted, actual, max_error, feedback, context):
        """Preserve the failing observation before execute() replaces targets in hold()."""
        try:
            self.guard.check_feedback(origin,wanted,actual,max_error=max_error)
        except Exception as exc:
            try:
                q,dq,msg,age = feedback
                with self.dds.lock:
                    commands = {key:getattr(self.dds,key).copy().tolist()
                                for key in ("arm_target", "arm_sent", "tau")}
                    published = self.dds.published
                elapsed = time.monotonic()-context["execution_start"]
                count = published-context["published_start"]
                diagnostic = {key:value for key,value in context.items()
                              if key not in ("execution_start", "published_start")}
                diagnostic.update(
                    event="motion_fault", error=str(exc), **commands,
                    elapsed_s=elapsed, published_commands=published,
                    published_since_start=count,
                    mean_publish_hz=count/elapsed if elapsed>0 else None,
                    state_age_s=age, tick=int(msg.tick),
                    arm_q=q[ARM].tolist(), arm_dq=dq[ARM].tolist(),
                    origin=origin.tolist(), desired=wanted.tolist(), measured=actual.tolist(),
                    position_error_xyz_m=(actual[:3,3]-wanted[:3,3]).tolist(),
                    position_error_m=float(np.linalg.norm(actual[:3,3]-wanted[:3,3])),
                    max_position_error_m=max_error,
                    lateral_drift_m=float(np.linalg.norm(actual[:2,3]-origin[:2,3])),
                    rotation_drift_rad=self.motion.errors([actual],[origin])[0][1],
                    clearance_m={key:self.guard.clearance(pose) for key,pose in
                                 (("origin",origin),("desired",wanted),("measured",actual))})
                self.log(diagnostic)
            except Exception:
                # Diagnostic failures must not suppress the original guard error or hold.
                pass
            raise

    def execute(self,plan):
        self.dds.assert_enabled()
        self.cancelled.clear()
        steps=self.prepare(plan)
        self.log({"event":"plan_start","plan":plan,"state":self.state()})
        completed=[]
        execution_start=time.monotonic()
        published_start=self.dds.published
        planned_seed=self.dds.state()[0][ARM].copy()
        try:
            for i,step in enumerate(steps):
                q,_,_,_=self.dds.state()
                previous=q[ARM]
                segment_start=self.motion.fk(previous)
                targets=step["targets"]
                samples=step["samples"]
                held=[False,False]
                if any(wp.get("gripper","hold")!="hold" for wp in step["waypoints"]):
                    previous,segment_start=self._settle_before_gripper(
                        [j for j,wp in enumerate(step["waypoints"]) if wp.get("gripper","hold")!="hold"])
                    intended=self.motion.errors(self.motion.fk(planned_seed),targets)
                    held=[wp.get("gripper","hold")!="hold" or (p<=.0005 and r<=.003)
                          for wp,(p,r) in zip(step["waypoints"],intended)]
                    self.dds.hold()
                    targets=[segment_start[j] if held[j] else target for j,target in enumerate(targets)]
                    samples=[]
                    if not all(held):
                        for target,duration in step["samples"]:
                            target=target.copy()
                            for j,stationary in enumerate(held):
                                if stationary:target[7*j:7*j+7]=previous[7*j:7*j+7]
                            samples.append((target,duration))
                planned_seed=step["samples"][-1][0].copy()
                active=[p>.0005 or r>.003 for p,r in self.motion.errors(segment_start,targets)]
                with self.dds.lock:
                    idle_target=self.dds.arm_target.copy()
                    idle_tau=self.dds.tau.copy()
                for sample_index,(target,duration) in enumerate(samples):
                    start=time.monotonic()
                    while True:
                        if self.cancelled.is_set():
                            raise RuntimeError("motion cancelled; holding")
                        elapsed=time.monotonic()-start
                        alpha=min(1.,elapsed/duration)
                        blend=alpha*alpha*(3-2*alpha)
                        command=previous+(target-previous)*blend
                        feedback=self.dds.state()
                        current=feedback[0]
                        measured=self.motion.fk(current[ARM])
                        desired=self.motion.fk(command)
                        for j,(origin,wanted,actual) in enumerate(zip(segment_start,desired,measured)):
                            if not active[j]:
                                continue
                            low=min(self.guard.clearance(origin),self.guard.clearance(wanted))<self.guard.travel_clearance
                            self._check_feedback(origin,wanted,actual,.009 if low else .018,feedback,
                                {"step":i,"sample":sample_index,"alpha":alpha,"duration_s":duration,
                                 "phase":"interpolation","arm":("left","right")[j],
                                 "execution_start":execution_start,"published_start":published_start})
                        tau=self.motion.tau(command)
                        for j,moving in enumerate(active):
                            if not moving:
                                command[7*j:7*j+7]=idle_target[7*j:7*j+7]
                                tau[7*j:7*j+7]=idle_tau[7*j:7*j+7]
                        self.dds.command_arm(command,tau)
                        if alpha>=1:
                            break
                        time.sleep(.01)
                    previous=target
                deadline=time.monotonic()+3
                errors=None
                while time.monotonic()<deadline:
                    if self.cancelled.is_set():
                        raise RuntimeError("motion cancelled; holding")
                    self.dds.assert_enabled()
                    feedback=self.dds.state()
                    q,dq,_,_=feedback
                    measured=self.motion.fk(q[ARM])
                    for j,(origin,wanted,actual) in enumerate(zip(segment_start,targets,measured)):
                        if not active[j]:
                            continue
                        low=min(self.guard.clearance(origin),self.guard.clearance(wanted))<self.guard.travel_clearance
                        self._check_feedback(origin,wanted,actual,.009 if low else .018,feedback,
                            {"step":i,"sample":len(samples)-1,"alpha":1.,
                             "phase":"settling","arm":("left","right")[j],
                             "execution_start":execution_start,"published_start":published_start})
                    errors=self.motion.errors(self.motion.fk(q[ARM]),step["targets"])
                    break
                grips={}
                for side,wp in zip(("left","right"),step["waypoints"]):
                    action=wp.get("gripper","hold")
                    if action!="hold":
                        grips[side]=self.gripper(side,4.5*self.dds.gripper_open_sign if action=="open" else 0.)
                observation=self.capture()
                completed.append({"step":i,"errors":errors,"grippers":grips,"observation":observation})
                self.log({"event":"waypoint_done",**completed[-1]})
        except Exception:
            try:
                self.dds.hold()
            except RuntimeError:
                pass
            raise
        return {"completed":completed,"state":self.state()}

    @staticmethod
    def _atomic_json(path, value):
        path = Path(path)
        temporary = path.with_name(path.name + ".tmp")
        with temporary.open("w") as stream:
            json.dump(value, stream, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)

    @staticmethod
    def _steps_summary(steps):
        return {"steps":len(steps), "samples":sum(len(s["samples"]) for s in steps),
                "duration_s":sum(t for s in steps for _,t in s["samples"])}

    def _cached_health(self, require_enabled=True):
        if require_enabled:
            self.dds.assert_enabled()
        feedback=self.dds.state()
        waist_enabled=getattr(self.dds,"waist_enabled",False)
        controlled=ARM+GRIP+(WAIST if waist_enabled else [])
        bad={str(i):int(feedback[2].motor_state[i].motorstate) for i in controlled
             if int(feedback[2].motor_state[i].motorstate)!=0}
        if bad:
            raise CachedHardwareFault(f"controlled motorstate is nonzero: {bad}")
        return feedback

    def _check_joint_segment(self, start, end, ignore_right_tool_envelope=False):
        """Preflight joint samples with the full planned-motion constraints."""
        origin = self.motion.fk(start)
        for alpha in np.linspace(0, 1, 5):
            poses = self.motion.fk(start + alpha*(end-start))
            for j,(a,b) in enumerate(zip(origin,poses)):
                if not (j==1 and ignore_right_tool_envelope):
                    self.guard.check_segment(a,b)

    def _check_runtime_joint_segment(self, start, end, ignore_right_tool_envelope=False):
        """Check surface clearance only; XY/RPY residuals never reject execution."""
        clearances=[self.guard.clearance(p) for p in self.motion.fk(start)]
        for alpha in np.linspace(0, 1, 5):
            poses=self.motion.fk(start + alpha*(end-start))
            for j,pose in enumerate(poses):
                clearance=self.guard.clearance(pose)
                if not np.isfinite(clearance):
                    raise GuardViolation("non-finite gripper pose")
                if (j==1 and ignore_right_tool_envelope and
                        min(clearances[j],clearance)<self.guard.travel_clearance):
                    clearances[j]=clearance
                    continue
                # An already-low start may hold or improve its clearance. Once
                # clear, no later sample may re-enter the surface envelope.
                floor=min(self.guard.minimum_clearance,clearances[j])
                if not np.isfinite(clearance) or clearance < floor-1e-9:
                    raise GuardViolation(f"gripper may touch surface: clearance {clearance:.4f} m")
                clearances[j]=clearance

    def _compile_cached(self, plan, seed, grip_seed, open_q, *, ignore_right_envelope_steps=()):
        if not isinstance(plan,dict) or plan.get("skip_preflight",False):
            raise ValueError("cached plans require skip_preflight=false")
        plan = copy.deepcopy(plan)
        kwargs=({"ignore_right_envelope_steps":ignore_right_envelope_steps}
                if ignore_right_envelope_steps else {})
        steps = self.motion.prepare(plan,seed.copy(),self.guard,**kwargs)
        previous = seed.copy()
        grip = grip_seed.copy()
        reverse_steps = []
        for index,step in enumerate(steps):
            exempt=index in ignore_right_envelope_steps
            if exempt:
                step["ignore_right_tool_envelope"]=True
            step["seed"] = previous.copy()
            step["gripper_targets"] = {}
            restore = {}
            for j,(side,wp) in enumerate(zip(("left","right"),step["waypoints"])):
                action = wp.get("gripper","hold")
                if action != "hold":
                    restore[side] = float(grip[j])
                    grip[j] = open_q*self.dds.gripper_open_sign if action == "open" else 0.
                    step["gripper_targets"][side] = float(grip[j])
            # Every reverse joint segment and inverse gripper operation is checked
            # now. Runtime recovery only selects a prefix of this frozen path.
            starts = [previous.copy()] + [q.copy() for q,_ in step["samples"][:-1]]
            reverse_samples = [(q.copy(),float(duration)) for q,(_,duration)
                               in reversed(list(zip(starts,step["samples"])))]
            end = step["samples"][-1][0].copy()
            cursor = end.copy()
            for target,_ in reverse_samples:
                self._check_joint_segment(cursor,target,ignore_right_tool_envelope=exempt)
                cursor = target
            reverse = {"seed":end, "samples":reverse_samples,
                       "targets":[p.copy() for p in self.motion.fk(previous)],
                       "waypoints":[waypoint(p) for p in self.motion.fk(previous)],
                       "gripper_targets":{}}
            undo = {"seed":end.copy(), "samples":[(end.copy(),.12)],
                    "targets":[p.copy() for p in self.motion.fk(end)],
                    "waypoints":[waypoint(p) for p in self.motion.fk(end)],
                    "gripper_targets":restore}
            if exempt:
                reverse["ignore_right_tool_envelope"]=True
                undo["ignore_right_tool_envelope"]=True
            for side in restore:
                if not (side=="right" and exempt):
                    self.guard.check_gripper(undo["targets"][0 if side=="left" else 1])
            reverse_steps.append((undo,reverse))
            previous = end
            step["end_grip"] = grip.copy()
        return {"plan":plan,"steps":steps,"reverse":reverse_steps,
                "seed":seed.copy(),"end_q":previous,"end_grip":grip}

    def _archive_cached(self, bundle):
        def serial(value):
            if isinstance(value,np.ndarray):
                return value.tolist()
            if isinstance(value,dict):
                return {k:serial(v) for k,v in value.items()}
            if isinstance(value,(tuple,list)):
                return [serial(v) for v in value]
            return value
        path = bundle["directory"] / (bundle["token"] + "_prepared.json")
        self._atomic_json(path,serial({k:v for k,v in bundle.items() if k!="directory"}))
        return str(path)

    @staticmethod
    def _restore_cached_arrays(value, key=None):
        if key in ("seed", "end_q", "pregrasp_seed", "low_pose_seed"):
            result = np.asarray(value, dtype=float)
            if result.shape != (14,) or not np.isfinite(result).all():
                raise ValueError(f"invalid archived {key}")
            return result
        if key in ("start_grip", "end_grip"):
            result = np.asarray(value, dtype=float)
            if result.shape != (2,) or not np.isfinite(result).all():
                raise ValueError(f"invalid archived {key}")
            return result
        if key == "targets":
            result = [np.asarray(p, dtype=float) for p in value]
            if len(result) != 2 or any(p.shape != (4, 4) or not np.isfinite(p).all() for p in result):
                raise ValueError("invalid archived targets")
            return result
        if key == "samples":
            if type(value) is int and value >= 0:
                return value
            if not isinstance(value, list):
                raise ValueError("invalid archived samples")
            result = []
            for sample in value:
                if not isinstance(sample, list) or len(sample) != 2:
                    raise ValueError("invalid archived sample")
                q, duration = np.asarray(sample[0], dtype=float), sample[1]
                if (q.shape != (14,) or not np.isfinite(q).all() or
                        type(duration) not in (int, float) or not np.isfinite(duration) or duration <= 0):
                    raise ValueError("invalid archived sample")
                result.append((q, float(duration)))
            return result
        if isinstance(value, dict):
            return {k: RobotService._restore_cached_arrays(v, k) for k, v in value.items()}
        if isinstance(value, list):
            return [RobotService._restore_cached_arrays(v) for v in value]
        return value

    def _rebase_cached_observer(self, bundle, current_arm_q):
        old = bundle["grasp"]["seed"][:7].copy()
        current = current_arm_q[:7].copy()
        difference = float(np.max(np.abs(current - old)))
        new_pose = self.motion.fk(np.r_[current, bundle["grasp"]["seed"][7:]])[0]
        new_waypoint = waypoint(new_pose)

        def rebase(value):
            if isinstance(value, np.ndarray) and value.shape == (14,):
                value[:7] = current
            elif isinstance(value, dict):
                if "gripper_targets" in value:
                    value["gripper_targets"].pop("left", None)
                if "left_arm" in value:
                    for point in value["left_arm"]:
                        point.update(new_waypoint)
                        point["gripper"] = "hold"
                for nested in value.values():
                    rebase(nested)
                if "targets" in value:
                    value["targets"][0] = new_pose.copy()
                if "waypoints" in value:
                    value["waypoints"][0].update(new_waypoint)
            elif isinstance(value, (list, tuple)):
                for nested in value:
                    rebase(nested)

        rebase(bundle)
        return difference

    def restore_cached_transaction(self, archive_path, sha256, output_dir, image_dir):
        if self.dds.enabled or self.guard is None:
            raise ValueError("restore requires idle control and a configured scene")
        archive = Path(archive_path).resolve()
        directory = self._cached_archive_directory(output_dir)
        images = self._cached_archive_directory(image_dir)
        self._cached_archive_directory(archive.parent)
        raw = archive.read_bytes()
        actual_sha256 = hashlib.sha256(raw).hexdigest()
        if sha256 != actual_sha256:
            raise ValueError("archived plan sha256 mismatch")
        data = json.loads(raw)
        if (not isinstance(data, dict) or data.get("kind") != "transaction" or
                data.get("camera_selection") != camera_selection(self.active_arm) or
                data.get("scene") != vars(self.guard) or
                data.get("gripper_open_sign") != self.dds.gripper_open_sign):
            raise ValueError("archived plan does not match current service configuration")
        old_token = data.get("token")
        if (not isinstance(old_token, str) or len(old_token) != 32 or
                archive.name != old_token + "_prepared.json"):
            raise ValueError("invalid archived transaction")
        bundle = self._restore_cached_arrays(data)
        for name in ("grasp", "success", "failure"):
            if not bundle.get(name, {}).get("steps"):
                raise ValueError(f"archived {name} path is missing")
        if bundle.get("decision_channel", {}).get("mode") != "serial":
            raise ValueError("restored transaction requires serial checkpoint decisions")
        q, dq, _, _ = self._cached_health(require_enabled=False)
        expected = bundle["grasp"]["seed"]
        errors = self.motion.errors(self.motion.fk(q[ARM]), self.motion.fk(expected))
        if (errors[1][0] > .05 or errors[1][1] > 5.0 or
                np.max(np.abs(dq[ARM])) >= .08 or
                np.max(np.abs(q[GRIP] - bundle["start_grip"])) > .15):
            raise ValueError("archived plan start does not match current feedback")
        reconnect = (np.max(np.abs(q[ARM][7:] - expected[7:])) > .05 or
                     errors[1][0] > WAYPOINT_POSITION_TOLERANCE_M)
        current_start = q[ARM].copy()
        observer_rebased_rad = self._rebase_cached_observer(bundle, current_start)
        first_step = bundle["grasp"]["steps"][0]
        first_reverse = bundle["grasp"]["reverse"][0][1]
        connector_ik_calls = 0
        if reconnect:
            # Leave a low measured start vertically before joining the frozen
            # first waypoint. Reuse the runtime connector, never full prepare.
            if first_step.get("gripper_targets") or any(
                    p.get("gripper", "hold") != "hold" for p in first_step["waypoints"]):
                raise ValueError("archived first step cannot accept a start connector")
            targets = first_step["targets"]
            if self.guard.clearance(targets[1]) < self.guard.travel_clearance:
                raise ValueError("archived first waypoint lacks connector clearance")
            runtime = RuntimeSourcePaths(self.motion, 3., self.dds.gripper_open_sign,
                                         lambda: self._cached_health(require_enabled=False))
            high = [waypoint(p) for p in self.motion.fk(current_start)]
            high[1]["z"] = max(high[1]["z"], float(targets[1][2, 3]))
            high[1]["z"] += max(0., self.guard.travel_clearance -
                                  self.guard.clearance(matrix(high[1])) + .001)
            speed = bundle["grasp"]["plan"].get("speed_scale", 1.)
            lift = runtime.step(high, current_start, speed_scale=speed)
            connector = runtime.step([waypoint(p) for p in targets],
                                     lift["samples"][-1][0], speed_scale=speed,
                                     fixed_end_q=first_step["samples"][-1][0])
            raw = {"samples": lift["samples"] + connector["samples"],
                   "targets": targets, "waypoints": first_step["waypoints"]}
            # Keep the original step count and frozen boundary for retries and
            # gates; rebuild its reverse so partial recovery returns to now.
            prefix = runtime.assemble(bundle["grasp"]["plan"], [raw],
                                      current_start, bundle["start_grip"])
            first_step = prefix["steps"][0]
            first_step["clearance_before_sample"] = len(lift["samples"])
            bundle["grasp"]["steps"][0] = first_step
            bundle["grasp"]["reverse"][0] = prefix["reverse"][0]
            first_reverse = bundle["grasp"]["reverse"][0][1]
            connector_ik_calls = runtime.solve_count
        bundle["grasp"]["seed"] = current_start.copy()
        first_step["seed"] = current_start.copy()
        first_reverse["samples"][-1] = (current_start.copy(), first_reverse["samples"][-1][1])
        first_reverse["targets"] = [p.copy() for p in self.motion.fk(current_start)]
        first_reverse["waypoints"] = [waypoint(p) for p in first_reverse["targets"]]
        bundle["token"] = secrets.token_hex(16)
        bundle["directory"] = directory
        bundle["image_dir"] = str(images)
        bundle["decision_channel"]["worker_id"] = secrets.token_hex(16)
        bundle["decision_channel"]["worker_ready_path"] = str(directory / (bundle["token"] + "_worker.json"))
        bundle["restored_from"] = {"archive_path": str(archive),
                                   "archive_sha256": actual_sha256,
                                   "original_token": old_token,
                                   "observer_rebased_max_rad": observer_rebased_rad,
                                   "start_connector_rebuilt": bool(reconnect),
                                   "start_connector_ik_calls": connector_ik_calls,
                                   "restored_at": datetime.now().isoformat()}
        directory.mkdir(parents=True, exist_ok=True)
        restored_archive = self._archive_cached(bundle)
        self.cached_plans[bundle["token"]] = bundle
        self.log({"event": "cached_transaction_restored", "token": bundle["token"],
                  **bundle["restored_from"]})
        return {"token": bundle["token"], "archive_path": restored_archive,
                "restored_from": bundle["restored_from"],
                "checkpoint_discovery_path": str(directory / (bundle["token"] + "_checkpoint_latest.json")),
                "checkpoint_path": str(directory / (bundle["token"] + "_checkpoint.json")),
                "decision_channel": copy.deepcopy(bundle["decision_channel"]),
                "wait_indefinitely": True,
                "image_dir": str(images),
                "preflight_performed": False,
                "start_verified": True,
                "camera_selection": bundle["camera_selection"]}

    @staticmethod
    def _cached_archive_directory(path):
        directory=Path(path).resolve()
        if not any(directory.is_relative_to((ROOT/name).resolve())
                   for name in ("runs",)):
            raise ValueError("cached plan files must be under project runs")
        return directory

    def _cached_image_directory(self,bundle,stage):
        if "image_dir" not in bundle:
            return bundle["directory"]/stage
        return self._cached_archive_directory(Path(bundle["image_dir"])/bundle["token"]/stage)

    def prepare_cached(self, request, transaction=False, *, _arm_seed=None, _grip_seed=None):
        if transaction:
            # Mark only the placement opening, never source/retry/recovery opens.
            # Preserve this metadata through source corrections and frozen tails.
            request = copy.deepcopy(request)
            actor = request["success_plan"]["right_arm"]
            for point in actor:
                point.pop("pose_audit_release", None)
            for point in actor:
                if point.get("gripper") == "open":
                    point["pose_audit_release"] = {k: point[k] for k in
                                                   ("x", "y", "z", "roll", "pitch", "yaw")}
                    break
        if self.guard is None:
            raise ValueError("set_scene required before motion")
        release_config = configure_release_gate(request, self.guard) if transaction else None
        ignore_source_envelope=request.get("grasp_retry",{}).get("ignore_grasp_tool_envelope",False)
        if type(ignore_source_envelope) is not bool:
            raise ValueError("ignore_grasp_tool_envelope must be boolean")
        directory = self._cached_archive_directory(request["checkpoint_dir" if transaction else "archive_dir"])
        image_dir = (self._cached_archive_directory(request["image_dir"])
                     if transaction and "image_dir" in request else None)
        open_q = request.get("gripper_open_q",3.0)
        timeout = request.get("decision_timeout_s",300.)
        if isinstance(open_q,bool) or not isinstance(open_q,(int,float)) or not np.isfinite(open_q) or not 0<open_q<=5.4:
            raise ValueError("gripper_open_q must be in (0,5.4]")
        if isinstance(timeout,bool) or not isinstance(timeout,(int,float)) or not np.isfinite(timeout) or not 1<=timeout<=300:
            raise ValueError("decision_timeout_s must be between 1 and 300")
        q,_,_,_=self._cached_health(require_enabled=False)
        arm_seed=q[ARM].copy() if _arm_seed is None else np.asarray(_arm_seed,dtype=float).copy()
        grip_seed=q[GRIP].copy() if _grip_seed is None else np.asarray(_grip_seed,dtype=float).copy()
        if arm_seed.shape!=(14,) or grip_seed.shape!=(2,) or not np.isfinite(arm_seed).all() or not np.isfinite(grip_seed).all():
            raise ValueError("cached preparation requires finite arm and gripper seeds")
        token = secrets.token_hex(16)
        bundle = {"token":token,"kind":"transaction" if transaction else "plan",
                  "camera_selection":camera_selection(self.active_arm),
                  "directory":directory,"scene":copy.deepcopy(vars(self.guard)),
                  "gripper_open_sign":self.dds.gripper_open_sign,
                  "gripper_open_q":open_q,"decision_timeout_s":float(timeout),
                  "start_grip":grip_seed.copy(),"prepared_at":datetime.now().isoformat()}
        if transaction:
            bundle["pose_audit_original_source"] = {
                k: request["success_plan"]["right_arm"][0][k]
                for k in ("x", "y", "z", "roll", "pitch", "yaw")}
        if release_config is not None:
            bundle["pre_release_adjustment"] = release_config
        if image_dir is not None:
            bundle["image_dir"]=str(image_dir)
        bundle["grasp"] = self._compile_cached(request["grasp_plan" if transaction else "plan"],
                                               arm_seed,grip_seed,open_q)
        if transaction:
            if "decision_channel" in request:
                channel=copy.deepcopy(request["decision_channel"])
                if not isinstance(channel,dict) or not isinstance(channel.get("require_worker",False),bool):
                    raise ValueError("decision_channel requires a boolean require_worker")
                mode=channel.get("mode","worker")
                if mode not in ("worker","serial"):
                    raise ValueError("decision_channel mode must be worker or serial")
                if mode=="serial" and channel.get("require_worker",False):
                    raise ValueError("serial decision channel cannot require a worker")
                heartbeat_timeout=channel.get("heartbeat_timeout_s",15.)
                if (isinstance(heartbeat_timeout,bool) or not isinstance(heartbeat_timeout,(int,float)) or
                        not np.isfinite(heartbeat_timeout) or not 1<=heartbeat_timeout<=300):
                    raise ValueError("heartbeat_timeout_s must be between 1 and 300")
                bundle["decision_channel"]={"mode":mode,"require_worker":channel.get("require_worker",False),
                    "heartbeat_timeout_s":float(heartbeat_timeout),"worker_id":secrets.token_hex(16),
                    "worker_ready_path":str(directory/(token+"_worker.json"))}
            end = bundle["grasp"]["end_q"]
            grip = bundle["grasp"]["end_grip"]
            if self.guard.clearance(self.motion.fk(end)[1]) < self.guard.travel_clearance:
                raise ValueError("grasp checkpoint must be above transport clearance")
            for name in ("success","failure"):
                kwargs=({"ignore_right_envelope_steps":(0,1,2)}
                        if name=="success" and ignore_source_envelope else {})
                bundle[name] = self._compile_cached(request[name+"_plan"],end,grip,open_q,**kwargs)
            if "auto_grasp_gate" in request:
                gate=copy.deepcopy(request["auto_grasp_gate"])
                if gate.get("vision",{}).get("camera") not in self._execution_cameras(bundle):
                    raise ValueError("grasp vision must use head or physical execution wrist")
                index=gate.get("after_success_step")
                success_steps=bundle["success"]["steps"]
                if (isinstance(index,bool) or not isinstance(index,int) or
                        not 0<=index<len(success_steps)-1):
                    raise ValueError("auto gate index must precede the success transfer tail")
                gate_step=success_steps[index]
                gate_seed=gate_step["samples"][-1][0]
                if self.guard.clearance(self.motion.fk(gate_seed)[1])<self.guard.travel_clearance:
                    raise ValueError("auto grasp gate must be above transport clearance")
                if gate_step["end_grip"][1]!=0.:
                    raise ValueError("auto grasp gate requires an earlier right gripper close")
                bundle["gate_failure"]=self._compile_cached(gate["failure_plan"],gate_seed,
                                                              gate_step["end_grip"],open_q)
                bundle["auto_grasp_gate"]={"after_success_step":index,
                                           "vision":prepare_vision(gate["vision"])}
            if "grasp_retry" in request:
                if "auto_grasp_gate" in request:
                    raise ValueError("grasp_retry replaces auto_grasp_gate")
                self._prepare_grasp_retry(bundle,request)
            if "pregrasp_adjustment" in request:
                adjustment=copy.deepcopy(request["pregrasp_adjustment"])
                self._validate_adjustment_config(bundle,adjustment)
                bundle["pregrasp_adjustment"]=adjustment
        directory.mkdir(parents=True,exist_ok=True)
        archive = self._archive_cached(bundle)
        if not hasattr(self,"cached_plans"):
            self.cached_plans = {}
        # Keep bounded process-local storage. Tokens are single-use and cannot
        # replay across a service restart or a different scene.
        self.cached_plans[token] = bundle
        while len(self.cached_plans)>4:
            self.cached_plans.pop(next(iter(self.cached_plans)))
        result = {"token":token,"archive_path":archive,"collision_checked":False,
                  "camera_selection":copy.deepcopy(bundle["camera_selection"]),
                  "gripper_open_q":open_q,
                  "paths":{name:self._steps_summary(bundle[name]["steps"])
                           for name in ("grasp","success","failure","gate_failure") if name in bundle}}
        if release_config is not None:
            result.update(pre_release_adjustment=copy.deepcopy(release_config),
                          checkpoint_discovery_path=str(directory/(token+"_checkpoint_latest.json")))
        if "grasp_retry" in bundle:
            result.update(checkpoint_discovery_path=str(directory/(token+"_checkpoint_latest.json")),
                          max_grasp_attempts=bundle["grasp_retry"]["max_attempts"],
                          grasp_z_m=bundle["grasp_z_m"],
                          grasp_z_options_m=bundle["grasp_z_options_m"])
            config=bundle["grasp_retry"]
            result["confirm_after_descent"]=config.get("confirm_after_descent",False)
            result["preclose_high_recheck_supported"]=config.get("alignment_failure_policy")=="recheck_high_then_grasp"
            result["alignment_failure_policy"]=config.get("alignment_failure_policy","return_init")
            result["max_high_rechecks"]=config.get("max_high_rechecks",3)
            result["low_z_step_m"]=.005
            result["max_low_z_adjustments"]=config.get("max_low_z_adjustments",2)
            result["close_after_low_z_limit"]=True
            result["ignore_grasp_tool_envelope"]=config.get("ignore_grasp_tool_envelope",False)
            result["grasp_tool_envelope_scope"]="right_source_descent_close_release_vertical_retreat_only"
            if "success_hold_s" in config:
                result["grasp_contact_policy"]={key:config[key] for key in (
                    "success_q","success_tolerance_q","max_abs_dq","success_hold_s","contact_timeout_s")}
                result["grasp_contact_policy"]["scope"]="stationary_after_close_only"
                # Contact classification is intentionally aperture-agnostic:
                # after a stable close, every aperture that remains above the
                # full-close/empty threshold is accepted as a held piece.
                # ``success_q`` and ``success_tolerance_q`` remain in the
                # response for compatibility with archived plans, but no
                # longer reject a valid, partially closed grasp.
                result["grasp_contact_policy"]["success_rule"]="stable_aperture_at_or_above_empty_below_q"
                result["grasp_contact_policy"]["success_min_q"]=config["empty_below_q"]
        if transaction:
            result.update(checkpoint_path=str(directory/(token+"_checkpoint.json")),
                          decision_path=str(directory/(token+"_decision.json")),
                          receipt_path=str(directory/(token+"_decision_receipt.json")),
                          decision_timeout_s=float(timeout))
            if image_dir is not None:
                result["image_dir"]=str(image_dir)
            if "decision_channel" in bundle:
                result["decision_channel"]=copy.deepcopy(bundle["decision_channel"])
                if bundle["decision_channel"].get("mode")=="serial":
                    result.update(decision_timeout_s=None,wait_indefinitely=True)
            if "pregrasp_adjustment" in bundle:
                result.update(post_adjustment_checkpoint_path=str(directory/(token+"_post_adjustment_checkpoint.json")),
                              post_adjustment_decision_path=str(directory/(token+"_post_adjustment_decision.json")),
                              max_corrections=bundle["pregrasp_adjustment"]["max_corrections"],
                              post_adjustment_checkpoints=[{
                                  "correction_index":i,
                                  "checkpoint_path":str(directory/(token+self._checkpoint_suffix(i)+"_checkpoint.json")),
                                  "decision_path":str(directory/(token+self._checkpoint_suffix(i)+"_decision.json")),
                                  "receipt_path":str(directory/(token+self._checkpoint_suffix(i)+"_decision_receipt.json"))}
                                  for i in range(1,bundle["pregrasp_adjustment"]["max_corrections"]+1)])
        self.log({"event":"cached_prepared",**result})
        return result

    def prepare_sequence(self, requests):
        """Precheck two transactions, including the predicted successful handoff."""
        if not isinstance(requests,list) or len(requests)!=2 or not all(isinstance(r,dict) for r in requests):
            raise ValueError("sequence requires exactly two transaction requests")
        prepared=[]
        try:
            prepared.append(self.prepare_cached(requests[0],transaction=True))
            first=self.cached_plans[prepared[0]["token"]]
            ending=first["success"]
            seed,grip=ending["end_q"],ending["end_grip"]
            poses=[waypoint(p) for p in self.motion.fk(seed)]
            second=copy.deepcopy(requests[1])
            # The observer stays at the predecessor's predicted ending in every
            # success, failure and retry path; no stale low observer connector.
            def hold_observer(value):
                if isinstance(value,dict):
                    if "left_arm" in value and "right_arm" in value:
                        if any(wp.get("gripper","hold")!="hold" for wp in value["left_arm"]):
                            raise ValueError("sequence second transaction must hold the observer gripper")
                        value["left_arm"]=[dict(poses[0],gripper="hold")]
                    for child in value.values():
                        hold_observer(child)
                elif isinstance(value,list):
                    for child in value:
                        hold_observer(child)
            hold_observer(second)
            lift=second["grasp_plan"]["right_arm"][0]
            if lift.get("gripper","hold")!="hold" or lift["z"]<poses[1]["z"]:
                raise ValueError("sequence second transaction must begin with a vertical holding lift")
            for key in ("x","y","roll","pitch","yaw"):
                lift[key]=poses[1][key]
            prepared.append(self.prepare_cached(second,transaction=True,_arm_seed=seed,_grip_seed=grip))
            sequence_token=secrets.token_hex(16)
            sequence={"sequence_token":sequence_token,"transactions":prepared,
                      "collision_checked":False,"handoff_arm_q":seed.tolist(),
                      "handoff_grippers":grip.tolist(),"handoff_poses":poses,
                      "second_start_refreshed":True,"recheck_between_transactions":False}
            directory=first["directory"]
            sequence["archive_path"]=str(directory/(sequence_token+"_sequence_prepared.json"))
            self._atomic_json(sequence["archive_path"],sequence)
            if not hasattr(self,"cached_sequences"):
                self.cached_sequences={}
            self.cached_sequences[sequence_token]=sequence
            while len(self.cached_sequences)>2:
                self.cached_sequences.pop(next(iter(self.cached_sequences)))
            self.log({"event":"sequence_prepared",**sequence})
            return sequence
        except Exception:
            for response in prepared:
                self.cached_plans.pop(response["token"],None)
            raise

    def execute_sequence(self, sequence_token):
        sequence=getattr(self,"cached_sequences",{}).pop(sequence_token,None)
        if sequence is None:
            raise ValueError("unknown or consumed sequence token")
        tokens=[response["token"] for response in sequence["transactions"]]
        bundles=[self.cached_plans.pop(token,None) for token in tokens]
        if any(bundle is None or bundle["kind"]!="transaction" for bundle in bundles):
            raise ValueError("sequence transaction was consumed or invalidated")
        result={"sequence_token":sequence_token,"transactions":[],"outcome":"not_started"}
        self.log({"event":"sequence_start","sequence_token":sequence_token,"tokens":tokens})
        for index,bundle in enumerate(bundles):
            # Only this internally prechecked continuation can omit entry pose
            # gates. Health, scene and collision checks remain active throughout.
            if index==0:
                bundle["require_release_open_before_continuation"]=True
            part=self._execute_cached_bundle(bundle,transaction=True,
                    verify_start=index==0,keep_control_on_placed=index==0)
            result["transactions"].append(part)
            if not part.get("ok",True) or part["outcome"]!="placed":
                result.update(ok=False,outcome="sequence_stopped",stopped_transaction=index+1)
                break
        else:
            result.update(ok=True,outcome="placed")
        result["control_enabled"]=bool(self.dds.enabled)
        result["dds_fault"]=self.dds.fault
        self._atomic_json(bundles[0]["directory"]/(sequence_token+"_sequence_result.json"),result)
        self.log({"event":"sequence_finished",**result})
        return result

    def _compile_grasp_attempts(self,plans,seed,grip,open_q,count,ignore_grasp_tool_envelope=False,precheck_unclosed_rise=False):
        attempts=[]
        exempt={"success":(0,1,2),"retry":(0,),"unknown":(0,1)} if ignore_grasp_tool_envelope else {}
        def compile_path(name,arm_seed,gripper_seed):
            kwargs=({"ignore_right_envelope_steps":exempt[name]} if name in exempt else {})
            return self._compile_cached(plans[name],arm_seed,gripper_seed,open_q,**kwargs)
        for _ in range(count):
            success=compile_path("success",seed,grip)
            close=success["steps"][1]
            retry=compile_path("retry",close["samples"][-1][0],close["end_grip"])
            # Also check the rise before any closure, with an open gripper.
            # _prepare_grasp_retry repeats this for every allowed low Z.
            descent=success["steps"][0]
            preclose_retry=(compile_path("retry",descent["samples"][-1][0],descent["end_grip"])
                            if precheck_unclosed_rise else None)
            unknown=compile_path("unknown",close["samples"][-1][0],close["end_grip"])
            failure=compile_path("failure",seed,grip)
            retry_failure=compile_path("failure",retry["end_q"],retry["end_grip"])
            prepared_attempt=dict(success=success,retry=retry,unknown=unknown,
                                  failure=failure,retry_failure=retry_failure)
            if preclose_retry is not None:
                prepared_attempt["preclose_retry"]=preclose_retry
            attempts.append(prepared_attempt)
            seed,grip=retry["end_q"],retry["end_grip"]
        return attempts

    def _prepare_grasp_retry(self,bundle,request):
        config=copy.deepcopy(request["grasp_retry"])
        config.setdefault("confirm_after_descent",False)
        config.setdefault("max_low_z_adjustments",2)
        config.setdefault("alignment_failure_policy","return_init")
        config.setdefault("max_high_rechecks",3)
        if (config["alignment_failure_policy"] not in ("return_init","recheck_high_then_grasp") or
                type(config["max_high_rechecks"]) is not int or not 1<=config["max_high_rechecks"]<=3):
            raise ValueError("invalid high recheck policy or budget")
        if (type(config["confirm_after_descent"]) is not bool or
                type(config["max_low_z_adjustments"]) is not int or
                not 0<=config["max_low_z_adjustments"]<=2):
            raise ValueError("invalid preclose confirmation or low Z adjustment count")
        if bundle.get("decision_channel",{}).get("mode")!="serial":
            raise ValueError("grasp_retry requires the serial decision channel")
        for key,value in (("max_attempts",3),("after_close_step",1),("empty_below_q",.15),
                          ("success_q",1.98),("success_tolerance_q",.25),("max_abs_dq",.1)):
            config.setdefault(key,value)
        if type(config["max_attempts"]) is not int or not 1<=config["max_attempts"]<=3 or config["after_close_step"]!=1:
            raise ValueError("grasp_retry supports at most three attempts and close step 1")
        for key in ("empty_below_q","success_q","success_tolerance_q","max_abs_dq"):
            if type(config[key]) not in (int,float) or not np.isfinite(config[key]) or config[key]<=0:
                raise ValueError("invalid grasp_retry q/dq threshold")
        # ``success_q``/``success_tolerance_q`` are retained as archived-plan
        # metadata only.  Contact acceptance is based on the full-close band
        # below ``empty_below_q``, so their historical band-overlap check would
        # incorrectly reject otherwise valid widened configurations.
        if "success_hold_s" in config:
            config.setdefault("contact_timeout_s",max(2.,config["success_hold_s"])
                              if type(config["success_hold_s"]) in (int,float) else 2.)
            for key in ("success_hold_s","contact_timeout_s"):
                if type(config[key]) not in (int,float) or not np.isfinite(config[key]) or not 0<config[key]<=10:
                    raise ValueError("invalid grasp_retry contact hold duration")
            if config["contact_timeout_s"]<config["success_hold_s"]:
                raise ValueError("contact timeout must cover the required continuous hold")
        plans={"success":copy.deepcopy(request["success_plan"]),"retry":config["retry_plan"],
               "unknown":config["unknown_failure_plan"],"failure":config["failure_plan"]}
        actions=[p.get("gripper","hold") for p in plans["success"]["right_arm"][:3]]
        if actions!=["hold","close","hold"] or len(plans["success"]["right_arm"])<4:
            raise ValueError("grasp_retry success must descend, close at zero, lift, then transfer")
        high=bundle["grasp"]["plan"]["right_arm"][-1]
        source=plans["success"]["right_arm"][0]
        if any(abs(plans["success"]["right_arm"][1][k]-source[k])>1e-6 for k in ("x","y","z","roll","pitch","yaw")):
            raise ValueError("close must occur at the unchanged source pose")
        if any(np.linalg.norm([plans["success"]["right_arm"][i][axis]-high[axis] for axis in ("x","y")])>.001 for i in (0,1,2)):
            raise ValueError("grasp_retry source, lift and pregrasp must share XY")
        retry=plans["retry"]["right_arm"]
        if len(retry)!=2 or [p.get("gripper","hold") for p in retry]!=["hold","open"]:
            raise ValueError("retry_plan must rise to pregrasp then open in a separate waypoint")
        for p in retry:
            if any(abs(p[k]-high[k])>1e-6 for k in ("x","y","z","roll","pitch","yaw")):
                raise ValueError("retry high pose must equal the original pregrasp pose")
        unknown=plans["unknown"]["right_arm"]
        if unknown[0].get("gripper")!="open" or any(abs(unknown[0][k]-source[k])>1e-6 for k in ("x","y","z","roll","pitch","yaw")):
            raise ValueError("unknown contact must release at the original source before retreat")
        ignore_source_envelope=config.get("ignore_grasp_tool_envelope",False)
        if ignore_source_envelope:
            if (len(unknown)<2 or unknown[1].get("gripper","hold")!="hold" or
                    unknown[1]["z"]<high["z"] or plans["success"]["right_arm"][2]["z"]<high["z"]):
                raise ValueError("source envelope exception must retreat vertically to high before reconnecting")
            for point in plans["success"]["right_arm"][:3]+unknown[:2]:
                if any(abs(point[k]-high[k])>1e-6 for k in ("x","y","roll","pitch","yaw")):
                    raise ValueError("source envelope exception requires unchanged nominal XY/RPY")
        for name,key in (("unknown","unknown_failure_source_steps"),("failure","failure_source_steps")):
            indices=config.get(key)
            if (not isinstance(indices,list) or not indices or any(type(i) is not int or not 0<=i<len(plans[name]["right_arm"]) for i in indices)):
                raise ValueError("grasp_retry requires explicit source indices for all return paths")
            if any(np.linalg.norm([plans[name]["right_arm"][i][axis]-source[axis] for axis in ("x","y")])>.001 for i in indices):
                raise ValueError("shifted retry failure steps must share source XY")
        if abs(bundle["grasp"]["end_grip"][1]-bundle["gripper_open_q"]*self.dds.gripper_open_sign)>.01:
            raise ValueError("grasp_retry starts with right gripper fully open")
        bundle["grasp_retry"]=config
        bundle["retry_base_plans"]=copy.deepcopy(plans)
        compile_kwargs=({"ignore_grasp_tool_envelope":True} if ignore_source_envelope else {})
        if config["alignment_failure_policy"]=="recheck_high_then_grasp":
            compile_kwargs["precheck_unclosed_rise"]=True
        bundle["retry_attempts"]=self._compile_grasp_attempts(plans,bundle["grasp"]["end_q"],
                    bundle["grasp"]["end_grip"],bundle["gripper_open_q"],config["max_attempts"],**compile_kwargs)
        source_z=float(source["z"])
        options=config.get("grasp_z_options_m",[source_z])
        if (not isinstance(options,list) or not 1<=len(options)<=8 or
                any(type(z) not in (int,float) or not np.isfinite(z) or z>=high["z"] for z in options) or
                len(set(options))!=len(options) or
                not any(abs(z-source_z)<=1e-9 for z in options)):
            raise ValueError("grasp_z_options_m must contain 1-8 unique finite heights below high, including source z")
        bundle["grasp_z_m"]=source_z
        bundle["grasp_z_options_m"]=[float(z) for z in options]
        bundle["grasp_z_preflights"]=[]
        # Every selectable height passes complete branch preparation before any
        # motion. At the visual gate only its already checked source height is
        # selected; target, init and high poses are never changed by this field.
        checked_heights=list(bundle["grasp_z_options_m"])
        if config["confirm_after_descent"]:
            checked_heights=sorted({round(z-.005*n,9) for z in checked_heights
                                   for n in range(config["max_low_z_adjustments"]+1)})
        bundle["low_z_prechecked_m"]=checked_heights
        for z in checked_heights:
            candidate=copy.deepcopy(plans)
            self._set_source_grasp_z(candidate,z)
            attempts=(bundle["retry_attempts"] if abs(z-source_z)<=1e-9 else
                      self._compile_grasp_attempts(candidate,bundle["grasp"]["end_q"],
                          bundle["grasp"]["end_grip"],bundle["gripper_open_q"],config["max_attempts"],**compile_kwargs))
            bundle["grasp_z_preflights"].append({"grasp_z_m":z,
                "attempts":[{name:self._steps_summary(compiled["steps"])
                             for name,compiled in attempt.items()} for attempt in attempts]})
        bundle["adjustment_base"]={"high_pose":copy.deepcopy(high)}
        bundle["pregrasp_seed"]=bundle["grasp"]["end_q"].copy()

    @staticmethod
    def _set_source_grasp_z(plans,z):
        """Only source descent/close and stationary unknown release use low Z."""
        for index in (0,1):
            plans["success"]["right_arm"][index]["z"]=float(z)
        plans["unknown"]["right_arm"][0]["z"]=float(z)

    def _classify_grasp_contact(self,bundle):
        config=bundle["grasp_retry"]
        if "success_hold_s" in config:
            return self._classify_held_grasp_contact(bundle)
        samples=[]
        for index in range(3):
            if index:
                time.sleep(.08)
            feedback=self._cached_health()
            feedback_at=time.monotonic()
            q,dq,_,_=feedback
            samples.append({"q":float(q[GRIP[1]]*self.dds.gripper_open_sign),"dq":float(dq[GRIP[1]])})
        aperture=[p["q"] for p in samples]
        stable=(max(aperture)-min(aperture)<=.08 and all(abs(p["dq"])<=config["max_abs_dq"] for p in samples))
        # The gripper's measured aperture is the contact signal.  A stable
        # fully-closed value indicates an empty grasp; any stable value that
        # remains open means something is between the fingers and is accepted,
        # regardless of how far it is from the old nominal ``success_q``.
        status=("empty" if stable and all(v<config["empty_below_q"] for v in aperture) else
                "success" if stable and all(v>=config["empty_below_q"] for v in aperture) else "unknown")
        evidence={"status":status,"stable":stable,"samples":samples,"grasp_attempt_index":bundle["grasp_attempt_index"],
                  "commanded_close_q":0.,"success_q":config["success_q"],"success_tolerance_q":config["success_tolerance_q"],
                  "success_rule":"stable_aperture_at_or_above_empty_below_q",
                  "success_min_q":config["empty_below_q"]}
        if status == "success":
            self._snapshot_grasp_success(bundle, feedback, feedback_at)
        self._atomic_json(bundle["directory"]/(bundle["token"]+"_grasp_"+str(bundle["grasp_attempt_index"])+"_contact.json"),evidence)
        self.log({"event":"grasp_contact_classified","token":bundle["token"],**evidence})
        return evidence

    def _classify_held_grasp_contact(self,bundle):
        """Observe real feedback only while stationary after close; never during carry."""
        config=bundle["grasp_retry"]
        started=time.monotonic()
        deadline=started+config["contact_timeout_s"]
        samples=[]
        success_since=None
        success_samples=0
        empty_samples=[]
        previous_time=None
        held_s=0.
        status="unknown"
        stable=False
        while True:
            if self.cancelled.is_set():
                raise RuntimeError("grasp contact observation cancelled")
            feedback=self._cached_health()
            feedback_at=time.monotonic()
            q,dq,_,_=feedback
            now=time.monotonic()
            value=float(q[GRIP[1]]*self.dds.gripper_open_sign)
            velocity=float(dq[GRIP[1]])
            sample={"q":value,"dq":velocity,"elapsed_s":now-started}
            samples.append(sample)
            # A missing observation interval is not evidence of continuous hold.
            if previous_time is not None and now-previous_time>.12:
                success_since=None
                success_samples=0
                empty_samples=[]
            previous_time=now
            settled=np.isfinite(value) and np.isfinite(velocity) and abs(velocity)<=config["max_abs_dq"]
            # Do not require the aperture to match a nominal object width.
            # Stable non-full-close feedback is sufficient to accept the
            # grasp; only the full-close band is treated as empty.
            if settled and value>=config["empty_below_q"]:
                if success_since is None:
                    success_since=now
                success_samples+=1
                held_s=now-success_since
            else:
                success_since=None
                success_samples=0
                held_s=0.
            if settled and value<config["empty_below_q"]:
                empty_samples.append(sample)
            else:
                empty_samples=[]
            if now<=deadline+1e-9:
                if success_samples>=3 and held_s+1e-9>=config["success_hold_s"]:
                    status,stable="success",True
                    break
                if (len(empty_samples)>=3 and
                        empty_samples[-1]["elapsed_s"]-empty_samples[0]["elapsed_s"]>=.16-1e-9 and
                        max(p["q"] for p in empty_samples)-min(p["q"] for p in empty_samples)<=.08):
                    status,stable="empty",True
                    break
            if now>=deadline:
                break
            time.sleep(min(.04,deadline-now))
        evidence={"status":status,"stable":stable,"samples":samples,
                  "grasp_attempt_index":bundle["grasp_attempt_index"],"commanded_close_q":0.,
                  "success_q":config["success_q"],"success_tolerance_q":config["success_tolerance_q"],
                  "success_rule":"stable_aperture_at_or_above_empty_below_q",
                  "success_min_q":config["empty_below_q"],
                  "required_hold_s":config["success_hold_s"],"held_s":held_s,
                  "contact_timeout_s":config["contact_timeout_s"],
                  "scope":"stationary_after_close_only"}
        if status == "success":
            self._snapshot_grasp_success(bundle, feedback, feedback_at)
        self._atomic_json(bundle["directory"]/(bundle["token"]+"_grasp_"+str(bundle["grasp_attempt_index"])+"_contact.json"),evidence)
        self.log({"event":"grasp_contact_classified","token":bundle["token"],**evidence})
        return evidence

    def _snapshot_grasp_success(self, bundle, feedback, feedback_at):
        audit = getattr(self, "_pose_audit", None)
        if audit is not None and audit.grasp_target is not None:
            audit.capture("grasp_success", feedback, self.dds, audit.grasp_target,
                          grasp_attempt_index=bundle["grasp_attempt_index"],
                          feedback_read_monotonic_s=feedback_at)

    def _adjust_retry_paths(self,bundle,offset,current):
        """Generate only the high correction before moving; no whole-path preflight."""
        started=time.monotonic()
        index=bundle.get("corrections_completed",0)+1
        config=bundle["pregrasp_adjustment"]
        evidence={"passed":False,"correction_index":index,"offset_xy_m":offset}
        try:
            if index>config["max_corrections"]:
                raise ValueError("correction limit reached")
            delta=np.asarray(offset,dtype=float)
            if delta.shape!=(2,) or not np.isfinite(delta).all() or np.linalg.norm(delta)<1e-6:
                raise ValueError("invalid correction")
            cumulative=np.asarray(bundle.get("cumulative_offset_xy_m",[0.,0.]))+delta
            plans=copy.deepcopy(bundle["retry_base_plans"])
            self._set_source_grasp_z(plans,bundle["grasp_z_m"])
            groups={"success":[0,1,2],"retry":[0,1],
                    "unknown":bundle["grasp_retry"]["unknown_failure_source_steps"],
                    "failure":bundle["grasp_retry"]["failure_source_steps"]}
            for name,indices in groups.items():
                for i in indices:
                    for axis,value in zip(("x","y"),cumulative):
                        plans[name]["right_arm"][i][axis]+=float(value)
            feedback=self._cached_health()
            seed=feedback[0][ARM].copy()
            actual=self.motion.fk(seed)
            # XY is interpreted in the published reference, while height and
            # orientation stay at the real high pose, not an idealized old FK.
            high=waypoint(actual[1])
            high["gripper"]="hold"
            for axis,value in zip(("x","y"),cumulative):
                high[axis]=bundle["adjustment_base"]["high_pose"][axis]+float(value)
            for point in plans["retry"]["right_arm"]:
                for key in ("z","roll","pitch","yaw"):
                    point[key]=high[key]
            correction_plan={"left_arm":[waypoint(actual[0])],"right_arm":[high]}
            runtime=RuntimeSourcePaths(self.motion,bundle["gripper_open_q"],self.dds.gripper_open_sign,
                                       self._cached_health)
            step=runtime.step([correction_plan["left_arm"][0],high],seed)
            # Cache commanded apertures, not the small measured tracking
            # residual; every empty-grasp retry commands the same open target.
            correction=runtime.assemble(correction_plan,[step],seed,bundle["grasp"]["end_grip"])
            self._cached_health()
            evidence.update(passed=True,cumulative_offset_xy_m=cumulative.tolist(),
                            runtime_ik_elapsed_s=time.monotonic()-started,runtime_ik_calls=runtime.solve_count,
                            preflight_performed=False,remaining_paths_update="after_fresh_checkpoint_publish")
            return correction,{"plans":plans,"runtime":runtime},evidence
        except CachedHardwareFault:
            raise
        except Exception as exc:
            evidence.update(error=str(exc),reason="adjustment_command_rejected",
                            runtime_ik_elapsed_s=time.monotonic()-started,preflight_performed=False)
            return None,None,evidence

    def _refresh_retry_source_paths(self,bundle,correction,pending,current):
        """Update necessary source commands after fresh images are already published."""
        started=time.monotonic()
        runtime=pending["runtime"]
        plans=pending["plans"]
        before=runtime.solve_count
        seed,grip=correction["end_q"],correction["end_grip"]
        success=runtime.prefix(current["success"],plans["success"],[0,1,2],seed,grip)
        close=success["steps"][1]
        retry=runtime.prefix(current["retry"],plans["retry"],[0,1],close["samples"][-1][0],
                             close["end_grip"],fixed_end_q=seed)
        unknown=runtime.prefix(current["unknown"],plans["unknown"],
                               bundle["grasp_retry"]["unknown_failure_source_steps"],
                               close["samples"][-1][0],close["end_grip"])
        failure=runtime.prefix(current["failure"],plans["failure"],
                               bundle["grasp_retry"]["failure_source_steps"],seed,grip)
        retry_failure=runtime.prefix(current["retry_failure"],plans["failure"],
                                     bundle["grasp_retry"]["failure_source_steps"],seed,grip)
        # Retry returns to the identical high joint seed with the same open
        # gripper state; all remaining attempts can share these frozen paths.
        if (np.max(np.abs(retry["end_q"]-seed))>1e-9 or
                np.max(np.abs(retry["end_grip"]-grip))>.01):
            raise ValueError("runtime retry does not restore the shared high seed")
        attempt=dict(success=success,retry=retry,unknown=unknown,failure=failure,retry_failure=retry_failure)
        for name,compiled in attempt.items():
            indices=[i for i,step in enumerate(current[name]["steps"])
                     if step.get("ignore_right_tool_envelope",False)]
            self._mark_right_source_envelope(compiled,indices)
        remaining=bundle["grasp_retry"]["max_attempts"]-bundle["grasp_attempt_index"]
        self.log({"event":"runtime_source_paths_ready","token":bundle["token"],
                  "correction_index":bundle["corrections_completed"],
                  "runtime_ik_elapsed_s":time.monotonic()-started,
                  "runtime_ik_calls":runtime.solve_count-before,"remaining_attempts":remaining,
                  "preflight_performed":False,"frozen_target_and_init_tail":True})
        return [attempt for _ in range(remaining)]

    @staticmethod
    def _mark_right_source_envelope(compiled,indices):
        for index in indices:
            compiled["steps"][index]["ignore_right_tool_envelope"]=True
            for reverse in compiled["reverse"][index]:
                reverse["ignore_right_tool_envelope"]=True

    def _bind_grasp_descent(self,compiled,context,grasp_z_m=None):
        """Bind descent to actual high XY/orientation; retain the frozen tail.

        Rebuild the reverse prefix too, so an interrupted descent never
        retreats through samples from the old nominal source pose.
        """
        feedback=self._cached_health()
        seed=feedback[0][ARM].copy()
        poses=[p.copy() for p in self.motion.fk(seed)]
        waypoints=[waypoint(p) for p in poses]
        waypoints[1]["z"]=float(compiled["steps"][0]["targets"][1][2,3]
                                 if grasp_z_m is None else grasp_z_m)
        runtime=RuntimeSourcePaths(self.motion,3.,self.dds.gripper_open_sign,self._cached_health)
        speed=compiled["plan"].get("speed_scale",1.)
        descent=runtime.step(waypoints,seed,speed)
        # Keep an explicitly slower prechecked descent's duration.
        scale=max(1.,sum(t for _,t in compiled["steps"][0]["samples"])/
                         sum(t for _,t in descent["samples"]))
        descent["samples"]=[(q,t*scale) for q,t in descent["samples"]]
        descent["fixed_grasp_descent"]=True
        close_waypoints=copy.deepcopy(waypoints)
        close_waypoints[1]["gripper"]="close"
        close=runtime.step(close_waypoints,descent["samples"][-1][0],speed)
        prefix=runtime.assemble(compiled["plan"],[descent,close],seed,
                                compiled["steps"][0]["end_grip"])
        if compiled["steps"][0].get("ignore_right_tool_envelope",False):
            self._mark_right_source_envelope(prefix,(0,1))
        bound=dict(compiled)
        bound.update(seed=seed,steps=prefix["steps"]+compiled["steps"][2:],
                     reverse=prefix["reverse"]+compiled["reverse"][2:])
        self.log({"event":"grasp_descent_bound","token":context["token"],
                  "grasp_attempt_index":context.get("grasp_attempt_index",0),
                  "actual_high":waypoint(poses[1]),"descent_target":waypoints[1],
                  "runtime_ik_calls":runtime.solve_count,"frozen_tail_start_step":2,
                  "ignore_right_tool_envelope":bool(compiled["steps"][0].get("ignore_right_tool_envelope",False)),
                  "state":self.state()})
        return bound

    def _execute_grasp_retries(self,bundle,result,context):
        queue=bundle["retry_attempts"]
        attempt=0
        while attempt<bundle["grasp_retry"]["max_attempts"]:
            bundle["grasp_attempt_index"]=attempt
            context["grasp_attempt_index"]=attempt
            if attempt and bundle["grasp_retry"].get("confirm_after_descent",False):
                # Each actual grasp owns two 5 mm steps from the reviewed
                # high-gate depth; failed closes must not accumulate an
                # unbounded, unprepared depth across later grasp attempts.
                bundle["grasp_z_m"]=bundle["preclose_start_z_m"]
            current=queue[0]
            decision,reason=self._checkpoint_decision(bundle,post_adjustment=bool(bundle.get("corrections_completed",0)))
            result.setdefault("alignment_checks",[]).append({"grasp_attempt_index":attempt,"decision":decision,"reason":reason})
            while decision=="adjust":
                correction,pending,evidence=self._adjust_retry_paths(bundle,bundle["requested_offset_xy_m"],current)
                result.setdefault("adjustments",[]).append(evidence)
                self.log({"event":"pregrasp_adjustment","token":bundle["token"],**evidence})
                if not evidence["passed"]:
                    decision,reason="failure",evidence["reason"]
                    break
                context["branch"]="correction"
                cursor={}
                start=self.state()
                self.log({"event":"correction_start","token":bundle["token"],"state":start,**evidence})
                try:
                    self._run_cached_steps(correction["steps"],context,cursor)
                except Exception:
                    if cursor and not self.dds.fault and not self.cancelled.is_set():
                        context["branch"]="correction_recovery"
                        self._run_cached_steps(self._cached_recovery(correction,cursor)+current["failure"]["steps"],context,{})
                    raise
                bundle["corrections_completed"]=evidence["correction_index"]
                bundle["cumulative_offset_xy_m"]=evidence["cumulative_offset_xy_m"]
                bundle["pregrasp_seed"]=correction["end_q"].copy()
                end=self.state()
                self.log({"event":"correction_done","token":bundle["token"],"state":end,
                          "correction_index":evidence["correction_index"],"grasp_attempt_index":attempt,
                          "actual_delta_xy_m":[end["right_arm"][axis]-start["right_arm"][axis] for axis in ("x","y")]})
                refreshed=False
                prior_current=current
                def refresh_remaining():
                    nonlocal queue,current,refreshed
                    queue=self._refresh_retry_source_paths(bundle,correction,pending,current)
                    current=queue[0]
                    refreshed=True
                try:
                    # Publish fresh images first. Necessary source IK overlaps
                    # the human/agent's image review; it never delays XY motion.
                    decision,reason=self._checkpoint_decision(bundle,post_adjustment=True,
                                                               after_publish=refresh_remaining)
                except Exception:
                    # Fresh images may already be under visual review. A source
                    # update error must keep healthy control at this high pose;
                    # do not silently execute a return while that gate is open.
                    context["branch"]="correction_source_update_failed_hold"
                    raise
                if not refreshed:
                    # Capture/decision-channel failure can return before the
                    # callback. Undo the correction before the old return path.
                    # Keep recovery outside the try so failure is not retried.
                    context["branch"]="correction_checkpoint_failure_recovery"
                    self._run_cached_steps(self._cached_recovery(correction,cursor)+prior_current["failure"]["steps"],context,{})
                    result.update(outcome="alignment_failed_returned",return_reason=reason)
                    return
                result["alignment_checks"].append({"grasp_attempt_index":attempt,"decision":decision,"reason":reason})
            if (decision=="retry_high" and reason=="external_decision" and
                    bundle["grasp_retry"].get("alignment_failure_policy")=="recheck_high_then_grasp"):
                if bundle.get("corrections_completed",0)>=bundle.get("pregrasp_adjustment",{}).get("max_corrections",3):
                    decision="failure"
                    reason="high_correction_limit_without_alignment"
                else:
                    bundle["source_review_cycle"]=bundle.get("source_review_cycle",0)+1
                    if bundle["source_review_cycle"]>bundle["grasp_retry"]["max_high_rechecks"]:
                        result.update(outcome="alignment_pending_high",return_reason="high_recheck_limit_without_three_xy_corrections")
                        return
                    continue
            if decision!="success":
                result["return_reason"]=reason
                context["branch"]="alignment_failure"
                self._cached_health()
                self._run_cached_steps(current["failure"]["steps"],context,{})
                result["outcome"]="alignment_failed_returned"
                return
            context["branch"]="grasp_close"
            self._cached_health()
            cursor={}
            grasp_success=current["success"]
            try:
                grasp_success=self._bind_grasp_descent(grasp_success,context,bundle["grasp_z_m"])
                if bundle["grasp_retry"].get("confirm_after_descent",False):
                    # Review at low pose, and close either on approval or
                    # automatically after the user's two 5 mm increments.
                    bundle["preclose_start_z_m"]=bundle["grasp_z_m"]
                    self._run_cached_steps(grasp_success["steps"][:1],context,cursor)
                    bundle["low_check_index"]=0
                    bundle["low_pose_seed"]=grasp_success["steps"][0]["samples"][-1][0].copy()
                    retry_high=False
                    while True:
                        low_decision,low_reason=self._checkpoint_decision(bundle,preclose=True)
                        result.setdefault("low_alignment_checks",[]).append({
                            "grasp_attempt_index":attempt,"low_check_index":bundle["low_check_index"],
                            "grasp_z_m":bundle["grasp_z_m"],"decision":low_decision,"reason":low_reason})
                        if low_decision=="retry_high":
                            if bundle.get("corrections_completed",0)>=bundle.get("pregrasp_adjustment",{}).get("max_corrections",3):
                                low_decision="failure"
                                low_reason="high_correction_limit_without_alignment"
                            else:
                                context["branch"]="preclose_rise_for_high_review"
                                rise=self._rise_for_high_review(bundle,current)
                                self._run_cached_steps(rise["steps"],context,{})
                                bundle["pregrasp_seed"]=rise["end_q"].copy()
                                bundle["source_review_cycle"]=bundle.get("source_review_cycle",0)+1
                                bundle["grasp_z_m"]=bundle["preclose_start_z_m"]
                                result.setdefault("high_rechecks",[]).append({
                                    "grasp_attempt_index":attempt,"cycle":bundle["source_review_cycle"],
                                    "correction_index":bundle.get("corrections_completed",0)})
                                if bundle["source_review_cycle"]>bundle["grasp_retry"]["max_high_rechecks"]:
                                    result.update(outcome="alignment_pending_high",return_reason="high_recheck_limit_without_three_xy_corrections")
                                    return
                                retry_high=True
                                break
                        if low_decision=="success":
                            close_cursor={}
                            self._run_cached_steps(grasp_success["steps"][1:2],context,close_cursor)
                            cursor.update(close_cursor,step=1)
                            break
                        if low_decision!="adjust_z":
                            context["branch"]="empty_grasp_rise_open"
                            self._run_cached_steps(current["retry"]["steps"],context,{})
                            context["branch"]="low_alignment_failure"
                            self._run_cached_steps(current["retry_failure"]["steps"],context,{})
                            result.update(outcome="alignment_failed_returned",return_reason=low_reason)
                            return
                        if bundle["low_check_index"]>=bundle["grasp_retry"]["max_low_z_adjustments"]:
                            close_cursor={}
                            self._run_cached_steps(grasp_success["steps"][1:2],context,close_cursor)
                            cursor.update(close_cursor,step=1)
                            break
                        # A 5 mm command increment is automatic; there is no
                        # second decision, XY move, opening or high return.
                        # The low image does not choose a numeric Z.  The
                        # service applies exactly one automatic 5 mm descent.
                        # After two such descents, close at the current pose.
                        bundle["grasp_z_m"]=round(bundle["grasp_z_m"]-.005,9)
                        grasp_success=self._lower_grasp_step(grasp_success,context,bundle["grasp_z_m"])
                        cursor={}
                        self._run_cached_steps(grasp_success["steps"][:1],context,cursor)
                        bundle["low_check_index"]+=1
                        bundle["low_pose_seed"]=grasp_success["steps"][0]["samples"][-1][0].copy()
                        if bundle["low_check_index"]>=bundle["grasp_retry"]["max_low_z_adjustments"]:
                            self.log({"event":"preclose_auto_close_after_z_limit",
                                      "token":bundle["token"],"low_check_index":bundle["low_check_index"]})
                            close_cursor={}
                            self._run_cached_steps(grasp_success["steps"][1:2],context,close_cursor)
                            cursor.update(close_cursor,step=1)
                            break
                    if retry_high:
                        # No close occurred. Keep the actual-grasp counter and
                        # remaining compiled attempts; publish a fresh high gate.
                        continue
                else:
                    self._run_cached_steps(grasp_success["steps"][:2],context,cursor)
            except Exception as exc:
                if (not bundle["grasp_retry"].get("confirm_after_descent",False) and
                        cursor and not isinstance(exc,CachedHardwareFault) and not self.dds.fault and
                        self.dds.enabled and not self.cancelled.is_set()):
                    context["branch"]="grasp_close_recovery"
                    self._run_cached_steps(self._cached_recovery(grasp_success,cursor)+current["failure"]["steps"],context,{})
                raise
            audit = getattr(self, "_pose_audit", None)
            if audit is not None:
                audit.grasp_target = {k: grasp_success["steps"][1]["waypoints"][1][k]
                                      for k in ("x", "y", "z", "roll", "pitch", "yaw")}
            contact=self._classify_grasp_contact(bundle)
            result.setdefault("grasp_attempts",[]).append(contact)
            if contact["status"]=="success":
                context["branch"]="success"
                self._run_placement_steps(grasp_success["steps"][2:],bundle,result,context,{})
                result["outcome"]="placed"
                return
            if contact["status"]=="unknown":
                context["branch"]="unknown_contact_source_release"
                if bundle["grasp_retry"].get("confirm_after_descent",False):
                    # An unsuccessful close may still have an object blocked
                    # between the fingers. Release at source before rising,
                    # then use the same high-image retry as an empty grasp.
                    self._run_cached_steps(current["unknown"]["steps"][:1],context,{})
                else:
                    result["return_reason"]="unverified_gripper_contact"
                    self._run_cached_steps(current["unknown"]["steps"],context,{})
                    result["outcome"]="unverified_grasp_returned"
                    return
            context["branch"]="empty_grasp_rise_open"
            self._run_cached_steps(current["retry"]["steps"],context,{})
            bundle["pregrasp_seed"]=current["retry"]["end_q"].copy()
            attempt+=1
            correction_limit=(bundle["grasp_retry"].get("alignment_failure_policy")=="recheck_high_then_grasp" and
                              bundle.get("corrections_completed",0)>=bundle.get("pregrasp_adjustment",{}).get("max_corrections",3))
            if attempt>=bundle["grasp_retry"]["max_attempts"] or correction_limit:
                result["return_reason"]="grasp_attempt_limit_reached"
                context["branch"]="empty_grasp_limit_return"
                self._run_cached_steps(current["retry_failure"]["steps"],context,{})
                result["outcome"]="empty_grasp_limit_returned"
                return
            queue=queue[1:]

    def _rise_for_high_review(self,bundle,current):
        """Open, unclosed source: vertical rise with unchanged commanded XY/RPY."""
        self._cached_health()
        with self.dds.lock:
            seed=self.dds.arm_target.copy()
        poses=[p.copy() for p in self.motion.fk(seed)]
        points=[waypoint(p) for p in poses]
        points[1]["z"]=max(points[1]["z"],self._adjustment_reference_right_arm(bundle)["z"])
        runtime=RuntimeSourcePaths(self.motion,bundle["gripper_open_q"],self.dds.gripper_open_sign,self._cached_health)
        step=runtime.step(points,seed,current["retry"]["plan"].get("speed_scale",1.))
        # Keep all runtime surface/hardware checks; no opening, closure or
        # lateral correction is issued on the way up.
        return runtime.assemble(current["retry"]["plan"],[step],seed,current["retry"]["end_grip"])

    def _lower_grasp_step(self,compiled,context,z):
        """Lower the retained command 5 mm without rebasing on tracking lag."""
        self._cached_health()
        with self.dds.lock:
            seed=self.dds.arm_target.copy()
        poses=[p.copy() for p in self.motion.fk(seed)]
        waypoints=[waypoint(p) for p in poses]
        waypoints[1]["z"]=float(z)
        runtime=RuntimeSourcePaths(self.motion,3.,self.dds.gripper_open_sign,self._cached_health)
        descent=runtime.step(waypoints,seed,compiled["plan"].get("speed_scale",1.))
        descent.update(fixed_grasp_descent=True,continue_arm_command=True)
        close_waypoints=copy.deepcopy(waypoints)
        close_waypoints[1]["gripper"]="close"
        close=runtime.step(close_waypoints,descent["samples"][-1][0])
        prefix=runtime.assemble(compiled["plan"],[descent,close],seed,compiled["steps"][0]["end_grip"])
        if compiled["steps"][0].get("ignore_right_tool_envelope",False):
            self._mark_right_source_envelope(prefix,(0,1))
        result=dict(compiled)
        result.update(seed=seed,steps=prefix["steps"]+compiled["steps"][2:],
                      reverse=prefix["reverse"]+compiled["reverse"][2:])
        self.log({"event":"low_grasp_z_step","token":context["token"],"step_m":.005,
                  "target":waypoints[1],"state":self.state()})
        return result

    @staticmethod
    def _validate_adjustment_config(bundle,config):
        if "grasp_retry" not in bundle and ("auto_grasp_gate" not in bundle or bundle["auto_grasp_gate"]["after_success_step"]!=2):
            raise ValueError("pregrasp adjustment requires automatic gate after success step 2")
        config.pop("max_offset_m",None)
        max_corrections=config.setdefault("max_corrections",3)
        if isinstance(max_corrections,bool) or not isinstance(max_corrections,int) or not 1<=max_corrections<=3:
            raise ValueError("max_corrections must be an integer between 1 and 3")
        post_check=config.setdefault("require_post_adjustment_check",True)
        post_timeout=config.setdefault("post_adjustment_timeout_s",300.)
        if post_check is not True:
            raise ValueError("require_post_adjustment_check must be true")
        if (isinstance(post_timeout,bool) or not isinstance(post_timeout,(int,float)) or
                not np.isfinite(post_timeout) or not 1<=post_timeout<=300):
            raise ValueError("post_adjustment_timeout_s must be between 1 and 300")
        if "grasp_retry" in bundle:
            if config.get("success_source_steps")!=[0,1,2]:
                raise ValueError("grasp_retry corrections require success_source_steps [0,1,2]")
            return
        if config.get("success_source_steps")!=[0,1,2] or config.get("gate_failure_source_steps")!=[0,1,2,3,4]:
            raise ValueError("pregrasp adjustment requires explicit supported source step lists")
        success=bundle["success"]["plan"]
        failure=bundle["gate_failure"]["plan"]
        for plan,actions in ((success,["hold","close","hold"]),
                             (failure,["hold","open","hold","close","hold"])):
            if len(plan["left_arm"])!=1 or len(plan["right_arm"])<=len(actions):
                raise ValueError("pregrasp adjustment supports one held left waypoint and explicit right prefix/tail")
            if any(wp.get("gripper","hold")!=action for wp,action in zip(plan["right_arm"],actions)):
                raise ValueError("pregrasp adjustment source actions do not match supported shape")
        source=success["right_arm"][0]
        for plan,key in ((success,"success_source_steps"),(failure,"gate_failure_source_steps")):
            if any(np.linalg.norm([plan["right_arm"][i][axis]-source[axis] for axis in ("x","y")])>.001
                   for i in config[key]):
                raise ValueError("all explicitly shifted source steps must share source XY")

    @staticmethod
    def _checkpoint_suffix(correction_index):
        if correction_index==0:
            return ""
        return "_post_adjustment"+("" if correction_index==1 else "_"+str(correction_index))

    def _adjustment_reference_right_arm(self,bundle):
        """Exact high target to which the next incremental XY offset is added."""
        base=bundle.get("adjustment_base")
        reference=(copy.deepcopy(base["high_pose"]) if base is not None
                   else waypoint(self.motion.fk(bundle["grasp"]["end_q"])[1]))
        for axis,offset in zip(("x","y"),bundle.get("cumulative_offset_xy_m",[0.,0.])):
            reference[axis]+=float(offset)
        return reference

    def _apply_pregrasp_adjustment(self,bundle,offset):
        """Build changed source commands, retaining the prechecked target/init tails."""
        started=time.monotonic()
        config=bundle["pregrasp_adjustment"]
        correction_index=bundle.get("adjustment_attempts",0)+1
        evidence={"offset_xy_m":offset,"correction_index":correction_index,
                  "offset_reference":"checkpoint_adjustment_reference_right_arm_xy",
                  "max_corrections":config["max_corrections"],"passed":False,
                  "preflight_performed":False}
        if correction_index>config["max_corrections"]:
            return dict(evidence,reason="correction_limit_reached",runtime_ik_elapsed_s=0.)
        bundle["adjustment_attempts"]=correction_index
        try:
            if (not isinstance(offset,list) or len(offset)!=2 or
                    any(isinstance(v,bool) or not isinstance(v,(int,float)) for v in offset)):
                raise ValueError("invalid pregrasp XY offset")
            delta=np.asarray(offset,dtype=float)
            if not np.isfinite(delta).all() or np.linalg.norm(delta)<1e-6:
                raise ValueError("invalid or zero pregrasp XY offset")
            # Keep pristine world-space tails. Repeated corrections accumulate
            # only over the source prefix, never over the target or init pose.
            base=bundle.setdefault("adjustment_base",{
                "success_plan":copy.deepcopy(bundle["success"]["plan"]),
                "gate_failure_plan":copy.deepcopy(bundle["gate_failure"]["plan"]),
                "success_compiled":copy.deepcopy(bundle["success"]),
                "gate_failure_compiled":copy.deepcopy(bundle["gate_failure"]),
                "high_pose":waypoint(self.motion.fk(bundle["grasp"]["end_q"])[1])})
            current_seed=self._cached_health()[0][ARM].copy()
            evidence["adjustment_reference_right_arm"]=self._adjustment_reference_right_arm(bundle)
            cumulative=np.asarray(bundle.get("cumulative_offset_xy_m",[0.,0.]))+delta
            success=copy.deepcopy(base["success_plan"])
            failure=copy.deepcopy(base["gate_failure_plan"])
            for plan,key in ((success,"success_source_steps"),(failure,"gate_failure_source_steps")):
                for index in config[key]:
                    for axis,change in zip(("x","y"),cumulative):
                        plan["right_arm"][index][axis]+=float(change)
            start_pose=copy.deepcopy(base["high_pose"])
            for axis,change in zip(("x","y"),cumulative):
                start_pose[axis]+=float(change)
            runtime=RuntimeSourcePaths(self.motion,bundle["gripper_open_q"],self.dds.gripper_open_sign,
                                       self._cached_health)
            correction=runtime.step([waypoint(self.motion.fk(current_seed)[0]),start_pose],current_seed,
                                    success.get("speed_scale",1.))
            corrected_seed=correction["samples"][-1][0]
            source=runtime.prefix(base["success_compiled"],success,config["success_source_steps"],
                                  corrected_seed,bundle["grasp"]["end_grip"])
            success["right_arm"].insert(0,start_pose)
            prepared_success=runtime.assemble(success,[correction]+source["steps"],current_seed,
                                               bundle["grasp"]["end_grip"])
            evidence.update(cumulative_offset_xy_m=cumulative.tolist(),
                            adjusted_plans={"success_plan":success,"gate_failure_plan":failure,
                                            "after_success_step":3})
            gate_step=prepared_success["steps"][3]
            gate_seed=gate_step["samples"][-1][0]
            prepared_failure=runtime.prefix(base["gate_failure_compiled"],failure,
                config["gate_failure_source_steps"],gate_seed,gate_step["end_grip"])
            self._cached_health()
            # Freeze the prior route before replacing the current correction.
            # The newest reverse correction is prepended to every earlier one.
            previous_return=copy.deepcopy(bundle.get("adjust_failure",bundle["failure"])["steps"])
            reverse_correction=copy.deepcopy(prepared_success["reverse"][0][1])
            bundle["success"]=prepared_success
            bundle["gate_failure"]=prepared_failure
            bundle["auto_grasp_gate"]["after_success_step"]=3
            bundle["correction_return_steps"]=previous_return
            bundle["adjust_failure"]={"seed":prepared_success["steps"][0]["samples"][-1][0].copy(),
                "steps":[reverse_correction]+previous_return}
            bundle["cumulative_offset_xy_m"]=cumulative.tolist()
            evidence.update(passed=True,reason="runtime_source_paths_ready",
                            runtime_ik_calls=runtime.solve_count,
                            paths={"success":self._steps_summary(prepared_success["steps"]),
                                   "gate_failure":self._steps_summary(prepared_failure["steps"]),
                                   "adjust_failure":self._steps_summary(bundle["adjust_failure"]["steps"])},
                            post_adjustment_return="frozen_reverse_corrections_then_original_failure")
        except CachedHardwareFault:
            raise
        except Exception as exc:
            evidence.update(reason="adjustment_command_rejected",error=str(exc))
        evidence["runtime_ik_elapsed_s"]=time.monotonic()-started
        suffix="_adjustment"+("" if correction_index==1 else "_"+str(correction_index))
        self._atomic_json(bundle["directory"]/(bundle["token"]+suffix+".json"),evidence)
        self.log({"event":"pregrasp_adjustment","token":bundle["token"],**evidence})
        return evidence

    def _verify_cached_start(self, expected, grippers=None):
        """Execution-entry preflight only; never gate an in-flight branch on error."""
        feedback=self._cached_health()
        q,dq,_,_=feedback
        actual = q[ARM]
        errors = self.motion.errors(self.motion.fk(actual),self.motion.fk(expected))
        if (np.max(np.abs(actual-expected))>.05 or
                any(p>WAYPOINT_POSITION_TOLERANCE_M or r>=.1 for p,r in errors) or
                np.max(np.abs(dq[ARM]))>=.08):
            raise RuntimeError(f"cached start does not match feedback: {errors}")
        if grippers is not None and np.max(np.abs(q[GRIP]-grippers))>.15:
            raise RuntimeError("cached initial gripper state changed")
        return feedback

    def _settle_before_gripper(self, action_arms, ignore_right_tool_envelope=False):
        """Allow a fixed settling interval, without any arrival/error threshold."""
        deadline=time.monotonic()+1.
        while True:
            if self.cancelled.is_set():
                raise RuntimeError("cached motion cancelled")
            feedback=self._cached_health()
            joints=feedback[0][ARM].copy()
            poses=[p.copy() for p in self.motion.fk(joints)]
            for j in action_arms:
                if not (j==1 and ignore_right_tool_envelope):
                    self.guard.check_gripper(poses[j])
            remaining=deadline-time.monotonic()
            if remaining<=0:
                return joints,poses
            time.sleep(min(.02,remaining))

    def _run_cached_steps(self, steps, context, cursor):
        completed = []
        for i,step in enumerate(steps):
            ignore_right_envelope=step.get("ignore_right_tool_envelope",False)
            cursor.update(step=i,sample=0,action_started=False,settling=False)
            q,_,_,_=self._cached_health()
            previous=q[ARM].copy()
            if step.get("continue_arm_command",False):
                with self.dds.lock:
                    previous=self.dds.arm_target.copy()
            segment_start=[p.copy() for p in self.motion.fk(previous)]
            targets=step["targets"]
            samples=step["samples"]
            clearance_before_sample=step.get("clearance_before_sample")
            if not step.get("gripper_targets") and samples and "seed" in step:
                nominal=self.motion.fk(step["seed"])
                displacement=targets[1][:3,3]-nominal[1][:3,3]
                rotation=self.motion.errors([nominal[1]],[targets[1]])[0][1]
                observer=self.motion.errors([nominal[0]],[targets[0]])[0]
                vertical=(displacement[2]>.001 and
                          np.linalg.norm(displacement[:2])<=self.guard.xy_tolerance and
                          rotation<=self.guard.rotation_tolerance and
                          observer[0]<=.0005 and observer[1]<=.003 and
                          self.guard.clearance(targets[1])>=self.guard.travel_clearance)
                empty_retry=(context.get("branch")=="empty_grasp_rise_open" and i==0)
                grasp_lift=(context.get("branch")=="success" and
                            "grasp_attempt_index" in context and i==0
                            and not step.get("release_descent",False))
                unknown_lift=(context.get("branch")=="unknown_contact_source_release" and i==1)
                release_lift=(context.get("branch")=="success" and i>0 and vertical and
                              steps[i-1].get("gripper_targets",{}).get("right",0)>0)
                rebuild_vertical=empty_retry or grasp_lift or unknown_lift or release_lift
                if vertical and not rebuild_vertical:
                    try:
                        self.guard.check_segment(segment_start[1],self.motion.fk(samples[0][0])[1])
                    except GuardViolation:
                        rebuild_vertical=True
                if rebuild_vertical:
                    # A missed descent must never be corrected sideways or
                    # downward before a retry rise. Leave from the actual
                    # XY/RPY, reach the original high height, then join its
                    # frozen endpoint. Every new joint segment still passes
                    # the normal collision and hardware checks below.
                    high=[waypoint(p) for p in segment_start]
                    high[1]["z"]=max(high[1]["z"],float(targets[1][2,3]))
                    # Preserve the measured rotation on departure and allow
                    # for its tool projection before reconnecting sideways.
                    high[1]["z"]+=max(0.,self.guard.travel_clearance-
                                       self.guard.clearance(matrix(high[1]))+.001)
                    runtime=RuntimeSourcePaths(self.motion,3.,self.dds.gripper_open_sign,
                                               self._cached_health)
                    lift=runtime.step(high,previous)
                    connector=runtime.step([waypoint(p) for p in targets],
                        lift["samples"][-1][0],fixed_end_q=samples[-1][0])
                    rebuilt=lift["samples"]+connector["samples"]
                    clearance_before_sample=len(lift["samples"])
                    scale=max(1.,sum(t for _,t in samples)/sum(t for _,t in rebuilt))
                    samples=[(target,duration*scale) for target,duration in rebuilt]
                    self.log({"event":"cached_vertical_departure","token":context["token"],
                              "branch":context["branch"],"step":i,
                              "post_release":release_lift,
                              "actual_departure":waypoint(segment_start[1]),
                              "vertical_high":high[1]})
            if step.get("gripper_targets"):
                # Preserve the preceding arm commands through open/close.
                # hold() would replace them with possibly lagging feedback,
                # discarding the descent endpoint while retaining stale tau.
                previous,segment_start=self._settle_before_gripper(
                    [j for j,side in enumerate(("left","right")) if side in step["gripper_targets"]],
                    ignore_right_tool_envelope=ignore_right_envelope)
                intended=self.motion.errors(self.motion.fk(step.get("seed",previous)),targets)
                held=[side in step["gripper_targets"] or (p<=.0005 and r<=.003)
                      for side,(p,r) in zip(("left","right"),intended)]
                targets=[segment_start[j] if held[j] else target for j,target in enumerate(targets)]
                samples=[]
                if not all(held):
                    for target,duration in step["samples"]:
                        target=target.copy()
                        for j,stationary in enumerate(held):
                            if stationary:target[7*j:7*j+7]=previous[7*j:7*j+7]
                        samples.append((target,duration))
            active=[p>.0005 or r>.003 for p,r in self.motion.errors(segment_start,targets)]
            if step.get("fixed_grasp_descent"):
                # Left observer tracking residual is not a request to move it.
                active=[False,True]
            with self.dds.lock:
                idle_target=self.dds.arm_target.copy()
                idle_tau=self.dds.tau.copy()
            for sample_index,(target,duration) in enumerate(samples):
                cursor["sample"]=sample_index
                if sample_index==clearance_before_sample:
                    # No camera/operator gate or settling wait: refuse the
                    # lateral connector unless live tool clearance is high.
                    lifted=self.motion.fk(self._cached_health()[0][ARM])[1]
                    clearance=self.guard.clearance(lifted)
                    if not np.isfinite(clearance) or clearance<self.guard.travel_clearance:
                        raise GuardViolation("vertical departure did not reach transport clearance")
                    self.log({"event":"vertical_departure_clearance_verified",
                              "token":context["token"],"branch":context["branch"],
                              "step":i,"clearance_m":float(clearance),
                              "actual_right_arm":waypoint(lifted)})
                self._check_runtime_joint_segment(previous,target,ignore_right_tool_envelope=ignore_right_envelope)
                start=time.monotonic()
                while True:
                    if self.cancelled.is_set():
                        raise RuntimeError("cached motion cancelled")
                    self._cached_health()
                    alpha=min(1.,(time.monotonic()-start)/duration)
                    blend=alpha*alpha*(3-2*alpha)
                    command=previous+(target-previous)*blend
                    feedback=self._cached_health()
                    measured=self.motion.fk(feedback[0][ARM])
                    desired=self.motion.fk(command)
                    for j,(origin,wanted,actual) in enumerate(zip(segment_start,desired,measured)):
                        if not active[j]:
                            continue
                        if (j==1 and ignore_right_envelope and
                                min(self.guard.clearance(wanted),self.guard.clearance(actual))<self.guard.travel_clearance):
                            continue
                        low=min(self.guard.clearance(origin),self.guard.clearance(wanted))<self.guard.travel_clearance
                        self._check_feedback(origin,wanted,actual,.009 if low else .018,feedback,
                            dict(context,step=i,sample=sample_index,alpha=alpha,duration_s=duration,
                                 phase="cached_interpolation",arm=("left","right")[j]))
                    tau=self.motion.tau(command)
                    for j,moving in enumerate(active):
                        if not moving:
                            command[7*j:7*j+7]=idle_target[7*j:7*j+7]
                            tau[7*j:7*j+7]=idle_tau[7*j:7*j+7]
                    self.dds.command_arm(command,tau)
                    if alpha>=1:
                        break
                    time.sleep(.01)
                previous=target
                cursor["sample"]=sample_index+1
            cursor["settling"]=True
            if self.cancelled.is_set():
                raise RuntimeError("cached motion cancelled")
            feedback=self._cached_health()
            measured=self.motion.fk(feedback[0][ARM])
            for j,(origin,wanted,actual) in enumerate(zip(segment_start,targets,measured)):
                if not active[j]:
                    continue
                if (j==1 and ignore_right_envelope and
                        min(self.guard.clearance(wanted),self.guard.clearance(actual))<self.guard.travel_clearance):
                    continue
                low=min(self.guard.clearance(origin),self.guard.clearance(wanted))<self.guard.travel_clearance
                self._check_feedback(origin,wanted,actual,.009 if low else .018,feedback,
                    dict(context,step=i,sample=len(samples)-1,alpha=1.,
                         phase="cached_waypoint_end",arm=("left","right")[j]))
            # Position, orientation and velocity are observations, not runtime
            # arrival gates. Keep the preflight path and collision feedback checks.
            errors=self.motion.errors(measured,step["targets"])
            cursor["action_started"]=True
            grips={side:self.gripper(side,target,
                        ignore_right_tool_envelope=ignore_right_envelope and side=="right",
                        pose_audit_release=(
                            {"original_target": step["waypoints"][1]["pose_audit_release"],
                             "effective_target": {k: step["waypoints"][1][k] for k in
                                                  ("x", "y", "z", "roll", "pitch", "yaw")},
                             "grasp_attempt_index": context.get("grasp_attempt_index")}
                            if side=="right" and context.get("branch")=="success"
                            and not context.get("release_approved_before_open",False)
                            and "pose_audit_release" in step["waypoints"][1] else None))
                   for side,target in step.get("gripper_targets",{}).items()}
            if (context.get("require_release_open_before_continuation") and
                    context.get("branch")=="success" and "right" in grips and
                    step["waypoints"][1].get("gripper")=="open" and
                    not grips["right"]["reached"]):
                raise RuntimeError("capture release opening not reached; sequence handoff stopped")
            completed.append({"step":i,"errors":errors,"grippers":grips})
            self.log({"event":"cached_waypoint_done","token":context["token"],
                      "branch":context["branch"],
                      "correction_index":context.get("correction_index",0),**completed[-1]})
            if context.get("branch")=="grasp_close":
                self.log({"event":"grasp_arm_snapshot","token":context["token"],
                          "grasp_attempt_index":context.get("grasp_attempt_index",0),
                          "step":i,"planned_waypoint":waypoint(step["targets"][1]),
                          "state":self.state()})
        return completed

    def _cached_recovery(self, compiled, cursor):
        """Select frozen reverse samples; never solve a new IK target."""
        self._cached_health()
        i=cursor["step"]
        step=compiled["steps"][i]
        count=min(cursor["sample"],len(step["samples"]))
        if cursor.get("settling") and not cursor["action_started"]:
            # A commanded final sample is not a verified reached pose. Retreat
            # toward its prior frozen boundary rather than retrying the miss.
            count=max(0,count-1)
        anchor=step["seed"] if count==0 else step["samples"][count-1][0]
        q,_,_,_=self.dds.state()
        exempt=step.get("ignore_right_tool_envelope",False)
        self._check_runtime_joint_segment(q[ARM],anchor,ignore_right_tool_envelope=exempt)
        poses=[p.copy() for p in self.motion.fk(anchor)]
        duration=step["samples"][min(count,len(step["samples"])-1)][1]
        connector={"seed":q[ARM].copy(),"samples":[(anchor.copy(),max(duration,.5))],
                   "targets":poses,"waypoints":[waypoint(p) for p in poses],"gripper_targets":{}}
        if exempt:
            connector["ignore_right_tool_envelope"]=True
        reverse=[connector]
        undo,motion=compiled["reverse"][i]
        if cursor["action_started"] and undo["gripper_targets"]:
            reverse.append(undo)
        if count:
            partial=dict(motion)
            partial["samples"]=motion["samples"][-count:]
            reverse.append(partial)
        for prior in range(i-1,-1,-1):
            undo,motion=compiled["reverse"][prior]
            if undo["gripper_targets"]:
                reverse.append(undo)
            reverse.append(motion)
        return reverse

    def _checkpoint_decision(self,bundle,post_adjustment=False,after_publish=None,preclose=False):
        entered=time.monotonic()
        token=bundle["token"]
        directory=bundle["directory"]
        correction_index=bundle.get("corrections_completed",0) if post_adjustment or preclose else 0
        suffix=self._checkpoint_suffix(correction_index)
        attempt_index=bundle.get("grasp_attempt_index",0)
        if "grasp_retry" in bundle:
            suffix="_attempt_"+str(attempt_index)+"_correction_"+str(correction_index)
            if preclose:
                suffix+="_preclose_"+str(bundle.get("low_check_index",0))
            if bundle.get("source_review_cycle",0):
                suffix+="_recheck_"+str(bundle["source_review_cycle"])
        timeout=(bundle["pregrasp_adjustment"]["post_adjustment_timeout_s"] if post_adjustment
                 else bundle["decision_timeout_s"])
        decision_path=directory/(token+suffix+"_decision.json")
        receipt_path=directory/(token+suffix+"_decision_receipt.json")
        checkpoint_id=secrets.token_hex(16)
        channel=bundle.get("decision_channel",{})
        wait_indefinitely=channel.get("mode")=="serial"
        if wait_indefinitely:
            timeout=None
        created_at=time.time()
        def received(decision,reason,accepted=False):
            receipt={"token":token,"worker_id":channel.get("worker_id"),
                     "checkpoint_id":checkpoint_id,"correction_index":correction_index,
                     "accepted":accepted,"decision":decision,"reason":reason,
                     "decision_path":str(decision_path),"receipt_path":str(receipt_path),
                     "received_at_unix_s":time.time(),"elapsed_s":time.monotonic()-entered,
                     "motion_executed":False}
            if "grasp_retry" in bundle:
                receipt["grasp_attempt_index"]=attempt_index
                receipt["grasp_z_m"]=bundle["grasp_z_m"]
                if preclose:
                    receipt.update(stage="preclose",low_check_index=bundle["low_check_index"])
            self._atomic_json(receipt_path,receipt)
            self.log({"event":"checkpoint_decision_received",**receipt})
            return decision,reason
        if decision_path.exists():
            return received("failure","decision_precedes_checkpoint")
        try:
            image_dir="checkpoint_images" if not post_adjustment else "post_adjustment_images_"+str(correction_index)
            if preclose:
                image_dir="preclose_attempt_"+str(attempt_index)+"_check_"+str(bundle["low_check_index"])
            if bundle.get("source_review_cycle",0):
                image_dir+="_recheck_"+str(bundle["source_review_cycle"])
            observation=self.capture(self._execution_cameras(bundle),self._cached_image_directory(bundle,image_dir))
        except Exception as exc:
            self.log({"event":"checkpoint_camera_failed","token":token,"error":str(exc)})
            return received("failure","camera_failed")
        actual_state=self.state()
        feedback=self._cached_health()
        planned=(self.motion.fk(bundle["low_pose_seed"]) if preclose else
                 self.motion.fk(bundle["pregrasp_seed"]) if "grasp_retry" in bundle else
                 bundle["success"]["steps"][0]["targets"] if post_adjustment
                 else bundle["grasp"]["steps"][-1]["targets"])
        errors=self.motion.errors(self.motion.fk(feedback[0][ARM]),planned)
        deadline=None if wait_indefinitely else time.monotonic()+timeout
        created_at=time.time()
        checkpoint={"token":token,"stage":"preclose" if preclose else ("post_adjustment" if post_adjustment else "pregrasp"),
             "physical_actor":self.active_arm, "camera_order":self._execution_cameras(bundle),
             "camera_selection":bundle.get("camera_selection",camera_selection(self.active_arm)),
             "checkpoint_id":checkpoint_id,"worker_id":channel.get("worker_id"),
             "created_at_unix_s":created_at,"deadline_unix_s":None if wait_indefinitely else created_at+timeout,
             "wait_indefinitely":wait_indefinitely,
             "receipt_path":str(receipt_path),
             "max_offset_m":None,
             "correction_index":correction_index,
             "corrections_remaining":bundle.get("pregrasp_adjustment",{}).get("max_corrections",0)-correction_index,
             "cumulative_offset_xy_m":bundle.get("cumulative_offset_xy_m",[0.,0.]),
             "observation":observation,"decision_timeout_s":timeout,
             "actual_state":actual_state,"actual_right_arm":actual_state["right_arm"],
             "planned_right_arm":waypoint(planned[1]),
             "adjustment_reference_right_arm":(self._adjustment_reference_right_arm(bundle)
                                                if "pregrasp_adjustment" in bundle else None),
             "plan_errors":[{"position_m":p,"orientation_rad":r} for p,r in errors],
             "decision_path":str(decision_path)}
        checkpoint_path=directory/(token+suffix+"_checkpoint.json")
        if "grasp_retry" in bundle:
            checkpoint["grasp_attempt_index"]=attempt_index
            checkpoint["preclose_high_recheck_supported"]=bundle["grasp_retry"].get("alignment_failure_policy")=="recheck_high_then_grasp"
            checkpoint["grasp_z_m"]=bundle["grasp_z_m"]
            checkpoint["grasp_z_options_m"]=bundle["grasp_z_options_m"]
            checkpoint["ignore_grasp_tool_envelope"]=bundle["grasp_retry"].get("ignore_grasp_tool_envelope",False)
            low_pose=matrix(actual_state["right_arm"])
            clearances=[]
            for z in bundle["grasp_z_options_m"]:
                low_pose[2,3]=z
                clearance=self.guard.clearance(low_pose)
                clearances.append({"grasp_z_m":z,"tool_clearance_m":clearance,
                    "meets_surface_minimum":clearance>=self.guard.minimum_clearance})
            checkpoint["grasp_z_clearances_m"]=clearances
            checkpoint["minimum_surface_clearance_m"]=self.guard.minimum_clearance
            checkpoint["grasp_z_surface_safe_options_m"]=[c["grasp_z_m"] for c in clearances
                                                           if c["meets_surface_minimum"]]
            checkpoint["grasp_z_selectable_options_m"]=(bundle["grasp_z_options_m"]
                if checkpoint["ignore_grasp_tool_envelope"] else checkpoint["grasp_z_surface_safe_options_m"])
            if preclose:
                checkpoint["preclose_high_recheck_supported"]=bundle["grasp_retry"].get("alignment_failure_policy")=="recheck_high_then_grasp"
                checkpoint["low_check_index"]=bundle.get("low_check_index",0)
                checkpoint["low_z_step_m"]=.005
                checkpoint["max_low_z_adjustments"]=bundle["grasp_retry"].get("max_low_z_adjustments",2)
                checkpoint["low_z_adjustments_remaining"]=max(0,checkpoint["max_low_z_adjustments"]-checkpoint["low_check_index"])
                checkpoint["grasp_z_m"]=bundle["grasp_z_m"]
        self._atomic_json(checkpoint_path,checkpoint)
        if "grasp_retry" in bundle or "pre_release_adjustment" in bundle:
            self._atomic_json(directory/(token+"_checkpoint_latest.json"),
                              dict(checkpoint,checkpoint_path=str(checkpoint_path)))
        if after_publish is not None:
            after_publish()
        while deadline is None or time.monotonic()<deadline:
            self._cached_health()
            if self.cancelled.is_set():
                raise RuntimeError("cached checkpoint cancelled")
            if decision_path.exists():
                try:
                    decision=json.loads(decision_path.read_text())
                except (OSError,ValueError):
                    return received("failure","invalid_decision_file")
                allowed=("success","failure","adjust_z","retry_high") if preclose else ("success","failure","adjust","retry_high")
                if (not isinstance(decision,dict) or decision.get("token")!=token or
                        decision.get("decision") not in allowed):
                    return received("failure","invalid_decision")
                if "grasp_retry" in bundle and (type(decision.get("grasp_attempt_index")) is not int or
                        decision["grasp_attempt_index"]!=attempt_index):
                    return received("failure","wrong_grasp_attempt_index")
                if preclose and (decision.get("stage")!="preclose" or
                                  type(decision.get("low_check_index")) is not int or
                                  decision["low_check_index"]!=bundle.get("low_check_index",0)):
                    return received("failure","wrong_preclose_checkpoint_identity")
                if channel.get("require_worker") or channel.get("mode")=="serial":
                    if decision.get("worker_id")!=channel["worker_id"]:
                        return received("failure","wrong_worker_id")
                    if decision.get("checkpoint_id")!=checkpoint_id:
                        return received("failure","wrong_checkpoint_id")
                    if (type(decision.get("correction_index")) is not int or
                            decision["correction_index"]!=correction_index):
                        return received("failure","wrong_checkpoint_index")
                if "correction_index" in decision and decision["correction_index"]!=correction_index:
                    return received("failure","wrong_checkpoint_index")
                if decision.get("decision")=="retry_high":
                    if (bundle.get("grasp_retry",{}).get("alignment_failure_policy")!="recheck_high_then_grasp" or
                            any(k in decision for k in ("offset_xy_m","grasp_z_m")) or
                            decision.get("actual_relative_offset_xy_m",[0,0])!=[0,0]):
                        return received("failure","invalid_grasp_high_recheck")
                    return received("retry_high","external_decision",accepted=True)
                if preclose:
                    if decision.get("actual_relative_offset_xy_m",[0,0])!=[0,0]:
                        return received("failure","preclose_xy_adjustment_forbidden")
                    if decision.get("decision")=="adjust_z":
                        if any(k in decision for k in ("offset_xy_m","grasp_z_m")):
                            return received("failure","preclose_adjust_z_must_not_choose_xy_or_z")
                        return received("adjust_z","external_decision",accepted=True)
                    if decision.get("decision")=="success" and any(k in decision for k in ("offset_xy_m","grasp_z_m")):
                        return received("failure","preclose_success_must_not_choose_xy_or_z")
                selected_z=None
                if "grasp_z_m" in decision:
                    z=decision["grasp_z_m"]
                    if ("grasp_retry" not in bundle or decision["decision"]=="failure" or
                            type(z) not in (int,float) or not np.isfinite(z)):
                        return received("failure","invalid_grasp_z")
                    selected_z=next((v for v in bundle["grasp_z_options_m"] if abs(v-z)<=1e-9),None)
                    if selected_z is None:
                        return received("failure","grasp_z_not_prechecked")
                if decision["decision"]=="adjust":
                    config=bundle.get("pregrasp_adjustment")
                    if config and correction_index>=config["max_corrections"]:
                        return received("failure","correction_limit_reached")
                    offset=decision.get("offset_xy_m")
                    if (config is None or not isinstance(offset,list) or len(offset)!=2 or
                            any(isinstance(v,bool) or not isinstance(v,(int,float)) or not np.isfinite(v) for v in offset) or
                            np.linalg.norm(offset)<1e-6):
                        return received("failure","invalid_adjustment")
                    bundle["requested_offset_xy_m"]=[float(v) for v in offset]
                if selected_z is not None:
                    bundle["grasp_z_m"]=selected_z
                return received(decision["decision"],"external_decision",accepted=True)
            time.sleep(.05)
        return received("failure","decision_timeout")

    def _verify_decision_worker(self,bundle):
        """Check the bound local worker once, before any transaction motion."""
        channel=bundle.get("decision_channel",{})
        if channel.get("mode")=="serial" or not channel.get("require_worker"):
            return
        try:
            worker=json.loads(Path(channel["worker_ready_path"]).read_text())
        except (OSError,ValueError) as exc:
            raise RuntimeError("decision worker is not ready: missing or invalid worker file") from exc
        if (not isinstance(worker,dict) or worker.get("token")!=bundle["token"] or
                worker.get("worker_id")!=channel["worker_id"] or worker.get("status")!="ready"):
            raise RuntimeError("decision worker is not ready: identity or status mismatch")
        heartbeat=worker.get("heartbeat_unix_s")
        if (isinstance(heartbeat,bool) or not isinstance(heartbeat,(int,float)) or
                not np.isfinite(heartbeat) or not 0<=time.time()-heartbeat<=channel["heartbeat_timeout_s"]):
            raise RuntimeError("decision worker is not ready: heartbeat stale or invalid")
        pid=worker.get("pid")
        if type(pid) is not int or pid<=0:
            raise RuntimeError("decision worker is not ready: invalid pid")
        try:
            os.kill(pid,0)
        except OSError as exc:
            raise RuntimeError("decision worker is not ready: pid not alive or inaccessible") from exc
        self.log({"event":"decision_worker_ready","token":bundle["token"],
                  "worker_id":channel["worker_id"],"pid":pid,
                  "heartbeat_age_s":time.time()-heartbeat})

    def _auto_grasp_check(self,bundle):
        q,_,_,_=self._cached_health()
        motor_q=float(q[GRIP[1]]*self.dds.gripper_open_sign)
        evidence={"passed":False,"right_gripper_q":motor_q}
        if motor_q<.15:
            evidence["reason"]="right_gripper_closed_empty"
        else:
            try:
                observation=self.capture(self._execution_cameras(bundle),self._cached_image_directory(bundle,"auto_gate_images"))
                self._cached_health()
                config=bundle["auto_grasp_gate"]["vision"]
                evidence.update(evaluate_image(cv2.imread(observation["paths"][config["camera"]]),config))
                evidence["observation"]=observation
            except CachedHardwareFault:
                raise
            except Exception as exc:
                evidence.update(reason="auto_gate_image_failed",error=str(exc))
        self._atomic_json(bundle["directory"]/(bundle["token"]+"_auto_gate.json"),evidence)
        self.log({"event":"auto_grasp_gate","token":bundle["token"],**evidence})
        return evidence

    def execute_cached(self,token,transaction=False):
        bundle=getattr(self,"cached_plans",{}).pop(token,None)
        if bundle is None or bundle["kind"]!=("transaction" if transaction else "plan"):
            raise ValueError("unknown, consumed, or wrong-kind cached token")
        return self._execute_cached_bundle(bundle,transaction=transaction)

    def _execute_cached_bundle(self,bundle,transaction=False,*,verify_start=True,keep_control_on_placed=False):
        token=bundle["token"]
        physical_motion = getattr(self.motion, "physical", self.motion)
        self._pose_audit = PoseAudit(token, self.active_arm,
                                    bundle.get("pose_audit_original_source"),
                                    physical_motion.base, self.log)
        result={"token":token,"outcome":"not_started","control_enabled":False}
        context={"token":token,"branch":"grasp","execution_start":time.monotonic(),
                 "published_start":self.dds.published}
        cursor={}
        branch="grasp"
        recovery_compiled=bundle["grasp"]
        recovery_tail=[]
        try:
            if vars(self.guard)!=bundle["scene"] or self.dds.gripper_open_sign!=bundle["gripper_open_sign"]:
                raise RuntimeError("cached scene or gripper configuration changed")
            if transaction:
                self._verify_decision_worker(bundle)
            if verify_start:
                self._verify_cached_start(bundle["grasp"]["seed"],bundle["start_grip"])
            else:
                self._cached_health()
                if self.cancelled.is_set():
                    raise RuntimeError("cached sequence cancelled before continuation")
            self.cancelled.clear()
            self.log({"event":"cached_start","token":token,"kind":bundle["kind"]})
            result["grasp"]=self._run_cached_steps(bundle["grasp"]["steps"],context,cursor)
            if transaction and "grasp_retry" in bundle:
                branch="grasp_retry"
                self._execute_grasp_retries(bundle,result,context)
                return result
            if transaction:
                decision,reason=self._checkpoint_decision(bundle)
                result.update(decision=decision,decision_reason=reason)
                self.cancelled.clear()
                while decision=="adjust":
                    adjustment=self._apply_pregrasp_adjustment(bundle,bundle["requested_offset_xy_m"])
                    result.setdefault("adjustments",[]).append(adjustment)
                    result["adjustment"]=adjustment
                    if not adjustment["passed"]:
                        decision="failure"
                        reason=adjustment["reason"]
                        break
                    self._cached_health()
                    branch="correction"
                    context["branch"]=branch
                    context["correction_index"]=adjustment["correction_index"]
                    recovery_compiled=bundle["success"]
                    recovery_tail=bundle["correction_return_steps"]
                    cursor={}
                    start_state=self.state()
                    self.log({"event":"correction_start","token":token,"state":start_state,
                              "correction_index":adjustment["correction_index"],
                              "offset_xy_m":adjustment["offset_xy_m"],
                              "cumulative_offset_xy_m":adjustment["cumulative_offset_xy_m"]})
                    completed=self._run_cached_steps(bundle["success"]["steps"][:1],context,cursor)
                    result.setdefault("corrections",[]).append(completed)
                    bundle["corrections_completed"]=adjustment["correction_index"]
                    bundle["pregrasp_seed"]=bundle["success"]["steps"][0]["samples"][-1][0].copy()
                    end_state=self.state()
                    self.log({"event":"correction_done","token":token,"state":end_state,
                              "correction_index":adjustment["correction_index"],
                              "actual_delta_xy_m":[end_state["right_arm"][axis]-start_state["right_arm"][axis]
                                                   for axis in ("x","y")],
                              "arrival":completed})
                    decision,reason=self._checkpoint_decision(bundle,post_adjustment=True)
                    result.update(post_adjustment_decision=decision,post_adjustment_reason=reason)
                    result.setdefault("post_adjustment_checks",[]).append({
                        "correction_index":adjustment["correction_index"],"decision":decision,"reason":reason})
                # Keep the last completed correction cursor available until the
                # selected frozen tail passes its health/collision checks.
                corrected=bundle.get("corrections_completed",0)>0
                if decision!="success":
                    result["return_reason"]=reason
                    selected="adjust_failure" if corrected else "failure"
                    self._cached_health()
                    branch=selected
                    context["branch"]=branch
                    result[branch]=self._run_cached_steps(bundle[branch]["steps"],context,{})
                    result["outcome"]=("post_adjustment_rejected_returned" if corrected
                                       else "grasp_failed_returned")
                    return result
                success_start=1 if corrected else 0
                self._cached_health()
                branch="success"
                context["branch"]=branch
                cursor={}
                if "auto_grasp_gate" in bundle:
                    split=bundle["auto_grasp_gate"]["after_success_step"]+1
                    success_steps=bundle["success"]["steps"]
                    result["success"]=self._run_cached_steps(success_steps[success_start:split],context,cursor)
                    result["auto_grasp_gate"]=self._auto_grasp_check(bundle)
                    if result["auto_grasp_gate"]["passed"]:
                        result["success"].extend(self._run_placement_steps(success_steps[split:],bundle,result,context,{}))
                        result["outcome"]="placed"
                    else:
                        branch="gate_failure"
                        context["branch"]=branch
                        self._cached_health()
                        result[branch]=self._run_cached_steps(bundle[branch]["steps"],context,{})
                        result["outcome"]="auto_grasp_failed_returned"
                else:
                    result[branch]=self._run_placement_steps(bundle[branch]["steps"],bundle,result,context,cursor)
                    result["outcome"]="placed" if branch=="success" else "grasp_failed_returned"
            else:
                result["outcome"]="completed"
        except Exception as exc:
            result.update(ok=False,error=str(exc),dds_fault=self.dds.fault,outcome="stopped_after_fault")
            # A real transport/motor fault is never cleared to force a retreat.
            # Only the grasp/initialization prefix has a prechecked inverse path.
            if (branch in ("grasp","correction") and cursor and not self.dds.fault and self.dds.enabled
                    and not isinstance(exc,CachedHardwareFault)
                    and not self.cancelled.is_set()):
                try:
                    recovery=self._cached_recovery(recovery_compiled,cursor)+recovery_tail
                    self.cancelled.clear()
                    context["branch"]="recovery"
                    result["recovery"]=self._run_cached_steps(recovery,context,{})
                    result["outcome"]="motion_failed_returned_to_start"
                except Exception as recovery_exc:
                    result["recovery_error"]=str(recovery_exc)
        finally:
            # Release only after a complete prechecked ending. Exceptions may
            # leave a piece in the gripper; keep healthy control actively holding.
            finished=result.get("ok",True) and result["outcome"] in {
                "completed","placed","alignment_failed_returned",
                "grasp_failed_returned","post_adjustment_rejected_returned",
                "auto_grasp_failed_returned","unverified_grasp_returned",
                "empty_grasp_limit_returned"}
            continuing=finished and result["outcome"]=="placed" and keep_control_on_placed
            if finished and not continuing:
                with self.dds.lock:
                    self.dds.enabled=False
            elif not continuing and self.dds.enabled:
                try:
                    # hold() reads finite, fresh feedback without advancing the
                    # trajectory. A motorstate fault must not leave an old high
                    # target queued for when that hardware status recovers.
                    self.dds.hold()
                except Exception as hold_exc:
                    result["hold_error"]=str(hold_exc)
            if not continuing:
                self.cancelled.set()
            with self.dds.lock:
                result["control_enabled"]=bool(self.dds.enabled)
            result["dds_fault"]=self.dds.fault
            try:
                result["state"]=self.state()
            except Exception as state_exc:
                result["state_error"]=str(state_exc)
            result["pose_audit"] = self._pose_audit.summary()
            self._pose_audit = None
            self._atomic_json(bundle["directory"]/(token+"_result.json"),result)
            self.log({"event":"cached_finished",**result})
        return result

    def gripper(self,side,target,*,ignore_right_tool_envelope=False,pose_audit_release=None):
        if self.guard is None:
            raise ValueError("set_scene required before gripper motion")
        feedback=self.dds.state()
        feedback_at=time.monotonic()
        q,_,_,_=feedback
        if not (side=="right" and ignore_right_tool_envelope):
            self.guard.check_gripper(self.motion.fk(q[ARM])[0 if side=="left" else 1])
        audit = getattr(self, "_pose_audit", None)
        if audit is not None and pose_audit_release is not None:
            audit.capture("release_start", feedback, self.dds,
                          feedback_read_monotonic_s=feedback_at, **pose_audit_release)
        self.dds.command_gripper(side,float(target))
        idx=31 if side=="left" else 33
        start=time.monotonic()
        previous=self.dds.state()[0][idx]
        still_since=start
        deadline=start+abs(float(target)-previous)/.8+2
        reached=False
        while time.monotonic()<deadline:
            if self.cancelled.is_set():
                self.dds.hold()
                raise RuntimeError("gripper cancelled; holding")
            self.dds.assert_enabled()
            q,dq,msg,_=self.dds.state()
            if abs(q[idx]-target)<.08 and abs(dq[idx])<.1:
                reached=True
                break
            if abs(q[idx]-previous)>.008:
                still_since=time.monotonic()
                previous=q[idx]
            if time.monotonic()-still_since>1. and abs(dq[idx])<.05:
                break
            time.sleep(.04)
        result=self.state()["grippers"][side]
        result.update(target_q=target,reached=reached,
                      contact_candidate=bool(not reached and abs(result["dq"])<.1))
        self.log({"event":"gripper", "side":side,**result})
        # Aperture/arrival residuals are diagnostic during execution. Keep the
        # requested aperture and continue the prechecked path; grasp decisions
        # still use their separate contact evidence and visual checkpoints.
        return result

    def lock_waist_zero(self):
        """Explicitly authorized waist-only zeroing; arm plans are invalidated."""
        if self.dds.enabled:
            raise RuntimeError("release arm control before waist zeroing")
        q,_,msg,_=self.dds.state()
        if self.dds.fault:
            raise RuntimeError(self.dds.fault)
        if any(int(msg.motor_state[i].motorstate) for i in WAIST):
            raise CachedHardwareFault("waist motorstate is nonzero")
        if abs(q[12])>.5 or np.max(np.abs(q[13:15]))>.15:
            raise ValueError("waist zeroing start exceeds supported yaw/roll/pitch range")
        if np.max(np.abs(q[GRIP]))>.15:
            raise ValueError("waist zeroing requires empty closed grippers")
        self.cached_plans.clear()
        self.cancelled.clear()
        self.log({"event":"waist_zero_start","actual_q":q[WAIST].tolist(),
                  "target_q":[0.,0.,0.],"max_velocity_rad_s":.1})
        self.dds.enable_waist_zero()
        deadline=time.monotonic()+15.
        stable_since=None
        try:
            while time.monotonic()<deadline:
                q,dq,msg,_=self.dds.state()
                if self.dds.fault or not self.dds.waist_enabled:
                    raise CachedHardwareFault(self.dds.fault or "waist hold stopped")
                if self.cancelled.is_set():
                    raise RuntimeError("waist zeroing cancelled")
                if any(int(msg.motor_state[i].motorstate) for i in WAIST):
                    raise CachedHardwareFault("waist motorstate is nonzero")
                reached=np.max(np.abs(q[WAIST]))<=.01 and np.max(np.abs(dq[WAIST]))<=.03
                if reached:
                    if stable_since is None:
                        stable_since=time.monotonic()
                    if time.monotonic()-stable_since>=.5:
                        result=self.state()
                        self.log({"event":"waist_zero_verified","waist_q":result["waist_q"],
                                  "waist_hold_enabled":True,"state":result})
                        return result
                else:
                    stable_since=None
                time.sleep(.04)
            raise TimeoutError("waist did not settle at zero within 15 seconds")
        except Exception as exc:
            self.dds.release_waist()
            self.log({"event":"waist_zero_failed","error":str(exc)})
            raise

    def dispatch(self,req):
        cmd=req.get("cmd")
        if cmd=="ping":
            return {"pong":True,"backend":"dex1_internal","log_dir":str(self.log_dir)}
        if cmd in ("get_state","get_arm_pose","get_arm_q","get_full_q","get_grippers"):
            return self.state()
        if cmd=="capture_cameras":
            return self.capture(req.get("cameras"),req.get("output_dir"))
        if cmd=="enable_control":
            return self.enable()
        if cmd=="lock_waist_zero":
            return self.lock_waist_zero()
        if cmd=="release_waist":
            if self.dds.enabled:
                raise RuntimeError("release arm control before releasing waist hold")
            self.cached_plans.clear()
            self.dds.release_waist()
            self.log({"event":"waist_hold_released"})
            return self.state()
        if cmd=="set_scene":
            if self.dds.enabled:
                raise RuntimeError("scene cannot change during active control")
            self.guard=BoardGuard.from_dict(req["scene"])
            self.log({"event":"scene_set","scene":vars(self.guard),"observation":req.get("observation")})
            return {"scene":vars(self.guard)}
        if cmd=="hold":
            self.cancelled.set()
            self.dds.hold()
            return self.state()
        if cmd=="set_gripper":
            self.cancelled.clear()
            return self.gripper(req["side"],req["q"])
        if cmd=="prepare_plan":
            steps=self.prepare(req["plan"])
            return {"steps":len(steps),"samples":sum(len(s["samples"]) for s in steps),
                    "duration_s":sum(t for s in steps for q,t in s["samples"]),
                    "collision_checked":False}
        if cmd=="prepare_cached_plan":
            return self.prepare_cached(req)
        if cmd=="execute_cached_plan":
            return self.execute_cached(req["token"])
        if cmd=="prepare_transaction":
            return self.prepare_cached(req["transaction"],transaction=True)
        if cmd=="restore_cached_transaction":
            return self.restore_cached_transaction(req["archive_path"], req["sha256"],
                                                   req["output_dir"], req["image_dir"])
        if cmd=="execute_transaction":
            return self.execute_cached(req["token"],transaction=True)
        if cmd=="prepare_sequence":
            return self.prepare_sequence(req["transactions"])
        if cmd=="execute_sequence":
            return self.execute_sequence(req["sequence_token"])
        if cmd=="execute_plan":
            return self.execute(req["plan"])
        if cmd=="shutdown":
            if self.dds.enabled or getattr(self.dds,"waist_enabled",False):
                raise RuntimeError("control active: explicit release_control required before shutdown")
            return {"shutdown":True}
        if cmd=="release_control":
            # Stops this publisher only, with no automatic return-to-start motion.
            self.cancelled.set()
            # Explicit release must remain possible with invalid feedback and
            # must not silently clear a latched hardware/transport fault.
            self.dds.release()
            return {"control_enabled":False,"waist_hold_enabled":getattr(self.dds,"waist_enabled",False)}
        raise ValueError(f"unknown command {cmd}")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface",default=os.environ.get("CHESS_INTERFACE"))
    parser.add_argument("--endpoint",default=os.environ.get("CHESS_ENDPOINT") or "ipc:///tmp/g1d_chess.sock")
    parser.add_argument("--camera-host",default=os.environ.get("CHESS_CAMERA_HOST"))
    parser.add_argument("--log-dir")
    parser.add_argument("--gripper-open-sign",type=int,choices=[-1,1],default=os.environ.get("CHESS_GRIPPER_SIGN") or 1,
                        help="Internal Dex1 opening sign; default +1 for both physical hands")
    args=parser.parse_args()
    if not args.interface or not args.camera_host:
        parser.error("configure interface and camera-host via CLI or CHESS_* variables")
    if not (args.endpoint.startswith("ipc://") or args.endpoint.startswith("tcp://127.0.0.1:")):
        parser.error("control endpoint must be local")
    context=zmq.Context()
    socket=context.socket(zmq.REP)
    socket.setsockopt(zmq.LINGER,0)
    socket.bind(args.endpoint)
    service=RobotService(args.interface,args.camera_host,args.log_dir,args.gripper_open_sign)
    print(json.dumps({"ready":True,"endpoint":args.endpoint,"state":service.state()}),flush=True)
    def interrupt(*_):
        if service.dds.enabled:
            service.cancelled.set()
            service.dds.hold()
            print("Interrupt: arms held; service remains available for recovery.",flush=True)
        else:
            raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt)
    try:
        while True:
            req=socket.recv_json()
            try:
                response={"ok":True,**service.dispatch(req)}
            except Exception as exc:
                response={"ok":False,"error":str(exc)}
                service.log({"event":"error","request":req,"error":str(exc)})
            socket.send_json(response)
            if response.get("shutdown"):
                break
    finally:
        service.dds.close()
        if service.camera is not None:
            service.camera.disconnect()
        socket.close()
        context.term()
        service.event_log.close()


if __name__=="__main__":
    main()
