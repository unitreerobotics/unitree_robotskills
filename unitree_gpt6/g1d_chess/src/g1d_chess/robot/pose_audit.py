"""Two event snapshots only. No FK, IK, file I/O or blocking queue operations."""
import copy
import time

from g1d_chess.robot.dex1_internal import ARM, GRIP


class PoseAudit:
    def __init__(self, token, active_arm, original_source, base, emit):
        self.token = token
        self.active_arm = active_arm
        self.original_source = copy.deepcopy(original_source)
        self.base = base.tolist()
        self.emit = emit
        self.records = []
        self.errors = []
        self.grasp_target = None

    def capture(self, event, feedback, dds, effective_target, original_target=None,
                grasp_attempt_index=None, feedback_read_monotonic_s=None,
                event_stage=None, placement_target=None, approval_reason=None):
        # Diagnostic failures must never change motion/recovery decisions.
        try:
            if event not in ("grasp_success", "release_start"):
                raise ValueError("unsupported pose snapshot event")
            if any(row["moment"] == event for row in self.records):
                return
            q, dq, msg, age = feedback
            commands = None
            # Never wait for the DDS publisher. Keep the feedback even if the
            # command snapshot is unavailable; review marks that comparison null.
            if dds.lock.acquire(blocking=False):
                try:
                    commands = {name: getattr(dds, name).copy().tolist()
                                for name in ("arm_target", "arm_sent")}
                finally:
                    dds.lock.release()
            row = {
                "event": "pose_snapshot", "schema_version": 1,
                "token": self.token, "moment": event,
                "event_stage": event_stage or ("source_contact_success" if event == "grasp_success"
                                                else "before_low_open_command"),
                "placement_target": copy.deepcopy(placement_target),
                "approval_reason": approval_reason,
                "event_monotonic_s": time.monotonic(),
                "feedback_tick": int(msg.tick), "feedback_age_s": float(age),
                "feedback_read_monotonic_s": feedback_read_monotonic_s,
                "physical_actor": self.active_arm,
                "joint_order": "observer_7_then_actor_7",
                "coordinate_frame": "arm_model_locked_zero_waist",
                "fk_base": self.base,
                "arm_q": q[ARM].tolist(), "arm_dq": dq[ARM].tolist(),
                "gripper_q": q[GRIP].tolist(), "gripper_dq": dq[GRIP].tolist(),
                "waist_q": q[12:15].tolist(),
                "imu_rpy": list(msg.imu_state.rpy),
                "commands": commands,
                "command_snapshot_available": commands is not None,
                "original_target": copy.deepcopy(
                    self.original_source if original_target is None else original_target),
                "effective_target": copy.deepcopy(effective_target),
                "grasp_attempt_index": grasp_attempt_index,
            }
            # Only two bounded in-memory records; final result persists them
            # after execution even if the best-effort async audit log drops one.
            self.records.append(row)
            self.emit(row)
        except Exception as exc:
            if len(self.errors) < 2:
                self.errors.append(str(exc))

    def summary(self):
        return {"schema_version": 1, "records": self.records,
                "missing_events": [name for name in ("grasp_success", "release_start")
                                   if not any(r["moment"] == name for r in self.records)],
                "errors": self.errors, "fk_computed_during_capture": False}
