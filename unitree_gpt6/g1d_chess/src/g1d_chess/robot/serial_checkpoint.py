"""Local serial checkpoints: read the next fresh image gate or submit one review.

No robot connection, background model, planning or autonomous decision generation.
The main conversation reviews head + physical actor wrist immediately, then submits.
Do not run code inspection, other models or unrelated queries while a gate is open.
Audit mirroring runs independently; only checkpoint/decision/receipt I/O is needed.
"""
import argparse
import json
from pathlib import Path
import sys
import time

from g1d_chess.paths import ROOT
from g1d_chess.robot.checkpoint_runner import atomic_json, decision_from_evaluation, read_json, RELEASE_SCHEMA
from g1d_chess.robot.camera_policy import checkpoint_cameras


def failure_from_checkpoint(checkpoint):
    """Bind an operator-requested source return to the current gate."""
    if checkpoint.get("stage") not in ("pregrasp", "post_adjustment", "preclose"):
        raise ValueError("Direct failure is only available at source grasp gates")
    decision = {key: checkpoint[key] for key in
                ("token", "worker_id", "checkpoint_id", "correction_index")}
    if "grasp_attempt_index" in checkpoint:
        decision["grasp_attempt_index"] = checkpoint["grasp_attempt_index"]
    decision.update(decision="failure", reason="operator_requested_return")
    if checkpoint["stage"] == "preclose":
        decision.update(stage="preclose", low_check_index=checkpoint["low_check_index"])
    return decision


def current_gate(prepared):
    result = Path(prepared["checkpoint_path"]).parent / (prepared["token"] + "_result.json")
    if result.exists():
        value = read_json(result)
        return {"finished": True, "result_path": str(result),
                **{k: value.get(k) for k in ("outcome", "return_reason", "error", "control_enabled")}}
    discovery = prepared.get("checkpoint_discovery_path")
    if discovery:
        if not Path(discovery).exists():
            return {"waiting": "motion_or_next_image", "discovery_path": discovery}
        descriptors = [read_json(discovery)]
    else:
        descriptors = [prepared] + prepared.get("post_adjustment_checkpoints", [])
    for descriptor in descriptors:
        path = Path(descriptor["checkpoint_path"])
        if not path.exists():
            return {"waiting": "motion_or_next_image", "checkpoint_path": str(path)}
        checkpoint = read_json(path)
        if checkpoint["token"] != prepared["token"]:
            raise ValueError("Discovered checkpoint belongs to another transaction")
        receipt_path = Path(checkpoint["receipt_path"])
        if receipt_path.exists():
            receipt = read_json(receipt_path)
            if receipt.get("accepted") is True and receipt.get("decision") == "adjust":
                if discovery:
                    return {"waiting": "motion_or_next_image", "receipt": receipt}
                continue
            if discovery and receipt.get("accepted") is True:
                return {"waiting": "motion_or_next_image", "receipt": receipt}
            return {"waiting": "frozen_tail_completion", "receipt": receipt}
        if Path(checkpoint["decision_path"]).exists():
            return {"waiting": "service_receipt", "checkpoint_path": str(path)}
        cameras = checkpoint_cameras(checkpoint)
        return {"needs_review": True, "checkpoint_path": str(path),
                "workflow": "view_head_and_actor_wrist_then_submit_immediately",
                "decision_target_s": 5,
                "checkpoint_age_s": max(0., time.time() - checkpoint.get("created_at_unix_s", time.time())),
                "paths": {key: checkpoint["observation"]["paths"][key] for key in cameras},
                "camera_order": cameras, "camera_selection": checkpoint.get("camera_selection"),
                "actual_right_arm": checkpoint["actual_right_arm"],
                "adjustment_reference_right_arm": checkpoint["adjustment_reference_right_arm"],
                "correction_index": checkpoint["correction_index"],
                "grasp_attempt_index": checkpoint.get("grasp_attempt_index", 0),
                "grasp_z_m": checkpoint.get("grasp_z_m"),
                "grasp_z_options_m": checkpoint.get("grasp_z_options_m", []),
                "grasp_z_clearances_m": checkpoint.get("grasp_z_clearances_m", []),
                "grasp_z_surface_safe_options_m": checkpoint.get("grasp_z_surface_safe_options_m", []),
                "grasp_z_selectable_options_m": checkpoint.get("grasp_z_selectable_options_m", []),
                "stage": checkpoint.get("stage"),
                "physical_actor": checkpoint.get("physical_actor"),
                "target_description": checkpoint.get("target_description"),
                "target_evidence": checkpoint.get("target_evidence"),
                "original_target": checkpoint.get("original_target"),
                "on_correction_limit": checkpoint.get("on_correction_limit"),
                "review_pose": checkpoint.get("review_pose"),
                "adjustment_policy": checkpoint.get("adjustment_policy"),
                "alignment_criterion": checkpoint.get("alignment_criterion"),
                "correction_motion": checkpoint.get("correction_motion"),
                "release_review_instruction": (
                    "已下降至释放高度、尚未张爪。只看本次头部和物理执行手腕部两图，以执行手腕部图判断棋子张爪后预计落点与目标板面交点的关系，头图辅助核对格名和邻子。"
                    "预计落点在相邻两个交点之间、方格中间或明显错开目标交点，均属于明显偏差，须提交adjust并设obvious_large_offset=true，说明偏向和腕图依据；不能仅因棋子边缘覆盖交点或FK误差小就放行。"
                    "预计棋子落点已在目标交点附近、只有小幅偏心且可安全释放时，提交success并设obvious_large_offset=false直接释放，不要求完全精准对齐交点。结合夹持偏心和视差判断预计落点，不能把夹爪外壳/影子位置当落点。棋子覆盖目标交点不属于看不清，应结合可见格线延伸、邻近交点和棋子中心判断；不能仅因交点被遮住而调整。"
                    "真正无法判断预计落点或有碰撞风险均按需要修正处理，提交adjust并设obvious_large_offset=true、release_alignment_verified=false，说明不确定处或风险以及有依据的修正方向。修正仍须识别目标、给出有依据的非零XY并通过运动检查；不能为提交adjust虚构目标识别或位移。"
                    "XY相对当前低位实际FK；仅豁免修正段低位横移禁令，保持实际Z/RPY直接微调，不抬升，保留表面净空/IK/关节/硬件检查。首次和第二次调整后在释放高度复拍；"
                    "第三次低位XY调整后直接张爪，不再复拍。放行时记录低位FK快照，不查历史或做复盘。"
                    if checkpoint.get('alignment_criterion') not in (
                        'actor_wrist_predicted_landing_near_offboard_target',
                        'actor_wrist_predicted_landing_inside_same_side_tray') else
                    ("已下降至圆盘释放高度、尚未张爪。用本次执行手腕图为主、头图辅助，按physical_actor核对左手左盘、右手右盘及target_description。"
                     "预计棋子整体落在执行手同侧圆盘内可用区域、避开盘沿和已放棋子且可安全释放则success；不要求对准圆盘中心，不套棋盘交点判据。"
                     "预计落在盘外、压盘沿、与已放棋子干涉、真正无法判断预计落点或有碰撞风险均按需要修正处理，提交adjust并设obvious_large_offset=true、release_alignment_verified=false。"
                     "棋子遮住盘内目标点本身不算看不清；修正须有目标识别和有依据的非零XY并通过运动检查，不能虚构位移。"
                     "保持实际Z/RPY直接低位XY微调；前两次后复拍，第三次后直接张爪。放行时记录低位FK快照。"
                     if checkpoint.get('alignment_criterion') == 'actor_wrist_predicted_landing_inside_same_side_tray' else
                    "已下降至棋外释放高度、尚未张爪。只看本次头部和物理执行手腕部两图，核对预计落点与预先确认的棋外目标及安全表面。"
                    "目标附近且可安全释放则success；明显偏离棋外目标、真正无法判断预计落点或有碰撞风险均按需要修正处理，提交adjust并设obvious_large_offset=true、release_alignment_verified=false，说明腕图依据。修正仍须识别目标、给出有依据的非零XY并通过运动检查；不能虚构目标识别或位移。棋外不套棋盘交点判据。"
                    "修正保持实际Z/RPY，直接低位XY调整，不抬升；前两次后复拍，第三次后直接张爪。放行时记录低位FK快照。")
                    ) if checkpoint.get("stage")=="pre_release" else None,
                "release_evaluation_schema": RELEASE_SCHEMA if checkpoint.get("stage")=="pre_release" else None,
                "low_check_index": checkpoint.get("low_check_index"),
                "low_z_step_m": checkpoint.get("low_z_step_m"),
                "low_z_adjustments_remaining": checkpoint.get("low_z_adjustments_remaining"),
                "minimum_surface_clearance_m": checkpoint.get("minimum_surface_clearance_m"),
                "ignore_grasp_tool_envelope": checkpoint.get("ignore_grasp_tool_envelope", False),
                "preclose_high_recheck_supported": checkpoint.get("preclose_high_recheck_supported", False),
                "corrections_remaining": checkpoint["corrections_remaining"],
                "wait_indefinitely": checkpoint.get("wait_indefinitely", False),
                "remaining_s": (None if checkpoint.get("deadline_unix_s") is None
                                else checkpoint["deadline_unix_s"] - time.time())}
    return {"waiting": "transaction_result"}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("action", choices=("next", "submit"))
    p.add_argument("--prepared", required=True, type=Path)
    p.add_argument("--evaluation", type=Path)
    p.add_argument("--checkpoint", type=Path)
    p.add_argument("--failure", action="store_true",
                   help="Return from the current source grasp gate without a visual evaluation")
    p.add_argument("--seconds", type=float, default=45)
    a = p.parse_args()
    prepared = read_json(a.prepared)
    channel = prepared["decision_channel"]
    if channel.get("mode") != "serial" or channel.get("require_worker") is not False:
        raise ValueError("This helper requires an explicitly prepared serial channel")
    if a.action == "next":
        deadline = time.monotonic() + min(55, max(0, a.seconds))
        while True:
            status = current_gate(prepared)
            if status.get("needs_review") or status.get("finished") or time.monotonic() >= deadline:
                print(json.dumps(status, ensure_ascii=False), flush=True)
                return
            time.sleep(.1)
    if a.checkpoint is None or ((a.evaluation is None) != a.failure):
        raise ValueError("submit requires a checkpoint and exactly one of --evaluation or --failure")
    current = current_gate(prepared)
    if not current.get("needs_review") or Path(current["checkpoint_path"]).resolve() != a.checkpoint.resolve():
        raise ValueError("This is not the current undecided checkpoint")
    c = read_json(a.checkpoint)
    if c["token"] != prepared["token"] or c["worker_id"] != channel["worker_id"]:
        raise ValueError("Checkpoint belongs to another transaction")
    if c.get("deadline_unix_s") is not None and time.time() >= c["deadline_unix_s"] - 1:
        raise TimeoutError("No late decisions")
    if a.failure:
        decision = failure_from_checkpoint(c)
    else:
        evaluation = json.load(sys.stdin) if str(a.evaluation) == "-" else read_json(a.evaluation)
        decision = decision_from_evaluation(c, evaluation)
    decision["checkpoint_to_submit_s"] = max(0., time.time() - c.get("created_at_unix_s", time.time()))
    decision["producer"] = "main_agent_serial_visual_review"
    atomic_json(c["decision_path"], decision, exclusive=True)
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        if Path(c["receipt_path"]).exists():
            receipt = read_json(c["receipt_path"])
            identity = ("token", "worker_id", "checkpoint_id", "correction_index")
            if "grasp_attempt_index" in decision:
                identity += ("grasp_attempt_index",)
            if decision.get("stage") == "preclose":
                identity += ("stage", "low_check_index")
            elif decision.get("stage") == "pre_release":
                identity += ("stage",)
            if any(receipt.get(k) != decision[k] for k in identity):
                raise ValueError("Receipt identity mismatch")
            if receipt.get("accepted") is not True or receipt.get("decision") != decision["decision"]:
                raise ValueError("Service rejected decision")
            print(json.dumps({**receipt, "checkpoint_to_submit_s": decision["checkpoint_to_submit_s"]}, ensure_ascii=False))
            return
        time.sleep(.05)
    raise TimeoutError("Receipt not yet received; use next to inspect, never resubmit")


if __name__ == "__main__":
    main()
