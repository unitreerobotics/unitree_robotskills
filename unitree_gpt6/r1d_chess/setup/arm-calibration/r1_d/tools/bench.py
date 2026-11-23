"""Interactive local R1D control console; SDK owns the control session."""

import argparse
import json
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from r1_d.sdk import R1D, ControlSession
from r1_d.sdk.client import FeedbackUnavailable
from r1_d.sdk.model import FAULTY_LINK_MOTORS, calibrated_urdf, json_default, model_info, saved_home_pose
from r1_d.tools.commands import HELP, execute_command
from r1_d.config import DEFAULTS, HOME_POSE, REPORT_DIR


def input_loop(commands):
    for line in sys.stdin:
        commands.put(line.strip())
    commands.put("quit")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--ip", default=DEFAULTS["ip"])
    p.add_argument("--interface", default=DEFAULTS["interface"])
    p.add_argument("--transport", choices=("ssh", "dds"), default=DEFAULTS["transport"])
    p.add_argument("--check", action="store_true", help="只读状态和运控查询，不接管或发送动作")
    p.add_argument(
        "--release-mode", action="store_true", help="显式释放厂商运控服务，进入 LowCmd 控制"
    )
    p.add_argument("--gripper-calibration", type=Path)
    p.add_argument("--no-gravity", action="store_true", help="关闭重力前馈，仅用 PD（对照用）")
    p.add_argument(
        "--ignore-link-loss",
        action="store_true",
        help="右腕 yaw（28）和右夹爪（33）电机掉线时继续运行；两者停在最后指令，掉线期间不能移动",
    )
    p.add_argument("--gravity-urdf", type=Path, default=calibrated_urdf)
    p.add_argument("--model-info", action="store_true", help="离线核验官方 URDF，随后退出")
    p.add_argument(
        "--log",
        type=Path,
        default=REPORT_DIR / "bench_session.jsonl",
    )
    args = p.parse_args()
    print(json.dumps(model_info(require_official=True), ensure_ascii=False, indent=2), flush=True)
    if args.model_info:
        return
    calibration = (
        json.loads(args.gripper_calibration.read_text()) if args.gripper_calibration else {}
    )
    args.log.parent.mkdir(parents=True, exist_ok=True)
    with args.log.open("a", buffering=1) as log:

        def record(event):
            log.write(json.dumps({"time_unix": time.time(), **event}, default=json_default) + "\n")
            if event["event"] in ("start", "holding", "stop"):
                note = ""
                if event["event"] == "holding":
                    worst = max(event["tracking_error_degrees"].values())
                    note = f"  到位时双臂最大误差 {worst:.2f}°"
                print(f"[{event['event']}] {event.get('label', '')}{note}", flush=True)
            if event["event"] == "holding" and not event.get("within_tolerance", True):
                print(
                    f"轨迹已发完但未准确到位，继续保持目标。双臂最大误差(°)："
                    f"{event['tracking_error_degrees']}；各组误差(rad/夹爪电机单位)："
                    f"{event['tracking_error']}",
                    flush=True,
                )
            if event["event"] == "feedback_paused":
                print("升降反馈暂时过期：已停止升降并暂停轨迹，双臂保持当前目标。", flush=True)
            if event["event"] == "feedback_resumed":
                print(
                    f"升降反馈恢复，继续轨迹（暂停 {event['pause_seconds']:.2f} 秒）。", flush=True
                )
            if event["event"] == "arm_feedback_paused":
                print("双臂反馈暂时过期：暂停发送，机器人保持上一帧指令。", flush=True)
            if event["event"] == "arm_feedback_resumed":
                print(
                    f"双臂反馈恢复，继续轨迹（暂停 {event['pause_seconds']:.2f} 秒）。", flush=True
                )

        robot = R1D(args.ip, args.interface, multicast=True, transport=args.transport)
        if args.ignore_link_loss:
            robot.link_ignored = FAULTY_LINK_MOTORS
        try:
            report = robot.inspect(2.0)
            robot.arm_state()
            robot.height()
            code, mode = robot.CheckMode()
            record(
                {
                    "event": "connection",
                    "report": report,
                    "motion_mode": {"code": code, "state": mode},
                    "ssh_rtt_seconds": robot.relay.last_rtt if robot.relay else None,
                }
            )
            print(f"已连接，运控状态={mode}", flush=True)
            if code:
                raise RuntimeError(f"MotionSwitcher query failed: {code}")
            if args.check:
                print(
                    json.dumps(
                        {
                            "received_samples": robot.counts,
                            "sent_commands": robot.sent,
                            "height_mm": robot.height() * 1000.0,
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
                return
            robot.prepare_control(release=args.release_mode)
            gravity = None
            if DEFAULTS.get("gravity_compensation", True) and not args.no_gravity:
                if not args.gravity_urdf.is_file():
                    raise RuntimeError(f"缺少标定 URDF：{args.gravity_urdf}；可加 --no-gravity")
                from r1_d.sdk.dynamics import ArmGravity

                gravity = ArmGravity(args.gravity_urdf)
            print(
                f"重力前馈：{args.gravity_urdf.name if gravity else '关闭（仅 PD，受力姿态会下垂）'}",
                flush=True,
            )
            home_pose = saved_home_pose()
            session = ControlSession(robot, log=record, gravity=gravity, home_pose=home_pose)
            if home_pose:
                print(f"归位姿态：{HOME_POSE}（高度 {home_pose['height'] * 1000:.0f} mm）", flush=True)
            print(HELP, flush=True)
            print(("归位直达 assets/poses/home.json" if home_pose else "归位参考为本次启动姿态") + "；正在保持当前姿态。输入指令后按回车。", flush=True)
            commands = queue.Queue()
            threading.Thread(target=input_loop, args=(commands,), daemon=True).start()
            next_tick = time.monotonic()
            while True:
                try:
                    line = commands.get_nowait()
                except queue.Empty:
                    line = ""
                if line in ("q", "quit"):
                    break
                if line:
                    try:
                        if line in ("s", "status"):
                            print(json.dumps(session.status(), ensure_ascii=False), flush=True)
                        elif line == "help":
                            print(HELP, flush=True)
                        else:
                            execute_command(session, line, calibration)
                    except ValueError as exc:
                        print(f"拒绝指令：{exc}", flush=True)
                    except FeedbackUnavailable as exc:
                        # tick() ends the session if the feedback stays away too long.
                        print(f"指令未执行，等待反馈恢复：{exc}", flush=True)
                session.tick()
                next_tick = max(next_tick + 0.004, time.monotonic())
                time.sleep(max(0.0, next_tick - time.monotonic()))
        except (Exception, KeyboardInterrupt) as exc:
            record({"event": "error", "error": repr(exc), "feedback": robot.feedback_status()})
            raise
        finally:
            try:
                robot.close()
            finally:
                record({"event": "closed", "sent_commands": robot.sent})
                print("已停止发送；未自动归位或清零双臂增益。", flush=True)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"启动/控制失败：{exc}", file=sys.stderr)
        raise SystemExit(1)
