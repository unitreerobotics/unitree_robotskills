"""Shared by the test tools: connection options, control session, measuring, reports."""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from r1_d.config import DEFAULTS, REPORT_DIR
from r1_d.sdk import R1D, ControlSession
from r1_d.sdk.model import (
    ARM_KD,
    ARM_KP,
    FAULTY_LINK_MOTORS,
    GROUPS,
    calibrated_urdf,
    json_default,
    model_info,
)

ARMS = ("left_arm", "right_arm")


def parser(description, name):
    p = argparse.ArgumentParser(
        description=description, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--ip", default=DEFAULTS["ip"])
    p.add_argument("--interface", default=DEFAULTS["interface"])
    p.add_argument("--transport", choices=("ssh", "dds"), default=DEFAULTS["transport"])
    p.add_argument("--release-mode", action="store_true", help="显式释放厂商运控服务")
    p.add_argument("--no-gravity", action="store_true", help="关闭 URDF 前馈，仅用 PD（对照用）")
    p.add_argument(
        "--ignore-link-loss",
        action="store_true",
        help="右腕 yaw（28）和右夹爪（33）电机掉线时继续运行；两者停在最后指令，掉线期间不能移动",
    )
    p.add_argument(
        "--report",
        type=Path,
        default=REPORT_DIR / f"{name}_{datetime.now():%Y%m%d_%H%M%S}.json",
        help="结果 JSON 路径",
    )
    return p


def connect(args, log=None):
    """Take control at the current pose; returns (robot, session)."""
    robot = R1D(args.ip, args.interface, multicast=True, transport=args.transport)
    if args.ignore_link_loss:
        robot.link_ignored = FAULTY_LINK_MOTORS
    try:
        robot.inspect(1.0)
        robot.prepare_control(release=args.release_mode)
        gravity = None
        if not args.no_gravity:
            from r1_d.sdk.dynamics import ArmGravity

            gravity = ArmGravity(calibrated_urdf)
        return robot, ControlSession(robot, log=log, gravity=gravity)
    except BaseException:
        robot.close()
        raise


def move(session, arms=None, grippers=None, height=None, label="move", each=None):
    """Start a move of the given groups (others keep their target) and wait until it is done."""
    target = {"q": {k: list(v) for k, v in session.commanded["q"].items()}}
    target["q"].update({k: list(v) for k, v in {**(arms or {}), **(grippers or {})}.items()})
    target["height"] = session.commanded["height"] if height is None else height
    session.move_to(target, label=label)
    session.wait(each=each)


def measure(session, seconds, window=0.5):
    """Hold for `seconds`; mean feedback over the last `window` seconds."""
    rows = []

    def sample():
        motors = session.state.motor_state
        rows.append(
            [motors[i].q for ids in GROUPS.values() for i in ids]
            + [motors[i].tau_est for ids in GROUPS.values() for i in ids]
            + [session.last_height]
        )

    session.wait(max(0.0, seconds - window))
    session.wait(window, each=sample)
    mean = np.mean(rows, axis=0)
    result, offset = {"q": {}, "tau_est": {}, "height": float(mean[-1])}, 0
    count = sum(len(ids) for ids in GROUPS.values())
    for name, ids in GROUPS.items():
        result["q"][name] = mean[offset : offset + len(ids)].tolist()
        result["tau_est"][name] = mean[count + offset : count + offset + len(ids)].tolist()
        offset += len(ids)
    return result


def arm_error_degrees(session, measured):
    """Measured minus commanded angle of the 14 arm joints, degrees."""
    return {
        name: [math.degrees(a - b) for a, b in zip(measured["q"][name], session.commanded["q"][name])]
        for name in ARMS
    }


def temperatures(session):
    return [max(session.state.motor_state[i].temperature) for name in ARMS for i in GROUPS[name]]


def header(args, session):
    return {
        "time": datetime.now().isoformat(timespec="seconds"),
        "robot_ip": args.ip,
        "model": model_info(),
        "feedforward": session.gravity is not None,
        "arm_kp": ARM_KP,
        "arm_kd": ARM_KD,
        "height_m": session.last_height,
    }


def save(path, report):
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(
        report,
        ensure_ascii=False,
        indent=1,
        default=lambda value: str(value) if isinstance(value, Path) else json_default(value),
    )
    path.write_text(text + "\n")
    print(f"报告：{path}", flush=True)


def retry_pause(attempt, error):
    seconds = min(120.0, 15.0 * attempt)
    print(f"连接或控制中断：{error!r}；{seconds:.0f} 秒后重连", flush=True)
    time.sleep(seconds)


def run(main):
    """Entry point: a connection or control failure ends with one line, not a traceback."""
    try:
        main()
    except (RuntimeError, TimeoutError, subprocess.CalledProcessError) as exc:
        print(f"启动/控制失败：{exc}", file=sys.stderr)
        raise SystemExit(2)
