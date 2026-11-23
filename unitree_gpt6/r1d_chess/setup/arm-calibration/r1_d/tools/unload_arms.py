"""Locally request zero effort on R1-D arms/grippers; --execute sends commands."""

import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from r1_d.config import DEFAULTS
from r1_d.sdk import R1D
from r1_d.sdk.model import GROUPS
from r1_d.sdk.protocol import arm_unload_command, lift_command


def arm_snapshot(state):
    return {
        name: [
            {
                "q_rad": round(state.motor_state[i].q, 4),
                "dq_rad_s": round(state.motor_state[i].dq, 4),
                "tau_est_nm": round(state.motor_state[i].tau_est, 3),
            }
            for i in ids
        ]
        for name, ids in GROUPS.items()
    }


def unload(robot, sleep=time.sleep):
    """One-shot zero-effort request; never starts a trajectory or changes modes."""
    code, mode = robot.CheckMode()
    if code != 0 or not isinstance(mode, dict) or mode.get("name") != "":
        raise RuntimeError(f"厂商运控尚未释放或查询失败：{code}, {mode}")
    state = robot.arm_state()
    if any(abs(state.motor_state[i].dq) > 0.15 for ids in GROUPS.values() for i in ids):
        raise RuntimeError("双臂或夹爪仍在运动，拒绝卸力")
    initial_mode = state.mode_machine
    before = arm_snapshot(state)
    robot.publish("rt/cmd_vel", lift_command(0.0))
    for _ in range(3):
        state = robot.arm_state()
        if state.mode_machine != initial_mode:
            raise RuntimeError("mode_machine 改变，停止卸力发送")
        robot.publish("rt/lowcmd", arm_unload_command(state))
        sleep(0.02)
    sleep(0.2)
    return {"before": before, "after": arm_snapshot(robot.arm_state()), "sent": robot.sent.copy()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="实际发送卸力指令；默认只读检查")
    parser.add_argument("--ip", default=DEFAULTS["ip"])
    parser.add_argument("--interface", default=DEFAULTS["interface"])
    parser.add_argument("--transport", choices=("ssh", "dds"), default=DEFAULTS["transport"])
    args = parser.parse_args()
    robot = R1D(args.ip, args.interface, transport=args.transport, multicast=True)
    try:
        robot.inspect(1.0)  # DDS discovery: the first state needs a moment
        state = robot.arm_state()
        code, mode = robot.CheckMode()
        print(
            json.dumps(
                {"mode": {"code": code, "state": mode}, "before": arm_snapshot(state)},
                ensure_ascii=False,
                indent=2,
            ),
            flush=True,
        )
        if not args.execute:
            print("只读检查完成；加 --execute 才会卸力。", flush=True)
            return
        print("正在发送双臂及夹爪零力矩指令；手臂可能立即下落。", flush=True)
        report = unload(robot)
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        print("零力矩指令已发送；请观察实物，反馈数值不能证明已断电。", flush=True)
    finally:
        robot.close()


if __name__ == "__main__":
    main()
