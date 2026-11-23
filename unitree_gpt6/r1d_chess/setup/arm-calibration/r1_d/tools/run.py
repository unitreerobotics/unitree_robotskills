"""Start the console or a test with this directory's Python environment.

  run.py [options]          interactive control console (bench.py)
  run.py joint [options]    joint test (joint_test.py)
  run.py height [options]   height test (height_test.py)
  run.py hold MODE ...      waist/head sweep, fit and stop-accuracy check (hold_calib.py)

Control runs locally; SSH only forwards DDS messages (or direct DDS with --transport dds).
"""

import argparse
from datetime import datetime
from pathlib import Path
import shlex
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from r1_d.config import DEFAULTS, LOCAL_PYTHON


def build_command(args):
    tools = Path(__file__).resolve().parent
    root = tools.parent
    log = args.log or root / "reports" / (
        "bench_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".jsonl"
    )
    command = [
        args.python,
        "-B",
        str(tools / "bench.py"),
        "--ip",
        args.ip,
        "--interface",
        args.interface,
        "--log",
        str(log),
        "--transport",
        args.transport,
    ]
    if args.check:
        command.append("--check")
    if args.release_mode:
        command.append("--release-mode")
    if args.no_gravity:
        command.append("--no-gravity")
    if args.ignore_link_loss:
        command.append("--ignore-link-loss")
    if args.gripper_calibration:
        command += ["--gripper-calibration", str(args.gripper_calibration.resolve())]
    return command


TESTS = {"joint": "joint_test.py", "height": "height_test.py", "hold": "hold_calib.py"}


def main():
    python = str(LOCAL_PYTHON) if LOCAL_PYTHON.is_file() else sys.executable
    if sys.argv[1:2] and sys.argv[1] in TESTS:
        script = Path(__file__).resolve().parent / TESTS[sys.argv[1]]
        raise SystemExit(subprocess.run([python, "-B", str(script), *sys.argv[2:]]).returncode)
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--python", default=python)
    p.add_argument("--ip", default=DEFAULTS["ip"], help="R1D Wi-Fi 地址")
    p.add_argument(
        "--interface", default=DEFAULTS["interface"], help="本机连接 unitree_internal Wi-Fi 的网卡"
    )
    p.add_argument("--log", type=Path)
    p.add_argument("--transport", choices=("ssh", "dds"), default=DEFAULTS["transport"])
    p.add_argument("--check", action="store_true", help="只读检查通信和模式，不发送动作")
    p.add_argument("--release-mode", action="store_true")
    p.add_argument("--no-gravity", action="store_true", help="关闭重力前馈，仅用 PD（对照用）")
    p.add_argument(
        "--ignore-link-loss", action="store_true", help="右腕 yaw / 右夹爪电机掉线时继续运行"
    )
    p.add_argument("--gripper-calibration", type=Path)
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()
    if args.gripper_calibration and not args.gripper_calibration.is_file():
        p.error("Calibration file does not exist")
    command = build_command(args)
    print("本机控制命令:", shlex.join(command), flush=True)
    if not args.dry_run:
        raise SystemExit(subprocess.run(command).returncode)


if __name__ == "__main__":
    main()
