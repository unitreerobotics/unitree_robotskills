"""R1-D 高度测试：升降柱按一组相对位移上下运动并回到起点，逐段核对到位高度。

双臂和夹爪保持接管时的姿态不动。每一段记录目标高度、静止后的实测高度、误差、
运动时间、平均速度，以及运动过程中双臂偏离保持目标的最大角度。升降自身的停止
容差是 2 mm；误差和回位误差不超过 --tolerance 且双臂偏离不超过 --arm-tolerance
记为 PASS。

  height_test.py                         # +50, +100, +50, 0, -50, 0 mm（相对起点）
  height_test.py --steps 20 0 -20 0
  height_test.py --steps 100 200 0 --repeat 2
"""

import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from r1_d.sdk.model import GROUPS
from r1_d.tools.common import ARMS, connect, header, measure, move, parser, run, save

# Below this the hands of hanging arms come close to the chassis (URDF meshes: 3..4 cm
# left at 0.30 m, contact near 0.20 m). Lower heights are left to the console.
MIN_HEIGHT, MAX_HEIGHT = 0.35, 0.84  # m
DEFAULT_STEPS = (50.0, 100.0, 50.0, 0.0, -50.0, 0.0)  # mm from the start height
MOVING = 0.0005  # m, height change that counts as travel


def targets(start, steps, low=MIN_HEIGHT, high=MAX_HEIGHT):
    """Absolute targets (m) for relative steps (mm), clipped to the allowed travel."""
    return [min(high, max(low, start + step / 1000.0)) for step in steps]


def travel(samples, begin, end):
    """Seconds and mean speed (m/s) between leaving `begin` and arriving at `end`."""
    left = next((t for t, h in samples if abs(h - begin) > MOVING), None)
    arrived = next((t for t, h in reversed(samples) if abs(h - end) > MOVING), None)
    if left is None or arrived is None or arrived <= left:
        return None, None
    return arrived - left, abs(end - begin) / (arrived - left)


def main():
    p = parser(__doc__, "height_test")
    p.add_argument("--steps", type=float, nargs="+", default=list(DEFAULT_STEPS), help="相对起点 mm")
    p.add_argument("--repeat", type=int, default=1, help="整组位移重复次数")
    p.add_argument("--settle", type=float, default=2.0, help="每段到位后的静止时间 s")
    p.add_argument("--tolerance", type=float, default=3.0, help="高度合格误差 mm")
    p.add_argument("--arm-tolerance", type=float, default=0.3, help="升降过程中双臂允许偏离 °")
    args = p.parse_args()
    if args.repeat < 1 or args.settle < 0.6 or not args.steps:
        p.error("repeat >= 1，settle >= 0.6 s")
    steps = list(args.steps) * args.repeat
    if steps[-1] != 0.0:
        steps.append(0.0)  # always end at the start height

    robot, session = connect(args)
    report = {"test": "height", "settings": vars(args), "segments": []}
    try:
        report.update(header(args, session))
        start = measure(session, 1.0)["height"]
        report["start_height_mm"] = start * 1000.0
        if not MIN_HEIGHT <= start <= MAX_HEIGHT:
            raise SystemExit(
                f"起始高度 {start * 1000:.0f} mm 不在本测试范围 "
                f"{MIN_HEIGHT * 1000:.0f}..{MAX_HEIGHT * 1000:.0f} mm 内"
            )
        print(f"起始高度 {start * 1000:.1f} mm", flush=True)
        previous = start
        for target in targets(start, steps):
            samples, deviation = [], [0.0]

            def watch():
                samples.append((time.monotonic(), session.last_height))
                motors = session.state.motor_state
                deviation[0] = max(
                    [deviation[0]]
                    + [
                        abs(motors[i].q - q)
                        for name in ARMS
                        for i, q in zip(GROUPS[name], session.commanded["q"][name])
                    ]
                )

            move(session, height=target, label=f"height {target * 1000:.0f}", each=watch)
            height = measure(session, args.settle)["height"]
            seconds, speed = travel(samples, previous, height)
            segment = {
                "target_mm": target * 1000.0,
                "measured_mm": height * 1000.0,
                "error_mm": (height - target) * 1000.0,
                "travel_mm": (height - previous) * 1000.0,
                "travel_seconds": seconds,
                "speed_mm_s": None if speed is None else speed * 1000.0,
                "arm_max_deviation_deg": math.degrees(deviation[0]),
            }
            segment["pass"] = (
                abs(segment["error_mm"]) <= args.tolerance
                and segment["arm_max_deviation_deg"] <= args.arm_tolerance
            )
            report["segments"].append(segment)
            print(
                f"  目标 {segment['target_mm']:7.1f} mm  实测 {segment['measured_mm']:7.1f} mm  "
                f"误差 {segment['error_mm']:+5.2f} mm  "
                + (f"用时 {seconds:5.1f} s  速度 {speed * 1000:4.1f} mm/s  " if seconds else "")
                + f"双臂最大偏离 {segment['arm_max_deviation_deg']:.3f}°  "
                + ("PASS" if segment["pass"] else "FAIL"),
                flush=True,
            )
            previous = height
        report["return_error_mm"] = (previous - start) * 1000.0
        report["pass"] = (
            all(s["pass"] for s in report["segments"])
            and abs(report["return_error_mm"]) <= args.tolerance
        )
        print(f"回到起点的误差 {report['return_error_mm']:+.2f} mm", flush=True)
    finally:
        robot.close()
        print("已停止发送；升降速度为零，机器人保持最后一帧目标。", flush=True)
        save(args.report, report)
    print("高度测试：" + ("PASS" if report["pass"] else "FAIL"), flush=True)
    raise SystemExit(0 if report["pass"] else 1)


if __name__ == "__main__":
    run(main)
