"""R1-D 关节测试：每个手臂关节单独走完整个行程并返回，逐点核对到位精度。

从基准姿态（双臂自然下垂、肘 80°）出发，被测关节依次到达行程两侧的中点和端点
再返回，其他 13 个关节保持不动。每个停点静止后记录目标角、实测角、误差、电机
力矩和前馈力矩，并给出每个关节的最大到位误差、回位误差、同一目标从两个方向
到达的差值（回差）和其他关节的最大偏离（连同是哪个关节）。全部不超过 --tolerance
记为 PASS。默认 0.5° 是当前增益下肩关节静摩擦带的宽度：前馈消除了系统偏差，停点
落在静摩擦带内的哪里是随机的（实测 p95 约 0.2°，最大 0.46°）。

  joint_test.py                      # 左臂 7 个关节，再右臂 7 个关节，最后两个夹爪
  joint_test.py --side left --joints elbow wrist_roll
  joint_test.py --fraction 0.5       # 只走一半行程
"""

import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from r1_d.sdk.model import ARM_NAMES, RIGHT_WRIST_PITCH_MAX
from r1_d.tools.commands import BASE
from r1_d.tools.common import (
    ARMS,
    arm_error_degrees,
    connect,
    header,
    measure,
    move,
    parser,
    retry_pause,
    run,
    save,
    temperatures,
)

# Absolute range (deg) of each joint while the others stay at BASE. Checked against the
# URDF collision meshes with 6 cm clearance to torso, head, column and chassis for lift
# heights from 0.40 m. The right wrist pitch stops below RIGHT_WRIST_PITCH_MAX, which
# leaves no travel above its base angle.
RANGE = {
    "left_arm": ((-130, 12), (0, 135), (-85, 105), (-38, 97), (-105, 105), (-26, 25), (-85, 85)),
    "right_arm": (
        (-130, 18),
        (-135, 0),
        (-105, 58),
        (-38, 102),
        (-105, 105),
        (-26, math.degrees(RIGHT_WRIST_PITCH_MAX) - 0.4),
        (-85, 85),
    ),
}
GRIPPER_STOPS = (1.0, 3.0, 5.0, 3.0, 1.0)  # motor units, 0 closed .. 5.4 open
MIN_HEIGHT = 0.40  # m
NEAR_BASE = math.radians(20.0)
# Stiction band of the shoulder joints at ARM_KP (about 1.6 Nm / 200 Nm/rad). The largest
# static error in 903 calibration waypoints was 0.46 deg, the 95th percentile 0.2 deg.
TOLERANCE = 0.5  # deg
HOT, COOL = 70, 62  # deg C


def stops(base, low, high, fraction):
    """Targets (deg) of one joint: middle and end of each side, back through the middle."""
    result = []
    for end in (low, high):
        end = base + fraction * (end - base)
        if abs(end - base) < 2.0:
            continue
        middle = [(base + end) / 2.0] if abs(end - base) >= 10.0 else []
        result += middle + [end] + middle + [base]
    return result


def summarize(points, tolerance, arm_tolerance):
    """Worst target error, return error and two-direction difference of one joint."""
    errors = [p["error"] for p in points]
    by_target = {}
    for p in points:
        by_target.setdefault(round(p["target"], 6), []).append(p["measured"])
    hysteresis = max([max(v) - min(v) for v in by_target.values() if len(v) > 1] + [0.0])
    others = max(points, key=lambda p: p["others_max_error"])
    result = {
        "max_error": max(map(abs, errors)),
        "return_error": errors[-1],
        "hysteresis": hysteresis,
        "others_max_error": others["others_max_error"],
        "others_worst": others.get("others_worst"),
    }
    result["pass"] = result["max_error"] <= tolerance and result["others_max_error"] <= arm_tolerance
    return result


class Runner:
    def __init__(self, args):
        self.args = args
        self.robot = self.session = None
        self.connections = 0
        self.gripper_start = {}  # where each gripper was found; kept over reconnects

    def close(self):
        if self.robot is not None:
            try:
                self.robot.close()
            finally:
                self.robot = self.session = None

    def ensure(self):
        """Connected, controlling, and at the base pose."""
        if self.session is not None:
            return self.session
        self.robot, self.session = connect(self.args)
        session = self.session
        session.arm_rate = self.args.rate
        if session.last_height < MIN_HEIGHT:
            raise SystemExit(
                f"升降高度 {session.last_height * 1000:.0f} mm 低于 {MIN_HEIGHT * 1000:.0f} mm："
                "基准姿态和行程只在该高度以上核对过，请先升高（控制台 height 500）"
            )
        away = max(
            abs(a - b) for name in ARMS for a, b in zip(session.commanded["q"][name], BASE[name])
        )
        if away > NEAR_BASE and not self.connections and not self.args.go_base:
            raise SystemExit(
                f"当前姿态偏离基准姿态 {math.degrees(away):.0f}°。先用控制台放回，"
                "或加 --go-base 由本工具直接关节插值过去（该段路径没有碰撞检查）"
            )
        self.connections += 1
        if away > 1e-4:
            move(session, arms=BASE, label="base")
        return session

    def cool(self):
        session = self.ensure()
        if max(temperatures(session)) < HOT:
            return
        print(f"电机温度 {max(temperatures(session))}°C，在基准姿态等待降温", flush=True)
        started = time.monotonic()
        while max(temperatures(session)) >= COOL and time.monotonic() - started < 1800:
            session.wait(10.0)

    def point(self, target, name, index, unit):
        """Measure after the stop: error of the tested joint and of the other arm joints."""
        session = self.session
        measured = measure(session, self.args.settle)
        errors = arm_error_degrees(session, measured)
        value = measured["q"][name][index]
        feedforward = session.feedforward()
        others = {
            (k, j): abs(e) for k in ARMS for j, e in enumerate(errors[k]) if (k, j) != (name, index)
        }
        worst = max(others, key=others.get)
        scale = math.degrees if name in ARMS else float
        return {
            "target": scale(target),
            "measured": scale(value),
            "error": scale(value - target),
            "unit": unit,
            "tau_est_nm": measured["tau_est"][name][index],
            "feedforward_nm": feedforward[name][index] if feedforward and name in ARMS else None,
            "others_max_error": others[worst],
            "others_worst": f"{worst[0]} {ARM_NAMES[worst[1]]}",
        }

    def arm_joint(self, name, index):
        session = self.ensure()
        base = math.degrees(BASE[name][index])
        low, high = RANGE[name][index]
        points = []
        for target in stops(base, low, high, self.args.fraction):
            q = list(BASE[name])
            q[index] = math.radians(target)
            move(session, arms={name: q}, label=f"{name} {ARM_NAMES[index]} {target:.1f}")
            point = self.point(q[index], name, index, "deg")
            points.append(point)
            print(
                f"    目标 {point['target']:8.2f}°  实测 {point['measured']:8.2f}°  "
                f"误差 {point['error']:+.3f}°  力矩 {point['tau_est_nm']:+.2f} Nm  "
                f"其他关节最大偏离 {point['others_max_error']:.3f}°（{point['others_worst']}）",
                flush=True,
            )
        return points

    def gripper(self, name):
        session = self.ensure()
        start = self.gripper_start.setdefault(name, session.commanded["q"][name][0])
        points = []
        for target in GRIPPER_STOPS + (start,):
            move(session, grippers={name: [target]}, label=f"{name} {target:.2f}")
            point = self.point(target, name, 0, "motor")
            points.append(point)
            print(
                f"    目标 {point['target']:.3f}  实测 {point['measured']:.3f}  "
                f"误差 {point['error']:+.4f}（电机单位）",
                flush=True,
            )
        return points


def main():
    p = parser(__doc__, "joint_test")
    p.add_argument("--side", choices=("left", "right", "both"), default="both")
    p.add_argument("--joints", nargs="+", choices=ARM_NAMES, default=list(ARM_NAMES))
    p.add_argument("--no-grippers", action="store_true", help="不测夹爪")
    p.add_argument("--fraction", type=float, default=1.0, help="行程比例 0.05..1")
    p.add_argument("--rate", type=float, default=0.3, help="关节峰值速度 rad/s，0.05..0.5")
    p.add_argument("--settle", type=float, default=1.5, help="每个停点的静止时间 s（取最后 0.5 s）")
    p.add_argument("--tolerance", type=float, default=TOLERANCE, help="手臂关节合格误差 °")
    p.add_argument("--gripper-tolerance", type=float, default=0.1, help="夹爪合格误差（电机单位）")
    p.add_argument("--retries", type=int, default=3, help="每个关节断连后的重试次数")
    p.add_argument("--go-base", action="store_true", help="允许从任意姿态直接插值到基准姿态")
    args = p.parse_args()
    if not 0.05 <= args.fraction <= 1.0 or not 0.05 <= args.rate <= 0.5 or args.settle < 0.6:
        p.error("fraction 0.05..1，rate 0.05..0.5 rad/s，settle >= 0.6 s")
    sides = ("left", "right") if args.side == "both" else (args.side,)
    todo = [(side + "_arm", ARM_NAMES.index(joint)) for side in sides for joint in args.joints]
    if not args.no_grippers:
        todo += [(side + "_gripper", 0) for side in sides]

    runner = Runner(args)
    report = {"test": "joint", "settings": vars(args), "joints": []}
    try:
        for attempt in range(1, args.retries + 2):
            try:
                report.update(header(args, runner.ensure()))
                break
            except Exception as exc:
                runner.close()
                if attempt > args.retries:
                    raise
                retry_pause(attempt, exc)
        for name, index in todo:
            label = f"{name} {ARM_NAMES[index]}" if name in ARMS else name
            tolerance = args.tolerance if name in ARMS else args.gripper_tolerance
            record = {"group": name, "joint": ARM_NAMES[index] if name in ARMS else "gripper"}
            for attempt in range(1, args.retries + 2):
                print(f"[{label}]" + (f" 第 {attempt} 次" if attempt > 1 else ""), flush=True)
                try:
                    runner.cool()
                    points = runner.arm_joint(name, index) if name in ARMS else runner.gripper(name)
                    record.update(points=points, **summarize(points, tolerance, args.tolerance))
                    break
                except KeyboardInterrupt:
                    raise
                except Exception as exc:
                    record.setdefault("interruptions", []).append(repr(exc))
                    runner.close()
                    if attempt > args.retries:
                        record.update({"pass": False, "error": repr(exc)})
                        break
                    retry_pause(attempt, exc)
            record["attempts"] = attempt
            report["joints"].append(record)
            save_quietly(args.report, report)
        runner.ensure()  # back at the base pose
    finally:
        runner.close()
        print("已停止发送；机器人保持最后一帧目标。", flush=True)

    print("\n组别           关节             行程          最大误差  回位误差   回差   他关节  结果  偏离最大的他关节")
    for r in report["joints"]:
        if "max_error" not in r:
            print(f"{r['group']:14s} {r['joint']:15s} 未完成：{r.get('error')}")
            continue
        targets = [p["target"] for p in r["points"]]
        unit = "°" if r["group"] in ARMS else " "
        print(
            f"{r['group']:14s} {r['joint']:15s} {min(targets):7.1f}..{max(targets):<6.1f}"
            f" {r['max_error']:8.3f}{unit} {r['return_error']:+8.3f}{unit} {r['hysteresis']:6.3f}"
            f" {r['others_max_error']:7.3f}  {'PASS' if r['pass'] else 'FAIL'}"
            f"  {r.get('others_worst') or ''}"
        )
    report["pass"] = all(r["pass"] for r in report["joints"])
    save(args.report, report)
    print("关节测试：" + ("全部 PASS" if report["pass"] else "存在 FAIL"), flush=True)
    raise SystemExit(0 if report["pass"] else 1)


def save_quietly(path, report):
    """Keep partial results on disk without the console line."""
    import contextlib
    import io

    with contextlib.redirect_stdout(io.StringIO()):
        save(path, report)


if __name__ == "__main__":
    run(main)
