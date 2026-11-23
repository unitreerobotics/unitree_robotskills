"""Parse operator commands into bounded joint/height targets."""

from copy import deepcopy
import math
from r1_d.sdk.model import ARM_NAMES, LIMITS, RIGHT_WRIST_PITCH_MAX, finite, validate_targets

HELP = """
1 / upper both 60     大臂：肩 pitch 相对启动角减 60°（向前抬）
2 / forearm both 60   小臂：肘关节相对启动角减 60°（向前抬）
upper left 20        只改变左肩；left / right / both 均可
joint right elbow 60  将右肘设为 URDF 绝对 +60°。关节名：shoulder_pitch(shoulder)
                     shoulder_roll shoulder_yaw elbow wrist_roll wrist_pitch wrist_yaw
jog left wrist_roll -30  在当前目标上相对转动 -30°
                     both 时角度按左臂符号，右臂的 roll / yaw 关节自动镜像
base                双臂回到下垂的基准姿态（关节测试的起点）
speed 0.2           手臂关节峰值速度 rad/s，0.02..0.5
grip both 40         夹爪开到 40% 并持续保持，0=闭，100=开
grip-mm left 30      标定后的夹爪内侧间距 30 mm（需提供标定文件）
up 20 / down 20      从当前实测高度上升 / 下降 20 mm
height 330          升降柱回零行程设为 330 mm
0 / home            直接回到 assets/poses/home.json 的关节、夹爪和高度（无文件时回到启动姿态）
stop                停止当前轨迹，保持当前实测姿态，升降速度置零
s / status          显示实测角度、目标角度、夹爪和升降高度
q / quit            停止发送并退出；不自动归位或卸力
help                显示帮助
动作期间仍可输入 stop / home / status / quit。其他新动作会被拒绝。
"""
JOINTS = {name: i for i, name in enumerate(ARM_NAMES)}
JOINTS["shoulder"] = 0
# Right arm angle = MIRROR * left arm angle for a symmetric pose.
MIRROR = (1, -1, -1, 1, -1, 1, -1)
# Hanging arms with bent elbows: low motor torque, start of the joint test (rad).
BASE = {
    "left_arm": [0.06, 0.10, 0.0, 1.40, 0.0, 0.09, 0.0],
    "right_arm": [0.09, -0.10, 0.0, 1.40, 0.0, -0.21, 0.0],
}


def sides(value):
    if value == "both":
        return ("left", "right")
    if value in ("left", "right"):
        return (value,)
    raise ValueError("侧别须为 left / right / both")


def width_to_motor(calibration, side, width):
    """Interpolate measured [motor_position, inner_gap_mm] pairs, never URDF travel."""
    points = calibration.get(side)
    if not isinstance(points, list) or len(points) < 2:
        raise ValueError(f"{side} 夹爪缺少实测毫米标定；可先使用 grip 的百分比开度")
    if any(not isinstance(p, list) or len(p) != 2 for p in points):
        raise ValueError("每个标定点须为 [电机位置, 间距毫米]")
    points = [(finite(p[0], "motor"), finite(p[1], "gap_mm")) for p in points]
    if any(not 0 <= motor <= 5.4 or gap < 0 for motor, gap in points):
        raise ValueError("标定电机位置须为 0..5.4，间距须非负")
    if any(b[0] <= a[0] or b[1] <= a[1] for a, b in zip(points, points[1:])):
        raise ValueError("标定点的电机位置与间距须严格递增")
    width = finite(width, "gap_mm")
    if not points[0][1] <= width <= points[-1][1]:
        raise ValueError("间距超出已标定范围，禁止外推")
    for (q0, w0), (q1, w1) in zip(points, points[1:]):
        if w0 <= width <= w1:
            return q0 + (q1 - q0) * (width - w0) / (w1 - w0)
    raise ValueError("无有效标定区间")


def plan_command(line, commanded, baseline, measured_height, calibration=None):
    """Pure target planning; all angles are degrees and all distances mm at the UI."""
    tokens = line.lower().split()
    if tokens == ["1"]:
        tokens = ["upper", "both", "60"]
    if tokens == ["2"]:
        tokens = ["forearm", "both", "60"]
    if not tokens:
        raise ValueError("空指令")
    result = deepcopy(commanded)
    action = tokens[0]
    if action in ("upper", "forearm", "grip", "grip-mm") and len(tokens) == 3:
        value = finite(tokens[2], "target")
        for side in sides(tokens[1]):
            if action in ("upper", "forearm"):
                if not -60 <= value <= 60:
                    raise ValueError("本测试的相对抬臂角度限制为 -60..60°")
                index = 0 if action == "upper" else 3
                result["q"][side + "_arm"][index] = baseline["q"][side + "_arm"][
                    index
                ] - math.radians(value)
            elif action == "grip":
                if not 0 <= value <= 100:
                    raise ValueError("夹爪开度须为 0..100%")
                result["q"][side + "_gripper"] = [5.4 * value / 100.0]
            else:
                result["q"][side + "_gripper"] = [width_to_motor(calibration or {}, side, value)]
    elif action in ("joint", "jog") and len(tokens) == 4:
        if tokens[2] not in JOINTS:
            raise ValueError("关节名须为 " + " / ".join(JOINTS))
        index = JOINTS[tokens[2]]
        value = math.radians(finite(tokens[3], "degrees"))
        chosen = sides(tokens[1])
        for side in chosen:
            sign = MIRROR[index] if side == "right" and len(chosen) == 2 else 1
            q = result["q"][side + "_arm"]
            q[index] = sign * value + (q[index] if action == "jog" else 0.0)
    elif tokens == ["base"]:
        for name, q in BASE.items():
            result["q"][name] = list(q)
    elif action in ("up", "down", "height") and len(tokens) == 2:
        value = finite(tokens[1], "mm") / 1000.0
        if action != "height" and value < 0:
            raise ValueError("up/down 距离须非负")
        result["height"] = (
            value if action == "height" else measured_height + (value if action == "up" else -value)
        )
    else:
        raise ValueError("指令格式不正确，输入 help 查看示例")
    return checked(result, commanded)


SIDE_NAMES = {"left_arm": "左臂", "right_arm": "右臂"}


def plan_targets(arms, grippers, height_mm, commanded):
    """Targets from the web panel: degrees per arm joint, gripper opening in %, lift travel
    in mm. None (or a missing entry) keeps the commanded value."""
    result = deepcopy(commanded)
    for name, values in (arms or {}).items():
        if name not in SIDE_NAMES or len(values) != len(ARM_NAMES):
            raise ValueError(f"{name}: 需要 {len(ARM_NAMES)} 个关节角")
        low, high = LIMITS[name]
        for i, value in enumerate(values):
            if value is None:
                continue
            q = math.radians(finite(value, ARM_NAMES[i]))
            if not low[i] <= q <= high[i]:
                raise ValueError(
                    f"{SIDE_NAMES[name]} {ARM_NAMES[i]} 目标 {value:.2f}° 超出限位 "
                    f"{math.degrees(low[i]):.1f}..{math.degrees(high[i]):.1f}°"
                )
            result["q"][name][i] = q
    for name, value in (grippers or {}).items():
        if name not in ("left_gripper", "right_gripper"):
            raise ValueError(f"未知夹爪 {name}")
        if value is not None:
            value = finite(value, name)
            if not 0 <= value <= 100:
                raise ValueError("夹爪开度须为 0..100%")
            result["q"][name] = [5.4 * value / 100.0]
    if height_mm is not None:
        result["height"] = finite(height_mm, "mm") / 1000.0
    return checked(result, commanded)


def checked(result, commanded):
    """Limits shared by every command; a value already commanded may stay as it is."""
    result["q"] = validate_targets(result["q"])
    pitch = result["q"]["right_arm"][5]
    if pitch > RIGHT_WRIST_PITCH_MAX and pitch != commanded["q"]["right_arm"][5]:
        raise ValueError(
            f"右腕 pitch 目标须不高于 {math.degrees(RIGHT_WRIST_PITCH_MAX):.1f}°："
            "本机右腕 pitch 抬高后右腕 yaw 和右夹爪电机掉线（硬件故障）"
        )
    if not math.isfinite(result["height"]) or not 0 <= result["height"] <= 0.84:
        raise ValueError("升降目标超出回零行程 0..840 mm")
    return result


def execute_command(session, line, calibration=None):
    line = line.strip().lower()
    if line in ("home", "0"):
        session.home()
    elif line == "stop":
        session.stop()
    elif line.split()[:1] == ["speed"] and len(line.split()) == 2:
        rate = finite(line.split()[1], "rad/s")
        if not 0.02 <= rate <= 0.5:
            raise ValueError("手臂速度须为 0.02..0.5 rad/s")
        session.arm_rate = rate
    else:
        if session.motion is not None:
            raise ValueError("正在执行动作；可输入 stop、home 或 status")
        target = plan_command(
            line, session.commanded, session.baseline, session.robot.height(), calibration
        )
        session.move_to(target, label=line)
