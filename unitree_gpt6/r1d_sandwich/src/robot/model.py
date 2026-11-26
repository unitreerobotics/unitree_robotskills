# Copied from r1_d sdk/model.py (copied 2026-11-26); only imports and paths changed. Keep in sync by copying; do not restyle.
"""Joint mapping and limits from the official R1D model and RoboTree config."""

import hashlib
import math
import xml.etree.ElementTree as ET

from pathlib import Path

MODEL_DIR = Path(__file__).resolve().parents[2] / "assets" / "r1d"
HOME_POSE = MODEL_DIR / "home.json"      # saved home pose of the original r1_d tools; not shipped

GROUPS = {
    "left_arm": tuple(range(15, 22)),
    "right_arm": tuple(range(22, 29)),
    "left_gripper": (31,),
    "right_gripper": (33,),
}
HOLD = (13, 29, 30)
# Arm PD gains: four times the source gains (50/40/30, kd 2). With the source gains the
# joint friction (0.2..1.5 Nm, stick-slip) left up to 1.8 deg at the target; with these
# the same moves end within 0.12 deg and run smoothly (measured 2026-11-11).
ARM_KP = (200.0, 200.0, 160.0, 160.0, 120.0, 120.0, 120.0)
ARM_KD = 8.0
LIMITS = {
    # Source limits intersected with official 2026-11-04 R1D wrist limits.
    "left_arm": (
        (-3.1416, -0.22689, -1.9199, -0.97564, -1.9199, -0.5236, -1.5708),
        (2.0944, 2.4784, 1.9199, 2.1852, 1.9199, 0.5236, 1.5708),
    ),
    "right_arm": (
        (-3.1416, -2.47849, -1.9199, -0.97564, -1.9199, -0.5236, -1.5708),
        (2.0944, 0.2268, 1.9199, 2.1852, 1.9199, 0.5326, 1.5708),
    ),
    "left_gripper": ((0.0,), (5.4,)),
    "right_gripper": ((0.0,), (5.4,)),
}
ARM_NAMES = (
    "shoulder_pitch",
    "shoulder_roll",
    "shoulder_yaw",
    "elbow",
    "wrist_roll",
    "wrist_pitch",
    "wrist_yaw",
)
# Feedforward bound per arm joint, well inside the URDF effort limits (60/33/10 Nm).
ARM_TAU_LIMIT = (20.0, 20.0, 12.0, 12.0, 12.0, 4.0, 4.0)
# Hardware fault of this robot (measured 2026-11-11): the link to the right wrist-yaw and
# gripper motors drops when the right wrist pitch is raised. In the morning that began
# above about -8 deg; by the evening 14 % of the samples between -11 and -10 deg and 30 %
# between -10 and -9 deg had no link (0.2..2.7 % below -13 deg). New targets stay below.
# This robot (192.168.123.161, checked 2026-11-13) has no faulty-link issue: all motors
# reported state=0, including right wrist-yaw (28) and right gripper (33). The right-wrist
# cap and the faulty-link list are therefore disabled for this unit. To restore the
# colleague's broken-robot limits, set RIGHT_WRIST_PITCH_MAX = -0.20 and FAULTY_LINK_MOTORS = (28, 33).
RIGHT_WRIST_PITCH_MAX = 0.5326  # rad = right wrist-pitch URDF upper limit, i.e. no extra cap
# No faulty link on this robot; --ignore-link-loss then has no special motors.
FAULTY_LINK_MOTORS = ()
OFFICIAL_URDF_SHA256 ="39ad1a206c34c3deecd33745ccda0c0c94ece4195f674dfbaf950f567f08809c"
# Use the intersection of source config and the downloaded official URDF limits.
urdf = MODEL_DIR / "r1d.urdf"
# Same kinematics and limits; arm mass/COM fitted to this robot's static joint torques.
calibrated_urdf = MODEL_DIR / "r1d_calibrated.urdf"
if urdf.exists():
    joints = {j.attrib["name"]: j.find("limit") for j in ET.parse(urdf).findall("joint")}
    for side in ("left", "right"):
        low, high = map(list, LIMITS[side + "_arm"])
        for i, name in enumerate(ARM_NAMES):
            limit = joints.get(f"{side}_{name}_joint")
            if limit is not None:
                low[i] = max(low[i], float(limit.attrib["lower"]))
                high[i] = min(high[i], float(limit.attrib["upper"]))
        LIMITS[side + "_arm"] = (tuple(low), tuple(high))


def model_info(require_official=False):
    """Identify the exact model used for limits; no IK/dynamics is implied."""
    digest = hashlib.sha256(urdf.read_bytes()).hexdigest() if urdf.is_file() else None
    info = {
        "path": str(urdf),
        "sha256": digest,
        "official_download_matches": digest == OFFICIAL_URDF_SHA256,
        "usage": "official URDF: joint limits; calibrated URDF: arm gravity feedforward; no IK or collision checking",
        "calibrated_path": str(calibrated_urdf) if calibrated_urdf.is_file() else None,
        "calibrated_sha256": (
            hashlib.sha256(calibrated_urdf.read_bytes()).hexdigest()
            if calibrated_urdf.is_file()
            else None
        ),
    }
    if require_official and not info["official_download_matches"]:
        raise RuntimeError(
            f"Official URDF missing/modified: {urdf}; deploy the complete r1_d bundle"
        )
    return info


def finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def json_default(value):
    # CycloneDDS decodes uint8 arrays such as wireless_remote as bytes.
    if isinstance(value, (bytes, bytearray)):
        return list(value)
    raise TypeError(f"Cannot serialize {type(value).__name__}")


def validate_targets(targets):
    result = {}
    for name, values in targets.items():
        if name not in GROUPS:
            raise ValueError(f"Unsupported target: {name}; allowed: {tuple(GROUPS)}")
        values = [finite(v, name) for v in values]
        if len(values) != len(GROUPS[name]):
            raise ValueError(f"{name}: expected {len(GROUPS[name])} positions")
        low, high = LIMITS[name]
        if any(not a <= q <= b for q, a, b in zip(values, low, high)):
            raise ValueError(f"{name}: target outside {low}, {high}")
        result[name] = values
    return result


def load_home_pose(path):
    """Saved home pose (tools/save_home.py) as {"q": arms and grippers, "height": m}."""
    import json

    data = json.loads(path.read_text())
    q = validate_targets(data["q"])
    if set(q) != set(GROUPS):
        raise ValueError(f"{path}: 需要完整的双臂和夹爪姿态")
    if q["right_arm"][5] > RIGHT_WRIST_PITCH_MAX:
        raise ValueError(f"{path}: 右腕 pitch 高于 {math.degrees(RIGHT_WRIST_PITCH_MAX):.1f}°（硬件故障限位）")
    height = finite(data["height"], "height")
    if not 0 <= height <= 0.84:
        raise ValueError(f"{path}: 高度超出 0..0.84 m")
    return {"q": q, "height": height}


def saved_home_pose():
    """assets/poses/home.json if it was saved, else None (home returns to the takeover pose)."""
    return load_home_pose(HOME_POSE) if HOME_POSE.is_file() else None
