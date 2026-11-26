"""How natural an R1-D arm configuration is, from r1-ik's model (calibrated masses).

abduction_deg  shoulder roll away from the body (0 = arm in the forward plane; > ~45 = elbow raised sideways)
elbow_below_shoulder_m  height of the elbow link below the shoulder roll link (negative = elbow above the shoulder)
tau_g          gravity torque (Nm) each shoulder / elbow motor has to hold in this pose; shoulder yaw (motor 17 / 24)
               is the one that overheats when the upper arm is raised sideways with the forearm forward
ugly           abduction > 45 deg or elbow above the shoulder or |tau_g shoulder yaw| > 2.5 Nm
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR.parent / "src"))

_RM = {}
JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
MAX_ABDUCTION_DEG = 45.0
MAX_SHOULDER_YAW_TORQUE_NM = 2.5


def is_ugly(abduction_deg, elbow_below_shoulder_m, shoulder_yaw_tau_nm):
    """Return whether a pose crosses the natural-arm posture thresholds."""
    return (
        abduction_deg > MAX_ABDUCTION_DEG
        or elbow_below_shoulder_m < 0.0
        or abs(shoulder_yaw_tau_nm) > MAX_SHOULDER_YAW_TORQUE_NM
    )


def _deps():
    import numpy as np
    import pinocchio as pin

    from ik.kin import ReducedModel

    return np, pin, ReducedModel


def model(lift, waist):
    _, _, reduced_model = _deps()
    key = (round(lift, 3), round(waist, 3))
    if key not in _RM:
        _RM[key] = reduced_model(
            "a7d_cal", "dual_arm", {"lift_stage1_joint": lift, "waist_yaw_joint": waist}
        )
    return _RM[key]


def posture(q14, side, lift, waist):
    np, pin, _ = _deps()
    rm = model(lift, waist)
    m = rm.model
    d = m.createData()
    q14 = np.asarray(q14, float)
    pin.forwardKinematics(m, d, q14)
    pin.updateFramePlacements(m, d)
    g = pin.computeGeneralizedGravity(m, d, q14)
    off = 0 if side == "left" else 7
    q = q14[off:off + 7]
    tau = {j: round(float(g[off + i]), 2) for i, j in enumerate(JOINTS[:4])}

    def p(name):
        return d.oMf[m.getFrameId(name)].translation

    drop = float(p(f"{side}_shoulder_roll_link")[2] - p(f"{side}_elbow_link")[2])
    abd = math.degrees(q[1]) * (1.0 if side == "left" else -1.0)
    ugly = is_ugly(abd, drop, tau["shoulder_yaw"])
    return {"abduction_deg": round(abd, 1), "elbow_below_shoulder_m": round(drop, 3), "tau_g": tau,
            "q": [round(float(v), 2) for v in q], "ugly": bool(ugly)}


if __name__ == "__main__":
    import json
    argparse.ArgumentParser(description="Report the current R1-D arm posture (read-only).").parse_args()
    from service.client import R1DClient, R1DError
    try:
        st = R1DClient().state()
    except R1DError as exc:
        sys.exit(f"posture failed: {exc}")
    q = st["arm_q"]["left"] + st["arm_q"]["right"]
    for side in ("left", "right"):
        print(side, json.dumps(posture(q, side, st["lift_m"], st["waist_rad"])))
    print("temps_max_c", st.get("temps_max_c"), "hot", st.get("hot_motors"))
