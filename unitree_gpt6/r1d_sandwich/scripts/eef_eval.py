"""Offline preview of EEF waypoint sequences with the SAME r1-ik planner the skill server runs
(``ik.ik_path.Planner``): for each candidate sequence, plan it from the robot's current joints and report
whether every waypoint is reachable / safe and how the elbow looks along the way (arm_posture). Nothing moves.

    bash r1d eval [--brief] '<json>'
    {"side": "right", "waist": 0.0 (optional: plan as if the waist were here), "start_q": [14] (optional),
     "end_at_start": true (optional: also check the joint-space return to the start joints, e.g. `home`),
     "candidates": {"name": [{"pos": [...], "rpy_deg": [...], "linear": true}, ...], ...}}
    {"side": "both", "candidates": {"name": [{"left": {"pos", "rpy_deg"}, "right": {"pos", "rpy_deg"},
                                               "linear": true}, ...]}}      # both TCPs at once, as `eef both`
Output per candidate: ok, where it failed, worst shoulder abduction (deg), min elbow drop below the shoulder (m),
worst shoulder-yaw gravity torque (Nm), max single-joint travel (rad), per-waypoint table gap and self-collision
clearance (m), final joints. --brief prints one line per candidate instead of the JSON.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path[:0] = [str(SCRIPT_DIR.parent / "src"), str(SCRIPT_DIR)]


def rot(r, p, y):
    import numpy as np

    r, p, y = map(math.radians, (r, p, y))
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def evaluate(P, q0, side, wps, lift, waist, end_at_start=False):
    """Plan one candidate from the joints q0 and summarise it. side "both" plans the two arms as one path."""
    import numpy as np

    from arm_posture import posture
    from ik.ik_path import se3

    sides = ("left", "right") if side == "both" else (side,)
    q, fail, travel, zmin, gaps, clear = q0.copy(), None, 0.0, [], [], []
    ps = {s: [] for s in sides}

    def step(r, name):
        nonlocal q, fail, travel
        if not r.ok:
            fail = f"{name}: {r.reason}"
            return False
        for qq in r.path[:: max(1, len(r.path) // 12)] + [r.path[-1]]:
            for s in sides:
                ps[s].append(posture(qq, s, lift, waist))
        zmin.append(min(float(P.fk(qq)[s].translation[2]) for qq in r.path for s in sides))
        gaps.append(r.min_table_gap_m)
        clear.append(r.min_clearance_m)
        travel = max(travel, float(np.max(np.abs(np.asarray(r.path[-1]) - q))))
        q = np.asarray(r.path[-1], float)
        return True

    for i, w in enumerate(wps):
        linear = bool(w.get("linear", True))
        if side == "both":
            goals = [se3(w[s]["pos"], rot(*w[s]["rpy_deg"])) for s in sides]
            r = P.plan_both(q, *goals, linear=linear)
        else:
            r = P.plan(q, side, se3(w["pos"], rot(*w["rpy_deg"])), linear=linear)
        if not step(r, f"wp{i}"):
            break
    if end_at_start and fail is None:
        step(P.plan_joints(q, q0), "return to start")

    def worst(s):
        fin = posture(q, s, lift, waist)
        return {"abd_max": max((p["abduction_deg"] for p in ps[s]), default=None),
                "elbow_drop_min": min((p["elbow_below_shoulder_m"] for p in ps[s]), default=None),
                "tau_yaw_max": max((abs(p["tau_g"]["shoulder_yaw"]) for p in ps[s]), default=None),
                "final": {"abd": fin["abduction_deg"], "drop": fin["elbow_below_shoulder_m"],
                          "tau_yaw": fin["tau_g"]["shoulder_yaw"], "ugly": fin["ugly"]}}

    out = {"ok": fail is None, "fail": fail}
    out.update(worst(side) if side != "both" else {s: worst(s) for s in sides})
    out.update({"max_joint_travel": round(travel, 2), "tcp_zmin_per_wp": [round(z, 3) for z in zmin],
                "link_gap_min_per_wp": gaps, "self_clearance_min_per_wp": clear,
                "q_end": [round(float(v), 4) for v in q]})
    return out


def brief(name, v):
    """One line per candidate: the numbers that decide between candidates."""
    arms = [v] if "abd_max" in v else [v["left"], v["right"]]
    clear, gaps = v["self_clearance_min_per_wp"], v["link_gap_min_per_wp"]
    worst = lambda key, pick: pick((a[key] for a in arms if a[key] is not None), default=None)  # noqa: E731
    return (f"{name:28s} {'OK ' if v['ok'] else 'NO '} abd_max {worst('abd_max', max)} "
            f"drop_min {worst('elbow_drop_min', min)} tau {worst('tau_yaw_max', max)} "
            f"travel {v['max_joint_travel']} selfmin {min(clear) if clear else None} "
            f"gapmin {min(gaps) if gaps else None} n_ok {len(clear)} {v['fail'] or ''}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="JSON candidate sequences")
    ap.add_argument("--brief", action="store_true", help="one line per candidate instead of the full JSON")
    a = ap.parse_args()
    import numpy as np

    from ik.ik_path import Planner
    from service.client import R1DClient, R1DError

    inp = json.loads(a.input)
    c = R1DClient(timeout=60)
    try:
        st = c.state()
    except R1DError as exc:
        sys.exit(f"eval failed: {exc}")
    waist = float(inp.get("waist", st["waist_rad"]))
    lift = st["lift_m"]
    P = Planner(lift, waist, dict(c.poses()["scene"]))
    q0 = np.array(inp.get("start_q") or (st["arm_q"]["left"] + st["arm_q"]["right"]), float)
    out = {name: evaluate(P, q0, inp["side"], wps, lift, waist, bool(inp.get("end_at_start")))
           for name, wps in inp["candidates"].items()}
    if a.brief:
        for name, v in out.items():
            print(brief(name, v))
    else:
        print(json.dumps(out))


if __name__ == "__main__":
    main()
