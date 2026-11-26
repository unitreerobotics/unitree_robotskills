"""Bring R1-D back to home (the ready pose) from any arms-down or low pose without passing under the table.

Problem: with the arms hanging at the sides and the hands below the table top, the direct path to home passes
under the table in front of its edge (x > table_x_min), and the planner's table check refuses it. Raising the
lift does not help (2026-11-20: lift 0.40, hands about 31 cm below the table top).

Method (every step goes through the skill server, so planning, collision, table and limit checks apply as
usual; in shadow mode it only runs on the twin):
  0. lift to --lift (default 0.522; skipped when already there)
  1. side raise: each arm in turn to shoulder abduction 1.5, elbow 1.2, shoulder pitch 0. The hand rises
     beside the body to shoulder height without moving forward (hands about 0.70 m out to each side)
  2. high reach forward: shoulder pitch -0.4, abduction 1.2, elbow 0. The hand moves forward about 20 cm
     above the table
  3. /home lowers the arms to the ready pose
Checked offline (lift 0.522, scene.json): every segment passes, closest approach to the table 9.5 cm.

    bash r1d home                            # server on 127.0.0.1:8765 (--shadow or --real)
    bash r1d home --lift 0.522 --speed 0.3
Real robot: with --confirm all the server asks at its terminal before every step. Emergency stop = controller L2+B.
"""
import argparse
import json
import sys
import urllib.request

from service.paths import SERVER_URL

# 7 joints per arm: shoulder pitch, abduction, rotation, elbow, wrist roll, pitch, yaw (abduction is + for the
# left arm and - for the right)
SIDE_RAISE = (0.0, 1.5, 0.0, 1.2, 0.0, 0.0, 0.0)
HIGH_FORWARD = (-0.4, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0)


def mirror(q7, side):
    q = list(q7)
    if side == "right":
        q[1], q[2], q[4], q[6] = -q[1], -q[2], -q[4], -q[6]
    return q


def call(url, path, body=None, timeout=960):
    req = urllib.request.Request(url + path, data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"}, method="GET" if body is None else "POST")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(req, timeout=timeout) as r:
        return json.loads(r.read())


def step(url, label, path, body):
    r = call(url, path, body)
    print(f"[go_home] {label}: ok={r.get('ok')} executed={r.get('executed')} {r.get('note') or r.get('reason') or ''}",
          flush=True)
    if not r.get("ok"):
        sys.exit(f"[go_home] stopped at this step: {label}: {r.get('reason')}")
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default=SERVER_URL)
    ap.add_argument("--lift", type=float, default=0.522, help="set the lift to this height first (m)")
    ap.add_argument("--speed", type=float, default=0.3, help="0..1, default 0.3 (slow)")
    a = ap.parse_args()

    h = call(a.url, "/health")
    st = call(a.url, "/state")
    print(f"[go_home] mode={h['mode']} stopped={h['stopped']} lift={st['lift_m']} "
          f"L={st['tcp']['left']['pos']} R={st['tcp']['right']['pos']}", flush=True)
    if h["stopped"]:
        sys.exit(f"[go_home] the server is stopped: {h.get('stopped_reason')}; the operator must clear it first "
                 "(POST /stop {\"clear\": true})")
    if h.get("fixed_lift") is not None and abs(h["fixed_lift"] - a.lift) > 0.003:
        sys.exit(f"[go_home] the server's --fixed-lift {h['fixed_lift']} differs from --lift {a.lift}")

    if abs(st["lift_m"] - a.lift) > 0.003:
        step(a.url, f"lift {st['lift_m']:.3f} -> {a.lift:.3f} m", "/lift", {"height_m": a.lift})
    for name, q7 in (("side raise", SIDE_RAISE), ("high reach forward", HIGH_FORWARD)):
        for side in ("left", "right"):
            step(a.url, f"{name} {side}", "/move_joints", {"side": side, "q": mirror(q7, side), "speed": a.speed})
    step(a.url, "home", "/home", {"speed": a.speed})
    st = call(a.url, "/state")
    print(f"[go_home] done: lift={st['lift_m']} L={st['tcp']['left']['pos']} R={st['tcp']['right']['pos']}", flush=True)


if __name__ == "__main__":
    main()
