"""Direct end-effector control: read camera images and give TCP waypoints.

The skill server solves and safety-checks each waypoint with r1-ik. No detector and no pose search: every
waypoint carries an explicit position and rpy. The server keeps the lift fixed.

    bash r1d eef new                                  # start a run dir under runs/, print it
    bash r1d eef prepare [--head-pitch 0.486]         # start pose, BEFORE the first arm motion of every run: lift to
                                                      # the service's fixed height, head down to look at the table
    bash r1d eef state
    bash r1d eef look [--cams head_left,left_wrist,right_wrist] [--z 0.595] [--mark "x,y,z;.."] [--crop u0,v0,u1,v1]
    bash r1d eef look --raw [--crop u0,v0,u1,v1 --scale 3]   # no ruler; with --crop: pixel ticks for `project`
    bash r1d eef project --cams head_left --z 0.61 "u,v;u,v"  # image pixels -> base-frame points on the plane z
    bash r1d eef exec '<json>'                        # {"note": .., "segments": [{"side", "speed", "waypoints": [
                                                      #   {"pos": [x,y,z], "rpy_deg": [r,p,y], "linear": true,
                                                      #    "gripper": 0..1 | "grasp", "min_opening": 0.1}]}]}
                                                      # min_opening: stop before this waypoint unless the hand's
                                                      # gripper is at least this open (= it still holds the object)
    bash r1d eef check '<json>'                       # same input; IK + safety check of each waypoint FROM THE
                                                      # CURRENT POSE, nothing moves
    bash r1d eef both '<json>'                        # both TCPs along straight lines at the same time:
                                                      # {"note", "speed", "linear": true, "left": {"pos", "rpy_deg"},
                                                      #  "right": {..}}; "linear": false = joint-space path
    bash r1d eef both-check '<json>'                  # same input, checked from the current pose, nothing moves
    bash r1d eef grip right grasp|open|close|0.45
    bash r1d eef waist 0.3
    bash r1d eef home right|left|both
    bash r1d eef keyframe TAG
    bash r1d eef rec-start | rec-stop                 # three-view recording into <run>/recording

exec/check print one line per waypoint (--json for everything); exec exits 1 when a waypoint was refused.
look draws a calibrated base-frame ruler and also saves each view as runs/latest/<cam>.jpg; every command that
moves the robot deletes those files first, so a picture there is never older than the last motion.
Every command is appended to <run>/eef_log.jsonl.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))
os.environ["no_proxy"] = os.environ["NO_PROXY"] = "127.0.0.1,localhost,192.168.123.164"
for _k in ("http_proxy", "https_proxy", "HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "all_proxy"):
    os.environ.pop(_k, None)

from service.session import Session, motion_summary  # noqa: E402
from service.client import R1DClient, R1DError  # noqa: E402
from service.paths import PACKAGE_ROOT, RUNS_DIR, SERVER_URL  # noqa: E402

CUR = RUNS_DIR / "current_run"
COMMANDS = ("new", "prepare", "state", "look", "project", "exec", "check", "both", "both-check", "grip", "waist", "home",
            "keyframe", "rec-start", "rec-stop")
MOVES = ("prepare", "exec", "both", "grip", "waist", "home")        # commands after which earlier pictures are out of date


class InputError(ValueError):
    """A malformed EEF command payload."""


def _vector3(value, field):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise InputError(f"{field} must be a list of three numbers")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise InputError(f"{field} must contain three finite numbers")
    return [float(v) for v in value]


def validate_eef_input(payload):
    """Validate the exec/check payload before opening a server connection."""
    if not isinstance(payload, dict):
        raise InputError("input must be a JSON object")
    segments = payload.get("segments")
    if not isinstance(segments, list):
        raise InputError("missing or invalid `segments`: expected a non-empty list")
    if not segments:
        raise InputError("`segments` must not be empty")
    for si, segment in enumerate(segments):
        prefix = f"segments[{si}]"
        if not isinstance(segment, dict):
            raise InputError(f"{prefix} must be an object")
        side = segment.get("side")
        if side not in ("left", "right"):
            raise InputError(f"{prefix}.side must be `left` or `right`")
        if "speed" in segment and (isinstance(segment["speed"], bool)
                                  or not isinstance(segment["speed"], (int, float))):
            raise InputError(f"{prefix}.speed must be a number")
        waypoints = segment.get("waypoints")
        if not isinstance(waypoints, list):
            raise InputError(f"{prefix}.waypoints must be a non-empty list")
        if not waypoints:
            raise InputError(f"{prefix}.waypoints must not be empty")
        for wi, waypoint in enumerate(waypoints):
            wprefix = f"{prefix}.waypoints[{wi}]"
            if not isinstance(waypoint, dict):
                raise InputError(f"{wprefix} must be an object")
            if "pos" not in waypoint:
                raise InputError(f"{wprefix}.pos is required")
            if "rpy_deg" not in waypoint:
                raise InputError(f"{wprefix}.rpy_deg is required")
            _vector3(waypoint["pos"], f"{wprefix}.pos")
            _vector3(waypoint["rpy_deg"], f"{wprefix}.rpy_deg")
            if "linear" in waypoint and not isinstance(waypoint["linear"], bool):
                raise InputError(f"{wprefix}.linear must be true or false")
            need = waypoint.get("min_opening")
            if need is not None and (isinstance(need, bool) or not isinstance(need, (int, float))
                                     or not 0.0 <= float(need) <= 1.0):
                raise InputError(f"{wprefix}.min_opening must be a number from 0 to 1")
            gripper = waypoint.get("gripper")
            if gripper is not None:
                if isinstance(gripper, bool) or not (
                    gripper == "grasp" or isinstance(gripper, (int, float))
                ):
                    raise InputError(f"{wprefix}.gripper must be `grasp` or a number from 0 to 1")
                if gripper != "grasp" and not 0.0 <= float(gripper) <= 1.0:
                    raise InputError(f"{wprefix}.gripper must be `grasp` or a number from 0 to 1")
    return payload


def parse_eef_input(raw):
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise InputError(f"invalid JSON: {exc.msg} at line {exc.lineno} column {exc.colno}") from None
    return validate_eef_input(payload)


def parse_both_input(raw):
    """'{"note": .., "speed": 0.1, "linear": true, "left": {"pos", "rpy_deg"}, "right": {"pos", "rpy_deg"}}'"""
    try:
        inp = json.loads(raw)
        specs = {side: {"pos": _vector3(inp[side]["pos"], f"{side}.pos"),
                        "rpy_deg": _vector3(inp[side]["rpy_deg"], f"{side}.rpy_deg")} for side in ("left", "right")}
    except (ValueError, KeyError, TypeError) as exc:
        raise InputError(f"invalid input: {exc!r}") from None
    return inp, specs


def _floats(text, sep=","):
    return [float(v) for v in text.split(sep)]


def latest_dir():
    return CUR.parent / "latest"


def clear_latest():
    for f in latest_dir().glob("*.jpg"):
        f.unlink()


# ---------------------------------------------------------------------- commands
def cmd_new(s, a):
    d = CUR.parent / f"{time.strftime('%Y%m%d-%H%M%S')}-{s.c.health().get('mode')}-eef-cc"
    (d / "keyframes").mkdir(parents=True)
    (d / "looks").mkdir()
    CUR.write_text(str(d))
    latest_dir().mkdir(exist_ok=True)
    clear_latest()
    print(d)


def cmd_prepare(s, a):
    out = s.prepare(a.head_pitch)
    s.log(kind="prepare", input=a.head_pitch, result=out)
    print(json.dumps(out, ensure_ascii=False))
    if not out.get("ok"):
        sys.exit(f"prepare failed: {out.get('reason')}")


def cmd_rec_start(s, a):
    out = s.run_dir / "recording"
    env = dict(os.environ, PYTHONPATH=str(PACKAGE_ROOT / "src"))
    pr = subprocess.Popen([sys.executable, "-B", "-m", "perception.record3", "--out", str(out)],
                          stdout=open(s.run_dir / "record3.log", "w"), stderr=subprocess.STDOUT, env=env,
                          start_new_session=True)
    (s.run_dir / "record3.pid").write_text(str(pr.pid))
    print(f"recording head / left_wrist / right_wrist -> {out} (pid {pr.pid})")


def cmd_rec_stop(s, a):
    import signal

    pid = int((s.run_dir / "record3.pid").read_text())
    try:
        os.kill(pid, signal.SIGINT)
    except ProcessLookupError:
        print("recorder not running")
        return
    for _ in range(20):
        time.sleep(0.5)
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            print("recording stopped:", s.run_dir / "recording")
            return
    print("recorder still running, pid", pid)


def cmd_state(s, a):
    print(json.dumps(s.state_summary(), ensure_ascii=False))


def cmd_look(s, a):
    import cv2

    z = a.z if a.z is not None else s.table_z()
    crop = _floats(a.crop) if a.crop else None
    marks = [_floats(m) for m in a.mark.split(";")] if a.mark else []
    n = len(list((s.run_dir / "looks").glob("*.jpg")))
    latest = latest_dir()
    latest.mkdir(exist_ok=True)
    out = []
    for cam in a.cams.split(","):
        if not a.raw:
            im, note = s.ruler_image(cam, z, marks, a.scale, crop)
        elif crop:
            im, note = s.pixel_crop(cam, crop, a.scale)
        else:
            im, note = s.frame(cam), "raw"
        fn = s.run_dir / "looks" / f"{n:03d}_{cam}{'' if a.raw else f'_z{z:.3f}'}.jpg"
        for path in (fn, latest / f"{cam}.jpg"):
            cv2.imwrite(str(path), im, [cv2.IMWRITE_JPEG_QUALITY, 92])
        out.append({"cam": cam, "file": str(fn), "latest": str(latest / f"{cam}.jpg"), "note": note})
        n += 1
    s.log(kind="look", z=z, files=[o["file"] for o in out])
    print(json.dumps({"looks": out, "state": s.state_summary()}, ensure_ascii=False))


def cmd_project(s, a):
    cam = a.cams.split(",")[0]
    method = a.method or ("plane" if a.z is not None else "auto")
    r = s.c.project(cam, [_floats(p) for p in a.args[0].split(";")], z=a.z, method=method)
    s.log(kind="project", cam=cam, z=a.z, input=a.args[0], result=r.get("points"))
    if a.json:
        print(json.dumps(r, ensure_ascii=False))
    else:
        for i, p in enumerate(r.get("points") or []):
            print(f"#{i} pixel {p['pixel']} -> xyz {p['xyz']} ({p['method'] or p['reason']})")
        for w in r.get("warnings") or []:
            print("warning:", w)
    if not r.get("ok"):
        sys.exit(f"project failed: {r.get('reason') or 'no pixel gave a point'}")


def cmd_check(s, a, inp):
    out = s.check_eef(inp)
    s.log(kind="check", input=inp, result=out)
    if a.json:
        print(json.dumps(out, ensure_ascii=False))
    else:
        for e in out["checked_from_current_pose"]:
            print(f"seg{e['seg']} wp{e['wp']} {'OK ' if e['ok'] else 'NO '} {e['reason'] or ''}")


def cmd_exec(s, a, inp):
    t0 = time.time()
    out = s.execute_eef(inp)
    out["seconds"] = round(time.time() - t0, 1)
    s.log(kind="exec", input=inp, result=out)
    if a.json:
        print(json.dumps(out, ensure_ascii=False))
    else:
        print(f"ok={out['ok']} stopped_at={out.get('stopped_at')} {out.get('seconds')} s")
        for e in out["results"]:
            if "gripper" in e:
                print(f"  {e['side']} wp{e['wp']} gripper {e['gripper']} -> opening {e.get('opening')} "
                      f"holding {e.get('holding')}")
            else:
                done = ("done" if e.get("executed") else "NOT HELD" if e.get("code") == "not_held"
                        else "checked, not executed (shadow)" if e.get("ok") else "REFUSED")
                print(f"  {e['side']} wp{e['wp']} {done} tcp {(e.get('final_tcp') or {}).get('pos')} "
                      f"{e.get('reason') or ''}")
        st = s.state_summary()
        print(f"  temps_max_c {st.get('temps_max_c')} hot {st.get('hot_motors')}")
    if not out.get("ok"):
        sys.exit(1)


def cmd_both(s, a, inp, specs):
    r = s.c.move_both(specs["left"], specs["right"], speed=float(inp.get("speed", 0.15)),
                      check_only=a.cmd == "both-check", linear=bool(inp.get("linear", True)))
    out = motion_summary(r)
    if r.get("final_tcp_both"):
        out["final_tcp"] = {k: v.get("pos") for k, v in r["final_tcp_both"].items()}
    s.log(kind=a.cmd, input=inp, result=out)
    print(json.dumps(out, ensure_ascii=False))
    if not out.get("ok"):
        sys.exit(1)


def cmd_simple(s, a):
    """grip SIDE grasp|open|close|0..1, waist RAD, home [SIDE], keyframe TAG."""
    if a.cmd == "grip":
        side, g = a.args[0], a.args[1]
        out = motion_summary(s.c.gripper(side, action=g) if g in ("grasp", "open", "close")
                             else s.c.gripper(side, float(g)))
    elif a.cmd == "waist":
        out = motion_summary(s.c.waist(float(a.args[0])))
    elif a.cmd == "home":
        out = motion_summary(s.c.home(a.args[0] if a.args else "both", 0.3))
    else:
        out = s.keyframe(a.args[0])
    s.log(kind=a.cmd, input=a.args, result=out)
    print(json.dumps(out, ensure_ascii=False))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=COMMANDS)
    ap.add_argument("args", nargs="*")
    ap.add_argument("--url", default=SERVER_URL)
    ap.add_argument("--cams", default="head_left")
    ap.add_argument("--z", type=float, default=None, help="look: ruler plane; project: plane the pixel rays meet")
    ap.add_argument("--raw", action="store_true", help="look: no ruler; with --crop, pixel ticks instead")
    ap.add_argument("--mark", default=None, help='look: base-frame points to draw, "x,y,z;x,y,z"')
    ap.add_argument("--crop", default=None, help="look: u0,v0,u1,v1 in original pixels")
    ap.add_argument("--scale", type=int, default=2)
    ap.add_argument("--method", choices=("auto", "stereo", "plane"), default=None,
                    help="project: default plane when --z is given, else auto (head stereo, plane fallback)")
    ap.add_argument("--head-pitch", type=float, default=None,
                    help="prepare: head pitch in rad, + = down (default: scene head_pitch, else 0.486)")
    ap.add_argument("--json", action="store_true", help="exec/check/project: print the full JSON")
    a = ap.parse_intermixed_args()        # options may come before the positional argument
    parsed = ()
    try:
        if a.cmd in ("exec", "check", "both", "both-check", "project") and not a.args:
            raise InputError(f"{a.cmd} needs one argument")
        if a.cmd in ("exec", "check"):
            parsed = (parse_eef_input(a.args[0]),)
        elif a.cmd in ("both", "both-check"):
            parsed = parse_both_input(a.args[0])
    except InputError as exc:
        ap.error(str(exc))
    s = Session(R1DClient(a.url, timeout=900), Path(CUR.read_text().strip()) if CUR.exists() else None)
    if a.cmd == "new":
        return cmd_new(s, a)
    if s.run_dir is None:
        sys.exit("no run dir: run `bash r1d eef new` first")
    if a.cmd in MOVES:
        clear_latest()
    handler = {"prepare": cmd_prepare, "rec-start": cmd_rec_start, "rec-stop": cmd_rec_stop, "state": cmd_state, "look": cmd_look,
               "project": cmd_project, "exec": cmd_exec, "check": cmd_check, "both": cmd_both,
               "both-check": cmd_both}.get(a.cmd, cmd_simple)
    try:
        handler(s, a, *parsed)
    except R1DError as exc:                 # server unreachable, or no fresh camera frame
        sys.exit(f"{a.cmd} failed: {exc}")


if __name__ == "__main__":
    main()
