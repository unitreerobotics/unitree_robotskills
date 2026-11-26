"""R1-D skill server (HTTP/JSON on 127.0.0.1). Run through ``bash r1d serve``."""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from service.paths import CONFIG_DIR, RUNS_DIR

VERSION = "0.1.0"


NATIVE_SCENE_KEYS = ("table_z", "table_x_min", "tcp_min_above_table", "link_min_above_table", "floor_z",
                     "table_height_from_floor", "transit_z")
NATIVE_TRANSIT_Z = 0.78   # m, base frame: table + 0.185 -- the highest band both arms reach over the table at lift 0.522


class App:
    def __init__(self, args):
        for k in ("http_proxy", "https_proxy", "HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "all_proxy"):
            os.environ.pop(k, None)       # the robot LAN must never go through the desktop's proxy
        from robot.robot import Robot
        self.args = args
        self.scene = json.loads(Path(args.scene).read_text())
        if args.native:
            self.scene.setdefault("transit_z", NATIVE_TRANSIT_Z)
        if args.mode == "sim" and args.fixed_lift is not None:
            self.scene["lift_m"] = args.fixed_lift      # the simulated robot starts at the height the task uses
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.log_dir = RUNS_DIR / f"{stamp}-{args.mode}"
        self.world = None
        if args.mode == "sim":
            from robot.simworld import SimWorld
            self.world = SimWorld(self.scene)
        self.robot = Robot(args.mode, self.scene, self.log_dir, confirm=args.confirm, temp_guard=args.temp_guard,
                           interface=args.interface, world=self.world, poses_path=Path(args.poses),
                           grip_kp=args.grip_kp, grip_kd=args.grip_kd,
                           ignore_gripper_fault=tuple(args.ignore_gripper_fault or ()))
        if args.mode == "sim" and not args.real_cameras:
            from perception.cameras import SimCameras
            self.cams = SimCameras(self.world)
        else:
            from perception.cameras import CameraHub
            self.cams = CameraHub(args.camera_host)
        self.projector = None             # pixel -> base-frame point; needs the real cameras' calibration
        self.projector_error = "not available with the simulated cameras"
        if args.mode != "sim" or args.real_cameras:
            try:
                from perception.projection import Projector
                self.projector = Projector(str(CONFIG_DIR), self.scene)
            except Exception as e:
                self.projector_error = f"{type(e).__name__}: {e}"
        self.robot.fixed_lift = args.fixed_lift
        self.robot.state()   # world sync / feedback sanity at start

    # ------------------------------------------------------------------ routes
    def health(self):
        cam = self.cams.status()
        return {"ok": True, "mode": self.args.mode, "busy": self.robot.lock.locked(),
                "stopped": bool(self.robot.stopped), "stopped_reason": self.robot.stopped,
                "camera_names": [k for k in cam if k not in ("error", "sim")],
                "cameras": cam, "projection": "ok" if self.projector else f"unavailable: {self.projector_error}",
                "confirm": self.args.confirm, "fixed_lift": self.args.fixed_lift, "grip_gains": [self.args.grip_kp, self.args.grip_kd], "version": VERSION, "log_dir": str(self.log_dir)}

    def project(self, body):
        """Pixel(s) of /image/<cam>.jpg -> base-frame 3-D points (geometry only, no detector)."""
        if self.projector is None:
            return {"ok": False, "points": [], "reason": f"camera model unavailable: {self.projector_error}"}
        px = body.get("pixels") or ([body["pixel"]] if body.get("pixel") is not None else None)
        if not px:
            return {"ok": False, "points": [], "reason": "give pixel [u, v] or pixels [[u, v], ...] (image px)"}
        r = self.projector.project(body.get("cam", "head"), px, self.cams.all(), self.robot.robot_q(),
                                   z=body.get("z"), method=body.get("method", "auto"), window=body.get("window", 7))
        self.robot._log({"call": "project", "body": body, "result": {k: r.get(k) for k in ("ok", "reason")}})
        return r

    def image(self, cam):
        import cv2
        f = self.cams.get(cam)
        if f is None:
            return None
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(f, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 90])
        return buf.tobytes() if ok else None

    def handle(self, method, path, body):
        r = self.robot
        if method == "GET":
            if path == "/health":
                return self.health()
            if path == "/state":
                return r.state()
            if path == "/poses":
                if self.args.native:      # environment facts only: no object layout hints, no taught poses
                    return {"poses": {}, "scene": {k: self.scene[k] for k in NATIVE_SCENE_KEYS if k in self.scene}}
                return {"poses": r.load_poses(), "scene": self.scene}
            if path == "/sim/scene":
                return self.world.snapshot() if self.world else {"ok": False, "reason": "not in sim mode"}
        elif method == "POST":
            if path == "/move":
                spec = {k: body.get(k) for k in ("pos", "orient", "yaw_deg", "rpy_deg", "quat_wxyz", "pose")}
                return r.move(body.get("side"), spec, linear=bool(body.get("linear", True)), speed=body.get("speed"),
                              auto_lift=bool(body.get("auto_lift", True)), check_only=bool(body.get("check_only", False)))
            if path == "/move_both":
                keys = ("pos", "orient", "yaw_deg", "rpy_deg", "quat_wxyz", "pose")
                specs = [{k: (body.get(s) or {}).get(k) for k in keys} for s in ("left", "right")]
                return r.move_both(specs[0], specs[1], speed=body.get("speed"),
                                   check_only=bool(body.get("check_only", False)),
                                   linear=bool(body.get("linear", True)))
            if path == "/gripper":
                return r.gripper(body.get("side"), body.get("opening"), body.get("action"))
            if path == "/home":
                return r.home(body.get("side", "both"), body.get("speed"))
            if path == "/scene":
                # runtime update of the table model (robot pushed to / away from the table); not persisted
                upd = {}
                for k, lo, hi in (("table_x_min", 0.0, 3.0), ("table_z", -0.2, 1.2)):
                    if body.get(k) is not None:
                        v = float(body[k])
                        if not lo <= v <= hi:
                            return {"ok": False, "reason": f"{k} must be in [{lo}, {hi}]"}
                        upd[k] = v
                if "obstacles" in body:         # list of {name, center [x,y], size [lx,ly], top_z, yaw?, margin_xy?}
                    obs = body["obstacles"] or []
                    for o in obs:
                        if not (len(o.get("center", [])) == 2 and len(o.get("size", [])) == 2 and "top_z" in o):
                            return {"ok": False, "reason": "obstacle needs center [x,y], size [lx,ly], top_z"}
                    upd["obstacles"] = obs
                with r.lock:
                    self.scene.update(upd)
                self.robot._log({"call": "scene", "update": upd})
                return {"ok": True, "scene": {k: self.scene.get(k) for k in ("table_z", "table_x_min", "obstacles")}}
            if path == "/move_joints":
                return r.move_joints(body.get("side"), body.get("q") or [], body.get("speed"))
            if path == "/waist":
                return r.waist(float(body.get("target", 0.0)))
            if path == "/head":
                return r.head(body.get("pitch"), body.get("yaw"))
            if path == "/lift":
                return r.lift(body.get("height_m"), body.get("speed"))
            if path == "/stop":
                return r.stop(clear=bool(body.get("clear")))
            if path == "/resync":
                return r.resync()
            if path == "/project":
                return self.project(body)
            if path == "/sim/reset" and self.world is not None:
                self.world.reset(randomize=bool(body.get("randomize")), seed=body.get("seed"))
                r.stop(clear=True)
                r.home()
                r.lift(float(self.scene.get("lift_m", 0.45)))
                r.home()
                return {"ok": True, "layout": self.world.snapshot()["objects"]}
        return {"ok": False, "reason": f"no route {method} {path}"}


def make_handler(app: App):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *a):
            if app.args.verbose:
                sys.stderr.write("[http] " + fmt % a + "\n")

        def _send(self, code, payload: bytes, ctype="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def _dispatch(self, method):
            path = self.path.split("?")[0]
            try:
                if method == "GET" and path.startswith("/image/"):
                    cam = path[len("/image/"):].removesuffix(".jpg")
                    jpg = app.image(cam)
                    if jpg is None:
                        return self._send(404, json.dumps({"ok": False, "reason": f"no fresh frame from {cam}"}).encode())
                    return self._send(200, jpg, "image/jpeg")
                body = {}
                if method == "POST":
                    n = int(self.headers.get("Content-Length") or 0)
                    body = json.loads(self.rfile.read(n) or b"{}")
                out = app.handle(method, path, body)
                code = 404 if str(out.get("reason") or "").startswith("no route") else 200
                self._send(code, json.dumps(out, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)).encode())
            except Exception as e:
                traceback.print_exc()
                self._send(500, json.dumps({"ok": False, "executed": False, "reason": f"server error: {type(e).__name__}: {e}"}).encode())

        def do_GET(self):
            self._dispatch("GET")

        def do_POST(self):
            self._dispatch("POST")

    return H


def main():
    ap = argparse.ArgumentParser(description="R1-D skill server (sim / shadow / real)")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--sim", dest="mode", action="store_const", const="sim")
    g.add_argument("--shadow", dest="mode", action="store_const", const="shadow")
    g.add_argument("--real", dest="mode", action="store_const", const="real")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--scene", default=str(CONFIG_DIR / "scene.json"))
    ap.add_argument("--poses", default=str(CONFIG_DIR / "poses.json"))
    ap.add_argument("--confirm", choices=("all", "none"), default="all",
                    help="real mode: ask on the server terminal before every motion (default all)")
    ap.add_argument("--temp-guard", action="store_true",
                    help="refuse to start >= 70 °C / trip >= 80 °C on arm and waist motors (default off: the operator "
                         "relies on the motors' own fault protection; temperatures are reported in /state)")
    ap.add_argument("--interface", default="enp4s0")
    ap.add_argument("--fixed-lift", type=float, default=None,
                    help="fixed lift height (m): the lift may only go to this value, it is not changed during the task, "
                         "and automatic lifting is off")
    ap.add_argument("--ignore-gripper-fault", action="append", choices=("left", "right"),
                    help="for recovery: ignore the fault bit of this gripper motor and allow arm motion only "
                         "(commands to this gripper are disabled)")
    ap.add_argument("--grip-kp", type=float, default=5.0, help="Dex1 gripper kp (r1_d verified 5; official initial 40)")
    ap.add_argument("--grip-kd", type=float, default=0.05, help="Dex1 gripper kd (r1_d 0.05; official initial 2)")
    ap.add_argument("--camera-host", default="192.168.123.164")
    ap.add_argument("--real-cameras", action="store_true", help="sim: use the real cameras (and their calibration for /project)")
    ap.add_argument("--native", action="store_true",
                    help="agent frameworks in native mode: /poses returns only environment facts (table height and "
                         "edge, clearances, floor), no object layout hints and no taught poses")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()
    if args.mode == "real" and args.confirm == "all" and not sys.stdin.isatty():
        sys.exit("--real with --confirm all needs an interactive terminal (or pass --confirm none deliberately)")
    import socket
    with socket.socket() as probe:
        if probe.connect_ex((args.host, args.port)) == 0:
            sys.exit(f"port {args.port} is already in use (a skill server may already be running): "
                     f"check with `ss -ltnp | grep {args.port}`, then stop it or choose another --port")
    app = App(args)
    srv = ThreadingHTTPServer((args.host, args.port), make_handler(app))
    print(f"[r1d-skills] {args.mode} mode on http://{args.host}:{args.port}  logs: {app.log_dir}", flush=True)
    if args.mode == "real":
        gate = ("every motion is shadow-checked first and then confirmed in this terminal" if args.confirm == "all"
                else "every motion is shadow-checked first; per-motion confirmation is OFF (--confirm none)")
        print(f"[r1d-skills] real mode: {gate}. Emergency stop = controller L2+B.", flush=True)
    def terminate(*_):                      # `kill`: leave through the same clean-up as Ctrl+C
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        app.cams.close()
        app.robot.close()


if __name__ == "__main__":
    main()
