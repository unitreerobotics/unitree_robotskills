"""R1-D skill server client. Python >= 3.8, standard library only.

The server (``service.server``, run through ``bash r1d serve``) owns DDS, IK, the safety checks,
and the cameras.

Frame: R1-D URDF base, metres, x forward / y left / z up. TCP = Dex1 finger pads.
At the standard pose the TCP axes equal the base axes: tool x = along the fingers (forward),
tool y = finger closing axis (left), tool z = up.

Orientation of a target, give at most one of
    orient="forward"  fingers point forward (tool = base axes)              [default: keep current]
    orient="down"     fingers point down, closing axis = base y (left/right)
    orient="flat"     fingers forward, closing axis vertical (tool roll 90°): carries a slice flat
    yaw_deg           extra rotation about base z applied on top of `orient` (turns the closing axis)
    rpy_deg=[r,p,y]   tool orientation as R = Rz(y) Ry(p) Rx(r) in the base frame
    quat_wxyz=[w,x,y,z]

Every motion returns a dict with at least {"ok": bool, "executed": bool, "reason": str|None,
"code": str, "stopped": bool}. code is "ok" or one of: unreachable, too_far, collision, joint_limit, table,
lift_range, invalid_argument, operator_declined, watchdog, stopped, not_ready, executor_refused, refused.
ok=False never raises; check it.  executed=False in shadow mode (checks only, robot does not move).
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request

from service.paths import SERVER_URL


class R1DError(RuntimeError):
    pass


class R1DClient:
    def __init__(self, url: str = SERVER_URL, timeout: float = 180.0):
        self.url = url.rstrip("/")
        self.timeout = timeout
        # the robot LAN must never go through the desktop's http proxy
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    # ------------------------------------------------------------------ transport
    def _call(self, method: str, path: str, body: dict | None = None, raw: bool = False):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.url + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"})
        try:
            with self._opener.open(req, timeout=self.timeout) as r:
                payload = r.read()
        except urllib.error.HTTPError as e:
            payload = e.read()
            try:
                return json.loads(payload)
            except ValueError:
                raise R1DError(f"{method} {path}: HTTP {e.code} {payload[:200]!r}") from None
        except urllib.error.URLError as e:
            raise R1DError(f"R1-D skill server unreachable at {self.url}: {e.reason}. "
                           "Start it with `bash r1d serve --sim|--shadow|--real`.") from None
        return payload if raw else json.loads(payload)

    # ------------------------------------------------------------------ observation
    def health(self) -> dict:
        """{"ok", "mode": "sim"|"shadow"|"real", "busy", "cameras": [...], "version"}"""
        return self._call("GET", "/health")

    def state(self) -> dict:
        """Robot state.
        {"mode", "lift_m", "waist_rad",
         "arm_q": {"left": [7], "right": [7]},
         "tcp": {"left": {"pos": [x,y,z], "quat_wxyz": [...], "rpy_deg": [...]}, "right": {...}},
         "gripper": {"left": {"opening": 0..1, "cmd_opening": 0..1|None, "holding": bool}, "right": {...}},
         "temps_max_c", "hot_motors", "mode_machine", "feedback_age_ms"}"""
        return self._call("GET", "/state")

    def image(self, cam: str = "head_left") -> bytes:
        """JPEG bytes. cam in head_left | head_right | left_wrist | right_wrist.
        Raises R1DError when the server has no fresh frame from that camera."""
        data = self._call("GET", f"/image/{cam}.jpg", raw=True)
        if isinstance(data, dict):          # the server answered with a JSON error instead of a picture
            raise R1DError(f"no image from {cam}: {data.get('reason')}")
        return data

    def project(self, cam: str, pixels, z: float | None = None, method: str = "auto", window: int = 7) -> dict:
        """Pixels [[u, v], ...] of /image/<cam>.jpg -> base-frame points; geometry only, nothing is detected.
        method "plane": where each pixel's ray meets the horizontal plane z (default the table top);
        "stereo": head-camera depth; "auto": stereo for the head with the plane as fallback, plane for a wrist.
        {"ok", "points": [{"pixel", "xyz": [x,y,z]|None, "method", "reason"}], "warnings": [...]}"""
        body = {"cam": cam, "pixels": pixels, "method": method, "window": window}
        if z is not None:
            body["z"] = float(z)
        return self._call("POST", "/project", body)

    # ------------------------------------------------------------------ named poses / scene
    def poses(self) -> dict:
        """Taught / configured named poses: {"poses": {name: {"side", "pos", "quat_wxyz"}}, "scene": {...}}"""
        return self._call("GET", "/poses")

    # ------------------------------------------------------------------ motion
    def move(self, side: str, pos=None, *, orient: str | None = None, yaw_deg: float | None = None,
             rpy_deg=None, quat_wxyz=None, pose: str | None = None, speed: float = 0.5,
             linear: bool = True, check_only: bool = False, auto_lift: bool = True) -> dict:
        """Move one TCP to an absolute pose (the other arm holds still).
        pos: [x,y,z] base frame (None = keep position). pose: a named pose instead of pos/orient.
        speed 0..1 (1 = fastest the server allows). linear=True: straight Cartesian line.
        check_only=True: plan and check (IK, limits, collisions, table) without moving; if only another lift
        height makes it reachable, ok=true with needs_lift_m (a real move would auto-lift there first).
        auto_lift=False: fail instead of driving the torso lift when the pose is out of reach.
        Every motion result also carries "stopped": true once the server latched a stop."""
        body = {"side": side, "pos": pos, "orient": orient, "yaw_deg": yaw_deg, "rpy_deg": rpy_deg,
                "quat_wxyz": quat_wxyz, "pose": pose, "speed": speed, "linear": linear,
                "check_only": check_only, "auto_lift": auto_lift}
        return self._call("POST", "/move", {k: v for k, v in body.items() if v is not None})

    def move_both(self, left: dict, right: dict, speed: float = 0.5, check_only: bool = False,
                  linear: bool = True) -> dict:
        """Move both TCPs at the same time. left / right: {"pos": [x,y,z], "rpy_deg": [r,p,y]}.
        linear=True: straight lines; linear=False: one joint-space path. check_only=True plans without moving."""
        return self._call("POST", "/move_both", {"left": left, "right": right, "speed": speed,
                                                 "check_only": check_only, "linear": linear})

    def gripper(self, side: str, opening: float | None = None, *, action: str | None = None) -> dict:
        """opening 0 (closed) .. 1 (fully open), or action="open"|"close"|"grasp".
        "grasp" closes softly and reports whether something is held (fingers stopped early)."""
        body = {"side": side}
        if opening is not None:
            body["opening"] = float(opening)
        if action is not None:
            body["action"] = action
        return self._call("POST", "/gripper", body)

    def set_obstacles(self, obstacles: list) -> dict:
        """Scene boxes the hands must keep out of: [{name, center [x,y], size [lx,ly], top_z, yaw?, margin_xy?}]."""
        return self._call("POST", "/scene", {"obstacles": obstacles})

    def set_table(self, table_x_min: float | None = None, table_z: float | None = None) -> dict:
        """Tell the server where the table is now (base frame): table_x_min = table edge distance ahead of the
        base (chassis front is at 0.227), table_z = table top. Not persisted; config/scene.json is the default."""
        body = {k: v for k, v in (("table_x_min", table_x_min), ("table_z", table_z)) if v is not None}
        return self._call("POST", "/scene", body)

    def move_joints(self, side: str, q7, speed: float = 0.5) -> dict:
        """Replay: move one arm to these 7 joint angles (rad), joint-space path, safety-checked, no IK search."""
        return self._call("POST", "/move_joints", {"side": side, "q": [float(v) for v in q7], "speed": speed})

    def waist(self, target: float = 0.0) -> dict:
        """Turn the torso (waist yaw, rad, |target| <= 0.35) slowly; arms keep their joint angles. Use away from
        the table to re-centre the torso (it can turn while the robot is in damping)."""
        return self._call("POST", "/waist", {"target": float(target)})

    def head(self, pitch: float | None = None, yaw: float | None = None) -> dict:
        """Point the head (stereo camera). pitch rad, + = look down (<= 0.60); yaw rad, + = left (<= 0.50).
        Look down ~0.5 rad to see the table in front of the robot."""
        body = {}
        if pitch is not None:
            body["pitch"] = float(pitch)
        if yaw is not None:
            body["yaw"] = float(yaw)
        return self._call("POST", "/head", body)

    def lift(self, height_m: float) -> dict:
        """Drive the torso lift (0.35..0.84 m). Arms keep their joint angles: both TCPs move vertically
        by the same amount (a held object goes with them). Slow: about 3 cm/s."""
        return self._call("POST", "/lift", {"height_m": float(height_m)})

    def home(self, side: str = "both", speed: float = 0.4) -> dict:
        """Glide to the standard ready pose (forearms forward)."""
        return self._call("POST", "/home", {"side": side, "speed": speed})

    def stop(self) -> dict:
        """Refuse queued motions and soft-stop the one running."""
        return self._call("POST", "/stop", {})

    # ------------------------------------------------------------------ sim helpers
    def sim_reset(self, randomize: bool = False, seed: int | None = None) -> dict:
        """Sim only: restore the scene; randomize=True jitters the layout (±3 cm, toaster yaw ±10°)."""
        body = {"randomize": bool(randomize)}
        if seed is not None:
            body["seed"] = int(seed)
        return self._call("POST", "/sim/reset", body)

    def sim_scene(self) -> dict:
        """Sim only: the simulated objects and what is attached to which gripper."""
        return self._call("GET", "/sim/scene")


if __name__ == "__main__":
    import sys
    c = R1DClient()
    print(json.dumps(c.health(), indent=1))
    if len(sys.argv) > 1 and sys.argv[1] == "state":
        print(json.dumps(c.state(), indent=1))
