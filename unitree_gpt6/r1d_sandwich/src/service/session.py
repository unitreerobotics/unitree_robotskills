"""What `bash r1d eef` does with the skill server: run waypoints, summarise state, and draw the camera views.

No model client, prompt, memory or autonomous loop lives here. Heavy imports (cv2, numpy, the camera model) are
loaded only by the image methods.
"""

from __future__ import annotations

import json
import time

from service.paths import CONFIG_DIR

START_HEAD_PITCH = 0.486     # rad, + = look down: the head view then covers the plate, the lettuce dish and the toaster
LIFT_TOL = 0.003             # m
HEAD_TOL = 0.05              # rad; the head settles about 0.025 rad past the commanded pitch
MOTION_KEYS = ("ok", "executed", "code", "reason", "stopped", "pos_err_mm", "rot_err_deg", "min_table_gap_m",
               "min_clearance_m", "duration_s", "opening", "holding", "lift_m", "needs_lift_m", "auto_lift_m")


def rnd(v, n=3):
    if isinstance(v, float):
        return round(v, n)
    if isinstance(v, list):
        return [rnd(x, n) for x in v]
    if isinstance(v, dict):
        return {k: rnd(x, n) for k, x in v.items()}
    return v


def motion_summary(r: dict) -> dict:
    keep = {k: r[k] for k in MOTION_KEYS if r.get(k) is not None}
    if r.get("final_tcp"):
        keep["final_tcp"] = {k: r["final_tcp"].get(k) for k in ("pos", "rpy_deg") if k in r["final_tcp"]}
    return rnd(keep)


class Session:
    """One `bash r1d eef` command: the server client plus the run directory it logs to."""

    def __init__(self, client, run_dir=None):
        self.c = client
        self.run_dir = run_dir
        self._cm = None

    def log(self, **kw):
        if self.run_dir is None:
            return
        kw["t"] = round(time.time(), 2)
        with open(self.run_dir / "eef_log.jsonl", "a") as f:
            f.write(json.dumps(kw, ensure_ascii=False, default=str) + "\n")

    # ------------------------------------------------------------------ state
    def table_z(self):
        return float((self.c.poses().get("scene") or {}).get("table_z", 0.595))

    def state_summary(self):
        st, h = self.c.state(), self.c.health()
        out = {"mode": h.get("mode"), "stopped": h.get("stopped"), "stopped_reason": h.get("stopped_reason"),
               "fixed_lift": h.get("fixed_lift"), "lift_m": st.get("lift_m"), "waist_rad": st.get("waist_rad"),
               "head_pitch": st.get("head_pitch"), "head_yaw": st.get("head_yaw"),
               "tcp": {s: {"pos": st["tcp"][s]["pos"], "rpy_deg": st["tcp"][s].get("rpy_deg")}
                       for s in ("left", "right")},
               "gripper": {s: {k: st["gripper"][s].get(k) for k in ("opening", "holding")}
                           for s in ("left", "right")},
               "camera_frame_age_s": {k: v for k, v in (h.get("cameras") or {}).items()
                                      if k in ("head_left", "head_right", "left_wrist", "right_wrist")}}
        for k in ("temps_max_c", "hot_motors"):
            if st.get(k) is not None:
                out[k] = st[k]
        return rnd(out)

    # ------------------------------------------------------------------ start pose
    def prepare(self, head_pitch=None, head_yaw=None):
        """Bring the robot to the start pose of a run: the lift at the service's fixed height and the head
        looking down at the table. Moves only what is off. The arms keep their joint angles."""
        h, st = self.c.health(), self.c.state()
        scene = self.c.poses().get("scene") or {}
        lift = h.get("fixed_lift")
        pitch = float(scene.get("head_pitch", START_HEAD_PITCH)) if head_pitch is None else float(head_pitch)
        yaw = float(scene.get("head_yaw", 0.0)) if head_yaw is None else float(head_yaw)
        out = {"ok": False, "steps": []}
        if lift is None:
            out["reason"] = "the service was started without --fixed-lift; restart it with --fixed-lift 0.522"
            return out
        if abs(st["lift_m"] - lift) > LIFT_TOL:
            r = motion_summary(self.c.lift(lift))
            out["steps"].append({"lift": lift, **r})
            if not r.get("ok"):
                out["reason"] = f"lift to {lift} refused: {r.get('reason')}"
                return out
        if abs(st["head_pitch"] - pitch) > HEAD_TOL or abs(st["head_yaw"] - yaw) > HEAD_TOL:
            r = motion_summary(self.c.head(pitch, yaw))
            out["steps"].append({"head_pitch": pitch, "head_yaw": yaw, **r})
            if not r.get("ok"):
                out["reason"] = f"head move refused: {r.get('reason')}"
                return out
        st = self.c.state()
        out.update(lift_m=st["lift_m"], head_pitch=st["head_pitch"], head_yaw=st["head_yaw"])
        executed = h.get("mode") != "shadow"            # shadow checks the moves and leaves the robot where it is
        out["ok"] = not executed or (abs(st["lift_m"] - lift) <= LIFT_TOL and abs(st["head_pitch"] - pitch) <= HEAD_TOL
                                     and abs(st["head_yaw"] - yaw) <= HEAD_TOL)
        if not out["ok"]:
            out["reason"] = f"not at the start pose: wanted lift {lift}, head pitch {pitch}, head yaw {yaw}"
        return rnd(out)

    # ------------------------------------------------------------------ waypoints
    def execute_eef(self, inp):
        """Send the waypoints to the IK in order. Stop at the first refusal, or before a waypoint whose
        `min_opening` is not met by that hand's gripper (the object was not grasped, or it was dropped)."""
        c, out = self.c, []
        for si, seg in enumerate(inp["segments"]):
            side, sp = seg["side"], float(seg.get("speed", 0.15))
            opening = None                      # reported by the last grasp in this segment
            for wi, w in enumerate(seg["waypoints"]):
                at = {"segment": si, "waypoint": wi}
                need = w.get("min_opening")
                if need is not None:
                    if opening is None:
                        opening = c.state()["gripper"][side].get("opening")
                    if opening is None or opening < need:
                        reason = f"{side} gripper opening {rnd(opening)} is below min_opening {need}: nothing is held"
                        out.append({"seg": si, "wp": wi, "side": side, "ok": False, "executed": False,
                                    "code": "not_held", "reason": reason})
                        return {"ok": False, "stopped_at": {**at, "not_held": True}, "results": out}
                r = c.move(side, w["pos"], rpy_deg=w["rpy_deg"], linear=bool(w.get("linear", True)), speed=sp,
                           auto_lift=False)
                out.append({"seg": si, "wp": wi, "side": side, **motion_summary(r)})
                if not r.get("ok"):
                    return {"ok": False, "stopped_at": at, "results": out}
                g = w.get("gripper")
                if g is not None:
                    gr = c.gripper(side, action="grasp") if g == "grasp" else c.gripper(side, float(g))
                    out.append({"seg": si, "wp": wi, "side": side, "gripper": g, **motion_summary(gr)})
                    if not gr.get("ok"):
                        return {"ok": False, "stopped_at": {**at, "gripper": True}, "results": out}
                    opening = gr.get("opening") if g == "grasp" else None
        return {"ok": True, "results": out}

    def check_eef(self, inp):
        """IK and safety check of every waypoint from the current pose. Nothing moves."""
        res = []
        for si, seg in enumerate(inp["segments"]):
            for wi, w in enumerate(seg["waypoints"]):
                r = self.c.move(seg["side"], w["pos"], rpy_deg=w["rpy_deg"], linear=bool(w.get("linear", True)),
                                check_only=True, auto_lift=False)
                res.append({"seg": si, "wp": wi, "ok": r.get("ok"), "code": r.get("code"),
                            "reason": None if r.get("ok") else str(r.get("reason"))[:140]})
        return {"checked_from_current_pose": res}

    # ------------------------------------------------------------------ images
    def frame(self, cam_name):
        import cv2
        import numpy as np

        return cv2.imdecode(np.frombuffer(self.c.image(cam_name), np.uint8), cv2.IMREAD_COLOR)

    def keyframe(self, tag):
        import cv2
        import numpy as np

        ims = [cv2.resize(self.frame(cam), size) for cam, size in
               (("head_left", (544, 448)), ("left_wrist", (299, 224)), ("right_wrist", (299, 224)))]
        tile = np.hstack([ims[0], np.vstack(ims[1:])])
        cv2.putText(tile, tag, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        fn = self.run_dir / "keyframes" / f"{time.strftime('%H%M%S')}_{tag}.jpg"
        cv2.imwrite(str(fn), tile)
        return {"ok": True, "file": fn.name}

    def pixel_crop(self, cam_name, crop, scale=3):
        """The raw image region crop = (u0, v0, u1, v1), enlarged, with a tick every 10 px labelled in the pixels
        of the full image. Read pixel coordinates from it and hand them to `project`."""
        import cv2

        im = self.frame(cam_name)
        h, w = im.shape[:2]
        u0, v0, u1, v1 = (int(v) for v in crop)
        u0, v0, u1, v1 = max(0, u0), max(0, v0), min(w, u1), min(h, v1)
        im = cv2.resize(im[v0:v1, u0:u1], None, fx=scale, fy=scale, interpolation=cv2.INTER_LANCZOS4)
        red, font = (0, 0, 255), cv2.FONT_HERSHEY_SIMPLEX
        for u in range(-(-u0 // 10) * 10, u1, 10):
            x = (u - u0) * scale
            cv2.line(im, (x, 0), (x, 8), red, 1)
            cv2.putText(im, str(u), (x + 1, 20), font, 0.35, red, 1)
        for v in range(-(-v0 // 10) * 10, v1, 10):
            y = (v - v0) * scale
            cv2.line(im, (0, y), (8, y), red, 1)
            cv2.putText(im, str(v), (10, y + 4), font, 0.35, red, 1)
        return im, f"raw crop {u0},{v0},{u1},{v1} x{scale}, ticks every 10 px in full-image pixels"

    def ruler_image(self, cam_name, z, marks=(), scale=2, crop=None):
        """Camera image (upscaled) with a thin base-frame ruler on the plane z: lines every 10 cm (x red, y blue),
        dots every 5 cm, labels on every line, the TCPs, and optional marks = base-frame points [x, y, z] drawn as
        numbered crosses (to check an estimated position against the picture). Calibrated projection only."""
        import cv2
        import numpy as np

        from perception.camera_model import CameraModel

        if self._cm is None:
            self._cm = CameraModel(str(CONFIG_DIR))
        st = self.c.state()
        q = {"head_pitch": st["head_pitch"], "head_yaw": st["head_yaw"], "lift": st["lift_m"],
             "waist": st["waist_rad"], "left_arm": st["arm_q"]["left"], "right_arm": st["arm_q"]["right"]}
        im = self.frame(cam_name)
        key = "head" if cam_name.startswith("head") else cam_name
        cam = self._cm.cam(key)
        T_cb = np.linalg.inv(self._cm.T_base_cam(key, q))
        h, w = im.shape[:2]
        im = cv2.resize(im, (w * scale, h * scale), interpolation=cv2.INTER_CUBIC)
        W, H = w * scale, h * scale

        def proj(P):
            P = np.asarray(P, float).reshape(-1, 3)
            pc = (T_cb[:3, :3] @ P.T).T + T_cb[:3, 3]
            uv = np.full((len(P), 2), np.nan)
            ok = pc[:, 2] > 0.05
            if ok.any():
                uv[ok] = cam.project(pc[ok]) * scale
            return uv

        def visible(P):
            """Integer pixel of one base-frame point, or None when it is outside the image."""
            a_, b_ = proj(P)[0]
            return (int(a_), int(b_)) if np.isfinite(a_) and 0 <= a_ < W and 0 <= b_ < H else None

        def text(t, org, col, fs=0.5):
            cv2.putText(im, t, org, cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(im, t, org, cv2.FONT_HERSHEY_SIMPLEX, fs, col, 1, cv2.LINE_AA)

        xs, ys = np.arange(0.2, 0.801, 0.1), np.arange(-0.5, 0.501, 0.1)
        for axis, vals, lo, hi, col in (("x", xs, -0.5, 0.5, (40, 40, 220)), ("y", ys, 0.2, 0.8, (220, 110, 30))):
            for v in vals:
                t = np.linspace(lo, hi, 80)
                fixed, plane = np.full_like(t, v), np.full_like(t, z)
                P = np.c_[fixed, t, plane] if axis == "x" else np.c_[t, fixed, plane]
                pts = [(int(a_), int(b_)) for a_, b_ in proj(P)
                       if np.isfinite(a_) and -W < a_ < 2 * W and -H < b_ < 2 * H]
                for a_, b_ in zip(pts[:-1], pts[1:]):
                    cv2.line(im, a_, b_, col, 1, cv2.LINE_AA)
                inside = [p_ for p_ in pts if 8 <= p_[0] < W - 60 and 14 <= p_[1] < H - 4]
                if inside:
                    p0 = inside[0] if axis == "y" else inside[len(inside) // 5]
                    text(f"{axis}{v:+.1f}" if axis == "y" else f"x{v:.1f}", (p0[0] + 3, p0[1] - 3), col)
        dots = np.array([[x_, y_, z] for x_ in np.arange(0.2, 0.801, 0.05) for y_ in np.arange(-0.5, 0.501, 0.05)])
        for a_, b_ in proj(dots):
            if np.isfinite(a_) and 0 <= a_ < W and 0 <= b_ < H:
                cv2.circle(im, (int(a_), int(b_)), 2, (0, 0, 0), -1)
        for side, col in (("left", (0, 170, 0)), ("right", (0, 150, 255))):
            p = st["tcp"][side]["pos"]
            px = visible(p)
            if px:
                cv2.circle(im, px, 7, col, 2)
                text(f"{side[0].upper()}tcp({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})", (px[0] + 10, px[1] + 4), col, 0.45)
        for i, m in enumerate(marks):
            px = visible(m)
            if px:
                cv2.drawMarker(im, px, (255, 0, 255), cv2.MARKER_CROSS, 18, 2)
                text(f"#{i}({m[0]:.3f},{m[1]:.3f},{m[2]:.3f})", (px[0] + 10, px[1] - 8), (200, 0, 200), 0.45)
        if crop:
            u0, v0, u1, v1 = [int(c_ * scale) for c_ in crop]
            im = im[max(0, v0):v1, max(0, u0):u1]
        calibrated = getattr(cam, "calibrated", None)
        return im, (f"ruler on z={z:.3f}, lines every 10 cm, dots every 5 cm, scale x{scale} "
                    f"(calibrated={calibrated})")
