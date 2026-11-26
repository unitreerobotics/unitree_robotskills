"""Toy tabletop world for --sim: the sandwich scene, finger blocking, attach on grasp, drop on release.

Not physics. It exists so agent frameworks can exercise the whole task logic (grasp
success / miss, carrying, laying a slice flat, stacking) without the robot. Rules:
  * a closing gripper is blocked by an object whose grasp region contains the TCP (and, for a standing
    slice, whose plane is roughly perpendicular to the closing axis);
  * blocked + commanded closed => the object is attached (rigidly, relative to the TCP);
  * opening (> 0.5) releases it: it lands on the highest support below its centre. A slice lands flat
    only if its plane is within ~35° of horizontal at release, otherwise it is "toppled" (failure).
"""
from __future__ import annotations

import copy
import math
import threading

import numpy as np

GRIP_WIDTH_M = 0.09   # Dex1 inner gap when fully open (approx.)


def _rot(rpy):
    r, p, y = (math.radians(v) for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _quat_to_R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class SimWorld:
    """Objects: name -> dict(kind, pos (centre), R (3x3), size (along local x, y, z), color, state)."""

    def __init__(self, scene: dict):
        self.scene = scene
        self.lock = threading.Lock()
        self.reset()

    def reset(self, randomize=False, seed=None):
        sc = copy.deepcopy(self.scene)
        if randomize:
            rng = np.random.default_rng(seed)
            for k in ("toaster", "plate", "pink_plate"):
                sc[k]["x"] += float(rng.uniform(-0.03, 0.03))
                sc[k]["y"] += float(rng.uniform(-0.03, 0.03))
            sc["toaster"]["slot_yaw_deg"] = float(sc["toaster"].get("slot_yaw_deg", 0.0) + rng.uniform(-10, 10))
        tz = float(sc["table_z"])
        objs = {}
        t = sc["toaster"]
        tyaw = float(t.get("slot_yaw_deg", 0.0))
        objs["toaster"] = dict(kind="toaster", pos=np.array([t["x"], t["y"], tz + t["h"] / 2]), R=_rot([0, 0, tyaw]),
                               size=np.array([t["l"], t["w"], t["h"]]), color=(235, 235, 225), state="fixed")
        for k, off in enumerate(t["slot_offsets"]):
            # local frame of a slice: x = slot axis (width), y = slot normal (thickness), z = up (height)
            c = np.array([t["x"], t["y"], 0.0]) + _rot([0, 0, tyaw]) @ np.array([0.0, off, 0.0])
            c[2] = tz + t["h"] + t["slice_protrude"] - sc["slice"]["h"] / 2
            objs[f"bread{k + 1}"] = dict(kind="bread", pos=c, R=_rot([0, 0, tyaw]),
                                        size=np.array([sc["slice"]["w"], sc["slice"]["t"], sc["slice"]["h"]]),
                                        color=(222, 184, 135), state="in_toaster")
        for name in ("plate", "pink_plate"):
            p = sc[name]
            objs[name] = dict(kind="plate", pos=np.array([p["x"], p["y"], tz + 0.0075]), R=np.eye(3),
                              size=np.array([2 * p["r"], 2 * p["r"], 0.015]),
                              color=(245, 245, 245) if name == "plate" else (240, 150, 170), state="fixed")
        lp = sc["pink_plate"]
        objs["lettuce"] = dict(kind="lettuce", pos=np.array([lp["x"], lp["y"], tz + 0.015 + 0.015]), R=np.eye(3),
                               size=np.array([0.09, 0.07, 0.03]), color=(90, 180, 70), state="free")
        with self.lock:
            self.objs = objs
            self.attached = {"left": None, "right": None}
            self.tcp = {}
            self.grip = {}
            self.events = []

    # ------------------------------------------------------------------ geometry
    def top(self, o):
        """Highest z of an object's box."""
        h = 0.5 * np.abs(o["R"]) @ o["size"]
        return float(o["pos"][2] + h[2])

    def _support_z(self, xy, exclude):
        z = float(self.scene["table_z"])
        for n, o in self.objs.items():
            if n == exclude or o["kind"] == "toaster" or o["state"] == "attached":
                continue
            half = 0.5 * np.abs(o["R"]) @ o["size"]
            if abs(xy[0] - o["pos"][0]) <= half[0] and abs(xy[1] - o["pos"][1]) <= half[1]:
                z = max(z, self.top(o))
        return z

    def _tool(self, side):
        t = self.tcp.get(side)
        if t is None:
            return None, None
        return np.asarray(t["pos"], float), _quat_to_R(*t["quat_wxyz"])

    def _grasp_candidate(self, side):
        p, R = self._tool(side)
        if p is None:
            return None
        closing = R[:, 1]
        for n, o in self.objs.items():
            if o["state"] in ("attached", "fixed", "toppled"):
                continue
            d = o["R"].T @ (p - o["pos"])
            if o["kind"] == "bread" and o["state"] in ("in_toaster", "standing"):
                normal = o["R"][:, 1]
                if (abs(d[1]) < 0.02 and abs(d[0]) < o["size"][0] / 2 and
                        o["size"][2] / 2 - 0.05 < d[2] < o["size"][2] / 2 + 0.005 and abs(closing @ normal) > 0.7):
                    return n
            elif o["kind"] == "lettuce":
                top = self.top(o)
                if np.hypot(*(p[:2] - o["pos"][:2])) < 0.05 and top - 0.03 < p[2] < top + 0.012:
                    return n
            elif o["kind"] == "bread" and o["state"] == "flat":
                pass   # a flat slice cannot be pinched with a parallel gripper from above
        return None

    # ------------------------------------------------------------------ hooks from the robot
    def blocker(self, side, q):
        """Called every control tick with the gripper's next position (rad, 0 = closed)."""
        with self.lock:
            name = self.attached.get(side)
            if name is None:
                name = self._grasp_candidate(side)
            if name is None:
                return q
            o = self.objs[name]
            width = o["size"][1] if o["kind"] == "bread" else 0.05
            q_block = 5.4 * min(1.0, width / GRIP_WIDTH_M)
            return max(q, q_block)

    def sync(self, tcp: dict, grip: dict):
        """Update from a fresh robot state: attach / carry / release."""
        with self.lock:
            prev = dict(self.tcp)
            self.tcp, self.grip = tcp, grip
            for side in ("left", "right"):
                g = grip.get(side, {})
                cmd = g.get("cmd_opening")
                name = self.attached[side]
                p, R = self._tool(side)
                closing = cmd is not None and (cmd < 0.05 or g.get("grasping"))
                if name is None and closing and g.get("opening", 0) > 0.04:
                    cand = self._grasp_candidate(side)
                    if cand:
                        o = self.objs[cand]
                        o["rel_R"] = R.T @ o["R"]
                        o["rel_p"] = R.T @ (o["pos"] - p)
                        o["state"] = "attached"
                        self.attached[side] = cand
                        self.events.append(f"{side} grasped {cand}")
                elif name is not None:
                    o = self.objs[name]
                    o["R"] = R @ o["rel_R"]
                    o["pos"] = p + R @ o["rel_p"]
                    if cmd is not None and cmd > 0.5 and not g.get("grasping"):
                        self._release(side, name)

    def _release(self, side, name):
        o = self.objs[name]
        self.attached[side] = None
        if o["kind"] == "bread":
            normal = o["R"][:, 1]
            if abs(normal[2]) > math.cos(math.radians(35)):
                yaw = math.degrees(math.atan2(o["R"][1, 0], o["R"][0, 0]))
                o["R"] = _rot([90, 0, yaw]) if normal[2] > 0 else _rot([-90, 0, yaw])
                o["state"] = "flat"
            else:
                o["state"] = "toppled"
        else:
            o["state"] = "free"
        z = self._support_z(o["pos"][:2], name)
        half = 0.5 * np.abs(o["R"]) @ o["size"]
        drop = o["pos"][2] - half[2] - z
        o["pos"][2] = z + half[2]
        self.events.append(f"{side} released {name} -> {o['state']} (fell {drop * 100:.1f} cm)")

    def snapshot(self):
        with self.lock:
            objs = {n: {"kind": o["kind"], "state": o["state"], "pos": [round(float(v), 4) for v in o["pos"]],
                        "top_z": round(self.top(o), 4)} for n, o in self.objs.items()}
            return {"objects": objs, "attached": dict(self.attached), "events": list(self.events),
                    "success": self.success()}

    def success(self):
        plate = self.objs["plate"]
        stack = [n for n, o in self.objs.items() if o["kind"] in ("bread", "lettuce") and o["state"] in ("flat", "free")
                 and np.hypot(*(o["pos"][:2] - plate["pos"][:2])) < 0.07]
        order = [self.objs[n]["kind"] for n in sorted(stack, key=lambda n: self.objs[n]["pos"][2])]
        return {"done": order == ["bread", "lettuce", "bread"], "stack_on_plate": order}

    def render(self, w=544, h=448):
        """Top-down schematic (sim stand-in for the head camera)."""
        import cv2
        img = np.full((h, w, 3), 200, np.uint8)
        x0, x1, y0, y1 = 0.15, 0.75, 0.5, -0.5

        def px(x, y):
            return int((y - y0) / (y1 - y0) * w), int((x1 - x) / (x1 - x0) * h)

        with self.lock:
            for n, o in sorted(self.objs.items(), key=lambda kv: self.top(kv[1])):
                half = 0.5 * np.abs(o["R"]) @ o["size"]
                a, b = px(o["pos"][0] + half[0], o["pos"][1] + half[1]), px(o["pos"][0] - half[0], o["pos"][1] - half[1])
                col = tuple(int(c) for c in o["color"])
                if o["kind"] == "plate":
                    cv2.circle(img, px(*o["pos"][:2]), int(half[0] / (x1 - x0) * h), col, -1)
                else:
                    cv2.rectangle(img, a, b, col, -1)
                cv2.putText(img, n, (min(a[0], b[0]), max(a[1], b[1]) + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
            for side, t in self.tcp.items():
                cv2.drawMarker(img, px(*t["pos"][:2]), (0, 0, 255) if side == "right" else (255, 0, 0),
                               cv2.MARKER_CROSS, 14, 2)
        cv2.putText(img, "SIM top view (x up, y left)", (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        return img
