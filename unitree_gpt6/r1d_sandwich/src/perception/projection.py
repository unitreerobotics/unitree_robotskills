"""Pixel -> base-frame point for the skill server's /project endpoint. Geometry only: nothing is detected.

    proj = Projector(calib_dir="assets/config", scene=json.load(open("assets/config/scene.json")))
    r = proj.project("head_left", [[261, 226]], frames, robot_q, z=0.61, method="plane")

Head camera: the raw left image is rectified with assets/config/calib_head.json; depth comes from SGBM
disparity with a left-right check, or from the ray's intersection with a horizontal plane.
Wrist cameras: monocular, camera pose = arm FK + URDF camera link, plane intersection only.
Camera poses come from CameraModel (head joints + lift + waist through the URDF, calibrated camera mount).
Runs with numpy, OpenCV and the standard library only.
"""
from __future__ import annotations

import math
import threading

import cv2
import numpy as np

from perception.camera_model import CameraModel, transform

CAM_ALIASES = {"head": "head", "head_left": "head", "left": "left_wrist", "right": "right_wrist",
               "left_wrist": "left_wrist", "right_wrist": "right_wrist"}
DEFAULTS = {
    "max_range_m": 2.5,         # stereo points farther than this are ignored
    "num_disparities": 96,      # SGBM search range (multiple of 16); left band of this width has no depth
    "min_disparity": -16,       # negative allowed: an unverified rectification may have a disparity offset
    "block_size": 5,
    "lr_check_px": 1.5,         # left-right consistency tolerance
}


def sgbm_disparity(left_rgb: np.ndarray, right_rgb: np.ndarray, num_disp: int = 112, min_disp: int = -16,
                   block: int = 5, lr_tol: float = 1.5) -> np.ndarray:
    """Float disparity of the rectified left image (NaN = invalid), with a left-right consistency check."""
    gl = cv2.cvtColor(left_rgb, cv2.COLOR_RGB2GRAY)
    gr = cv2.cvtColor(right_rgb, cv2.COLOR_RGB2GRAY)
    num_disp = int(math.ceil(num_disp / 16) * 16)
    common = dict(numDisparities=num_disp, blockSize=block, P1=8 * block * block, P2=32 * block * block,
                  disp12MaxDiff=-1, uniquenessRatio=8, speckleWindowSize=80, speckleRange=2, preFilterCap=31,
                  mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    dl = cv2.StereoSGBM_create(minDisparity=min_disp, **common).compute(gl, gr).astype(np.float32) / 16.0
    # right-view disparity via the mirrored pair (no ximgproc needed)
    # mirrored pair: a right pixel u_r with disparity d sits at u_r + d in the left image
    dr = cv2.StereoSGBM_create(minDisparity=min_disp, **common).compute(
        np.ascontiguousarray(gr[:, ::-1]), np.ascontiguousarray(gl[:, ::-1])).astype(np.float32)[:, ::-1] / 16.0
    invalid_l = dl < min_disp + 0.5
    h, w = dl.shape
    uu = np.arange(w)[None, :].repeat(h, 0)
    ur = np.clip(np.round(uu - dl).astype(int), 0, w - 1)
    d_back = dr[np.arange(h)[:, None], ur]
    bad = invalid_l | (np.abs(dl - d_back) > lr_tol) | ((uu - dl) < 0)
    out = dl.copy()
    out[bad] = np.nan
    return out



def plane_points(rays_cam: np.ndarray, T_base_cam: np.ndarray, z: float) -> np.ndarray:
    """Intersect optical-frame rays with the horizontal base plane z = const (only points in front)."""
    d = rays_cam @ T_base_cam[:3, :3].T
    o = T_base_cam[:3, 3]
    with np.errstate(divide="ignore", invalid="ignore"):
        s = (z - o[2]) / d[:, 2]
    ok = np.isfinite(s) & (s > 0)
    return o + s[ok, None] * d[ok]


def _as_rgb(img):
    if img is None:
        return None
    img = np.asarray(img)
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    if img.dtype != np.uint8:
        img = np.clip(img, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(img[..., :3])


class Projector:
    def __init__(self, calib_dir: str, scene: dict):
        self.scene = dict(scene or {})
        self.cfg = dict(DEFAULTS, **(self.scene.get("perception") or {}))
        self.cams = CameraModel(str(calib_dir), self.cfg.get("urdf"))
        self._lock = threading.Lock()

    @property
    def table_z(self):
        tz = self.scene.get("table_z")
        return None if tz is None else float(tz)

    def project(self, cam: str, pixels, frames: dict, robot_q: dict, z=None, method: str = "auto",
                window: int = 7) -> dict:
        """Pixels of the image the caller SAW (``/image/<cam>.jpg``, any size) -> 3-D points in the base frame.

        Pure geometry with the calibrated cameras; nothing is detected or recognised.
          method "stereo" (head only): median disparity in a ``window`` px box around the pixel -> depth.
          method "plane": the pixel's ray meets the horizontal plane z (base frame, m; default the table top).
          method "auto": stereo for the head, falling back to the plane where the disparity is invalid;
                         the plane for the wrist cameras.
        """
        with self._lock:
            try:
                return self._project(cam, pixels, frames, robot_q, z, method, int(window))
            except Exception as e:  # noqa: BLE001 - the server must always get an answer
                return {"ok": False, "points": [], "reason": f"projection error: {type(e).__name__}: {e}"}

    def _project(self, cam, pixels, frames, robot_q, z, method, window):
        cam = CAM_ALIASES.get(str(cam), None)
        if cam is None:
            return {"ok": False, "points": [], "reason": f"unknown cam; use one of {sorted(CAM_ALIASES)}"}
        if method not in ("auto", "stereo", "plane"):
            return {"ok": False, "points": [], "reason": "method must be auto, stereo or plane"}
        if method == "stereo" and cam != "head":
            return {"ok": False, "points": [], "reason": "stereo depth exists only for the head camera"}
        uv = np.asarray(pixels, float).reshape(-1, 2)
        asked = uv.copy()
        key = "head_left" if cam == "head" else cam
        img = _as_rgb((frames or {}).get(key))
        if img is None:
            return {"ok": False, "points": [], "reason": f"no frame from {key}"}
        model = self.cams.cam(cam)
        w, h = (self.cams.head.left.size if cam == "head" else model.size)
        uv = uv * [w / img.shape[1], h / img.shape[0]]          # caller's image size -> calibrated size
        T_bc = self.cams.T_base_cam(cam, robot_q or {})
        z_plane = self.table_z if z is None else float(z)
        disp = None
        if cam == "head":
            hd = self.cams.head
            # raw stream pixel -> rectified-left pixel (the camera T_bc and the disparity live in)
            uv = cv2.undistortPoints(uv.reshape(-1, 1, 2), hd.left.K, hd.left.D, R=hd.R1, P=hd.P1).reshape(-1, 2)
            if method in ("auto", "stereo"):
                right = _as_rgb((frames or {}).get("head_right"))
                if right is not None:
                    li, ri = hd.rectify(img, right)
                    c = self.cfg
                    disp = sgbm_disparity(li, ri, c["num_disparities"], c["min_disparity"], c["block_size"],
                                          c["lr_check_px"])
            rays = np.c_[(uv - [hd.cx, hd.cy]) / hd.f, np.ones(len(uv))]
        else:
            rays = model.rays(uv)
        out = []
        for (u, v), ray in zip(uv, rays):
            p, how, depth, bad = None, None, None, None
            if disp is not None:
                hw = window // 2
                ui, vi = int(round(u)), int(round(v))
                box = disp[max(vi - hw, 0):vi + hw + 1, max(ui - hw, 0):ui + hw + 1]
                good = box[np.isfinite(box) & (box > 0.5)]
                if good.size >= max(3, box.size // 4):
                    d = float(np.median(good))
                    depth = self.cams.head.f * self.cams.head.baseline / d
                    if 0.1 < depth < self.cfg["max_range_m"]:
                        p = transform(T_bc, (ray * depth)[None])[0]
                        how = "stereo"
                        x_min = float(self.scene.get("table_x_min", 0.2))
                        if z_plane is not None and p[0] > x_min and p[2] < self.table_z - 0.02:
                            # nothing can be under the table top over the table: a stereo mismatch
                            bad = f"stereo point {p[2] - self.table_z:+.3f} m below the table (mismatch)"
                            p, how, depth = None, None, None
            if p is None and method != "stereo" and z_plane is not None:
                q = plane_points(ray[None], T_bc, z_plane)
                if len(q):
                    p, how = q[0], f"plane z={z_plane:.3f}"
                    depth = float(np.linalg.norm(p - T_bc[:3, 3]))
            out.append({"pixel": [float(x) for x in asked[len(out)]],
                        "xyz": None if p is None else [round(float(x), 4) for x in p], "method": how,
                        "range_m": None if depth is None else round(float(depth), 3),
                        "reason": None if p is not None else (bad or ("no valid stereo depth here" if method == "stereo"
                                                                      else "ray does not reach the plane")),
                        "note": bad if (p is not None and bad) else None})
        cam_pos = [round(float(x), 4) for x in T_bc[:3, 3]]
        return {"ok": any(o["xyz"] for o in out), "cam": cam, "camera_xyz": cam_pos, "points": out,
                "T_base_cam": [[round(float(v), 6) for v in row] for row in T_bc],   # optical frame: z forward, x right, y down
                "warnings": self.cams.warnings(cam), "frame": "base (chassis), metres"}
