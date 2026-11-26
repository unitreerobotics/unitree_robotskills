"""Camera models of the R1-D: head rectified stereo + two wrist cameras, intrinsics and base-frame poses.

Only numpy + OpenCV (+ stdlib xml): the kinematic chain is read straight from the URDF, so this runs in
any env. Conventions:
  * base frame = URDF base_link (x fwd, y left, z up, metres) -- the same frame the skill server uses;
  * camera frames are OpenCV optical frames (x right, y down, z forward);
  * T_a_b = 4x4 pose of frame b expressed in frame a (p_a = T_a_b @ p_b).

Calibration files (assets/config/), read here:
  calib_head.json        rectified head pair: K/D of both eyes, R/T right-wrt-left, and the pose of the
                         LEFT eye in head_yaw_Link (T_parent_cam) + optional head joint zero offsets
  calib_left_wrist.json  K/D + pose in left_dex1_camera_link     (same for calib_right_wrist.json)
Missing file / missing part -> NOMINAL values (best guess from URDF geometry + EEPROM), flagged
calibrated=False; every detection made with them carries a warning.

robot_q (from the server): {"head_pitch", "head_yaw", "lift" (merged lift joint = total rise, m),
"waist", "left_arm": [7], "right_arm": [7]} in URDF joint convention (rad / m).
"""
from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from service.paths import R1D_URDF

URDF_CANDIDATES = (str(R1D_URDF),) if R1D_URDF else ()
ARM_JOINTS7 = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
HEAD_PARENT = "head_yaw_Link"
CAMS = ("head", "left_wrist", "right_wrist")

# ---------------------------------------------------------------- NOMINAL values (NOT calibrated)
# Rectified head stream 544x448. f = 334.195 px is the EEPROM "intin_info tx" field (paired with
# ty = 60.6 = baseline mm, i.e. the camera's own depth parameters for its 544x448 rectified output).
# The right-eye principal point offsets were measured on one sample pair (2026-11-14):
# right rows are ~3.3 px higher; the cx offset assumes the far desk in that frame was ~3 m away. Both are
# rough -- calibrate the head camera and write assets/config/calib_head.json.
NOMINAL_HEAD_SIZE = (544, 448)
NOMINAL_HEAD_F = 334.195
NOMINAL_RIGHT_DCX = 7.0
NOMINAL_RIGHT_DCY = -3.7   # 2026-11-17: SIFT on a live pair, left v - right v = +3.69 px (std 1.3), eyes not swapped
NOMINAL_BASELINE = 0.0606
# Left eye in head_yaw_Link: front face of the head mesh is at x ~ 0.085-0.09, face centre z ~ -0.02,
# eyes +-B/2 in y. Optical axis = head x, no extra tilt.
NOMINAL_HEAD_EYE_XYZ = (0.085, NOMINAL_BASELINE / 2, -0.02)
R_HEAD_OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])  # columns: optical x,y,z in head
# Wrist cams (640x480): URDF *_dex1_camera_link x = viewing direction (30 deg below the fingers),
# link z = image right, link -y = image down. Focal length is a guess (~75 deg HFOV).
NOMINAL_WRIST_SIZE = (640, 480)
NOMINAL_WRIST_F = 420.0
R_WRISTLINK_OPT = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])


# ---------------------------------------------------------------- small SE3 helpers
def rpy_to_R(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def make_T(R=None, t=None) -> np.ndarray:
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = np.asarray(t, float).ravel()
    return T


def axis_angle_R(axis, angle: float) -> np.ndarray:
    return cv2.Rodrigues(np.asarray(axis, float).reshape(3) * angle)[0]


def transform(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, float)
    return pts @ T[:3, :3].T + T[:3, 3]


# ---------------------------------------------------------------- URDF chain
class UrdfChain:
    """Forward kinematics of a URDF tree (revolute/prismatic/fixed/continuous) from joint values by name."""

    def __init__(self, path: str | None = None):
        path = path or next((p for p in URDF_CANDIDATES if p and Path(p).is_file()), None)
        if path is None:
            raise FileNotFoundError(f"no R1-D URDF found (tried {URDF_CANDIDATES}); set R1D_URDF")
        self.path = str(path)
        self.joints = {}  # child link -> (joint name, type, parent link, T_origin, axis)
        for j in ET.parse(path).getroot().findall("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            a = j.find("axis")
            axis = np.array([float(v) for v in a.get("xyz").split()]) if a is not None else np.zeros(3)
            self.joints[j.find("child").get("link")] = (j.get("name"), j.get("type"), j.find("parent").get("link"),
                                                        make_T(rpy_to_R(*rpy), xyz), axis)

    def T_root_link(self, link: str, q: dict) -> np.ndarray:
        T = np.eye(4)
        while link in self.joints:
            name, typ, parent, T_o, axis = self.joints[link]
            v = float(q.get(name, 0.0))
            if typ in ("revolute", "continuous") and v:
                T_j = make_T(axis_angle_R(axis, v))
            elif typ == "prismatic" and v:
                T_j = make_T(t=axis * v)
            else:
                T_j = np.eye(4)
            T = T_o @ T_j @ T
            link = parent
        return T


def joint_dict(robot_q: dict, offsets: dict | None = None) -> dict:
    """robot_q (server format) -> URDF joint name -> value. The merged lift is split over both stages."""
    q = {}
    lift = float(robot_q.get("lift", 0.0) or 0.0)
    q["lift_stage1_joint"] = q["lift_stage2_joint"] = lift / 2.0
    q["waist_yaw_joint"] = float(robot_q.get("waist", 0.0) or 0.0)
    q["head_pitch_joint"] = float(robot_q.get("head_pitch", 0.0) or 0.0)
    q["head_yaw_joint"] = float(robot_q.get("head_yaw", 0.0) or 0.0)
    for side in ("left", "right"):
        arm = robot_q.get(f"{side}_arm")
        if arm is not None:
            for n, v in zip(ARM_JOINTS7, arm):
                q[f"{side}_{n}_joint"] = float(v)
    for k, v in (offsets or {}).items():
        q[k] = q.get(k, 0.0) + float(v)
    return q


# ---------------------------------------------------------------- cameras
@dataclass
class MonoCam:
    name: str
    size: tuple                 # (w, h)
    K: np.ndarray
    D: np.ndarray
    parent_link: str
    T_parent_cam: np.ndarray
    intrinsics_calibrated: bool = False
    extrinsics_calibrated: bool = False
    joint_offsets: dict = field(default_factory=dict)
    source: str = "nominal"

    @property
    def calibrated(self) -> bool:
        return self.intrinsics_calibrated and self.extrinsics_calibrated

    def rays(self, uv: np.ndarray) -> np.ndarray:
        """Pixel coords (N,2) -> unit-z rays (N,3) in the optical frame (undistorted)."""
        uv = np.asarray(uv, np.float64).reshape(-1, 1, 2)
        if np.any(self.D):
            xy = cv2.undistortPoints(uv, self.K, self.D).reshape(-1, 2)
        else:
            xy = (uv.reshape(-1, 2) - self.K[:2, 2]) / np.diag(self.K)[:2]
        return np.c_[xy, np.ones(len(xy))]

    def project(self, pts_cam: np.ndarray) -> np.ndarray:
        pts = np.asarray(pts_cam, np.float64).reshape(-1, 3)
        uv, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), self.K, self.D)
        return uv.reshape(-1, 2)


@dataclass
class StereoHead:
    left: MonoCam               # raw left eye (as streamed); its pose is the head extrinsic
    K_r: np.ndarray
    D_r: np.ndarray
    R: np.ndarray               # x_right = R x_left + T
    T: np.ndarray

    def __post_init__(self):
        w, h = self.left.size
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(self.left.K, self.left.D, self.K_r, self.D_r, (w, h),
                                                     self.R, self.T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        self.R1, self.P1, self.P2 = R1, P1, P2
        self.map_l = cv2.initUndistortRectifyMap(self.left.K, self.left.D, R1, P1, (w, h), cv2.CV_32FC1)
        self.map_r = cv2.initUndistortRectifyMap(self.K_r, self.D_r, R2, P2, (w, h), cv2.CV_32FC1)
        self.f = float(P1[0, 0])
        self.cx, self.cy = float(P1[0, 2]), float(P1[1, 2])
        self.baseline = float(-P2[0, 3] / P2[0, 0])
        # the rectified left camera: pinhole, no distortion, rotated by R1 w.r.t. the raw left eye
        self.rect = MonoCam("head", self.left.size, P1[:3, :3].copy(), np.zeros(5), self.left.parent_link,
                            self.left.T_parent_cam @ make_T(R1.T), self.left.intrinsics_calibrated,
                            self.left.extrinsics_calibrated, self.left.joint_offsets, self.left.source)

    def rectify(self, left: np.ndarray, right: np.ndarray | None):
        """Raw stream images -> rectified (left, right). Sizes other than calibrated are resized first."""
        w, h = self.left.size
        out = []
        for img, m in ((left, self.map_l), (right, self.map_r)):
            if img is None:
                out.append(None)
                continue
            if img.shape[1] != w or img.shape[0] != h:
                img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
            out.append(cv2.remap(img, m[0], m[1], cv2.INTER_LINEAR))
        return out[0], out[1]


def _nominal_head() -> dict:
    w, h = NOMINAL_HEAD_SIZE
    f = NOMINAL_HEAD_F
    K = [[f, 0, (w - 1) / 2], [0, f, (h - 1) / 2], [0, 0, 1]]
    Kr = [[f, 0, (w - 1) / 2 + NOMINAL_RIGHT_DCX], [0, f, (h - 1) / 2 + NOMINAL_RIGHT_DCY], [0, 0, 1]]
    return {"image_size": [w, h], "intrinsics_calibrated": False, "extrinsics_calibrated": False,
            "left": {"K": K, "D": [0.0] * 5}, "right": {"K": Kr, "D": [0.0] * 5},
            "R": np.eye(3).tolist(), "T": [-NOMINAL_BASELINE, 0.0, 0.0],
            "parent_link": HEAD_PARENT, "T_parent_cam": make_T(R_HEAD_OPT, NOMINAL_HEAD_EYE_XYZ).tolist(),
            "joint_offsets": {}, "source": "nominal"}


def _nominal_wrist(side: str) -> dict:
    w, h = NOMINAL_WRIST_SIZE
    f = NOMINAL_WRIST_F
    return {"image_size": [w, h], "intrinsics_calibrated": False, "extrinsics_calibrated": False,
            "K": [[f, 0, (w - 1) / 2], [0, f, (h - 1) / 2], [0, 0, 1]], "D": [0.0] * 5,
            "parent_link": f"{side}_dex1_camera_link", "T_parent_cam": make_T(R_WRISTLINK_OPT).tolist(),
            "joint_offsets": {}, "source": "nominal"}


def calib_path(calib_dir: str, cam: str) -> Path:
    return Path(calib_dir) / f"calib_{cam}.json"


def load_calib(calib_dir: str, cam: str) -> dict:
    """Nominal values overlaid with whatever the calibration file holds."""
    base = _nominal_head() if cam == "head" else _nominal_wrist(cam.split("_")[0])
    p = calib_path(calib_dir, cam)
    if p.is_file():
        data = json.loads(p.read_text())
        base.update({k: v for k, v in data.items() if v is not None})
        base["source"] = str(p)
    return base


def _mono_from(cfg: dict, name: str, K_key="K", D_key="D") -> MonoCam:
    K = np.array(cfg[K_key] if K_key in cfg else cfg["left"]["K"], float)
    D = np.array(cfg[D_key] if D_key in cfg else cfg["left"]["D"], float).ravel()
    return MonoCam(name, tuple(cfg["image_size"]), K, D, cfg["parent_link"], np.array(cfg["T_parent_cam"], float),
                   bool(cfg.get("intrinsics_calibrated")), bool(cfg.get("extrinsics_calibrated")),
                   dict(cfg.get("joint_offsets") or {}), cfg.get("source", "nominal"))


class CameraModel:
    """All cameras of the robot. T_base_cam(cam, robot_q) returns the optical-frame pose in the base frame;
    for "head" that is the RECTIFIED left camera (the frame Projector works in)."""

    def __init__(self, calib_dir: str, urdf_path: str | None = None):
        self.calib_dir = str(calib_dir)
        self.urdf = UrdfChain(urdf_path)
        hc = load_calib(calib_dir, "head")
        left = _mono_from(hc, "head_left")
        self.head = StereoHead(left, np.array(hc["right"]["K"], float), np.array(hc["right"]["D"], float).ravel(),
                               np.array(hc["R"], float), np.array(hc["T"], float).reshape(3, 1))
        self.wrist = {s: _mono_from(load_calib(calib_dir, f"{s}_wrist"), f"{s}_wrist") for s in ("left", "right")}

    def cam(self, cam: str) -> MonoCam:
        return self.head.rect if cam == "head" else self.wrist[cam.split("_")[0]]

    def T_base_cam(self, cam: str, robot_q: dict) -> np.ndarray:
        c = self.cam(cam)
        return self.urdf.T_root_link(c.parent_link, joint_dict(robot_q, c.joint_offsets)) @ c.T_parent_cam

    def warnings(self, cam: str) -> list[str]:
        c = self.cam(cam)
        w = []
        if not c.intrinsics_calibrated:
            w.append(f"{cam}: intrinsics NOT calibrated (nominal{', stereo rectification unverified' if cam == 'head' else ''})")
        if not c.extrinsics_calibrated:
            w.append(f"{cam}: camera pose NOT calibrated (nominal from URDF geometry) -- expect several cm error")
        return w
