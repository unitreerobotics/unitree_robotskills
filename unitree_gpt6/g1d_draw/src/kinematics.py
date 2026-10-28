"""Optional fixed-base G1 kinematics; no devices, private drivers, or model calls.

Poses use meters and XYZ Euler radians in the configured URDF root frame.
Each configured task frame is ``URDF frame @ tool_transform``. A task frame
is not implicitly a brush tip. Meshes are not needed or loaded for FK/RNEA;
``package_dirs`` is retained as deployment metadata for geometry consumers.
"""

from __future__ import annotations

import copy
import math
import threading
from pathlib import Path

import numpy as np

ARM_JOINT_NAMES = tuple(
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in (
        "shoulder_pitch",
        "shoulder_roll",
        "shoulder_yaw",
        "elbow",
        "wrist_roll",
        "wrist_pitch",
        "wrist_yaw",
    )
)


def _vector(value, length, label):
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(f"{label} requires {length} finite values")
    return result.copy()


def _transform(value, label):
    result = np.asarray(value, dtype=float)
    if (
        result.shape != (4, 4)
        or not np.isfinite(result).all()
        or not np.allclose(result[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
        or not np.allclose(result[:3, :3].T @ result[:3, :3], np.eye(3), atol=1e-7)
        or not np.isclose(np.linalg.det(result[:3, :3]), 1.0, atol=1e-7)
    ):
        raise ValueError(f"{label} requires a rigid 4x4 transform")
    return result.copy()


def _pose_transform(pose):
    x, y, z, roll, pitch, yaw = _vector(
        [pose[k] for k in ("x", "y", "z", "roll", "pitch", "yaw")], 6, "pose"
    )
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    result = np.eye(4)
    result[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    result[:3, 3] = [x, y, z]
    return result


class PinocchioKinematics:
    """Explicit non-arm configuration, 14 ordered arm joints, bounded arm IK."""

    def __init__(self, config: dict):
        try:
            import pinocchio as pin
        except ImportError as exc:
            raise ImportError("PinocchioKinematics requires the optional 'pin' dependency") from exc
        self._pin = pin
        settings = config["model"]
        urdf = Path(settings["urdf"]).expanduser()
        if not urdf.is_absolute() or not urdf.is_file():
            raise ValueError("model.urdf must be an existing absolute URDF path")
        packages = settings.get("package_dirs", [])
        if not isinstance(packages, list) or not all(isinstance(p, str) for p in packages):
            raise ValueError("model.package_dirs must be a list of paths")
        self.package_dirs = tuple(packages)
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.data = self.model.createData()
        self._lock = threading.RLock()
        names = set(self.model.names[1:])
        if not set(ARM_JOINT_NAMES) <= names:
            raise ValueError("URDF is missing required G1 arm joints")
        references = settings.get("reference_positions")
        non_arm = names - set(ARM_JOINT_NAMES)
        if not isinstance(references, dict) or set(references) != non_arm:
            raise ValueError("reference_positions must specify exactly every non-arm movable joint")
        self._base_q = np.zeros(self.model.nq)
        for joint_id in range(1, self.model.njoints):
            joint, name = self.model.joints[joint_id], self.model.names[joint_id]
            if joint.nq != 1 or joint.nv != 1:
                raise ValueError(f"Only fixed-base scalar joints are supported: {name}")
            if name in references:
                value = references[name]
                if (
                    type(value) not in (int, float)
                    or not math.isfinite(value)
                    or not self.model.lowerPositionLimit[joint.idx_q]
                    <= value
                    <= self.model.upperPositionLimit[joint.idx_q]
                ):
                    raise ValueError(f"Invalid reference position: {name}")
                if name == "waist_roll_joint" and value != 0.0:
                    raise ValueError("G1-D has no waist roll; model waist_roll_joint must be zero")
                self._base_q[joint.idx_q] = value
        self._q_indices = np.array(
            [self.model.joints[self.model.getJointId(n)].idx_q for n in ARM_JOINT_NAMES]
        )
        self._v_indices = np.array(
            [self.model.joints[self.model.getJointId(n)].idx_v for n in ARM_JOINT_NAMES]
        )
        self.lower = np.asarray(self.model.lowerPositionLimit)[self._q_indices].copy()
        self.upper = np.asarray(self.model.upperPositionLimit)[self._q_indices].copy()
        if not np.isfinite([self.lower, self.upper]).all() or np.any(self.lower >= self.upper):
            raise ValueError("Arm joints require finite, ordered URDF position limits")
        self._frames, self._tools = {}, {}
        for hand in ("left", "right"):
            frame = settings[f"{hand}_frame"]
            frame_id = self.model.getFrameId(frame)
            if frame_id >= self.model.nframes:
                raise ValueError(f"Unknown {hand} task frame: {frame}")
            self._frames[hand] = frame_id
            self._tools[hand] = _transform(
                settings.get(f"{hand}_tool_transform", np.eye(4)), f"{hand}_tool_transform"
            )
        motion = config.get("motion", {})
        self.position_tolerance = float(motion.get("fk_position_tolerance_m", 0.003))
        self.orientation_tolerance = float(motion.get("fk_orientation_tolerance_rad", 0.08))
        if not all(
            math.isfinite(v) and v > 0
            for v in (self.position_tolerance, self.orientation_tolerance)
        ):
            raise ValueError("FK tolerances must be finite and positive")

    @staticmethod
    def _arm_slice(hand):
        if hand not in ("left", "right"):
            raise ValueError("hand must be left or right")
        return slice(0, 7) if hand == "left" else slice(7, 14)

    def _checked_q14(self, q14):
        values = _vector(q14, 14, "arm configuration")
        if np.any(values < self.lower) or np.any(values > self.upper):
            raise ValueError("Arm configuration exceeds URDF limits")
        return values

    def _configuration(self, q14):
        full = self._base_q.copy()
        full[self._q_indices] = self._checked_q14(q14)
        return full

    def fk(self, q14, hand):
        self._arm_slice(hand)
        full = self._configuration(q14)
        with self._lock:
            self._pin.framesForwardKinematics(self.model, self.data, full)
            return (
                np.asarray(self.data.oMf[self._frames[hand]].homogeneous).copy() @ self._tools[hand]
            )

    def gravity(self, q14):
        full = self._configuration(q14)
        with self._lock:
            tau = self._pin.rnea(
                self.model, self.data, full, np.zeros(self.model.nv), np.zeros(self.model.nv)
            )
            result = np.asarray(tau)[self._v_indices].copy()
        if not np.isfinite(result).all():
            raise RuntimeError("RNEA returned non-finite gravity torque")
        return result

    def _error(self, q14, target, hand):
        actual = self.fk(q14, hand)
        return np.r_[
            actual[:3, 3] - target[:3, 3], self._pin.log3(target[:3, :3].T @ actual[:3, :3])
        ]

    def check_pose(self, q14, pose, hand):
        error = self._error(q14, _pose_transform(pose), hand)
        position, orientation = np.linalg.norm(error[:3]), np.linalg.norm(error[3:])
        if (
            not np.isfinite(error).all()
            or position > self.position_tolerance
            or orientation > self.orientation_tolerance
        ):
            raise ValueError(
                f"FK target mismatch: position={position:.6f}m, orientation={orientation:.6f}rad"
            )

    def solve(self, pose, seed14, hand):
        from scipy.optimize import least_squares

        seed = self._checked_q14(seed14)
        active, target = self._arm_slice(hand), _pose_transform(pose)
        scale = np.r_[np.full(3, self.position_tolerance), np.full(3, self.orientation_tolerance)]

        def residual(values):
            candidate = seed.copy()
            candidate[active] = values
            return np.r_[
                self._error(candidate, target, hand) / scale, 1e-4 * (values - seed[active])
            ]

        solved = least_squares(
            residual,
            seed[active],
            bounds=(self.lower[active], self.upper[active]),
            max_nfev=500,
            ftol=1e-10,
            xtol=1e-10,
            gtol=1e-10,
        )
        if solved.status < 0:
            raise ValueError(f"IK failed: {solved.message}")
        result = seed.copy()
        result[active] = solved.x
        # A redundant arm may exhaust the optimization budget while its task
        # residual is already within tolerance. Conversely, an optimizer's
        # success flag alone never establishes that a target is reachable.
        self.check_pose(result, pose, hand)
        return result


def compile_job(plan, kinematics):
    """Copy a plan and fill missing joint paths; validate supplied paths by FK.

    Entry/parking route continuity, reload sequencing, and material policy are
    checked by the job validator. This function performs no hardware access.
    """
    result = copy.deepcopy(plan)
    result.setdefault("format", "astra-paint-execute/v1")
    result.setdefault("reload_every", 3)
    hand = result.setdefault("hand", "right")
    active = kinematics._arm_slice(hand)
    frozen = slice(7, 14) if active.start == 0 else slice(0, 7)
    seed = kinematics._checked_q14(result["entry_q14"][-1])
    frozen_q7 = seed[frozen].copy()
    for batch in result["batches"]:
        action = batch["action"]
        if not isinstance(action, list) or not action:
            raise ValueError("Every batch requires a nonempty action list")
        if batch["phase"] == "LOAD":
            index = batch.get("contact_index")
            if type(index) is not int or not 0 <= index < len(action):
                raise ValueError("LOAD requires an explicit valid contact_index")
        path = batch.get("joint_path_q14")
        if path is not None and (not isinstance(path, list) or len(path) != len(action)):
            raise ValueError("joint_path_q14 must have one target per action pose")
        compiled = []
        for index, pose in enumerate(action):
            target = (
                kinematics.solve(pose, seed, hand)
                if path is None
                else kinematics._checked_q14(path[index])
            )
            if not np.allclose(target[frozen], frozen_q7, atol=1e-10, rtol=0):
                raise ValueError("Compiled painting trajectory changes the frozen arm")
            kinematics.check_pose(target, pose, hand)
            compiled.append(target.tolist())
            seed = target
        if path is None:
            batch["joint_path_q14"] = compiled
    if result.get("park_q14") and result["park_q14"][0] is None:
        result["park_q14"][0] = seed.tolist()
    return result
