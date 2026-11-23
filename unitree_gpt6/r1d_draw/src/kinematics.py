"""Arm kinematics of the R1-D from the user's URDF; no devices and no model files bundled.

Poses use meters and XYZ Euler radians in ``model.base_frame`` (the pelvis by
default), so the lift height does not enter them. A hand's pose is the pose of
its configured frame times its tool transform, i.e. the brush tip. The waist
yaw sits between the pelvis and the arms and is fixed per job.
"""

from __future__ import annotations

import copy
import math
import threading
from pathlib import Path

import numpy as np

from .job import TRAVEL_SPACING_M
from .schema import ARM_JOINTS, HANDS, POSE_KEYS, array, number, pose, vector

ARM_JOINT_NAMES = tuple(f"{side}_{joint}_joint" for side in HANDS for joint in ARM_JOINTS)


def pose_transform(value):
    """4x4 transform of an x/y/z/roll/pitch/yaw pose (Rz(yaw) Ry(pitch) Rx(roll))."""
    x, y, z, roll, pitch, yaw = pose(value)
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    result[:3, 3] = [x, y, z]
    return result


def transform_pose(transform):
    """Inverse of ``pose_transform``."""
    rotation = transform[:3, :3]
    pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))
    if math.hypot(rotation[0, 0], rotation[1, 0]) > 1e-9:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll, yaw = math.atan2(-rotation[1, 2], rotation[1, 1]), 0.0
    return dict(zip(POSE_KEYS, [*transform[:3, 3].tolist(), roll, pitch, yaw], strict=True))


class PinocchioKinematics:
    """FK, bounded IK and joint limits of both 7-axis arms at one waist yaw."""

    is_simulation = False

    def __init__(self, config: dict, waist_yaw: float = 0.0):
        try:
            import pinocchio as pin
        except ImportError as exc:
            raise ImportError("kinematics requires the 'pin' package") from exc
        self._pin = pin
        settings = config.get("model")
        if not settings:
            raise ValueError("the configuration has no model section")
        urdf = Path(settings["urdf"])
        if not urdf.is_file():
            raise ValueError(f"model.urdf does not exist: {urdf}")
        full = pin.buildModelFromUrdf(str(urdf))
        names = set(full.names[1:])
        if not set(ARM_JOINT_NAMES) <= names or "waist_yaw_joint" not in names:
            raise ValueError("URDF lacks the R1-D arm or waist joints")
        self.waist_yaw = number(waist_yaw, "waist_yaw")
        reference = pin.neutral(full)
        reference[full.joints[full.getJointId("waist_yaw_joint")].idx_q] = self.waist_yaw
        locked = [i for i in range(1, full.njoints) if full.names[i] not in ARM_JOINT_NAMES]
        self.model = pin.buildReducedModel(full, locked, reference)
        self.data = self.model.createData()
        self._lock = threading.RLock()
        self._q_indices = np.array(
            [self.model.joints[self.model.getJointId(n)].idx_q for n in ARM_JOINT_NAMES]
        )
        self.lower = np.asarray(self.model.lowerPositionLimit)[self._q_indices].copy()
        self.upper = np.asarray(self.model.upperPositionLimit)[self._q_indices].copy()
        if not np.isfinite([self.lower, self.upper]).all() or np.any(self.lower >= self.upper):
            raise ValueError("arm joints require finite, ordered URDF position limits")
        self._base = self._frame(settings["base_frame"], "base")
        self._frames = {hand: self._frame(settings[f"{hand}_frame"], hand) for hand in HANDS}
        self._tools = {
            hand: array(settings[f"{hand}_tool_transform"], (4, 4), f"{hand}_tool_transform")
            for hand in HANDS
        }
        motion = config["motion"]
        self.position_tolerance = motion["fk_position_tolerance_m"]
        self.orientation_tolerance = motion["fk_orientation_tolerance_rad"]

    def _frame(self, name, label):
        frame_id = self.model.getFrameId(name)
        if frame_id >= self.model.nframes:
            raise ValueError(f"unknown {label} frame: {name}")
        return frame_id

    @staticmethod
    def arm_slice(hand):
        if hand not in HANDS:
            raise ValueError("hand must be left or right")
        return slice(0, 7) if hand == "left" else slice(7, 14)

    def checked_q14(self, q14):
        values = vector(q14, 14, "arm configuration")
        if np.any(values < self.lower) or np.any(values > self.upper):
            raise ValueError("arm configuration exceeds URDF limits")
        return values

    def fk(self, q14, hand, *, check_limits=True):
        """Brush-tip transform of one hand in the base frame."""
        self.arm_slice(hand)
        values = self.checked_q14(q14) if check_limits else vector(q14, 14, "arm configuration")
        full = np.zeros(self.model.nq)
        full[self._q_indices] = values
        with self._lock:
            self._pin.framesForwardKinematics(self.model, self.data, full)
            base = self.data.oMf[self._base].homogeneous
            tip = self.data.oMf[self._frames[hand]].homogeneous @ self._tools[hand]
            return np.linalg.inv(base) @ tip

    def _error(self, q14, target, hand):
        actual = self.fk(q14, hand)
        rotation = self._pin.log3(target[:3, :3].T @ actual[:3, :3])
        return np.r_[actual[:3, 3] - target[:3, 3], rotation]

    def check_pose(self, q14, target_pose, hand):
        error = self._error(q14, pose_transform(target_pose), hand)
        position, orientation = np.linalg.norm(error[:3]), np.linalg.norm(error[3:])
        if position > self.position_tolerance or orientation > self.orientation_tolerance:
            raise ValueError(
                f"FK target mismatch: position={position:.6f}m, orientation={orientation:.6f}rad"
            )

    def solve(self, target_pose, seed14, hand):
        """Arm angles that reach a pose, staying close to the seed."""
        from scipy.optimize import least_squares

        seed = self.checked_q14(seed14)
        active, target = self.arm_slice(hand), pose_transform(target_pose)
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
        # The optimizer's success flag alone never establishes that a target is reachable.
        self.check_pose(result, target_pose, hand)
        return result


def travel_path(kinematics, start_q14, target_pose, hand, spacing=TRAVEL_SPACING_M):
    """Arm angles along the straight line from where the brush tip is to a pose.

    The target itself is left out. A single joint move between two distant
    poses lets the tip sag or swing off the line by centimetres; these targets
    keep it on the line.
    """
    from scipy.spatial.transform import Rotation

    seed = np.clip(np.asarray(start_q14, dtype=float), kinematics.lower, kinematics.upper)
    begin, end = kinematics.fk(seed, hand), pose_transform(target_pose)
    turn = Rotation.from_matrix(begin[:3, :3].T @ end[:3, :3]).as_rotvec()
    count = math.ceil(float(np.linalg.norm(end[:3, 3] - begin[:3, 3])) / spacing)
    result = []
    for index in range(1, count):
        share, step = index / count, np.eye(4)
        step[:3, :3] = begin[:3, :3] @ Rotation.from_rotvec(share * turn).as_matrix()
        step[:3, 3] = begin[:3, 3] + share * (end[:3, 3] - begin[:3, 3])
        seed = kinematics.solve(transform_pose(step), seed, hand)
        result.append(seed)
    return result


def compile_job(plan, kinematics):
    """Copy a plan and fill in the joint path of every batch; no hardware access."""
    result = copy.deepcopy(plan)
    hand = result.setdefault("hand", "right")
    active = kinematics.arm_slice(hand)
    frozen = slice(7, 14) if active.start == 0 else slice(0, 7)
    seed = kinematics.checked_q14(result["entry_q14"][-1])
    frozen_q7 = seed[frozen].copy()
    for batch in result["batches"]:
        action = batch["action"]
        if not isinstance(action, list) or not action:
            raise ValueError("every batch requires a nonempty action list")
        path = batch.get("joint_path_q14")
        if path is not None and (not isinstance(path, list) or len(path) != len(action)):
            raise ValueError("joint_path_q14 must have one target per action pose")
        if "travel_q14" not in batch:
            travel = travel_path(kinematics, seed, action[0], hand)
            batch["travel_q14"] = [q.tolist() for q in travel]
            seed = travel[-1] if travel else seed
        compiled = []
        for index, target_pose in enumerate(action):
            target = (
                kinematics.solve(target_pose, seed, hand)
                if path is None
                else kinematics.checked_q14(path[index])
            )
            if not np.allclose(target[frozen], frozen_q7, atol=1e-10, rtol=0):
                raise ValueError("compiled trajectory changes the idle arm")
            kinematics.check_pose(target, target_pose, hand)
            compiled.append(target.tolist())
            seed = target
        if path is None:
            batch["joint_path_q14"] = compiled
    if result.get("park_q14") and result["park_q14"][0] is None:
        result["park_q14"][0] = seed.tolist()
    return result
