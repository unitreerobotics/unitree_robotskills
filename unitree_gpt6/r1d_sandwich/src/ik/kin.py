# Vendored from r1-ik r1ik/kin.py (5b867e6 + working tree, 2026-11-19). Keep in sync by copying; do not restyle.
"""Pinocchio model helpers: per-mode reduced models, end-effector frames, metrics."""

from __future__ import annotations

import numpy as np
import pinocchio as pin

from .models import VARIANTS, Variant, build_ik_urdf

EE_FRAME = {"left": "left_ee", "right": "right_ee"}

# Nominal "ready" arm posture (left-arm values; MIRRORED joints flip sign on the right).
# R1 arms at q=0 are already an L: upper arm down, forearm forward (elbow joint
# range [-0.98, 2.19] never reaches straight). Nominal = that L, hands slightly
# raised and shoulders slightly abducted to clear the torso.
# Signs follow the URDF limits: left shoulder_roll in [-0.23, 2.48] means + is outward.
NOMINAL_ARM = {"shoulder_pitch": -0.2, "shoulder_roll": 0.15, "elbow": 0.0}
MIRRORED = {"shoulder_roll", "shoulder_yaw", "wrist_roll", "wrist_yaw"}
ARM_JOINTS7 = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")


def nominal_joints() -> dict:
    """NOMINAL_ARM on both arms (mirrored), every other arm joint 0, waist 0: {joint name: rad}.
    The standard start pose of the real-robot deploy layer (forearms forward, wrists neutral)."""
    q = {"waist_yaw_joint": 0.0}
    for side, sign in (("left", 1.0), ("right", -1.0)):
        for name in ARM_JOINTS7:
            q[f"{side}_{name}_joint"] = NOMINAL_ARM.get(name, 0.0) * (sign if name in MIRRORED else 1.0)
    return q


def base_model(variant: Variant) -> pin.Model:
    urdf = build_ik_urdf(variant)
    model = pin.buildModelFromUrdf(str(urdf))
    assert model.nq == model.nv, f"{variant.name}: nq={model.nq} nv={model.nv} (continuous joint left?)"
    for side in ("left", "right"):
        name = EE_FRAME[side]
        if not model.existFrame(name):
            parent = variant.ee_parent.format(side=side)
            jid = model.getJointId(parent) if model.existJointName(parent) else model.getFrameId(parent)
            parent_name = parent if model.existJointName(parent) else model.frames[jid].parentJoint
            model.addFrame(
                pin.Frame(name, model.getJointId(parent_name), pin.SE3(np.eye(3), np.array(variant.ee_offset)), pin.FIXED_JOINT)
            )
    return model


class ReducedModel:
    """A variant restricted to one control mode; other joints are frozen."""

    def __init__(self, variant_name: str, mode: str, locked_q: dict[str, float] | None = None):
        self.variant = VARIANTS[variant_name]
        self.mode = mode
        self.full = base_model(self.variant)
        locked_q = dict(locked_q or {})
        active = self.variant.mode_joints(mode)
        self.active_names = list(active)

        full_names = list(self.full.names[1:])
        unknown = set(locked_q) - set(full_names)
        assert not unknown, f"unknown joints in locked_q: {sorted(unknown)}"
        missing = set(active) - set(full_names)
        assert not missing, f"{variant_name}: active joints missing from URDF: {sorted(missing)}"

        q_full = pin.neutral(self.full)
        for name, value in locked_q.items():
            q_full[self.full.joints[self.full.getJointId(name)].idx_q] = value
        # A lift-locked model still needs a sane posture for the arms.
        lock_ids = [self.full.getJointId(n) for n in full_names if n not in active]
        self.model = pin.buildReducedModel(self.full, lock_ids, q_full)

        # buildReducedModel emits the kept joints in their original order.
        self.active_names = [n for n in full_names if n in set(active)]
        self.kept_full_ids = {n: self.full.getJointId(n) for n in self.active_names}
        self.q_locked_full = np.array([q_full[j.idx_q] for j in self.full.joints[1:]])
        self.lower = self.model.lowerPositionLimit.copy()
        self.upper = self.model.upperPositionLimit.copy()
        assert np.all(self.lower < self.upper), f"{active}: degenerate limits {self.lower} {self.upper}"
        self.data = self.model.createData()

    # --- index helpers -------------------------------------------------
    def index(self, joint: str) -> int:
        return self.model.joints[self.model.getJointId(joint)].idx_q

    def has(self, joint: str) -> bool:
        return joint in self.active_names

    # --- kinematics ----------------------------------------------------
    def fk(self, q: np.ndarray) -> dict[str, pin.SE3]:
        pin.forwardKinematics(self.model, self.data, np.asarray(q, dtype=float))
        pin.updateFramePlacements(self.model, self.data)
        return {s: self.data.oMf[self.model.getFrameId(n)].copy() for s, n in EE_FRAME.items()}

    def nominal_q(self, lift: float = 0.25) -> np.ndarray:
        """Relaxed working posture used as IK seed and posture prior (see NOMINAL_ARM)."""
        q = np.zeros(self.model.nq)
        for name, value in NOMINAL_ARM.items():
            for side, sign in (("left", 1.0), ("right", -1.0)):
                joint = f"{side}_{name}_joint"
                if self.has(joint):
                    q[self.index(joint)] = value * (sign if name in MIRRORED else 1.0)
        if self.has("lift_stage1_joint"):
            q[self.index("lift_stage1_joint")] = lift
        return self.clip(q)

    def sample_q(self, rng: np.random.Generator, margin: float = 0.05) -> np.ndarray:
        return rng.uniform(self.lower + margin, self.upper - margin)

    def clip(self, q: np.ndarray) -> np.ndarray:
        return np.clip(q, self.lower, self.upper)


def pose_error(T_target: pin.SE3, T_actual: pin.SE3) -> tuple[float, float]:
    """Position error [m] and orientation error [deg]."""
    pos = float(np.linalg.norm(T_target.translation - T_actual.translation))
    rot = pin.log3(T_target.rotation.T @ T_actual.rotation)
    return pos, float(np.linalg.norm(rot)) * 180.0 / np.pi


def rot_error_deg(T_target: pin.SE3, T_actual: pin.SE3) -> float:
    return pose_error(T_target, T_actual)[1]
