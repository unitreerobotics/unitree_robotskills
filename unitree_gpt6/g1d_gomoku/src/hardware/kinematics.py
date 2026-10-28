"""Strict pose IK using the bundled G1 model and CasADi functions."""

import sys

import numpy as np
from scipy.spatial.transform import Rotation

from .common import positive, vector

FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")
SIDES = ("left", "right")


def pose_matrix(pose):
    if isinstance(pose, dict):
        pose = [pose[key] for key in FIELDS]
    pose = vector(pose, 6, "xyz+rpy")
    matrix = np.eye(4)
    matrix[:3, 3] = pose[:3]
    # Extrinsic xyz: R = Rz(yaw) Ry(pitch) Rx(roll), matching original matrix_to_xyzrpy.
    matrix[:3, :3] = Rotation.from_euler("xyz", pose[3:]).as_matrix()
    return matrix


def matrix_pose(matrix):
    return np.r_[matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz")].tolist()


class Kinematics:
    def __init__(self, position_tolerance=0.005, rotation_tolerance=0.03):
        try:
            from unitree_gpt6.g1d_gomoku.models.g1.arm_model import G1_29_ArmIK
        except ImportError as exc:
            raise RuntimeError(
                f"G1 numerical backend import failed in {sys.executable}: "
                f"{type(exc).__name__}: {exc}. "
                "Run the hardware model-check command with this same Python interpreter. "
                "The backend requires pinocchio, pinocchio.casadi and casadi; "
                "bundled G1 assets do not provide these numerical libraries."
            ) from exc
        self.ik = G1_29_ArmIK()
        self.model = self.ik.reduced_robot.model
        self.names = list(self.model.names)[1:]
        if self.model.nq != 14 or self.model.nv != 14 or len(self.names) != 14:
            raise ValueError("Expected the original G1 14-joint reduced model")
        self.position_tolerance = positive(position_tolerance, "position_tolerance")
        self.rotation_tolerance = positive(rotation_tolerance, "rotation_tolerance")
        self.set_waist([0, 0, 0])

    def set_waist(self, waist):
        """Map the original zero-waist model into the measured URDF pelvis frame."""
        import pinocchio as pin

        self.waist = vector(waist, 3, "waist yaw/roll/pitch")
        full = self.ik.robot.model
        self.full_q = np.zeros(full.nq)
        for name, value in zip(
            ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"),
            self.waist,
            strict=False,
        ):
            joint_id = full.getJointId(name)
            if joint_id == 0 or full.joints[joint_id].nq != 1:
                raise ValueError(f"Bundled G1 model is missing scalar joint {name}")
            self.full_q[full.joints[joint_id].idx_q] = value
        data = full.createData()
        frame = full.getFrameId("torso_link")
        if frame >= full.nframes:
            raise ValueError("Bundled G1 model is missing torso_link")
        pin.framesForwardKinematics(full, data, np.zeros(full.nq))
        neutral = data.oMf[frame].homogeneous.copy()
        pin.framesForwardKinematics(full, data, self.full_q)
        self.base_transform = data.oMf[frame].homogeneous.copy() @ np.linalg.inv(neutral)

    def check_joints(self, q):
        q = vector(q, 14, "arm joints")
        lower, upper = self.model.lowerPositionLimit, self.model.upperPositionLimit
        invalid = np.flatnonzero((q < lower - 1e-6) | (q > upper + 1e-6))
        if invalid.size:
            details = "; ".join(
                f"{self.names[i]}={q[i]:.6f} rad outside [{lower[i]:.6f}, {upper[i]:.6f}]"
                for i in invalid
            )
            raise ValueError(f"Arm joints exceed URDF position limits: {details}")
        return q

    def fk(self, q):
        q = self.check_joints(q)
        return dict(
            zip(SIDES, (self.base_transform @ m for m in self.ik.solve_fk_matrix(q)), strict=True)
        )

    def tau(self, q):
        import pinocchio as pin

        q = self.check_joints(q)
        full = self.ik.robot.model
        full_q = self.full_q.copy()
        indices = []
        for name, value in zip(self.names, q, strict=False):
            joint = full.joints[full.getJointId(name)]
            full_q[joint.idx_q] = value
            indices.append(joint.idx_v)
        tau = pin.rnea(full, full.createData(), full_q, np.zeros(full.nv), np.zeros(full.nv))
        return vector(tau[indices], 14, "gravity compensation")

    def solve(self, targets, seed):
        import casadi as ca

        seed = self.check_joints(seed)
        if not targets or set(targets) - set(SIDES):
            raise ValueError("Specify left and/or right targets")
        matrices = self.fk(seed)
        for side, pose in targets.items():
            matrices[side] = pose_matrix(pose)
        opti = ca.Opti()
        blocks, variables = [], {}
        for i, side in enumerate(SIDES):
            sl = slice(i * 7, (i + 1) * 7)
            if side in targets:
                var = opti.variable(7)
                variables[side] = var
                opti.subject_to(
                    opti.bounded(
                        self.model.lowerPositionLimit[sl], var, self.model.upperPositionLimit[sl]
                    )
                )
                opti.set_initial(var, seed[sl])
                blocks.append(var)
            else:
                blocks.append(
                    ca.DM(seed[sl])
                )  # Inactive arm fixed exactly, not a soft pose constraint.
        q = ca.vertcat(*blocks)
        inverse = np.linalg.inv(self.base_transform)
        translation = self.ik.translational_error(
            q, inverse @ matrices["left"], inverse @ matrices["right"]
        )
        rotation = self.ik.rotational_error(
            q, inverse @ matrices["left"], inverse @ matrices["right"]
        )
        opti.minimize(
            10000 * ca.sumsqr(translation) + 100 * ca.sumsqr(rotation) + 0.001 * ca.sumsqr(q - seed)
        )
        opti.solver(
            "ipopt",
            {
                "expand": True,
                "print_time": False,
                "ipopt.print_level": 0,
                "ipopt.sb": "yes",
                "ipopt.max_iter": 150,
                "ipopt.tol": 1e-7,
            },
        )
        try:
            solution = opti.solve()
        except RuntimeError as exc:
            raise ValueError("IK did not converge; no motion command generated") from exc
        result = seed.copy()
        for i, side in enumerate(SIDES):
            if side in variables:
                result[i * 7 : (i + 1) * 7] = np.asarray(solution.value(variables[side])).reshape(7)
        result = self.check_joints(result)
        actual = self.fk(result)
        errors = {}
        for side in targets:
            pos = float(np.linalg.norm(actual[side][:3, 3] - matrices[side][:3, 3]))
            angle = float(
                Rotation.from_matrix(actual[side][:3, :3] @ matrices[side][:3, :3].T).magnitude()
            )
            errors[side] = {"position_m": pos, "rotation_rad": angle}
            if pos > self.position_tolerance or angle > self.rotation_tolerance:
                raise ValueError(f"{side} target unreachable within tolerance: {errors[side]}")
        return {
            "joint_positions": result.tolist(),
            "joint_names": self.names,
            "gravity_torques": self.tau(result).tolist(),
            "poses": {side: matrix_pose(m) for side, m in actual.items()},
            "errors": errors,
        }
