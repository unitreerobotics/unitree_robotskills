"""G1 arm model and symbolic errors used by the skill's strict IK solver.

Retains the unitree_deploy 0.0.5 reduction, frame offsets and error definitions.
See README.md and licenses/ for attribution. No device, viewer or disk cache
is created here; optimization remains in the skill's Kinematics.solve().
"""

import casadi as ca
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin

from .config import ARM_JOINT_NAMES
from .resources import URDF_NAME, asset_directory, validate_assets


class G1_29_ArmIK:
    def __init__(self):
        validate_assets()
        assets = asset_directory()
        self.robot = pin.RobotWrapper.BuildFromURDF(str(assets / URDF_NAME), str(assets))
        full_names = set(self.robot.model.names)
        if not set(ARM_JOINT_NAMES) <= full_names:
            raise ValueError("Bundled G1 model is missing expected arm joints")
        locked = [name for name in list(self.robot.model.names)[1:] if name not in ARM_JOINT_NAMES]
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=locked,
            reference_configuration=np.zeros(self.robot.model.nq),
        )
        model = self.reduced_robot.model
        if tuple(model.names)[1:] != ARM_JOINT_NAMES or model.nq != 14 or model.nv != 14:
            raise ValueError("Bundled G1 arm joint order does not match DDS motor order")
        for side, frame in (("left", "L_ee"), ("right", "R_ee")):
            model.addFrame(
                pin.Frame(
                    frame,
                    model.getJointId(f"{side}_wrist_yaw_joint"),
                    pin.SE3(np.eye(3), np.array([0.05, 0.0, 0.0])),
                    pin.FrameType.OP_FRAME,
                )
            )
        self.reduced_robot.data = model.createData()
        self.L_hand_id = model.getFrameId("L_ee")
        self.R_hand_id = model.getFrameId("R_ee")

        symbolic_model = cpin.Model(model)
        symbolic_data = symbolic_model.createData()
        q = ca.SX.sym("q", model.nq, 1)
        left = ca.SX.sym("tf_l", 4, 4)
        right = ca.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(symbolic_model, symbolic_data, q)
        frames = (self.L_hand_id, self.R_hand_id)
        targets = (left, right)
        self.translational_error = ca.Function(
            "translational_error",
            [q, left, right],
            [
                ca.vertcat(
                    *(
                        symbolic_data.oMf[frame].translation - target[:3, 3]
                        for frame, target in zip(frames, targets, strict=True)
                    )
                )
            ],
        )
        self.rotational_error = ca.Function(
            "rotational_error",
            [q, left, right],
            [
                ca.vertcat(
                    *(
                        cpin.log3(symbolic_data.oMf[frame].rotation @ target[:3, :3].T)
                        for frame, target in zip(frames, targets, strict=True)
                    )
                )
            ],
        )

    def solve_fk_matrix(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (14,) or not np.isfinite(q).all():
            raise ValueError("FK requires 14 finite arm joint positions")
        model, data = self.reduced_robot.model, self.reduced_robot.data
        pin.framesForwardKinematics(model, data, q)
        return tuple(
            data.oMf[frame].homogeneous.copy() for frame in (self.L_hand_id, self.R_hand_id)
        )
