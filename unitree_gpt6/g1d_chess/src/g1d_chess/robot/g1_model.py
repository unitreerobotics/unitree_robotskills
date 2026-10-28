"""Package-local G1 reduced arm model; no unitree_deploy or robot connection.

Preserves the previous URDF, locked joints, zero reference configuration and
50 mm wrist-yaw-to-end-effector frames. IK and dynamics remain in motion_harness.
"""
from pathlib import Path

import numpy as np
import pinocchio as pin

from g1d_chess.paths import ROOT
MODEL_DIR = ROOT / 'assets' / 'models' / 'g1'
URDF_PATH = MODEL_DIR / 'g1_body29_hand14.urdf'

LOCKED_JOINTS = (
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
    'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
    'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
    'left_hand_thumb_0_joint', 'left_hand_thumb_1_joint', 'left_hand_thumb_2_joint',
    'left_hand_middle_0_joint', 'left_hand_middle_1_joint',
    'left_hand_index_0_joint', 'left_hand_index_1_joint',
    'right_hand_thumb_0_joint', 'right_hand_thumb_1_joint', 'right_hand_thumb_2_joint',
    'right_hand_index_0_joint', 'right_hand_index_1_joint',
    'right_hand_middle_0_joint', 'right_hand_middle_1_joint',
)


class G1ArmModel:
    def __init__(self):
        self.robot = pin.RobotWrapper.BuildFromURDF(str(URDF_PATH), str(MODEL_DIR))
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=list(LOCKED_JOINTS),
            reference_configuration=np.zeros(self.robot.model.nq),
        )
        for prefix, side in [('L', 'left'), ('R', 'right')]:
            self.reduced_robot.model.addFrame(pin.Frame(
                prefix + '_ee',
                self.reduced_robot.model.getJointId(side + '_wrist_yaw_joint'),
                0,
                pin.SE3(np.eye(3), np.array([0.05, 0.0, 0.0])),
                pin.FrameType.OP_FRAME,
            ))
        self.reduced_robot.data = self.reduced_robot.model.createData()
