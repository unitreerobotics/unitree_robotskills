"""R1-A7-D Runtime URDF kinematics in the waist_yaw_link frame."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from r1d_chess.robot.r1d_config import MODEL_PATH, load_profile, require_calibration, finite_vector
from r1d_chess.robot.r1d_urdf import Arm

COMMAND_LIMIT_TOLERANCE_RAD = .02   # measured-feedback excursion past a URDF limit that is clipped, not rejected


class R1DKinematics:
    def __init__(self, profile=None):
        self.profile = load_profile() if profile is None else profile
        root = ET.parse(MODEL_PATH).getroot()
        self.arms = [Arm(root, side) for side in ('left', 'right')]
        for side, arm in zip(('left', 'right'), self.arms):
            # Missing calibration permits only labelled wrist-frame previews.
            value = self.profile['tcp_xyz_rpy'][side]
            pose = np.zeros(6) if value is None else finite_vector(value, 6, side)
            arm.tcp = np.eye(4)
            arm.tcp[:3, 3] = pose[:3]
            arm.tcp[:3, :3] = Rotation.from_euler('xyz', pose[3:]).as_matrix()

    @staticmethod
    def finite_joints(q):
        q = np.asarray(q, dtype=float)
        if q.shape != (14,) or not np.isfinite(q).all():
            raise ValueError('expected 14 finite physical left-then-right arm joints')
        return q

    def limits(self):
        return (np.r_[self.arms[0].lower, self.arms[1].lower], np.r_[self.arms[0].upper, self.arms[1].upper])

    def check_joints(self, q):
        """Strict URDF limit check for planned targets and IK solutions."""
        q = self.finite_joints(q)
        lower, upper = self.limits()
        if np.any(q < lower) or np.any(q > upper):
            raise ValueError('arm target outside R1-D URDF limits')
        return q

    def clip_joints(self, q, tolerance=COMMAND_LIMIT_TOLERANCE_RAD):
        """For commands and seeds derived from measured feedback: tracking can carry a joint a little past
        a URDF limit (e.g. 0.1 mrad at the 0.5236 rad wrist pitch limit). Clip small excursions into the
        limits; anything farther than `tolerance` outside is still an error."""
        q = self.finite_joints(q)
        lower, upper = self.limits()
        if np.any(q < lower-tolerance) or np.any(q > upper+tolerance):
            raise ValueError('arm target outside R1-D URDF limits')
        return np.clip(q, lower, upper)

    def fk(self, q):
        # FK is also evaluated on measured feedback, which may sit slightly past a limit: never reject it.
        q = self.finite_joints(q)
        return [self.arms[0].fk(q[:7]), self.arms[1].fk(q[7:])]

    def solve(self, targets, seed):
        require_calibration(self.profile)
        seed = self.clip_joints(seed)
        if len(targets) != 2:
            raise ValueError('both physical arm target transforms required')
        solutions = [arm.solve(target, start) for arm, target, start in
                     zip(self.arms, targets, (seed[:7], seed[7:]))]
        if not all(s['accepted'] for s in solutions):
            raise ValueError(f'R1-D IK rejected: {solutions}')
        return self.check_joints(np.concatenate([s['q_rad'] for s in solutions]))
