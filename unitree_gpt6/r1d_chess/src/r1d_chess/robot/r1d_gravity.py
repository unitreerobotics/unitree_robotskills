"""Static gravity torque of both R1-A7-D arms from the Runtime URDF.

Offline only: no DDS, IPC or robot commands. Frame is waist_yaw_link with
gravity along -z (pelvis assumed level, as in the rest of this adapter).
Joint order is physical left 7 then physical right 7 (motors 15..28).

The returned torque is the motor torque needed to hold the arm statically,
i.e. dV/dq, the same convention as Pinocchio computeGeneralizedGravity.
"""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

SUFFIXES = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
            'wrist_roll', 'wrist_pitch', 'wrist_yaw')
GRAVITY = np.array([0., 0., -9.81])


def _xyz(element, key):
    return np.fromstring(element.get(key, '0 0 0') if element is not None else '0 0 0', sep=' ')


class _ArmModel:
    def __init__(self, root, side):
        joints = {j.get('name'): j for j in root.findall('joint')}
        links = {l.get('name'): l for l in root.findall('link')}
        self.origins, self.axes, self.masses, self.coms = [], [], [], []
        parent = 'waist_yaw_link'
        for suffix in SUFFIXES:
            joint = joints[f'{side}_{suffix}_joint']
            if joint.find('parent').get('link') != parent or joint.get('type') != 'revolute':
                raise ValueError(f'unexpected URDF chain at {side}_{suffix}_joint')
            parent = joint.find('child').get('link')
            origin = joint.find('origin')
            transform = np.eye(4)
            transform[:3, :3] = Rotation.from_euler('xyz', _xyz(origin, 'rpy')).as_matrix()
            transform[:3, 3] = _xyz(origin, 'xyz')
            self.origins.append(transform)
            self.axes.append(_xyz(joint.find('axis'), 'xyz'))
            inertial = links[parent].find('inertial')
            self.masses.append(float(inertial.find('mass').get('value')))
            self.coms.append(_xyz(inertial.find('origin'), 'xyz'))
        self.masses = np.array(self.masses)
        # Bodies rigidly carried by the last arm link (e.g. Dex1 base, fingers at
        # q=0, camera): mass and COM expressed in the wrist_yaw frame.
        self.tool_masses, self.tool_coms = [], []
        stack = [(parent, np.eye(4))]
        while stack:
            link, transform = stack.pop()
            for joint in root.findall('joint'):
                if joint.find('parent').get('link') != link or joint.get('type') not in ('fixed', 'prismatic'):
                    continue
                origin = joint.find('origin')
                local = np.eye(4)
                local[:3, :3] = Rotation.from_euler('xyz', _xyz(origin, 'rpy')).as_matrix()
                local[:3, 3] = _xyz(origin, 'xyz')
                child = joint.find('child').get('link')
                frame = transform @ local
                inertial = links[child].find('inertial')
                if inertial is not None:
                    self.tool_masses.append(float(inertial.find('mass').get('value')))
                    self.tool_coms.append((frame @ np.r_[_xyz(inertial.find('origin'), 'xyz'), 1.])[:3])
                stack.append((child, frame))
        self.tool_mass = float(sum(self.tool_masses))

    def frames(self, q):
        transform = np.eye(4)
        result = []
        for origin, axis, angle in zip(self.origins, self.axes, q):
            joint = np.eye(4)
            joint[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            transform = transform @ origin @ joint
            result.append(transform.copy())
        return result

    def bodies(self, q, payload_mass=0., payload_com=(0., 0., 0.)):
        frames = self.frames(q)
        masses = list(self.masses) + list(self.tool_masses)
        coms = ([(f @ np.r_[c, 1.])[:3] for f, c in zip(frames, self.coms)] +
                [(frames[-1] @ np.r_[c, 1.])[:3] for c in self.tool_coms])
        if payload_mass:
            masses.append(float(payload_mass))
            coms.append((frames[-1] @ np.r_[np.asarray(payload_com, float), 1.])[:3])
        return frames, np.array(masses), np.array(coms)

    def torque(self, q, payload_mass=0., payload_com=(0., 0., 0.)):
        frames, masses, coms = self.bodies(q, payload_mass, payload_com)
        n = len(frames)
        body_joint = np.r_[np.arange(n), [n - 1] * (len(masses) - n)]
        tau = np.zeros(n)
        for i, frame in enumerate(frames):
            axis = frame[:3, :3] @ self.axes[i]
            point = frame[:3, 3]
            moving = body_joint >= i
            moment = np.cross(coms[moving] - point, masses[moving, None] * GRAVITY).sum(axis=0)
            tau[i] = -float(moment @ axis)
        return tau

    def potential(self, q, payload_mass=0., payload_com=(0., 0., 0.)):
        _, masses, coms = self.bodies(q, payload_mass, payload_com)
        return float(-(masses * (coms @ GRAVITY)).sum())


class ArmGravity:
    """Gravity torques for both arms, with optional per-side wrist payload."""

    def __init__(self, urdf_path=None):
        if urdf_path is None:
            from r1d_chess.robot.r1d_config import MODEL_PATH
            urdf_path = MODEL_PATH
        root = ET.parse(str(urdf_path)).getroot()
        self.arms = {side: _ArmModel(root, side) for side in ('left', 'right')}

    def torque(self, q14, payload=None):
        q14 = np.asarray(q14, dtype=float)
        if q14.shape != (14,) or not np.isfinite(q14).all():
            raise ValueError('gravity requires 14 finite physical arm joints')
        payload = payload or {}
        out = []
        for side, q in (('left', q14[:7]), ('right', q14[7:])):
            p = payload.get(side, {})
            out.append(self.arms[side].torque(q, p.get('mass_kg', 0.), p.get('com_xyz_m', (0., 0., 0.))))
        return np.concatenate(out)

    def potential(self, side, q7, payload_mass=0., payload_com=(0., 0., 0.)):
        return self.arms[side].potential(np.asarray(q7, float), payload_mass, payload_com)
