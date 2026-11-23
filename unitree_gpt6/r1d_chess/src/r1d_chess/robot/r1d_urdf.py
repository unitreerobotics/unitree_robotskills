#!/usr/bin/env python3
"""URDF FK / bounded 7-DoF IK verification. Never connects to DDS or sends commands.
Requires numpy/scipy. Root frame: waist_yaw_link; TCP: wrist_yaw_link + 0.05 m X
(example TCP only; measure/calibrate the installed tool before using on hardware).
"""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

def rotation_matrix(rotation):
    return rotation.as_matrix() if hasattr(rotation, 'as_matrix') else rotation.as_dcm()


def rotation_from_matrix(matrix):
    return (Rotation.from_matrix(matrix) if hasattr(Rotation, 'from_matrix')
            else Rotation.from_dcm(matrix))


# Primary IK: analytic-Jacobian damped least squares (scipy least_squares was
# 98% of a 13-minute preflight). Tolerances are far tighter than the acceptance (1e-4 m, 1e-3 rad).
DLS_POSITION_M = 1e-7
DLS_ROTATION_RAD = 1e-6
DLS_DAMPING = 1e-4
DLS_MAX_ITERATIONS = 60

SUFFIXES = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
            'wrist_roll', 'wrist_pitch', 'wrist_yaw')


def xyz(element, key):
    return np.fromstring(element.get(key, '0 0 0'), sep=' ')


class Arm:
    def __init__(self, root, side):
        self.names = [side + '_' + suffix + '_joint' for suffix in SUFFIXES]
        joints = {j.get('name'): j for j in root.findall('joint')}
        self.origins, self.axes, self.lower, self.upper = [], [], [], []
        parent = 'waist_yaw_link'
        for name in self.names:
            j = joints[name]
            if j.find('parent').get('link') != parent or j.get('type') != 'revolute':
                raise ValueError('Unexpected URDF chain at ' + name)
            parent = j.find('child').get('link')
            origin = j.find('origin')
            t = np.eye(4)
            t[:3, :3] = rotation_matrix(Rotation.from_euler('xyz', xyz(origin, 'rpy')))
            t[:3, 3] = xyz(origin, 'xyz')
            self.origins.append(t)
            self.axes.append(xyz(j.find('axis'), 'xyz'))
            limit = j.find('limit')
            self.lower.append(float(limit.get('lower')))
            self.upper.append(float(limit.get('upper')))
        self.lower, self.upper = np.array(self.lower), np.array(self.upper)
        self.tcp = np.eye(4)
        self.tcp[0, 3] = 0.05

    def fk(self, q):
        t = np.eye(4)
        for origin, axis, angle in zip(self.origins, self.axes, q):
            r = np.eye(4)
            r[:3, :3] = rotation_matrix(Rotation.from_rotvec(axis * angle))
            t = t @ origin @ r
        return t @ self.tcp

    def frames(self, q):
        """TCP transform plus each joint's world axis and origin (for the analytic Jacobian)."""
        t, axes, origins = np.eye(4), [], []
        for origin, axis, angle in zip(self.origins, self.axes, q):
            t = t @ origin
            axes.append(t[:3, :3] @ axis)
            origins.append(t[:3, 3].copy())
            r = np.eye(4)
            r[:3, :3] = rotation_matrix(Rotation.from_rotvec(axis * angle))
            t = t @ r
        return t @ self.tcp, np.array(axes), np.array(origins)

    def solve_dls(self, target, seed):
        """Bounded damped least squares with the analytic Jacobian.

        Primary IK. Returns as soon as the error is below
        DLS_POSITION_M / DLS_ROTATION_RAD, so a stationary arm returns its seed
        immediately; the minimum-norm step keeps the solution next to the seed.
        Returns (q, iterations, converged); the caller falls back to scipy."""
        q = np.clip(seed, self.lower, self.upper)
        for iteration in range(DLS_MAX_ITERATIONS):
            t, axes, origins = self.frames(q)
            position = target[:3, 3] - t[:3, 3]
            rotation = rotation_from_matrix(target[:3, :3] @ t[:3, :3].T).as_rotvec()
            if np.linalg.norm(position) < DLS_POSITION_M and np.linalg.norm(rotation) < DLS_ROTATION_RAD:
                return q, iteration, True
            jacobian = np.vstack([np.cross(axes, t[:3, 3] - origins).T, axes.T])
            step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + DLS_DAMPING*np.eye(6),
                                                np.r_[position, rotation])
            q = np.clip(q + step, self.lower, self.upper)
        return q, DLS_MAX_ITERATIONS, False

    def error(self, q, target):
        t = self.fk(q)
        return np.r_[t[:3, 3] - target[:3, 3],
                     rotation_from_matrix(target[:3, :3].T @ t[:3, :3]).as_rotvec()]

    def solve(self, target, seed):
        if not np.all(np.isfinite(target)) or target.shape != (4, 4):
            raise ValueError('Target must be a finite SE(3) 4x4 matrix')
        r = target[:3, :3]
        if (not np.allclose(r.T @ r, np.eye(3), atol=1e-7)
                or not np.isclose(np.linalg.det(r), 1., atol=1e-7)
                or not np.allclose(target[3], [0, 0, 0, 1])):
            raise ValueError('Invalid rigid transform')
        seed = np.asarray(seed, dtype=float)
        if seed.shape != (7,) or not np.all(np.isfinite(seed)):
            raise ValueError('Seed must contain 7 finite angles')
        q, iterations, converged = self.solve_dls(target, seed)
        if converged:
            error = self.error(q, target)
            pos, rot = float(np.linalg.norm(error[:3])), float(np.linalg.norm(error[3:]))
            if pos < 1e-4 and rot < 1e-3:
                return {'accepted': True, 'q_rad': q.tolist(), 'position_error_m': pos,
                        'rotation_error_rad': rot, 'evaluations': iterations, 'method': 'dls',
                        'max_delta_from_seed_rad': float(np.max(np.abs(q-seed)))}
        # Fallback: the original bounded scipy solver (unchanged acceptance).
        result = least_squares(lambda q: self.error(q, target),
                               np.clip(seed, self.lower + 1e-8, self.upper - 1e-8),
                               bounds=(self.lower, self.upper), max_nfev=400,
                               ftol=1e-11, xtol=1e-11, gtol=1e-11)
        error = self.error(result.x, target)
        pos, rot = float(np.linalg.norm(error[:3])), float(np.linalg.norm(error[3:]))
        evaluations = result.nfev
        method = 'trf'
        if not (pos < 1e-4 and rot < 1e-3):
            # The redundant 7-DoF chain can stall with TRF near a bound. Try a
            # second bounded algorithm from that solution; acceptance, limits
            # and the caller's interpolation/branch checks remain unchanged.
            alternate = least_squares(lambda q: self.error(q, target), result.x,
                                      bounds=(self.lower, self.upper), method='dogbox',
                                      max_nfev=400, ftol=1e-11, xtol=1e-11, gtol=1e-11)
            evaluations += alternate.nfev
            other_error = self.error(alternate.x, target)
            other_pos = float(np.linalg.norm(other_error[:3]))
            other_rot = float(np.linalg.norm(other_error[3:]))
            if other_pos < 1e-4 and other_rot < 1e-3:
                result, pos, rot = alternate, other_pos, other_rot
                method = 'trf_then_dogbox'
        return {'accepted': bool(pos < 1e-4 and rot < 1e-3),
                'q_rad': result.x.tolist(), 'position_error_m': pos,
                'rotation_error_rad': rot, 'evaluations': evaluations, 'method': method,
                'max_delta_from_seed_rad': float(np.max(np.abs(result.x-seed)))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--state', type=Path, required=True,
                        help='JSON line emitted by r1_state_probe.cpp')
    args = parser.parse_args()
    state = next(json.loads(line) for line in args.state.read_text().splitlines()
                 if line.startswith('{'))
    if state['valid_samples'] < 1 or state['crc_errors']:
        raise ValueError('No valid CRC-checked state')
    root = ET.parse(args.urdf).getroot()
    rng = np.random.default_rng(20261015)
    output = {'mode': 'offline_only_no_motor_commands', 'frame': 'waist_yaw_link',
              'tcp_offset_m': [0.05, 0, 0], 'arms': {}}
    ok = True
    for side, start in [('left', 15), ('right', 22)]:
        arm = Arm(root, side)
        motors = state['motors'][start:start+7]
        if any(m['mode'] != 1 or m['vol'] <= 0 for m in motors):
            raise ValueError('Missing arm motor feedback')
        q = np.array([m['q'] for m in motors])
        if np.any(q < arm.lower) or np.any(q > arm.upper):
            raise ValueError('Measured angle outside URDF limits')
        current = arm.fk(q)
        small = current.copy()
        small[0, 3] += 0.005
        reachable = []
        for _ in range(10):
            known = np.clip(q + rng.uniform(-0.08, 0.08, 7),
                            arm.lower + 1e-5, arm.upper - 1e-5)
            reachable.append(arm.solve(arm.fk(known), q))
        impossible = current.copy()
        impossible[0, 3] += 5.0
        unreachable = arm.solve(impossible, q)
        small_solution = arm.solve(small, q)
        passed = all(s['accepted'] for s in reachable) and small_solution['accepted'] and not unreachable['accepted']
        ok &= passed
        output['arms'][side] = {'dds_indices': list(range(start, start+7)),
            'joint_names': arm.names, 'measured_q_rad': q.tolist(),
            'current_tcp_transform': current.tolist(), 'target_5mm_x': small.tolist(),
            'solution_5mm_x': small_solution, 'reachable_trials': len(reachable),
            'reachable_passed': sum(s['accepted'] for s in reachable),
            'max_trial_position_error_m': max(s['position_error_m'] for s in reachable),
            'max_trial_rotation_error_rad': max(s['rotation_error_rad'] for s in reachable),
            'unreachable_5m_rejected': not unreachable['accepted'], 'passed': passed}
    output['passed'] = bool(ok)
    print(json.dumps(output, indent=2))
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
