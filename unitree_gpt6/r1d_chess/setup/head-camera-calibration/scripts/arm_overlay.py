#!/usr/bin/env python3
"""Offline: project arm FK (joint frames + Dex1 tip) into a head image with the head calibration. A visual
camera-vs-arm consistency check, not a quantitative acceptance. Needs a stage-2 manifest pose."""
import argparse, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2])); import _common  # noqa
import cv2, numpy as np
from scipy.spatial.transform import Rotation as R
from r1d_chess.robot.r1d_gravity import ArmGravity

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('capture'); p.add_argument('--pose', type=int, default=0); p.add_argument('--yaw-direction', type=float, default=-1)
p.add_argument('--output', required=True); a = p.parse_args()
c = json.loads((_common.ROOT/'config/head_camera_calibration.json').read_text())
K, D = np.array(c['K_left']), np.array(c['D_left']).reshape(4, 1)
m = json.loads((Path(a.capture)/'manifest.json').read_text()); x = m['poses'][a.pose]
def T(t=(0, 0, 0), r=None):
    o = np.eye(4); o[:3, 3] = t
    if r is not None: o[:3, :3] = r
    return o
ref, s = np.array(c['reference_head_q_rad']), np.array(c['joint_scale_pitch_yaw'])
e = ref+(np.array(x['head_q'])-ref)*(1+s); e[1] += c['yaw_backlash_rad']*a.yaw_direction
Tw = (T((-.006, .03155, .255)) @ T(r=R.from_euler('y', e[0]).as_matrix()) @ T((.0010126, -.03155, .1165))
      @ T(r=R.from_euler('z', e[1]).as_matrix()) @ np.array(c['X_head_yaw_to_left_camera']))
Tc = np.linalg.inv(Tw)
img = cv2.imread(str(Path(a.capture)/x['directory']/'0.jpg'))[:, :640].copy()
g = ArmGravity(); q = np.array(x['arm_q'])
for side, qa, col in (('left', q[:7], (0, 255, 255)), ('right', q[7:], (255, 255, 0))):
    F = g.arms[side].frames(qa)
    pts = np.array([f[:3, 3] for f in F]+[(F[-1] @ np.r_[d, 0, 0, 1])[:3] for d in (.0485, .10, .15)])
    P = (Tc @ np.c_[pts, np.ones(len(pts))].T)[:3].T
    uv = cv2.fisheye.projectPoints(P.reshape(-1, 1, 3), np.zeros((3, 1)), np.zeros((3, 1)), K, D)[0].reshape(-1, 2)
    for u, z in zip(uv, P[:, 2]):
        if z > 0: cv2.circle(img, tuple(int(v) for v in u), 4, col, -1)
    for u, w in zip(uv[3:-1], uv[4:]):
        cv2.line(img, tuple(u.astype(int)), tuple(w.astype(int)), col, 1)
cv2.imwrite(a.output, cv2.resize(img, None, fx=2, fy=2))
print('written', a.output, 'camera in waist frame mm', np.round(Tw[:3, 3]*1e3, 1))
