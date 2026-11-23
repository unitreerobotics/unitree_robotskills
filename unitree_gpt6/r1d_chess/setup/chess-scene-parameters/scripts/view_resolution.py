#!/usr/bin/env python3
"""Offline: how well the board is seen in a head frame: board pixel extent, cell size near/mid/far, piece
radius, sharpness, plus an overlay of the 90 intersections. Use to compare head reference poses."""
import argparse, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2])); import _common  # noqa
import cv2, numpy as np
from scipy.spatial.transform import Rotation as R

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--image', required=True); p.add_argument('--head-q', type=float, nargs=2, required=True)
p.add_argument('--yaw-direction', type=float, default=-1); p.add_argument('--overlay', required=True); a = p.parse_args()
C = json.loads((_common.ROOT/'config/head_camera_calibration.json').read_text())
G = json.loads((_common.ROOT/'config/board_geometry.json').read_text())
K, D = np.array(C['K_left']), np.array(C['D_left']).reshape(4, 1)
def T(t=(0, 0, 0), r=None):
    o = np.eye(4); o[:3, 3] = t
    if r is not None: o[:3, :3] = r
    return o
ref, s = np.array(C['reference_head_q_rad']), np.array(C['joint_scale_pitch_yaw'])
e = ref+(np.array(a.head_q)-ref)*(1+s); e[1] += C['yaw_backlash_rad']*a.yaw_direction
Tc = np.linalg.inv(T((-.006, .03155, .255)) @ T(r=R.from_euler('y', e[0]).as_matrix()) @ T((.0010126, -.03155, .1165))
                   @ T(r=R.from_euler('z', e[1]).as_matrix()) @ np.array(C['X_head_yaw_to_left_camera']))
z = G['surface_z_m']; P = G['intersections_xy_m']
def proj(xy, h=z):
    c = (Tc @ np.r_[xy, h, 1])[:3]
    return cv2.fisheye.projectPoints(c.reshape(1, 1, 3), np.zeros((3, 1)), np.zeros((3, 1)), K, D)[0].ravel(), c[2]
img = cv2.imread(a.image)[:, :640].copy()
uv = np.array([proj(np.array(xy))[0] for xy in P.values()])
for u in uv: cv2.circle(img, tuple(int(x) for x in u), 2, (0, 255, 0), -1)
cv2.imwrite(a.overlay, img)
inside = bool((uv[:, 0] > 40).all() and (uv[:, 0] < 598).all() and (uv[:, 1] > 2).all() and (uv[:, 1] < 478).all())
print('board pixel extent', uv.min(0).round(0), uv.max(0).round(0), 'all in valid view', inside)
for sq, up, side in (('e0', 'e1', 'f0'), ('e4', 'e5', 'f4'), ('e9', 'e8', 'f9')):
    o, d = proj(np.array(P[sq]))
    print(f'{sq}: rank {np.linalg.norm(o-proj(np.array(P[up]))[0]):.1f} px  file {np.linalg.norm(o-proj(np.array(P[side]))[0]):.1f} px  depth {d:.2f} m'
          f'  piece radius {np.linalg.norm(proj(np.array(P[sq]), z+G["piece"]["height_m"])[0]-proj(np.array(P[sq])+[0, G["piece"]["diameter_m"]/2], z+G["piece"]["height_m"])[0]):.1f} px')
g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY); x0, y0 = np.maximum(uv.min(0).astype(int), 0); x1, y1 = np.minimum(uv.max(0).astype(int), [639, 479])
print('sharpness (laplacian var)', round(float(cv2.Laplacian(g[y0:y1, x0:x1], cv2.CV_64F).var()), 1))
