#!/usr/bin/env python3
"""Offline: measure the xiangqi board in the waist frame from one head stereo frame. Read-only.

Plane sweep: both eyes are warped to a top-down view of plane z=h; the true surface maximises their
agreement. A grid of known pitch is then fitted to the top-down line image.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

from r1d_chess.paths import ROOT
PX = .001
X0, X1, Y0, Y1 = .03, .50, -.27, .27


def T(t=(0, 0, 0), r=None):
    m = np.eye(4)
    m[:3, 3] = t
    if r is not None:
        m[:3, :3] = r
    return m


def camera_pose(c, q, direction):
    ref, s = np.array(c['reference_head_q_rad']), np.array(c['joint_scale_pitch_yaw'])
    e = ref+(np.array(q)-ref)*(1+s)
    e[1] += c['yaw_backlash_rad']*direction
    head = (T((-.006, .03155, .255)) @ T(r=R.from_euler('y', e[0]).as_matrix()) @ T((.0010126, -.03155, .1165))
            @ T(r=R.from_euler('z', e[1]).as_matrix()))
    return head @ np.array(c['X_head_yaw_to_left_camera'])


def warp(image, K, D, Tcw, h):
    xs, ys = np.arange(X0, X1, PX), np.arange(Y1, Y0, -PX)
    gx, gy = np.meshgrid(xs, ys)
    P = np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, h), np.ones(gx.size)]
    C = (Tcw @ P.T)[:3].T
    uv, _ = cv2.fisheye.projectPoints(C.reshape(-1, 1, 3), np.zeros((3, 1)), np.zeros((3, 1)), K, D)
    uv = uv.reshape(gx.shape+(2,)).astype(np.float32)
    return cv2.remap(image, uv[..., 0], uv[..., 1], cv2.INTER_LINEAR)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--image', required=True)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument('--snapshot', help='json with the full joint array q (35)')
    src.add_argument('--head-q', type=float, nargs=2, metavar=('PITCH', 'YAW'), help='head joint angles at the image')
    p.add_argument('--yaw-direction', type=float, required=True, help='sign of the last head yaw move')
    p.add_argument('--output', required=True)
    p.add_argument('--prior', help='board_geometry.json: search within half a cell of its centre (the grid is periodic)')
    a = p.parse_args()
    c = json.loads((ROOT/'config/head_camera_calibration.json').read_text())
    q = json.loads(Path(a.snapshot).read_text())['q'] if a.snapshot else [0.]*29+list(a.head_q)
    img = cv2.imread(a.image, cv2.IMREAD_GRAYSCALE)
    L, Rr = img[:, :640], img[:, 640:]
    KL, DL = np.array(c['K_left']), np.array(c['D_left']).reshape(4, 1)
    KR, DR = np.array(c['K_right']), np.array(c['D_right']).reshape(4, 1)
    Tw = camera_pose(c, q[29:31], a.yaw_direction)
    TL = np.linalg.inv(Tw)
    TR = np.array(c['T_right_from_left']) @ TL
    roi = (slice(int((Y1-.2)/PX), int((Y1+.2)/PX)), slice(int((.12-X0)/PX), int((.45-X0)/PX)))
    scores = []
    for h in np.arange(-.04, .06, .001):
        a1, b1 = warp(L, KL, DL, TL, h)[roi].astype(float), warp(Rr, KR, DR, TR, h)[roi].astype(float)
        m = (a1 > 5) & (b1 > 5)
        scores.append((h, float(np.corrcoef(a1[m], b1[m])[0, 1])))
    s = np.array(scores)
    i = int(np.argmax(s[:, 1]))
    co = np.polyfit(s[max(0, i-3):i+4, 0], s[max(0, i-3):i+4, 1], 2)
    h = float(-co[1]/(2*co[0]))
    print('plane sweep best z mm', round(h*1e3, 2), 'ncc', round(float(s[i, 1]), 3), 'ncc at +-10mm', s[max(0, i-10), 1].round(3), s[min(len(s)-1, i+10), 1].round(3))
    top = warp(L, KL, DL, TL, h)
    topr = warp(Rr, KR, DR, TR, h)
    avg = ((top.astype(float)+topr)/2)
    cv2.imwrite(str(Path(a.output).with_suffix('.jpg')), np.hstack((top, topr)))
    # Grid lines are darker than the surrounding wood. Thin dark-line response:
    line = np.clip(cv2.GaussianBlur(avg, (0, 0), 3)-cv2.GaussianBlur(avg, (0, 0), 1), 0, 40)
    line = cv2.GaussianBlur(line, (0, 0), 1.2)
    lines_x = np.arange(10)*.0337          # ranks along waist x
    lines_y = np.arange(9)*.0356           # files along waist y
    def rank_line(cx, cy, th, u):
        ca, sa = np.cos(th), np.sin(th)
        w = np.arange(-.1424, .1424, .002)
        return np.median(sample(cx+u*ca-w*sa, cy+u*sa+w*ca))

    def file_line(cx, cy, th, w):
        ca, sa = np.cos(th), np.sin(th)
        u = np.r_[np.arange(-.1517, -.0169, .002), np.arange(.0169, .1517, .002)]
        return np.median(sample(cx+u*ca-w*sa, cy+u*sa+w*ca))

    def score(v):
        # Medians ignore pieces; lines one pitch outside the grid must be empty, which breaks periodicity.
        cx, cy, th = v
        inside = [rank_line(cx, cy, th, u) for u in lines_x-.1517]+[file_line(cx, cy, th, w) for w in lines_y-.1424]
        outside = [rank_line(cx, cy, th, u) for u in (-.1517-.0337, .1517+.0337)]+[file_line(cx, cy, th, w) for w in (-.1424-.0356, .1424+.0356)]
        return -(float(np.mean(inside))-float(np.mean(outside)))
    def sample(x, y):
        col = (x-X0)/PX
        row = (Y1-y)/PX
        return cv2.remap(line.astype(np.float32), col.astype(np.float32).reshape(-1, 1), row.astype(np.float32).reshape(-1, 1), cv2.INTER_LINEAR).ravel()
    best = None
    if a.prior:
        g = json.loads(Path(a.prior).read_text())
        px0, py0 = g['centre_xy_m']
        xs, ys = np.arange(px0-.016, px0+.0161, .002), np.arange(py0-.017, py0+.0171, .002)
    else:
        # No prior: coarse search over the whole reachable table area; check the overlay for a one-cell shift.
        xs, ys = np.arange(.15, .45, .004), np.arange(-.15, .15, .004)
    for cx in xs:
        for cy in ys:
            for th in np.arange(-.06, .061, .01):
                f = score((cx, cy, th))
                if best is None or f < best[0]:
                    best = (f, cx, cy, th)
    x0 = np.array(best[1:])
    simplex = np.vstack([x0, x0+[.002, 0, 0], x0+[0, .002, 0], x0+[0, 0, .006]])
    r = minimize(score, x0, method='Nelder-Mead', options={'xatol': 1e-4, 'fatol': 1e-4, 'initial_simplex': simplex})
    cx, cy, th = r.x
    print('grid fit centre mm', round(cx*1e3, 1), round(cy*1e3, 1), 'yaw mrad', round(th*1e3, 1), 'score', round(-r.fun, 2), 'bg', round(float(line.mean()), 2))
    v = cv2.cvtColor(top, cv2.COLOR_GRAY2BGR)
    ca, sa = np.cos(th), np.sin(th)
    def px(u, w):
        return (int((cx+u*ca-w*sa-X0)/PX), int((Y1-(cy+u*sa+w*ca))/PX))
    for u in lines_x-.1517:
        cv2.line(v, px(u, -.1424), px(u, .1424), (0, 255, 0), 1)
    for w in lines_y-.1424:
        cv2.line(v, px(-.1517, w), px(.1517, w), (0, 255, 0), 1)
    cv2.imwrite(str(Path(a.output).with_name(Path(a.output).stem+'_grid.jpg')), v)
    Path(a.output).write_text(json.dumps({'surface_z_m': h, 'ncc': float(s[i, 1]), 'sweep': s.tolist(), 'grid_centre_xy_m': [float(cx), float(cy)],
                                          'grid_yaw_rad': float(th), 'cell_m': [.0356, .0337], 'grid_score': float(-r.fun), 'line_background': float(line.mean()),
                                          'camera_waist_m': Tw[:3, 3].tolist()}, indent=1))


if __name__ == '__main__':
    main()
