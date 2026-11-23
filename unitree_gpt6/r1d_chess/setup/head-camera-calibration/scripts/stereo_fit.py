#!/usr/bin/env python3
"""Offline: stereo extrinsics from a recorded board video with fixed fisheye intrinsics (stage 1).
Needs <root>/detections.json and <root>/intrinsics_fit.json; writes <root>/stereo_fit.json."""
import argparse, json
from pathlib import Path
import cv2, numpy as np

p = argparse.ArgumentParser(description=__doc__); p.add_argument('root'); a = p.parse_args()
root = Path(a.root)
fit = json.loads((root/'intrinsics_fit.json').read_text()); det = json.loads((root/'detections.json').read_text())
OBJ = cv2.aruco.CharucoBoard((5, 7), .035, .026, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)).getChessboardCorners().astype(np.float64)
KD = {e: (np.array(fit[e]['fisheye']['K']), np.array(fit[e]['fisheye']['D']).reshape(4, 1)) for e in ('left', 'right')}
def mot(x, y):
    c = set(x) & set(y)
    return np.inf if len(c) < 4 else float(np.median([np.hypot(*np.subtract(x[i], y[i])) for i in c]))
idx = [i for i in range(1, len(det)-1) if len(set(det[i]['left']) & set(det[i]['right'])) >= 12 and
       all(mot(det[i][e], det[i+s][e]) <= 1. for e in ('left', 'right') for s in (-1, 1))]
def norm(e, pts):
    return cv2.fisheye.undistortPoints(np.array(pts, float).reshape(-1, 1, 2), *KD[e]).reshape(-1, 2)
def solve(ids):
    o, l, r = [], [], []
    for i in ids:
        c = sorted(set(det[i]['left']) & set(det[i]['right']), key=int)
        o.append(OBJ[[int(x) for x in c]].astype(np.float32))
        l.append(norm('left', [det[i]['left'][x] for x in c]).astype(np.float32))
        r.append(norm('right', [det[i]['right'][x] for x in c]).astype(np.float32))
    I, Z = np.eye(3), np.zeros(5)
    rms, _, _, _, _, R, T, _, _ = cv2.stereoCalibrate(o, l, r, I, Z, I, Z, (640, 480), flags=cv2.CALIB_FIX_INTRINSIC)
    return rms, R, T.ravel()
ids = idx[::max(1, len(idx)//80)]
rms, R, T = solve(ids)
rng = np.random.default_rng(1)
b = [np.linalg.norm(solve(list(rng.choice(ids, len(ids))))[2]) for _ in range(20)]
fx = KD['left'][0][0, 0]
print('views', len(ids), 'rms px', round(rms*fx, 3), 'baseline mm', round(np.linalg.norm(T)*1e3, 2), '+-', round(np.std(b)*1e3, 2))
(root/'stereo_fit.json').write_text(json.dumps({'views': len(ids), 'rms_px_equiv': rms*fx, 'baseline_m': float(np.linalg.norm(T)),
    'baseline_std_m': float(np.std(b)), 'R': R.tolist(), 'T': T.tolist(), 'convention': 'x_right = R x_left + T'}, indent=1))
