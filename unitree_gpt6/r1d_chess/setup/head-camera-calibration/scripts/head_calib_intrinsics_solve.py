#!/usr/bin/env python3
"""Offline: select frames from a recorded board video and fit head stereo intrinsics."""
import json
import sys
from pathlib import Path
import cv2
import numpy as np

SIZE = (640, 480)
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
OBJ = cv2.aruco.CharucoBoard((5, 7), .035, .026, d).getChessboardCorners().astype(np.float64)
MAX_MOTION_PX = 1.0
MIN_CORNERS = 10
GRID = (16, 12)


def arrays(view):
    ids = sorted(view, key=int)
    return OBJ[[int(i) for i in ids]], np.array([view[i] for i in ids], float)


def motion(a, b):
    common = set(a) & set(b)
    return np.inf if len(common) < 4 else float(np.median([np.hypot(*np.subtract(a[i], b[i])) for i in common]))


def candidates(det, eye):
    out = []
    for i in range(1, len(det)-1):
        v = det[i][eye]
        if len(v) >= MIN_CORNERS and max(motion(v, det[i-1][eye]), motion(v, det[i+1][eye])) <= MAX_MOTION_PX:
            out.append(i)
    return out


def tilt_key(view):
    """Coarse board orientation from the homography, to spread tilts."""
    o, p = arrays(view)
    H, _ = cv2.findHomography(o[:, :2], p)
    if H is None:
        return (0, 0)
    H = H/np.linalg.norm(H[:, 0])
    return (int(np.round(H[2, 0]*.5)), int(np.round(H[2, 1]*.5)))


def select(det, eye, idx, count, gap, taken=()):
    """Greedy: new fine image cells first, tilt novelty second, frames kept >= gap apart."""
    covered, tilts, chosen = set(), {}, []
    blocked = list(taken)
    pool = list(idx)
    while pool and len(chosen) < count:
        best, score = None, -1.
        for i in pool[::3]:
            if any(abs(i-j) < gap for j in blocked):
                continue
            pts = arrays(det[i][eye])[1]
            cells = {(int(x/SIZE[0]*GRID[0]), int(y/SIZE[1]*GRID[1])) for x, y in pts}
            s = len(cells-covered)+3./(1+tilts.get(tilt_key(det[i][eye]), 0))+.02*len(pts)
            if s > score:
                best, score, bc = i, s, cells
        if best is None:
            break
        chosen.append(best)
        blocked.append(best)
        covered |= bc
        k = tilt_key(det[best][eye])
        tilts[k] = tilts.get(k, 0)+1
    return chosen, len(covered)


class Fisheye:
    name = 'fisheye'

    @staticmethod
    def fit(views):
        o = [arrays(v)[0].reshape(-1, 1, 3) for v in views]
        p = [arrays(v)[1].reshape(-1, 1, 2) for v in views]
        K = np.array([[215., 0, 319], [0, 215., 240], [0, 0, 1]])
        D = np.zeros((4, 1))
        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS
        rms, K, D, _, _ = cv2.fisheye.calibrate(o, p, SIZE, K, D, flags=flags,
                                                criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_COUNT, 200, 1e-9))
        return rms, K, D

    @staticmethod
    def residual(view, K, D):
        o, p = arrays(view)
        und = cv2.fisheye.undistortPoints(p.reshape(-1, 1, 2), K, D)
        ok, r, t = cv2.solvePnP(o, und, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return None
        proj, _ = cv2.fisheye.projectPoints(o.reshape(-1, 1, 3), r, t, K, D)
        return np.linalg.norm(proj.reshape(-1, 2)-p, axis=1)


class Rational:
    name = 'rational8'

    @staticmethod
    def fit(views):
        o = [arrays(v)[0].astype(np.float32) for v in views]
        p = [arrays(v)[1].astype(np.float32) for v in views]
        K = np.array([[215., 0, 319], [0, 215., 240], [0, 0, 1]])
        flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_RATIONAL_MODEL
        rms, K, D, _, _ = cv2.calibrateCamera(o, p, SIZE, K, np.zeros(8), flags=flags)
        return rms, K, D

    @staticmethod
    def residual(view, K, D):
        o, p = arrays(view)
        ok, r, t = cv2.solvePnP(o, p, K, D, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return None
        ok, r, t = cv2.solvePnP(o, p, K, D, r, t, True, cv2.SOLVEPNP_ITERATIVE)
        return np.linalg.norm(cv2.projectPoints(o, r, t, K, D)[0].reshape(-1, 2)-p, axis=1)


def evaluate(model, views, K, D):
    e = [model.residual(v, K, D) for v in views]
    e = [x for x in e if x is not None]
    flat = np.concatenate(e)
    return {'views': len(e), 'rms_px': float(np.sqrt(np.mean(flat**2))), 'p90_px': float(np.percentile(flat, 90)),
            'max_px': float(flat.max()), 'per_view_rms': [float(np.sqrt(np.mean(x**2))) for x in e]}


def fit_clean(model, views):
    for _ in range(3):
        rms, K, D = model.fit(views)
        res = evaluate(model, views, K, D)['per_view_rms']
        keep = [v for v, r in zip(views, res) if r <= max(1., 2.5*np.median(res))]
        if len(keep) == len(views):
            break
        views = keep
    return views, rms, K, D


def summary(K, D):
    return {'fx': K[0, 0], 'fy': K[1, 1], 'cx': K[0, 2], 'cy': K[1, 2], 'fx_fy_ratio': K[0, 0]/K[1, 1],
            'K': K.tolist(), 'D': np.asarray(D).ravel().tolist()}


def main():
    root = Path(sys.argv[1])
    det = json.loads((root/'detections.json').read_text())
    rng = np.random.default_rng(0)
    result = {'board': 'ChArUco 5x7, 35/26 mm, DICT_5X5_100', 'max_motion_px_per_frame': MAX_MOTION_PX}
    chosen = {}
    for eye in ('left', 'right'):
        idx = candidates(det, eye)
        train, cells = select(det, eye, idx, 60, 10)
        hold, _ = select(det, eye, idx, 24, 15, taken=train)
        pts = np.vstack([arrays(det[i][eye])[1] for i in train])
        r = np.hypot(pts[:, 0]-319, pts[:, 1]-240)
        info = {'candidates': len(idx), 'train': len(train), 'holdout': len(hold), 'fine_cells': cells,
                'cells_total': GRID[0]*GRID[1], 'max_radius_px': float(r.max()), 'radius_p95_px': float(np.percentile(r, 95))}
        tv, hv = [det[i][eye] for i in train], [det[i][eye] for i in hold]
        for model in (Fisheye, Rational):
            try:
                views, rms, K, D = fit_clean(model, tv)
                h = evaluate(model, hv, K, D)
                boot = []
                for _ in range(20):
                    s = [views[j] for j in rng.integers(0, len(views), len(views))]
                    try:
                        boot.append(model.fit(s)[1])
                    except cv2.error:
                        pass
                b = np.array([[k[0, 0], k[1, 1], k[0, 2], k[1, 2]] for k in boot])
                info[model.name] = {**summary(K, D), 'train_views': len(views), 'train_rms_px': rms,
                                    'holdout': {k: v for k, v in h.items() if k != 'per_view_rms'},
                                    'bootstrap_std': b.std(0).tolist(), 'bootstrap_n': len(boot)}
            except cv2.error as e:
                info[model.name] = {'error': str(e)[:200]}
        result[eye] = info
        chosen[eye] = {'train': train, 'holdout': hold}
    (root/'intrinsics_fit.json').write_text(json.dumps(result, indent=1))
    (root/'selected_frames.json').write_text(json.dumps(chosen))
    for eye in ('left', 'right'):
        i = result[eye]
        print(eye, {k: i[k] for k in ('candidates', 'train', 'holdout', 'fine_cells', 'cells_total', 'max_radius_px', 'radius_p95_px')})
        for m in ('fisheye', 'rational8'):
            x = i[m]
            if 'error' in x:
                print(' ', m, x)
                continue
            print(f"  {m}: fx {x['fx']:.2f} fy {x['fy']:.2f} ratio {x['fx_fy_ratio']:.4f} cx {x['cx']:.2f} cy {x['cy']:.2f} train {x['train_rms_px']:.3f}px ({x['train_views']}v) hold rms {x['holdout']['rms_px']:.3f} p90 {x['holdout']['p90_px']:.3f} max {x['holdout']['max_px']:.2f} ({x['holdout']['views']}v) boot std {np.round(x['bootstrap_std'], 2).tolist()}")
            print('    D', np.round(x['D'], 4).tolist())


main()
