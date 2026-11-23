"""Intrinsics + hand-eye (Link7 <- camera) + board plane from partial checkerboard views.
usage: python3 -m calibration.solve OUT.json FRAME_PREFIX...   (each prefix has .jpg and .json with measured 'q')"""
import json, sys, cv2, numpy as np
from scipy.optimize import least_squares
from robot.rt import *
np.set_printoptions(precision=4, suppress=True, linewidth=160)
out, names = sys.argv[1], sys.argv[2:]

def rt2T(r, t): return T(cv2.Rodrigues(np.asarray(r, float))[0], np.asarray(t, float).ravel())
def T2rt(M): return np.r_[cv2.Rodrigues(M[:3, :3])[0].ravel(), M[:3, 3]]

# 1. detect ---------------------------------------------------------------
F = []
for n in names:
    g = cv2.imread(n + '.jpg', 0); d = corners(g)
    if d is None or len(d[0]) < 16: continue
    meta = json.load(open(n + '.json'))
    F.append(dict(name=n, gray=g, px=d[0], loc=d[1], q=np.array(meta.get('q', meta.get('q_after'))), Tw7=fk(meta.get('q', meta.get('q_after')))[0]))
size = F[0]['gray'].shape[::-1]
print(len(F), 'of', len(names), 'frames with a grid;', sum(len(f['px']) for f in F), 'corners; image', size)

# 2. intrinsics from local grids (independent of the arm) -------------------
objs = [np.c_[f['loc'] * SQUARE, np.zeros(len(f['loc']))].astype(np.float32) for f in F]
imgs = [f['px'].astype(np.float32) for f in F]
best = None
for label, flags in (('pinhole k1k2p1p2k3', 0), ('rational k1..k6', cv2.CALIB_RATIONAL_MODEL)):
    rms, K, dist, rv, tv = cv2.calibrateCamera(objs, imgs, size, None, None, flags=flags)
    print('%-20s rms %.3f px  f %.1f %.1f  c %.1f %.1f  dist %s' % (label, rms, K[0, 0], K[1, 1], K[0, 2], K[1, 2], dist.ravel().round(3)))
    if best is None or rms < 0.8 * best[0]: best = (rms, K, dist, rv, tv, label)      # take rational only if clearly better
rms_intr, K, dist, rv, tv, model = best
for f, r, t in zip(F, rv, tv): f['Tcg'] = rt2T(r, t)                                   # camera <- local grid
print('using', model, ' HFOV %.0f deg' % np.degrees(2 * np.arctan(size[0] / 2 / K[0, 0])))

# 3. put every local grid into one board frame -----------------------------
def parity(f):
    """Sign of (diagonal-square minus anti-diagonal-square brightness)*(-1)^(i+j): identical for every correct frame."""
    P = {tuple(l.astype(int)): p for l, p in zip(f['loc'], f['px'])}; B = {tuple(l.astype(int)): b for l, b in zip(f['loc'], f['brd'])}
    votes = []
    for (a, b), p in P.items():
        if (a + 1, b) in P and (a, b + 1) in P:
            e1, e2 = P[a + 1, b] - p, P[a, b + 1] - p
            s = float(cv2.getRectSubPix(f['gray'], (3, 3), tuple((p + .35 * (e1 + e2)).astype(np.float32)))[1, 1]) - \
                float(cv2.getRectSubPix(f['gray'], (3, 3), tuple((p + .35 * (e1 - e2)).astype(np.float32)))[1, 1])
            d = (B[a + 1, b] - B[a, b]) + (B[a, b + 1] - B[a, b])                       # board direction of local (1,1)
            votes.append(s * (1 if d[0] * d[1] > 0 else -1) * (-1) ** int(B[a, b].sum()))
    return float(np.sign(np.median(votes))), float(np.mean(np.sign(votes) == np.sign(np.median(votes))))

def project(f, X, Twb, brd):
    Tcb = np.linalg.inv(f['Tw7'] @ X) @ Twb
    obj = np.c_[np.asarray(brd, float) * SQUARE, np.zeros(len(brd))]
    z = (Tcb[:3, :3] @ obj.T).T[:, 2] + Tcb[2, 3]
    uv = cv2.projectPoints(obj, cv2.Rodrigues(Tcb[:3, :3])[0], Tcb[:3, 3], K, dist)[0].reshape(-1, 2)
    uv[z < 0.03] = 1e6                                                                 # behind the camera
    return uv

LAT = np.array([(i, j) for i in range(-5, 16) for j in range(-5, 16)], float)           # board lattice incl. margin
def assign(X, Twb, sigma, verbose=False):
    """Map each frame's local grid onto the common board lattice: nearest predicted lattice point per corner,
    snapped to one integer rigid map, with the square colours required to match (rules out one-square slips)."""
    from scipy.spatial import cKDTree
    for f in F:
        uv = project(f, X, Twb, LAT); _, nn = cKDTree(uv).query(f['px'])
        P = np.linalg.lstsq(np.c_[f['loc'], np.ones(len(f['loc']))], LAT[nn], rcond=None)[0]
        A = np.round(P[:2].T)
        f['score'] = 9.0
        if not (np.abs(A).sum(0) == 1).all() or abs(np.linalg.det(A)) != 1:
            if verbose: print('  %-8s no consistent lattice map' % f['name'])
            continue
        t0 = np.round(np.median(LAT[nn] - f['loc'] @ A.T, 0)); cand = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                f['brd'] = f['loc'] @ A.T + t0 + (dx, dy)
                if parity(f)[0] != sigma: continue
                cand.append((float(np.sqrt(((project(f, X, Twb, f['brd']) - f['px']) ** 2).sum(1).mean())), dx, dy))
        cand.sort(); e, dx, dy = cand[0]
        f['brd'] = f['loc'] @ A.T + t0 + (dx, dy)
        f['score'] = e / f['spacing']; f['margin'] = cand[1][0] / max(e, 1e-9)
        if verbose: print('  %-8s %3d corners  err %5.1f px = %.2f of corner spacing; next-best hypothesis %.1fx worse' % (f['name'], len(f['px']), e, f['score'], f['margin']))

from scipy.spatial import cKDTree
for f in F: f['spacing'] = float(np.median(cKDTree(f['px']).query(f['px'], 2)[0][:, 1]))
ref = max(F, key=lambda f: len(f['px']))
X = L7_CAM_NOMINAL.copy(); Twb = ref['Tw7'] @ X @ ref['Tcg']                            # board frame := ref's local grid
ref['brd'] = ref['loc'].copy(); sigma = parity(ref)[0]
print('reference frame', ref['name'], '; assignment with nominal hand-eye:'); assign(X, Twb, sigma, True)

DELTA = np.zeros(7)                                                                     # joint zero offsets: FK uses q + DELTA
def set_delta(d):
    DELTA[1:6] = d                                                                      # J1 is absorbed by the board pose, J7 by the hand-eye
    for f in F: f['Tw7'] = fk(f['q'] + DELTA)[0]

def residuals(x, frames, with_delta=False):
    X, Twb = rt2T(x[:3], x[3:6]), rt2T(x[6:9], x[9:12])
    if with_delta: set_delta(x[12:17])
    return np.concatenate([(project(f, X, Twb, f['brd']) - f['px']).ravel() for f in frames])

def rms_of(frames): return float(np.sqrt((residuals(np.r_[T2rt(X), T2rt(Twb)], frames).reshape(-1, 2) ** 2).sum(1).mean()))

for stage in (False, True):                                                             # 1: nominal joint zeros, 2: with joint zero offsets
    for it in range(8):                                                                 # refine on the unambiguous frames, re-assign, repeat
        good = [f for f in F if f['score'] < 0.3]
        x0 = np.r_[T2rt(X), T2rt(Twb), DELTA[1:6]] if stage else np.r_[T2rt(X), T2rt(Twb)]
        sol = least_squares(residuals, x0, args=(good, stage), loss='soft_l1', f_scale=3.0)
        X, Twb = rt2T(sol.x[:3], sol.x[3:6]), rt2T(sol.x[6:9], sol.x[9:12])
        if stage: set_delta(sol.x[12:17])
        before = [f.get('brd', np.zeros(0)).copy() for f in F]; assign(X, Twb, sigma)
        same = all(a.shape == f['brd'].shape and np.array_equal(a, f['brd']) for a, f in zip(before, F))
        if same and len(good) == len([f for f in F if f['score'] < 0.3]): break
    good = [f for f in F if f['score'] < 0.3]
    print('%s: %d/%d frames unambiguous, chain rms %.2f px, board normal %s, joint offsets (rad) %s' % (
        'with joint zero offsets' if stage else 'nominal joint zeros', len(good), len(F), rms_of(good), Twb[:3, 2].round(4), DELTA.round(4)))
print('final assignment:'); assign(X, Twb, sigma, True)
bad = [f['name'] for f in F if f['score'] >= 0.3]
if bad: print('DROPPING ambiguous frames:', bad)
F = [f for f in F if f['score'] < 0.3]
sol = least_squares(residuals, np.r_[T2rt(X), T2rt(Twb), DELTA[1:6]], args=(F, True))  # final plain least squares
X, Twb = rt2T(sol.x[:3], sol.x[3:6]), rt2T(sol.x[6:9], sol.x[9:12]); set_delta(sol.x[12:17])
J = sol.jac; cov = np.linalg.inv(J.T @ J) * (sol.fun ** 2).sum() / (len(sol.fun) - len(sol.x))
print('joint zero offsets J1..J7 (rad):', DELTA.round(4), ' 1-sigma J2..J6:', np.sqrt(np.diag(cov))[12:17].round(4))
allb = np.vstack([f['brd'] for f in F]); span = allb.max(0) - allb.min(0) + 1
print('board corners used: %d x %d (physical board is 11 x 8)' % tuple(span)); assert sorted(span) <= [8, 11]

# 4. report ------------------------------------------------------------------
res = residuals(np.r_[T2rt(X), T2rt(Twb)], F).reshape(-1, 2); k = 0; per = []
for f in F:
    e = res[k:k + len(f['px'])]; k += len(f['px']); per.append(float(np.sqrt((e ** 2).sum(1).mean())))
    Tcb = np.linalg.inv(f['Tw7'] @ X) @ Twb; f['dist'] = float(np.linalg.norm(Tcb[:3, 3]))
    brd = np.c_[f['brd'] * SQUARE, np.zeros(len(f['brd']))]
    _, r, t = cv2.solvePnP(brd, f['px'], K, dist, cv2.Rodrigues(Tcb[:3, :3])[0], Tcb[:3, 3].reshape(3, 1).copy(), useExtrinsicGuess=True)
    seen = (f['Tw7'] @ X @ rt2T(r, t) @ np.c_[brd, np.ones(len(brd))].T).T[:, :3]       # where this single view puts its corners in world
    f['mm'] = 1000 * (seen - (Twb @ np.c_[brd, np.ones(len(brd))].T).T[:, :3]).mean(0)
E = np.array([f['mm'] for f in F])
print('chain reprojection rms per frame (px):', np.round(per, 1))
print('overall %.2f px (camera-only %.2f px)' % (np.sqrt((res ** 2).sum(1).mean()), rms_intr))
print('per-frame board position error through arm+hand-eye, world mm: rms xyz %s, max |.| %s' % (np.sqrt((E ** 2).mean(0)).round(1), np.abs(E).max(0).round(1)))
print('Link7 <- camera:\n', X, '\nnominal CAD:\n', L7_CAM_NOMINAL)
dR = cv2.Rodrigues(L7_CAM_NOMINAL[:3, :3].T @ X[:3, :3])[0].ravel()
print('vs nominal: rotation %.2f deg, translation %s mm' % (np.degrees(np.linalg.norm(dR)), ((X[:3, 3] - L7_CAM_NOMINAL[:3, 3]) * 1000).round(1)))
C = (Twb @ np.c_[allb * SQUARE, np.zeros(len(allb)), np.ones(len(allb))].T).T[:, :3]
print('board in world: normal %s, corner z %.4f..%.4f m, xy %s..%s' % (Twb[:3, 2].round(4), C[:, 2].min(), C[:, 2].max(), C[:, :2].min(0).round(3), C[:, :2].max(0).round(3)))
json.dump(dict(image_size=size, model=model, K=K.tolist(), dist=dist.ravel().tolist(), link7_from_camera=X.tolist(), joint_zero_offset=DELTA.tolist(),
               world_from_board=Twb.tolist(), board_top_plane=dict(normal=Twb[:3, 2].tolist(), point=Twb[:3, 3].tolist()),
               board_index_min=allb.min(0).tolist(), board_index_max=allb.max(0).tolist(),
               frames=len(F), corners=int(len(res)), rms_camera_px=float(rms_intr), rms_chain_px=float(np.sqrt((res ** 2).sum(1).mean())),
               rms_chain_mm=np.sqrt((E ** 2).mean(0)).tolist()), open(out, 'w'), indent=1)
