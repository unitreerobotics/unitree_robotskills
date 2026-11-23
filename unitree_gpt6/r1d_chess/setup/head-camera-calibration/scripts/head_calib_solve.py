#!/usr/bin/env python3
"""Offline eye-on-head solve: camera mount X, board pose Y, head pitch offset, optional hypotheses.

Model: T_pelvis_cam = Rz(qw) * T_pitch * Ry(qp + dp) * T_yaw * Rz(qy) * X; corners project through the
stage-1 fisheye intrinsics. Corner medians over the frames of each pose; K is fixed.
Head yaw has about 16 mrad of direction-dependent backlash, modelled from the last direction of motion.
The waist encoder is not trusted (measured motion differs from image motion), see GroupModel.
"""
import argparse
import glob
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
BOARD = cv2.aruco.CharucoBoard((5, 7), .035, .026, d)
OBJ = BOARD.getChessboardCorners().astype(float)
DET = cv2.aruco.CharucoDetector(BOARD)


def T(t=(0, 0, 0), r=None):
    m = np.eye(4)
    m[:3, 3] = t
    if r is not None:
        m[:3, :3] = r
    return m


def rot(axis, a):
    return R.from_euler(axis, a).as_matrix()


def exp6(x):
    return T(x[:3], R.from_rotvec(x[3:]).as_matrix())


def log6(m):
    return np.r_[m[:3, 3], R.from_matrix(m[:3, :3]).as_rotvec()]


def head_chain(p, y):
    return T((-.006, .03155, .255)) @ T(r=rot('y', p)) @ T((.0010126, -.03155, .1165)) @ T(r=rot('z', y))


def load(root, eye, K, D):
    manifest = json.loads((root/'manifest.json').read_text())
    views, previous, direction = [], None, np.zeros(3)
    x0 = 0 if eye == 'left' else 640
    for p in manifest['poses']:
        per = []
        for f in sorted(glob.glob(str(root/p['directory']/'*.jpg'))):
            g = cv2.imread(f, 0)[:, x0:x0+640]
            c, ids, _, _ = DET.detectBoard(g)
            if ids is not None:
                per.append({int(i): v for i, v in zip(ids.ravel(), c.reshape(-1, 2))})
        counts = {}
        for x in per:
            for i in x:
                counts[i] = counts.get(i, 0)+1
        ids = sorted(i for i, n in counts.items() if n >= 4)
        if len(ids) < 8:
            continue
        pix = np.array([np.median([x[i] for x in per if i in x], 0) for i in ids])
        q = np.array([*p['head_q'], p['waist_q']])
        # Backlash state persists until that axis reverses, so keep the last nonzero direction.
        if previous is not None:
            step = np.sign(np.round(q-previous, 2))
            direction = np.where(step != 0, step, direction)
        previous = q
        move = direction.copy()
        views.append({'name': p['name'], 'index': p['index'], 'q': q, 'move': move, 'ids': ids, 'pix': pix})
    return views


def project(Tcb, ids, K, D):
    r = cv2.Rodrigues(Tcb[:3, :3])[0]
    uv, _ = cv2.fisheye.projectPoints(OBJ[ids].reshape(-1, 1, 3), np.ascontiguousarray(r), np.ascontiguousarray(Tcb[:3, 3]).reshape(3, 1), K, D)
    return uv.reshape(-1, 2)


def pnp(view, K, D):
    und = cv2.fisheye.undistortPoints(view['pix'].reshape(-1, 1, 2), K, D).reshape(-1, 2)
    ok, r, t = cv2.solvePnP(OBJ[view['ids']], und, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE)
    return T(t.ravel(), cv2.Rodrigues(r)[0])


class Model:
    def __init__(self, views, K, D, backlash=False, segments=1, free_dp=True, dp_sigma=None):
        self.views, self.K, self.D = views, K, D
        self.backlash, self.segments, self.free_dp, self.dp_sigma = backlash, segments, free_dp, dp_sigma
        last = max(v['index'] for v in views)+1
        for v in views:
            v['segment'] = min(segments-1, v['index']*segments//last)
        self.n = 6+6*segments+1+(3 if backlash else 0)

    def unpack(self, x):
        X = exp6(x[:6])
        Ys = [exp6(x[6+6*i:12+6*i]) for i in range(self.segments)]
        k = 6+6*self.segments
        dp = x[k] if self.free_dp else 0.
        b = x[k+1:k+4] if self.backlash else np.zeros(3)
        return X, Ys, dp, b

    def pose(self, view, X, dp, b):
        q = view['q']+b*view['move']
        return T(r=rot('z', q[2])) @ head_chain(q[0]+dp, q[1]) @ X

    def residual_view(self, x, view):
        X, Ys, dp, b = self.unpack(x)
        Tcb = np.linalg.inv(self.pose(view, X, dp, b)) @ Ys[view['segment']]
        return (project(Tcb, view['ids'], self.K, self.D)-view['pix']).ravel()

    def residual(self, x):
        r = np.concatenate([self.residual_view(x, v) for v in self.views])
        if self.dp_sigma:
            r = np.r_[r, self.unpack(x)[2]/self.dp_sigma]
        return r

    def fit(self, x0):
        return least_squares(self.residual, x0, loss='huber', f_scale=1., x_scale='jac', max_nfev=200)


def initial(views, K, D):
    """X by Tsai-style hand-eye on poses of fixed board, seeded with robot-world solver."""
    r_w2g, t_w2g, r_b2c, t_b2c = [], [], [], []
    for v in views:
        g = T(r=rot('z', v['q'][2])) @ head_chain(v['q'][0], v['q'][1])
        gi = np.linalg.inv(g)
        c = pnp(v, K, D)
        r_w2g.append(gi[:3, :3]); t_w2g.append(gi[:3, 3:4])
        r_b2c.append(c[:3, :3]); t_b2c.append(c[:3, 3:4])
    # world=pelvis, base=board frame in OpenCV naming; yields gripper->camera
    Rb, tb, Rg, tg = cv2.calibrateRobotWorldHandEye(r_b2c, t_b2c, r_w2g, t_w2g)
    Xcam = T(tg.ravel(), Rg)            # head_yaw -> camera inverse convention
    Y = T(tb.ravel(), Rb)
    return np.linalg.inv(Xcam), np.linalg.inv(Y)


def stats(m, x, views):
    e = [m.residual_view(x, v).reshape(-1, 2) for v in views]
    n = np.concatenate([np.linalg.norm(a, axis=1) for a in e])
    return {'rms_px': float(np.sqrt(np.mean(n**2))), 'p90_px': float(np.percentile(n, 90)), 'max_px': float(n.max()),
            'per_view': [(v['name'], round(float(np.sqrt(np.mean(np.sum(a**2, 1)))), 2)) for v, a in zip(views, e)]}


def run(views, K, D, **kw):
    m = Model(views, K, D, **kw)
    X0, Y0 = initial(views, K, D)
    x0 = np.r_[log6(X0), np.tile(log6(Y0), m.segments), np.zeros(m.n-6-6*m.segments)]
    return m, m.fit(x0)




class Rig:
    """Both eyes on one rigid head mount; stage-1 intrinsics and stereo extrinsics stay fixed.

    Unknowns: X (head_yaw_Link -> left camera), a board pose per group, yaw backlash, pitch/yaw scale.
    Effective joint angle: q_eff = ref + (q - ref) * (1 + s) + b * last_direction (yaw only).
    Images show the head turning about 2 % less than its encoder reports; this is measured directly from
    PnP rotation, without any mount model. The waist encoder is not used: T_waist_cam does not contain the
    waist angle, and each waist setting has its own board pose. Head pitch zero offset stays at 0: it is
    absorbed by the board pose, so it is bounded by board levelness rather than estimated.
    """

    def __init__(self, views, intr, S, groups, pivot):
        self.views, self.intr, self.S, self.groups, self.pivot = views, intr, S, groups, np.asarray(pivot, float)
        self.n = 6+6*len(groups)+3

    def unpack(self, x):
        k = 6+6*len(self.groups)
        return exp6(x[:6]), [exp6(x[6+6*i:12+6*i]) for i in range(len(self.groups))], x[k], x[k+1:k+3]

    def residual_view(self, x, eye, v):
        X, Ys, b, s = self.unpack(x)
        q = self.pivot+(v['q'][:2]-self.pivot)*(1+s)
        q[1] += b*v['move'][1]
        Tcb = np.linalg.inv(head_chain(q[0], q[1]) @ X) @ Ys[self.groups.index(v['group'])]
        if eye == 'right':
            Tcb = self.S @ Tcb
        return (project(Tcb, v['ids'], *self.intr[eye])-v['pix']).ravel()

    def residual(self, x, subset=None):
        return np.concatenate([self.residual_view(x, e, v) for e in self.views for i, v in enumerate(self.views[e])
                               if subset is None or i in subset])

    def fit(self, x0, subset=None, nfev=40):
        return least_squares(self.residual, x0, args=(subset,), loss='huber', f_scale=1., x_scale='jac', max_nfev=nfev).x


def _model_path():
    """Kinematic URDF of the package (config/runtime.json model_urdf, relative to the package root)."""
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    import _common  # noqa: F401  (adds src/ to sys.path)
    from r1d_chess.robot.r1d_config import MODEL_PATH
    return MODEL_PATH


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage2', nargs='+', required=True)
    p.add_argument('--intrinsics', required=True)
    p.add_argument('--stereo', required=True)
    p.add_argument('--accept-note', help='write --config even if a gate fails; the note is stored as the acceptance record')
    p.add_argument('--output', required=True)
    p.add_argument('--config', help='written only when every gate passes')
    a = p.parse_args()
    fit = json.loads(Path(a.intrinsics).read_text())
    intr = {e: (np.array(fit[e]['fisheye']['K']), np.array(fit[e]['fisheye']['D']).reshape(4, 1)) for e in ('left', 'right')}
    st = json.loads(Path(a.stereo).read_text())
    S = T(st['T'], np.array(st['R']))
    views = {e: [] for e in intr}
    for e in intr:
        for root in a.stage2:
            views[e] += [v for v in load(Path(root), e, *intr[e]) if not v['name'].startswith('ref')]
        for v in views[e]:
            v['group'] = v['name'].split('_')[0]
    # Keep poses seen by both eyes so fold indices refer to the same pose.
    common = {v['name'] for v in views['left']} & {v['name'] for v in views['right']}
    views = {e: [v for v in views[e] if v['name'] in common] for e in views}
    groups = sorted({v['group'] for v in views['left']})
    # Scale pivot: median commanded head pose of this capture (data-derived, robot independent).
    pivot = np.median(np.array([v['q'][:2] for v in views['left']]), 0)
    m = Rig(views, intr, S, groups, pivot)
    # Initial X and board pose from a closed-form robot-world/hand-eye solve on the left eye.
    X0, Y0 = initial(views['left'], *intr['left'])
    x0 = np.r_[log6(X0), np.tile(log6(Y0), len(groups)), 0., 0., 0.]
    x = m.fit(x0, nfev=80)
    X, Ys, b, sc = m.unpack(x)
    n = len(views['left'])
    res = np.linalg.norm(m.residual(x).reshape(-1, 2), axis=1)
    rng = np.random.default_rng(0)
    held, shift, rot, centre = [], [], [], []
    centre0 = [(y @ np.r_[.0875, .1225, 0, 1])[:3] for y in Ys]
    for f in np.array_split(rng.permutation(n), 5):
        out = set(f.tolist())
        y = m.fit(x, set(range(n))-out, 15)
        held.append(m.residual(y, out))
        Xy, Yy, _, _ = m.unpack(y)
        shift.append(float(np.linalg.norm(Xy[:3, 3]-X[:3, 3])*1e3))
        rot.append(float(R.from_matrix(Xy[:3, :3] @ X[:3, :3].T).magnitude()*1e3))
        centre.append(max(float(np.linalg.norm((Yy[i] @ np.r_[.0875, .1225, 0, 1])[:3]-centre0[i])*1e3) for i in range(len(groups))))
    h = np.linalg.norm(np.concatenate(held).reshape(-1, 2), axis=1)
    tilt = [float(np.degrees(np.arccos(min(1., abs(y[2, 2]))))) for y in Ys]
    half = float(np.linalg.norm(st['T'])/2*1e3)
    gates = {'heldout_rms_px<=1': float(np.sqrt(np.mean(h**2))), 'heldout_p90_px<=1.5': float(np.percentile(h, 90)),
             'fold_position_mm<=2': max(shift), 'fold_rotation_mrad<=2': max(rot), 'board_tilt_deg<=1': max(tilt),
             'fold_board_centre_mm<=3': max(centre)}
    limit = {'heldout_rms_px<=1': 1., 'heldout_p90_px<=1.5': 1.5, 'fold_position_mm<=2': 2., 'fold_rotation_mrad<=2': 2.,
             'board_tilt_deg<=1': 1., 'fold_board_centre_mm<=3': 3.}
    passed = {k: v <= limit[k] for k, v in gates.items()}
    pose_ref = head_chain(*pivot) @ X
    result = {'poses': n, 'groups': groups, 'fit_rms_px': float(np.sqrt(np.mean(res**2))), 'gates': gates, 'passed': passed,
              'X_head_yaw_to_left_camera': X.tolist(), 'yaw_backlash_rad': float(b), 'joint_scale_pitch_yaw': sc.tolist(),
              'scale_pivot_head_q_rad': pivot.tolist(), 'T_waist_left_camera_at_pivot': pose_ref.tolist(), 'board_centre_waist_mm': [(c*1e3).tolist() for c in centre0],
              'board_normal': [y[:3, 2].tolist() for y in Ys], 'half_baseline_mm': half, 'mount_y_mm': float(X[1, 3]*1e3)}
    Path(a.output).write_text(json.dumps(result, indent=1))
    np.set_printoptions(precision=3, suppress=True)
    print('poses', n, 'fit rms', round(result['fit_rms_px'], 3), 'yaw backlash mrad', round(float(b)*1e3, 2), 'scale % (pitch, yaw)', sc*100)
    for k, v in gates.items():
        print(f'  {k:28s} {v:7.3f}  {"PASS" if passed[k] else "FAIL"}')
    print('X t mm', X[:3, 3]*1e3, 'optical axis', X[:3, 2], '| left cam y vs half baseline', round(result['mount_y_mm'], 2), round(half, 2))
    print('camera at scale pivot', np.round(pivot, 3), 'waist frame mm', pose_ref[:3, 3]*1e3)
    for g, c, t, y in zip(groups, centre0, tilt, Ys):
        print(f'  group {g:8s} board centre mm {c*1e3} tilt {t:.2f} deg normal {y[:3, 2]}')
    if a.config and (all(passed.values()) or a.accept_note):
        Path(a.config).write_text(json.dumps({
            'schema_version': 1, 'camera': 'head stereo, stream 640x480 per eye', 'model': 'cv2.fisheye', 'status': 'model_estimate',
            'frame_chain': 'waist_yaw_link -> head_pitch -> head_yaw_Link -> left camera (OpenCV axes)',
            'K_left': intr['left'][0].tolist(), 'D_left': intr['left'][1].ravel().tolist(),
            'K_right': intr['right'][0].tolist(), 'D_right': intr['right'][1].ravel().tolist(),
            'T_right_from_left': S.tolist(), 'X_head_yaw_to_left_camera': X.tolist(), 'reference_head_q_rad': pivot.tolist(),
            'reference_head_q_note': 'scale pivot of the joint model (median head pose of the capture), not the locked head pose',
            'accepted': ({'note': a.accept_note, 'failed_gates': [k for k, v in passed.items() if not v]} if a.accept_note else 'all gates passed'),
            'joint_scale_pitch_yaw': sc.tolist(), 'yaw_backlash_rad': float(b), 'head_pitch_offset_rad': 0.,
            'head_pitch_offset_note': 'not estimated; bounded by board levelness', 'gates': gates,
            'kinematic_model_sha256': __import__('hashlib').sha256(_model_path().read_bytes()).hexdigest(),
            'evidence': str(Path(a.output).resolve())}, indent=1))
        print('written', a.config)


if __name__ == '__main__':
    main()
