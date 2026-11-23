"""Real-robot helpers for R1-7A. Units: m, rad.
Robot I/O is only state(), move(), gripper() here and frame() in robot/grab.py. They talk to the author's own joint controller
over HTTP (127.0.0.1:8000); replace them with your controller's interface (contract in ../README.md)."""
import json, sys, time, urllib.request
from pathlib import Path
import numpy as np
SRC = Path(__file__).resolve().parents[1]      # src/: run every tool from here as python3 -m <dir>.<tool>
from ik.kinematics import RobotModel
from robot.grab import frame, URL

CAL = json.loads((SRC / 'calib.json').read_text()) if (SRC / 'calib.json').exists() else None
DELTA = np.array(CAL['joint_zero_offset']) if CAL else np.zeros(7)      # calibrated FK uses q + DELTA

MODEL = RobotModel(SRC.parent / 'r1-7a_sdk/urdf/URDF_R1-7A.tar.gz', 'right')      # URDF from the official r1-7a_sdk; point this at your copy
PATTERN, SQUARE = (11, 8), 0.030          # Q12-400-30: inner corners, square size
TIP_LEN = 0.0527 + 0.143                   # Link7 origin -> fingertip along Link7 -Y (nominal CAD)

def T(R=np.eye(3), t=(0, 0, 0)):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M

# Nominal CAD hand-eye (starting point only): Link7 <- gripper <- camera optical (x right, y down, z forward)
_c, _s = np.cos(np.pi / 6), np.sin(np.pi / 6)
L7_GRIP = T(np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1.]]), (0, -0.0527, 0))   # Rz(-90)Rx(180)
GRIP_CAM = T(np.array([[0, -_s, _c], [-1, 0, 0], [0, -_c, -_s]]), (0.068743, 0, 0.047476))
L7_CAM_NOMINAL = L7_GRIP @ GRIP_CAM

def fk(q):
    p, R, pts = MODEL.forward(q)
    return T(R, p), np.array(pts)

def state():
    return json.load(urllib.request.urlopen(URL + '/api/state', timeout=3))

def post(path, body):
    req = urllib.request.Request(URL + path, json.dumps(body).encode(), {'Content-Type': 'application/json'})
    try:
        return json.load(urllib.request.urlopen(req, timeout=5))
    except urllib.error.HTTPError as e:
        raise RuntimeError(f'{path}: {e.read().decode()}')

def move(q, speed=0.4, tol=0.02, settle=0.4):
    """Joint move; returns measured q after the arm is done and still. Raises on fault/timeout."""
    r = post('/api/move', {'q': [float(v) for v in q], 'speed': speed})['result']
    deadline = time.time() + r['duration_s'] + 6
    while time.time() < deadline:
        time.sleep(0.1)
        s = state()
        if s['fault'] or not s['armed']:
            raise RuntimeError(f"fault: {s['fault']}")
        if not s['busy'] and max(map(abs, s['dq'])) < 0.02:
            still = 0
            for _ in range(30):                                    # overshoot turnaround / friction creep: need 3 still intervals in a row
                a = np.array(state()['q']); time.sleep(settle / 2); s = state()
                still = still + 1 if np.abs(np.array(s['q']) - a).max() < 0.0008 else 0
                if still >= 3: break
            err = np.abs(np.array(s['q']) - q).max()
            if err > tol:
                print(f'  warn: steady-state joint error {err:.3f} rad')
            return np.array(s['q'])
    raise RuntimeError('move timeout')

def snap(name):
    """Still frame + joint state bracketing it."""
    for _ in range(6):
        s0 = state(); jpg = frame(); s1 = state()
        if not (s0['busy'] or s1['busy']) and np.abs(np.array(s0['q']) - s1['q']).max() < 0.002: break
        time.sleep(0.5)
    else:
        raise RuntimeError('arm moved during capture')
    Path(name + '.jpg').write_bytes(jpg)
    meta = {'t': time.time(), 'q': s1['q'], 'tau': s1['tau'], 'gripper': s1['gripper']['opening_fraction']}
    Path(name + '.json').write_text(json.dumps(meta))
    return jpg, meta

def _ring(gray, p, r, n=16):
    a = np.arange(n) * 2 * np.pi / n
    x = (p[0] + r * np.cos(a)).astype(np.float32)[None]; y = (p[1] + r * np.sin(a)).astype(np.float32)[None]
    import cv2
    return cv2.remap(gray, x, y, cv2.INTER_LINEAR)[0].astype(float)

def _saddle(gray, p, r):
    """Checkerboard X-corner test: 4 black/white transitions on a ring, point-symmetric, good contrast."""
    v = _ring(gray, p, r); lo, hi = v.min(), v.max()
    if hi - lo < 50: return False
    b = v > (lo + hi) / 2
    return np.count_nonzero(b != np.roll(b, 1)) == 4 and np.count_nonzero(b != np.roll(b, 8)) <= 2

def corners(gray, min_corners=16):
    """Partial checkerboard: saddle points grown into a lattice by local steps, colour alternation enforced.
    Returns (points Nx2, local grid indices Nx2) or None. The local origin is NOT a fixed board corner."""
    import cv2
    from scipy.spatial import cKDTree
    blur = cv2.GaussianBlur(gray, (0, 0), 1.5)
    c = cv2.goodFeaturesToTrack(blur, 3000, 0.005, 9, blockSize=7)
    if c is None: return None
    c = cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01)).reshape(-1, 2)
    h, w = gray.shape
    c = np.array([p for p in c if 12 < p[0] < w - 12 and 12 < p[1] < h - 12 and _saddle(blur, p, 5) and _saddle(blur, p, 9)])
    if len(c) < min_corners: return None
    keep = np.ones(len(c), bool); tree = cKDTree(c)                      # merge duplicates after sub-pixel refinement
    for a, b in tree.query_pairs(4.0): keep[b] = False
    c = c[keep].astype(np.float64); tree = cKDTree(c)

    def colour(p, e1, e2):                                               # + if the (e1+e2) square is brighter than the (e1-e2) one
        g = lambda q: float(blur[int(np.clip(round(q[1]), 0, h - 1)), int(np.clip(round(q[0]), 0, w - 1))])
        return g(p + .3 * (e1 + e2)) + g(p - .3 * (e1 + e2)) - g(p + .3 * (e1 - e2)) - g(p - .3 * (e1 - e2))

    def grow(seed):
        _, nb = tree.query(c[seed], 9); v = [(c[j] - c[seed], j) for j in nb[1:] if j < len(c)]
        pairs = sorted((max(np.linalg.norm(a), np.linalg.norm(b)), tuple(a))
                       for i, (a, _) in enumerate(v) for b, _ in v[i + 1:]
                       if np.linalg.norm(a + b) < 0.25 * max(np.linalg.norm(a), np.linalg.norm(b)))
        if not pairs: return {}
        e1 = np.array(pairs[0][1]); e2 = next((np.array(p[1]) for p in pairs[1:]
                                              if abs(np.dot(p[1], e1)) / (np.linalg.norm(p[1]) * np.linalg.norm(e1)) < 0.8), None)
        if e2 is None: return {}
        s0 = colour(c[seed], e1, e2)
        if abs(s0) < 60: return {}
        G = {(0, 0): seed}; B = {(0, 0): (e1, e2)}; used = {seed}; todo = [(0, 0)]
        while todo:
            ij = todo.pop(); p = c[G[ij]]; ba = B[ij]
            for ax, sg in ((0, 1), (0, -1), (1, 1), (1, -1)):
                nx = (ij[0] + sg * (ax == 0), ij[1] + sg * (ax == 1))
                if nx in G: continue
                back = (ij[0] - sg * (ax == 0), ij[1] - sg * (ax == 1))
                step = p - c[G[back]] if back in G else sg * ba[ax]
                d, j = tree.query(p + step)
                if d > 0.3 * np.linalg.norm(step) or j in used: continue
                nb_basis = list(ba); nb_basis[ax] = sg * (c[j] - p)
                sc = colour(c[j], *nb_basis)
                if sc * s0 * (-1) ** (nx[0] + nx[1]) < 30 * abs(s0) / 255: continue     # colours must alternate
                G[nx] = j; B[nx] = tuple(nb_basis); used.add(j); todo.append(nx)
        return G

    best = {}; rng = np.random.default_rng(0)
    for seed in rng.permutation(len(c))[:25]:
        if seed in best.values(): continue
        G = grow(int(seed))
        if len(G) > len(best): best = G
    if len(best) < min_corners: return None
    idx = np.array(list(best.keys()), float); pts = c[list(best.values())]
    return pts, idx - idx.min(0)

OBJ = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
OBJ[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE

# ---- calibrated geometry (needs calib.json) ---------------------------------
def cam_pose(q):
    """world <- camera optical frame at MEASURED joints q."""
    return fk(np.asarray(q, float) + DELTA)[0] @ np.array(CAL['link7_from_camera'])

def project_world(q, pts):
    import cv2
    Tcw = np.linalg.inv(cam_pose(q)); pc = (Tcw[:3, :3] @ np.atleast_2d(pts).T).T + Tcw[:3, 3]
    return cv2.projectPoints(pc, np.zeros(3), np.zeros(3), np.array(CAL['K']), np.array(CAL['dist']))[0].reshape(-1, 2)

def pixel_to_plane(q, uv, normal, point):
    """Intersect pixel rays with a world plane (normal . (x - point) = 0). uv: Nx2 raw (distorted) pixels."""
    import cv2
    n = cv2.undistortPoints(np.asarray(uv, float).reshape(-1, 1, 2), np.array(CAL['K']), np.array(CAL['dist'])).reshape(-1, 2)
    Twc = cam_pose(q); d = (Twc[:3, :3] @ np.c_[n, np.ones(len(n))].T).T; o = Twc[:3, 3]
    t = ((np.asarray(point) - o) @ normal) / (d @ normal)
    return o + t[:, None] * d

# ---- tool frame: x = approach (finger direction), y = finger closing axis, z = camera side --------------
TCP_X = 0.1433                                                   # gripper base -> fingertip plane (CAD)
def link7_tcp(back=0.0):
    """Link7 <- TCP. Gripper pose in Link7 comes from the calibrated camera + CAD camera mount. back = metres behind the fingertips."""
    return np.array(CAL['link7_from_camera']) @ np.linalg.inv(GRIP_CAM) @ T(t=(TCP_X - back, 0, 0))

def tcp_pose(q, back=0.0):
    return fk(np.asarray(q, float) + DELTA)[0] @ link7_tcp(back)

def down_pose(xyz, yaw, back=0.0):
    """Tool pointing straight down at world xyz; closing axis at angle yaw from world +x."""
    y = np.array([np.cos(yaw), np.sin(yaw), 0.]); x = np.array([0, 0, -1.])
    return T(np.c_[x, y, np.cross(x, y)], xyz)

def ik_tcp(Tw_tcp, seed, back=0.0):
    """Joint COMMAND (already corrected for the zero offsets) for a TCP pose, or None."""
    Tw7 = Tw_tcp @ np.linalg.inv(link7_tcp(back))
    r = MODEL.solve(Tw7[:3, 3], Tw7[:3, :3], np.asarray(seed, float) + DELTA)
    if not r['success']: return None
    q = np.array(r['q']) - DELTA
    return q if np.all(q > MODEL.lower + 0.01) and np.all(q < MODEL.upper - 0.01) else None

# ---- crude clearance model (no meshes): heights above the table + gripper/camera vs. own links -----------
def _seg_dist(p, a, b):
    ab = b - a; t = np.clip((p - a) @ ab / max(ab @ ab, 1e-12), 0, 1); return np.linalg.norm(p - a - t * ab)

OBSTACLES = []      # world AABBs [(xmin, ymin, zmin), (xmax, ymax, zmax)] the hand/camera must stay out of (e.g. the pen box)

def clear(q, tip_min=0.05, half_open=0.045):
    """q: joint command/measurement (uncorrected). tip_min: lowest allowed fingertip z in world (m)."""
    q = np.asarray(q, float)
    # measured joints may sit slightly outside the URDF range (e.g. J2/J4 at -0.03 near the zero pose); targets never do
    if np.any(q < MODEL.lower - 0.06) or np.any(q > MODEL.upper + 0.06): return False
    Tw7, pts = fk(q + DELTA); Tt = Tw7 @ link7_tcp()
    tips = [(Tt @ [0, y, 0, 1])[:3] for y in (-half_open, 0, half_open)]; cam = (Tw7 @ np.array(CAL['link7_from_camera']))[:3, 3]
    if min(t[2] for t in tips) < tip_min or cam[2] < tip_min + 0.05 or pts[4:, 2].min() < 0.10: return False
    hand = [pts[8] + s * (tips[1] - pts[8]) for s in (0.3, 0.6, 1.0)] + [cam]
    for lo, hi in OBSTACLES:
        if any(np.all(p >= lo) and np.all(p <= hi) for p in hand + tips + [pts[8], pts[7]]): return False
    return min(_seg_dist(p, pts[i], pts[i + 1]) for p in hand for i in range(1, 5)) > 0.08

def path_clear(a, b, tip_min=0.05, n=24):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return all(clear(a + s * (b - a), tip_min) for s in np.linspace(0, 1, n))

def goto(q, speed=0.4, tip_min=0.05, trim=True):
    """Checked joint move from the measured pose. trim: one extra small move that cancels the steady-state joint error."""
    q = np.asarray(q, float); q_now = np.array(state()['q'])
    if not path_clear(q_now, q, tip_min): raise RuntimeError('path fails the clearance model')
    qm = move(q, speed)
    if trim and np.abs(q - qm).max() > 0.002:
        cmd = np.clip(q + (q - qm), MODEL.lower, MODEL.upper)      # command past the target by the observed error
        qm = move(cmd, speed)
    return qm

def gripper(opening, timeout=6.0):
    """opening 0..1 of the calibrated travel. Waits until the finger motor has really stopped (it starts smoothly,
    so 'not moving' right after the command means nothing). Returns the gripper snapshot."""
    g = state()['gripper']
    if not g['q_min'] - 0.3 <= g['q'] <= g['q_max'] + 0.3:
        # Seen 2026-11-17 after a power cycle: encoder reading shifted by exactly -2*pi, fingers physically open at "0 %".
        # Any command would then drive the fingers a full turn into the hard stop.
        raise RuntimeError('gripper reading %.2f rad is outside the calibrated travel %.2f..%.2f: recalibrate the gripper first'
                           % (g['q'], g['q_min'], g['q_max']))
    post('/api/gripper', {'opening': float(opening)})
    t0 = time.time(); still = 0; g = None
    while time.time() - t0 < timeout:
        time.sleep(0.15); g = state()['gripper']
        still = still + 1 if abs(g['dq']) < 0.05 else 0
        if time.time() - t0 > 1.0 and still >= 3: break
    return g

def table_plane(height=0.0):
    """(unit normal pointing up, point) of the table surface raised by `height` m, in world.
    From the calibrated board top face minus the board thickness; valid while the arm base and table are not moved."""
    Twb = np.array(CAL['world_from_board']); n = Twb[:3, 2] * (1 if Twb[2, 2] > 0 else -1)
    return n, Twb[:3, 3] + n * (height - CAL['board_thickness_m'])

def ik_all(Tw_tcp, n_seeds=80, rng=None, extra_seeds=()):
    """Many-seed IK (fast damped least squares per seed). Returns distinct joint COMMANDS, most 'natural' first
    (small J3/J5 twist, far from limits). The arm is redundant: one pose has a family of solutions."""
    rng = rng or np.random.default_rng(0); Tw7 = Tw_tcp @ np.linalg.inv(link7_tcp()); sols = []
    seeds = list(extra_seeds) + list(rng.uniform(MODEL.lower, MODEL.upper, (n_seeds, 7)))
    for sd in seeds:
        r = MODEL.solve(Tw7[:3, 3], Tw7[:3, :3], np.clip(np.asarray(sd, float) + DELTA, MODEL.lower, MODEL.upper), realtime=True)
        if not r['success']: continue
        q = np.array(r['q']) - DELTA
        if np.any(q < MODEL.lower + 0.02) or np.any(q > MODEL.upper - 0.02): continue
        if all(np.abs(q - s).max() > 0.15 for s in sols): sols.append(q)
    cost = lambda q: abs(q[2]) + 0.5 * abs(q[4]) - min(0.3, np.min(np.minimum(q - MODEL.lower, MODEL.upper - q)))
    return sorted(sols, key=cost)

def pixel_ray(q, uv):
    """World ray (origin, unit direction) through a raw pixel at measured joints q."""
    import cv2
    n = cv2.undistortPoints(np.asarray(uv, float).reshape(-1, 1, 2), np.array(CAL['K']), np.array(CAL['dist'])).reshape(2)
    Twc = cam_pose(q); d = Twc[:3, :3] @ np.r_[n, 1.0]
    return Twc[:3, 3], d / np.linalg.norm(d)

def triangulate(rays):
    """Least-squares point closest to several (origin, direction) rays; also returns the rms distance to the rays."""
    A = np.zeros((3, 3)); b = np.zeros(3)
    for o, d in rays:
        M = np.eye(3) - np.outer(d, d); A += M; b += M @ o
    x = np.linalg.solve(A, b)
    return x, float(np.sqrt(np.mean([np.linalg.norm((np.eye(3) - np.outer(d, d)) @ (x - o)) ** 2 for o, d in rays])))

def servo(q_target, off=None, speed=0.12, tol=0.004, iters=4):
    """Outer integral loop for the PD-only controller: re-command with the accumulated joint error until the MEASURED
    joints are within tol of q_target. Returns (measured q, offset to reuse for the next nearby target)."""
    q_target = np.asarray(q_target, float); off = np.zeros(7) if off is None else np.asarray(off, float).copy()
    for _ in range(iters):
        qm = move(np.clip(q_target + off, MODEL.lower, MODEL.upper), speed, settle=0.3)
        e = q_target - qm
        if np.abs(e).max() < tol: break
        off += np.clip(e, -0.02, 0.02)
    return qm, off

def axis_from_views(views):
    """3D direction of a straight object from >=2 views. views: [(q, px_a, px_b)] with two pixels on its axis per view.
    Each view gives a plane through the camera centre; the axis is the common line. Returns a unit vector pointing down."""
    normals = []
    for q, a, b in views:
        (_, da), (_, db) = pixel_ray(q, a), pixel_ray(q, b); n = np.cross(da, db); normals.append(n / np.linalg.norm(n))
    d = np.linalg.svd(np.array(normals))[2][-1]
    return d * (-1 if d[2] > 0 else 1)


class EventLog:
    """Tee for sys.stdout: every printed line is also appended to a JSONL file with its wall-clock time, so a recorded
    run can be overlaid on video later without anyone writing a log by hand. usage: sys.stdout = EventLog(path, step)"""
    def __init__(self, path, step):
        self.out = sys.stdout; self.f = open(path, 'a'); self.step = step; self.buf = ''
        self._put('START ' + step)
    def _put(self, text):
        self.f.write(json.dumps({'t': time.time(), 'step': self.step, 'text': text}, ensure_ascii=False) + '\n'); self.f.flush()
    def write(self, s):
        self.out.write(s); self.buf += s
        while '\n' in self.buf:
            line, self.buf = self.buf.split('\n', 1)
            if line.strip(): self._put(line)
    def flush(self): self.out.flush()
