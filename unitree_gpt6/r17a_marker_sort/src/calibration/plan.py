"""Pick diverse look-down camera poses that see a large part of the board, predicted with a preliminary calibration. No motion."""
import json, sys, cv2, numpy as np
from robot.rt import *
cal = json.load(open(sys.argv[1])); N = int(sys.argv[2]); rng = np.random.default_rng(int(sys.argv[3]))
W, H = cal['image_size']; K = np.array(cal['K']); dist = np.array(cal['dist']); X = np.array(cal['link7_from_camera']); Twb = np.array(cal['world_from_board'])
(i0, j0), (i1, j1) = cal['board_index_min'], cal['board_index_max']
B = np.array([(i, j, 0) for i in range(int(i0), int(i1) + 1) for j in range(int(j0), int(j1) + 1)], float) * SQUARE
PB = (Twb @ np.c_[B, np.ones(len(B))].T).T[:, :3]; floor = max(PB[:, 2].max(), 0.0)
q0 = np.array(state()['q'])
FINGERS = [np.array(p, np.float32) for p in ([[0, 720], [0, 620], [230, 420], [410, 400], [420, 450], [280, 720]],
                                             [[830, 395], [990, 410], [1280, 620], [1280, 720], [890, 720], [835, 450]])]   # open gripper, padded
LO = np.maximum(MODEL.lower + 0.03, [-.6, 0, -.4, 0, -.6, -.76, -.9])
HI = np.minimum(MODEL.upper - 0.03, [.6, 1.1, .4, 1.3, .6, .3, .9])

def seg_dist(p, a, b):
    ab = b - a; t = np.clip((p - a) @ ab / max(ab @ ab, 1e-12), 0, 1); return np.linalg.norm(p - a - t * ab)

def safe(q):
    Tw7, pts = fk(q)
    tips = [(Tw7 @ [x, -TIP_LEN, 0, 1])[:3] for x in (-0.05, 0, 0.05)]; cam = (Tw7 @ X)[:3, 3]
    if min(t[2] for t in tips) < floor + 0.09 or cam[2] < floor + 0.12 or pts[4:, 2].min() < floor + 0.12: return False
    tip = tips[1]
    if not (-0.25 < tip[0] < 0.25 and -0.45 < tip[1] < -0.06): return False        # stay in the seen front area
    hand = [pts[8] + s * (tip - pts[8]) for s in (0.3, 0.6, 1.0)] + [cam]
    return min(seg_dist(p, pts[i], pts[i + 1]) for p in hand for i in range(1, 5)) > 0.085   # gripper/camera vs own links

def view(q):
    Twc = fk(q)[0] @ X
    if Twc[2, 2] > -0.7: return None                                                # look down (within ~45 deg of vertical)
    Tcw = np.linalg.inv(Twc); pc = (Tcw[:3, :3] @ PB.T).T + Tcw[:3, 3]
    ok = pc[:, 2] > 0.08
    uv = cv2.projectPoints(pc, np.zeros(3), np.zeros(3), K, dist)[0].reshape(-1, 2)
    ok &= (uv[:, 0] > 15) & (uv[:, 0] < W - 15) & (uv[:, 1] > 15) & (uv[:, 1] < H - 15)
    for f in FINGERS:
        ok &= np.array([cv2.pointPolygonTest(f, (float(u), float(v)), False) < 0 for u, v in uv])
    return (Twc, uv[ok]) if ok.sum() >= 45 else None

def path_safe(a, b):
    return all(safe(a + s * (b - a)) for s in np.linspace(0, 1, 12))

cands = []
for q in rng.uniform(LO, HI, (80000, 7)):
    if not safe(q): continue
    v = view(q)
    if v is None: continue
    Twc, uv = v
    roll = np.arctan2(Twc[2, 0], -Twc[2, 1])
    cands.append((q, np.r_[Twc[:3, 2] * 2.0, roll * 0.7, Twc[:3, 3] / 0.08, uv.mean(0) / 300], uv))
print('candidates', len(cands))
feats = np.array([c[1] for c in cands]); chosen = []; d = np.full(len(cands), np.inf); cur = 0
for _ in range(N):
    chosen.append(cur); d = np.minimum(d, np.linalg.norm(feats - feats[cur], axis=1)); cur = int(d.argmax())
qs = [cands[i][0] for i in chosen]
assert safe(q0), 'current pose fails the clearance model'
order, left, at = [], list(range(len(qs))), q0
while left:
    left.sort(key=lambda i: np.abs(qs[i] - at).max())
    nxt = next((i for i in left if path_safe(at, qs[i])), None)
    if nxt is None: print('dropping', len(left), 'poses with unsafe transition'); break
    order.append(nxt); left.remove(nxt); at = qs[nxt]
plan = [qs[i].tolist() for i in order]
json.dump({'q0': q0.tolist(), 'poses': plan}, open('calib_plan.json', 'w'))
cov = np.zeros((H // 8, W // 8), np.uint8)
for i in order:
    for u, v in cands[chosen[i]][2]: cv2.circle(cov, (int(u / 8), int(v / 8)), 2, 255, -1)
cv2.imwrite('calib_cover.png', cov)
D = np.array([(fk(q)[0] @ X)[:3, 2] for q in plan]); P = np.array([(fk(q)[0] @ X)[:3, 3] for q in plan])
print(len(plan), 'poses; view dir range', D.min(0).round(2), D.max(0).round(2), 'cam pos range', P.min(0).round(2), P.max(0).round(2))
print('predicted image coverage %.0f%%; min tip z %.3f' % (100 * (cov > 0).mean(), min((fk(q)[0] @ [0, -TIP_LEN, 0, 1])[2] for q in plan)))
