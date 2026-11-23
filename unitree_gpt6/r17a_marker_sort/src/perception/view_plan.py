"""Find empty-hand camera poses that look steeply down at a world point. No motion.
usage: python3 -m perception.view_plan X Y Z N OUT.json  -> N joint targets (steepest first, >= 6 cm apart), e.g. OUT = RUN/box_views.json"""
import sys, json, cv2, numpy as np
from robot import rt
from robot.rt import *
tgt = np.array(sys.argv[1:4], float); N = int(sys.argv[4]); rng = np.random.default_rng(0)
rt.OBSTACLES.append((np.array([0.13, -0.36, -0.05]), np.array([0.33, -0.15, 0.19])))        # pen box + margin
q0 = np.array(state()['q'])
LO = np.maximum(MODEL.lower + 0.03, [-1.2, 0, -.5, 0, -.8, -.76, -.9]); HI = np.minimum(MODEL.upper - 0.03, [.3, 1.4, .5, 1.6, .8, .5, .9])
out = []
for q in rng.uniform(LO, HI, (150000, 7)):
    Twc = cam_pose(q)
    if Twc[2, 2] > -0.75: continue
    d = tgt - Twc[:3, 3]; dist = np.linalg.norm(d)
    if not 0.16 < dist < 0.36: continue
    uv = project_world(q, [tgt])[0]
    if not (430 < uv[0] < 850 and 120 < uv[1] < 400) or (Twc[:3, :3].T @ d)[2] < 0.1: continue
    if not clear(q, 0.21): continue                                                       # fingertips stay above the caps
    out.append((Twc[2, 2] + 1.5 * abs(dist - 0.24), q, Twc[:3, 3], uv))                   # steep and ~24 cm away
out.sort(key=lambda t: t[0]); picked = []
for sc, q, c, uv in out:
    if all(np.linalg.norm(c - p[2]) > 0.06 for p in picked) and path_clear(picked[-1][1] if picked else q0, q, 0.12):
        picked.append((sc, q, c, uv))
        if len(picked) == N: break
print(len(out), 'candidates')
for sc, q, c, uv in picked: print('cam', c.round(3), 'view z %.2f' % cam_pose(q)[2, 2], 'target px', uv.round(0), 'q', q.round(3).tolist())
json.dump([p[1].tolist() for p in picked], open(sys.argv[5], 'w'))
