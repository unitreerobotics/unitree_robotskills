"""Reseat a pen standing proud in slot (I, J) - the sequence that worked by hand on 2026-11-18 run16:
cap ray from the verify photo -> pick one extra close, steep view in which the cap is clear of the fingers -> triangulate
(small-blob fallback when the cap top is seen edge-on) -> grip the cap, hold it over the slot at its own height, loosen to a
cage, jiggle -> lift straight up. Always run `python3 -m sorting.sort RUN verify` afterwards. usage: python3 -m sorting.reseat2 RUN I J [FIRST_VIEW]"""
import sys, json, cv2
from robot import rt
from sorting import task
from robot.rt import *
from sorting.task import *
RUN, I, J = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
sys.stdout = EventLog(RUN + '/events.jsonl', 'reseat %d %d' % (I, J))
box = json.load(open(RUN + '/box.json')); P = np.array(box['P']); top = set_box(box); slot = P[J * 6 + I]
hue = json.load(open(RUN + '/held.json'))['hue'] or 'dark'; hue = hue if hue == 'dark' else tuple(hue)
s = state(); assert s['armed'] and not s['fault'] and not s['busy'], s['fault']
rt.OBSTACLES[0] = (rt.OBSTACLES[0][0], rt.OBSTACLES[0][1] + np.array([0, 0, 0.10]))      # stay above the proud pen while looking

def blob_near(im, uv, rmax=45):
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV).astype(int)
    if hue == 'dark': m = (hsv[..., 2] < 75).astype(np.uint8)
    else:
        dh = np.abs(hsv[..., 0] - hue[0]); dh = np.minimum(dh, 180 - dh); m = ((dh < 8) & (hsv[..., 1] > hue[1]) & (hsv[..., 2] > 60)).astype(np.uint8)
    n_, lab, st, cen = cv2.connectedComponentsWithStats(m)
    ks = [k for k in range(1, n_) if st[k, cv2.CC_STAT_AREA] > 40 and np.linalg.norm(cen[k] - uv) < rmax]
    return cen[max(ks, key=lambda k: st[k, cv2.CC_STAT_AREA])] if ks else None

if len(sys.argv) > 4 and sys.argv[4] != 'px':      # the verify photo hid the cap behind a finger: take the first photo from another box view
    v = np.array(json.load(open(RUN + '/box_views.json'))[int(sys.argv[4])]); assert path_clear(np.array(state()['q']), v, 0.17), 'path to the first view'
    move(v, 0.35); snap(RUN + '/reseat_a'); qa = json.load(open(RUN + '/reseat_a.json'))['q']; ima = cv2.imread(RUN + '/reseat_a.jpg')
else:
    qa = json.load(open(RUN + '/verify_raw.json'))['q']; ima = cv2.imread(RUN + '/verify_raw.jpg')
c = (np.array([float(sys.argv[5]), float(sys.argv[6])]), 0) if len(sys.argv) > 6 and sys.argv[4] == 'px' else find_cap(ima, qa, slot + [0, 0, 0.08], hue)
if c is None and hue != 'dark' and not 93 <= hue[0] <= 112:              # dull caps: accept lower saturation (not for blue: the box plastic is pale blue)
    hue = (hue[0], 70); c = find_cap(ima, qa, slot + [0, 0, 0.08], hue)
assert c is not None, 'proud cap not found in the verify photo'
o, d = pixel_ray(qa, c[0]); cap0 = o + d * ((top + 0.08 - o[2]) / d[2])
print('reseat: cap seen in the verify photo, about %s mm off the slot centre' % ((cap0[:2] - slot[:2]) * 1000).round(0))
qn = np.array(state()['q']); rng = np.random.default_rng(3); best = None
for strict in (True, False):
    for c_ in rng.uniform([-1.0, 0, -0.4, 0, -0.6, -0.76, -0.8], [0.3, 1.3, 0.4, 1.5, 0.6, 0.2, 0.8], (12000, 7)):
        Tc = cam_pose(c_); dep = (cap0 - Tc[:3, 3]) @ Tc[:3, 2]
        if not 0.15 < dep < (0.25 if strict else 0.32) or Tc[2, 2] > (-0.8 if strict else -0.6): continue
        uv = project_world(c_, [cap0])[0]
        if not (500 < uv[0] < 780 and 100 < uv[1] < 340): continue
        base = np.linalg.norm(np.cross(Tc[:3, 3] - o, d))
        if base < 0.06 or not clear(c_, 0.17) or not path_clear(qn, c_, 0.17): continue
        if best is None or base > best[0]: best = (base, c_, uv)
    if best: break
assert best is not None, 'no second view found'
move(best[1], 0.35); jpg, meta = snap(RUN + '/reseat_b'); imb = cv2.imdecode(np.frombuffer(jpg, np.uint8), 1)
cb = find_cap(imb, meta['q'], cap0, hue, radius=60); cb = cb[0] if cb is not None else blob_near(imb, project_world(meta['q'], [cap0])[0])
if len(sys.argv) > 6 and sys.argv[4] == 'px':       # hand-marked run: never trust the automatic match in the second view either
    uvb = project_world(meta['q'], [cap0])[0]; x0 = int(max(0, uvb[0] - 200)); y0 = int(max(0, uvb[1] - 150))
    cv2.imwrite(RUN + '/reseat_b_zoom.jpg', cv2.resize(imb[y0:y0 + 300, x0:x0 + 400], None, fx=2.4, fy=2.4, interpolation=cv2.INTER_CUBIC))
    json.dump({'x0': x0, 'y0': y0, 'ca': c[0].tolist(), 'qa': qa, 'scale': 2.4}, open(RUN + '/reseat_b_crop.json', 'w'))
    print('reseat: second view taken - mark the cap in reseat_b_zoom.jpg'); sys.exit(4)
assert cb is not None, 'proud cap not found in the second view'
cap, rms = triangulate([(o, d), pixel_ray(meta['q'], cb)]); proud = cap[2] - slot[2]
print('reseat: cap at %s, ray rms %.1f mm, proud %.0f mm, lean xy mm %s' % (cap.round(3), rms * 1000, proud * 1000, ((cap[:2] - slot[:2]) * 1000).round(0)))
assert rms < 0.004 and 0.03 < proud < 0.13, 'cap position not credible'
set_box(box)
r = regrasp_cap(cap, box, top); assert r is not None, 'could not grip the cap'
st_, _ = cage_jiggle(slot, top, r[0], np.radians(25), float(cap[2]) - 0.012, ref_hue=hue, snap_prefix=RUN + '/reseat'); print('cage_jiggle:', st_)
gripper(0.45); q = np.array(state()['q']); Tt = tcp_pose(q)
back = follow([T(Tt[:3, :3], Tt[:3, 3] + [0, 0, dz]) for dz in (0.02, 0.04, 0.06)], q, tip_min=top + 0.004); assert back is not None, 'no retreat path'
run(back, 0.2, limit=0.08); print('reseat: backed off - verify from above')
