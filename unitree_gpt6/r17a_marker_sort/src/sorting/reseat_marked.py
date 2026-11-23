"""Finish a reseat whose cap was marked by hand in both photos (dark caps).
usage: python3 -m sorting.reseat_marked RUN I J ZU ZV   (ZU, ZV = cap centre in RUN/reseat_b_zoom.jpg; the first photo's pixel is in reseat_b_crop.json)"""
import sys, json, cv2
from robot import rt
from sorting import task
from robot.rt import *
from sorting.task import *
RUN, I, J, ZU, ZV = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])
sys.stdout = EventLog(RUN + '/events.jsonl', 'reseat %d %d (cap marked by hand)' % (I, J))
box = json.load(open(RUN + '/box.json')); P = np.array(box['P']); top = set_box(box); slot = P[J * 6 + I]
s = state(); assert s['armed'] and not s['fault'] and not s['busy'], s['fault']
cr = json.load(open(RUN + '/reseat_b_crop.json')); qb = json.load(open(RUN + '/reseat_b.json'))['q']
cap, rms = triangulate([pixel_ray(cr['qa'], cr['ca']), pixel_ray(qb, [cr['x0'] + ZU / cr['scale'], cr['y0'] + ZV / cr['scale']])]); proud = cap[2] - slot[2]
print('reseat: cap at %s, ray rms %.1f mm, proud %.0f mm, lean xy mm %s' % (cap.round(3), rms * 1000, proud * 1000, ((cap[:2] - slot[:2]) * 1000).round(0)))
assert rms < 0.004 and 0.03 < proud < 0.13, 'cap position not credible'
r = regrasp_cap(cap, box, top); assert r is not None, 'could not grip the cap'
st_, _ = cage_jiggle(slot, top, r[0], np.radians(25), float(cap[2]) - 0.012, ref_hue='dark', snap_prefix=RUN + '/reseat'); print('cage_jiggle:', st_)
gripper(0.45); q = np.array(state()['q']); Tt = tcp_pose(q)
back = follow([T(Tt[:3, :3], Tt[:3, 3] + [0, 0, dz]) for dz in (0.02, 0.04, 0.06)], q, tip_min=top + 0.004); assert back is not None, 'no retreat path'
run(back, 0.2, limit=0.08); print('reseat: backed off - verify from above')
