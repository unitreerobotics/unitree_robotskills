"""Direct pick of pen K (index in RUN/ov_pens.json) - the version that worked on 2026-11-18 run13:
searches gripper yaw offsets and grasp distance, servoes the fingertip height before closing, lifts, flips to the carry
pose, checks the pen hangs straight. usage: python3 -m sorting.pick_direct RUN K [--dry]   exit 0 ok, 2 no plan / not holding, 3 swung."""
import sys, json, cv2
from robot import rt
from sorting import task
from robot.rt import *
from sorting.task import *
RUN, K, dry = sys.argv[1], int(sys.argv[2]), '--dry' in sys.argv
wide, alone = '--wide' in sys.argv, '--alone' in sys.argv      # --wide: larger gripper turns (the pen is swung round on the table as the fingers close; only with nothing near it)
if not dry: sys.stdout = EventLog(RUN + '/events.jsonl', 'pick %d (direct)' % K)
box = json.load(open(RUN + '/box.json')); top = set_box(box)
s = state(); assert s['armed'] and not s['fault'] and not s['busy'], s['fault']; q0 = np.array(s['q'])
allp = json.load(open(RUN + '/ov_pens.json')); pen = allp[K]; others = [p['ends_world'] for p in allp if p is not pen]
be = pen['big_end']; assert be is not None, 'big end unknown'
big, small = np.array(pen['ends_world'][be]), np.array(pen['ends_world'][1 - be]); n, pt = table_plane()
CARRY = np.array([0, 0.6, 0, 0.6, 0, 0, 0.]) - DELTA; VIEW = np.array([0.000, 1.766, 0.000, 1.225, -1.465, -0.079, -1.029]) - DELTA
OVER = np.array([0.07, 0.156, 0.004, 0.222, -0.007, -0.572, 0.055])
def G(lift, d, ang):                      # top-down grasp turned by ang about the vertical through the grasp point
    P = grasp_pose(big, small, lift, d); P[:3, :3] = cv2.Rodrigues(n * ang)[0] @ P[:3, :3]; return P
def route(a, b):
    for via in ([], [OVER], [VIEW], [OVER, VIEW]):
        pts = [a] + via + [b]
        if all(path_clear(pts[i], pts[i + 1], 0.05) for i in range(len(pts) - 1)): return via
    return None
LS = [0.015 - 0.002 * i for i in range(1, 12)]        # touch-down probe: nominal fingertip height 17 mm ... -3 mm, 2 mm steps
boxc = np.array(box['P'])[:, :2].mean(0)
def carry_route(q_up, P_up):
    """Joint waypoints from the lifted pose to CARRY with the held pen clear: direct, or first sideways away from the box
    (a pen picked right next to the box would otherwise sweep through it while flipping)."""
    hc = lambda a, b: all(held_clear(a + t * (b - a), top) for t in np.linspace(0, 1, 30))
    if hc(q_up, CARRY): return []
    away = P_up[:2, 3] - boxc; away = np.r_[away / np.linalg.norm(away), 0.0]
    for sft in (0.05, 0.08, 0.12):
        for dz in (0.0, 0.04):
            Ps = P_up.copy(); Ps[:3, 3] += sft * away + [0, 0, dz]
            qs = follow([Ps], q_up, max_step=0.6, tip_min=0.02)
            if qs and hc(q_up, qs[0]) and hc(qs[0], CARRY): return [qs[0]]
    return None
plan = None
if dry: q0 = OVER
if alone: others = []
for ang in np.radians([0, 10, -10, 20, -20, 30, -30] + ([45, -45, 60, -60] if wide else [])):
    for d in (0.018, 0.03, 0.045) + ((0.06,) if wide else ()):
        task.GRASP_D = d
        opening = next((o for o in (0.7, 0.5) if fingers_clear(G(0.0, d, ang), others, o)), None)
        if opening is None: continue
        for qp in ik_all(G(0.07, d, ang), 60, extra_seeds=[VIEW]):
            if not clear(qp, 0.05): continue
            via = route(q0, qp)
            if via is None: continue
            down = follow([G(l, d, ang) for l in (0.04, 0.015, 0.0)], qp, tip_min=-0.005)
            up = down and follow([G(l, d, ang) for l in (0.01, 0.02, 0.03, 0.045, 0.06, 0.08, 0.10, 0.12)], down[-1], max_step=0.15, tip_min=-0.005)
            probe = up and follow([G(l, d, ang) for l in LS], down[1], max_step=0.1, tip_min=-0.012)
            if not probe: continue
            if min(np.min(np.minimum(q_ - MODEL.lower, MODEL.upper - q_)) for q_ in [qp] + down + up + probe) < 0.10: continue   # stay off the joint limits
            cr = carry_route(up[-1], G(0.12, d, ang))
            if cr is not None: plan = (d, ang, opening, via, qp, down, up, probe, cr); break
        if plan: break
    if plan: break
if plan is None: print('pick %d: no plan' % K); sys.exit(2)
d, ang, opening, via, qp, down, up, probe, cr = plan
print('pick %d: gripper turned %+.0f deg, %.0f mm from the big end, opening %.1f%s' % (K, np.degrees(ang), d * 1000, opening, ', sideways before flipping' if cr else ''))
if dry: sys.exit(0)
open(RUN + '/grasp_d.txt', 'w').write(str(d))
gripper(opening)
for w in via: move(w, 0.35)
goto(qp, 0.4, tip_min=0.05); run(down[:2], 0.2)
# Drive the computed fingertip height to the target, but stop as soon as the arm starts pushing (tracking error grows):
# the computed height is only good to several mm, so "too high" often means the fingers already rest on the table.
# Resting on the table is fine for the grasp (user, 2026-11-18); pushing harder is what must not happen.
tgt = G(0.0, d, ang)[:3, 3]; off = np.zeros(7); track0 = None; why = 'height reached'
for k in range(6):
    qm, off, e = servo_point(lambda q: tcp_pose(q)[:3, 3], down[-1], off, tgt, tol=0.0015, iters=1, axes=(0, 1, 2))
    track = float(np.abs(np.array(state()['command_q']) - qm).max()); track0 = track if track0 is None else track0
    if abs(e @ n) < 0.0015: break
    if track - track0 > 0.02: why = 'fingers on the table'; break
print('pick: computed fingertip height %.1f mm (%s)' % (1000 * ((tcp_pose(qm)[:3, 3] - pt) @ n), why))
g = gripper(GRIP_CMD); print('pick: closed, opening %.3f tau %.2f holding %s' % (g['opening_fraction'], g['tau'], holding(g, GRIP_CMD)))
if not holding(g, GRIP_CMD): gripper(opening); run(up, 0.2, limit=0.1); sys.exit(2)
run(up, 0.15, limit=0.08)
for w in cr: move(w, 0.25)
move(CARRY, 0.3); g = state()['gripper']; assert holding(g, GRIP_CMD), 'lost the pen'
jpg, _ = snap(RUN + '/carry'); c = cap_in_hand(jpg); cap = pen.get('caps_hsv', [None, None])[be]
json.dump({'hue': list(c[2]) if c else ([cap[0], 110] if cap else None), 'grasp_d': d}, open(RUN + '/held.json', 'w'))
im = cv2.imread(RUN + '/carry.jpg'); cv2.imwrite(RUN + '/carry_zoom.jpg', cv2.resize(im[300:450, 540:720], None, fx=5, fy=5, interpolation=cv2.INTER_CUBIC))
ok = hangs_straight(jpg); print('holding; pen hangs straight:', ok); sys.exit(0 if ok else 3)
