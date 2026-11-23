"""Marker sorting steps on the real R1-7A (see ../SKILL.md): restage -> pick_big_end -> carry -> lower_to_grid -> release ->
regrasp_cap -> cage_jiggle. Each step returns a status; look at the wrist image between steps. Units m, rad."""
import json, sys, numpy as np
from robot import rt
from robot.rt import *

PEN_LEN = 0.153          # measured from the wrist image (153-155 mm)
GRASP_D = float(__import__('os').environ.get('GRASP_D', 0.018))          # TCP distance from the big-cap end; fingers then cover ~0..38 mm of the pen (all on the cap)
FINGER_HALF = 0.020      # finger extent along the pen each side of the TCP (CAD)
TIP_ABOVE_TABLE = 0.004  # fingertip height when grasping a pen lying on the table (pen axis is 7.5 mm up); 2-3 mm poked the table in places (user, 2026-11-17)
PEN_INSET = 0.0075 - TIP_ABOVE_TABLE   # so the held pen axis sits this far behind the fingertip plane
BOX_MARGIN = 0.008       # finger underside above the neighbours' cap tops at release

# Closing command for the big-cap grasp. The pen stops the fingers at ~0.157, and the motor is a position loop (kp 5), so the
# grip torque is ~26 Nm x (0.157 - command): 0.13 gave only 0.7 Nm and the pen swung in the fingers on lift (5 of 12 picks,
# user diagnosis 2026-11-18); 0.08 gives ~2 Nm.
GRIP_CMD = 0.08

def frame(x, z):
    x = x / np.linalg.norm(x); z = z - (z @ x) * x; z /= np.linalg.norm(z); return np.c_[x, np.cross(z, x), z]

def grasp_pose(big_end, small_end, lift=0.0, dist=None, cam_to_big=True):
    """Top-down grasp of a lying pen, `dist` from the big end (default GRASP_D): approach = -table normal, closing axis
    across the pen, camera side (+z_tcp) towards the big end (needed for insertion) or away from it (for re-staging)."""
    n, p = table_plane(); u = np.asarray(big_end) - small_end; u -= (u @ n) * n; u /= np.linalg.norm(u)
    g = np.asarray(big_end) - (GRASP_D if dist is None else dist) * u; g = g - ((g - p) @ n) * n + (TIP_ABOVE_TABLE + lift) * n
    return T(frame(-n, u if cam_to_big else -u), g)

def holding(g, commanded):
    """Something is between the fingers: the motor stalls short of the commanded opening and keeps pushing.
    Measured: pen held -> opening 0.155 for command 0.13, tau -0.67; empty -> 0.137, tau -0.20."""
    return g['opening_fraction'] - commanded > 0.015 and abs(g['tau']) > 0.4

def lattice_headings(box):
    """Gripper headings (rad) for which a square pen held in the fingers is square to the slots."""
    a = np.array(box['a']); b = np.array(box['b'])
    base = np.mean([np.arctan2(a[1], a[0]) % (np.pi / 2), np.arctan2(b[1], b[0]) % (np.pi / 2)])
    return [(base + k * np.pi / 2 + np.pi) % (2 * np.pi) - np.pi for k in range(4)]

def plan_slot(slot_top, box_top, box, q_from):
    """Level-gripper descent over a slot at a lattice-aligned heading. Picks the branch with the least elbow twist whose
    whole descent is continuous and whose top is reachable from q_from with the held pen clear."""
    z_hi = box_top + (PEN_LEN - GRASP_D) + 0.015; z_lo = box_top + FINGER_HALF + BOX_MARGIN; zs = np.arange(z_hi, z_lo - 1e-6, -0.008)
    best = None
    for hd in lattice_headings(box):
        # the low-twist solutions come first but often run J6 into its limit on the way down: look at many, not five
        for q_top in ik_all(slot_pose(slot_top, hd, z_hi), 80)[:12]:
            if not held_clear(q_top, box_top): continue
            if not all(held_clear(q_from + s * (q_top - q_from), box_top) for s in np.linspace(0, 1, 30)): continue
            qs = follow([slot_pose(slot_top, hd, z) for z in zs[1:]], q_top, max_step=0.2, tip_min=box_top + 0.004)
            if qs is None: continue
            # steady-state corrections need room: a branch that runs along a joint limit stalls half way down (2026-11-18, J2 at 0)
            margin = min(np.min(np.minimum(q - MODEL.lower, MODEL.upper - q)) for q in [q_top] + qs)
            if margin < 0.05: continue
            cost = max(abs(q[2]) + 0.5 * abs(q[4]) for q in [q_top] + qs) + 3.0 * max(0.0, 0.15 - margin)      # prefer room to the limits
            if best is None or cost < best['cost']: best = dict(cost=float(cost), heading=float(hd), qs=[q_top] + qs, zs=zs.tolist())
    return best

def pen_points(Tw_tcp):
    """Held pen as points (big end ... small end) in world, for clearance checks."""
    return [(Tw_tcp @ [-PEN_INSET, 0, s, 1])[:3] for s in np.linspace(GRASP_D, GRASP_D - PEN_LEN, 7)]

def slot_pose(slot_top, heading, tcp_z):
    """Gripper horizontal, camera side up, pen hanging over the slot. slot_top: world xyz of the slot at cap-top level."""
    n, _ = table_plane(); h = np.array([np.cos(heading), np.sin(heading), 0.]); R = frame(h - (h @ n) * n, n)
    t = np.array([slot_top[0], slot_top[1], tcp_z]) + R @ [PEN_INSET, 0, 0]                                   # pen axis, not the fingertip, over the slot
    return T(R, t)

def held_clear(q, box_top):
    """clear() plus the hanging pen: above the table and never inside the box volume below the cap tops (+1 cm)."""
    if not clear(q, 0.02): return False
    pts = fk(np.asarray(q, float) + DELTA)[1]
    for p in pen_points(tcp_pose(q)):
        if p[2] < 0.015: return False
        if min(rt._seg_dist(p, pts[i], pts[i + 1]) for i in range(1, 5)) < 0.06: return False     # tail of the pen vs base/upper arm
        for lo, hi in rt.OBSTACLES:
            if np.all(p[:2] >= lo[:2]) and np.all(p[:2] <= hi[:2]) and p[2] < box_top + 0.010: return False
    return True

def set_box(box):
    """Keep-out volumes from the fitted lattice: box body up to just above the cap tops, and the open lid beside row j=0."""
    P = np.array(box['P']); b = np.array(box['b']); top = float(P[:, 2].max())
    lo, hi = P[:, :2].min(0) - 0.022, P[:, :2].max(0) + 0.022
    rt.OBSTACLES[:] = [(np.r_[lo, -0.05], np.r_[hi, top + 0.004])]
    out = -b[:2] / np.linalg.norm(b[:2])                                     # lid hangs outside row j=0
    pts = np.vstack([P[[0, 5], :2] + out * 0.022, P[[0, 5], :2] + out * 0.14])
    rt.OBSTACLES.append((np.r_[pts.min(0) - 0.02, -0.05], np.r_[pts.max(0) + 0.02, top + 0.02]))
    return top

def cap_in_hand(jpg, hue=None):
    """Held pen's end cap in front of the camera: (pixel centroid, area, hue). The cap sits at a fixed place in the image
    (~620, 388); if the pen is pushed up through the fingers during insertion the centroid moves up / the area changes.
    hue=None learns the cap hue from the patch at that place; pass it back in later so the background cannot leak in."""
    import cv2
    im = cv2.imdecode(np.frombuffer(jpg, np.uint8), 1)
    if hue == 'dark':                                                             # black cap: only the strip between the (black) fingers
        x0, y0 = 590, 300; m = (cv2.cvtColor(im[y0:450, x0:656], cv2.COLOR_BGR2HSV)[..., 2] < 75).astype(np.uint8)
        n, lab, stats, cen = cv2.connectedComponentsWithStats(m)
        if n < 2: return None
        k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        if stats[k, cv2.CC_STAT_AREA] < 400: return None
        return np.array([x0 + cen[k][0], y0 + cen[k][1]]), int(stats[k, cv2.CC_STAT_AREA]), 'dark'
    x0, y0 = 540, 300; hsv = cv2.cvtColor(im[y0:460, x0:720], cv2.COLOR_BGR2HSV).astype(int)
    if hue is None: hue = (int(np.median(hsv[80:96, 72:88, 0])), max(110, int(0.75 * np.median(hsv[80:96, 72:88, 1]))))
    if isinstance(hue, int): hue = (hue, 110)
    dh = np.abs(hsv[..., 0] - hue[0]); dh = np.minimum(dh, 180 - dh)
    m = ((dh < 9) & (hsv[..., 1] > hue[1]) & (hsv[..., 2] > 80)).astype(np.uint8)     # s_min keeps the beige table out
    n, lab, stats, cen = cv2.connectedComponentsWithStats(m)
    if n < 2: return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))                       # the cap is the biggest blob of that colour
    if stats[k, cv2.CC_STAT_AREA] < 400: return None
    return np.array([x0 + cen[k][0], y0 + cen[k][1]]), int(stats[k, cv2.CC_STAT_AREA]), hue


def upright_grip(axis_pt, phi, beta=0.0, inset=0.015):
    """Grip on a pen standing upright: closing axis horizontal at angle phi (world), approach pitched down by beta
    (25 deg is far more reachable than level), pen `inset` m inside the fingertips (5 mm lets it squirt out).
    axis_pt = point on the pen axis at the finger centre. A square pen squares up to the fingers -> faces at phi mod 90."""
    y = np.array([np.cos(phi), np.sin(phi), 0.]); h = np.array([np.sin(phi), -np.cos(phi), 0.])   # camera side up; heading = phi - 90 deg
    x = np.cos(beta) * h - np.sin(beta) * np.array([0, 0, 1.])
    return T(np.c_[x, y, np.cross(x, y)], np.asarray(axis_pt, float) + inset * x)

def servo_point(f, q_nom, off, target, tol=0.0015, iters=4, speed=0.1, axes=(0, 1)):
    """Drive the MEASURED world point f(q) to `target` by correcting the joint command through the point Jacobian
    (outer integral loop; the PD controller alone leaves 2-8 mm). Returns (measured q, offset, remaining error)."""
    J = np.array([(f(q_nom + d) - f(q_nom - d)) / 2e-4 for d in np.eye(7) * 1e-4]).T
    Jp = np.linalg.pinv(J); off = np.asarray(off, float).copy(); axes = list(axes)
    for _ in range(iters):
        qm = move(np.clip(q_nom + off, MODEL.lower, MODEL.upper), speed, settle=0.3); e = np.asarray(target) - f(qm)
        if np.linalg.norm(e[axes]) < tol: break
        off += Jp @ np.clip(e, -0.006, 0.006)
    return qm, off, e

def follow(poses, q_start, max_step=0.3, tip_min=0.02):
    """Seeded IK along closely spaced TCP poses (stays on q_start's branch). Returns joint commands or None."""
    out = [np.asarray(q_start, float)]
    for P in poses:
        Tw7 = P @ np.linalg.inv(link7_tcp())
        r = MODEL.solve(Tw7[:3, 3], Tw7[:3, :3], out[-1] + DELTA, realtime=True)
        q = np.array(r['q']) - DELTA
        if not r['success'] or np.abs(q - out[-1]).max() > max_step or not clear(q, tip_min): return None
        out.append(q)
    return out[1:]

def run(qs, speed=0.15, names=None, limit=0.035):
    """Execute joint commands one by one with the steady-state error of the first step carried as a constant offset.
    Stops (returns False) if the tracking error grows by more than `limit` rad over the first step's - i.e. contact."""
    base = None
    for k, q in enumerate(qs):
        qm = move(q, speed, settle=0.3); e = np.array(state()['command_q']) - qm
        if base is None: base = e
        if np.abs(e - base).max() > limit:
            print('  contact? tracking error changed by', np.abs(e - base).max().round(3), 'at step', k); return False
    return True


def mid_pen_clear(q, margin=0.012, span=PEN_LEN / 2):
    """Pen held away from its ends (restage): up to `span` m sticks out on each side of the fingers. True if no point of it
    is inside a keep-out box. Without this the pen's tail swept into the pen box while turning (2026-11-17)."""
    Tt = tcp_pose(q)
    for sc in np.linspace(-span, span, 7):
        pt = (Tt @ [-PEN_INSET, 0, sc, 1])[:3]
        if any(np.all(pt >= lo - margin) and np.all(pt <= hi + margin) for lo, hi in rt.OBSTACLES): return False
    return True

def fingers_clear(G, others, opening=0.7):
    """Open fingers of a top-down grasp G must not land on another pen lying nearby. others: [ends_world] of the other pens."""
    for sgn in (-1, 1):
        for y in (0.029, 0.035, 0.041):                                                   # finger pad, inner to outer face
            for z in (-0.02, 0.0, 0.02):
                pt = (G @ [0, sgn * (y - 0.029 + opening * 0.041), z, 1])[:2]
                for e in others:
                    if rt._seg_dist(pt, np.array(e[0])[:2], np.array(e[1])[:2]) < 0.0075 + 0.006: return False
    return True

def restage(pen, big_end, centre, cam_big, q0, grip_cmd=0.13, log=print, snap_prefix=None, others=(), avoid=None):
    """Move a lying pen to `centre`, lying along world x with the big end at -x (there the big-end grasp with the camera
    side towards the big end is comfortable). Picks near the pen middle; with `others` (ends_world of nearby pens) and
    `avoid` (world point of a crossing) the grasp point slides along the pen until the fingers are clear of the other pens
    and >= 4 cm from the crossing - this is how the top pen of a stack is taken off. Returns True when released."""
    big, small = np.array(pen['ends_world'][big_end]), np.array(pen['ends_world'][1 - big_end]); mid = PEN_LEN / 2
    sb = np.r_[centre[0] - mid, centre[1], 0.0]; ss = np.r_[centre[0] + mid, centre[1], 0.0]
    u = (big - small) / np.linalg.norm(big - small); plan = None
    for g in sorted(np.arange(0.04, PEN_LEN - 0.035, 0.01), key=lambda g: abs(g - mid)):
        if avoid is not None and np.linalg.norm((big - g * u)[:2] - np.asarray(avoid)[:2]) < 0.04: continue
        if not fingers_clear(grasp_pose(big, small, 0.0, g, cam_big), others): continue
        span = max(g, PEN_LEN - g)
        for qp in ik_all(grasp_pose(big, small, 0.07, g, cam_big), 40):
            if not clear(qp, 0.05) or not path_clear(q0, qp, 0.05): continue
            down = follow([grasp_pose(big, small, l, g, cam_big) for l in (0.04, 0.015, 0.0)], qp, tip_min=-0.005)
            up = down and follow([grasp_pose(big, small, l, g, cam_big) for l in (0.01, 0.02, 0.04, 0.065, 0.09)], down[-1], max_step=0.2, tip_min=-0.005)
            if not up: continue
            for qs in ik_all(grasp_pose(sb, ss, 0.09, g, cam_big), 40):
                if not clear(qs, 0.05) or not path_clear(up[-1], qs, 0.05): continue
                if not all(mid_pen_clear(up[-1] + t * (qs - up[-1]), span=span) for t in np.linspace(0, 1, 30)) or not all(mid_pen_clear(q, span=span) for q in up): continue
                put = follow([grasp_pose(sb, ss, l, g, cam_big) for l in (0.05, 0.02, 0.004)], qs, tip_min=-0.005)
                off = put and follow([grasp_pose(sb, ss, l, g, cam_big) for l in (0.04, 0.09)], put[-1], tip_min=-0.005)
                if off: plan = (qp, down, up, qs, put, off); break
            if plan: break
        if plan: log('restage: grasp %.0f mm from the big end' % (g * 1000)); break
    if plan is None: log('restage: no plan'); return False
    qp, down, up, qs, put, off = plan
    gripper(0.7); goto(qp, 0.4, tip_min=0.05)
    run(down[:-1], 0.2); move(down[-1], 0.15)
    g = gripper(grip_cmd); log('restage: closed, opening %.3f tau %.2f holding %s' % (g['opening_fraction'], g['tau'], holding(g, grip_cmd)))
    if not holding(g, grip_cmd): gripper(0.7); run(up, 0.2); return False
    run(up, 0.15, limit=0.08); g = state()['gripper']
    if snap_prefix: snap(snap_prefix + '_lift')
    if not holding(g, grip_cmd): log('restage: lost the pen while lifting'); return False
    move(qs, 0.35); run(put, 0.15); gripper(0.5); time.sleep(0.5)
    run(off, 0.2); gripper(1.0)
    if snap_prefix: snap(snap_prefix + '_done')
    return True


def pick_big_end(pen, big_end, q0, grip_cmd=GRIP_CMD, log=print, ok_after=None, others=(), avoid=None):
    """Top-down grasp GRASP_D from the big end with the camera side towards it, lift 12 cm. Returns measured q or None."""
    big, small = np.array(pen['ends_world'][big_end]), np.array(pen['ends_world'][1 - big_end])
    G0 = grasp_pose(big, small, 0.0)                                         # pens touching/crossing: stay off the other pens and the crossing
    # crowded pens: approach with the fingers opened only as far as needed (0.5 = ~1 cm clearance each side of the pen)
    opening = next((o for o in (0.7, 0.5) if fingers_clear(G0, others, o)), None)
    if opening is None or (avoid is not None and np.linalg.norm(G0[:2, 3] - np.asarray(avoid)[:2]) < 0.04):
        log('pick: fingers would land on another pen or the crossing'); return None
    if opening < 0.7: log('pick: crowded, approaching with the gripper opened to %.1f' % opening)
    for qp in ik_all(grasp_pose(big, small, 0.07), 40):
        if not clear(qp, 0.05) or not path_clear(q0, qp, 0.05): continue
        down = follow([grasp_pose(big, small, l) for l in (0.04, 0.015, 0.0)], qp, tip_min=-0.005)
        # small steps first: joint interpolation between far-apart waypoints dips the far end of the pen into the table (it then swings in the fingers)
        up = down and follow([grasp_pose(big, small, l) for l in (0.01, 0.02, 0.03, 0.045, 0.06, 0.08, 0.10, 0.12)], down[-1], max_step=0.15, tip_min=-0.005)
        if not up or (ok_after and not ok_after(up[-1])): continue
        gripper(opening); goto(qp, 0.4, tip_min=0.05)
        assert run(down[:-1], 0.2), 'pick: resistance during approach; inspect the current pose'
        move(down[-1], 0.15)
        g = gripper(grip_cmd); log('pick: closed, opening %.3f tau %.2f holding %s' % (g['opening_fraction'], g['tau'], holding(g, grip_cmd)))
        if not holding(g, grip_cmd): gripper(opening); run(up, 0.2); return None
        assert run(up, 0.15, limit=0.06), 'pick: resistance while lifting; inspect the pen before moving again'
        if not holding(state()['gripper'], grip_cmd): log('pick: lost the pen while lifting'); return None
        return np.array(state()['q'])
    log('pick: no plan'); return None


def servo_pen_bottom(q_nom, off, target, tol=0.0015, iters=4, speed=0.1):
    """servo_point for the lower end of a pen held from the table grasp."""
    return servo_point(lambda q: pen_points(tcp_pose(q))[-1], q_nom, off, target, tol, iters, speed)


def cage_jiggle(slot_top, box_top, phi, beta, z_grip, ref_hue=None, log=print, snap_prefix='/tmp/cage', inset=0.015, cage=0.21):
    """Human-style seating of an upright pen: the fingers loosely cage its cap (the pen can slide down and wobble) and the
    gripper jiggles in small circles / yaw; only gravity pushes the pen down, so nothing jams.
    Returns ('dropped' | 'gone' | 'stuck' | 'no_ik', cap colour key). 'dropped' = the cap left its place in the wrist image."""
    f = lambda q: (tcp_pose(q) @ [-inset, 0, 0, 1])[:3]
    G = lambda dx=0., dy=0., dp=0.: upright_grip([slot_top[0] + dx, slot_top[1] + dy, z_grip], phi + dp, beta, inset)
    nom = follow([G()], np.array(state()['q']), tip_min=box_top + 0.004)
    if nom is None: return 'no_ik', ref_hue
    qm, off, e = servo_point(f, nom[0], np.zeros(7), np.r_[slot_top[:2], z_grip])
    if ref_hue is None:
        c = cap_in_hand(snap(snap_prefix + '_held')[0]); ref_hue = None if c is None else c[2]
    gripper(cage); time.sleep(0.8)
    ref = cap_in_hand(snap(snap_prefix + '_caged')[0], ref_hue)
    if ref is None: return 'gone', ref_hue
    log('cage: grip-centre xy err mm %s, cap at %s area %d' % ((e[:2] * 1000).round(1), ref[0].round(0), ref[1]))
    seq = []
    for r in (0.002, 0.004, 0.006):
        seq += [(r * np.cos(a), r * np.sin(a), 0.) for a in np.arange(0, 2 * np.pi, np.pi / 4)] + [(0, 0, 0.14), (0, 0, -0.14), (0, 0, 0.)]
    for k, (dx, dy, dp) in enumerate(seq):
        nx = follow([G(dx, dy, dp)], nom[0], tip_min=box_top + 0.004)
        if nx is None: continue
        move(nx[0] + off, 0.25, settle=0.1)
        c = cap_in_hand(snap(snap_prefix + '_j')[0], ref_hue)
        if c is None or c[0][1] - ref[0][1] > 40 or c[1] < 0.5 * ref[1]:
            log('cage: pen dropped at jiggle %d (%+.0f, %+.0f mm, yaw %+.0f deg)' % (k, dx * 1000, dy * 1000, np.degrees(dp)))
            return 'dropped', ref_hue
    return 'stuck', ref_hue


def lower_to_grid(slot_top, box, box_top, log=print):
    """Held pen over the slot at a lattice-aligned heading, lowered until its tip meets the hole plate inside the box
    (arm held up > 5 mm) or the fingers reach the caps. The tip xy is servoed only while it is still above the caps;
    it must merely enter the shaft between the neighbours (+-5 mm), not the hole."""
    pl = plan_slot(slot_top, box_top, box, np.array(state()['q']))
    if pl is None: return None
    qs = pl['qs']; zs = [pen_points(tcp_pose(q))[-1][2] for q in qs]; off = np.zeros(7); move(qs[0], 0.3)
    for k in range(len(qs)):
        tgt = np.r_[slot_top[:2], zs[k]]
        if zs[k] > box_top - 0.012: qm, off, e = servo_pen_bottom(qs[k], off, tgt)
        else:
            cmd = qs[k] + off
            assert np.all(cmd >= MODEL.lower) and np.all(cmd <= MODEL.upper), 'lower: corrected joint target outside limits; inspect current pose'
            qm = move(cmd, 0.1, settle=0.3); e = tgt - pen_points(tcp_pose(qm))[-1]
        if e[2] < -0.005: break
    log('lower: heading %.0f deg, stopped at step %d/%d, pen bottom %.0f mm below the caps, held up %.1f mm' % (
        np.degrees(pl['heading']), k, len(qs) - 1, 1000 * (box_top - zs[k] + e[2]), -1000 * e[2]))
    return dict(heading=pl['heading'], qs=qs, k=k, off=off, grip_z=float((tcp_pose(qm) @ [-PEN_INSET, 0, 0, 1])[2]))


def regrasp_cap(cap_top, box, box_top, beta=np.radians(25), log=print):
    """Grip the cap of a pen standing/leaning in its slot: gripper pitched down by beta, heading square to the slot lattice,
    pen 15 mm inside the fingers. cap_top = world xyz of the cap end face (triangulated). Returns (phi, approach joints) or None.
    (Same logic as the inline code that worked for pen 35 on 2026-11-17; not yet re-run as a function.)"""
    ax = np.asarray(cap_top, float) - [0, 0, 0.012]; q0 = np.array(state()['q']); best = None
    for hd in lattice_headings(box):
        G = upright_grip(ax, hd + np.pi / 2, beta); back = lambda d: T(G[:3, :3], G[:3, 3] - d * G[:3, 0])
        for qp in ik_all(back(0.08), 40)[:6]:
            if not clear(qp, box_top + 0.03) or not path_clear(q0, qp, box_top + 0.03): continue
            a = follow([back(d) for d in (0.06, 0.04, 0.02, 0.0)], qp, tip_min=box_top + 0.02)
            if a is None: continue
            cost = abs(a[-1][2]) + 0.5 * abs(a[-1][4])
            if best is None or cost < best[0]: best = (cost, hd + np.pi / 2, qp, a)
    if best is None: return None
    _, phi, qp, a = best
    gripper(0.45); goto(qp, 0.35, tip_min=box_top + 0.03, trim=False); run(a, 0.15)
    g = gripper(0.13); log('regrasp: opening %.3f tau %.2f holding %s' % (g['opening_fraction'], g['tau'], holding(g, 0.13)))
    return (phi, [qp] + a) if holding(g, 0.13) else None


def hangs_straight(jpg):
    """ONLY in the carry pose: a pen hanging straight down fills the gap between the fingers at the bottom of the wrist
    image. Two tests, both must pass: (kraft paper) that strip is unsaturated, not table-coloured; (white desk, where the
    table is as white as the pen) the strip shows the pen's shading across its width (column std 11-23), while the bare
    desk is flat (0.6). Calibrated on 2 straight + 1 swung frame per test; a swung pen went into the box unnoticed on
    2026-11-18 before the second test existed."""
    import cv2
    im = cv2.imdecode(np.frombuffer(jpg, np.uint8), 1)
    unsat = float((cv2.cvtColor(im[660:715, 600:650], cv2.COLOR_BGR2HSV)[..., 1] < 60).mean()) > 0.5
    # straight pens measured 3.6-5.8 (blue ones lowest), the bare desk 0.6; 4.0 gave two false alarms on blue pens
    shaded = float(cv2.cvtColor(im[640:700, 605:645], cv2.COLOR_BGR2GRAY).astype(float).mean(0).std()) > 2.0
    return unsat and shaded

def put_down_swung(xy, towards, q0, log=print):
    """Release a pen that swung in the fingers: fingers pointing down 7 cm above the table at xy; the pen's tail lands
    roughly in direction `towards` (unit xy). Returns True when released and the hand is back up."""
    best = None
    for yaw in np.radians(np.arange(0, 360, 45)):
        G = down_pose([xy[0], xy[1], 0.15], yaw); score = float(-G[:2, 2] @ np.asarray(towards))
        if best is not None and score <= best[0]: continue
        for qp in ik_all(G, 25)[:4]:
            if clear(qp, 0.13) and path_clear(q0, qp, 0.13):
                dn = follow([down_pose([xy[0], xy[1], z], yaw) for z in (0.13, 0.11, 0.09, 0.07)], qp, tip_min=0.05)
                if dn: best = (score, qp, dn); break
    if best is None: log('put_down_swung: no plan'); return False
    _, qp, dn = best; move(qp, 0.3)
    for qd in dn:
        qm = move(qd, 0.1, settle=0.3)
        if tcp_pose(qm)[2, 3] - tcp_pose(qd)[2, 3] > 0.006: break               # pen touched the table
    gripper(0.7); time.sleep(0.7); run(dn[::-1][1:] + [qp], 0.2, limit=0.1); gripper(1.0)
    return True


def find_cap(im, q, guess, hue, radius=220):
    """Pixel centre of the cap of a pen standing proud of the box: the largest COMPACT blob of the cap colour near the
    projected guess (a proud cap is closer to the camera, so bigger than same-coloured caps of seated pens; a leaning
    pen can put it far from the slot, and loose colour blobs merge - hence the square/solid test)."""
    import cv2
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV).astype(int)
    if hue == 'dark':                                                             # black / dark caps: no usable hue
        m = (hsv[..., 2] < 75).astype(np.uint8)
    else:
        dh = np.abs(hsv[..., 0] - hue[0]); dh = np.minimum(dh, 180 - dh)
        m = ((dh < 8) & (hsv[..., 1] > hue[1]) & (hsv[..., 2] > 60)).astype(np.uint8)      # V > 90 cut the dark green 47 cap into pieces
    n, lab, stats, cen = cv2.connectedComponentsWithStats(cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)))
    g = project_world(q, [guess])[0]; best = None
    for k in range(1, n):
        area = stats[k, cv2.CC_STAT_AREA]
        if area < 300 or area > 8000 or np.linalg.norm(cen[k] - g) > radius: continue
        c = max(cv2.findContours((lab == k).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], key=cv2.contourArea)
        (w, h) = cv2.minAreaRect(c)[1]                                            # rotated box: an upright box under-fills a turned square cap
        if min(w, h) < 1 or not 0.6 < w / h < 1.6 or area / (w * h) < 0.6: continue
        if best is None or area > best[1]: best = (cen[k], int(area))
    return best
