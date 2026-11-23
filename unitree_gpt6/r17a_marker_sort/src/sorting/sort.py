"""Step driver for one round (run in the foreground, one step per call, look at the images in between).
  python3 -m sorting.sort RUN look [J1]            -> overview photo + pen list (RUN/ov.jpg, RUN/ov_pens.json); J1 turns the view (0.45 = right side)
  python3 -m sorting.sort RUN addpen U1 V1 U2 V2   -> pen the detector cannot find (dark cap on the white desk, one end hidden): pixel of its
                                      big-cap end and of any other point on its axis in RUN/ov.jpg; the rest follows from the pen length
  (K may be @ = the pen lying on the staging spot)
  python3 -m sorting.sort RUN pick K               -> pick pen K near its big cap, flip to the carry pose, photo of the cap code
  python3 -m sorting.sort RUN restage K            -> move pen K to the staging spot (when pick says 'no plan')
  python3 -m sorting.sort RUN place I J            -> lower the held pen into slot (I, J) until it meets the grid or the caps, release, back off
  python3 -m sorting.sort RUN reseat I J           -> pen standing proud in slot (I, J): locate its cap from 2 views, grip it firmly, hold it
                                      upright over the slot centre, loosen to a cage and jiggle until it drops
  python3 -m sorting.sort RUN verify               -> top view of the box (RUN/verify.jpg)
RUN must contain box.json (perception.box) and box_views.json (perception.view_plan)."""
import json, sys, time, cv2
from robot import rt
from sorting import task
from robot.rt import *
from sorting.task import *

RUN, step, args = sys.argv[1], sys.argv[2], sys.argv[3:]
sys.stdout = EventLog(RUN + '/events.jsonl', ' '.join(sys.argv[2:]))      # timestamped copy of everything printed
OVER = np.array([0.07, 0.156, 0.004, 0.222, -0.007, -0.572, 0.055]); CARRY = np.array([0, 0.6, 0, 0.6, 0, 0, 0.]) - DELTA
STAGE = (0.06, -0.20)
box = json.load(open(RUN + '/box.json')); P = np.array(box['P']); top = set_box(box)
s = state(); assert s['armed'] and not s['fault'] and not s['busy'], s['fault']
q0 = np.array(s['q'])

def to(q, tip_min, speed=0.35, what=''):
    a = np.array(state()['q']); assert path_clear(a, q, tip_min), 'path not clear: ' + what
    return move(q, speed)

if step == 'look':
    v = OVER.copy(); v[0] = float(args[0]) if args else OVER[0]          # optional J1 angle: pens hidden behind a finger
    if s['gripper']['opening_fraction'] < 0.95 and not holding(s['gripper'], GRIP_CMD): gripper(1.0)   # half-open fingers hide pens
    to(v, 0.05, what='overview'); snap(RUN + '/ov')
    from perception import pens
    found = pens.find_pens(cv2.imread(RUN + '/ov.jpg'), json.load(open(RUN + '/ov.json'))['q'])
    json.dump(found, open(RUN + '/ov_pens.json', 'w'))
    for k, p in enumerate(found):
        print(k, 'len %.0f mm' % (p['length_m'] * 1000), 'big end', p['big_end'], np.round(p['ends_world'], 3).tolist(),
              ''.join('  CROSSES pen %d' % c['other'] for c in p.get('crossings', [])))

elif step in ('pick', 'restage'):
    allp = json.load(open(RUN + '/ov_pens.json'))
    # K = '@': the pen lying on the staging spot (so 'restage K && look && pick @' can run as one chain)
    k = int(np.argmin([np.linalg.norm(np.mean(p_['ends_world'], 0)[:2] - STAGE) for p_ in allp])) if args[0] == '@' else int(args[0])
    pen = allp[k]; assert pen['big_end'] is not None, 'big end unknown'
    if args[0] == '@': assert np.linalg.norm(np.mean(pen['ends_world'], 0)[:2] - STAGE) < 0.04, 'no pen on the staging spot'
    others = [p['ends_world'] for p in allp if p is not pen]; avoid = pen['crossings'][0]['world'] if pen.get('crossings') else None
    if step == 'restage':
        for cam_big in (True, False):
            msgs = []; ok = restage(pen, pen['big_end'], STAGE, cam_big, q0, log=lambda m: (msgs.append(m), print(m)),
                         others=others, avoid=avoid)
            if ok or 'restage: no plan' not in msgs: break
        print('restaged' if ok else 'RESTAGE FAILED')
    else:
        q = pick_big_end(pen, pen['big_end'], q0, ok_after=lambda ql: all(held_clear(ql + t * (CARRY - ql), top) for t in np.linspace(0, 1, 30)), others=others, avoid=avoid)
        if q is None: print('PICK FAILED (no plan -> restage; not holding -> look again)'); sys.exit(2)
        assert all(held_clear(q + t * (CARRY - q), top) for t in np.linspace(0, 1, 30)), 'carry path not clear'
        move(CARRY, 0.3); g = state()['gripper']; assert holding(g, GRIP_CMD), 'lost the pen'
        jpg, _ = snap(RUN + '/carry'); c = cap_in_hand(jpg) or cap_in_hand(jpg, 'dark')
        im = cv2.imread(RUN + '/carry.jpg'); cv2.imwrite(RUN + '/carry_zoom.jpg', cv2.resize(im[300:450, 540:720], None, fx=5, fy=5, interpolation=cv2.INTER_CUBIC))
        if not hangs_straight(jpg): print('PEN SWUNG IN THE FINGERS - put it down (python3 -m sorting.sort RUN putdown X Y DX DY) and restage'); sys.exit(3)
        # the cap colour is only needed by reseat; dark caps (brown 91) are not picked up by it
        json.dump({'hue': None if c is None else c[2]}, open(RUN + '/held.json', 'w'))
        print('holding; read the code in carry_zoom.jpg;', 'cap colour not recognised (dark cap?)' if c is None else 'cap colour key %s' % (c[2],))

elif step == 'place':
    i, j = int(args[0]), int(args[1]); slot = P[j * 6 + i]
    assert holding(s['gripper'], GRIP_CMD), 'not holding a pen'
    info = lower_to_grid(slot, box, top); assert info is not None, 'no insertion plan'
    gripper(0.45); time.sleep(0.5)                                   # pen now stands on the grid, leaning on its neighbours
    q = np.array(state()['q']); Tt = tcp_pose(q)
    back = follow([T(Tt[:3, :3], Tt[:3, 3] - d * Tt[:3, 0] + [0, 0, 0.25 * d]) for d in (0.02, 0.04, 0.06, 0.08)], q, tip_min=top + 0.004)
    assert back is not None, 'no retreat path'; run(back, 0.2, limit=0.08)
    print('released and backed off - run verify; if the pen stands proud run reseat I J')

elif step == 'verify':
    v = np.array(json.load(open(RUN + '/box_views.json'))[1]); to(v, 0.12, what='box view'); gripper(1.0); snap(RUN + '/verify_raw')
    im = cv2.imread(RUN + '/verify_raw.jpg'); uv = project_world(json.load(open(RUN + '/verify_raw.json'))['q'], P)
    x0, y0 = np.clip(uv.min(0) - 60, 0, None).astype(int); x1, y1 = (uv.max(0) + 50).astype(int)
    cv2.imwrite(RUN + '/verify.jpg', cv2.resize(im[y0:y1, x0:x1], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)); print('saved verify.jpg')

elif step == 'putdown':
    x, y, dx, dy = map(float, args); print('released' if put_down_swung((x, y), (dx, dy), q0) else 'PUT-DOWN FAILED')

elif step == 'reseat':
    i, j = int(args[0]), int(args[1]); slot = P[j * 6 + i]; hue = json.load(open(RUN + '/held.json'))['hue']
    assert hue is not None, 'cap colour unknown for this pen: locate its cap by hand (pixel_ray + triangulate) and call regrasp_cap'
    hue = hue if hue == 'dark' else tuple(hue)
    rt.OBSTACLES[0] = (rt.OBSTACLES[0][0], rt.OBSTACLES[0][1] + np.array([0, 0, 0.10]))       # stay above the proud pen while looking
    rays = []; gripper(1.0)
    for k in ([int(x) for x in args[2].split(',')] if len(args) > 2 else (1, 0)):      # optional view pair, e.g. 1,2 when a finger hides the cap
        v = np.array(json.load(open(RUN + '/box_views.json'))[k]); to(v, 0.17, what='box view %d' % k)
        jpg, meta = snap(RUN + '/reseat_v%d' % k); c = find_cap(cv2.imdecode(np.frombuffer(jpg, np.uint8), 1), meta['q'], slot + [0, 0, 0.08], hue)
        assert c is not None, 'proud cap not found in view %d' % k
        print('view', k, 'cap px', c[0].round(0), 'area', c[1]); rays.append(pixel_ray(meta['q'], c[0]))
    cap, rms = triangulate(rays); proud = cap[2] - slot[2]
    print('cap at', cap.round(3), 'ray rms %.1f mm, proud %.0f mm, lean xy mm %s' % (rms * 1000, proud * 1000, ((cap[:2] - slot[:2]) * 1000).round(0)))
    assert rms < 0.004 and 0.03 < proud < 0.11, 'cap position not credible'
    set_box(box)
    r = regrasp_cap(cap, box, top); assert r is not None, 'could not grip the cap'
    # keep the height the pen is at (tip on the grid: ~85 mm proud; tip in the hole, body not square: ~55 mm); lifting it would pull the tip out
    st, _ = cage_jiggle(slot, top, r[0], np.radians(25), float(cap[2]) - 0.012, ref_hue=hue, snap_prefix=RUN + '/reseat')
    print('cage_jiggle:', st)
    gripper(0.45); q = np.array(state()['q']); Tt = tcp_pose(q)
    back = follow([T(Tt[:3, :3], Tt[:3, 3] + [0, 0, dz]) for dz in (0.02, 0.04, 0.06)], q, tip_min=top + 0.004)
    assert back is not None, 'no retreat path'; run(back, 0.2, limit=0.08)
    print('backed off; ' + {'dropped': 'pen dropped while jiggling', 'gone': 'pen was already down when caging'}.get(st, 'PEN NOT SEATED') + ' - verify from above')

elif step == 'addpen':
    from perception import pens
    u1, v1, u2, v2 = map(float, args); q = json.load(open(RUN + '/ov.json'))['q']; plane = table_plane(pens.PEN_R)
    W = pixel_to_plane(q, [[u1, v1], [u2, v2]], *plane); u = (W[1] - W[0]) / np.linalg.norm(W[1] - W[0])
    ends = [W[0], W[0] + PEN_LEN * u]; found = json.load(open(RUN + '/ov_pens.json'))
    found.append(dict(ends_world=[e.tolist() for e in ends], ends_px=project_world(q, ends).tolist(), length_m=PEN_LEN, big_end=0, manual=True))
    json.dump(found, open(RUN + '/ov_pens.json', 'w'))
    im = cv2.imread(RUN + '/ov.jpg')
    for k, p_ in enumerate(found):
        a, b = [tuple(int(v) for v in e) for e in p_['ends_px']]; cv2.line(im, a, b, (0, 0, 255), 2)
        cv2.circle(im, a if p_['big_end'] == 0 else b, 9, (0, 255, 0), 2); cv2.putText(im, str(k), a, 0, 1, (255, 0, 0), 2)
    cv2.imwrite(RUN + '/ov_pens.jpg', im)
    print('added pen %d by hand: big end %s, other end %s' % (len(found) - 1, np.round(ends[0], 3).tolist(), np.round(ends[1], 3).tolist()))
