"""Find loose markers lying on the table in a wrist image: white elongated bodies (+ coloured caps).
usage: python3 -m perception.pens FRAME_PREFIX  -> prints/saves world endpoints; FRAME_pens.jpg overlay."""
import json, sys, cv2, numpy as np
from robot.rt import *
PEN_R = 0.0075                      # body radius: endpoints are intersected with the plane through the pen axis

def _describe(im, hsv, white, q, plane, ends, d, Wd, ignore=None):
    """Cap colours, big-cap end (ring line) and world endpoints of one pen axis given in pixels. None if not pen-sized."""
    H, S, V = cv2.split(hsv); n, p = plane; perp = np.array([-d[1], d[0]]); L = float(np.linalg.norm(ends[1] - ends[0]))
    hh, ww = white.shape; inb = lambda x, y: 0 <= int(round(y)) < hh and 0 <= int(round(x)) < ww
    caps = []
    for e, sgn in zip(ends, (1, -1)):                                                # cap colour: most saturated patch near each end
        m = np.zeros(S.shape, np.uint8); cv2.circle(m, tuple((e + sgn * 9 * d).astype(int)), 8, 255, -1)
        sel = (m > 0) & (white > 0)
        caps.append([int(np.median(H[sel])), int(np.median(S[sel])), int(np.median(V[sel]))] if sel.sum() > 20 else None)
    widths = []
    for e, sgn in zip(ends, (1, -1)):
        ws = []
        for t in np.linspace(0.06, 0.16, 6) * L:                                     # ~9..25 mm in from the end
            c0 = e + sgn * t * d
            ws.append(sum(white[int(round(c0[1] + k * perp[1])), int(round(c0[0] + k * perp[0]))] > 0
                          for k in range(-int(Wd), int(Wd) + 1) if inb(c0[0] + k * perp[0], c0[1] + k * perp[1])))
        widths.append(float(np.median(ws)))
    # the big square cap meets the barrel at a sharp dark ring ~30 mm from that end; the fine-tip end only has a soft step.
    # The ring is dark across the whole barrel, printed text/arrows only across part of it -> take a high percentile across.
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY).astype(float); ts = np.arange(0.08, 0.92, 0.004); r = max(3, int(0.3 * Wd))
    prof = np.array([np.percentile([gray[int(round(pt[1] + k * perp[1])), int(round(pt[0] + k * perp[0]))] for k in range(-r, r + 1)
                                    if inb(pt[0] + k * perp[0], pt[1] + k * perp[1])] or [0], 75)
                     for pt in (ends[0] + t * (ends[1] - ends[0]) for t in ts)])
    dip = cv2.blur(prof.reshape(1, -1), (41, 1)).ravel() - prof                       # local darkness
    if ignore is not None:                                                            # another pen's edge is not this pen's ring
        dip[[bool(ignore[int(round(pt[1])), int(round(pt[0]))]) for pt in (ends[0] + t * (ends[1] - ends[0]) for t in ts)]] = 0
    ring_t = float(ts[int(np.argmax(dip))]); ring_depth = float(dip.max())
    W = pixel_to_plane(q, np.array(ends), n, p)
    if not 0.09 < np.linalg.norm(W[1] - W[0]) < 0.19: return None                     # pen-sized on the table (the white wall is not)
    return dict(ends_px=np.array(ends).round(1).tolist(), ends_world=W.tolist(), length_m=float(np.linalg.norm(W[1] - W[0])),
                width_px=float(Wd), caps_hsv=caps, end_width_px=widths, ring_t=ring_t, ring_depth=ring_depth,
                big_end=int(ring_t > 0.5) if ring_depth > 25 else None)

def _split_lines(pts, Wd, rng):
    """Pens touching/crossing in one blob: RANSAC for thick lines of width Wd. Returns [(end_a, end_b, d)] in pixels."""
    sub = pts[rng.choice(len(pts), min(len(pts), 4000), replace=False)]; used = np.zeros(len(sub), bool); lines = []; first = None
    for _ in range(3):
        best = None
        for _ in range(500):
            i, j = rng.integers(len(sub), size=2); v = sub[j] - sub[i]; nv = np.linalg.norm(v)
            if nv < 4 * Wd: continue
            inl = np.abs((sub - sub[i]) @ (np.array([-v[1], v[0]]) / nv)) < 0.5 * Wd
            sc = np.count_nonzero(inl & ~used)                                       # pixels not yet explained by an earlier line
            if best is None or sc > best[0]: best = (sc, inl)
        if best is None: break
        sc, inl = best
        if first is None: first = sc
        elif sc < 0.4 * first: break
        for _ in range(2):                                                           # refine direction and offset on the inliers
            mu = sub[inl].mean(0); d = np.linalg.svd(sub[inl] - mu, full_matrices=False)[2][0]; perp = np.array([-d[1], d[0]])
            sp = (sub - mu) @ perp; s0 = np.median(sp[inl]); inl = np.abs(sp - s0) < 0.5 * Wd
        tt = (sub - mu) @ d; order = np.sort(tt[inl])                                # extent: longest run without a gap wider than the pen
        seg = max(np.split(order, np.nonzero(np.diff(order) > Wd)[0] + 1), key=len); ctr = mu + s0 * perp
        lines.append((ctr + seg[0] * d, ctr + seg[-1] * d, d)); used |= inl & (tt >= seg[0]) & (tt <= seg[-1])
    return lines

def _cap_pairs(im, hsv, q, plane, taken):
    """Pens on a background that looks like the pen body (white desk): every pen carries a cap of the same colour at each
    end. Pair same-hue saturated blobs that are one pen length apart on the table, with a bright unsaturated body between
    them. `taken`: ends_px of pens already found another way (skipped)."""
    H, S, V = cv2.split(hsv); hh, ww = S.shape
    # S > 90: the box plastic and shaded pen bodies read as pale blue (S 60-85) on the kraft paper; the palest real cap seen is S 101
    m = cv2.morphologyEx(((S > 90) & (V > 70)).astype(np.uint8) * 255, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m); blobs = []
    for k in range(1, n):
        if not 40 < st[k, cv2.CC_STAT_AREA] < 2500: continue
        ys, xs = np.nonzero(lab == k); blobs.append(dict(c=cen[k], hue=int(np.median(H[ys, xs])), pts=np.c_[xs, ys].astype(float)))
    out = []; used = set()
    for i, A in enumerate(blobs):
        for j, B in enumerate(blobs):
            if j <= i or i in used or j in used: continue
            dh = abs(A['hue'] - B['hue']); dh = min(dh, 180 - dh)
            if dh > 10: continue
            d = B['c'] - A['c']; L0 = np.linalg.norm(d)
            if L0 < 40: continue
            d = d / L0; perp = np.array([-d[1], d[0]])
            a_ = A['c'] + ((A['pts'] - A['c']) @ d).min() * d; b_ = B['c'] + ((B['pts'] - B['c']) @ d).max() * d      # outer faces of the caps
            W = pixel_to_plane(q, [a_, b_], *plane)
            if not 0.140 < np.linalg.norm(W[1] - W[0]) < 0.166: continue
            if any(np.linalg.norm(np.array(t).mean(0) - (a_ + b_) / 2) < 30 for t in taken): continue
            sc = np.linalg.norm(W[1] - W[0]) / np.linalg.norm(b_ - a_)                                             # m per pixel on the table
            if any(np.ptp((X_['pts'] - X_['c']) @ perp) * sc > 0.021 or np.ptp((X_['pts'] - X_['c']) @ d) * sc > 0.016 for X_ in (A, B)):
                continue                                                                                         # cap-sized blobs only (caps in the box look 2x bigger)
            body = [hsv[int(round(pt[1])), int(round(pt[0]))] for pt in (a_ + t * (b_ - a_) for t in np.linspace(0.18, 0.82, 60))]
            if np.mean([(px[1] < 45 and px[2] > 110) for px in body]) < 0.6: continue                                # mostly white body between the caps (print, a finger corner)
            Wd = float(max(np.ptp((A['pts'] - A['c']) @ perp), np.ptp((B['pts'] - B['c']) @ perp))) + 2
            pen = _describe(im, hsv, m, q, plane, [a_, b_], d, Wd)
            # a pen has its ring line ~30 mm from one end (t ~ 0.2 or 0.8); anything else is not a pen
            if pen is not None and pen['big_end'] is not None and 0.12 < min(pen['ring_t'], 1 - pen['ring_t']) < 0.34:
                pen['by_caps'] = True; out.append(pen); used |= {i, j}
    return out

def find_pens(im, q):
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV); H, S, V = cv2.split(hsv)
    # Real camera: table is low-saturation mid-grey (S<25, V~170); pen bodies are bright (V~250); caps are saturated.
    white = (((V > 222) | (S > 55)) & (V > 110)).astype(np.uint8) * 255
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    plane = table_plane(PEN_R); out = []; rng = np.random.default_rng(0)
    for c in cv2.findContours(white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
        if cv2.contourArea(c) < 1500: continue
        (cx, cy), (w, h), ang = cv2.minAreaRect(c)
        L, Wd = max(w, h), min(w, h)
        comp = np.zeros(white.shape, np.uint8); cv2.drawContours(comp, [c], -1, 255, -1)
        ys, xs = np.nonzero(comp & white); pts = np.c_[xs, ys].astype(float); single = None
        if L / max(Wd, 1) >= 2.5 and cv2.contourArea(c) / (L * Wd) >= 0.3:              # loose pre-filter; the strict test follows the robust fit
            box = cv2.boxPoints(((cx, cy), (w, h), ang))
            d0 = box[1] - box[0] if np.linalg.norm(box[1] - box[0]) > np.linalg.norm(box[2] - box[1]) else box[2] - box[1]
            # Robust axis: things stuck to the pen (torn paper under it, seen 2026-11-17) inflate the bounding rectangle.
            # PCA of the region's pixels gives the direction; the median across the axis gives its position; the extent is
            # measured only on pixels within the pen's own width.
            mu = pts.mean(0)
            d = np.linalg.svd(pts - mu, full_matrices=False)[2][0]; d = d * (1 if d @ d0 > 0 else -1); perp = np.array([-d[1], d[0]])
            sp = (pts - mu) @ perp; s0 = np.median(sp); tt = (pts - mu) @ d
            Wd = float(np.percentile(np.abs(sp - s0), 90) * 2 / 0.9)                      # width from the bulk of the pixels
            keep = np.abs(sp - s0) < 0.6 * Wd; L = float(tt[keep].max() - tt[keep].min())
            if L / max(Wd, 1) >= 3.5 and keep.sum() / (L * Wd) >= 0.6:                    # a pen: elongated, solid
                ctr = mu + s0 * perp + 0.5 * (tt[keep].max() + tt[keep].min()) * d
                single = _describe(im, hsv, white, q, plane, [ctr - L / 2 * d, ctr + L / 2 * d], d, Wd)   # mask already includes saturated caps
        if single is not None: out.append(single); continue
        if len(pts) > 250000: continue                                                   # wall / box sized regions
        # not one pen: maybe several touching or crossing. Pen width from the distance transform (uniform in 0..W/2 for a bar).
        dt = cv2.distanceTransform(comp & white, cv2.DIST_L2, 3); Wd = float(np.percentile(dt[dt > 0], 90) / 0.45)
        lines = _split_lines(pts, Wd, rng)
        for k, (a_, b_, d) in enumerate(lines):
            ignore = np.zeros(white.shape, np.uint8)                                      # the other pens of this blob, a bit fattened
            for k2, (a2, b2, _) in enumerate(lines):
                if k2 != k: cv2.line(ignore, tuple(int(v) for v in a2), tuple(int(v) for v in b2), 1, int(Wd) + 8)
            inside = lambda pt: bool(ignore[int(np.clip(round(pt[1]), 0, white.shape[0] - 1)), int(np.clip(round(pt[0]), 0, white.shape[1] - 1))])
            L = np.linalg.norm(b_ - a_)
            while L > 4 * Wd and inside(a_): a_ = a_ + 2 * d; L -= 2                      # an end that runs into another pen is not this pen's end
            while L > 4 * Wd and inside(b_): b_ = b_ - 2 * d; L -= 2
            pen = _describe(im, hsv, white, q, plane, [a_, b_], d, Wd, ignore)
            if pen is not None: pen['split'] = True; out.append(pen)
    out += _cap_pairs(im, hsv, q, plane, [p_['ends_px'] for p_ in out])
    # crossings (image geometry only). Which pen is on top is NOT decided: two image cues got it wrong, and it is not
    # needed - either pen may be taken as long as the fingers stay clear of the other one and of the crossing.
    for i, A in enumerate(out):
        for j, B in enumerate(out):
            if j <= i: continue
            a0, a1 = np.array(A['ends_px']); b0, b1 = np.array(B['ends_px']); da, db = a1 - a0, b1 - b0
            den = da[0] * db[1] - da[1] * db[0]
            if abs(den) < 1e-6: continue
            ta = ((b0 - a0)[0] * db[1] - (b0 - a0)[1] * db[0]) / den; tb = ((b0 - a0)[0] * da[1] - (b0 - a0)[1] * da[0]) / den
            if not (-0.05 < ta < 1.05 and -0.05 < tb < 1.05): continue
            X = pixel_to_plane(q, [a0 + ta * da], *plane)[0].tolist()
            A.setdefault('crossings', []).append(dict(other=j, t=float(ta), world=X))
            B.setdefault('crossings', []).append(dict(other=i, t=float(tb), world=X))
    return out

if __name__ == '__main__':
    name = sys.argv[1]; im = cv2.imread(name + '.jpg'); q = json.load(open(name + '.json'))['q']
    pens = find_pens(im, q)
    for i, pn in enumerate(pens):
        a, b = [tuple(int(v) for v in e) for e in pn['ends_px']]
        cv2.line(im, a, b, (0, 0, 255), 2); cv2.circle(im, a, 6, (0, 255, 0), 2); cv2.circle(im, b, 6, (0, 255, 0), 2)
        cv2.putText(im, str(i), a, 0, 1, (255, 0, 0), 2)
        print(i, 'len %.0f mm' % (pn['length_m'] * 1000), 'ends world', np.round(pn['ends_world'], 3).tolist(), 'caps hsv', pn['caps_hsv'], 'ring at %.2f depth %.0f' % (pn['ring_t'], pn['ring_depth']), 'big end', pn['big_end'],
              ''.join('  CROSSES pen %d at t=%.2f' % (c['other'], c['t']) for c in pn.get('crossings', [])))
    cv2.imwrite(name + '_pens.jpg', im); json.dump(pens, open(name + '_pens.json', 'w'))
