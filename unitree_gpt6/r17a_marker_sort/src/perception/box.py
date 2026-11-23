"""Fit the 4x6 slot lattice of the pen box from several wrist views (coloured cap tops, multi-view).
Model: P(i, j) = P0 + i*a + j*b, cap-top plane parallel to the table. i = 0..5 along a row, j = 0..3.
usage: python3 -m perception.box OUT.json X0 Y0 Z0 AX AY BX BY FRAME_PREFIX...   (rough initial guess in world metres)"""
import json, sys, cv2, numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from robot.rt import *

def cap_mask(im):
    """Hue image with -1 where the pixel is not cap-coloured (saturated cap tops; grey/black caps are skipped)."""
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV); H, S, V = cv2.split(hsv)
    # The translucent box plastic and the white pen bodies read as pale blue here (H ~100-106, S 85-120): without the
    # extra saturation demand in that hue band the whole box counts as "cap colour" (root cause of three bad fits, 2026-11-17).
    pale = (H >= 93) & (H <= 112) & (S < 170)
    return np.where((S > 85) & (V > 50) & ~pale, H.astype(np.int16), -1)

def _one_hue(win):
    """Pixels of the dominant hue in a window of the hue image (boolean), so two neighbouring caps never count as one."""
    h = win[win >= 0]
    if h.size == 0: return np.zeros(win.shape, bool)
    hist = np.bincount(h // 6, minlength=30).astype(float); hist = hist + np.roll(hist, 1) + np.roll(hist, -1)
    mode = int(np.argmax(hist)) * 6 + 3; d = np.abs(win - mode); d = np.minimum(d, 180 - d)
    return (win >= 0) & (d <= 9)

def local_caps(hue, uv, pitch_px):
    """For each predicted cap centre: centroid of the dominant-hue cap pixels in a window of ~0.42 pitch, or None if too few
    (empty slot, grey/black cap). Robust to the white code print; a point between two caps is pulled onto one of them
    (with all colours pooled it sat there happily at zero residual - seen 2026-11-17)."""
    r = max(4, int(0.42 * pitch_px)); out = []; h, w = hue.shape
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]; disk = xx ** 2 + yy ** 2 <= r * r
    for u, v in uv:
        u0, v0 = int(round(u)), int(round(v))
        if not (r <= u0 < w - r and r <= v0 < h - r): out.append(None); continue
        win = _one_hue(np.where(disk, hue[v0 - r:v0 + r + 1, u0 - r:u0 + r + 1], -1))
        out.append((u0 + xx[win].mean(), v0 + yy[win].mean()) if win.sum() > 0.3 * disk.sum() else None)
    return out

# Pitch and cap height are properties of the box, not of the photos (good fits: 17.7-18.1 mm, 149-152 mm). Left free,
# oblique views pulled the pitch to 17.4 mm and the far rows ended 5-8 mm off (2026-11-18 run19: a pen was lowered onto its
# neighbours' caps). They enter the fit as soft priors; a hard-fixed pitch pushed the height to 153 mm on old data.
PITCH, CAP_H = 0.01785, 0.1505

def lattice(x, n):
    """world points of all 24 cap-top centres. x = P0(3), a(2), b(2) with a, b in the table plane."""
    e1 = np.cross(n, [0, 1, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(n, e1)       # in-plane basis ~ (+x, -y... ) fixed by n
    # rigid square grid (measured pitch 17.7 x 17.8 mm, 88-90 deg): b is a turned by 90 deg, handedness taken from x[5:7].
    # Free pitches let a bad start stretch the grid to fit anything (15 x 21.6 mm seen).
    sg = 1.0 if x[3] * x[6] - x[4] * x[5] > 0 else -1.0
    a = x[3] * e1 + x[4] * e2; b = sg * (-x[4] * e1 + x[3] * e2)
    # Depth along the (near-vertical) viewing rays is the weakest direction of the fit: with the height left free the lattice
    # slid 4 mm up and shrank to 17.6 mm (2026-11-18 run20). The cap tops are CAP_H above the table - a property of the box.
    _, p_tab = table_plane(); P0 = x[:3] - ((x[:3] - p_tab) @ n - CAP_H) * n
    return np.array([P0 + i * a + j * b for j in range(4) for i in range(6)]), a, b

def fit(frames, x0):
    n, p_tab = table_plane(); x = np.array(x0, float)
    for it in range(60):          # each pass re-centres on the colour inside small windows (mean-shift): far starts need many passes
        x_prev = x.copy()
        P, _, _ = lattice(x, n); obs = []
        for f in frames:
            uv = project_world(f['q'], P)
            c = local_caps(f['mask'], uv, np.median(np.linalg.norm(uv[1:6] - uv[0:5], axis=1)))
            obs.append([(s, np.array(ci)) for s, ci in enumerate(c) if ci is not None])
        def res(x):
            P, _, _ = lattice(x, n)
            return np.concatenate([(project_world(f['q'], P[[s for s, _ in ob]]) - np.array([c for _, c in ob])).ravel()
                                   for f, ob in zip(frames, obs)])
        def res_prior(x):
            P, a, _ = lattice(x, n)
            return np.r_[res(x), 4.0 * (np.linalg.norm(a) - PITCH) / 0.0001, 4.0 * ((P[0] - p_tab) @ n - CAP_H) / 0.001]
        x = least_squares(res_prior, x, loss='soft_l1', f_scale=2.0).x
        if it >= 3 and np.abs(x - x_prev).max() < 3e-5: break
    r = res(x).reshape(-1, 2); P, a, b = lattice(x, n)
    seen = sorted({s for ob in obs for s, _ in ob})
    return x, P, a, b, float(np.sqrt((r ** 2).sum(1).mean())), seen, [len(o) for o in obs]

if __name__ == '__main__':
    out = sys.argv[1]; x0 = [float(v) for v in sys.argv[2:9]]; frames = []
    for name in sys.argv[9:]:
        im = cv2.imread(name + '.jpg'); frames.append(dict(q=json.load(open(name + '.json'))['q'], mask=cap_mask(im), name=name))
    n, p = table_plane()
    e1 = np.cross(n, [0, 1, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(n, e1)
    x0v = np.r_[x0[:3], np.array([x0[3], x0[4], 0]) @ e1, np.array([x0[3], x0[4], 0]) @ e2, np.array([x0[5], x0[6], 0]) @ e1, np.array([x0[5], x0[6], 0]) @ e2]
    # The residual cannot tell a slipped fit from a good one (seen 2026-11-17: 1.7 px rms with every circle between the
    # caps after the box was nudged; a whole-pitch slip after it was bumped and turned ~10 deg). So: coarse search over
    # rotation + shift of the initial guess, scored by how much cap colour sits right at the slot centres, then a full
    # fit from the best few, again picked by that score.
    def coverage(P):
        tot = 0.0
        for f in frames:
            uv = project_world(f['q'], P); r = max(2, int(0.3 * np.median(np.linalg.norm(uv[1:6] - uv[0:5], axis=1))))
            for u, v in uv.astype(int):
                win = f['mask'][max(v - r, 0):v + r + 1, max(u - r, 0):u + r + 1]
                tot += float(_one_hue(win).mean()) if win.size else 0.0
        return tot / (24 * len(frames))
    def moved(x, th, dx, dy):
        c, s_ = np.cos(th), np.sin(th); R2 = np.array([[c, -s_], [s_, c]]); a2, b2 = x[3:5], x[5:7]; a3, b3 = R2 @ a2, R2 @ b2
        d = 2.5 * (a2 - a3) + 1.5 * (b2 - b3)                                  # keep the lattice centre, then shift it
        return np.r_[x[:3] + d[0] * e1 + d[1] * e2 + [dx, dy, 0], a3, b3]
    coarse = sorted(((coverage(lattice(moved(x0v, th, dx, dy), n)[0]), th, dx, dy)
                     for th in np.radians(np.arange(-45, 46, 3)) for dx in np.arange(-0.04, 0.041, 0.005) for dy in np.arange(-0.04, 0.041, 0.005)),
                    reverse=True)
    best = None; tried = []
    for sc0, th, dx, dy in coarse:
        if any(abs(th - t) < 0.08 and abs(dx - u) < 0.008 and abs(dy - v) < 0.008 for t, u, v in tried): continue
        tried.append((th, dx, dy))
        try: r_ = fit(frames, moved(x0v, th, dx, dy))
        except Exception: continue
        sc = coverage(r_[1])
        if best is None or sc > best[0]: best = (sc, r_, th, dx, dy)
        if len(tried) == 10: break
    cov, (x, P, a, b, rms, occ, cnt), th, dx, dy = best
    print('coarse search: start turned %.0f deg, shifted %.0f, %.0f mm; cap-colour coverage at the slot centres %.2f' % (np.degrees(th), dx * 1000, dy * 1000, cov))
    h = (P[0] - p) @ n
    print('matches per view', cnt, ' reprojection rms %.1f px' % rms)
    print('pitch a %.1f mm, b %.1f mm, angle %.1f deg; cap tops %.1f mm above the table' % (
        1000 * np.linalg.norm(a), 1000 * np.linalg.norm(b), np.degrees(np.arccos(a @ b / np.linalg.norm(a) / np.linalg.norm(b))), 1000 * h))
    print('P0', P[0].round(4), 'a', a.round(4), 'b', b.round(4))
    if not 0.147 < h < 0.154: print('WARNING: cap height %.0f mm is not the 150 mm of this box - look at the overlay' % (h * 1000))
    json.dump(dict(P=P.tolist(), a=a.tolist(), b=b.tolist(), cap_height_m=float(h), rms_px=rms, matched_slots=occ), open(out, 'w'))
    for f in frames:                                                                        # overlay for review
        im = cv2.imread(f['name'] + '.jpg'); uv = project_world(f['q'], P)
        for s, (u, v) in enumerate(uv):
            cv2.circle(im, (int(u), int(v)), 9, (0, 255, 0) if s in occ else (0, 0, 255), 2)
            cv2.putText(im, '%d,%d' % (s % 6, s // 6), (int(u) - 12, int(v) - 12), 0, 0.4, (255, 255, 255), 1)
        x0_, y0_ = np.clip(uv.min(0) - 60, 0, None).astype(int); x1_, y1_ = (uv.max(0) + 60).astype(int)
        cv2.imwrite(f['name'] + '_box.jpg', cv2.resize(im[y0_:y1_, x0_:x1_], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
