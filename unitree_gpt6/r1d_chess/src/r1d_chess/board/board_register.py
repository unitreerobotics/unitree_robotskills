#!/usr/bin/env python3
"""Global registration of the printed xiangqi board in the head image, from fixed printed features only.

Pieces move, so nothing here depends on where pieces are. The printed pattern is matched as a whole on a
top-down warp of the board plane (both eyes averaged):
  * the 10 rank lines and the 9 file lines inside the grid,
  * the two edge files (a, i) run through the river while the 7 inner files are broken there,
  * the two palace X diagonals,
  * one pitch outside the grid there is no printed line.
The river and palace terms are not periodic, so a one-file or one-rank shift scores clearly worse; each
line contributes the median of its samples, so pieces covering parts of lines do not bias the result.

Search: coarse grid over centre ±45 mm / yaw ±0.12 rad around the prior, then Nelder-Mead. Returns
centre, yaw and the per-term scores; board_points.py refines every line from this start.
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import minimize

from r1d_chess.paths import ROOT
from r1d_chess.board.head_board_measure import camera_pose  # noqa: E402

PX = .0005                     # 0.5 mm top-down resolution
# Non-periodic terms carry the uniqueness (a one-rank/one-file shift only changes these), so they are weighted
# up against the ~160 periodic line segments.
RIVER_W, PALACE_W, OUTSIDE_W = 8., 6., 2.
X0, X1, Y0, Y1 = .05, .55, -.30, .26


def topdown_eyes(images, c, head_q, yaw_dir, z):
    """Left and right eye, each warped to the plane z and averaged over frames (NaN outside the view)."""
    TL = np.linalg.inv(camera_pose(c, head_q, yaw_dir))
    TR = np.array(c['T_right_from_left']) @ TL
    xs, ys = np.arange(X0, X1, PX), np.arange(Y1, Y0, -PX)
    gx, gy = np.meshgrid(xs, ys)
    P = np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, z), np.ones(gx.size)]
    out = []
    for eye, T, x0 in (('left', TL, 0), ('right', TR, 640)):
        K, D = np.array(c[f'K_{eye}']), np.array(c[f'D_{eye}']).reshape(4, 1)
        C = (T @ P.T)[:3].T
        uv, _ = cv2.fisheye.projectPoints(C.reshape(-1, 1, 3), np.zeros((3, 1)), np.zeros((3, 1)), K, D)
        uv = uv.reshape(gx.shape+(2,)).astype(np.float32)
        ok = ((uv[..., 0] >= 38) & (uv[..., 0] <= 600) & (uv[..., 1] >= 0) & (uv[..., 1] <= 479)
              & (C[:, 2].reshape(gx.shape) > .05))
        frames = [cv2.remap(img[:, x0:x0+640], uv[..., 0], uv[..., 1], cv2.INTER_LINEAR)
                  for img in images if img.shape[1] >= x0+640]
        if not frames:
            continue
        out.append(np.where(ok, np.mean(frames, 0), np.nan).astype(np.float32))
    return out


def topdown(images, c, head_q, yaw_dir, z):
    eyes = topdown_eyes(images, c, head_q, yaw_dir, z)
    return np.nanmean(np.stack(eyes), 0) if len(eyes) > 1 else eyes[0]


def directional_response(top, vertical):
    """Dark thin line response for long lines of one orientation: smooth ~4 mm along the line, DoG across.
    Short structures (piece rims, glyph strokes) are suppressed by the along-line smoothing."""
    valid = np.isfinite(top)
    t = np.where(valid, top, np.nanmedian(top)).astype(np.float32)
    mm = 1e-3/PX
    along = cv2.getGaussianKernel(int(24*mm) | 1, 4*mm)
    def across(sig):
        return cv2.getGaussianKernel(int(8*sig*mm) | 1, sig*mm)
    if vertical:   # lines along rows direction (constant column): smooth along y (rows), DoG along x (cols)
        wide = cv2.sepFilter2D(t, -1, across(3.), along)
        thin = cv2.sepFilter2D(t, -1, across(.7), along)
    else:
        wide = cv2.sepFilter2D(t, -1, along, across(3.))
        thin = cv2.sepFilter2D(t, -1, along, across(.7))
    r = np.clip(wide-thin, 0, 40)
    r[~valid] = np.nan
    return r


def line_response(top):
    """Thin dark printed lines: small blur minus slightly larger blur, clipped; NaN outside the view."""
    valid = np.isfinite(top)
    t = np.where(valid, top, np.nanmedian(top)).astype(np.float32)
    s = 1.0e-3/PX                      # ~1 mm line width
    r = np.clip(cv2.GaussianBlur(t, (0, 0), 3*s)-cv2.GaussianBlur(t, (0, 0), .7*s), 0, 40)
    r = cv2.GaussianBlur(r, (0, 0), .8*s)
    r[~valid] = np.nan
    return r


class Pattern:
    """Printed board pattern in board coordinates u (toward rank 9) and v (toward file a), metres."""

    def __init__(self, du, dv):
        self.du, self.dv = du, dv
        hu, hv = 4.5*du, 4*dv
        step = .0015
        self.terms = []                         # (name, weight, sign, u-array, v-array)

        def seg(name, u0, v0, u1, v1, weight=1., sign=1., margin=.006, k=12):
            # k samples, kept `margin` away from crossings so a crossing line does not dominate
            L = np.hypot(u1-u0, v1-v0)
            m = min(margin, L/4)
            t = np.linspace(m/L, 1-m/L, k)
            self.terms.append((name, weight, sign, u0+(u1-u0)*t, v0+(v1-v0)*t))
        # rank lines between neighbouring files (each gap its own segment: medians per line below)
        for r in range(10):
            for f in range(8):
                seg(f'rank{r}', (r-4.5)*du, (4-f)*dv, (r-4.5)*du, (3-f)*dv)
        # file lines between neighbouring ranks; inner files have no line across the river
        for f in range(9):
            for r in range(9):
                inner = f not in (0, 8)
                if r == 4 and inner:
                    seg(f'river{f}', (r-4.5)*du, (4-f)*dv, (r-3.5)*du, (4-f)*dv, weight=RIVER_W, sign=-1.)
                else:
                    seg(f'file{f}', (r-4.5)*du, (4-f)*dv, (r-3.5)*du, (4-f)*dv, weight=1.5 if r == 4 else 1.)
        # palace diagonals (files d..f, ranks 0..2 and 7..9)
        for r0 in (0, 7):
            for (f0, f1) in ((3, 5), (5, 3)):
                for k in range(2):
                    ua, ub = (r0+k-4.5)*du, (r0+k+1-4.5)*du
                    va = (4-(f0+(f1-f0)*k/2))*dv
                    vb = (4-(f0+(f1-f0)*(k+1)/2))*dv
                    seg(f'palace{r0}', ua, va, ub, vb, weight=PALACE_W)
        # one pitch outside the grid there is no printed grid line
        for i, u in enumerate((-hu-du, hu+du)):
            for f in range(8):
                seg(f'outside{i}', u, (4-f)*dv, u, (3-f)*dv, weight=OUTSIDE_W, sign=-1.)
        for i, v in enumerate((-hv-dv, hv+dv)):
            for r in range(9):
                seg(f'outside{2+i}', (r-4.5)*du, v, (r-3.5)*du, v, weight=OUTSIDE_W, sign=-1.)
        self.outside_line = np.array([int(t[0][7:]) if t[0].startswith('outside') else -1 for t in self.terms])
        # orientation of each segment: constant u (rank-like) is a vertical line in the top-down image
        self.kind = np.array(['iso' if t[0].startswith('palace') else
                              ('vert' if abs(t[3][0]-t[3][-1]) < 1e-9 else 'horiz') for t in self.terms])
        self.family = np.array([t[0].rstrip('0123456789') for t in self.terms])
        self.U = np.array([t[3] for t in self.terms])          # (nseg, k)
        self.V = np.array([t[4] for t in self.terms])
        self.W = np.array([t[1]*t[2] for t in self.terms])
        self.Wabs = np.array([t[1] for t in self.terms])


def score_terms(resp, pat, cx, cy, yaw):
    ca, sa = np.cos(yaw), np.sin(yaw)
    out = []
    for name, w, sign, u, v in pat.terms:
        x, y = cx+u*ca-v*sa, cy+u*sa+v*ca
        col, row = (x-X0)/PX, (Y1-y)/PX
        vals = cv2.remap(resp, col.astype(np.float32).reshape(-1, 1), row.astype(np.float32).reshape(-1, 1),
                         cv2.INTER_LINEAR, borderValue=np.nan).ravel()
        vals = vals[np.isfinite(vals)]
        if len(vals) < 3:
            continue
        # a piece over part of a segment lowers its median a little but cannot create a line elsewhere
        out.append((name, w, sign, float(np.median(vals))))
    return out


def segment_medians(resp, pat, cx, cy, yaw):
    ca, sa = np.cos(yaw), np.sin(yaw)
    x, y = cx+pat.U*ca-pat.V*sa, cy+pat.U*sa+pat.V*ca
    col, row = ((x-X0)/PX).astype(np.float32), ((Y1-y)/PX).astype(np.float32)
    vals = np.full(pat.U.shape, np.nan, np.float32)
    for kind, r in resp.items():
        sel = pat.kind == kind
        if sel.any():
            vals[sel] = cv2.remap(r, col[sel], row[sel], cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT, borderValue=np.nan)
    with np.errstate(all='ignore'):
        return np.nanmedian(np.where(np.isfinite(vals), vals, np.nan), axis=1)


def family_means(pat, m):
    out = {}
    for fam in ('rank', 'file', 'river', 'palace', 'outside'):
        sel = pat.family == fam
        v = m[sel]
        v = v[np.isfinite(v)]
        out[fam] = float(np.median(v)) if len(v) >= max(2, sel.sum()//3) else None
    return out


def outside_max(pat, m):
    """Largest line response one pitch outside the grid (4 lines, each the median of its segments). On the
    real board nothing is printed there; with a one-file or one-rank alias one of them lies on a real line."""
    best = 0.
    for i in range(4):
        v = m[pat.outside_line == i]
        v = v[np.isfinite(v)]
        if len(v) >= 3:
            best = max(best, float(np.median(v)))
    return best


def score(resp, pat, x):
    """Both line families must match (geometric mean, so one family at zero gives zero); the palace X is a
    bonus and a printed line inside the river (inner files are broken there) a penalty. A one-line alias of
    the periodic grid scores lower than the true pose (palace and river terms) but can still look valid on
    its own, so register() explicitly probes the four one-line shifts of its best pose."""
    m = segment_medians(resp, pat, *x)
    f = family_means(pat, m)
    alias = 0.      # one pitch outside the grid lies on the board's black frame: not usable as a penalty
    if f['rank'] is None or f['file'] is None:
        return 0.
    s = np.sqrt(max(f['rank'], 0)*max(f['file'], 0))
    if f['palace'] is not None:
        # bonus only on top of matching lines: the X alone (both line families at 0) must not score
        # (a lines-free pose reached 2.07 from the palace term alone)
        s += min(.5*f['palace'], s)
    if f['river'] is not None:
        s -= 2.*f['river']
    return -(s - alias)


def _blur_nan(a, sigma):
    m = np.isfinite(a)
    num = cv2.GaussianBlur(np.where(m, a, 0).astype(np.float32), (0, 0), sigma)
    den = cv2.GaussianBlur(m.astype(np.float32), (0, 0), sigma)
    out = num/np.maximum(den, 1e-3)
    out[den < .3] = np.nan
    return out


def register(images, c, head_q, prior, yaw_dir=-1., span_xy=.045, span_yaw=.12):
    """Return dict(centre_xy_m, yaw_rad, score, runner_up, ...) from printed features only."""
    z = float(prior['surface_z_m'])
    eyes = topdown_eyes(images, c, head_q, yaw_dir, z)
    top = np.nanmean(np.stack(eyes), 0) if len(eyes) > 1 else eyes[0]
    # A printed line lies on the board plane, so after warping to that plane it sits at the same place in
    # both eyes; pieces stand ~10 mm above it and are displaced between the eyes. The minimum of the two
    # eyes' responses keeps the printed pattern and suppresses pieces, wherever the pieces are.
    def both(fn, *args):
        rs = [fn(e, *args) for e in eyes]
        return np.fmin.reduce(rs) if len(rs) > 1 else rs[0]
    resp = {'iso': both(line_response), 'vert': both(directional_response, True),
            'horiz': both(directional_response, False)}
    du, dv = prior['cell_m']['rank_x'], prior['cell_m']['file_y']
    pat = Pattern(du, dv)
    cx0, cy0 = prior['centre_xy_m']
    yaw0 = float(prior['yaw_rad'])
    # Coarse-to-fine: line responses are ~2 mm wide, too narrow for a 4 mm / 0.012 rad grid, so the grid and
    # a first refinement run on responses blurred to ~2.5 mm, the final refinement on the sharp ones.
    blur = {k: _blur_nan(v, 2.5e-3/PX) for k, v in resp.items()}
    cand = []
    for cx in np.arange(cx0-span_xy, cx0+span_xy+1e-9, .004):
        for cy in np.arange(cy0-span_xy, cy0+span_xy+1e-9, .004):
            for yaw in np.arange(yaw0-span_yaw, yaw0+span_yaw+1e-9, .008):
                cand.append((score(blur, pat, (cx, cy, yaw)), cx, cy, yaw))
    cand.sort()
    starts = []
    for s_, cx, cy, yaw in cand:
        if all(np.hypot(cx-a, cy-b) > .012 or abs(yaw-t) > .02 for _, a, b, t in starts):
            starts.append((s_, cx, cy, yaw))
        if len(starts) >= 8:
            break
    refined = []
    for s_, cx, cy, yaw in starts:
        x = np.array([cx, cy, yaw])
        for r_ in (blur, resp):
            simplex = np.vstack([x, x+[.002, 0, 0], x+[0, .002, 0], x+[0, 0, .006]])
            x = minimize(lambda v: score(r_, pat, v), x, method='Nelder-Mead',
                         options={'xatol': 1e-4, 'fatol': 1e-5, 'initial_simplex': simplex, 'maxiter': 400}).x
        refined.append((float(score(resp, pat, x)), *map(float, x)))
    refined.sort()
    best = refined[0]
    # The grid is periodic: probe the four one-line shifts of the best pose even when they are outside the
    # search span (a prior that is itself one file off would otherwise be confirmed).
    ca, sa = np.cos(best[3]), np.sin(best[3])
    alias_checked = []
    for su, sv in ((du, 0.), (-du, 0.), (0., dv), (0., -dv)):
        x = np.array([best[1]+su*ca-sv*sa, best[2]+su*sa+sv*ca, best[3]])
        for r_ in (blur, resp):
            simplex = np.vstack([x, x+[.002, 0, 0], x+[0, .002, 0], x+[0, 0, .006]])
            x = minimize(lambda v: score(r_, pat, v), x, method='Nelder-Mead',
                         options={'xatol': 1e-4, 'fatol': 1e-5, 'initial_simplex': simplex, 'maxiter': 200}).x
        alias_checked.append((float(score(resp, pat, x)), *map(float, x)))
    refined = sorted(refined+alias_checked)
    best = refined[0]
    second = next((r for r in refined[1:] if np.hypot(r[1]-best[1], r[2]-best[2]) > .01 or abs(r[3]-best[3]) > .015),
                  None)
    med = segment_medians(resp, pat, *best[1:])
    by = {}
    for (name, w, s, _, _), m in zip(pat.terms, med):
        if not np.isfinite(m):
            continue
        key = name.rstrip('0123456789') if not name.startswith('palace') else name
        by.setdefault(key, []).append(m)
    fam = family_means(pat, med)
    return {'centre_xy_m': [round(best[1], 5), round(best[2], 5)], 'yaw_rad': round(best[3], 5),
            'score': round(-best[0], 3),
            'line_score': round(float(np.sqrt(max(fam['rank'] or 0, 0)*max(fam['file'] or 0, 0))), 3),
            'outside_max': round(outside_max(pat, med), 3),
            'runner_up': None if second is None else {'centre_xy_m': [round(second[1], 5), round(second[2], 5)],
                                                      'yaw_rad': round(second[3], 5), 'score': round(-second[0], 3)},
            'term_medians': {k: round(float(np.median(v)), 2) for k, v in by.items()},
            'topdown': top, 'response': resp['iso'], 'pattern': pat}


def overlay(reg, path):
    """Top-down image with the registered printed pattern drawn on it."""
    top = np.nan_to_num(reg['topdown'], nan=0).clip(0, 255).astype(np.uint8)
    v = cv2.cvtColor(top, cv2.COLOR_GRAY2BGR)
    cx, cy = reg['centre_xy_m']
    ca, sa = np.cos(reg['yaw_rad']), np.sin(reg['yaw_rad'])
    for name, w, sign, u, vv in reg['pattern'].terms:
        x, y = cx+u*ca-vv*sa, cy+u*sa+vv*ca
        col = ((0, 0, 255) if sign < 0 else (0, 200, 0) if not name.startswith('palace') else (255, 0, 255))
        for a, b in zip(((x-X0)/PX).astype(int), ((Y1-y)/PX).astype(int)):
            if 0 <= b < v.shape[0] and 0 <= a < v.shape[1]:
                v[b, a] = col
    cv2.imwrite(str(path), v)
