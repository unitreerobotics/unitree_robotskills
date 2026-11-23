#!/usr/bin/env python3
"""Locate every xiangqi intersection directly in the left head image. Offline; no robot.

The 10 rank lines and 9 file lines are measured in the image itself: each line is sampled in the gaps
between neighbouring intersections (pieces sit on intersections; the ~6 mm gaps between pieces usually show
the printed line), each sample is a sub-pixel dark-line peak on the board-top plane, and every line is fitted
robustly through its samples. Intersections are the crossings of the fitted lines. config/board_geometry.json
is used only as the starting guess and search window; the result is compared against it.

Outputs in --output:
  board_points.json   per intersection: left-image pixel (640x480 eye), waist-frame xyz on the board top,
                      deviation from board_geometry.json; fitted centre/yaw/cell size; line fit statistics
  left_points.jpg     left eye image with all 90 points (green = both lines measured nearby, yellow = inferred)
                      and coordinates; left_points_x2.jpg is the same enlarged 2x
Uses config/head_camera_calibration.json. Image: raw stereo (1280x480, left half used) or a single 640x480 eye.
"""
import argparse
import json
import sys
from pathlib import Path
import cv2
import numpy as np

from r1d_chess.paths import ROOT
from r1d_chess.board.head_board_measure import camera_pose  # noqa: E402

FILES = 'abcdefghi'
RES = 2e-4                  # 0.2 mm sampling on the board plane
HALF_ACROSS = .009          # search +-9 mm across a line
HALF_ALONG = .0018          # average +-1.8 mm along a line (inside the gap between pieces)
EDGE_ACROSS = .009          # outermost lines (kept separate so it can be tuned; the frame is ~10 mm outside)


def load(a):
    if a.observation:
        obs = json.loads(Path(a.observation).read_text())
        left = obs['paths']['cam_left_high']
        stereo = Path(left).with_name(Path(left).stem+'_stereo.jpg')
        images, head_q = [str(stereo if stereo.exists() else left)], obs['state']['head_q']
    else:
        images, head_q = a.image, a.head_q
    if not images or head_q is None:
        raise SystemExit('need --observation, or --image (repeatable) with --head-q')
    eyes = []
    for path in images:
        img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if img is None or img.shape not in ((480, 1280), (480, 640)):
            raise SystemExit(f'{path}: expected a 1280x480 stereo or 640x480 left-eye JPEG')
        eyes.append(img[:, :640].astype(np.float32))
    return np.mean(eyes, 0), [float(x) for x in head_q], images


class Board:
    """Board frame: u along ranks (toward rank 9), v along files (toward file a); origin = board centre."""

    def __init__(self, g, c, head_q, yaw_dir):
        self.c0 = np.array(g['centre_xy_m'], float)
        self.yaw = float(g['yaw_rad'])
        self.z = float(g['surface_z_m'])
        self.du, self.dv = g['cell_m']['rank_x'], g['cell_m']['file_y']
        self.Tcw = np.linalg.inv(camera_pose(c, head_q, yaw_dir))
        self.K, self.D = np.array(c['K_left']), np.array(c['D_left']).reshape(4, 1)

    def xy(self, u, v):
        ca, sa = np.cos(self.yaw), np.sin(self.yaw)
        return self.c0[0]+u*ca-v*sa, self.c0[1]+u*sa+v*ca

    def project(self, u, v, z=None):
        x, y = self.xy(np.asarray(u, float), np.asarray(v, float))
        P = np.stack((x, y, np.full_like(x, self.z if z is None else z), np.ones_like(x)), -1).reshape(-1, 4)
        C = (self.Tcw @ P.T)[:3].T
        uv, _ = cv2.fisheye.projectPoints(C.reshape(-1, 1, 3), np.zeros((3, 1)), np.zeros((3, 1)), self.K, self.D)
        return uv.reshape(np.shape(x)+(2,)).astype(np.float32)

    def ideal(self, f, r):
        return (r-4.5)*self.du, (4-f)*self.dv


def line_peak(img, board, u0, v0, across, half_across=HALF_ACROSS):
    """Sub-pixel dark-line position across a short strip centred at (u0, v0) on the board plane.
    across='u' finds a rank line (constant u), 'v' a file line. Returns (offset_m, prominence) or None."""
    s = np.arange(-half_across, half_across+1e-9, RES)
    t = np.arange(-HALF_ALONG, HALF_ALONG+1e-9, RES)
    S, Tt = np.meshgrid(s, t, indexing='ij')
    U, V = (u0+S, v0+Tt) if across == 'u' else (u0+Tt, v0+S)
    uv = board.project(U, V)
    if (uv[..., 0] < 38).any() or (uv[..., 0] > 600).any() or (uv[..., 1] < 0).any() or (uv[..., 1] > 479).any():
        return None
    patch = cv2.remap(img, uv[..., 0], uv[..., 1], cv2.INTER_LINEAR)
    prof = patch.mean(1)
    prof = cv2.GaussianBlur(prof.reshape(-1, 1), (1, 0), sigmaX=0, sigmaY=2.5).ravel()
    base = np.median(prof)
    dark = base-prof
    i = int(np.argmax(dark))
    if i in (0, len(dark)-1):
        return None
    noise = np.median(np.abs(prof-base))+1e-3
    prominence = dark[i]/noise
    a, b, cc = dark[i-1], dark[i], dark[i+1]
    sub = .5*(a-cc)/(a-2*b+cc) if (a-2*b+cc) != 0 else 0.
    return float(s[i]+sub*RES), float(prominence)


def robust_line(pos, off, min_prom, prom):
    """Fit offset = a + b*pos through samples, iterative 2.5-MAD rejection."""
    keep = prom >= min_prom
    if keep.sum() < 3:
        return None, keep
    for _ in range(4):
        A = np.c_[np.ones(keep.sum()), pos[keep]]
        coef, *_ = np.linalg.lstsq(A, off[keep], rcond=None)
        res = off-(coef[0]+coef[1]*pos)
        mad = np.median(np.abs(res[keep]))+2e-4
        new = keep & (np.abs(res) <= max(2.5*1.4826*mad, 6e-4))
        if new.sum() == keep.sum() or new.sum() < 3:
            break
        keep = new
    A = np.c_[np.ones(keep.sum()), pos[keep]]
    coef, *_ = np.linalg.lstsq(A, off[keep], rcond=None)
    return coef, keep


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--observation', help='capture_cameras response json')
    p.add_argument('--image', action='append', help='head JPEG (stereo raw or left eye); repeat to average frames')
    p.add_argument('--head-q', type=float, nargs=2, metavar=('PITCH', 'YAW'))
    p.add_argument('--yaw-direction', type=float, default=-1.)
    p.add_argument("--min-prominence", type=float, default=3., help="peak depth over noise; collinearity of the samples rejects false peaks")
    p.add_argument('--output', required=True)
    p.add_argument('--geometry', help='starting geometry instead of config/board_geometry.json (read only), e.g. a '
                   'prior shifted by the offset seen in board-view when the board moved more than ~9 mm')
    p.add_argument('--no-register', action='store_true',
                   help='skip the global registration on fixed printed features (river, palaces, border) and start '
                        'the line fit directly from the stored geometry')
    p.add_argument('--register-span-mm', type=float, default=45., help='registration search half-width around the prior')
    p.add_argument('--register-span-yaw', type=float, default=.12, help='registration yaw half-range (rad)')
    p.add_argument('--write-geometry', action='store_true',
                   help='store the measured intersections, centre, yaw and cell size in config/board_geometry.json')
    a = p.parse_args()
    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=True)
    img, head_q, sources = load(a)
    c = json.loads((ROOT/'config/head_camera_calibration.json').read_text())
    g = json.loads(Path(a.geometry or ROOT/'config/board_geometry.json').read_text())
    stored = {k: g[k] for k in ('centre_xy_m', 'yaw_rad', 'cell_m')}
    registration = None
    if not a.no_register:
        # start from a whole-board match of the fixed printed pattern, not from where the stored
        # geometry or the pieces suggest; pieces move and a stale prior locks onto a one-file shift.
        from r1d_chess.board.board_register import register, overlay
        raw = [cv2.imread(str(Path(sp)), cv2.IMREAD_GRAYSCALE).astype(np.float32) for sp in sources]
        reg = register(raw, c, head_q, g, a.yaw_direction, span_xy=a.register_span_mm/1000., span_yaw=a.register_span_yaw)
        overlay(reg, out/'registration_topdown.jpg')
        registration = {k: reg[k] for k in ('centre_xy_m', 'yaw_rad', 'score', 'line_score', 'outside_max',
                                            'runner_up', 'term_medians')}
        g = dict(g, centre_xy_m=reg['centre_xy_m'], yaw_rad=reg['yaw_rad'])
    board = Board(g, c, head_q, a.yaw_direction)

    # Rank lines (constant u): samples at file-gap midpoints. File lines (constant v): rank-gap midpoints;
    # inner files have no line across the river (between ranks 4 and 5), so that gap is not sampled.
    rank_fit, file_fit, used, samples = {}, {}, {}, {}
    for r in range(10):
        pos, off, prom = [], [], []
        for f in range(8):
            u0, v0 = board.ideal(f+.5, r)
            # The outermost lines run next to the thick board frame: narrower window.
            peak = line_peak(img, board, u0, v0, 'u', EDGE_ACROSS if r in (0, 9) else HALF_ACROSS)
            if peak:
                pos.append(v0); off.append(peak[0]); prom.append(peak[1])
        coef, keep = robust_line(np.array(pos), np.array(off), a.min_prominence, np.array(prom)) if pos else (None, None)
        rank_fit[r] = coef
        samples[('rank', r)] = (np.array(pos), np.array(off), keep)
        used[f'rank{r}'] = {'samples': len(pos), 'inliers': int(keep.sum()) if keep is not None else 0,
                            'positions_v_m': [float(x) for x in np.array(pos)[keep]] if keep is not None else []}
    for f in range(9):
        pos, off, prom = [], [], []
        for r in range(9):
            if r == 4 and f not in (0, 8):
                continue
            u0, v0 = board.ideal(f, r+.5)
            peak = line_peak(img, board, u0, v0, 'v', EDGE_ACROSS if f in (0, 8) else HALF_ACROSS)
            if peak:
                pos.append(u0); off.append(peak[0]); prom.append(peak[1])
        coef, keep = robust_line(np.array(pos), np.array(off), a.min_prominence, np.array(prom)) if pos else (None, None)
        file_fit[f] = coef
        samples[('file', f)] = (np.array(pos), np.array(off), keep)
        used[f'file{FILES[f]}'] = {'samples': len(pos), 'inliers': int(keep.sum()) if keep is not None else 0,
                                   'positions_u_m': [float(x) for x in np.array(pos)[keep]] if keep is not None else []}

    # Printed lines of one family are parallel: share one slope (median of well-measured lines) and fit only
    # each line's offset from its inliers, so a line hidden on one side by pieces cannot tilt.
    for family, fit in (('rank', rank_fit), ('file', file_fit)):
        slopes = [fit[k][1] for k in fit if fit[k] is not None and samples[(family, k)][2].sum() >= 5]
        if not slopes:
            continue
        slope = float(np.median(slopes))
        for k in fit:
            if fit[k] is not None:
                pos, off, keep = samples[(family, k)]
                fit[k] = np.array([float(np.median(off[keep]-slope*pos[keep])), slope])

    # Missing lines fall back to a fit over the measured lines of the same family (spacing and slope).
    def fill(fit, n, ideal):
        good = [k for k in range(n) if fit[k] is not None]
        if len(good) < 2:
            raise SystemExit('too few board lines measured; check calibration, head pose and board geometry')
        A = np.c_[np.ones(len(good)), good]
        o = np.linalg.lstsq(A, [ideal(k)+fit[k][0] for k in good], rcond=None)[0]
        s = np.median([fit[k][1] for k in good])
        return {k: (fit[k] if fit[k] is not None else np.array([o[0]+o[1]*k-ideal(k), s])) for k in range(n)}, good
    rank_fit, rank_good = fill(rank_fit, 10, lambda r: board.ideal(0, r)[0])
    file_fit, file_good = fill(file_fit, 9, lambda f: board.ideal(f, 0)[1])

    points, gx, gy, ix, iy = {}, [], [], [], []
    for f in range(9):
        for r in range(10):
            ur, vf = board.ideal(f, r)
            a_r, b_r = rank_fit[r]          # u = ur + a_r + b_r * v
            a_f, b_f = file_fit[f]          # v = vf + a_f + b_f * u
            u = ur+a_r+b_r*(vf+a_f)
            for _ in range(3):
                v = vf+a_f+b_f*u
                u = ur+a_r+b_r*v
            x, y = board.xy(u, v)
            px = board.project(np.array([u]), np.array([v]))[0]
            near = (any(abs(q-vf) < .02 for q in used[f'rank{r}']['positions_v_m']) and
                    any(abs(q-ur) < .02 for q in used[f'file{FILES[f]}']['positions_u_m']))
            gx0, gy0 = board.xy(ur, vf)
            points[f'{FILES[f]}{r}'] = {
                'pixel_left': [round(float(px[0]), 2), round(float(px[1]), 2)],
                'xyz_waist_m': [round(float(x), 4), round(float(y), 4), round(board.z, 4)],
                'deviation_from_geometry_mm': round(float(np.hypot(x-gx0, y-gy0)*1e3), 2),
                'measured': bool(near and r in rank_good and f in file_good)}
            gx.append(x); gy.append(y); ix.append(ur); iy.append(vf)

    # Similarity fit of the measured grid: centre, yaw and the actual cell size.
    P = np.c_[gx, gy]; Q = np.c_[ix, iy]
    pc, qc = P.mean(0), Q.mean(0)
    H = (Q-qc).T @ (P-pc)
    Uu, _, Vt = np.linalg.svd(H)
    Rm = Vt.T @ Uu.T
    if np.linalg.det(Rm) < 0:
        Vt[-1] *= -1; Rm = Vt.T @ Uu.T
    yaw = float(np.arctan2(Rm[1, 0], Rm[0, 0]))
    proj_u = (P-pc) @ Rm[:, 0]; proj_v = (P-pc) @ Rm[:, 1]
    cell_u = float(np.polyfit(Q[:, 0]-qc[0], proj_u, 1)[0]*board.du)
    cell_v = float(np.polyfit(Q[:, 1]-qc[1], proj_v, 1)[0]*board.dv)
    dev = np.array([pt['deviation_from_geometry_mm'] for pt in points.values()])
    summary = {'centre_xy_m': [round(float(pc[0]), 4), round(float(pc[1]), 4)], 'yaw_rad': round(yaw, 4),
               'cell_m': {'rank_x': round(cell_u, 5), 'file_y': round(cell_v, 5)},
               'stored_geometry': stored,
               'registration': registration,
               'deviation_from_geometry_mm': {'mean': round(float(dev.mean()), 2), 'max': round(float(dev.max()), 2)},
               'measured_points': int(sum(p['measured'] for p in points.values())),
               'lines_measured': {'ranks': rank_good, 'files': [FILES[f] for f in file_good]}}

    # Annotated left image
    view = cv2.cvtColor(np.clip(img, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    for r in range(10):
        a_px = board.project(*[np.array([x]) for x in (board.ideal(0, r)[0]+rank_fit[r][0]+rank_fit[r][1]*board.ideal(0, r)[1],
                                                        board.ideal(0, r)[1])])[0]
        b_px = board.project(*[np.array([x]) for x in (board.ideal(8, r)[0]+rank_fit[r][0]+rank_fit[r][1]*board.ideal(8, r)[1],
                                                        board.ideal(8, r)[1])])[0]
        cv2.line(view, tuple(int(x) for x in a_px), tuple(int(x) for x in b_px), (255, 160, 0), 1, cv2.LINE_AA)
    for name, pt in points.items():
        col = (0, 220, 0) if pt['measured'] else (0, 220, 255)
        x, y = (int(round(v)) for v in pt['pixel_left'])
        cv2.circle(view, (x, y), 2, col, -1, cv2.LINE_AA)
    for f in range(9):
        for r, dy in ((0, 14), (9, -6)):
            x, y = (int(round(v)) for v in points[f'{FILES[f]}{r}']['pixel_left'])
            cv2.putText(view, FILES[f], (x-4, y+dy), cv2.FONT_HERSHEY_SIMPLEX, .38, (0, 0, 255), 1, cv2.LINE_AA)
    for r in range(10):
        for f, dx in ((0, -14), (8, 6)):
            x, y = (int(round(v)) for v in points[f'{FILES[f]}{r}']['pixel_left'])
            cv2.putText(view, str(r), (x+dx, y+4), cv2.FONT_HERSHEY_SIMPLEX, .38, (0, 0, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(out/'left_points.jpg'), view, [cv2.IMWRITE_JPEG_QUALITY, 95])
    big = cv2.resize(view, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
    cv2.imwrite(str(out/'left_points_x2.jpg'), big, [cv2.IMWRITE_JPEG_QUALITY, 95])
    (out/'board_points.json').write_text(json.dumps({
        'note': 'image measurement on the board-top plane (model estimate); pixel_left is in the 640x480 left eye',
        'sources': [str(s) for s in sources], 'head_q': head_q, 'yaw_direction': a.yaw_direction,
        'plane_z_m': board.z, 'summary': summary, 'lines': used, 'points': points}, indent=1))
    if a.write_geometry:
        # The line fit only refines the registration: a large disagreement means one of them is wrong.
        problems = []
        if summary['measured_points'] < 20:
            problems.append(f"only {summary['measured_points']} intersections measured (< 20)")
        if registration and summary['deviation_from_geometry_mm']['mean'] > 4.:
            problems.append(f"line fit moved {summary['deviation_from_geometry_mm']['mean']} mm (mean) away from the registration")
        if registration and registration['runner_up'] and registration['score'] - registration['runner_up']['score'] < .5:
            problems.append('registration not unique (runner-up score within 0.5)')
        if problems:
            raise SystemExit('not written to config/board_geometry.json: ' + '; '.join(problems))
        gpath = ROOT/'config/board_geometry.json'
        g['previous_fit'] = dict(stored)
        g.update(centre_xy_m=summary['centre_xy_m'], yaw_rad=summary['yaw_rad'], cell_m=summary['cell_m'],
                 intersections_xy_m={k: v['xyz_waist_m'][:2] for k, v in points.items()},
                 intersections_source='visual_tools/board-points: lines measured in the left image',
                 intersections_evidence=str((out/'board_points.json').resolve()))
        gpath.write_text(json.dumps(g, indent=1))
        print('written', gpath)
    print(json.dumps({'output': str(out), **summary}, indent=1))


if __name__ == '__main__':
    main()
