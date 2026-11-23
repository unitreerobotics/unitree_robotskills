#!/usr/bin/env python3
"""Read the board more easily: undistorted, top-down views of the xiangqi board from the R1-D head camera.

PLANNING REFERENCE ONLY. The outputs help read the position (FEN). They are resampled images and must not
be used to estimate source/target poses; those come from the normal planning image and round build.

Outputs in --output:
  board_topdown.jpg   board seen from straight above at the piece-top plane, red (rank 0) at the bottom,
                      files a-i left to right, coordinates on all sides, piece legend underneath
  board_tiles.jpg     every intersection enlarged and labelled (10 rows x 9 columns, rank 9 on top)
  undistorted_<eye>.jpg   ordinary perspective (pinhole) image of the board area without fisheye bending
  board_view.json     inputs, scale and the pixel of every intersection in board_topdown.jpg

Several frames and both eyes can be averaged to reduce noise. Uses config/head_camera_calibration.json and
config/board_geometry.json; if the board moved, re-measure it first (scripts/head_board_measure.py).
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
LEGEND = ROOT/'assets/piece_legend.jpg'


def load_inputs(a):
    if a.observation:
        obs = json.loads(Path(a.observation).read_text())
        left = obs['paths']['cam_left_high']
        stereo = str(Path(left).with_name(Path(left).stem+'_stereo.jpg'))
        images = [stereo]
        head_q = obs['state']['head_q']
    else:
        images, head_q = a.image, a.head_q
    if not images or head_q is None:
        raise SystemExit('need --observation, or --image (repeatable) with --head-q')
    frames = []
    for path in images:
        img = cv2.imread(str(path))
        if img is None or img.shape[:2] != (480, 1280):
            raise SystemExit(f'{path}: expected the raw 1280x480 head stereo JPEG (use the *_stereo.jpg file)')
        frames.append(img)
    return frames, [float(x) for x in head_q], images


def topdown_maps(c, Tcw, K, D, z, cx, cy, yaw, ppm, margin):
    """Remap grids for a top-down image: rank 9 at the top, file a on the left (robot +y)."""
    half_u, half_w = 4.5*.0337+margin, 4*.0356+margin
    h, w = int(2*half_u*ppm), int(2*half_w*ppm)
    rows, cols = np.mgrid[0:h, 0:w].astype(float)
    u = half_u-rows/ppm                     # along ranks, + toward rank 9 (away from the robot)
    v = half_w-cols/ppm                     # along files, + toward file a (robot left)
    ca, sa = np.cos(yaw), np.sin(yaw)
    x, y = cx+u*ca-v*sa, cy+u*sa+v*ca
    P = np.stack((x, y, np.full_like(x, z), np.ones_like(x)), -1).reshape(-1, 4)
    C = (Tcw @ P.T)[:3].T
    ok = C[:, 2] > .05
    uv, _ = cv2.fisheye.projectPoints(C.reshape(-1, 1, 3), np.zeros((3, 1)), np.zeros((3, 1)), K, D)
    uv = uv.reshape(h, w, 2).astype(np.float32)
    uv[~ok.reshape(h, w)] = -1
    return uv, (h, w), half_u, half_w


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--observation', help='capture_cameras response json (uses its *_stereo.jpg and state.head_q)')
    p.add_argument('--image', action='append', help='raw head stereo JPEG; repeat to average several frames')
    p.add_argument('--head-q', type=float, nargs=2, metavar=('PITCH', 'YAW'))
    p.add_argument('--yaw-direction', type=float, default=-1., help='sign of the last head yaw move (locked convention: -1)')
    p.add_argument('--eye', choices=('left', 'right', 'both'), default='both')
    p.add_argument('--ppm', type=float, default=4., help='top-down pixels per millimetre')
    p.add_argument('--sharpen', type=float, default=.6, help='unsharp mask amount, 0 disables')
    p.add_argument('--output', required=True)
    a = p.parse_args()
    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=True)
    frames, head_q, sources = load_inputs(a)
    c = json.loads((ROOT/'config/head_camera_calibration.json').read_text())
    g = json.loads((ROOT/'config/board_geometry.json').read_text())
    eyes = ['left', 'right'] if a.eye == 'both' else [a.eye]
    TL = np.linalg.inv(camera_pose(c, head_q, a.yaw_direction))
    poses = {'left': TL, 'right': np.array(c['T_right_from_left']) @ TL}
    cal = {e: (np.array(c[f'K_{e}']), np.array(c[f'D_{e}']).reshape(4, 1)) for e in ('left', 'right')}
    z = g['surface_z_m']+g['piece']['height_m']          # characters are on the piece tops
    cx, cy = g['centre_xy_m']
    ppm = a.ppm*1000

    # 1. top-down: average every frame and eye in float, masking invalid samples
    acc = None
    for e in eyes:
        uv, (h, w), half_u, half_w = topdown_maps(c, poses[e], *cal[e], z, cx, cy, g['yaw_rad'], ppm, .03)
        x0 = 0 if e == 'left' else 640
        valid = ((uv[..., 0] >= 38) & (uv[..., 0] <= 600) & (uv[..., 1] >= 0) & (uv[..., 1] <= 479)).astype(np.float32)
        for f in frames:
            img = cv2.remap(f[:, x0:x0+640], uv[..., 0], uv[..., 1], cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT)
            if acc is None:
                acc, wsum = np.zeros(img.shape, np.float64), np.zeros((h, w), np.float64)
            acc += img*valid[..., None]
            wsum += valid
    top = np.clip(acc/np.maximum(wsum, 1e-6)[..., None], 0, 255).astype(np.uint8)
    if a.sharpen > 0:   # mild unsharp mask; strong values add ringing
        blur = cv2.GaussianBlur(top, (0, 0), 1.5*a.ppm/4)
        top = cv2.addWeighted(top, 1+a.sharpen, blur, -a.sharpen, 0)
    pix = {}
    for i, f in enumerate(FILES):
        for r in range(10):
            col = int(round((half_w-(4-i)*.0356)*ppm))
            row = int(round((half_u-(r-4.5)*.0337)*ppm))
            pix[f'{f}{r}'] = [col, row]
    view = top.copy()
    for (col, row) in pix.values():
        cv2.drawMarker(view, (col, row), (0, 200, 0), cv2.MARKER_CROSS, 10, 1)
    pad = 60
    canvas = cv2.copyMakeBorder(view, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=(255, 255, 255))
    for i, f in enumerate(FILES):
        col = pix[f'{f}0'][0]+pad
        for row in (pad//2+12, canvas.shape[0]-pad//2+12):
            cv2.putText(canvas, f, (col-8, row), cv2.FONT_HERSHEY_SIMPLEX, 1., (0, 0, 0), 2)
    for r in range(10):
        row = pix[f'a{r}'][1]+pad
        for col in (pad//2-12, canvas.shape[1]-pad//2-10):
            cv2.putText(canvas, str(r), (col, row+10), cv2.FONT_HERSHEY_SIMPLEX, 1., (0, 0, 0), 2)
    title = np.full((70, canvas.shape[1], 3), 255, np.uint8)
    cv2.putText(title, 'TOP-DOWN BOARD (planning reference only, not for source/target poses)', (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, .75, (0, 0, 180), 2)
    cv2.putText(title, 'black side (rank 9) top, red side (rank 0) bottom; file a = robot left', (10, 58),
                cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 0), 1)
    sheet = [title, canvas]
    if LEGEND.exists():
        leg = cv2.imread(str(LEGEND))
        scale = min(1., canvas.shape[1]/leg.shape[1])
        leg = cv2.resize(leg, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        sheet.append(cv2.copyMakeBorder(leg, 0, 0, 0, canvas.shape[1]-leg.shape[1], cv2.BORDER_CONSTANT, value=(255, 255, 255)))
    cv2.imwrite(str(out/'board_topdown.jpg'), np.vstack(sheet), [cv2.IMWRITE_JPEG_QUALITY, 95])

    # 2. tiles: one enlarged crop per intersection, rank 9 on top like the board
    half = int(.0175*ppm)
    tile_px = 150
    rows = []
    for r in range(9, -1, -1):
        row = []
        for f in FILES:
            col, rw = pix[f'{f}{r}']
            t = top[max(0, rw-half):rw+half, max(0, col-half):col+half]
            t = cv2.resize(t, (tile_px, tile_px), interpolation=cv2.INTER_CUBIC)
            radius = int(g['piece']['diameter_m']/2/(2*.0175)*tile_px)
            cv2.circle(t, (tile_px//2, tile_px//2), radius, (0, 200, 0), 1, cv2.LINE_AA)
            lab = np.full((26, tile_px, 3), 255, np.uint8)
            cv2.putText(lab, f'{f}{r}', (6, 20), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 0), 2)
            row.append(cv2.copyMakeBorder(np.vstack((lab, t)), 3, 3, 3, 3, cv2.BORDER_CONSTANT, value=(255, 255, 255)))
        rows.append(np.hstack(row))
    tiles = np.vstack(rows)
    head = np.full((40, tiles.shape[1], 3), 255, np.uint8)
    cv2.putText(head, 'every intersection enlarged (rank 9 top, rank 0 bottom) - planning reference only', (8, 28),
                cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 180), 2)
    note = np.full((34, tiles.shape[1], 3), 255, np.uint8)
    cv2.putText(note, 'a square holds a piece only if the character is centred in its green circle; edges of neighbours may show',
                (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 1)
    head = np.vstack((head, note))
    cv2.imwrite(str(out/'board_tiles.jpg'), np.vstack((head, tiles)), [cv2.IMWRITE_JPEG_QUALITY, 92])

    # 3. ordinary perspective (pinhole) view without fisheye bending, looking at the board centre
    for e in eyes:
        K, D = cal[e]
        centre = poses[e] @ np.r_[cx, cy, z, 1.]
        d = centre[:3]/np.linalg.norm(centre[:3])
        zc = d
        xc = np.cross([0., 1., 0.], zc)
        xc /= np.linalg.norm(xc)
        yc = np.cross(zc, xc)
        Rv = np.stack((xc, yc, zc))                 # virtual camera rows in the real camera frame
        f = 900.
        Knew = np.array([[f, 0, 640.], [0, f, 480.], [0, 0, 1.]])
        mx, my = cv2.fisheye.initUndistortRectifyMap(K, D, Rv, Knew, (1280, 960), cv2.CV_32FC1)
        x0 = 0 if e == 'left' else 640
        img = np.mean([cv2.remap(fr[:, x0:x0+640].astype(np.float32), mx, my, cv2.INTER_CUBIC) for fr in frames], 0)
        cv2.imwrite(str(out/f'undistorted_{e}.jpg'), np.clip(img, 0, 255).astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 95])

    (out/'board_view.json').write_text(json.dumps({
        'purpose': 'planning reference only; never use these resampled images for source/target poses',
        'sources': [str(s) for s in sources], 'head_q': head_q, 'yaw_direction': a.yaw_direction, 'eyes': eyes,
        'plane_z_m': z, 'pixels_per_mm': a.ppm, 'board_geometry_measured_at': g.get('measured_at'),
        'topdown_intersection_px': pix, 'topdown_padding_px': pad, 'topdown_title_px': 70}, indent=1))
    print(json.dumps({'output': str(out), 'frames': len(frames), 'eyes': eyes, 'files': ['board_topdown.jpg', 'board_tiles.jpg'] +
                      [f'undistorted_{e}.jpg' for e in eyes]}))


if __name__ == '__main__':
    main()
