"""Held-out check: back-project detected board corners onto the calibrated board plane and compare with the 30 mm lattice.
This is the task quantity (pixel -> position on the table). usage: python3 -m calibration.check FRAME_PREFIX..."""
import json, sys, cv2, numpy as np
from robot.rt import *
Twb = np.array(CAL['world_from_board']); Tbw = np.linalg.inv(Twb); allerr = []
for n in sys.argv[1:]:
    d = corners(cv2.imread(n + '.jpg', 0))
    if d is None: print(n, 'no board'); continue
    q = json.load(open(n + '.json'))['q']
    P = pixel_to_plane(q, d[0], Twb[:3, 2], Twb[:3, 3])
    b = ((Tbw[:3, :3] @ P.T).T + Tbw[:3, 3])[:, :2] / SQUARE                      # board coordinates in squares
    shift = np.round(np.median(b - d[1], 0))                                       # lattice is known up to an integer shift
    # local grid -> board: same integer rigid map for all corners; take residual to nearest lattice point
    e = (b - np.round(b)) * SQUARE * 1000
    off = e.mean(0); allerr.append(e)
    print('%-8s %3d corners  mean offset %5.1f %5.1f mm  |offset| %4.1f  scatter %.1f mm  cam height %.2f m' % (
        n, len(e), off[0], off[1], np.linalg.norm(off), np.sqrt(((e - off) ** 2).sum(1).mean()), cam_pose(q)[2, 3]))
E = np.vstack(allerr); M = np.array([e.mean(0) for e in allerr])
print('all corners: rms %.1f mm, 95%% %.1f mm, max %.1f mm; per-frame offsets rms %.1f mm, max %.1f mm' % (
    np.sqrt((E ** 2).sum(1).mean()), np.percentile(np.linalg.norm(E, axis=1), 95), np.linalg.norm(E, axis=1).max(),
    np.sqrt((M ** 2).sum(1).mean()), np.linalg.norm(M, axis=1).max()))
