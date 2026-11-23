#!/usr/bin/env python3
"""Offline: detect ChArUco corners in every recorded stereo frame. Writes detections.json."""
import json
import sys
from multiprocessing import Pool
from pathlib import Path
import cv2
import numpy as np

W = 640
_det = None


def init():
    global _det
    cv2.setNumThreads(1)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    _det = cv2.aruco.CharucoDetector(cv2.aruco.CharucoBoard((5, 7), .035, .026, d))


def work(path):
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    out = {'frame': path.name}
    for eye, gray in (('left', image[:, :W]), ('right', image[:, W:])):
        c, ids, _, _ = _det.detectBoard(gray)
        out[eye] = {} if ids is None else {str(int(i)): p.tolist() for i, p in zip(ids.ravel(), c.reshape(-1, 2))}
    return out


def main():
    root = Path(sys.argv[1])
    frames = sorted((root/'frames').glob('*.jpg'))
    with Pool(8, init) as pool:
        result = pool.map(work, frames, chunksize=32)
    (root/'detections.json').write_text(json.dumps(result))
    n = np.array([[len(r['left']), len(r['right'])] for r in result])
    print('frames', len(result), 'with>=8 L', int((n[:, 0] >= 8).sum()), 'R', int((n[:, 1] >= 8).sum()),
          'both>=8', int(((n >= 8).all(1)).sum()), 'max', n.max(0).tolist(), 'median nonzero', np.median(n[n[:, 0] > 0, 0]) if (n[:, 0] > 0).any() else 0)


main()
