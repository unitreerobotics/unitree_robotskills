#!/usr/bin/env python3
"""Live stereo ChArUco capture for head-camera intrinsics. Read-only: images only, no robot commands.

A preview window shows detected corners and image coverage. Hold the board still for about half a
second and a stereo frame is saved when it adds a new view. q/Esc ends; also stops at --target saves
or --timeout seconds. status.json is rewritten continuously for remote monitoring.
"""
import argparse
import json
import os
import time
from datetime import datetime
from pathlib import Path
import cv2
import numpy as np
import zmq

HOST, PORT = json.loads((Path(__file__).resolve().parents[3]/'config/runtime.json').read_text())['camera_host'], 55555
X0, X1, W, H = 37, 602, 640, 480          # valid columns inside each padded eye
GX, GY = 8, 6


def board():
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    return cv2.aruco.CharucoBoard((5, 7), .035, .026, d)


def detect(det, gray):
    corners, ids, _, _ = det.detectBoard(gray)
    if ids is None or len(ids) == 0:
        return {}
    return {int(i): c for i, c in zip(ids.ravel(), corners.reshape(-1, 2))}


def cells(points):
    out = set()
    for x, y in points:
        out.add((min(GX-1, max(0, int((x-X0)/(X1-X0)*GX))), min(GY-1, int(y/H*GY))))
    return out


def motion(a, b):
    common = set(a) & set(b)
    if len(common) < 4:
        return np.inf
    return float(np.median([np.linalg.norm(a[i]-b[i]) for i in common]))


def novelty(current, saved):
    """Smallest median corner distance to any saved view of the same eye."""
    best = np.inf
    for old in saved:
        best = min(best, motion(current, old))
    return best


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True)
    p.add_argument('--target', type=int, default=60)
    p.add_argument('--timeout', type=float, default=900.)
    p.add_argument('--min-corners', type=int, default=8)
    p.add_argument('--novelty-px', type=float, default=25.)
    p.add_argument('--still-px', type=float, default=.6)
    p.add_argument('--still-s', type=float, default=.5)
    a = p.parse_args()
    out = Path(a.output)
    (out/'images').mkdir(parents=True, exist_ok=True)
    det = cv2.aruco.CharucoDetector(board())
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.SUBSCRIBE, b'')
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt(zmq.RCVTIMEO, 3000)
    sock.connect(f'tcp://{HOST}:{PORT}')
    saved = {'left': [], 'right': []}
    coverage = {'left': np.zeros((GY, GX), int), 'right': np.zeros((GY, GX), int)}
    records, last, still_since, last_save = [], None, None, 0.
    started = time.monotonic()
    frames = 0
    cv2.namedWindow('head calibration', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('head calibration', 1280, 560)
    reason = 'timeout'

    def status(state):
        tmp = out/'status.json.tmp'
        tmp.write_text(json.dumps({'state': state, 'saved': len(records), 'target': a.target, 'frames': frames,
                                   'elapsed_s': round(time.monotonic()-started, 1),
                                   'coverage_cells': {k: int((v > 0).sum()) for k, v in coverage.items()},
                                   'cells_total': GX*GY}))
        os.replace(tmp, out/'status.json')

    while time.monotonic()-started < a.timeout and len(records) < a.target:
        raw = sock.recv()
        image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (H, 2*W):
            continue
        frames += 1
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        now = time.monotonic()
        found = {'left': detect(det, gray[:, :W]), 'right': detect(det, gray[:, W:])}
        best_eye = max(found, key=lambda e: len(found[e]))
        enough = len(found[best_eye]) >= a.min_corners
        moved = motion(found[best_eye], last) if last else np.inf
        last = found[best_eye] if enough else None
        still_since = (still_since or now) if enough and moved < a.still_px else None
        held = still_since is not None and now-still_since >= a.still_s
        new = min(novelty(found[e], saved[e]) for e in found if len(found[e]) >= a.min_corners) if enough else 0.
        message = 'show board' if not enough else 'moving' if still_since is None else 'hold...'
        if enough and held and new < a.novelty_px:
            message = 'seen: move to a new place/tilt'
        if enough and held and new >= a.novelty_px and now-last_save > .8:
            name = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            (out/'images'/(name+'.jpg')).write_bytes(raw)
            record = {'image': f'images/{name}.jpg'}
            for eye, points in found.items():
                record[eye] = {str(k): v.tolist() for k, v in points.items()}
                if len(points) >= a.min_corners:
                    saved[eye].append(points)
                    for x, y in cells(points.values()):
                        coverage[eye][y, x] += 1
            records.append(record)
            (out/'detections.json').write_text(json.dumps(records))
            last_save, still_since = now, None
            message = f'SAVED {len(records)}'
        view = image.copy()
        for offset, eye in ((0, 'left'), (W, 'right')):
            for y in range(GY):
                for x in range(GX):
                    if coverage[eye][y, x]:
                        x0 = int(offset+X0+(X1-X0)*x/GX)
                        x1 = int(offset+X0+(X1-X0)*(x+1)/GX)
                        y0, y1 = H*y//GY, H*(y+1)//GY
                        view[y0:y1, x0:x1, 1] = np.minimum(255, view[y0:y1, x0:x1, 1].astype(int)+50)
            for u, v in found[eye].values():
                cv2.circle(view, (int(u)+offset, int(v)), 3, (0, 0, 255), -1)
        bar = np.zeros((80, 2*W, 3), np.uint8)
        text = (f'{message}   saved {len(records)}/{a.target}   corners L{len(found["left"])} R{len(found["right"])}'
                f'   coverage L{int((coverage["left"] > 0).sum())} R{int((coverage["right"] > 0).sum())}/{GX*GY}')
        cv2.putText(bar, text, (12, 50), cv2.FONT_HERSHEY_SIMPLEX, .9, (255, 255, 255), 2)
        cv2.imshow('head calibration', np.vstack((view, bar)))
        if frames % 15 == 0:
            status('running')
        if cv2.waitKey(1) & 255 in (ord('q'), 27):
            reason = 'operator'
            break
    else:
        if len(records) >= a.target:
            reason = 'target'
    status('finished:'+reason)
    cv2.destroyAllWindows()
    sock.close()
    ctx.term()
    print(json.dumps({'saved': len(records), 'reason': reason, 'output': str(out),
                      'coverage_cells': {k: int((v > 0).sum()) for k, v in coverage.items()}}))


if __name__ == '__main__':
    main()
