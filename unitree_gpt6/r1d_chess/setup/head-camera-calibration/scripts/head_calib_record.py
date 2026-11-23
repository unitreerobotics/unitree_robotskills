#!/usr/bin/env python3
"""Record the head stereo stream as the original JPEGs, with a live preview. Read-only: no robot commands.

Frames are stored byte-for-byte (no re-encoding) so corner accuracy is not degraded. q/Esc stops early.
"""
import argparse
import json
import os
import time
from pathlib import Path
import cv2
import numpy as np
import zmq

HOST = json.loads((Path(__file__).resolve().parents[3]/'config/runtime.json').read_text())['camera_host']


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True)
    p.add_argument('--seconds', type=float, default=150.)
    a = p.parse_args()
    out = Path(a.output)
    (out/'frames').mkdir(parents=True, exist_ok=False)
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.SUBSCRIBE, b'')
    sock.setsockopt(zmq.RCVTIMEO, 3000)
    sock.connect(f'tcp://{HOST}:55555')
    cv2.namedWindow('head recording', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('head recording', 1280, 540)
    started, count, index, reason = time.monotonic(), 0, [], 'time'
    while (left := a.seconds-(time.monotonic()-started)) > 0:
        raw = sock.recv()
        name = f'{count:06d}.jpg'
        (out/'frames'/name).write_bytes(raw)
        index.append({'frame': name, 't': round(time.monotonic()-started, 4)})
        count += 1
        if count % 2 == 0:
            image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
            bar = np.zeros((60, image.shape[1], 3), np.uint8)
            cv2.putText(bar, f'RECORDING {left:5.0f} s left   frames {count}   q = stop', (12, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, .9, (255, 255, 255), 2)
            cv2.imshow('head recording', np.vstack((image, bar)))
            if cv2.waitKey(1) & 255 in (ord('q'), 27):
                reason = 'operator'
                break
    (out/'index.json').write_text(json.dumps(index))
    cv2.destroyAllWindows()
    sock.close()
    ctx.term()
    print(json.dumps({'frames': count, 'seconds': round(time.monotonic()-started, 1), 'reason': reason, 'output': str(out)}))


if __name__ == '__main__':
    main()
