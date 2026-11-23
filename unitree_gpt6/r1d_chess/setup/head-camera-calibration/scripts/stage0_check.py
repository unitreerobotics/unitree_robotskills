#!/usr/bin/env python3
"""Read-only: TeleImager config, one raw head stereo frame, valid columns and ChArUco corners per eye.
No robot commands. Use before stage 1 (board in hand) and stage 2 (board fixed on the table)."""
import argparse, json, pickle, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2])); import _common  # noqa
import cv2, numpy as np, zmq

HOST = _common.CAMERA_HOST
p = argparse.ArgumentParser(description=__doc__); p.add_argument('--output', required=True); a = p.parse_args()
out = Path(a.output); out.mkdir(parents=True, exist_ok=True)
ctx = zmq.Context()
r = ctx.socket(zmq.REQ); r.setsockopt(zmq.LINGER, 0); r.setsockopt(zmq.RCVTIMEO, 3000); r.connect(f'tcp://{HOST}:60000')
r.send(b'GET_CONFIG'); raw = r.recv()
try:
    cfg = json.loads(raw)
except ValueError:
    cfg = pickle.loads(raw)
(out/'teleimager_config.json').write_text(json.dumps(cfg, indent=1, default=str))
print('head stream', cfg['head_camera']['image_shape'], cfg['head_camera']['fps'], 'fps')
s = ctx.socket(zmq.SUB); s.setsockopt(zmq.LINGER, 0); s.setsockopt(zmq.SUBSCRIBE, b''); s.setsockopt(zmq.CONFLATE, 1)
s.setsockopt(zmq.RCVTIMEO, 5000); s.connect(f'tcp://{HOST}:55555')
for _ in range(6):
    frame = s.recv()
(out/'head_stereo.jpg').write_bytes(frame)
g = cv2.imdecode(np.frombuffer(frame, np.uint8), cv2.IMREAD_GRAYSCALE)
det = cv2.aruco.CharucoDetector(cv2.aruco.CharucoBoard((5, 7), .035, .026, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)))
for name, e in (('left', g[:, :640]), ('right', g[:, 640:])):
    cols = np.where(e.mean(0) > 6)[0]
    _, ids, _, mi = det.detectBoard(e)
    print(f'{name}: valid columns {cols.min()}-{cols.max()}  charuco corners {0 if ids is None else len(ids)}/24  markers {0 if mi is None else len(mi)}/17')
