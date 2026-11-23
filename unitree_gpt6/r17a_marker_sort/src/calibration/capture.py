"""Run the planned poses: move -> still frame + joints -> detect board. Never stops or disconnects the arm (that drops torque)."""
import json, sys, cv2, numpy as np
from robot.rt import *
plan = json.load(open('calib_plan.json'))
s = state(); assert s['armed'] and not s['fault'] and not s['busy'], s['fault']
start = int(sys.argv[1]) if len(sys.argv) > 1 else 0   # resume: arm must be at pose start-1 or start
n = 0
for i, q in enumerate(plan['poses']):
    if i < start: continue
    qm = move(np.array(q))
    jpg, meta = snap(f'{sys.argv[2] if len(sys.argv) > 2 else "cal"}/{i:02d}')
    c = corners(cv2.imdecode(np.frombuffer(jpg, np.uint8), 0))
    n += c is not None
    print(i, 'NO BOARD' if c is None else '%d corners' % len(c[0]), 'err %.4f' % np.abs(qm - q).max(), flush=True)
move(np.array(plan['q0']))
print('detected', n, 'of', len(plan['poses']))
