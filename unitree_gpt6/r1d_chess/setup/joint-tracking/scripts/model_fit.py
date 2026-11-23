#!/usr/bin/env python3
"""Offline: compare settled tau_est with URDF gravity (one or more models). Use a run whose errors converged
(gravity_i or full), so tau_est approximates the real static load. Prints per-joint LS scale per model."""
import argparse, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2])); import _common  # noqa
import numpy as np
from r1d_chess.robot.r1d_config import MODEL_PATH
from r1d_chess.robot.r1d_gravity import ArmGravity

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('run'); p.add_argument('--urdf', action='append', help='repeatable; default: active model')
a = p.parse_args()
models = {Path(u).name: ArmGravity(u) for u in (a.urdf or [str(MODEL_PATH)])}
rows = json.loads((Path(a.run)/'samples.json').read_text())
t = np.array([r['t'] for r in rows]); q = np.array([r['q'] for r in rows]); te = np.array([r['tau_est'] for r in rows])
qd = np.array([r['q_des'] for r in rows])
mask = (np.abs(np.gradient(qd, t, axis=0)).max(1) < 1e-5) & (np.abs(qd-q).max(1) < .02)
idx = np.where(mask)[0][::20]
print('settled samples', len(idx))
M = te[idx]
names = ['L_sp', 'L_sr', 'L_sy', 'L_el', 'L_wr', 'L_wp', 'L_wy', 'R_sp', 'R_sr', 'R_sy', 'R_el', 'R_wr', 'R_wp', 'R_wy']
G = {n: np.array([m.torque(q[i]) for i in idx]) for n, m in models.items()}
print('joint  measured ' + ' '.join(f'{n[:14]:>14s}(mean rms scale)' for n in G))
for j in range(14):
    cells = []
    for n, X in G.items():
        sc = float(X[:, j] @ M[:, j]/max(X[:, j] @ X[:, j], 1e-9))
        cells.append(f'{X[:, j].mean():+6.2f} {np.sqrt(((M[:, j]-X[:, j])**2).mean()):5.2f} {sc:5.2f}')
    print(f'{names[j]:5s} {M[:, j].mean():+7.2f}  ' + '  '.join(cells))
