#!/usr/bin/env python3
"""Offline: per-joint Coulomb friction from gravity-only runs: (tau_est - g(q)) regressed on sign(dq_des)."""
import argparse, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2])); import _common  # noqa
import numpy as np
from r1d_chess.robot.r1d_gravity import ArmGravity

p = argparse.ArgumentParser(description=__doc__); p.add_argument('runs', nargs='+'); p.add_argument('--urdf'); a = p.parse_args()
g = ArmGravity(a.urdf)
Rr, S = [], []
for run in a.runs:
    rows = json.loads((Path(run)/'samples.json').read_text())
    t = np.array([r['t'] for r in rows]); qd = np.array([r['q_des'] for r in rows])
    q = np.array([r['q'] for r in rows]); te = np.array([r['tau_est'] for r in rows])
    idx = np.arange(0, len(rows), 5)
    Rr.append(te[idx]-np.array([g.torque(q[i]) for i in idx])); S.append(np.gradient(qd, t, axis=0)[idx])
Rr, S = np.vstack(Rr), np.vstack(S)
names = ['L_sp', 'L_sr', 'L_sy', 'L_el', 'L_wr', 'L_wp', 'L_wy', 'R_sp', 'R_sr', 'R_sy', 'R_el', 'R_wr', 'R_wp', 'R_wy']
out = []
for j in range(14):
    m = np.abs(S[:, j]) > .02
    A = np.c_[np.sign(S[m, j]), np.ones(m.sum())]
    c, b = np.linalg.lstsq(A, Rr[m, j], rcond=None)[0]
    out.append(max(0., float(c)))
    print(f'{names[j]}  n={m.sum():4d}  coulomb {c:+.2f} Nm  offset {b:+.2f}  resid rms {np.sqrt(((Rr[m, j]-A@[c, b])**2).mean()):.2f}')
print(json.dumps({'left': [round(x, 3) for x in out[:7]], 'right': [round(x, 3) for x in out[7:]]}))
