#!/usr/bin/env python3
"""Offline: where the remaining error is. Moving vs static RMS, error/velocity vs kd/kp per joint (damping lag),
the five worst peaks with their commanded velocity, and the error during the first 0.4 s after motion starts."""
import argparse, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2])); import _common  # noqa
import numpy as np

p = argparse.ArgumentParser(description=__doc__); p.add_argument('run'); a = p.parse_args()
rows = json.loads((Path(a.run)/'samples.json').read_text())
t = np.array([r['t'] for r in rows]); qd = np.array([r['q_des'] for r in rows]); q = np.array([r['q'] for r in rows])
v = np.gradient(qd, t, axis=0); e = qd-q
kp, kd = _common.arm_gains()
names = ['L_sp', 'L_sr', 'L_sy', 'L_el', 'L_wr', 'L_wp', 'L_wy', 'R_sp', 'R_sr', 'R_sy', 'R_el', 'R_wr', 'R_wp', 'R_wy']
mv = np.abs(v).max(1) > .01
print('rms moving / static (mrad):', round(np.sqrt((e[mv]**2).mean())*1e3, 1), round(np.sqrt((e[~mv]**2).mean())*1e3, 1))
print('joint  e/v fit (s)  kd/kp (s)  rms moving (mrad)')
for j in range(14):
    m = np.abs(v[:, j]) > .02
    k = float(np.linalg.lstsq(v[m, j:j+1], e[m, j], rcond=None)[0][0])
    print(f'{names[j]}   {k:7.3f}   {kd[j]/kp[j]:7.3f}   {np.sqrt((e[m, j]**2).mean())*1e3:6.1f}')
print('worst peaks:')
for j in np.argsort(-np.abs(e).max(0))[:5]:
    i = int(np.argmax(np.abs(e[:, j])))
    print(f'  {names[j]} {e[i, j]*1e3:+.1f} mrad at t={t[i]:.2f}s v_des={v[i, j]:+.3f} rad/s')
start = np.zeros_like(e, bool); dt = np.median(np.diff(t))
for j in range(14):
    m = np.abs(v[:, j]) > .005
    for k in np.where(m & ~np.r_[False, m[:-1]])[0]:
        start[k:k+int(.4/dt), j] = True
m = np.abs(v) > .005
print('rms moving: first 0.4 s after start', round(np.sqrt((e[start & m]**2).mean())*1e3, 1), 'mrad; rest', round(np.sqrt((e[~start & m]**2).mean())*1e3, 1))
