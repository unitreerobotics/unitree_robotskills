#!/usr/bin/env python3
"""Offline: one iterative-learning update of the Coulomb terms: c_new = clip(c + gain*kp*mean(e*sign(v_des)), 0, 2)."""
import argparse, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2])); import _common  # noqa
import numpy as np

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('run'); p.add_argument('config_in'); p.add_argument('config_out'); p.add_argument('--gain', type=float, default=.5)
a = p.parse_args()
rows = json.loads((Path(a.run)/'samples.json').read_text())
t = np.array([r['t'] for r in rows]); qd = np.array([r['q_des'] for r in rows]); q = np.array([r['q'] for r in rows])
v = np.gradient(qd, t, axis=0); e = qd-q
kp, _ = _common.arm_gains()
c = json.loads(Path(a.config_in).read_text())
cur = np.r_[c['friction']['coulomb_nm']['left'], c['friction']['coulomb_nm']['right']]
delta = np.array([kp[j]*np.mean(e[np.abs(v[:, j]) > .02, j]*np.sign(v[np.abs(v[:, j]) > .02, j])) for j in range(14)])
new = np.clip(cur+a.gain*delta, 0., 2.)
print('delta Nm', np.round(delta, 2).tolist()); print('new   Nm', np.round(new, 2).tolist())
c['friction']['coulomb_nm'] = {'left': [round(x, 3) for x in new[:7]], 'right': [round(x, 3) for x in new[7:]]}
c['evidence'] = c.get('evidence', '')+f' | coulomb ILC update from {Path(a.run).name} gain {a.gain}'
Path(a.config_out).write_text(json.dumps(c, indent=2))
