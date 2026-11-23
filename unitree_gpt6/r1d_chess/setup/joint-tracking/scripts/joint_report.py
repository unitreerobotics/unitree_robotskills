#!/usr/bin/env python3
"""Offline: per-joint RMS/max error, static error per dwell window and load vs model gravity for one run."""
import argparse, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2])); import _common  # noqa
import numpy as np
import arm_tracking_benchmark as bench
from r1d_chess.robot.r1d_gravity import ArmGravity

p = argparse.ArgumentParser(description=__doc__); p.add_argument('run'); a = p.parse_args()
s = bench.summarize(a.run)
spec = json.loads((Path(a.run)/'trajectory.json').read_text())
keys = {k['name']: np.array(k['q']) for k in spec['keyframes']}
g = ArmGravity()
print(s['meta']['path'], s['meta']['ff'], 'completed', s['overall']['completed'], s['overall'].get('error'))
w = s['windows']; plus = next((k for k in w if 'plus' in k), None)
print('motor joint                rms  maxabs (mrad) | static error per window (mrad) | tau_est vs g(q) at plus (Nm)')
for j, J in enumerate(s['joints']):
    st = [round(v*1e3) for v in J['static_error_rad'].values()]
    load = f"{w[plus]['tau_load_nm_est'][j]:+.2f} / {g.torque(keys['plus'])[j]:+.2f}" if plus else '-'
    print(f"{J['motor']} {J['joint']:<22} {J['rms_error_rad']*1e3:5.1f} {J['max_abs_error_rad']*1e3:6.1f} | {st} | {load}")
o = s['overall']
print('TCP rms/max mm  L', round(o['tcp_error_m']['left']['rms']*1e3, 1), round(o['tcp_error_m']['left']['max']*1e3, 1),
      ' R', round(o['tcp_error_m']['right']['rms']*1e3, 1), round(o['tcp_error_m']['right']['max']*1e3, 1))
