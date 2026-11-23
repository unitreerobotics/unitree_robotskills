#!/usr/bin/env python3
"""Offline: IK reachability of sampled board points (5x5 squares) per arm, grasp pitch and yaw, at a fixed
fingertip height. Position IK with a few random seeds; not a path preflight and no collision checking."""
import argparse, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2])); import _common  # noqa
import numpy as np
from scipy.spatial.transform import Rotation as R
from r1d_chess.robot.r1d_config import load_profile
from r1d_chess.robot.r1d_kinematics import R1DKinematics

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--board', default=str(_common.ROOT/'config/board_geometry.json'))
p.add_argument('--tip-z', type=float, help='fingertip height (waist frame); default from robot.json board_release_z_m and release_pitch_rad')
p.add_argument('--pitch', type=float, nargs='+', default=[.35, .6, .785]); p.add_argument('--yaw', type=float, nargs='+', default=[-.5, 0., .5, 1.])
a = p.parse_args()
prof = load_profile(); k = R1DKinematics(prof); g = json.loads(Path(a.board).read_text())
cfg = json.loads(Path(_common.ROOT/'config/robot.json').read_text()); D = cfg['scene']['finger_end']
if a.tip_z is None:
    a.tip_z = cfg['board_release_z_m']-D*np.sin(cfg.get('release_pitch_rad', a.pitch[0]))
init = {s: np.array(prof['init_joint_rad'][s]) for s in ('left', 'right')}
rng = np.random.default_rng(0)
squares = [f'{f}{r}' for r in (0, 2, 5, 7, 9) for f in 'acegi']
def ok(side, xy, th, psi):
    Rm = R.from_euler('xyz', [0, th, psi]).as_matrix(); tcp = np.r_[xy, a.tip_z]-Rm @ [D, 0, 0]
    T = np.eye(4); T[:3, :3] = Rm; T[:3, 3] = tcp; arm = k.arms[0 if side == 'left' else 1]
    for s in range(4):
        seed = init[side] if s == 0 else np.clip(init[side]+rng.normal(0, .5, 7), arm.lower, arm.upper)
        r = arm.solve(T, seed)
        if r['accepted']:
            return True
    return False
for th in a.pitch:
    print(f'pitch {th} (TCP z {a.tip_z+D*np.sin(th):.3f})')
    for side, sign in (('left', -1), ('right', 1)):
        for psi0 in a.yaw:
            m = [ok(side, np.array(g['intersections_xy_m'][sq]), th, sign*psi0) for sq in squares]
            rows = ' '.join(''.join('#' if x else '.' for x in m[i:i+5]) for i in range(0, 25, 5))
            print(f'  {side} yaw {sign*psi0:+.1f} {sum(m)}/25  ranks 0,2,5,7,9 x files a,c,e,g,i: {rows}', flush=True)
