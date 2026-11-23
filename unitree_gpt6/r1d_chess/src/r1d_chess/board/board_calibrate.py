#!/usr/bin/env python3
"""One-command board calibration from fixed printed features only (`bash r1d boardcal`).

usage:
  bash r1d boardcal                      # capture a head frame, measure, write config/board_geometry.json
  bash r1d boardcal --check              # measure and report the change, do not write
  bash r1d boardcal --full               # first use in a new place / board moved a lot: wide search
  bash r1d boardcal --observation <capture.json> [--check]   # offline, from an existing capture

What it does (pieces are never used; they may be anywhere):
  1. captures N head stereo frames (default 3) through the chess service, unless --observation is given;
  2. registers the printed pattern (rank/file lines, river gap, palace X) on both eyes — default "update"
     mode searches ±20 mm / ±0.06 rad around the stored geometry (a restart or a small bump), --full
     searches ±60 mm / ±0.15 rad; if update mode is not unique or not consistent it retries as --full;
  3. refines every line and intersection (visual_tools/board-points);
  4. accepts only if >= MIN_POINTS intersections were measured, the refinement agrees with the registration
     and the registration is unique; otherwise nothing is written and the reason is printed (exit 3);
  5. writes the geometry (trays are not part of the printed board: kept, moved rigidly with the board) and
     prints the change relative to the previous geometry (shift, rotation, max intersection change).

The waist must be at its chess reference before calibrating: the geometry is in the waist frame, so a
waist offset shows up as a board shift. boardcal prints the waist error and refuses above WAIST_TOL
unless --ignore-waist is given.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from r1d_chess.paths import ROOT
GEOMETRY = ROOT/'config'/'board_geometry.json'
MIN_POINTS = 30
MAX_REFINE_DEV_MM = 4.
MIN_UNIQUE_MARGIN = .5
MAX_OUTSIDE_RATIO = .3
WAIST_TOL_RAD = .012
MODES = {'update': (20., .06), 'full': (60., .15)}


def run(cmd, timeout=300):
    return subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT, timeout=timeout)


def client(*args):
    r = run(['bash', str(ROOT/'r1d'), 'client', *map(str, args)])
    try:
        return json.loads(r.stdout)
    except json.JSONDecodeError:
        return {'ok': False, 'error': (r.stderr or r.stdout)[-300:]}


def measure(observations, out, mode, geometry):
    span, yaw = MODES[mode]
    cmd = ['bash', str(ROOT/'r1d'), 'boardpoints', '--output', str(out), '--geometry', str(geometry),
           '--register-span-mm', str(span), '--register-span-yaw', str(yaw)]
    first = json.loads(Path(observations[0]).read_text())
    if len(observations) == 1:
        cmd += ['--observation', str(observations[0])]
    else:
        for o in observations:
            left = json.loads(Path(o).read_text())['paths']['cam_left_high']
            cmd += ['--image', str(Path(left).with_name(Path(left).stem + '_stereo.jpg'))]
        cmd += ['--head-q', *map(str, first['state']['head_q'])]
    r = run(cmd)
    bp = out/'board_points.json'
    if r.returncode != 0 or not bp.exists():
        return None, (r.stderr or r.stdout)[-400:]
    return json.loads(bp.read_text()), None


def problems(summary):
    out = []
    reg = summary.get('registration') or {}
    if summary['measured_points'] < MIN_POINTS:
        out.append(f"只测到 {summary['measured_points']} 个交叉点（< {MIN_POINTS}）")
    if summary['deviation_from_geometry_mm']['mean'] > MAX_REFINE_DEV_MM:
        out.append(f"逐线精修与整体配准平均相差 {summary['deviation_from_geometry_mm']['mean']} mm（> {MAX_REFINE_DEV_MM}）")
    ru = reg.get('runner_up')
    if ru and reg['score'] - ru['score'] < MIN_UNIQUE_MARGIN:
        out.append(f"配准不唯一（最优 {reg['score']}，次优 {ru['score']}）")
    return out


def change(old, new_points):
    a = np.array([old['intersections_xy_m'][k] for k in sorted(new_points)])
    b = np.array([new_points[k] for k in sorted(new_points)])
    d = np.linalg.norm(b - a, axis=1)*1000
    ca, cb = a.mean(0), b.mean(0)
    H = (a - ca).T @ (b - cb)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    return dict(shift_mm=[round(float(x), 2) for x in (cb - ca)*1000],
                rotation_mrad=round(float(math.atan2(R[1, 0], R[0, 0]))*1000, 2),
                intersection_change_mm=dict(mean=round(float(d.mean()), 2), max=round(float(d.max()), 2))), (ca, cb, R)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--observation', type=Path, nargs='+', help='existing capture response(s) instead of capturing')
    p.add_argument('--frames', type=int, default=3)
    p.add_argument('--full', action='store_true', help='wide search (new environment / board moved a lot)')
    p.add_argument('--check', action='store_true', help='measure and report only')
    p.add_argument('--ignore-waist', action='store_true')
    p.add_argument('--output', type=Path, help='run directory (default runs/<time>_boardcal)')
    p.add_argument('--geometry', type=Path, help='prior geometry instead of config/board_geometry.json (implies --check)')
    a = p.parse_args()
    t0 = time.time()
    out = a.output or ROOT/'runs'/(time.strftime('%Y%m%d_%H%M%S') + '_boardcal')
    (out/'tmp').mkdir(parents=True, exist_ok=True)
    result = dict(run=str(out), written=False)
    if a.observation:
        obs = [o if o.is_absolute() else ROOT/o for o in a.observation]
    else:
        st = client('get_state')
        if not st.get('ok'):
            raise SystemExit(f'[ERROR] 象棋服务不可用：{st.get("error")}')
        waist_err = abs(st['waist_q'][0] - st['waist_reference_q'][0])
        result['waist_error_rad'] = round(waist_err, 4)
        if waist_err > WAIST_TOL_RAD and not a.ignore_waist:
            raise SystemExit(f'[ERROR] 腰偏离参考位 {waist_err*1000:.1f} mrad（> {WAIST_TOL_RAD*1000:.0f}）：先把腰调回参考位'
                             f'（bash r1d start），或加 --ignore-waist 按当前姿态标定')
        obs = []
        for i in range(a.frames):
            path = out/'tmp'/f'obs_{i}.json'
            r = client('capture_cameras', '--cameras', 'cam_left_high', '--output-dir', out/'images', '--save-response', path)
            if not r.get('ok'):
                raise SystemExit(f'[ERROR] 头部相机取图失败：{r.get("error")}（头部断流时运行 bash r1d headcam）')
            obs.append(path)
            time.sleep(.2)
    prior = a.geometry or GEOMETRY
    if a.geometry:
        a.check = True
    old = json.loads(Path(prior).read_text())
    mode = 'full' if a.full else 'update'
    data, err = measure(obs, out/f'bp_{mode}', mode, prior)
    bad = [err] if err else problems(data['summary'])
    if bad and mode == 'update':
        result['update_mode_problems'] = bad
        mode = 'full'
        data, err = measure(obs, out/'bp_full', mode, prior)
        bad = [err] if err else problems(data['summary'])
    result.update(mode=mode, seconds=round(time.time() - t0, 1))
    if bad:
        result.update(ok=False, problems=bad)
        print(json.dumps(result, ensure_ascii=False, indent=1))
        raise SystemExit(3)
    s = data['summary']
    new_points = {k: v['xyz_waist_m'][:2] for k, v in data['points'].items()}
    delta, (ca, cb, R) = change(old, new_points)
    result.update(ok=True, measured_points=s['measured_points'], centre_xy_m=s['centre_xy_m'], yaw_rad=s['yaw_rad'],
                  cell_m=s['cell_m'], registration_score=s['registration']['score'],
                  runner_up_score=(s['registration']['runner_up'] or {}).get('score'), change=delta,
                  overlay=str(out/f'bp_{mode}'/'left_points_x2.jpg'),
                  registration_overlay=str(out/f'bp_{mode}'/'registration_topdown.jpg'))
    if not a.check:
        g = dict(old)
        g['previous_fit'] = {k: old[k] for k in ('centre_xy_m', 'yaw_rad', 'cell_m')}
        g.update(centre_xy_m=s['centre_xy_m'], yaw_rad=s['yaw_rad'], cell_m=s['cell_m'], intersections_xy_m=new_points,
                 intersections_source='scripts/board_calibrate.py: printed-pattern registration + line fit',
                 intersections_evidence=str((out/f'bp_{mode}'/'board_points.json').resolve()),
                 measured_at=time.strftime('%Y-%m-%d %H:%M:%S'))
        # trays stand beside the board: move them rigidly with it
        g['trays_xy_m'] = {k: [round(float(x), 4) for x in (cb + R @ (np.array(v) - ca))]
                           for k, v in old.get('trays_xy_m', {}).items()}
        (out/'board_geometry_before.json').write_text(json.dumps(old, indent=1))
        GEOMETRY.write_text(json.dumps(g, indent=1))
        result['written'] = True
    result['seconds'] = round(time.time() - t0, 1)
    print(json.dumps(result, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
