#!/usr/bin/env python3
"""Fit board-plane points in R1-D waist_yaw_link; never infers gripper TCP poses."""
import argparse
import json
from pathlib import Path
import re
import numpy as np


def fit(data):
    if data.get('frame') != 'waist_yaw_link' or not data.get('evidence'):
        raise ValueError('measured board points require waist_yaw_link frame and evidence')
    spacing = np.asarray(data['spacing_m'], dtype=float)
    if spacing.shape != (2,) or not np.isfinite(spacing).all() or np.any(spacing <= 0):
        raise ValueError('measured column/row spacing required')
    rows = data['points']
    if len(rows) < 4 or len({r['square'] for r in rows}) != len(rows):
        raise ValueError('at least four distinct surveyed intersections required')
    local, measured = [], []
    for row in rows:
        s = row['square']
        if not re.fullmatch('[a-i][0-9]', s):
            raise ValueError('invalid board square')
        local.append([(ord(s[0])-ord('a'))*spacing[0], int(s[1])*spacing[1], 0.])
        measured.append(row['xyz_m'])
    local, measured = np.asarray(local), np.asarray(measured, dtype=float)
    if measured.shape != local.shape or not np.isfinite(measured).all():
        raise ValueError('every intersection requires measured XYZ in metres')
    if np.linalg.matrix_rank(local-local.mean(axis=0)) < 2:
        raise ValueError('surveyed points are collinear')
    u, _, vt = np.linalg.svd((local-local.mean(axis=0)).T@(measured-measured.mean(axis=0)))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    translation = measured.mean(axis=0)-rotation@local.mean(axis=0)
    residual = local@rotation.T+translation-measured
    norms = np.linalg.norm(residual, axis=1)
    if norms.max() > data.get('max_fit_error_m', .005):
        raise ValueError(f'board survey inconsistent: maximum residual {norms.max():.6f} m')
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = rotation, translation
    result = {'frame':'waist_yaw_link','board_to_frame':T.tolist(),
              'spacing_m':spacing.tolist(),'fit_rms_m':float(np.sqrt(np.mean(norms**2))),
              'fit_max_m':float(norms.max()),'evidence':data['evidence'],
              'points':{},'scope':'board surface intersections only; not source/target TCP poses or current piece occupancy'}
    for column in 'abcdefghi':
        for rank in range(10):
            point=np.array([(ord(column)-ord('a'))*spacing[0],rank*spacing[1],0.])
            result['points'][column+str(rank)]=(rotation@point+translation).tolist()
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--survey',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    result=fit(json.loads(args.survey.read_text()))
    with args.output.open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(json.dumps({'output':str(args.output),'fit_rms_m':result['fit_rms_m'],'points':90}))


if __name__=='__main__':main()
