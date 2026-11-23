#!/usr/bin/env python3
"""Offline: quality of a stage-2 capture: joint jitter, command-vs-feedback error, corner jitter across frames,
drift between the repeated reference poses (board/base movement or hysteresis) and arm drift."""
import argparse, glob, json
from pathlib import Path
import cv2, numpy as np

p = argparse.ArgumentParser(description=__doc__); p.add_argument('root'); a = p.parse_args()
root = Path(a.root); m = json.loads((root/'manifest.json').read_text())
det = cv2.aruco.CharucoDetector(cv2.aruco.CharucoBoard((5, 7), .035, .026, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)))
std = np.array([x['hw_q_std'] for x in m['poses']])
err = np.array([np.array(x['cmd'])-np.r_[x['head_q'], x['waist_q']] for x in m['poses']])
print('joint jitter mrad median/max (pitch, yaw, waist):', (np.median(std, 0)*1e3).round(3), (std.max(0)*1e3).round(3))
print('command - feedback mrad mean/maxabs:', (err.mean(0)*1e3).round(1), (np.abs(err).max(0)*1e3).round(1))
spread, refs = [], {}
for x in m['poses']:
    per = []
    for f in sorted(glob.glob(str(root/x['directory']/'*.jpg'))):
        c, ids, _, _ = det.detectBoard(cv2.imread(f, 0)[:, :640])
        per.append({} if ids is None else {int(i): v for i, v in zip(ids.ravel(), c.reshape(-1, 2))})
    common = set.intersection(*[set(y) for y in per]) if per else set()
    if not common:
        continue
    arr = np.array([[y[i] for i in sorted(common)] for y in per])
    spread.append(float(np.median(np.linalg.norm(arr-np.median(arr, 0), axis=2).max(0))))
    if x['name'].startswith('ref'):
        refs[x['name']] = {i: np.median([y[i] for y in per], 0) for i in common}
print('corner jitter px median/p90/max:', np.round([np.median(spread), np.percentile(spread, 90), max(spread)], 3))
names = list(refs)
for i in range(len(names)):
    for j in range(i+1, len(names)):
        c = set(refs[names[i]]) & set(refs[names[j]])
        v = np.array([refs[names[j]][k]-refs[names[i]][k] for k in c])
        print(f'{names[i]} -> {names[j]}: corner shift median {np.median(np.linalg.norm(v, axis=1)):.2f} px')
print('arm drift max rad', round(float(np.abs(np.array([x['arm_q'] for x in m['poses']])-np.array(m['start_q'])[15:29]).max()), 4))
