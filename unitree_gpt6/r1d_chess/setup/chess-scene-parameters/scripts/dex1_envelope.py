#!/usr/bin/env python3
"""Offline: Dex1 gripper envelope in each wrist_yaw frame from the URDF meshes (fingers closed and fully open).
Cross-check against the drawing (prompts/dex1.jpg): flange to tip 143.2 mm, open width 117 mm, finger 32 mm wide."""
import argparse, re, struct, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2])); import _common  # noqa
import numpy as np, xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
from r1d_chess.robot.r1d_config import MODEL_PATH

p = argparse.ArgumentParser(description=__doc__); p.add_argument('--urdf', default=str(MODEL_PATH)); a = p.parse_args()
base = Path(a.urdf).parent; root = ET.parse(a.urdf).getroot()
J = {j.get('name'): j for j in root.findall('joint')}; L = {l.get('name'): l for l in root.findall('link')}
v = lambda e, k: np.fromstring(e.get(k, '0 0 0'), sep=' ')
def T(j):
    o = j.find('origin'); m = np.eye(4); m[:3, :3] = R.from_euler('xyz', v(o, 'rpy')).as_matrix(); m[:3, 3] = v(o, 'xyz'); return m
def mesh(link):
    g = L[link].find('visual').find('geometry').find('mesh').get('filename')
    f = base/g if (base/g).exists() else base/'meshes'/Path(g).name
    raw = f.read_bytes(); n = struct.unpack('<I', raw[80:84])[0]
    if 84+50*n == len(raw):
        return np.frombuffer(raw[84:], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))['v'].reshape(-1, 3).astype(float)
    return np.array(re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', raw), dtype=float)
np.set_printoptions(precision=1, suppress=True)
for side in ('left', 'right'):
    Tb = T(J[f'{side}_dex1_base_joint']); pts = []
    for link, joint in ((f'{side}_dex1_base_link', None), (f'{side}_dex1_finger_link_1', f'{side}_dex1_finger_joint_1'),
                        (f'{side}_dex1_finger_link_2', f'{side}_dex1_finger_joint_2')):
        tr = Tb if joint is None else Tb @ T(J[joint])
        W = (tr @ np.c_[mesh(link), np.ones(len(mesh(link)))].T)[:3].T; pts.append(W)
        if joint:
            pts.append(W+tr[:3, :3] @ v(J[joint].find('axis'), 'xyz')*float(J[joint].find('limit').get('upper')))
    A = np.vstack(pts); flange = Tb[:3, 3]
    print(f'{side}: flange (TCP) mm {flange*1e3}  envelope open min {A.min(0)*1e3} max {A.max(0)*1e3}')
    print(f'  flange->tip {(A[:, 0].max()-flange[0])*1e3:.1f} mm   open width {(A[:, 1].max()-A[:, 1].min())*1e3:.1f} mm')
