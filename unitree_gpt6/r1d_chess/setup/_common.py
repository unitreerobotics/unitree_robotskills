"""Shared paths for the setup scripts. Import with:
sys.path.insert(0, str(Path(__file__).resolve().parents[2])); import _common
"""
import json as _json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]          # package root
if str(ROOT/'src') not in sys.path:
    sys.path.insert(0, str(ROOT/'src'))

RUNTIME = _json.loads((ROOT/'config/runtime.json').read_text())
CAMERA_HOST = RUNTIME['camera_host']


def arm_gains():
    """Runtime PD gains of both arms (physical left 7 then right 7), read from the bundled Runtime robot config."""
    import numpy as np
    import yaml
    path = ROOT/(RUNTIME.get('runtime_root') or 'third_party/robot_runtime')/'configs/robots/components/joint_groups/bodies/r1_a7_d.yaml'
    groups = {g['schema_key']: g for g in yaml.safe_load(path.read_text())['joint_groups']}
    kp = np.r_[groups['left_arm']['kp'], groups['right_arm']['kp']].astype(float)
    kd = np.r_[groups['left_arm']['kd'], groups['right_arm']['kd']].astype(float)
    return kp, kd
