"""R1-D geometry configuration and provenance checks; no hardware imports."""
import json
import os
from pathlib import Path
import sys
import numpy as np

from r1d_chess.paths import ROOT
PROFILE = ROOT / 'config/robot.json'
RUNTIME = json.loads((ROOT / 'config/runtime.json').read_text())
# Kinematic model: R1D_MODEL_URDF, else runtime.json model_urdf (unitree_ros robots/r1d_description/r1d.urdf,
# includes Dex1). A relative path is resolved against the package root (assets/models/r1d/r1d.urdf).
MODEL_PATH = ROOT / Path(os.environ.get('R1D_MODEL_URDF') or RUNTIME['model_urdf'])


def load_profile(path=None):
    path = Path(path or os.environ.get('R1D_CHESS_PROFILE', PROFILE)).resolve()
    value = json.loads(path.read_text())
    if value.get('robot') != 'R1-D' or value.get('schema_version') != 1:
        raise ValueError('expected R1-D profile schema_version=1')
    return value


def finite_vector(value, n, label):
    if (not isinstance(value, list) or len(value) != n or
            any(type(x) not in (int, float) for x in value)):
        raise ValueError(f'{label}: requires {n} numeric values')
    array = np.asarray(value, dtype=float)
    if not np.isfinite(array).all():
        raise ValueError(f'{label}: non-finite geometry')
    return array


def model_sha256(path=None):
    import hashlib
    return hashlib.sha256(Path(path or MODEL_PATH).read_bytes()).hexdigest()


def check_kinematic_model(profile):
    """Refuse profiles whose init/scene geometry was not derived on the active URDF.

    A profile must name the URDF it was computed with (kinematic_model_sha256). Profiles
    made on the older Runtime r1_a7.urdf (no Dex1, different wrist) are rejected; derive a
    fresh per-round profile from config/robot.json instead.
    """
    recorded = profile.get('kinematic_model_sha256')
    active = model_sha256()
    if recorded != active:
        raise ValueError(
            f'profile kinematic_model_sha256={recorded!r} does not match active URDF {MODEL_PATH} '
            f'({active[:16]}...); derive the profile from config/robot.json and re-estimate geometry')
    return profile


def head_waist_reference(profile):
    """Measured chess reference; profiles predating snapshots default to zero."""
    head = finite_vector(profile.get('head_q_rad', [0., 0.]), 2, 'head reference')
    waist = finite_vector(profile.get('waist_q_rad', [0.]), 1, 'waist reference')
    reference = np.concatenate((head, waist))
    if np.any(np.abs(reference) > [0.62832, 2.0071, 2.618]):
        raise ValueError('head/waist reference outside R1-D limits')
    return head, waist


def geometry_source(profile):
    """A model estimate is usable when its provenance is explicit and complete."""
    if profile.get('calibration_verified') is True:
        if not profile.get('calibration_evidence') or profile.get('camera_mapping_verified') is not True:
            raise ValueError('verified geometry requires calibration evidence and verified camera mapping')
        return 'verified'
    if profile.get('geometry_source') != 'model_estimate':
        raise ValueError('R1-D geometry missing; supply verified calibration or documented model estimates')
    if not isinstance(profile.get('estimate_evidence'), str) or not profile['estimate_evidence'].strip():
        raise ValueError('model estimates require estimate_evidence covering geometry and camera mapping')
    return 'model_estimate'


def require_calibration(profile=None, *, hardware=False):
    p = load_profile() if profile is None else profile
    check_kinematic_model(p)
    if p.get('frame') != 'waist_yaw_link':
        raise ValueError('R1-D chess profile frame must be waist_yaw_link')
    geometry_source(p)
    if hardware and p.get('simulation_only') is not False:
        raise ValueError('simulation profile cannot enable real hardware')
    head_waist_reference(p)
    for side in ('left', 'right'):
        finite_vector(p['tcp_xyz_rpy'][side], 6, f'{side} wrist-to-TCP')
        finite_vector(p['init_xyz_rpy'][side], 6, f'{side} init')
    if p.get('head_eye') not in ('left', 'right'):
        raise ValueError('head_eye must be left or right')
    for key in ('feedback_timeout_s','arm_tracking_limit_rad','head_waist_limit_rad',
                'lift_tolerance_m','max_motor_temperature_c'):
        value = p.get(key)
        if type(value) not in (int,float) or not np.isfinite(value) or value <= 0:
            raise ValueError(f'invalid {key}')
    if type(p.get('lift_height_m')) not in (int, float) or not np.isfinite(p['lift_height_m']):
        raise ValueError('calibration lift_height_m is required')
    finite_vector([p.get('board_release_z_m'), p.get('tray_release_z_m')], 2, 'release heights')
    from r1d_chess.robot.board_guard import BoardGuard
    BoardGuard.from_dict(p.get('scene'))
    return p
