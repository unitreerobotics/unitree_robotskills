#!/usr/bin/env python3
"""Install the arm / waist / head calibration results of r1_d into the chess configuration.

  python setup/arm-calibration/scripts/install_results.py            # dry run: print what would change
  python setup/arm-calibration/scripts/install_results.py --write

Sources (setup/arm-calibration/r1_d/assets/r1d/):
  r1d_calibrated.urdf   fitted link mass/COM and joint <dynamics friction load_friction> of both arms
  hold_joints.json      waist / head friction, bias and motor gains

Targets:
  assets/models/r1d/r1d_arm_dynamics.urdf   copy of the fitted URDF (torque feedforward only; FK/IK keep r1d.urdf)
  config/arm_torque_ff.json                 model_urdf, friction.coulomb_nm, friction.load_coefficient, gravity.scale = 1
  config/hold_joints.json                   copy
  config/runtime.json                       joint_gains of waist and head (Runtime must be restarted to load them)
Integral, damping, limits and the stiction law of arm_torque_ff.json are kept.
"""
import argparse
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
R1D = ROOT/'setup/arm-calibration/r1_d/assets/r1d'
JOINTS = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')
TARGET_URDF = 'assets/models/r1d/r1d_arm_dynamics.urdf'


def friction(urdf):
    joints = {j.attrib['name']: j for j in ET.parse(urdf).getroot().iter('joint')}
    out = {}
    for key in ('friction', 'load_friction'):
        out[key] = {}
        for side in ('left', 'right'):
            values = []
            for name in JOINTS:
                dynamics = joints[f'{side}_{name}_joint'].find('dynamics')
                if dynamics is None:
                    raise SystemExit(f'{urdf}: {side}_{name}_joint has no <dynamics>; is this a fitted URDF?')
                values.append(round(float(dynamics.attrib.get(key, 0.)), 3))
            out[key][side] = values
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--urdf', type=Path, default=R1D/'r1d_calibrated.urdf')
    p.add_argument('--hold', type=Path, default=R1D/'hold_joints.json')
    p.add_argument('--evidence', default='', help='where the calibration data and reports of this robot are kept')
    p.add_argument('--write', action='store_true')
    a = p.parse_args()
    f = friction(a.urdf)
    hold = json.loads(a.hold.read_text())
    ff_path, rt_path = ROOT/'config/arm_torque_ff.json', ROOT/'config/runtime.json'
    ff, rt = json.loads(ff_path.read_text()), json.loads(rt_path.read_text())
    new_ff = json.loads(json.dumps(ff))
    new_ff['model_urdf'] = TARGET_URDF
    new_ff['gravity']['scale'] = {'left': [1.0]*7, 'right': [1.0]*7}
    new_ff['friction']['coulomb_nm'] = f['friction']
    new_ff['friction']['load_coefficient'] = f['load_friction']
    if a.evidence:
        new_ff['evidence'] = a.evidence
    gains = {'waist': {'kp': [hold['waist_yaw']['kp']], 'kd': [hold['waist_yaw']['kd']]},
             'head': {'kp': [hold['head_pitch']['kp'], hold['head_yaw']['kp']],
                      'kd': [hold['head_pitch']['kd'], hold['head_yaw']['kd']]}}
    new_rt = dict(rt, joint_gains=gains)
    current_hold = json.loads((ROOT/'config/hold_joints.json').read_text())
    report = {
        'arm_dynamics_urdf': {'source': str(a.urdf), 'target': TARGET_URDF,
                              'same_as_current_model': (ROOT/ff['model_urdf']).exists()
                              and (ROOT/ff['model_urdf']).read_bytes() == a.urdf.read_bytes()},
        'arm_torque_ff': {'coulomb_nm': f['friction'], 'load_coefficient': f['load_friction'],
                          'friction_changed': ff['friction'].get('coulomb_nm') != f['friction']
                          or ff['friction'].get('load_coefficient') != f['load_friction'],
                          'gravity_scale_changed': ff['gravity']['scale'] != new_ff['gravity']['scale']},
        'hold_joints_changed': current_hold != hold,
        'runtime_joint_gains': gains, 'runtime_joint_gains_changed': rt.get('joint_gains') != gains,
        'written': bool(a.write)}
    if a.write:
        shutil.copyfile(a.urdf, ROOT/TARGET_URDF)
        ff_path.write_text(json.dumps(new_ff, indent=2, ensure_ascii=False)+'\n')
        (ROOT/'config/hold_joints.json').write_text(json.dumps(hold, indent=2)+'\n')
        rt_path.write_text(json.dumps(new_rt, indent=2)+'\n')
    print(json.dumps(report, indent=1, ensure_ascii=False))
    if not a.write:
        print('dry run; add --write to install. Restart r1d-runtime.service afterwards so the waist/head gains load.')


if __name__ == '__main__':
    main()
