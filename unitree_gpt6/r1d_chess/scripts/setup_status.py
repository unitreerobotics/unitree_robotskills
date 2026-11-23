#!/usr/bin/env python3
"""Which site configuration is still the reference robot's, and which has been set up here.

  python scripts/setup_status.py                         table, exit 0
  python scripts/setup_status.py --require               exit 1 while a required item is still the reference value
  python scripts/setup_status.py --confirm ITEM --note "why the reference value is valid here"
  python scripts/setup_status.py --unconfirm ITEM

An item counts as set up when its file differs from the copy in config/reference/, or when the
reference value has been explicitly confirmed for this robot (recorded in config/setup_status.json
with the file hash, so a later change of the file drops the confirmation). Offline; no robot access.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
STATUS = ROOT/'config/setup_status.json'
# item: (config file, required before playing, how to set it up)
ITEMS = {
    'runtime': ('runtime.json', True, 'edit: network_interface, camera_host (setup step 0)'),
    'camera_mapping': ('chess_camera_policy.json', True, 'setup/camera-check'),
    'arm_torque_ff': ('arm_torque_ff.json', True, 'setup/arm-calibration, then setup/joint-tracking'),
    'hold_joints': ('hold_joints.json', True, 'setup/arm-calibration (waist and head)'),
    'head_camera': ('head_camera_calibration.json', True, 'setup/head-camera-calibration'),
    'robot_profile': ('robot.json', True, 'setup/chess-scene-parameters (init pose, TCP, scene, release heights)'),
    'board_geometry': ('board_geometry.json', True, 'setup/chess-scene-parameters, later: python scripts/board_calibrate.py'),
    'initial_pose_record': ('chess_initial_pose.json', False, 'written with robot.json by capture_initial_pose.py'),
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


def status():
    confirmed = json.loads(STATUS.read_text()).get('confirmed', {}) if STATUS.exists() else {}
    rows = []
    for item, (name, required, how) in ITEMS.items():
        active, reference = digest(ROOT/'config'/name), digest(ROOT/'config/reference'/name)
        if active is None:
            state = 'missing'
        elif active != reference:
            state = 'site'
        elif confirmed.get(item, {}).get('sha256') == active:
            state = 'confirmed'
        else:
            state = 'reference'
        rows.append(dict(item=item, file='config/'+name, required=required, state=state, setup=how,
                         note=confirmed.get(item, {}).get('note') if state == 'confirmed' else None))
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--require', action='store_true')
    p.add_argument('--confirm', choices=ITEMS)
    p.add_argument('--unconfirm', choices=ITEMS)
    p.add_argument('--note', default='')
    p.add_argument('--json', action='store_true')
    a = p.parse_args()
    if a.confirm or a.unconfirm:
        data = json.loads(STATUS.read_text()) if STATUS.exists() else {'confirmed': {}}
        if a.confirm:
            if not a.note.strip():
                raise SystemExit('--confirm needs --note: say what was checked on this robot')
            data['confirmed'][a.confirm] = {'sha256': digest(ROOT/'config'/ITEMS[a.confirm][0]), 'note': a.note}
        else:
            data['confirmed'].pop(a.unconfirm, None)
        STATUS.write_text(json.dumps(data, indent=1, ensure_ascii=False)+'\n')
    rows = status()
    pending = [r['item'] for r in rows if r['required'] and r['state'] in ('reference', 'missing')]
    if a.json:
        print(json.dumps({'items': rows, 'pending': pending, 'ready': not pending}, indent=1, ensure_ascii=False))
    else:
        label = {'site': 'updated (this robot) / 已更新（本机）', 'confirmed': 'confirmed to keep the reference value / 已确认沿用参考值',
                 'reference': 'reference value, not set up / 参考值，未 setup', 'missing': 'missing / 缺失'}
        for r in rows:
            print(f"{'*' if r['required'] else ' '} {r['item']:20s} {label[r['state']]:58s} {r['file']:36s} {r['setup']}"
                  + (f"  [{r['note']}]" if r['note'] else ''))
        print('\n* required before playing / 下棋前必须完成。' + ('All done / 全部完成。' if not pending else 'Pending / 尚未完成: ' + ', '.join(pending) + '  (setup/README.md)'))
    raise SystemExit(1 if a.require and pending else 0)


if __name__ == '__main__':
    main()
