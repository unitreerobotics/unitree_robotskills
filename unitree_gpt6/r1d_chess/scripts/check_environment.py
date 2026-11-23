#!/usr/bin/env python3
"""Offline environment check: no cameras, no DDS, no robot commands.

  python scripts/check_environment.py      exit 0 when every required item passes
"""
import hashlib
import importlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
results = []


def check(name, ok, detail='', required=True):
    results.append((name, bool(ok), required))
    print(f"[{'ok' if ok else ('FAIL' if required else 'warn')}] {name}" + (f': {detail}' if detail else ''))


def main():
    for mod in ('numpy', 'scipy', 'cv2', 'zmq', 'pyffish', 'msgspec'):
        try:
            importlib.import_module(mod)
            check(f'python module {mod}', True)
        except Exception as exc:  # noqa: BLE001
            check(f'python module {mod}', False, str(exc)[:120])
    try:
        importlib.import_module('unitree_sdk2py')
        check('python module unitree_sdk2py', True)
    except Exception as exc:  # noqa: BLE001
        check('python module unitree_sdk2py', False, str(exc)[:120])
    for name in ('robot.json', 'runtime.json', 'board_geometry.json', 'head_camera_calibration.json',
                 'chess_camera_policy.json', 'arm_torque_ff.json', 'hold_joints.json', 'xy_error_learning.json'):
        try:
            json.loads((ROOT/'config'/name).read_text())
            check(f'config/{name}', True)
        except Exception as exc:  # noqa: BLE001
            check(f'config/{name}', False, str(exc)[:120])
    try:
        from r1d_chess.robot.r1d_config import MODEL_PATH, RUNTIME, load_profile, check_kinematic_model
        profile = load_profile()
        digest = hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest()
        check('kinematic URDF', MODEL_PATH.exists(), str(MODEL_PATH))
        check('URDF sha256 matches robot.json', digest == profile.get('kinematic_model_sha256'), digest[:16])
        check_kinematic_model(profile)
        check('profile geometry source', True, str(profile.get('geometry_source') or 'calibration_verified'))
        root = ROOT/(RUNTIME.get('runtime_root') or 'third_party/robot_runtime')
        check('bundled Runtime (third_party/robot_runtime)', (root/'src/robot_runtime/runtime/server.py').is_file(), str(root))
        check('DDS interface (config/runtime.json network_interface)', bool(RUNTIME.get('network_interface')), RUNTIME.get('network_interface') or 'not set')
        ff = json.loads((ROOT/'config/arm_torque_ff.json').read_text())
        if ff.get('model_urdf'):
            check('arm dynamics URDF', (ROOT/ff['model_urdf']).exists(), str(ROOT/ff['model_urdf']))
    except Exception as exc:  # noqa: BLE001
        check('profile / model', False, f'{type(exc).__name__}: {str(exc)[:200]}')
    try:
        from r1d_chess.chess.xiangqi import ENGINE
        found = shutil.which(str(ENGINE))
        check('Fairy-Stockfish engine (CHESS_ENGINE or fairy-stockfish on PATH)', bool(found), found or f'{ENGINE} not found')
        import pyffish
        check('pyffish supports xiangqi', 'xiangqi' in pyffish.variants())
    except Exception as exc:  # noqa: BLE001
        check('engine', False, str(exc)[:160])
    for path in ('assets/templates/round_spec.json', 'assets/piece_legend.jpg', 'r1d'):
        check(path, (ROOT/path).exists())
    setup = subprocess.run([sys.executable, '-B', str(ROOT/'scripts/setup_status.py'), '--json'], capture_output=True, text=True)
    try:
        pending = json.loads(setup.stdout)['pending']
        check('site setup complete (informational; python scripts/setup_status.py)', not pending,
              'still reference values: ' + ', '.join(pending) if pending else '', required=False)
    except Exception as exc:  # noqa: BLE001
        check('site setup status', False, str(exc)[:120], required=False)
    unit = subprocess.run(['systemctl', '--user', 'is-active', 'r1d-runtime.service'], capture_output=True, text=True)
    check('r1d-runtime.service active (informational)', unit.stdout.strip() == 'active', unit.stdout.strip(), required=False)
    failed = [n for n, ok, req in results if req and not ok]
    print(f'\n{sum(ok for _, ok, _ in results)} of {len(results)} passed' + (f', FAILED: {failed}' if failed else ', none failed'))
    raise SystemExit(1 if failed else 0)


if __name__ == '__main__':
    main()
