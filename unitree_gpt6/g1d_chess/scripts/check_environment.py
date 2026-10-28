#!/usr/bin/env python3
"""Offline dependency/model/engine check; never initializes DDS or opens cameras."""
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))


def check():
    errors = []
    modules = {}
    versions = {}
    names = {
        'numpy': 'numpy', 'scipy': 'scipy', 'cv2': 'opencv-python',
        'yaml': 'PyYAML', 'zmq': 'pyzmq', 'pinocchio': 'pin',
        'cyclonedds': 'cyclonedds', 'pyffish': 'pyffish',
        'unitree_sdk2py': 'unitree_sdk2py',
        'unitree_sdk2py.core.channel': 'unitree_sdk2py',
        'unitree_sdk2py.idl.unitree_hg.msg.dds_': 'unitree_sdk2py',
        'unitree_sdk2py.idl.default': 'unitree_sdk2py',
        'unitree_sdk2py.utils.crc': 'unitree_sdk2py',
        'g1d_chess.robot.motion_harness': None,
    }
    for name, distribution in names.items():
        try:
            module = importlib.import_module(name)
            modules[name] = getattr(module, '__file__', None)
            if distribution:
                versions[distribution] = importlib.metadata.version(distribution)
        except Exception as exc:
            errors.append(name + ': ' + str(exc))
    model_ok = False
    mesh_count = 0
    urdf = ROOT / 'assets/models/g1/g1_body29_hand14.urdf'
    try:
        for node in ET.parse(urdf).iter('mesh'):
            name = node.attrib['filename']
            mesh_count += 1
            if name.startswith('package://'):
                name = name[len('package://'):]
            if not (urdf.parent / name).is_file():
                errors.append('missing mesh: ' + name)
        from g1d_chess.robot.motion_harness import MotionHarness
        import numpy as np
        harness = MotionHarness()
        expected = [f'{side}_{joint}_joint' for side in ('left', 'right')
                    for joint in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw',
                                  'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')]
        if list(harness.model.names)[1:] != expected or harness.model.nq != 14:
            raise ValueError('expected physical left 7 + right 7 arm joint order')
        values = harness.fk(np.zeros(14))
        torques = harness.tau(np.zeros(14))
        model_ok = all(np.isfinite(v).all() for v in [*values, torques])
        if not model_ok:
            errors.append('model FK/dynamics returned non-finite values')
    except Exception as exc:
        errors.append('model: ' + str(exc))
    engine_setting = os.environ.get('CHESS_ENGINE') or 'fairy-stockfish'
    engine = shutil.which(engine_setting)
    engine_ok = False
    if not engine:
        errors.append('engine not executable/found: ' + engine_setting + '; set CHESS_ENGINE')
    else:
        try:
            result = subprocess.run([engine], input='uci\nisready\nquit\n',
                                    text=True, capture_output=True, timeout=15)
            engine_ok = (result.returncode == 0 and 'uciok' in result.stdout
                         and 'readyok' in result.stdout and 'xiangqi' in result.stdout)
            if not engine_ok:
                errors.append('engine must support UCI and the xiangqi variant')
        except Exception as exc:
            errors.append('engine: ' + str(exc))
    legal_count = None
    try:
        import pyffish
        fen = 'rnbakabnr/9/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/9/RNBAKABNR w - - 0 1'
        legal_count = len(pyffish.legal_moves('xiangqi', fen, []))
        if legal_count != 44:
            errors.append('xiangqi starting position expected 44 moves')
    except Exception as exc:
        errors.append('pyffish: ' + str(exc))
    return {'status': 'failed' if errors else 'passed', 'root': str(ROOT),
            'python': sys.version, 'modules': modules, 'versions': versions,
            'urdf': str(urdf), 'mesh_references': mesh_count, 'model_fk_dynamics_ok': model_ok,
            'unitree_deploy_imported': any(n == 'unitree_deploy' or n.startswith('unitree_deploy.')
                                           for n in sys.modules),
            'engine': engine, 'engine_uci_ok': engine_ok,
            'xiangqi_initial_legal_moves': legal_count, 'robot_calls': False, 'errors': errors}


if __name__ == '__main__':
    result = check()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(bool(result['errors']))
