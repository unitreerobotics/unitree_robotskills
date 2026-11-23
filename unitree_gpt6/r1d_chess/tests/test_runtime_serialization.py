"""Exercise actual Runtime command encoding without a socket or robot session."""
from pathlib import Path
import sys
import unittest
import numpy as np
import msgspec

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from r1d_chess.robot.r1d_runtime import R1DRuntimeClient, R1D_JOINT_GROUPS, load_experiment


def _offline_runtime_config():
    """config/runtime.json with a placeholder DDS interface: these tests never open a connection."""
    import json, tempfile
    from r1d_chess.paths import ROOT as _ROOT
    config = json.loads((_ROOT/'config/runtime.json').read_text())
    config['network_interface'] = config.get('network_interface') or 'lo'
    handle = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False)
    json.dump(config, handle)
    handle.close()
    return handle.name


class RuntimeSerializationTests(unittest.TestCase):
    def test_numpy_joint_targets_encode_and_roundtrip(self):
        load_experiment(_offline_runtime_config())
        from robot_runtime.runtime.ipc.cmd import RuntimeCmd
        client = R1DRuntimeClient.__new__(R1DRuntimeClient)
        client.include_base = False
        actions = []
        client.send_action = actions.append
        sizes = dict(left_arm=7, right_arm=7, waist=1, head=2, left_ee=1, right_ee=1)
        targets = {key: tuple(np.arange(sizes[key], dtype=np.float64) * .01)
                   for key in R1D_JOINT_GROUPS}
        client.send_joint_targets(targets)
        command = RuntimeCmd(cmd_session_id='offline-test', cmd_sequence=1,
                             timestamp=0., action=actions[0])
        decoded = msgspec.msgpack.decode(msgspec.msgpack.encode(command), type=RuntimeCmd)
        self.assertEqual(len(decoded.action.joint_groups), 6)
        for group in decoded.action.joint_groups:
            self.assertEqual(group.qpos, [float(v) for v in targets[group.schema_key]])


if __name__ == '__main__':
    unittest.main()
