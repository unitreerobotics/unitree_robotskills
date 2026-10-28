"""Verify the bundled SDK and CRC without starting DDS or accessing hardware."""

import hashlib
import importlib.util
import json
import subprocess
import sys
import unittest
from pathlib import Path


class BundledSDKTests(unittest.TestCase):
    def test_sdk_snapshot_and_crc_libraries_are_present(self):
        spec = importlib.util.find_spec("unitree_gpt6.g1d_gomoku.src.unitree_sdk2py")
        self.assertIsNotNone(spec)
        root = Path(spec.origin).parent
        manifest = json.loads((root / "manifest.json").read_text())
        self.assertEqual(manifest["license"], "BSD-3-Clause")
        for name, expected in manifest["files"].items():
            with self.subTest(file=name):
                self.assertEqual(hashlib.sha256((root / name).read_bytes()).hexdigest(), expected)
        for name in ("crc_amd64.so", "crc_aarch64.so"):
            self.assertTrue((root / "utils" / "lib" / name).is_file())
        self.assertIn("BSD 3-Clause", (root / "LICENSE").read_text())

    def test_dependency_check_locates_skill_sdk_without_importing_device_modules(self):
        code = """
import sys
from unitree_gpt6.g1d_gomoku.src.hardware.main import module_available
assert module_available('unitree_sdk2py')
assert 'unitree_gpt6.g1d_gomoku.src.unitree_sdk2py' not in sys.modules
assert 'unitree_sdk2py' not in sys.modules
assert 'cyclonedds' not in sys.modules
"""
        subprocess.run([sys.executable, "-B", "-c", code], check=True)

    @unittest.skipUnless(importlib.util.find_spec("cyclonedds"), "hardware extra not installed")
    def test_driver_imports_and_crc_work_without_dds_initialization(self):
        code = """
import importlib.abc
import sys
from unittest.mock import patch
class BlockExternalSDK(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'unitree_sdk2py' or fullname.startswith('unitree_sdk2py.'):
            raise AssertionError('External SDK import forbidden')
sys.meta_path.insert(0, BlockExternalSDK())
with patch('cyclonedds.domain.DomainParticipant', side_effect=AssertionError('DDS forbidden')):
    from unitree_gpt6.g1d_gomoku.src.unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize, ChannelPublisher,
    )
    from unitree_gpt6.g1d_gomoku.src.unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_gpt6.g1d_gomoku.src.unitree_sdk2py.idl.unitree_go.msg.dds_ import (
        MotorCmds_, MotorStates_,
    )
    from unitree_gpt6.g1d_gomoku.src.unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
        LowCmd_, LowState_,
    )
    from unitree_gpt6.g1d_gomoku.src.unitree_sdk2py.utils.crc import CRC
    crc = CRC()
    words = [0, 1, 0xffffffff, 0x12345678]
    assert crc._crc_py(words) == 0xD6A764A7
    command = unitree_hg_msg_dds__LowCmd_()
    assert crc.Crc(command) == 0xFE172F9F
    from unitree_gpt6.g1d_gomoku.src.unitree_sdk2py.idl.default import (
        unitree_go_msg_dds__MotorCmd_,
    )
    commands = MotorCmds_([unitree_go_msg_dds__MotorCmd_()])
    assert len(MotorCmds_.deserialize(commands.serialize()).cmds) == 1
    assert LowCmd_.deserialize(command.serialize()).__idl_typename__ == command.__idl_typename__
    assert command.__idl_typename__ == 'unitree_hg.msg.dds_.LowCmd_'
    command.motor_cmd[15].q = 0.1
    assert crc.Crc(command) != 0xFE172F9F
    if hasattr(crc, 'crc_lib'):
        assert crc._crc_ctypes(words) == crc._crc_py(words)
"""
        subprocess.run([sys.executable, "-B", "-c", code], check=True)
