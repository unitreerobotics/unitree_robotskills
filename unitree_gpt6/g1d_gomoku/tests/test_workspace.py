"""Runtime initialization must preserve private data and keep source read-only."""

import json
import tempfile
import unittest
from pathlib import Path

from unitree_gpt6.g1d_gomoku.src.workspace import initialize_workspace


class WorkspaceTests(unittest.TestCase):
    def test_empty_workspace_and_preservation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "game"
            result = initialize_workspace(root)
            self.assertFalse(result["robot_connected"])
            self.assertEqual(
                json.loads((root / "experience/records.json").read_text())["records"], []
            )
            self.assertEqual(list((root / "calibration").iterdir()), [])
            config = Path(result["hardware_config"])
            config.write_text("private configuration\n")
            initialize_workspace(root)
            self.assertEqual(config.read_text(), "private configuration\n")

    def test_reject_source(self):
        package = Path(__file__).resolve().parents[1]
        with self.assertRaises(ValueError):
            initialize_workspace(package / "runtime")

    def test_reject_escaping_symlinks_before_writing(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "game"
            outside = Path(temporary) / "outside"
            root.mkdir()
            outside.mkdir()
            (root / "config").symlink_to(outside, target_is_directory=True)
            with self.assertRaises(ValueError):
                initialize_workspace(root)
            self.assertEqual(list(outside.iterdir()), [])
            self.assertFalse((root / "experience").exists())
