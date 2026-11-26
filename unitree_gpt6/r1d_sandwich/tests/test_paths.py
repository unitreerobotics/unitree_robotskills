from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from service import paths  # noqa: E402


class PathResolutionTest(unittest.TestCase):
    def test_environment_beats_dotenv_and_default(self):
        with (
            patch.dict(os.environ, {"SERVER_URL": "http://env"}, clear=True),
            patch.object(paths, "_DOTENV", {"SERVER_URL": "http://dotenv"}),
        ):
            self.assertEqual(paths.getenv("SERVER_URL"), "http://env")

    def test_dotenv_beats_default(self):
        with (
            patch.dict(os.environ, {}, clear=True),
            patch.object(paths, "_DOTENV", {"SERVER_URL": "http://dotenv"}),
        ):
            self.assertEqual(paths.getenv("SERVER_URL"), "http://dotenv")

    def test_default_is_last_and_an_empty_value_counts_as_unset(self):
        with (
            patch.dict(os.environ, {"SERVER_URL": ""}, clear=True),
            patch.object(paths, "_DOTENV", {}),
        ):
            self.assertEqual(paths.getenv("SERVER_URL"), "http://127.0.0.1:8765")

    def test_no_machine_paths_and_no_unitree_deploy_setting(self):
        source = Path(paths.__file__).read_text(encoding="utf-8")
        self.assertNotIn("/home/", source)
        self.assertNotIn("UNITREE_DEPLOY", source)

    def test_model_is_bundled_in_assets(self):
        self.assertEqual(paths.MODEL_DIR, paths.PACKAGE_ROOT / "assets" / "r1d")
        with patch.dict(os.environ, {}, clear=True), patch.object(paths, "_DOTENV", {}):
            self.assertIsNone(paths.getenv("R1D_URDF"))      # no setting needed: the bundled URDF is the default
        self.assertTrue((paths.MODEL_DIR / "r1d.urdf").is_file())

    def test_ensure_external_paths_adds_only_a_configured_sdk_checkout(self):
        with patch.object(paths, "UNITREE_SDK2_ROOT", None), patch.object(sys, "path", list(sys.path)):
            before = list(sys.path)
            paths.ensure_external_paths()
            self.assertEqual(sys.path, before)
        with (
            patch.object(paths, "UNITREE_SDK2_ROOT", Path("/somewhere/unitree_sdk2_python")),
            patch.object(sys, "path", list(sys.path)),
        ):
            paths.ensure_external_paths()
            self.assertEqual(sys.path[0], "/somewhere/unitree_sdk2_python")


if __name__ == "__main__":
    unittest.main()
