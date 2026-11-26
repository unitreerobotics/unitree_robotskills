from __future__ import annotations

import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

SKILL_ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(SKILL_ROOT / "src"), str(SKILL_ROOT / "scripts")]

import check_environment  # noqa: E402


class EnvironmentCheckTest(unittest.TestCase):
    def test_missing_model_returns_nonzero_json(self):
        with tempfile.TemporaryDirectory() as tmp:
            missing = Path(tmp) / "missing"
            stdout = io.StringIO()
            with (
                patch.object(check_environment.paths, "MODEL_DIR", missing),
                patch.object(check_environment.paths, "R1D_URDF", missing / "r1d.urdf"),
                patch.object(check_environment.paths, "CONFIG_DIR", Path(tmp) / "config"),
                contextlib.redirect_stdout(stdout),
            ):
                code = check_environment.main(["--json"])

        report = json.loads(stdout.getvalue())
        self.assertEqual(code, 1)
        self.assertFalse(report["ok"])
        self.assertTrue(any(row["ok"] is False for row in report["checks"]))

    def test_checks_cover_the_code_the_model_and_the_ik_packages(self):
        paths = check_environment.paths
        rows = check_environment.collect_checks(
            src_dir=paths.PACKAGE_ROOT / "src", model_dir=paths.MODEL_DIR, urdf=paths.R1D_URDF,
            config_dir=paths.CONFIG_DIR, sdk2_root=None, python="/nonexistent/python",
        )
        by_name = {row["check"]: row for row in rows}
        self.assertIs(by_name["IK code and R1-D model"]["ok"], True)
        self.assertIs(by_name["R1-D URDF"]["ok"], True)
        self.assertIn("import robokit, warp", by_name)          # the planner's IK solver needs both
        self.assertIn("import unitree_sdk2py", by_name)
        self.assertFalse(any("deploy" in name for name in by_name))

    def test_code_and_model_are_complete_and_have_no_machine_paths(self):
        root = check_environment.paths.PACKAGE_ROOT
        for rel in ("src/ik/kin.py", "src/ik/quality.py", "src/ik/solvers.py", "src/ik/models.py",
                    "src/robot/ik_executor.py", "src/robot/dds_io.py", "src/robot/brain.py", "src/robot/model.py",
                    "src/robot/protocol.py", "src/robot/dynamics.py", "assets/r1d/r1d.urdf",
                    "assets/r1d/r1d_calibrated.urdf", "assets/r1d/a7d_ik.urdf", "assets/r1d/a7d_cal_ik.urdf",
                    "assets/r1d/SHA256SUMS"):
            self.assertTrue((root / rel).is_file(), rel)
        self.assertEqual(len(list((root / "assets/r1d/meshes").glob("*.STL"))), 36)
        self.assertFalse((root / "third_party").exists())
        for source in (root / "src").rglob("*.py"):
            self.assertNotIn("/home/", source.read_text(encoding="utf-8"), source.name)


if __name__ == "__main__":
    unittest.main()
