"""Check CLI JSON and exit codes from a caller's perspective."""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


class CLITests(unittest.TestCase):
    def run_cli(self, *arguments):
        process = subprocess.run(
            [
                sys.executable,
                "-m",
                "unitree_gpt6.g1d_r1d_navigation.src.main",
                "--instruction",
                "home",
                *arguments,
            ],
            capture_output=True,
            text=True,
            check=False,
            timeout=10,
        )
        self.assertEqual(process.stderr, "")
        return process.returncode, json.loads(process.stdout)

    def test_successful_preview(self):
        code, result = self.run_cli()
        self.assertEqual(code, 0)
        self.assertEqual(result["task_status"], "previewed")
        self.assertEqual(result["execution_feedback"]["hardware_commands_sent"], 0)

    def test_hardware_rejection_is_structured(self):
        code, result = self.run_cli("--mode", "hardware")
        self.assertEqual(code, 1)
        self.assertEqual(result["task_status"], "failed")
        self.assertEqual(result["action_plan"], [])

    def test_malformed_yaml_and_missing_file_are_structured_errors(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "config.yaml"
            config.write_text("limits: [", encoding="utf-8")
            code, result = self.run_cli("--config", str(config))
            self.assertEqual(code, 1)
            self.assertEqual(result["task_status"], "failed")
            code, result = self.run_cli("--config", str(Path(directory) / "missing.yaml"))
            self.assertEqual(code, 1)
            self.assertEqual(result["error_info"]["type"], "FileNotFoundError")
