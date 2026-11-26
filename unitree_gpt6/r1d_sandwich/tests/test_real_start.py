from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

HAVE_DEPS = all(importlib.util.find_spec(m) for m in ("numpy", "pinocchio", "unitree_sdk2py"))


class NoUnitreeDeployTest(unittest.TestCase):
    def test_nothing_in_the_skill_uses_unitree_deploy(self):
        """Code, launcher, settings template and requirements: no import of, path to or option for unitree-deploy."""
        files = [*(ROOT / "src").rglob("*.py"), *(ROOT / "scripts").glob("*"), ROOT / "r1d", ROOT / ".env.example",
                 ROOT / "requirements.txt"]
        for p in files:
            if p.is_file():
                text = p.read_text(encoding="utf-8").lower()
                self.assertNotIn("unitree_deploy", text, p.name)
                self.assertNotIn("unitree-deploy", text, p.name)
        self.assertFalse((ROOT / "src/robot/deploy_io.py").exists())
        self.assertNotIn("--backend", (ROOT / "src/service/server.py").read_text(encoding="utf-8"))


@unittest.skipUnless(HAVE_DEPS, "needs numpy, pinocchio and unitree_sdk2py")
class RealStartGateTest(unittest.TestCase):
    def test_only_s_starts_the_real_robot(self):
        from robot.robot import confirm_real_start

        self.assertTrue(confirm_real_start(lambda prompt: "s"))
        self.assertTrue(confirm_real_start(lambda prompt: "  S \n"))
        for answer in ("", "y", "yes", "start", "n"):
            with self.subTest(answer=answer):
                self.assertFalse(confirm_real_start(lambda prompt, a=answer: a))

    def test_closed_stdin_does_not_start(self):
        from robot.robot import confirm_real_start

        def closed(prompt):
            raise EOFError

        self.assertFalse(confirm_real_start(closed))


if __name__ == "__main__":
    unittest.main()
