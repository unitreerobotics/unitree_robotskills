"""Paths and R1-D defaults; no SDK or hardware imports."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DEFAULTS = json.loads((ROOT / "config.json").read_text())
# Relative to this directory; not resolved, so the venv interpreter symlink is kept.
LOCAL_PYTHON = ROOT / DEFAULTS["local_python"]
MODEL_DIR = ROOT / "assets/r1d"
REPORT_DIR = ROOT / "reports"
RUNTIME_DIR = ROOT / "runtime"
# Saved home poses (tools/save_home.py); home.json is the one every home goes to.
POSE_DIR = ROOT / "assets/poses"
HOME_POSE = POSE_DIR / "home.json"
