"""Initialize runtime files without robot access or invented calibration."""

from __future__ import annotations

import argparse
import json
from importlib.resources import files

from .experience import initialize, inside, workspace

DIRECTORIES = (
    "config",
    "calibration",
    "experience",
    "scratch",
    "runtime/rounds",
    "logs",
    "cache",
    "tmp",
    "verification",
)


def initialize_workspace(path):
    root = workspace(path)
    # Validate all destinations before creating anything, including existing symlinks.
    destinations = [inside(root / name, root) for name in DIRECTORIES]
    config = inside(root / "config/hardware.local.yaml", root)
    inside(root / "experience/records.json", root)
    for directory in destinations:
        directory.mkdir(parents=True, exist_ok=True)
    template = files("unitree_gpt6.g1d_gomoku").joinpath("config/hardware.example.yaml")
    try:
        with config.open("x", encoding="utf-8") as stream:
            stream.write(template.read_text(encoding="utf-8"))
    except FileExistsError:
        pass
    initialize(root)
    return {"workspace": str(root), "hardware_config": str(config), "robot_connected": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", required=True)
    args = parser.parse_args(argv)
    print(json.dumps(initialize_workspace(args.workspace), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
