"""Shared module CLI; print a single machine-readable result and return an exit code."""

import argparse
import json
from pathlib import Path

import yaml

from .config import load_config, validate_config
from .contracts import SkillInput, SkillOutput
from .runtime import PreviewAgent


def run_cli(skill: str, agent: PreviewAgent) -> int:
    parser = argparse.ArgumentParser(description=f"{skill}: local preview only")
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--config", type=Path, help="YAML configuration path")
    parser.add_argument("--mode", choices=("preview", "hardware"), default=None)
    args = parser.parse_args()
    try:
        config = load_config(skill, agent.robot_model, args.config)
        if args.mode is not None:
            config["mode"] = args.mode
        validate_config(config, agent.robot_model)
        result = agent.run(SkillInput(args.instruction, skill_config=config))
    except (OSError, ValueError, yaml.YAMLError) as exc:
        result = SkillOutput("failed", error_info={"type": type(exc).__name__, "message": str(exc)})
    print(json.dumps(result.to_dict(), ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result.task_status == "previewed" else 1
