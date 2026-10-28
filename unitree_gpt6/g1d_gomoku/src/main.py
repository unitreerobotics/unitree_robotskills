"""Agent-led autonomous Gomoku: observe, choose, align, confirm and return."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from .agent import GomokuAgent
from .robot_adapter import HardwareAdapter


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument(
        "command", choices=("play", "decide", "prepare-release", "release", "status")
    )
    parser.add_argument("--round", required=True, type=Path, help="Unique directory for this turn")
    parser.add_argument(
        "--config", type=Path, help="Private hardware YAML, required for live tools"
    )
    parser.add_argument(
        "--owner-root", type=Path, help="Existing healthy continuous-controller root"
    )
    parser.add_argument("--confirmation", help="Exact user approval, only for release")
    args = parser.parse_args(argv)
    try:
        agent = GomokuAgent(args.round)
        if args.confirmation is not None and args.command != "release":
            raise ValueError("--confirmation is only valid for release")
        if args.command in ("play", "prepare-release", "release"):
            if not args.config or not args.owner_root:
                raise ValueError("Live workflow requires --config and --owner-root")
            adapter = HardwareAdapter(args.config, args.owner_root)
        with agent.lock():
            if args.command == "play":
                result = agent.start(adapter)
            elif args.command == "decide":
                result = agent.decide()
            elif args.command == "prepare-release":
                result = agent.prepare_release(adapter)
            elif args.command == "release":
                result = agent.release(adapter, args.confirmation)
            else:
                result = agent.state()
        # Detailed evidence and reference paths stay in this round's files.
        output = {
            key: result[key]
            for key in (
                "task_status",
                "selected_move",
                "next_action",
                "physical_placement_verified",
                "reset_joint_verified",
            )
            if key in result
        }
        print(json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False))
        return 1 if result["task_status"].endswith(("failed", "unverified")) else 0
    except (ValueError, OSError, RuntimeError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(json.dumps({"task_status": "failed", "error": str(exc)}, ensure_ascii=False))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
