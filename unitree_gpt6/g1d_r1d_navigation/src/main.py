"""Run with python -m unitree_gpt6.g1d_r1d_navigation.src.main."""

from unitree_gpt6.g1d_r1d_navigation.common.cli import run_cli

from .agent import NavigationAgent

if __name__ == "__main__":
    raise SystemExit(run_cli("g1d_r1d_navigation", NavigationAgent()))
