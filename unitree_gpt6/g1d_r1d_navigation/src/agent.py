"""Deterministic demo coordination; replace understanding with a model integration later."""

from typing import Any

from unitree_gpt6.g1d_r1d_navigation.common.contracts import Action, SkillInput
from unitree_gpt6.g1d_r1d_navigation.common.runtime import PreviewAgent

from .navigation_adapter import NavigationAdapter
from .waypoint_planner import build_plan


class NavigationAgent(PreviewAgent):
    robot_model = "R1-D"

    def __init__(self):
        super().__init__(NavigationAdapter())

    def understand(self, task: SkillInput) -> str:
        instruction = task.user_instruction.strip()
        waypoints = task.skill_config.get("waypoints")
        if not isinstance(waypoints, dict) or instruction not in waypoints:
            raise ValueError("Preview requires an exact configured waypoint name, such as home")
        return instruction

    def plan(self, intent: str, config: dict[str, Any]) -> list[Action]:
        return build_plan(intent, config)
