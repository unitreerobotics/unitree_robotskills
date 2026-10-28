"""Initialize → Understand → Plan → Execute → Observe → Verify (preview only)."""

from abc import ABC, abstractmethod
from time import monotonic
from typing import Any

from .config import validate_config
from .contracts import Action, SkillInput, SkillOutput


class PreviewAdapter:
    """Record symbolic actions without importing an SDK or connecting to hardware."""

    def execute(self, actions: list[Action]) -> dict[str, Any]:
        return {"mode": "preview", "hardware_commands_sent": 0, "actions_previewed": len(actions)}


class PreviewAgent(ABC):
    robot_model: str

    def __init__(self, adapter: PreviewAdapter):
        self.adapter = adapter

    @abstractmethod
    def understand(self, task: SkillInput) -> str:
        """Recognize only documented deterministic demo instructions."""

    @abstractmethod
    def plan(self, intent: str, config: dict[str, Any]) -> list[Action]:
        """Build symbolic actions; hardware-specific validation belongs to future adapters."""

    def run(self, task: SkillInput) -> SkillOutput:
        try:
            started = monotonic()
            validate_config(task.skill_config, self.robot_model)
            if not isinstance(task.user_instruction, str) or not task.user_instruction.strip():
                raise ValueError("user_instruction must be a non-empty string")
            intent = self.understand(task)
            actions = self.plan(intent, task.skill_config)
            if not actions or len(actions) > task.skill_config["limits"]["max_actions"]:
                raise ValueError("Plan is empty or exceeds limits.max_actions")
            if monotonic() - started > task.skill_config["limits"]["task_timeout_s"]:
                raise ValueError("Preview planning exceeded limits.task_timeout_s")
            feedback = self.adapter.execute(actions)
            # No sensor observation or physical task verification exists in this skeleton.
            feedback.update({"observation_available": False, "task_verified": False})
            return SkillOutput("previewed", actions, feedback)
        except (ValueError, NotImplementedError) as exc:
            return SkillOutput(
                "failed", error_info={"type": type(exc).__name__, "message": str(exc)}
            )
