"""Serializable input/output contracts for the initial skill lifecycle."""

from dataclasses import asdict, dataclass, field
from typing import Any, Literal


@dataclass
class SkillInput:
    user_instruction: str
    robot_state: dict[str, Any] = field(default_factory=dict)
    environment_state: dict[str, Any] = field(default_factory=dict)
    skill_config: dict[str, Any] = field(default_factory=dict)


@dataclass
class Action:
    name: str
    parameters: dict[str, Any]


@dataclass
class SkillOutput:
    task_status: Literal["previewed", "failed"]
    action_plan: list[Action] = field(default_factory=list)
    execution_feedback: dict[str, Any] = field(default_factory=dict)
    error_info: dict[str, str] | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
