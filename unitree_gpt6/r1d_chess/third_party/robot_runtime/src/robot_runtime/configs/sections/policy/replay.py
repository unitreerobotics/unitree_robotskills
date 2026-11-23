"""Validate replay policy configuration."""

from robot_runtime.configs.sections.policy.config import PolicyConfig
from robot_runtime.configs.common import (
    reject_unknown_fields,
    require_finite_float,
    require_mapping,
    require_non_empty_str,
    require_positive_float,
)


def require_duration(value: object, *, label: str) -> float:
    """Validate a non-negative duration in seconds; zero disables the phase."""
    duration = require_finite_float(value, label=label)
    if duration < 0.0:
        raise ValueError(f"{label} must be non-negative.")
    return duration


class ReplayConfig(PolicyConfig):
    """Normalize one resolved ``policy`` section for episode replay.

    Args:
        policy_config: Replay policy mapping containing update frequency,
            episode paths, and optional smooth transition durations.

    Attributes:
        episode_path: Directory containing the replay episode data.
        schema_path: Optional explicit recording schema path.
        approach_duration: Seconds spent ramping from the measured pose to the
            first recorded action; zero replays the first action directly.
        return_duration: Seconds spent ramping from the last recorded action
            back to the robot reset pose; zero ends replay on the last action.
        hold_reset_pose: Whether Policy keeps commanding the reset pose after
            the return ramp until the operator ends the session.
        ramp_velocity_limit: Optional peak joint velocity for both ramps in
            rad/s; defaults to the robot spec's reset pose limit.
        loop_count: How many times the episode is replayed; ``0`` repeats it
            until the operator ends the session.

    Example:
        config = ReplayConfig(
            {
                "policy_id": "replay",
                "update_frequency": 30.0,
                "episode_path": "./data/demo/episode_000000",
            }
        )
        assert config.episode_path.endswith("episode_000000")
        assert config.approach_duration == 0.0
    """

    def __init__(self, policy_config: dict[str, object]) -> None:
        policy_config = require_mapping(policy_config, label="policy")
        reject_unknown_fields(
            policy_config,
            frozenset(
                {
                    "policy_id",
                    "update_frequency",
                    "episode_path",
                    "schema_path",
                    "approach_duration",
                    "return_duration",
                    "hold_reset_pose",
                    "ramp_velocity_limit",
                    "loop_count",
                }
            ),
            label="policy",
        )
        super().__init__(policy_config)
        if self.policy_id != "replay":
            raise ValueError(
                "policy.policy_id must be 'replay' for ReplayConfig."
            )

        self.episode_path = require_non_empty_str(
            policy_config["episode_path"],
            label="policy.episode_path",
        )
        schema_path = policy_config.get("schema_path")
        if schema_path is not None:
            schema_path = require_non_empty_str(
                schema_path,
                label="policy.schema_path",
            )
        self.schema_path = schema_path

        self.approach_duration = require_duration(
            policy_config.get("approach_duration", 0.0),
            label="policy.approach_duration",
        )
        self.return_duration = require_duration(
            policy_config.get("return_duration", 0.0),
            label="policy.return_duration",
        )
        hold_reset_pose = policy_config.get("hold_reset_pose", False)
        if type(hold_reset_pose) is not bool:
            raise TypeError("policy.hold_reset_pose must be a boolean.")
        if hold_reset_pose and self.return_duration == 0.0:
            raise ValueError(
                "policy.hold_reset_pose requires a positive "
                "policy.return_duration."
            )
        self.hold_reset_pose = hold_reset_pose

        ramp_velocity_limit = policy_config.get("ramp_velocity_limit")
        if ramp_velocity_limit is not None:
            ramp_velocity_limit = require_positive_float(
                ramp_velocity_limit,
                label="policy.ramp_velocity_limit",
            )
        self.ramp_velocity_limit = ramp_velocity_limit

        loop_count = policy_config.get("loop_count", 1)
        if type(loop_count) is not int:
            raise TypeError("policy.loop_count must be an integer.")
        if loop_count < 0:
            raise ValueError("policy.loop_count must be non-negative.")
        self.loop_count = loop_count
