"""Common configuration values shared by all policy types."""

from robot_runtime.configs.common import (
    require_non_empty_str,
    require_positive_float,
)


class PolicyConfig:
    """Base configuration consumed by the policy process.

    Subclasses validate their fixed ``policy_id`` and add only their own fields.

    Args:
        policy_config: Resolved policy mapping containing ``policy_id`` and
            ``update_frequency``.

    Attributes:
        policy_id: Validated policy type identifier.
        update_frequency: Policy evaluation frequency in Hz.

    Raises:
        KeyError: A required common field is absent.
        ValueError: The policy ID or update frequency is invalid.
    """

    def __init__(self, policy_config: dict[str, object]) -> None:
        self.policy_id = require_non_empty_str(
            policy_config["policy_id"],
            label="policy.policy_id",
        )
        self.update_frequency = require_positive_float(
            policy_config["update_frequency"],
            label="policy.update_frequency",
        )
