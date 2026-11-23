"""Common configuration for one teleoperation HMI input."""

from robot_runtime.configs.common import require_mapping, require_non_empty_str


class HMIConfig:
    """Normalize fields shared by every ``policy.hmi`` input type.

    Args:
        hmi_config: Configuration for the single HMI input.

    Attributes:
        driver_type: Concrete HMI driver and input type.
        source_type: Input state type consumed by Teleop retargeters.

    Raises:
        ValueError: The driver type does not use ``<driver>.<source>``.

    Example:
        config = HMIConfig(
            {
                "type": "xrobotoolkit.xr",
            }
        )
        assert config.source_type == "xr"
    """

    def __init__(self, hmi_config: dict[str, object]) -> None:
        hmi_config = require_mapping(hmi_config, label="policy.hmi")
        self.driver_type = require_non_empty_str(
            hmi_config.get("type"),
            label="policy.hmi.type",
        )
        driver_name, separator, source_type = self.driver_type.rpartition(".")
        if not separator or not driver_name or not source_type:
            raise ValueError(
                "policy.hmi.type must use <driver>.<source>."
            )
        self.source_type = source_type

    def to_dict(self) -> dict[str, object]:
        """Return the normalized common HMI fields."""
        return {"type": self.driver_type}
