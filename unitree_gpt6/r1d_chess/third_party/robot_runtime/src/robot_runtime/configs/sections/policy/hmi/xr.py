"""Validate XR HMI source configurations."""

from robot_runtime.configs.common import (
    reject_unknown_fields,
    require_choice,
    require_mapping,
    require_non_empty_str,
)
from robot_runtime.configs.sections.policy.hmi.config import HMIConfig

XR_INPUT_MODES = frozenset({"controller", "hand"})
TELEVUER_DISPLAY_MODES = frozenset({"immersive", "ego", "pass-through"})
TELEVUER_IMAGE_TRANSPORTS = frozenset({"auto", "webrtc", "zmq"})


class XRConfig(HMIConfig):
    """Normalize one resolved XR ``policy.hmi`` configuration.

    Args:
        source_config: XR driver type and input/display options.

    Attributes:
        driver_type: Concrete XR HMI driver type.
        input_mode: XR controller or hand-tracking input mode.
        display_mode: Televuer display mode, otherwise ``None``.
        display_camera: Televuer camera stream key, otherwise ``None``.
        image_transport: Televuer image transport, otherwise ``None``.

    Example:
        config = XRConfig(
            {
                "type": "xrobotoolkit.xr",
                "input_mode": "controller",
            }
        )
        assert config.input_mode == "controller"
    """

    def __init__(self, source_config: dict[str, object]) -> None:
        source_config = require_mapping(
            source_config,
            label="policy.hmi",
        )
        super().__init__(source_config)
        if self.driver_type not in {"televuer.xr", "xrobotoolkit.xr"}:
            raise KeyError(
                f"Unsupported XR HMI driver {self.driver_type!r}."
            )
        self.input_mode = require_choice(
            source_config.get("input_mode"),
            choices=XR_INPUT_MODES,
            label="policy.hmi.input_mode",
        )

        if self.driver_type == "xrobotoolkit.xr":
            reject_unknown_fields(
                source_config,
                frozenset({"type", "input_mode"}),
                label="policy.hmi",
            )
            self.display_mode = None
            self.display_camera = None
            self.image_transport = None
            return

        reject_unknown_fields(
            source_config,
            frozenset(
                {
                    "type",
                    "input_mode",
                    "display_mode",
                    "display_camera",
                    "image_transport",
                }
            ),
            label="policy.hmi",
        )
        self.display_mode = require_choice(
            source_config.get("display_mode"),
            choices=TELEVUER_DISPLAY_MODES,
            label="policy.hmi.display_mode",
        )
        self.display_camera = require_non_empty_str(
            source_config.get("display_camera", "head_camera"),
            label="policy.hmi.display_camera",
        )
        self.image_transport = require_choice(
            source_config.get("image_transport", "auto"),
            choices=TELEVUER_IMAGE_TRANSPORTS,
            label="policy.hmi.image_transport",
        )

    def to_dict(self) -> dict[str, object]:
        """Return a new mapping containing normalized XR values."""
        source_config = super().to_dict()
        source_config["input_mode"] = self.input_mode
        if self.display_mode is not None:
            source_config.update(
                {
                    "display_mode": self.display_mode,
                    "display_camera": self.display_camera,
                    "image_transport": self.image_transport,
                }
            )
        return source_config
