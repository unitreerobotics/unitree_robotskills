"""Validate recording output and task metadata configuration.

``RecordingConfig`` is the optional configuration boundary consumed by the
standalone recorder and recording Schema builder.
"""

from robot_runtime.configs.common import (
    reject_unknown_fields,
    require_choice,
    require_mapping,
    require_non_empty_str,
    require_positive_int,
)


class RecordingConfig:
    """Normalize recording output, sampling, and task metadata values.

    Experiments that do not record omit the ``recording`` section;
    ``ExperimentConfig`` represents that case with ``None`` instead of
    constructing an empty ``RecordingConfig``.

    Args:
        recording_config: Recording section from a resolved experiment. It
            must define a positive frame rate, a non-empty output directory,
            dataset name, and task instruction.

    Attributes:
        format: Output format: ``robot_runtime`` (default) or ``mcap``.
        fps: Recording frame rate in frames per second.
        output_dir: Directory where recorded task data is stored.
        dataset_name: Dataset name used for the output directory.
        task: Natural-language task stored with each frame.

    Raises:
        KeyError: A required field is missing or an unsupported field is
            present.
        TypeError: The recording value is not a mapping or ``fps`` is not an
            integer.
        ValueError: A required string is invalid or ``fps`` is not positive.

    Example:
        recording = RecordingConfig(
            {
                "fps": 30,
                "output_dir": "./data",
                "dataset_name": "pick_box",
                "task": "Pick up the box.",
            }
        )
        assert recording.fps == 30
        assert recording.dataset_name == "pick_box"
    """

    def __init__(
        self,
        recording_config: dict[str, object],
    ) -> None:
        recording_config = require_mapping(
            recording_config,
            label="recording",
        )
        reject_unknown_fields(
            recording_config,
            frozenset({
                "format",
                "dataset_name",
                "fps",
                "output_dir",
                "task",
            }),
            label="recording",
        )

        self.fps = require_positive_int(
            recording_config["fps"],
            label="recording.fps",
        )
        self.output_dir = require_non_empty_str(
            recording_config["output_dir"],
            label="recording.output_dir",
        )
        self.dataset_name = require_non_empty_str(
            recording_config["dataset_name"],
            label="recording.dataset_name",
        )
        self.task = require_non_empty_str(
            recording_config["task"],
            label="recording.task",
        )
        self.format = require_choice(
            recording_config.get("format", "robot_runtime"),
            choices=frozenset({"robot_runtime", "mcap"}),
            label="recording.format",
        )

    def to_dict(self) -> dict[str, object]:
        """Return a new mapping containing normalized recording values.

        Returns:
            Recording configuration suitable for serialization or another
            ``RecordingConfig`` construction.
        """
        return {
            "format": self.format,
            "fps": self.fps,
            "output_dir": self.output_dir,
            "dataset_name": self.dataset_name,
            "task": self.task,
        }
