"""Driver-neutral state values for color-camera components.

Robot drivers construct these values directly or through a
``ComponentInterface``. A ``RobotChannel`` exposes them inside
``ComponentStates``,
keyed first by ``component_id`` and then by schema key.
"""

import numpy as np


class ColorCameraState:
    """One encoded or in-memory color image.

    The payload is retained without copying. ``shape`` describes spatial image
    size only and therefore excludes a channel dimension.

    Args:
        raw: Encoded image bytes or an in-memory image array.
        format: Payload encoding or array format, such as ``"jpg"``.
        shape: Image size ordered as ``(height, width)``.
        is_padded: Whether this sample reuses the last valid image.

    Attributes:
        raw: Stored image payload.
        format: Stored payload format.
        shape: Stored ``(height, width)`` image size.
        is_padded: Whether this sample is a padded replacement.

    Raises:
        ValueError: ``shape`` does not contain exactly two dimensions.

    Example:
        state = ColorCameraState(
            raw=b"encoded-jpeg",
            format="jpg",
            shape=(480, 640),
            is_padded=False,
        )
        component_states = {"head_color": state}
    """

    def __init__(
        self,
        *,
        raw: bytes | np.ndarray,
        format: str,
        shape: tuple[int, int],
        is_padded: bool = False,
    ) -> None:
        if len(shape) != 2:
            raise ValueError(
                "ColorCameraState.shape expected 2 dimensions, "
                f"got {len(shape)}."
            )

        self.raw = raw
        self.format = format
        self.shape = shape
        self.is_padded = is_padded
