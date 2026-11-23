"""Shared constants and input validators. No hardware or model imports.

Every module validates numbers through these functions, so a rule such as
"finite, no booleans" is written once.
"""

from __future__ import annotations

import hashlib
import json
import math

import numpy as np

# R1-D LowState/LowCmd layout: 35 motor slots.
MOTOR_COUNT = 35
MODE_MACHINE = 11
LIFT_SLOT = 12
WAIST_YAW_SLOT = 13
LEFT_ARM_SLOTS = tuple(range(15, 22))
RIGHT_ARM_SLOTS = tuple(range(22, 29))
HEAD_SLOTS = (29, 30)  # pitch, yaw
GRIP_SLOTS = (31, 33)  # left, right

HANDS = ("left", "right")
# Order of the seven joints of one arm, shoulder to wrist.
ARM_JOINTS = (
    "shoulder_pitch",
    "shoulder_roll",
    "shoulder_yaw",
    "elbow",
    "wrist_roll",
    "wrist_pitch",
    "wrist_yaw",
)
# End-effector pose: meters and XYZ Euler radians in the URDF root frame.
POSE_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


def arm_slots(hand):
    """Motor slots of one arm, in ARM_JOINTS order."""
    if hand not in HANDS:
        raise ValueError("hand must be left or right")
    return LEFT_ARM_SLOTS if hand == "left" else RIGHT_ARM_SLOTS


def number(value, label, *, positive=False, minimum=None, maximum=None):
    """Return a finite float; booleans and numeric strings are rejected."""
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    if positive and value <= 0:
        raise ValueError(f"{label} must be positive")
    if minimum is not None and value < minimum:
        raise ValueError(f"{label} must be at least {minimum}")
    if maximum is not None and value > maximum:
        raise ValueError(f"{label} must be at most {maximum}")
    return float(value)


def array(value, shape, label):
    """Return a new float array of exactly ``shape`` holding finite numbers."""
    try:
        raw = np.asarray(value, dtype=object)
    except ValueError as exc:  # ragged nested lists
        raise ValueError(f"{label} must have shape {shape}") from exc
    if raw.shape != tuple(shape):
        raise ValueError(f"{label} must have shape {tuple(shape)}")
    if any(isinstance(item, (bool, np.bool_)) for item in raw.flat) or not all(
        isinstance(item, (int, float, np.integer, np.floating)) for item in raw.flat
    ):
        raise ValueError(f"{label} must contain only numbers")
    result = raw.astype(float)
    if not np.isfinite(result).all():
        raise ValueError(f"{label} must contain only finite numbers")
    return result


def vector(value, count, label):
    """Return a new float vector of ``count`` finite numbers."""
    return array(value, (count,), label)


def text(value, label):
    """Return a nonblank string unchanged."""
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label} must be a nonblank string")
    return value


def pose(value, label="pose"):
    """Return the six POSE_KEYS values of a pose object as a float vector."""
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object with {', '.join(POSE_KEYS)}")
    return vector([value.get(key) for key in POSE_KEYS], 6, label)


def digest(value):
    """SHA-256 of the canonical JSON form; rejects NaN, infinity and non-JSON data."""
    try:
        raw = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise ValueError("value must be finite JSON data") from exc
    return hashlib.sha256(raw.encode()).hexdigest()
