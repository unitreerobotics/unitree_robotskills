"""Timing of joint paths: how long each point takes and which points are passed at speed.

No devices and no model: the caller supplies the joint targets and, for a
constant-speed run, the brush-tip position at each of them.
"""

from __future__ import annotations

import numpy as np

from .onboard.controller import VELOCITY_STEP_MAX_RAD_S

DURATION_MIN_S = 0.05
STEP_MIN_M = 1e-4  # shorter steps give no tip direction to keep a speed along
STEP_MARGIN = 0.999


def rest_to_rest(start, targets, joint_speed):
    """Stop at every target; the fastest joint peaks at ``joint_speed``."""
    result, previous = [], np.asarray(start, dtype=float)
    for target in np.atleast_2d(np.asarray(targets, dtype=float)):
        step = float(np.max(np.abs(target - previous)))
        result.append((target, max(DURATION_MIN_S, 1.875 * step / joint_speed), "quintic"))
        previous = target
    return result


def constant_speed(start, targets, tips, *, tip_speed, joint_speed, ramp_m=None):
    """Pass through the targets at one brush-tip speed, stopping only where the path turns sharply.

    ``tips`` holds the tip position at the start and at every target. The speed
    is lowered until no joint exceeds ``joint_speed``. Where the joint speed
    would jump between two neighbouring steps (a corner, a reversal) the tip
    stops. A run speeds up over its first step and slows down over its last one,
    or over only ``ramp_m`` of them when that is given and the step is longer.
    Returns [(joint target, duration_s, profile)]; ramps may add targets.
    """
    points = np.vstack([np.asarray(start, dtype=float), np.asarray(targets, dtype=float)])
    tips = np.asarray(tips, dtype=float)
    if len(tips) != len(points):
        raise ValueError("one tip position is needed for the start and for every target")
    moves = np.diff(points, axis=0)
    lengths = np.linalg.norm(np.diff(tips, axis=0), axis=1)
    widest = np.max(np.abs(moves), axis=1)
    usable = (lengths >= STEP_MIN_M) & (widest > 0)
    included = set(np.flatnonzero(usable))
    while True:
        speed = min([tip_speed, *(joint_speed * lengths[i] / widest[i] for i in included)])
        runs, run, previous = [], [], None
        for index in range(len(moves)):
            rate = moves[index] * speed / lengths[index] if usable[index] else None
            if run and (
                rate is None
                or np.max(np.abs(rate - previous)) > VELOCITY_STEP_MAX_RAD_S * STEP_MARGIN
            ):
                runs.append(run)
                run = []
            if rate is not None:
                run.append(index)
                previous = rate
        if run:
            runs.append(run)
        blended = {index for run in runs if len(run) > 1 for index in run}
        if blended == included:
            break
        included = blended  # a higher speed can only add stops, so this ends
    first = {run[0] for run in runs if len(run) > 1}
    last = {run[-1] for run in runs if len(run) > 1}
    result = []
    for index, target in enumerate(points[1:]):
        length, source = lengths[index], points[index]
        if index not in blended:
            slowest = 1.875 * length / speed if usable[index] else 0.0
            duration = max(DURATION_MIN_S, 1.875 * widest[index] / joint_speed, slowest)
            result.append((target, duration, "quintic"))
        elif index not in first and index not in last:
            result.append((target, length / speed, "linear"))
        elif ramp_m is None or length < 2 * ramp_m:
            result.append((target, 2 * length / speed, "ease_in" if index in first else "ease_out"))
        elif index in first:
            result.append(
                (source + (target - source) * ramp_m / length, 2 * ramp_m / speed, "ease_in")
            )
            result.append((target, (length - ramp_m) / speed, "linear"))
        else:
            split = source + (target - source) * (1 - ramp_m / length)
            result.append((split, (length - ramp_m) / speed, "linear"))
            result.append((target, 2 * ramp_m / speed, "ease_out"))
    return result
