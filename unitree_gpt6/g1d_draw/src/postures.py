"""Default arm poses and gripper positions for new fixed-base G1-D tasks.

Customize entry_q14 and park_q14, then recompile to change task postures.
Gripper positions are configured separately from arm angles and waist targets.
"""

# Default hold targets in Dex1 native units, left then right.
DEFAULT_GRIP_Q2 = (0.0, 0.0)


# Radians, left7 then right7; the common G1 arm order is used for each side.
DEFAULT_WORK_Q14 = (
    0.12276879996061325,
    0.8761894851922989,
    -0.5731587529182434,
    0.8019999444484711,
    -1.2821353495121002,
    -0.8873959183692932,
    -1.611615204811096,
    0.23186180293560027,
    -0.7953037351369858,
    -0.248862624168396,
    -0.048300623707473275,
    0.838913744688034,
    0.07691475637257099,
    1.103063428401947,
)


DEFAULT_REST_Q14 = (
    0.23448634445667266,
    0.005416270438581705,
    -0.09395093508064747,
    0.18951674401760102,
    0.04285139609128237,
    -0.37840190529823303,
    0.030670628882944584,
    0.1540943644940853,
    -0.11473996937274933,
    -0.1819397121667862,
    0.1391045033931732,
    0.9907161206007004,
    0.17223309427499772,
    0.6847265154123306,
)


def default_work_q14():
    """Return a fresh editable list for a new task's work posture."""
    return list(DEFAULT_WORK_Q14)


def default_rest_q14():
    """Return a fresh editable list for a new task's parking route."""
    return list(DEFAULT_REST_Q14)


def default_grip_q2():
    """Return a fresh editable list of default gripper hold targets."""
    return list(DEFAULT_GRIP_Q2)
