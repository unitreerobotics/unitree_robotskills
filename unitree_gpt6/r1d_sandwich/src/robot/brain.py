# Copied from r1-ik deploy/brain.py (90285d2), trimmed to READY and ready16(); only imports and paths changed. Keep in sync by copying; do not restyle.
"""The standard start pose. Trimmed from r1-ik deploy/brain.py: only READY and ready16()."""

import numpy as np

from ik.kin import nominal_joints
from robot import ik_executor as X

# r1ik's nominal working posture: upper arms slightly forward and out, forearms forward, wrists neutral, waist 0
READY = nominal_joints()


def ready16(lift=0.40):
    """The standard start pose in ik_executor's 16-joint order."""
    q = np.zeros(16)
    q[X.I_LIFT] = lift
    for i, n in enumerate(X.IK_ACTIVE_NAMES):
        if n in READY:
            q[i] = READY[n]
    return q
