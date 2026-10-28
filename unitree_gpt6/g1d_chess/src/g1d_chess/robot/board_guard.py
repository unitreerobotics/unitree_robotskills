"""Fail-closed swept gripper clearance checks, independent of the IK solver.

Geometry is an explicitly supplied visual estimate, not a real calibration.
All limits apply to the entire oriented gripper box, not just the wrist frame.
"""
from dataclasses import dataclass
from itertools import product

import numpy as np
from scipy.spatial.transform import Rotation

LOW_ROTATION_DRIFT_TOLERANCE_RAD = .1


class GuardViolation(ValueError):
    pass


@dataclass(frozen=True)
class BoardGuard:
    surface_z: float
    uncertainty: float
    finger_end: float
    half_width: float
    half_thickness: float
    travel_clearance: float = .040
    minimum_clearance: float = .003
    xy_tolerance: float = .001
    rotation_tolerance: float = .008

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            raise GuardViolation("a scene envelope is required before any motion")
        obj = cls(**data)
        vals = np.array(list(vars(obj).values()), dtype=float)
        if not np.isfinite(vals).all() or obj.uncertainty < 0:
            raise GuardViolation("invalid scene geometry")
        if not .1 <= obj.finger_end <= .3 or not .02 <= obj.half_width <= .09:
            raise GuardViolation("invalid Dex1 dimensions")
        if obj.half_thickness <= 0 or obj.travel_clearance < .03 or obj.minimum_clearance < .002:
            raise GuardViolation("invalid minimum clearances")
        return obj

    def corners(self, pose):
        local = np.array(list(product((-.04, self.finger_end),
                                     (-self.half_width, self.half_width),
                                     (-self.half_thickness, self.half_thickness))))
        return local @ pose[:3,:3].T + pose[:3,3]

    def clearance(self, pose):
        return float(np.min(self.corners(pose)[:,2]) - self.surface_z - self.uncertainty)

    def check_segment(self, start, end):
        shift = end[:3,3] - start[:3,3]
        angle = Rotation.from_matrix(start[:3,:3].T @ end[:3,:3]).magnitude()
        clearance = min(self.clearance(start), self.clearance(end))
        same_xy = np.linalg.norm(shift[:2]) <= self.xy_tolerance
        same_rotation = angle <= self.rotation_tolerance
        stationary = same_xy and abs(shift[2]) <= .0005 and same_rotation
        # A low initial pose may only hold or retreat straight upwards.
        if stationary:
            return
        if same_xy and same_rotation and shift[2] >= 0:
            return
        if clearance < self.minimum_clearance:
            raise GuardViolation(f"gripper may touch surface: clearance {clearance:.4f} m")
        if clearance < self.travel_clearance and (not same_xy or not same_rotation):
            raise GuardViolation("below travel height: lateral motion and rotation prohibited")

    def check_feedback(self, start, desired, measured, max_error=.006):
        # Only the collision constraint remains: prevent the gripper envelope
        # from touching the surface. Tracking/drift checks are removed.
        retreat = (desired[2,3] >= start[2,3] and
                   np.linalg.norm(desired[:2,3]-start[:2,3]) <= self.xy_tolerance)
        if self.clearance(measured) < self.minimum_clearance and not retreat:
            raise GuardViolation("measured gripper may touch surface")

    def check_gripper(self, pose):
        if self.clearance(pose) < self.minimum_clearance:
            raise GuardViolation("gripper open/close denied: surface clearance not verified")
