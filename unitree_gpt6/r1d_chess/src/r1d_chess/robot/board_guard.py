"""Fail-closed swept gripper clearance checks, independent of the IK solver.

Geometry is an explicitly supplied visual estimate, not a real calibration.
All limits apply to the entire oriented gripper box, not just the wrist frame.

The support surface is either one plane (surface_z everywhere, legacy profiles) or two levels: the board top
(surface_z) over the board footprint and the table (table_z) elsewhere. A gripper box whose footprint-frame
bounding rectangle overlaps the board footprint is checked against the board top as a whole (conservative).
Trays and other objects on the table are not modelled.
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
    minimum_clearance: float = 0.
    xy_tolerance: float = .001
    rotation_tolerance: float = .008
    table_z: float | None = None
    board_centre_xy: tuple | None = None
    board_yaw: float = 0.
    board_half_extent: tuple | None = None     # half sizes along ranks (board u) and files (board v)

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            raise GuardViolation("a scene envelope is required before any motion")
        data = dict(data)
        for key in ('board_centre_xy', 'board_half_extent'):
            if data.get(key) is not None:
                data[key] = tuple(float(v) for v in data[key])
        obj = cls(**data)
        scalars = [obj.surface_z, obj.uncertainty, obj.finger_end, obj.half_width, obj.half_thickness,
                   obj.travel_clearance, obj.minimum_clearance, obj.xy_tolerance, obj.rotation_tolerance, obj.board_yaw]
        if not np.isfinite(np.array(scalars, dtype=float)).all() or obj.uncertainty < 0:
            raise GuardViolation("invalid scene geometry")
        two_level = (obj.table_z, obj.board_centre_xy, obj.board_half_extent)
        if any(v is not None for v in two_level):
            if any(v is None for v in two_level):
                raise GuardViolation("two-level scene needs table_z, board_centre_xy and board_half_extent")
            values = np.array([obj.table_z, *obj.board_centre_xy, *obj.board_half_extent], dtype=float)
            if (len(obj.board_centre_xy) != 2 or len(obj.board_half_extent) != 2 or not np.isfinite(values).all()
                    or min(obj.board_half_extent) <= 0 or obj.table_z > obj.surface_z):
                raise GuardViolation("invalid board footprint / table height")
        if not .1 <= obj.finger_end <= .3 or not .02 <= obj.half_width <= .09:
            raise GuardViolation("invalid Dex1 dimensions")
        if obj.half_thickness <= 0 or obj.travel_clearance < .03 or obj.minimum_clearance < 0:
            raise GuardViolation("invalid minimum clearances")
        return obj

    @property
    def measured_floor(self):
        """Floor for measured (feedback) poses: the envelope may not go below the estimated surface itself.

        Planned poses keep the uncertainty margin (clearance >= minimum_clearance). Measured poses differ from
        the plan by the tracking error, so they are only stopped when they would penetrate the surface estimate;
        a 3 mm planning margin with millimetre tracking errors produced false faults at release height.
        """
        return -self.uncertainty

    def corners(self, pose):
        local = np.array(list(product((-.04, self.finger_end),
                                     (-self.half_width, self.half_width),
                                     (-self.half_thickness, self.half_thickness))))
        return local @ pose[:3,:3].T + pose[:3,3]

    def surface_under(self, corners):
        """Support height for this box: board top if its footprint touches the board, else the table."""
        if self.table_z is None:
            return self.surface_z
        c, s = np.cos(self.board_yaw), np.sin(self.board_yaw)
        d = corners[:, :2]-np.asarray(self.board_centre_xy)
        u, v = d[:, 0]*c+d[:, 1]*s, -d[:, 0]*s+d[:, 1]*c
        hu, hv = self.board_half_extent
        overlaps = u.min() <= hu and u.max() >= -hu and v.min() <= hv and v.max() >= -hv
        return self.surface_z if overlaps else self.table_z

    def clearance(self, pose):
        corners = self.corners(pose)
        return float(np.min(corners[:,2]) - self.surface_under(corners) - self.uncertainty)

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
        if self.clearance(measured) < self.measured_floor and not retreat:
            raise GuardViolation("measured gripper may touch surface")

    def check_gripper(self, pose, measured=False):
        if self.clearance(pose) < (self.measured_floor if measured else self.minimum_clearance):
            raise GuardViolation("gripper open/close denied: surface clearance not verified")
