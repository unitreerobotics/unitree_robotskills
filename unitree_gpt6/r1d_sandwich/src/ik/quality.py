# Vendored from r1-ik r1ik/quality.py (6d28f1f + working tree, 2026-11-19). Keep in sync by copying; do not restyle.
"""Solver-independent posture quality checks.

"Accurate" is not enough: a 7-DoF arm can hit the hand pose with the elbow
flipped up, a wrist at its stop, or a forearm through the torso. Every IK
solution is scored here on the same metrics regardless of which solver made it.

Per-solution metrics (both arms, worst arm reported):
  limit_margin   min distance to a joint limit as a fraction of its range
  sigma_min      smallest singular value of the 6xN arm Jacobian (singularity)
  elbow_up_m     height of the elbow above the shoulder->hand line; a natural arm
                 has the elbow below it (negative), > 0 means the elbow points up
  elbow_out_m    sideways offset of the elbow from the shoulder->hand line, away
                 from the body ("chicken wing")
  elbow_in_m     elbow pushed past the shoulder toward the body midline
  collision_m    min mesh distance over non-trivial link pairs (coal)
  waist / lift   whole-body joint usage

Flags are explicit thresholds so a "weird pose" has a definition that can be
argued with and tuned, instead of an impression.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pinocchio as pin

from .kin import EE_FRAME, ReducedModel

SIDES = ("left", "right")


def _convex_hull(points: np.ndarray):
    """A proper qhull convex (triangulated, with the vertex neighbours GJK's support search needs)."""
    try:
        import coal as fcl
    except ImportError:  # older pinocchio builds ship hpp-fcl
        import hppfcl as fcl
    return fcl.ConvexBase.convexHull([np.asarray(p, float) for p in points], True, "Qt")


@dataclass
class Thresholds:
    limit_margin: float = 0.05  # <5% of range from a stop
    sigma_min: float = 0.02  # near-singular arm Jacobian
    elbow_up_m: float = 0.05  # elbow 5 cm above the shoulder->hand line
    elbow_out_m: float = 0.20  # elbow 20 cm sideways out of the shoulder->hand line
    elbow_in_m: float = 0.05  # elbow 5 cm past the shoulder toward the midline
    collision_m: float = 0.01  # closer than 1 cm between links
    joint_step_rad: float = 0.25  # per IK frame at 30-50 Hz on trajectories


class PostureChecker:
    def __init__(self, rm: ReducedModel, with_collision: bool = True, acm_samples: int = 400):
        self.rm = rm
        self.model = rm.model
        self.data = rm.model.createData()
        self.th = Thresholds()
        self.arm_cols = {s: [rm.index(n) for n in rm.variant.arm_joint_names(s)] for s in SIDES}
        self.limit_lo, self.limit_hi = rm.lower, rm.upper
        self.jid = {s: {"shoulder": self.model.getJointId(f"{s}_shoulder_roll_joint"),
                        "elbow": self.model.getJointId(f"{s}_elbow_joint")} for s in SIDES}
        self.geom = None
        if with_collision:
            self._build_collision(acm_samples)

    # --- collision -----------------------------------------------------
    def _hops(self, a: int, b: int) -> int:
        """Kinematic-tree distance between two joints of the reduced model."""
        def chain(j):
            out = [j]
            while j != 0:
                j = self.model.parents[j]
                out.append(j)
            return out
        ca, cb = chain(a), chain(b)
        common = next(j for j in ca if j in cb)
        return ca.index(common) + cb.index(common)

    def _build_collision(self, acm_samples: int):
        """Allowed-collision matrix the way MoveIt Setup Assistant builds one.

        Disabled: pairs on the same body or within 2 joints of each other
        (adjacent / shoulder-cluster links), pairs already < 1 cm apart in the
        nominal posture (CAD contact), and pairs < 1 cm apart in > 90% of random
        configurations ("always in collision"). What remains are pairs that are
        usually apart, so closeness there means a real self-collision.
        """
        urdf = str(self.rm.variant.urdf_path)
        mesh_dir = str(self.rm.variant.source_urdf.parent)
        try:
            geom = pin.buildGeomFromUrdf(self.model, urdf, pin.GeometryType.COLLISION, package_dirs=[mesh_dir])
        except Exception as e:  # noqa: BLE001 - report and continue without collision
            print(f"[quality] collision model unavailable: {e}")
            return
        objs = geom.geometryObjects
        # Convex hulls: GJK distance is ~50x faster than BVH mesh-mesh, and hulls are
        # slightly conservative (never under-report closeness). Built with qhull: the convex that
        # buildConvexRepresentation(False) returns keeps every mesh point (3770 for an elbow whose hull has
        # 845 vertices), and GJK's hill-climbing support on it stalls -- distances came out up to 97 mm too
        # large (left elbow / right wrist: 38.5 mm reported, <= 20 mm real).
        from scipy.spatial import ConvexHull
        for obj in objs:
            g = obj.geometry
            if hasattr(g, "buildConvexRepresentation"):
                g.buildConvexRepresentation(False)
                pts = np.asarray(g.convex.points())
                pts = pts[ConvexHull(pts).vertices]
                obj.geometry = _convex_hull(pts)
        for a in range(len(objs)):
            for b in range(a + 1, len(objs)):
                if self._hops(objs[a].parentJoint, objs[b].parentJoint) > 2:
                    geom.addCollisionPair(pin.CollisionPair(a, b))
        gdata = pin.GeometryData(geom)

        def dists(q):
            pin.computeDistances(self.model, self.data, geom, gdata, q)
            return np.array([r.min_distance for r in gdata.distanceResults])

        nominal = dists(self.rm.nominal_q())
        rng = np.random.default_rng(123)
        close = np.zeros(len(geom.collisionPairs))
        for _ in range(acm_samples):
            close += dists(self.rm.sample_q(rng, margin=0.0)) < self.th.collision_m
        keep = (nominal >= self.th.collision_m) & (close / acm_samples <= 0.9)
        pairs = [p for p, k in zip(geom.collisionPairs, keep) if k]
        self.acm_stats = {"candidate_pairs": len(keep), "kept_pairs": int(keep.sum()),
                          "nominal_contact": int((nominal < self.th.collision_m).sum()),
                          "always_close": int((close / acm_samples > 0.9).sum())}
        geom.removeAllCollisionPairs()
        for p in pairs:
            geom.addCollisionPair(p)
        self.geom, self.gdata = geom, pin.GeometryData(geom)
        self.pair_names = [(objs[p.first].name, objs[p.second].name) for p in pairs]

    def min_distance(self, q) -> tuple[float, str]:
        if self.geom is None or not self.pair_names:
            return np.nan, ""
        pin.computeDistances(self.model, self.data, self.geom, self.gdata, q)
        d = np.array([r.min_distance for r in self.gdata.distanceResults])
        i = int(np.argmin(d))
        return float(d[i]), "/".join(self.pair_names[i])

    # --- per-solution metrics ------------------------------------------
    def evaluate(self, q: np.ndarray) -> dict:
        q = np.asarray(q, dtype=float)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data, q)

        rng = self.limit_hi - self.limit_lo
        margin = np.minimum(q - self.limit_lo, self.limit_hi - q) / rng
        if self.rm.has("lift_stage1_joint"):  # resting at an end of lift travel is normal use
            margin[self.rm.index("lift_stage1_joint")] = 1.0
        out = {"limit_margin": float(margin.min()), "limit_joint": self.rm.active_names[int(np.argmin(margin))]}

        sig, up, side_out, side_in = [], [], [], []
        for s in SIDES:
            fid = self.model.getFrameId(EE_FRAME[s])
            J = pin.getFrameJacobian(self.model, self.data, fid, pin.LOCAL_WORLD_ALIGNED)
            sig.append(np.linalg.svd(J[:, self.arm_cols[s]], compute_uv=False).min())

            sh = self.data.oMi[self.jid[s]["shoulder"]].translation
            el = self.data.oMi[self.jid[s]["elbow"]].translation
            hand = self.data.oMf[fid].translation
            axis = hand - sh
            axis /= np.linalg.norm(axis) + 1e-9
            off = (el - sh) - axis * ((el - sh) @ axis)  # elbow offset from the shoulder->hand line
            # torso lateral direction (+y of the shoulder pitch parent frame), outward for this side
            R_body = self.data.oMi[self.model.parents[self.model.getJointId(f"{s}_shoulder_pitch_joint")]].rotation
            lateral = R_body[:, 1] * (1.0 if s == "left" else -1.0)
            up.append(float(off[2]))
            side_out.append(float(off @ lateral))
            # elbow past the shoulder toward the midline, in the body frame
            side_in.append(float(-(el - sh) @ lateral))
        out["sigma_min"] = float(min(sig))
        out["elbow_up_m"] = float(max(up))
        out["elbow_out_m"] = float(max(side_out))
        out["elbow_in_m"] = float(max(side_in))
        d, pair = self.min_distance(q)
        out["collision_m"] = d
        out["collision_pair"] = pair
        if self.rm.has("waist_yaw_joint"):
            out["waist"] = float(q[self.rm.index("waist_yaw_joint")])
        if self.rm.has("lift_stage1_joint"):
            out["lift"] = float(q[self.rm.index("lift_stage1_joint")])
        out["flags"] = self.flags(out)
        return out

    def flags(self, m: dict) -> list[str]:
        f = []
        th = self.th
        if m["limit_margin"] < th.limit_margin:
            f.append(f"near_limit:{m['limit_joint']}")
        if m["sigma_min"] < th.sigma_min:
            f.append("near_singular")
        if m["elbow_up_m"] > th.elbow_up_m:
            f.append("elbow_up")
        if m["elbow_out_m"] > th.elbow_out_m:
            f.append("elbow_out")
        if m["elbow_in_m"] > th.elbow_in_m:
            f.append("elbow_inward")
        if np.isfinite(m["collision_m"]) and m["collision_m"] < th.collision_m:
            f.append(f"collision:{m['collision_pair']}")
        return f

    def trajectory_flags(self, qs: np.ndarray, skip: tuple[str, ...] = ("lift_stage1_joint",)) -> dict:
        qs = np.asarray(qs)
        steps = np.abs(np.diff(qs, axis=0))
        for n in skip:
            if self.rm.has(n):
                steps[:, self.rm.index(n)] = 0.0
        big = steps.max(axis=1) > self.th.joint_step_rad
        jump_joints = [self.rm.active_names[int(np.argmax(steps[i]))] for i in np.flatnonzero(big)]
        return {"max_step_rad": float(steps.max()) if len(steps) else 0.0, "jumps": int(big.sum()),
                "jump_joints": jump_joints}
