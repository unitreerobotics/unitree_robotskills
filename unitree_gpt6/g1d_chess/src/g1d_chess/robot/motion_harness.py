"""Offline Cartesian interpolation and bounded IK; never sends robot commands."""
import warnings

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp
from g1d_chess.robot.g1_model import G1ArmModel
from g1d_chess.robot.board_guard import BoardGuard

# A bounded fallback for optimizers that stop at their evaluation budget while
# already meeting the geometric task tolerance.  This is deliberately tighter
# than the runtime waypoint tolerance and does not bypass branch-jump or scene
# envelope checks.
IK_FALLBACK_POSITION_M = .005
IK_FALLBACK_ROTATION_RAD = .025
IK_SOURCE_BRANCH_JUMP_RAD = 1.0

KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


def matrix(wp):
    if not isinstance(wp, dict) or any(k not in wp for k in KEYS):
        raise ValueError("each waypoint requires x/y/z/roll/pitch/yaw")
    values = np.array([wp[k] for k in KEYS], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError("waypoint contains non-finite values")
    if wp.get("gripper", "hold") not in ("open", "close", "hold"):
        raise ValueError("gripper must be open/close/hold")
    result = np.eye(4)
    result[:3, 3] = values[:3]
    result[:3, :3] = Rotation.from_euler("xyz", values[3:]).as_matrix()
    return result


def waypoint(T, gripper="hold"):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        angles = Rotation.from_matrix(T[:3, :3]).as_euler("xyz")
    return dict(zip(KEYS, map(float, [*T[:3, 3], *angles])), gripper=gripper)


class MotionHarness:
    def __init__(self):
        self.ik = G1ArmModel()
        self.model = self.ik.reduced_robot.model
        self.data = self.model.createData()
        self.base = np.eye(4)

    def fk(self, q):
        pin.framesForwardKinematics(self.model, self.data, np.asarray(q))
        return [self.base @ self.data.oMf[self.model.getFrameId(name)].homogeneous for name in ("L_ee", "R_ee")]

    def tau(self, q):
        return pin.rnea(self.model, self.data, q, np.zeros(14), np.zeros(14)).copy()

    @staticmethod
    def errors(actual, target):
        return [(float(np.linalg.norm(a[:3, 3]-b[:3, 3])),
                 float(Rotation.from_matrix(a[:3, :3] @ b[:3, :3].T).magnitude()))
                for a, b in zip(actual, target)]

    def solve(self, targets, seed, check=True):
        def residual(q):
            out = []
            for actual, target in zip(self.fk(q), targets):
                out.extend(10 * (actual[:3, 3] - target[:3, 3]))
                out.extend(Rotation.from_matrix(actual[:3, :3] @ target[:3, :3].T).as_rotvec())
            out.extend(.006 * (q-seed))
            return out
        lower = self.model.lowerPositionLimit + .15
        upper = self.model.upperPositionLimit - .15
        # Tabletop task posture: avoid inward shoulder roll and extreme yaw.
        lower[1] = -.15
        upper[8] = .15
        lower[[2,9]] = -1.2
        upper[[2,9]] = 1.2
        lower[[3,10]] = -.8
        sol = least_squares(residual, np.clip(seed, lower, upper), bounds=(lower, upper),
                            max_nfev=120, ftol=1e-8, xtol=1e-8, gtol=1e-8)
        errors = self.errors(self.fk(sol.x), targets)
        geometric_ok = all(p <= IK_FALLBACK_POSITION_M and r <= IK_FALLBACK_ROTATION_RAD
                           for p, r in errors)
        if check and (not sol.success and not geometric_ok):
            raise ValueError(f"IK target rejected: errors(position m, angle rad)={errors}")
        return sol.x

    def prepare(self, plan, q, guard=None, *, ignore_right_envelope_steps=()):
        speed_scale = plan.get("speed_scale", 1.0)
        if (isinstance(speed_scale, bool) or not isinstance(speed_scale, (int, float))
                or not np.isfinite(speed_scale) or not 0 < speed_scale <= 1):
            raise ValueError("speed_scale must be finite and satisfy 0 < speed_scale <= 1")
        skip_preflight = plan.get("skip_preflight", False)
        guard = guard or BoardGuard.from_dict(plan.get("scene"))
        arms = [plan.get(k) for k in ("left_arm", "right_arm")]
        if any(not isinstance(a, list) or not a for a in arms):
            raise ValueError("both arms need at least one waypoint")
        if max(map(len, arms)) > 100:
            raise ValueError("too many waypoints")
        # Validate the entire request before doing any hardware work.
        for arm in arms:
            for wp in arm:
                matrix(wp)
        previous = self.fk(q)
        seed = q.copy()
        result = []
        for i in range(max(map(len, arms))):
            waypoints = [arm[min(i, len(arm)-1)] for arm in arms]
            targets = [matrix(wp) for wp in waypoints]
            # Gripper actions hold the arm; movement needs a preceding hold waypoint.
            for j, wp in enumerate(waypoints):
                if wp.get("gripper", "hold") != "hold":
                    current = waypoint(previous[j])
                    waypoints[j] = dict(wp, **{key: current[key] for key in KEYS})
                    targets[j] = previous[j].copy()
            if not skip_preflight:
                for j,(start, end, wp) in enumerate(zip(previous, targets, waypoints)):
                    exempt=j==1 and i in ignore_right_envelope_steps
                    if not exempt:
                        guard.check_segment(start, end)
                    if wp.get("gripper", "hold") != "hold":
                        if not exempt:
                            guard.check_gripper(end)
            errors = self.errors(previous, targets)
            n = max(1, int(np.ceil(max(max(p/.008, r/.05) for p,r in errors))))
            rotations = [Slerp([0,1], Rotation.from_matrix(np.stack([a[:3,:3],b[:3,:3]])))
                         for a,b in zip(previous, targets)]
            samples = []
            branch_limit = (IK_SOURCE_BRANCH_JUMP_RAD
                            if i in ignore_right_envelope_steps else .3)

            def advance(start_alpha, alpha, depth=0):
                nonlocal seed
                poses = []
                for a,b,slerp in zip(previous,targets,rotations):
                    T = np.eye(4)
                    T[:3,3] = a[:3,3]*(1-alpha)+b[:3,3]*alpha
                    T[:3,:3] = slerp(alpha).as_matrix()
                    poses.append(T)
                solution = self.solve(poses, seed, check=not skip_preflight)
                if alpha == 1. and not skip_preflight and any(wp.get("gripper") == "open" for wp in waypoints):
                    endpoint_errors = self.errors(self.fk(solution), targets)
                    if any(p > IK_FALLBACK_POSITION_M or r > IK_FALLBACK_ROTATION_RAD
                           for p,r in endpoint_errors):
                        solution = self.solve(targets, solution, check=True)
                delta = np.max(np.abs(solution-seed))
                if not skip_preflight and delta > branch_limit:
                    if depth >= 5:
                        raise ValueError(f"IK branch jump ({delta:.3f} rad)")
                    # Subdivide inside this waypoint; do not reset the Cartesian
                    # endpoint to approximate FK or create extra action waypoints.
                    middle = (start_alpha + alpha) / 2
                    advance(start_alpha, middle, depth+1)
                    advance(middle, alpha, depth+1)
                    return
                # Validate the commanded joint interpolation, including its first
                # segment from current feedback. Endpoints alone are insufficient.
                if not skip_preflight:
                    for beta in np.linspace(0, 1, 5):
                        interpolated = self.fk(seed + beta*(solution-seed))
                        for j,(start, actual) in enumerate(zip(previous, interpolated)):
                            if not (j==1 and i in ignore_right_envelope_steps):
                                guard.check_segment(start, actual)
                duration = max(.12, 1.6*delta/.20,
                               max(p for p,r in errors)*(alpha-start_alpha)/.025,
                               max(r for p,r in errors)*(alpha-start_alpha)/.2)
                duration = float(duration) / speed_scale
                if not np.isfinite(duration):
                    raise ValueError("speed_scale produces a non-finite duration")
                samples.append((solution.copy(), duration))
                seed = solution
            last_alpha = 0.
            for alpha in np.linspace(0, 1, n+1)[1:]:
                advance(last_alpha, float(alpha))
                last_alpha = float(alpha)
            endpoint_errors = self.errors(self.fk(seed), targets)
            if (not skip_preflight and any(wp.get("gripper") == "open" for wp in waypoints)
                    and any(p > IK_FALLBACK_POSITION_M or r > IK_FALLBACK_ROTATION_RAD
                            for p,r in endpoint_errors)):
                raise ValueError(f"IK waypoint endpoint rejected: {endpoint_errors}")
            result.append({"samples": samples, "targets": targets, "waypoints": waypoints})
            previous = targets
        return result
