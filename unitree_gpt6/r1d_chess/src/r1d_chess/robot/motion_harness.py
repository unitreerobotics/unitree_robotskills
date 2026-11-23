"""Offline Cartesian interpolation and bounded IK; never sends robot commands."""
import warnings

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp
from r1d_chess.robot.r1d_kinematics import R1DKinematics
from r1d_chess.robot.board_guard import BoardGuard, GuardViolation

# A bounded fallback for optimizers that stop at their evaluation budget while
# already meeting the geometric task tolerance.  This is deliberately tighter
# than the runtime waypoint tolerance and does not bypass branch-jump or scene
# envelope checks.
IK_FALLBACK_POSITION_M = .005
IK_FALLBACK_ROTATION_RAD = .025
IK_SOURCE_BRANCH_JUMP_RAD = 1.0
# joint_q alignment may deviate this much.
JOINT_ALIGN_TOLERANCE_RAD = .1

KEYS = ("x", "y", "z", "roll", "pitch", "yaw")
# preflight checks only the actor (role index 1, logical right); the observer (index 0)
# is still held by IK but never rejects a plan.
CHECK_OBSERVER = False


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
        self.robot = R1DKinematics()
        self.base = np.eye(4)

    def fk(self, q):
        return [self.base @ pose for pose in self.robot.fk(q)]

    def tau(self, q):
        # R1-D Runtime supplies configured PD gains. No G1 torque model.
        return np.zeros(14)

    @staticmethod
    def errors(actual, target):
        return [(float(np.linalg.norm(a[:3, 3]-b[:3, 3])),
                 float(Rotation.from_matrix(a[:3, :3] @ b[:3, :3].T).magnitude()))
                for a, b in zip(actual, target)]

    def solve(self, targets, seed, check=True):
        inverse = np.linalg.inv(self.base)
        return self.robot.solve([inverse @ target for target in targets], seed)

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
                if "joint_q" in wp:
                    jq = np.asarray(wp["joint_q"], dtype=float)
                    if jq.shape != (7,) or not np.isfinite(jq).all() or wp.get("gripper", "hold") != "hold":
                        raise ValueError("joint_q waypoint needs 7 finite joints and gripper hold")
        previous = self.fk(q)
        seed = q.copy()
        result = []
        for i in range(max(map(len, arms))):
            waypoints = [arm[min(i, len(arm)-1)] for arm in arms]
            if any("joint_q" in wp for wp in waypoints[:2]):
                # Joint-space alignment: end exactly on the given joint angles,
                # e.g. init_joint_rad after the Cartesian return to init.
                goal = seed.copy()
                for j, wp in enumerate(waypoints):
                    if "joint_q" in wp:
                        goal[7*j:7*j+7] = np.asarray(wp["joint_q"], dtype=float)
                    elif max(self.errors([previous[j]], [matrix(wp)])[0]) > 1e-3:
                        raise ValueError(f"waypoint {i}: the arm without joint_q must hold its pose")
                goal_poses = self.fk(goal)
                # A goal below the step start (init after the travel-height return): build the vertical
                # chain upward from the goal joints (IK seeded from them), change configuration at the
                # start height, then descend along that chain. Below travel only vertical motion is
                # allowed, and the chain ends exactly on the goal joints.
                chain = [goal.copy()]
                descent_from = None
                low = [j for j in range(2) if "joint_q" in waypoints[j]
                       and goal_poses[j][2, 3] < previous[j][2, 3]-.002]
                if float(np.max(np.abs(goal-seed))) <= JOINT_ALIGN_TOLERANCE_RAD:
                    chain = [seed.copy()]
                    goal = seed.copy()
                elif low:
                    if len(low) != 1 or np.linalg.norm(goal_poses[low[0]][:2, 3]-previous[low[0]][:2, 3]) > .002:
                        raise ValueError(f"waypoint {i}: joint_q descent must be vertical below its start")
                    j = low[0]
                    z0, z1 = goal_poses[j][2, 3], previous[j][2, 3]
                    q_chain = goal.copy()
                    for z in list(np.arange(z0+.01, z1, .01))+[z1]:
                        poses = [T.copy() for T in previous]
                        poses[j] = goal_poses[j].copy(); poses[j][2, 3] = z
                        q_chain = self.solve(poses, q_chain, check=not skip_preflight)
                        chain.append(q_chain.copy())
                    chain.reverse()          # top (start height) ... goal
                    # Pose-constrained configuration swap: a plain joint interpolation into
                    # the chain top dips below travel height and moves sideways. Instead hold the actor
                    # TCP on the line start -> chain top (normally the same point) and step each IK seed
                    # from the current configuration toward the chain top, so the TCP does not move.
                    top = chain[0]
                    t_top = self.fk(top)
                    rot = Slerp([0, 1], Rotation.from_matrix(np.stack([previous[j][:3, :3], t_top[j][:3, :3]])))
                    n_swap = max(1, int(np.ceil(float(np.max(np.abs(top-seed)))/.04)))
                    swap = []
                    for k in range(1, n_swap):
                        a = k/n_swap
                        poses = [T.copy() for T in previous]
                        poses[j][:3, 3] = previous[j][:3, 3]*(1-a)+t_top[j][:3, 3]*a
                        poses[j][:3, :3] = rot(a).as_matrix()
                        swap.append(self.solve(poses, seed+(top-seed)*a, check=not skip_preflight))
                    chain = swap+chain
                    # The descent is checked against the chain top (after the swap), not the step start.
                    descent_from = len(swap)+1
                samples = []
                cursor = seed.copy()
                reference = previous
                for index_node, node in enumerate(chain):
                    if descent_from is not None and index_node == descent_from:
                        reference = self.fk(chain[descent_from-1])
                    d = float(np.max(np.abs(node-cursor)))
                    n = max(1, int(np.ceil(d/.05)))
                    for k in range(1, n+1):
                        qk = cursor + (node-cursor)*k/n
                        if not skip_preflight:
                            for beta in (.5, 1.):
                                pts = self.fk(cursor + (node-cursor)*(k-1+beta)/n)
                                for jj, (start, actual) in enumerate(zip(reference, pts)):
                                    if jj == 0 and not CHECK_OBSERVER:
                                        continue
                                    try:
                                        guard.check_segment(start, actual)
                                    except GuardViolation as exc:
                                        raise GuardViolation(f'waypoint {i}, arm {jj}, joint alignment: {exc}') from exc
                        samples.append((qk.copy(), max(.12, 1.6*(d/n)/.20)/speed_scale))
                    cursor = node.copy()
                seed = goal.copy()
                targets = self.fk(goal)
                result.append({"samples": samples, "targets": targets,
                               "waypoints": [waypoint(T) for T in targets], "joint_alignment": True})
                previous = targets
                continue
            targets = [matrix(wp) for wp in waypoints]
            # Gripper actions hold the arm; movement needs a preceding hold waypoint.
            for j, wp in enumerate(waypoints):
                if wp.get("gripper", "hold") != "hold":
                    current = waypoint(previous[j])
                    waypoints[j] = dict(wp, **{key: current[key] for key in KEYS})
                    targets[j] = previous[j].copy()
            if not skip_preflight:
                for j,(start, end, wp) in enumerate(zip(previous, targets, waypoints)):
                    if j == 0 and not CHECK_OBSERVER:
                        continue
                    exempt=j==1 and i in ignore_right_envelope_steps
                    if not exempt:
                        try:
                            guard.check_segment(start, end)
                        except GuardViolation as exc:
                            raise GuardViolation(f'waypoint {i}, arm {j}, endpoint: {exc}') from exc
                    if wp.get("gripper", "hold") != "hold":
                        if not exempt:
                            try:
                                guard.check_gripper(end)
                            except GuardViolation as exc:
                                raise GuardViolation(f'waypoint {i}, arm {j}, gripper: {exc}') from exc
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
                try:
                    solution = self.solve(poses, seed, check=not skip_preflight)
                except ValueError as exc:
                    if not skip_preflight and depth < 5:
                        middle = (start_alpha + alpha) / 2
                        advance(start_alpha, middle, depth+1)
                        advance(middle, alpha, depth+1)
                        return
                    raise ValueError(f'waypoint {i}, alpha {alpha:.3f}: {exc}') from exc
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
                            if j == 0 and not CHECK_OBSERVER:
                                continue
                            if not (j==1 and i in ignore_right_envelope_steps):
                                try:
                                    guard.check_segment(start, actual)
                                except GuardViolation as exc:
                                    raise GuardViolation(
                                        f'waypoint {i}, arm {j}, alpha {alpha:.3f}, '
                                        f'beta {beta:.2f}: {exc}'
                                    ) from exc
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
