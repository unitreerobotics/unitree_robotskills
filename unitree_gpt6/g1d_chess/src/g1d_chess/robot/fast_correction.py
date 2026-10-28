"""Runtime IK for small source corrections; never runs a path preflight.

The original transaction still owns the checked, frozen target/init tails.
Only changed source prefixes and their connection to a frozen joint boundary
are rebuilt. Hardware and scene feedback checks remain in the executor.
"""
import copy

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from g1d_chess.robot.motion_harness import matrix, waypoint


class RuntimeSourcePaths:
    def __init__(self, motion, open_q, open_sign, health):
        self.motion = motion
        self.open_q = open_q * open_sign
        self.health = health
        self.solutions = {}
        self.solve_count = 0

    @staticmethod
    def _key(targets):
        return tuple(np.round(np.concatenate([p.ravel() for p in targets]), 10))

    def remember(self, targets, q):
        self.solutions[self._key(targets)] = np.asarray(q, dtype=float).copy()

    def step(self, waypoints, seed, speed_scale=1., fixed_end_q=None):
        """Convert one Cartesian command to bounded joint samples, without guard preflight."""
        if not np.isfinite(speed_scale) or not 0 < speed_scale <= 1:
            raise ValueError("invalid runtime correction speed")
        targets = [matrix(p) for p in waypoints]
        current = np.asarray(seed, dtype=float).copy()
        if current.shape != (14,) or not np.isfinite(current).all():
            raise ValueError("runtime correction IK produced invalid joints")
        previous = [p.copy() for p in self.motion.fk(current)]
        if any(p.get("gripper", "hold") != "hold" for p in waypoints):
            # These actions already passed the transaction's stationary-action
            # preflight. Residual pose error must never turn an open/close into
            # another arm move, including intermediate IK samples.
            if fixed_end_q is not None and not np.array_equal(current, fixed_end_q):
                raise ValueError("runtime gripper boundary must already be joined")
            self.health()
            return {"samples": [(current, .12/speed_scale)], "targets": previous,
                    "waypoints": copy.deepcopy(waypoints)}
        if fixed_end_q is None:
            # The observation arm holds its present joints; its old nominal
            # target is not a new runtime IK objective.
            targets[0] = previous[0].copy()
        errors = self.motion.errors(previous, targets)
        count = max(1, int(np.ceil(max(max(p/.008, r/.05) for p, r in errors))))
        rotations = [Slerp([0, 1], Rotation.from_matrix(np.stack([a[:3, :3], b[:3, :3]])))
                     for a, b in zip(previous, targets)]
        samples = []
        for index, alpha in enumerate(np.linspace(0, 1, count+1)[1:]):
            self.health()
            poses = []
            for start, end, rotation in zip(previous, targets, rotations):
                pose = np.eye(4)
                pose[:3, 3] = start[:3, 3]*(1-alpha) + end[:3, 3]*alpha
                pose[:3, :3] = rotation(alpha).as_matrix()
                poses.append(pose)
            key = self._key(poses)
            if fixed_end_q is not None and index == count-1:
                solution = np.asarray(fixed_end_q, dtype=float).copy()
            elif all(p < 1e-8 and r < 1e-8 for p, r in self.motion.errors(self.motion.fk(current), poses)):
                solution = current.copy()
            elif key in self.solutions:
                solution = self.solutions[key].copy()
            else:
                solution = np.asarray(self.motion.solve(poses, current.copy(), check=False), dtype=float)
                self.solve_count += 1
            if solution.shape != current.shape or not np.isfinite(solution).all():
                raise ValueError("runtime correction IK produced invalid joints")
            # Ordinary source IK holds the observer. The final connection must
            # retain all 14 frozen joints exactly, including its left boundary.
            if fixed_end_q is None or index != count-1:
                solution[:7] = seed[:7]
            delta = float(np.max(np.abs(solution-current)))
            duration = max(.12, 1.6*delta/.20,
                           max(p for p, _ in errors)/count/.025,
                           max(r for _, r in errors)/count/.2)/speed_scale
            # A source correction may join a different redundant IK branch at
            # the frozen boundary. Joint distance is a sampling/time input,
            # never an execution error gate. Keep each connector continuous and
            # retain the executor's collision and hardware checks for every
            # subsegment. Its cubic blend peaks at 1.5 * delta / duration.
            subdivisions = max(1, int(np.ceil(delta/.1)))
            sample_duration = max(.12/speed_scale, duration/subdivisions)
            for part in range(1, subdivisions+1):
                if part > 1:
                    self.health()
                endpoint = (solution.copy() if part == subdivisions else
                            current+(solution-current)*(part/subdivisions))
                samples.append((endpoint, float(sample_duration)))
            self.solutions[key] = solution.copy()
            current = solution
        return {"samples": samples, "targets": targets, "waypoints": copy.deepcopy(waypoints)}

    def assemble(self, plan, raw_steps, seed, grip_seed):
        """Add frozen reverse samples and gripper actions; this is not a preflight."""
        previous = np.asarray(seed).copy()
        grip = np.asarray(grip_seed).copy()
        steps, reverse_steps = [], []
        for raw in raw_steps:
            step = copy.deepcopy(raw)
            step["seed"] = previous.copy()
            step["gripper_targets"] = {}
            restore = {}
            for j, (side, wp) in enumerate(zip(("left", "right"), step["waypoints"])):
                action = wp.get("gripper", "hold")
                if action != "hold":
                    restore[side] = float(grip[j])
                    grip[j] = self.open_q if action == "open" else 0.
                    step["gripper_targets"][side] = float(grip[j])
            starts = [previous.copy()] + [q.copy() for q, _ in step["samples"][:-1]]
            reverse_samples = [(q.copy(), float(duration)) for q, (_, duration)
                               in reversed(list(zip(starts, step["samples"])))]
            end = step["samples"][-1][0].copy()
            previous_poses = [p.copy() for p in self.motion.fk(previous)]
            end_poses = [p.copy() for p in self.motion.fk(end)]
            reverse = {"seed": end.copy(), "samples": reverse_samples,
                       "targets": previous_poses,
                       "waypoints": [waypoint(p) for p in previous_poses], "gripper_targets": {}}
            undo = {"seed": end.copy(), "samples": [(end.copy(), .12)], "targets": end_poses,
                    "waypoints": [waypoint(p) for p in end_poses], "gripper_targets": restore}
            reverse_steps.append((undo, reverse))
            step["end_grip"] = grip.copy()
            steps.append(step)
            previous = end
        return {"plan": copy.deepcopy(plan), "steps": steps, "reverse": reverse_steps,
                "seed": np.asarray(seed).copy(), "end_q": previous, "end_grip": grip}

    def prefix(self, original, plan, shifted_indices, seed, grip_seed, fixed_end_q=None):
        """Rebuild changed prefix and first fixed waypoint, then retain frozen tail."""
        last_changed = max(shifted_indices)
        stop = min(last_changed+2, len(plan["right_arm"]))
        current = np.asarray(seed).copy()
        steps = []
        boundary_index = stop-1
        boundary = (original["steps"][boundary_index]["samples"][-1][0]
                    if boundary_index > last_changed else fixed_end_q)
        if boundary is not None:
            # Join a frozen endpoint during the preceding movement, so a
            # terminal open/close remains stationary even with IK residuals.
            while boundary_index >= 0 and any(
                    arm[min(boundary_index, len(arm)-1)].get("gripper", "hold") != "hold"
                    for arm in (plan["left_arm"], plan["right_arm"])):
                boundary_index -= 1
            if boundary_index < 0 and not np.array_equal(current, boundary):
                raise ValueError("runtime gripper boundary has no preceding movement")
        for index in range(stop):
            waypoints = [plan["left_arm"][min(index, len(plan["left_arm"])-1)],
                         plan["right_arm"][index]]
            frozen = boundary if index == boundary_index else None
            step = self.step(waypoints, current, plan.get("speed_scale", 1.), frozen)
            steps.append(step)
            current = step["samples"][-1][0]
        # These joint samples, target placement and init return are unchanged.
        steps.extend(original["steps"][stop:])
        result = self.assemble(plan, steps, seed, grip_seed)
        result["runtime_source_prefix_steps"] = stop
        result["frozen_tail_start_step"] = stop
        return result
