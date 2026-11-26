# Vendored from r1-ik r1ik/solvers.py (5b867e6 + working tree, 2026-11-19). Keep in sync by copying; do not restyle.
"""IK backends sharing one interface.

Both take per-side SE3 targets in the robot base frame and return joint values
in the ReducedModel's joint order (only the mode's active joints).

- CasadiIK: single-shot IPOPT, the formulation unirobot's teleop uses today.
- RoboKitIK: GPU-batched Levenberg-Marquardt with sampled candidates.
- "oracle": RoboKitIK with 512 seeds and no regularization, used only to label
  whether a target is reachable at all, so solver failures and infeasible
  targets are not mixed up.

Weights are residual weights (cost = sum (w * r)^2). Accuracy dominates: moving a
joint by 1 rad costs about as much as 1 mm of hand error, so the posture/smooth
terms only pick among (near-)exact solutions instead of pulling the hand off.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
import pinocchio as pin

from .kin import EE_FRAME, ReducedModel

SIDES = ("left", "right")


@dataclass
class IKWeights:
    position: float = 20.0  # residual weight per metre
    rotation: float = 10.0  # residual weight per radian
    posture: float = 0.002  # pull toward the nominal posture (null-space preference)
    smooth: float = 0.02  # pull toward the previous solution
    # Soft joint-limit band: penalize entering the outer `limit_band` fraction of each
    # range, so redundancy (waist/lift/elbow swivel) is used before a joint is pinned
    # at its stop. 0.1 rad into the band costs about as much as 5 mm of hand error.
    limit_band: float = 0.10
    limit_weight: float = 1.0
    # Extra smoothness multipliers for slow / heavy joints in whole-body modes.
    joint_smooth_scale: dict[str, float] = field(
        default_factory=lambda: {"lift_stage1_joint": 30.0, "waist_yaw_joint": 3.0}
    )
    # Multipliers on `posture` by joint kind (name without side / "_joint"); missing kinds count 1.
    # ANTHRO_POSTURE makes the pull human-like instead of a uniform joint distance.
    posture_scale: dict[str, float] = field(default_factory=dict)

    def smooth_vector(self, names: list[str]) -> np.ndarray:
        return np.array([self.smooth * self.joint_smooth_scale.get(n, 1.0) for n in names])

    def posture_vector(self, names: list[str]) -> np.ndarray:
        return np.array([self.posture * self.posture_scale.get(joint_kind(n), 1.0) for n in names])


def joint_kind(name: str) -> str:
    """'left_shoulder_roll_joint' -> 'shoulder_roll'."""
    for p in ("left_", "right_"):
        name = name.removeprefix(p)
    return name.removesuffix("_joint")


# What makes an arm look forced is the upper arm swung out (shoulder roll, "chicken wing") or twisted
# (shoulder yaw); a person reaches with shoulder pitch and elbow and lets the wrist take the orientation.
# So the posture pull is strongest on roll / yaw, light on pitch / elbow, weakest on the wrist; never
# on the lift or the waist (whole-body joints are steered by their own smoothness terms).
ANTHRO_POSTURE = {"shoulder_roll": 1.0, "shoulder_yaw": 0.5, "shoulder_pitch": 0.15, "elbow": 0.15,
                  "wrist_roll": 0.05, "wrist_pitch": 0.05, "wrist_yaw": 0.05,
                  "lift_stage1": 0.0, "waist_yaw": 0.0}


@dataclass
class IKResult:
    q: np.ndarray
    time_ms: float
    ok: bool = True


class CasadiIK:
    name = "casadi"

    def __init__(self, rm: ReducedModel, weights: IKWeights | None = None, max_iter: int = 30):
        import casadi
        from pinocchio import casadi as cpin

        self.rm = rm
        self.w = weights or IKWeights()
        model = rm.model
        cmodel = cpin.Model(model)
        cdata = cmodel.createData()
        q = casadi.SX.sym("q", model.nq)
        cpin.framesForwardKinematics(cmodel, cdata, q)
        targets = {s: casadi.SX.sym(f"T_{s}", 4, 4) for s in SIDES}
        p_err, r_err = [], []
        for s in SIDES:
            M = cdata.oMf[model.getFrameId(EE_FRAME[s])]
            p_err.append(M.translation - targets[s][:3, 3])
            # Frobenius residual: smooth everywhere (log3 has a NaN gradient at zero
            # error); ||R - Rt||_F^2 ~= 2 theta^2, so scale by 1/sqrt(2) to match.
            r_err.append(casadi.reshape(M.rotation - targets[s][:3, :3], 9, 1) / np.sqrt(2.0))
        err_fn = casadi.Function("err", [q, *targets.values()], [casadi.vertcat(*p_err), casadi.vertcat(*r_err)])

        opti = casadi.Opti()
        self.q = opti.variable(model.nq)
        self.q_prev = opti.parameter(model.nq)
        self.q_rest = opti.parameter(model.nq)
        self.T = {s: opti.parameter(4, 4) for s in SIDES}
        ep, er = err_fn(self.q, *self.T.values())
        smooth = casadi.DM(self.w.smooth_vector(rm.active_names) ** 2)
        band = self.w.limit_band * (rm.upper - rm.lower)
        band_mask = np.array([0.0 if n == "lift_stage1_joint" else 1.0 for n in rm.active_names])
        lo_b, hi_b = rm.lower + band, rm.upper - band
        soft = casadi.fmax(0, casadi.DM(lo_b) - self.q) + casadi.fmax(0, self.q - casadi.DM(hi_b))
        cost = (
            self.w.limit_weight**2 * casadi.sumsqr(casadi.DM(band_mask) * soft)
            +
            self.w.position**2 * casadi.sumsqr(ep)
            + self.w.rotation**2 * casadi.sumsqr(er)
            + casadi.sumsqr(casadi.DM(self.w.posture_vector(rm.active_names)) * (self.q - self.q_rest))
            + casadi.sum1(smooth * (self.q - self.q_prev) ** 2)
        )
        opti.subject_to(opti.bounded(rm.lower, self.q, rm.upper))
        opti.minimize(cost)
        opti.solver(
            "ipopt",
            {"expand": True, "print_time": False, "ipopt.sb": "yes", "ipopt.print_level": 0,
             "ipopt.max_iter": max_iter, "ipopt.tol": 1e-6, "ipopt.acceptable_tol": 1e-5},
        )
        self.opti = opti
        self.q_rest_value = rm.nominal_q()

    def solve(self, targets: dict[str, pin.SE3], q_seed: np.ndarray, q_prev: np.ndarray | None = None) -> IKResult:
        q_prev = q_seed if q_prev is None else q_prev
        t0 = time.perf_counter()
        self.opti.set_initial(self.q, q_seed)
        self.opti.set_value(self.q_prev, q_prev)
        self.opti.set_value(self.q_rest, self.q_rest_value)
        for s in SIDES:
            self.opti.set_value(self.T[s], targets[s].homogeneous)
        ok = True
        try:
            sol = self.opti.solve()
            q = np.asarray(sol.value(self.q)).reshape(-1)
        except RuntimeError:  # max_iter hit; keep the last iterate like unirobot's teleop fallback
            ok = False
            q = np.asarray(self.opti.debug.value(self.q)).reshape(-1)
        return IKResult(self.rm.clip(q), (time.perf_counter() - t0) * 1e3, ok)


class RoboKitIK:
    """robokit IKSolver over the full IK URDF; non-mode joints are masked."""

    name = "robokit"

    def __init__(self, rm: ReducedModel, weights: IKWeights | None = None, tracking: bool = False,
                 device: str | None = None, candidates: int = 64, select_k: int = 1, checker=None,
                 lm_steps: int = 30):
        import warp as wp
        from robokit.helpers.ik import IKContext, IKSolver, IKSolverConfig
        from robokit.opt.lm_optimizer import LMOptimizerConfig
        from robokit.opt.population_solver import PopulationSolverConfig, Stage
        from robokit.opt.sampler import RobertsSamplerConfig
        from robokit.robo import Robot
        from robokit.terms.dense.position_limit import PositionLimit
        from robokit.terms.dense.position_task import PositionTask
        from robokit.terms.dense.rotation_task import RotationTask
        from robokit.terms.dense.smoothness_task import SmoothnessTask
        from robokit.terms.dense.configuration_task import ConfigurationTask

        wp.config.quiet = True
        wp.init()
        self.wp = wp
        self.rm = rm
        self.w = weights or IKWeights()
        self.tracking = tracking
        self.device = device or ("cuda" if wp.is_cuda_available() else "cpu")
        self.robot = Robot.load(str(rm.variant.urdf_path))
        names = list(self.robot.spec.actuated_joint_names)
        self.names = names
        self.to_rk = np.array([names.index(n) for n in rm.active_names])

        # rest posture for every robokit joint: locked joints keep their locked value
        full_names = list(rm.full.names[1:])
        self.rest = np.zeros(len(names), dtype=np.float32)
        for i, n in enumerate(names):
            if n in full_names:
                self.rest[i] = rm.q_locked_full[full_names.index(n)]
        self.nominal = self.rest.copy()
        self.nominal[self.to_rk] = rm.nominal_q()
        mask = np.zeros(len(names), dtype=np.float32)
        mask[self.to_rk] = 1.0
        self.select_k = 1 if tracking else select_k
        self.checker = checker
        smooth = np.full(len(names), self.w.smooth, dtype=np.float32)
        smooth[self.to_rk] = self.w.smooth_vector(rm.active_names)

        if tracking:  # one warm-started candidate, like the robokit tracking presets
            solver = PopulationSolverConfig(stages=[Stage(lm_steps, 1, LMOptimizerConfig(lm_lambda=1.0))], cuda_graph_mode="full")
            sampler = RobertsSamplerConfig(keep_first=True, scale=1.0)
        else:  # global search: many sampled seeds, keep the best
            solver = PopulationSolverConfig(
                stages=[Stage(6, candidates, LMOptimizerConfig(lm_lambda=10.0)),
                        Stage(10 if candidates <= 64 else 25, max(self.select_k, 4, candidates // 16), LMOptimizerConfig(lm_lambda=1.0)),
                        Stage(0, self.select_k)],
                cuda_graph_mode="full",
                rank_final=self.select_k > 1,
            )
            sampler = RobertsSamplerConfig(keep_first=True, scale=1.0)
        config = (
            IKSolverConfig(solver=solver, sampler=sampler)
            .add(PositionTask(frame_index=IKContext.TARGET_LINKS, T_world_target=IKContext.TARGET, weight=self.w.position))
            .add(RotationTask(frame_index=IKContext.TARGET_LINKS, T_world_target=IKContext.TARGET, weight=self.w.rotation))
            .add(PositionLimit(weight=50.0))
            .add(SmoothnessTask(prev_var=IKContext.PREVIOUS_STATE, weight=smooth.tolist()))
        )
        if self.w.limit_weight > 0:
            band_mask = {n: (0.0 if n == "lift_stage1_joint" else 1.0) for n in names}
            config = config.add(PositionLimit(weight=self.w.limit_weight, activation=self.w.limit_band, mask=band_mask))
        if self.w.posture > 0:
            posture = np.zeros(len(names), dtype=np.float32)
            posture[self.to_rk] = self.w.posture_vector(rm.active_names)
            config = config.add(ConfigurationTask(rest_q=self.nominal, weight=posture.tolist()))
        self.ik = IKSolver(config, robot=self.robot, target_links=list(rm.variant.ee_links), device=self.device)
        self.ik.set_active_joint_mask(mask.tolist())
        self._rest_state = self.robot.state(q=wp.from_numpy(self.rest[None], dtype=wp.float32, device=self.device))
        self.ik.warmup(batch_size=1)

    def _state(self, q_reduced: np.ndarray):
        q = self.rest.copy()
        q[self.to_rk] = q_reduced
        return self.robot.state(q=self.wp.from_numpy(q[None], dtype=self.wp.float32, device=self.device))

    def solve(self, targets: dict[str, pin.SE3], q_seed: np.ndarray, q_prev: np.ndarray | None = None) -> IKResult:
        q_prev = q_seed if q_prev is None else q_prev
        T = np.stack([np.concatenate([targets[s].translation, pin.Quaternion(targets[s].rotation).coeffs()[[3, 0, 1, 2]]]) for s in SIDES])
        t0 = time.perf_counter()
        seed = self._state(q_seed)
        # rest_state pins masked joints (waist/lift outside their modes) to their locked values
        if self.select_k == 1:
            state = self.ik.solve_numpy(T.astype(np.float32), init_state=seed, prev_state=self._state(q_prev),
                                        rest_state=self._rest_state)
            q = state.q.numpy()[0][self.to_rk].astype(np.float64)
            return IKResult(self.rm.clip(q), (time.perf_counter() - t0) * 1e3)
        state = self.ik.solve_numpy(T.astype(np.float32), init_state=seed, rest_state=self._rest_state)
        cands = [self.rm.clip(c[self.to_rk].astype(np.float64)) for c in state.q.numpy()]  # best cost first
        q = self._select(cands, targets, q_prev)
        return IKResult(q, (time.perf_counter() - t0) * 1e3)

    def _select(self, cands, targets, q_ref, pos_tol=2e-3, rot_tol=1.0):
        """Among candidates that hit the target, prefer no posture flags, then closest to q_ref."""
        from .kin import pose_error

        scored = []
        for rank, q in enumerate(cands):
            ee = self.rm.fk(q)
            pe, re = zip(*(pose_error(targets[s], ee[s]) for s in SIDES))
            hit = max(pe) < pos_tol and max(re) < rot_tol
            n_flags = len(self.checker.evaluate(q)["flags"]) if (self.checker is not None and hit) else 0
            dist = float(np.linalg.norm(q - q_ref))
            scored.append((not hit, n_flags, dist if hit else rank, rank, q))
        return min(scored, key=lambda t: t[:4])[4]


class TrackThenSelect:
    """Warm-started tracking first; global posture-checked search only as recovery.

    A global multi-start search every frame random-walks along the redundancy
    (lift/waist/elbow swivel) even for a fixed target, because nearly-equal
    solutions are re-sampled each call. Tracking stays put; the global search is
    used only when the tracked solution misses the target or is flagged.
    """

    name = "robokit_sel"

    def __init__(self, rm, weights, checker, pos_tol=2e-3, rot_tol=1.0):
        from .kin import pose_error

        self.rm, self.checker, self._pe = rm, checker, pose_error
        self.track = RoboKitIK(rm, weights, tracking=True)
        self.search = RoboKitIK(rm, weights, tracking=False, select_k=8, checker=checker)
        self.pos_tol, self.rot_tol = pos_tol, rot_tol
        self.recoveries = 0

    def solve(self, targets, q_seed, q_prev=None):
        t0 = time.perf_counter()
        res = self.track.solve(targets, q_seed, q_prev)
        ee = self.rm.fk(res.q)
        hit = all(self._pe(targets[s], ee[s])[0] < self.pos_tol and self._pe(targets[s], ee[s])[1] < self.rot_tol for s in SIDES)
        if hit and not self.checker.evaluate(res.q)["flags"]:
            return IKResult(res.q, (time.perf_counter() - t0) * 1e3)
        self.recoveries += 1
        alt = self.search.solve(targets, q_seed, q_prev if q_prev is not None else q_seed)
        return IKResult(alt.q, (time.perf_counter() - t0) * 1e3)


def make_solver(kind: str, rm: ReducedModel, weights: IKWeights | None = None, checker=None, lm_steps: int = 30):
    """casadi | robokit (64 seeds) | robokit_track (1 warm-started candidate)
    | robokit_sel (track; on miss/flag fall back to 64-seed posture-checked pick of 8)
    | robokit_global_sel (the global posture-checked pick on every call)
    | oracle (512 seeds, no regularization)."""
    if kind == "casadi":
        return CasadiIK(rm, weights)
    if kind == "robokit":
        return RoboKitIK(rm, weights, tracking=False)
    if kind in ("robokit_sel", "robokit_global_sel"):
        if checker is None:
            from .quality import PostureChecker
            checker = PostureChecker(rm)
        if kind == "robokit_global_sel":  # global search every call (cold-start benchmark variant)
            return RoboKitIK(rm, weights, tracking=False, select_k=8, checker=checker)
        return TrackThenSelect(rm, weights, checker)
    if kind == "robokit_track":
        return RoboKitIK(rm, weights, tracking=True, lm_steps=lm_steps)
    if kind == "oracle":
        return RoboKitIK(rm, IKWeights(posture=0.0, smooth=0.0), tracking=False, candidates=512)
    raise ValueError(kind)
