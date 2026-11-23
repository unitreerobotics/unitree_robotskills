"""Waist and head joints: friction sweep, parameter fit and stop-accuracy check.

  sweep   move one joint to and fro at constant speeds, record torque (arms keep their pose)
  fit     dry/viscous friction and position dependent holding torque from sweep records
  verify  visit random targets with (or without) the fitted feedforward, report stop errors
"""

import json
import math
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from r1_d.config import MODEL_DIR
from r1_d.sdk.hold import JOINTS, HoldCompensator
from r1_d.sdk.model import GROUPS, HOLD
from r1_d.sdk.protocol import HOLD_GAINS, HOLD_TAU_LIMIT
from r1_d.tools.common import connect, parser, retry_pause, run, save

PARAMS = MODEL_DIR / "hold_joints.json"
# Arm poses (rad). "hang": rest pose beside the body, the waist may turn only +-0.5 rad there.
# "spread": elbows bent, upper arms out; clears the body by 0.08 m for the waist in +-1.05 rad.
ARM_POSES = {
    "hang": {"left_arm": [0.06, 0.10, 0.0, 1.40, 0.0, 0.09, 0.0],
             "right_arm": [0.09, -0.10, 0.0, 1.40, 0.0, -0.21, 0.0]},
    "spread": {"left_arm": [-0.73, 1.07, -0.09, 1.87, 0.0, 0.0, 0.0],
               "right_arm": [-0.73, -1.07, 0.09, 1.87, 0.0, -0.21, 0.0]},
}
MARGINS = {"distal_static": 0.05, "upper_static": 0.015, "hand_self": 0.03, "arm_arm": 0.06}
# Range used here (rad). The waist of this robot has end stops at -0.598 and +0.621 rad
# (measured; the URDF says +-2.618); head inside the URDF limits.
RANGE = {"waist_yaw": (-0.56, 0.58), "head_pitch": (-0.578, 0.578), "head_yaw": (-1.95, 1.95)}
INDEX = dict(zip(JOINTS, HOLD))
# Motor gains and integral tuned on the robot. Stiffness added through the
# 50 Hz feedforward shook the robot (waist 100..200, head yaw 15) and stays 0; the motor
# gains run inside the motor and are quiet: arm joints moved < 1.1 mrad during the tests.
TUNED = {
    "waist_yaw": {"kp": 150.0, "kd": 6.0, "stiffness": 0.0, "damping": 0.0, "ki": 30.0, "integral_limit": 0.5},
    "head_pitch": {"kp": 150.0, "kd": 4.0, "stiffness": 0.0, "damping": 0.0, "ki": 20.0, "integral_limit": 0.3},
    "head_yaw": {"kp": 40.0, "kd": 0.4, "stiffness": 0.0, "damping": 0.0, "ki": 3.0, "integral_limit": 0.08},
}


def load_params(path=PARAMS):
    return json.loads(Path(path).read_text()) if Path(path).is_file() else {}


class Mover:
    """Drives session.hold / session.hold_tau of one joint from the tick callback."""

    def __init__(self, session, joint, params=None):
        self.session, self.joint, self.index = session, joint, INDEX[joint]
        self.target = session.hold[self.index]
        self.velocity = 0.0
        self.comp = HoldCompensator(params) if params else None
        self.rows = []
        self.last = None
        self.goal, self.speed = self.target, 0.0
        # A joint that falls this far behind a moving target has met an end stop or an
        # obstacle: the move ends there instead of pushing on.
        self.stall = {"waist_yaw": 0.035, "head_pitch": 0.035, "head_yaw": 0.05}[joint]
        self.stalled_at = None
        self.period, self.torque_time, self.tau = 0.0, 0.0, 0.0
        self.arm_error, self.arm_joint = 0.0, None  # rad, largest arm deviation: shaking shows here

    def each(self):
        now = time.monotonic()
        dt = 0.004 if self.last is None else min(0.05, now - self.last)
        self.last = now
        step = self.speed * dt
        delta = self.goal - self.target
        if abs(delta) > step and step > 0:
            self.velocity = math.copysign(self.speed, delta)
            self.target += math.copysign(step, delta)
        else:
            self.velocity, self.target = 0.0, self.goal
        motor = self.session.state.motor_state[self.index]
        if self.velocity and abs(self.target - motor.q) > self.stall + 0.15 * self.speed:
            self.stalled_at = float(motor.q)
            self.goal = self.target = float(motor.q)
            self.velocity = 0.0
        # The chess service runs at 50 Hz: with --period the torque is updated at that rate.
        if self.comp and now - self.torque_time >= self.period:
            self.tau = self.comp.torque(
                motor.q, self.target, self.velocity, max(dt, now - self.torque_time if self.torque_time else dt), motor.dq
            )
            self.torque_time = now
        tau = self.tau
        self.session.hold[self.index] = self.target
        self.session.hold_tau[self.index] = tau
        arms = self.session.commanded["q"]
        for name in ("left_arm", "right_arm"):
            for j, (i, q) in enumerate(zip(GROUPS[name], arms[name])):
                error = abs(self.session.state.motor_state[i].q - q)
                if error > self.arm_error:
                    self.arm_error, self.arm_joint = error, f"{name}[{j}]"
        self.rows.append(
            (now, self.target, self.velocity, motor.q, motor.dq, motor.tau_est, tau,
             self.comp.integral if self.comp else 0.0)
        )

    def go(self, goal, speed, settle=0.0):
        low, high = RANGE[self.joint]
        self.goal, self.speed = min(high, max(low, goal)), speed
        while self.target != self.goal:
            self.session.wait(0.2, each=self.each)
        if settle:
            self.session.wait(settle, each=self.each)

    def mean(self, window=0.5):
        end = self.rows[-1][0]
        rows = np.array([r for r in self.rows if r[0] > end - window])
        return rows.mean(axis=0)


def collision_model(session, waist=None):
    from r1_d.sdk.collision import Collision
    from r1_d.sdk.model import urdf

    motors = session.state.motor_state
    return Collision(
        urdf, session.last_height, waist=motors[HOLD[0]].q if waist is None else waist,
        head=(motors[HOLD[1]].q, motors[HOLD[2]].q),
    )


def check_waist_range(session, low, high):
    """The arms, as commanded now, must clear the body over the whole waist range."""
    q = session.commanded["q"]
    for waist in np.linspace(low - 0.05, high + 0.05, 9):
        bad = collision_model(session, waist).violation(
            np.array(q["left_arm"]), np.array(q["right_arm"]), MARGINS
        )
        if bad:
            raise SystemExit(f"腰 {waist:+.2f} rad 时手臂与身体间距不足（{bad}）；先用 arms spread 抬起双臂")


def arms(args):
    robot, session = connect(args)
    try:
        q = session.commanded["q"]
        start = (np.array(q["left_arm"]), np.array(q["right_arm"]))
        pose = ARM_POSES[args.pose]
        end = (np.array(pose["left_arm"]), np.array(pose["right_arm"]))
        bad = collision_model(session).path_violation(start, end, dict(MARGINS, distal_static=0.04))
        if bad:
            raise SystemExit(f"到 {args.pose} 的直线路径间距不足：{bad}")
        from r1_d.tools.common import move

        session.arm_rate = args.rate
        move(session, arms=pose, height=args.height, label="arms " + args.pose)
        session.wait(1.0)
        print("arms", args.pose, session.status()["arm_degrees"], flush=True)
    finally:
        robot.close()


def temperature(session, joint):
    return list(session.state.motor_state[INDEX[joint]].temperature)


def sweep(args):
    robot, session = connect(args)
    try:
        mover = Mover(session, args.joint, None)
        start = mover.target
        low, high = RANGE[args.joint]
        low, high = max(low, -args.amplitude + args.centre), min(high, args.amplitude + args.centre)
        if args.joint == "waist_yaw":
            check_waist_range(session, low, high)
        # Probe the range slowly; an end stop inside it shortens the range by 0.03 rad.
        stops = {}
        for name, goal in (("high", high), ("low", low)):
            mover.stalled_at = None
            mover.go(goal, min(0.05, args.speeds[0]), 0.5)
            if mover.stalled_at is not None:
                stops[name] = mover.stalled_at
                print(f"{args.joint}: 在 {mover.stalled_at:+.4f} rad 处停住（限位或障碍）", flush=True)
        if "high" in stops:
            high = stops["high"] - 0.03
        if "low" in stops:
            low = stops["low"] + 0.03
        mover.go(high, args.speeds[0], 1.0)
        record = {"joint": args.joint, "gains": HOLD_GAINS[JOINTS.index(args.joint)],
                  "range": [low, high], "stops": stops, "start": start, "passes": []}
        for speed in args.speeds:
            for goal in (low, high):
                first = len(mover.rows)
                mover.go(goal, speed, 1.0)
                record["passes"].append({"speed": speed, "goal": goal,
                                         "rows": [list(map(float, r)) for r in mover.rows[first::5]]})
            print(f"{args.joint} speed {speed}: temperature {temperature(session, args.joint)}", flush=True)
            save(args.report, record)
        mover.go(start, args.speeds[-1], 2.0)
        record["end"] = float(mover.mean()[3])
        save(args.report, record)
    except BaseException:
        # Keep what led to the failure.
        record_tail = {"joint": args.joint, "tail": [list(map(float, r)) for r in mover.rows[-1500::5]]}
        save(args.report.with_suffix(".failed.json"), record_tail)
        raise
    finally:
        robot.close()


def fit(args):
    params = load_params(args.params)
    for path in args.records:
        record = json.loads(Path(path).read_text())
        joint = record["joint"]
        q, v, tau = [], [], []
        for p in record["passes"]:
            rows = np.array(p["rows"])
            moving = (np.abs(rows[:, 2]) > 0) & (np.abs(np.abs(rows[:, 4]) - p["speed"]) < 0.5 * p["speed"])
            # drop the first and last 15 % of the travel (acceleration, stop)
            lo, hi = record["range"]
            inner = (rows[:, 3] > lo + 0.15 * (hi - lo)) & (rows[:, 3] < hi - 0.15 * (hi - lo))
            keep = moving & inner
            q += list(rows[keep, 3]); v += list(rows[keep, 4]); tau += list(rows[keep, 5])
        q, v, tau = map(np.array, (q, v, tau))
        columns = [np.sign(v), v, np.ones_like(q)]
        if joint == "head_pitch":
            columns += [np.sin(q), np.cos(q)]
        solution, *_ = np.linalg.lstsq(np.stack(columns, axis=1), tau, rcond=None)
        residual = tau - np.stack(columns, axis=1) @ solution
        bias = [float(solution[2]), 0.0, 0.0]
        if joint == "head_pitch":
            bias[1:] = [float(solution[3]), float(solution[4])]
        limit = HOLD_TAU_LIMIT[JOINTS.index(joint)]
        coulomb = float(abs(solution[0]))
        tuned = TUNED[joint]
        params[joint] = {
            "coulomb": coulomb, "viscous": float(max(0.0, solution[1])), "bias": bias,
            "kp": tuned["kp"], "kd": tuned["kd"],
            "stiffness": tuned["stiffness"], "damping": tuned["damping"],
            "ki": tuned["ki"], "integral_limit": tuned["integral_limit"],
            "deadzone": 0.001, "creep": 0.004, "limit": limit, "fade_seconds": 0.5,
            "fit": {"samples": int(len(q)), "rms_nm": float(np.sqrt(np.mean(residual**2))),
                    "range": record["range"], "stops": record.get("stops", {}),
                    "source": [str(p) for p in args.records]},
        }
        print(joint, json.dumps(params[joint]))
    Path(args.params).write_text(json.dumps(params, indent=1) + "\n")


def verify(args):
    params = load_params(args.params).get(args.joint) if not args.no_feedforward else None
    if not args.no_feedforward and not params:
        raise SystemExit(f"{args.joint}: no parameters in {args.params}")
    rng = np.random.default_rng(args.seed)
    low, high = RANGE[args.joint]
    low, high = max(low, args.centre - args.amplitude), min(high, args.centre + args.amplitude)
    targets = list(rng.uniform(low, high, args.count))
    # Small steps matter most: they are the ones friction swallows.
    for k in range(0, len(targets), 3):
        targets[k] = float(np.clip(targets[k - 1] + rng.choice([-1, 1]) * rng.uniform(0.005, 0.05), low, high)) if k else targets[k]
    targets += list(args.final)
    if params:
        params = dict(params, **{k: float(v) for k, v in args.set})
    report = {"joint": args.joint, "feedforward": params, "period": args.period, "settle": args.settle, "stops": []}
    done, attempt, worst_arm = 0, 0, 0.0
    while done < len(targets):
        robot = None
        try:
            robot, session = connect(args)
            mover = Mover(session, args.joint, params)
            mover.period = args.period
            gains = [list(g) for g in HOLD_GAINS]
            if args.gains:
                gains[JOINTS.index(args.joint)] = args.gains
            elif params and "kp" in params:
                gains[JOINTS.index(args.joint)] = [params["kp"], params["kd"]]
            session.hold_gains = gains
            if args.joint == "waist_yaw":
                check_waist_range(session, low, high)
            while done < len(targets):
                first = len(mover.rows)
                mover.go(float(targets[done]), args.rate, args.settle)
                m = mover.mean()
                if args.trace:
                    report.setdefault("traces", []).append(
                        [list(map(float, r)) for r in mover.rows[first::10]]
                    )
                stop = {"target": float(targets[done]), "q": float(m[3]), "error_mrad": float(1000 * (m[3] - m[1])),
                        "tau_est": float(m[5]), "tau_ff": float(m[6]), "integral": float(m[7]),
                        "moving_rad_s": float(abs(m[4])), "arm_error_mrad": 1000 * mover.arm_error,
                        "arm_joint": mover.arm_joint}
                worst_arm = max(worst_arm, mover.arm_error)
                mover.arm_error = 0.0
                report["stops"].append(stop)
                done += 1
                attempt = 0
                if mover.comp and mover.comp.tripped:
                    report["oscillation_guard_tripped"] = True
                    raise SystemExit(f"{args.joint}: 振荡保护触发，已关闭附加刚度；停止本次验证")
                print(f"{args.joint} {done}/{len(targets)} target {stop['target']:+.4f} error {stop['error_mrad']:+.2f} mrad "
                      f"ff {stop['tau_ff']:+.2f} Nm arm {stop['arm_error_mrad']:.1f} mrad {stop['arm_joint']}", flush=True)
            errors = np.abs([s["error_mrad"] for s in report["stops"]])
            report["oscillation_guard_tripped"] = bool(mover.comp and mover.comp.tripped)
            report["summary"] = {"count": len(errors), "max_mrad": float(errors.max()),
                                 "mean_mrad": float(errors.mean()), "p95_mrad": float(np.percentile(errors, 95)),
                                 "arm_error_mrad": 1000 * worst_arm,
                                 "temperature": temperature(session, args.joint)}
            print(json.dumps(report["summary"]), flush=True)
        except (RuntimeError, TimeoutError) as exc:
            attempt += 1
            if attempt > args.retries:
                raise
            retry_pause(attempt, exc)
        finally:
            if robot is not None:
                robot.close()
            save(args.report, report)


def main():
    p = parser(__doc__, "hold_calib")
    p.add_argument("mode", choices=("sweep", "fit", "verify", "arms"))
    p.add_argument("--pose", choices=tuple(ARM_POSES), default="hang")
    p.add_argument("--joint", choices=JOINTS)
    p.add_argument("--amplitude", type=float, default=10.0, help="rad around --centre, clipped to the range")
    p.add_argument("--centre", type=float, default=0.0)
    p.add_argument("--speeds", type=float, nargs="+", default=[0.05, 0.1, 0.2])
    p.add_argument("--records", type=Path, nargs="*", default=[])
    p.add_argument("--params", type=Path, default=PARAMS)
    p.add_argument("--no-feedforward", action="store_true")
    p.add_argument("--count", type=int, default=30)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--rate", type=float, default=0.2)
    p.add_argument("--settle", type=float, default=3.0)
    p.add_argument("--final", type=float, nargs="*", default=[])
    p.add_argument("--retries", type=int, default=5)
    p.add_argument("--height", type=float, help="arms: lift height to go to (m)")
    p.add_argument("--gains", type=float, nargs=2, metavar=("KP", "KD"), help="motor gains of the joint")
    p.add_argument("--period", type=float, default=0.0, help="s between feedforward updates (0.02 = chess service)")
    p.add_argument("--set", nargs=2, action="append", default=[], metavar=("KEY", "VALUE"), help="override a parameter for this run")
    p.add_argument("--trace", action="store_true", help="keep the 25 Hz time series of every stop")
    args = p.parse_args()
    if any(not 0 < s <= 0.3 for s in args.speeds) or not 0 < args.rate <= 0.3:
        raise SystemExit("speed must be 0..0.3 rad/s")
    {"sweep": sweep, "fit": fit, "verify": verify, "arms": arms}[args.mode](args)


if __name__ == "__main__":
    run(main)
