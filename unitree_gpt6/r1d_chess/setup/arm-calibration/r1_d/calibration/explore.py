"""Whole-workspace arm exploration: plan collision-free waypoints offline, then visit
them on the robot while logging angle, velocity and torque of every arm joint.

plan  writes a waypoint file (both arms, uniform over the joint ranges).
run   executes a plan, reconnecting after dropouts, and resumes where it stopped.
"""

import argparse
import json
from pathlib import Path
import sys
import time
import traceback

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from collision import Collision
from r1_d.config import DEFAULTS
from r1_d.sdk.dynamics import ArmGravity, SIDES
from r1_d.sdk.model import (
    ARM_TAU_LIMIT,
    GROUPS,
    HOLD,
    LIMITS,
    calibrated_urdf,
    json_default,
    urdf,
)

ARM_SLOTS = [i for side in SIDES for i in GROUPS[side + "_arm"]]
LINK_SLOTS = ARM_SLOTS + [GROUPS["left_gripper"][0], GROUPS["right_gripper"][0]]
MARGINS = {"distal_static": 0.06, "upper_static": 0.015, "hand_self": 0.03, "arm_arm": 0.06}
LIMIT_MARGIN = 0.05  # rad inside the joint limits
# The link to the right wrist-yaw and gripper motors drops when the right wrist
# pitch is above about -8 deg; stay in the range that works.
RANGE_OVERRIDE = {("right", 5): (-0.47, -0.15)}
# Hanging rest pose (rad): low torque, used as hub, for cooling and as final pose.
HUB = {
    "left": [0.06, 0.10, 0.0, 1.40, 0.0, 0.09, 0.0],
    "right": [0.09, -0.10, 0.0, 1.40, 0.0, -0.21, 0.0],
}
TEMPERATURE_PAUSE = (72, 95)
TEMPERATURE_RESUME = (62, 80)
# Columns of a recorded row.
T, Q, DQ, TAU, CMD, FF, INDEX, MOVING, LOST = 0, 1, 15, 29, 43, 57, 71, 72, 73


def box(side):
    low, high = LIMITS[side + "_arm"]
    low, high = np.array(low) + LIMIT_MARGIN, np.array(high) - LIMIT_MARGIN
    for (s, j), (a, b) in RANGE_OVERRIDE.items():
        if s == side:
            low[j], high[j] = a, b
    return low, high


def torque_ok(gravity, q_left, q_right, fraction=0.85):
    for side, q in zip(SIDES, (q_left, q_right)):
        if np.any(np.abs(gravity.torques(side, q)) > fraction * np.array(ARM_TAU_LIMIT)):
            return False
    return True


def reachable(collision, gravity, start, end, margins):
    """None if the straight path start->end is acceptable, else the reason."""
    if not torque_ok(gravity, *end):
        return "torque"
    mid = [(np.array(a) + np.array(b)) / 2 for a, b in zip(start, end)]
    if not torque_ok(gravity, *mid):
        return "torque"
    return collision.path_violation(start, end, margins)


def plan(args):
    rng = np.random.default_rng(args.seed)
    collision = Collision(urdf, args.height, waist=args.waist, head=(args.head_pitch, 0.0))
    gravity = ArmGravity(args.gravity_urdf)
    margins = dict(MARGINS, distal_static=args.margin, arm_arm=max(args.margin, MARGINS["arm_arm"]))
    hub = (np.array(HUB["left"]), np.array(HUB["right"]))
    boxes = [box(side) for side in SIDES]
    scale = args.range_scale

    def sample(i, current):
        low, high = boxes[i]
        centre = np.clip(np.array(hub[i]), low, high)
        # range_scale < 1 shrinks the sampled box around the hub pose.
        q = rng.uniform(centre + (low - centre) * scale, centre + (high - centre) * scale)
        if args.local and rng.random() < args.local:
            # Move only a few joints: single-joint excitation with the others loaded.
            keep = rng.random(7) < 0.6
            q = np.where(keep, np.clip(current, low, high), q)
        return q

    current, waypoints, reasons = hub, [], {}
    while len(waypoints) < args.count:
        found = None
        for attempt in range(600):
            target = [sample(0, current[0]), sample(1, current[1])]
            if attempt >= 300:  # move one arm only
                keep = int(rng.integers(2))
                target[keep] = current[keep]
            reason = reachable(collision, gravity, current, target, margins)
            reasons[reason] = reasons.get(reason, 0) + 1
            if reason is None:
                found = target
                break
        if found is None:
            if reachable(collision, gravity, current, hub, dict(margins, distal_static=0.04)):
                raise RuntimeError(f"Stuck at waypoint {len(waypoints)}: {current}")
            found = hub
        waypoints.append([list(map(float, found[0])), list(map(float, found[1]))])
        current = (np.array(found[0]), np.array(found[1]))
        if len(waypoints) % 50 == 0:
            print(len(waypoints), reasons, flush=True)
    record = {
        "seed": args.seed,
        "height": args.height,
        "waist": args.waist,
        "head_pitch": args.head_pitch,
        "margins": margins,
        "gravity_urdf": str(args.gravity_urdf),
        "range_scale": scale,
        "local": args.local,
        "hub": HUB,
        "waypoints": waypoints,
    }
    args.plan.parent.mkdir(parents=True, exist_ok=True)
    args.plan.write_text(json.dumps(record) + "\n")
    q = np.array([w[0] + w[1] for w in waypoints])
    steps = np.abs(np.diff(np.vstack([np.concatenate(hub), q]), axis=0)).max(1)
    print("rejections", reasons)
    print("mean largest joint step (rad)", round(float(steps.mean()), 3))
    print(
        f"estimated minutes at {args.rate} rad/s",
        round(float((np.maximum(2.0, 1.5 * steps / args.rate) + args.dwell + 1.0).sum()) / 60, 1),
    )


class Recorder:
    """Rows of [t, q14, dq14, tau14, q_cmd14, ff14, waypoint, moving, lost mask]."""

    def __init__(self, directory):
        self.directory = directory
        self.rows = []
        self.index = len(list(directory.glob("chunk_*.npz")))

    def add(self, row):
        self.rows.append(row)

    def flush(self):
        if self.rows:
            np.savez_compressed(
                self.directory / f"chunk_{self.index:05d}.npz",
                data=np.array(self.rows, dtype=np.float64),
            )
            self.index += 1
            self.rows = []


def temperatures(state):
    return [list(state.motor_state[i].temperature) for i in ARM_SLOTS]


def hot(state, limits):
    return any(a >= limits[0] or b >= limits[1] for a, b in temperatures(state))


def lost_mask(state):
    return sum(1 << k for k, i in enumerate(LINK_SLOTS) if state.motor_state[i].motorstate)


def run(args):
    from r1_d.sdk import R1D, ControlSession

    plan_record = json.loads(args.plan.read_text())
    waypoints = plan_record["waypoints"]
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    progress_file = out / "progress.json"
    progress = json.loads(progress_file.read_text()) if progress_file.exists() else {"next": 0}
    events = (out / "events.jsonl").open("a", buffering=1)
    statics = (out / "waypoints.jsonl").open("a", buffering=1)
    recorder = Recorder(out)
    # More than one URDF: they take turns per waypoint, to compare them within one run.
    models = [ArmGravity(path) for path in [args.gravity_urdf] + args.alternate_urdf]
    gravity = models[0]
    margins = plan_record["margins"]
    resume_margins = {k: max(0.008, v - 0.02) for k, v in margins.items()}
    deadline = time.time() + args.hours * 3600.0
    hub = (HUB["left"], HUB["right"])
    failures, failed_at = 0, {}

    def event(kind, **fields):
        record = {"time_unix": time.time(), "event": kind, **fields}
        events.write(json.dumps(record, default=json_default) + "\n")
        print(json.dumps(record, default=json_default)[:400], flush=True)

    event("run", plan=str(args.plan), gravity_urdf=str(args.gravity_urdf), start=progress["next"])

    def loop(session, robot, seconds=None, index=-1, guard=args.guard):
        """Tick until the motion completes (or for `seconds`); record fresh states."""
        # Planning and file writes between loops take longer than the tick watchdog allows.
        session.last_tick = None
        start = next_tick = time.monotonic()
        last_stamp, hold_rows = None, []
        while True:
            session.tick()
            stamp = robot.stamps.get("rt/lowstate")
            if stamp != last_stamp and session.arm_pause_started is None:
                last_stamp = stamp
                state = robot.latest["rt/lowstate"]
                q = [state.motor_state[i].q for i in ARM_SLOTS]
                command = session.commanded["q"]["left_arm"] + session.commanded["q"]["right_arm"]
                worst = max(abs(a - b) for a, b in zip(q, command))
                if guard is not None and worst > guard:
                    raise RuntimeError(f"arm tracking error {worst:.3f} rad exceeds guard")
                ff = session.feedforward()
                ff = ff["left_arm"] + ff["right_arm"] if ff else [0.0] * 14
                row = (
                    [time.time()]
                    + q
                    + [state.motor_state[i].dq for i in ARM_SLOTS]
                    + [state.motor_state[i].tau_est for i in ARM_SLOTS]
                    + command
                    + ff
                    + [index, 1.0 if session.motion is not None else 0.0, lost_mask(state)]
                )
                recorder.add(row)
                if session.motion is None:
                    hold_rows.append(row)
            now = time.monotonic()
            if seconds is None and session.motion is None:
                return hold_rows
            if seconds is not None and now - start >= seconds:
                return hold_rows
            if now - start > 180.0:
                raise TimeoutError("Motion did not complete")
            next_tick = max(next_tick + 0.004, time.monotonic())
            time.sleep(max(0.0, next_tick - time.monotonic()))

    def go(session, robot, target, label, index=-1):
        destination = {"q": {k: list(v) for k, v in session.commanded["q"].items()}}
        destination["height"] = session.commanded["height"]
        destination["q"]["left_arm"], destination["q"]["right_arm"] = list(target[0]), list(target[1])
        session.move_to(destination, label=label)
        session.history = []  # the explorer returns to the hub itself
        loop(session, robot, index=index)

    def blocked(session, start, end):
        """Collision check of a path while the session is idle (it takes longer than a tick)."""
        bad = collision.path_violation(start, end, resume_margins)
        session.last_tick = None
        return bad

    def cool(session, robot):
        state = robot.latest["rt/lowstate"]
        if not hot(state, TEMPERATURE_PAUSE):
            return
        event("cooling", temperatures=temperatures(state))
        current = (session.commanded["q"]["left_arm"], session.commanded["q"]["right_arm"])
        if not blocked(session, current, hub):
            go(session, robot, hub, "cool")
        waited = time.monotonic()
        while hot(robot.latest["rt/lowstate"], TEMPERATURE_RESUME):
            loop(session, robot, seconds=10.0)
            if time.monotonic() - waited > 1800:
                break
        event(
            "cooled",
            seconds=time.monotonic() - waited,
            temperatures=temperatures(robot.latest["rt/lowstate"]),
        )

    def restore_link(session, robot):
        """Bring the right wrist pitch back to where its downstream motors answer."""
        rows = loop(session, robot, seconds=1.0, guard=None)
        if not any(row[LOST] for row in rows):
            return
        event("link_lost", mask=int(rows[-1][LOST]))
        target = (
            list(session.commanded["q"]["left_arm"]),
            list(session.commanded["q"]["right_arm"]),
        )
        target[1][5] = HUB["right"][5]
        go(session, robot, target, "restore link")
        rows = loop(session, robot, seconds=2.0, guard=None)
        if any(row[LOST] for row in rows[len(rows) // 2 :]):
            raise RuntimeError("motor link did not come back")
        event("link_restored")

    collision = None
    done = False
    while not done and time.time() < deadline:
        robot = None
        try:
            robot = R1D(args.ip, args.interface, transport=args.transport, multicast=True)
            robot.link_seconds = 1e9  # until restore_link has run
            robot.inspect(1.0)
            robot.prepare_control()
            state = robot.arm_state()
            height = robot.height()
            waist, head_pitch = state.motor_state[HOLD[0]].q, state.motor_state[HOLD[1]].q
            if (
                abs(height - plan_record["height"]) > 0.02
                or abs(waist - plan_record["waist"]) > 0.05
                or abs(head_pitch - plan_record["head_pitch"]) > 0.1
            ):
                raise SystemExit(f"Plan was made for another posture: {height} {waist} {head_pitch}")
            if collision is None:
                collision = Collision(
                    urdf, height, waist=waist, head=(head_pitch, state.motor_state[HOLD[2]].q)
                )
            session = ControlSession(robot, log=lambda e: None, gravity=gravity)
            session.arm_rate = args.rate
            session.friction_share = args.shares[progress["next"] % len(args.shares)]
            restore_link(session, robot)
            robot.link_seconds = 3.0
            measured = (session.commanded["q"]["left_arm"], session.commanded["q"]["right_arm"])
            target = waypoints[progress["next"]] if progress["next"] < len(waypoints) else hub
            bad = blocked(session, measured, target)
            via_hub = False
            if bad:
                bad_hub = blocked(session, measured, hub) or blocked(session, hub, target)
                if bad_hub:
                    raise SystemExit(
                        f"No checked path from the current pose: {bad}, {bad_hub}; pose {measured}"
                    )
                via_hub = True
            event(
                "connected",
                next=progress["next"],
                via_hub=via_hub,
                temperatures=temperatures(state),
            )
            if via_hub:
                go(session, robot, hub, "hub")
            while progress["next"] < len(waypoints) and time.time() < deadline:
                index = progress["next"]
                # Alternating shares compare friction feedforward settings within one run.
                session.friction_share = args.shares[index % len(args.shares)]
                session.gravity = models[index // len(args.shares) % len(models)]
                session.feedforward_cache = (None, None)
                go(session, robot, waypoints[index], f"wp{index}", index=index)
                rows = loop(session, robot, seconds=args.dwell, index=index)
                failures = 0
                if rows:
                    data = np.array(rows[-max(1, len(rows) // 2) :])
                    state = robot.latest["rt/lowstate"]
                    statics.write(
                        json.dumps(
                            {
                                "index": index,
                                "time_unix": time.time(),
                                "q": data[:, Q:DQ].mean(0).tolist(),
                                "q_std": data[:, Q:DQ].std(0).tolist(),
                                "dq_std": data[:, DQ:TAU].std(0).tolist(),
                                "friction_share": session.friction_share,
                                "model": session.gravity.path.name,
                                "tau_est": data[:, TAU:CMD].mean(0).tolist(),
                                "commanded": data[-1, CMD:FF].tolist(),
                                "feedforward": data[-1, FF:INDEX].tolist(),
                                "temperature": temperatures(state),
                                "lost": int(max(row[LOST] for row in rows)),
                                "samples": len(data),
                            }
                        )
                        + "\n"
                    )
                    if index % 10 == 9:
                        error = np.degrees(np.abs(data[:, Q:DQ].mean(0) - data[-1, CMD:FF]))
                        print(
                            f"wp {index} static error deg max {error.max():.2f} "
                            f"mean {error.mean():.2f}",
                            flush=True,
                        )
                progress["next"] = index + 1
                progress_file.write_text(json.dumps(progress))
                if index % 10 == 9:
                    recorder.flush()
                cool(session, robot)
            # Finished (or out of time): back to the hanging pose.
            current = (session.commanded["q"]["left_arm"], session.commanded["q"]["right_arm"])
            if blocked(session, current, hub):
                event("hub_blocked")
            else:
                go(session, robot, hub, "final hub")
                loop(session, robot, seconds=1.0)
            done = True
            event("finished", next=progress["next"])
        except KeyboardInterrupt:
            event("interrupted")
            raise
        except SystemExit as exc:
            event("abort", error=str(exc))
            raise
        except Exception as exc:
            failures += 1
            index = progress["next"]
            failed_at[index] = failed_at.get(index, 0) + 1
            event(
                "error",
                error=repr(exc)[:300],
                trace=traceback.format_exc()[-500:],
                failures=failures,
                next=index,
            )
            tracking = "exceeds guard" in repr(exc) or "跟踪误差" in repr(exc)
            if (tracking or failed_at[index] >= 2) and index < len(waypoints):
                # Do not push on: skip a waypoint that could not be reached.
                event("skip", index=index)
                progress["next"] = index + 1
                progress_file.write_text(json.dumps(progress))
        finally:
            recorder.flush()
            if robot is not None:
                try:
                    robot.close()
                except Exception as exc:
                    event("close_error", error=repr(exc))
        if not done:
            if failures > args.max_failures:
                event("giving_up", failures=failures)
                raise SystemExit(2)
            time.sleep(min(300.0, 15.0 * failures))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = p.add_subparsers(dest="command", required=True)
    s = sub.add_parser("plan")
    s.add_argument("plan", type=Path)
    s.add_argument("--count", type=int, default=300)
    s.add_argument("--seed", type=int, default=1)
    s.add_argument("--height", type=float, required=True)
    s.add_argument("--waist", type=float, default=0.0)
    s.add_argument("--head-pitch", type=float, default=0.0)
    s.add_argument("--margin", type=float, default=MARGINS["distal_static"])
    s.add_argument("--range-scale", type=float, default=1.0)
    s.add_argument("--local", type=float, default=0.0, help="fraction of few-joint moves")
    s.add_argument("--gravity-urdf", type=Path, default=calibrated_urdf)
    s.add_argument("--rate", type=float, default=0.3)
    s.add_argument("--dwell", type=float, default=0.6)
    s = sub.add_parser("run")
    s.add_argument("plan", type=Path)
    s.add_argument("out", type=Path)
    s.add_argument("--ip", default=DEFAULTS["ip"])
    s.add_argument("--interface", default=DEFAULTS["interface"])
    s.add_argument("--transport", default=DEFAULTS["transport"])
    s.add_argument("--gravity-urdf", type=Path, default=calibrated_urdf)
    s.add_argument("--rate", type=float, default=0.3)
    s.add_argument("--dwell", type=float, default=0.6)
    s.add_argument("--guard", type=float, default=0.12)
    s.add_argument("--shares", type=float, nargs="+", default=[1.0], help="friction feedforward")
    s.add_argument("--alternate-urdf", type=Path, nargs="*", default=[], help="A/B per waypoint")
    s.add_argument("--hours", type=float, default=2.0)
    s.add_argument("--max-failures", type=int, default=40)
    args = p.parse_args()
    {"plan": plan, "run": run}[args.command](args)


if __name__ == "__main__":
    main()
