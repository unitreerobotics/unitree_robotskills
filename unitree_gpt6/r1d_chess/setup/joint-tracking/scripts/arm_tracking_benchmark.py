#!/usr/bin/env python3
"""Dual-arm joint tracking benchmark: chess adapter path vs direct direct IPC.

Subcommands
  snapshot  read-only DDS lowstate snapshot (no Runtime, no commands)
  plan      offline: build a small all-joint trajectory around a measured start
            pose and check joint limits, table clearance and arm separation
  run       MOVES THE ROBOT (requires --confirm-motion): stream one trajectory
            through either path and record feedback, sent commands and LowCmd
  report    offline: per-joint/TCP metrics and a side-by-side comparison

Paths
  adapter   r1d_transport.InternalDDS exactly as the chess service uses it:
            100 Hz command_arm() updates, 50 Hz publish loop, 0.3 rad/s arm
            clip, saved head/waist reference, torque feedforward from config.
  direct  R1DRuntimeClient streamed at the Runtime's 250 Hz like direct
            teleop/replay: q_des every tick, head/waist/grippers held at the
            measured start, gravity feedforward at the command (as
            XRBodyRetargeter uses PinocchioGravityCompensator(command_qpos)).
Both paths use the same TorqueFeedforward limits/rate so the comparison
isolates the transport path; --ff selects the feedforward law.
"""
import argparse
import hashlib
import json
import os
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2])); import _common  # noqa: E402  (adds src/ to sys.path)
ROOT = _common.ROOT
from r1d_chess.robot.r1d_config import load_profile, MODEL_PATH
from r1d_chess.robot.r1d_torque_ff import MODES, TorqueFeedforward, load_config as load_ff, with_mode

ARM = list(range(15, 29))
GRIP = [31, 33]
HEAD_WAIST = [29, 30, 13]
JOINTS = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')
MIRROR = np.array([1, -1, -1, 1, -1, 1, -1], float)   # right-arm sign for a mirrored motion
DEFAULT_AMPLITUDE = [.06, .06, .10, .10, .15, .15, .15]
ABORT_TRACKING_RAD = .15
MAX_TEMPERATURE_C = 75
RUNTIME_CONFIG = json.loads((ROOT / 'config/runtime.json').read_text())


def jdump(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False))
    os.replace(tmp, path)


# ---------------------------------------------------------------- trajectory

def quintic(s):
    s = np.clip(s, 0., 1.)
    return s**3 * (10 - 15*s + 6*s*s)


class Trajectory:
    """Quintic moves between keyframes, each followed by a dwell window.

    Keyframes: start -> raised center -> center+E -> center -> center-E ->
    center -> start, so every joint moves in both directions around a pose
    with table clearance, and the first/last legs lift/lower against gravity.
    """

    def __init__(self, spec):
        self.spec = spec
        self.keys = [np.asarray(k['q'], float) for k in spec['keyframes']]
        self.names = [k['name'] for k in spec['keyframes']]
        self.start = self.keys[0]
        self.move, self.dwell, self.hold = spec['move_s'], spec['dwell_s'], spec['hold_s']
        self.approach = spec.get('approach_s', self.move)
        self.segments = []
        t = self.hold
        last = len(self.keys) - 2
        for i in range(len(self.keys)-1):
            # Legs to/from the start or anchor are long lifts; excursions use move_s.
            touches_start = {self.names[i], self.names[i+1]} & {'start', 'measured'}
            move = self.approach if touches_start else self.move
            self.segments.append(('move', t, t+move, i, i+1))
            t += move
            if i != last:
                self.segments.append(('dwell', t, t+self.dwell, i+1, i+1))
                t += self.dwell
        self.duration = t + self.hold

    def __call__(self, t):
        for kind, t0, t1, a, b in self.segments:
            if t0 <= t < t1:
                if kind == 'dwell':
                    return self.keys[b].copy()
                return self.keys[a] + (self.keys[b]-self.keys[a])*quintic((t-t0)/(t1-t0))
        return (self.keys[0] if t < self.hold else self.keys[-1]).copy()

    def windows(self):
        """Static windows: second half of each dwell plus both holds."""
        out = [('pre_hold', self.hold*.5, self.hold, 0)]
        for kind, t0, t1, a, b in self.segments:
            if kind == 'dwell':
                out.append((f'dwell{b}_{self.names[b]}', t0+(t1-t0)*.5, t1, b))
        out.append(('post_hold', self.duration-self.hold*.5, self.duration, len(self.keys)-1))
        return out


def read_snapshot(seconds=1.):
    from r1d_chess.robot.r1d_runtime import load_experiment
    load_experiment()
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
    box = {}
    ChannelFactoryInitialize(0, RUNTIME_CONFIG['network_interface'])
    s = ChannelSubscriber('rt/lowstate', LowState_)
    s.Init(lambda m: box.__setitem__('state', m), 1)
    c = ChannelSubscriber('rt/lowcmd', LowCmd_)
    c.Init(lambda m: box.__setitem__('cmd', m), 1)
    time.sleep(seconds)
    m = box.get('state')
    if m is None:
        raise RuntimeError('no rt/lowstate received')
    motors = m.motor_state
    return {'captured_at': datetime.now().astimezone().isoformat(), 'tick': int(m.tick),
            'q': [float(x.q) for x in motors], 'dq': [float(x.dq) for x in motors],
            'tau_est': [float(x.tau_est) for x in motors],
            'temperature': [int(max(x.temperature)) for x in motors],
            'lowcmd_publisher_seen': 'cmd' in box, 'read_only': True, 'sent_commands': 0}


def raised_center(kin, q, raise_m, amplitude=None, margin=.03):
    """Per arm: shoulder pitch, elbow and wrist pitch offsets lifting the TCP by raise_m
    while keeping its xy and orientation (least squares on the R1-D URDF)."""
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    center = q.copy()
    for k, arm in enumerate(kin.arms):
        q7 = q[7*k:7*k+7]
        p0 = arm.fk(q7)

        def residual(x):
            qq = q7.copy()
            qq[[0, 3, 5]] += x
            T = arm.fk(qq)
            rot = Rotation.from_matrix(T[:3, :3] @ p0[:3, :3].T).as_rotvec()
            return np.r_[T[:3, 3]-p0[:3, 3]-[0, 0, raise_m], .05*rot]
        # Keep center +/- excursion amplitude inside the model joint limits.
        idx = [0, 3, 5]
        amp = np.zeros(7) if amplitude is None else np.asarray(amplitude, float)
        lo = arm.lower[idx] + margin + amp[idx] - q7[idx]
        hi = arm.upper[idx] - margin - amp[idx] - q7[idx]
        lo, hi = np.minimum(lo, 0.), np.maximum(hi, 0.)
        x = least_squares(residual, np.zeros(3), bounds=(lo - 1e-9, hi + 1e-9)).x
        center[7*k:7*k+7][[0, 3, 5]] += x
    return center


def plan(args):
    import itertools
    from r1d_chess.robot.r1d_kinematics import R1DKinematics
    from r1d_chess.robot.board_guard import BoardGuard
    profile = load_profile(args.profile)
    snapshot = json.loads(Path(args.start).read_text())
    measured = np.asarray(snapshot['q'], float)[ARM]
    # A fixed anchor makes repeated runs comparable: measured -> anchor -> ... -> anchor.
    start = (np.asarray(json.loads(Path(args.anchor).read_text())['q'], float)[ARM]
             if args.anchor else measured)
    if np.max(np.abs(start-measured)) > args.max_anchor_distance:
        sys.exit(f'anchor differs from measured pose by {np.max(np.abs(start-measured)):.3f} rad '
                 f'(> {args.max_anchor_distance})')
    kin = R1DKinematics(profile)
    if profile.get('scene') is None and not args.no_scene_ok:
        sys.exit('profile has no scene: table clearance cannot be checked. Clear the area under both arms and pass --no-scene-ok')
    guard = BoardGuard.from_dict(profile['scene']) if profile.get('scene') else None

    class _NoGuard:
        @staticmethod
        def clearance(_pose):
            return 1.0   # unknown table: report a nominal 1 m clearance, never a violation
    guard = guard or _NoGuard()
    amplitude = np.asarray(args.amplitudes or DEFAULT_AMPLITUDE, float) * args.scale
    center = raised_center(kin, start, args.raise_m, amplitude, args.limit_margin)

    def excursion(signs):
        return np.r_[amplitude*signs, amplitude*signs*MIRROR]

    if args.first_sign:
        signs = np.asarray(args.first_sign, float)
    else:
        # Pick the first-direction pattern with the largest worst-case tool clearance.
        best = None
        for pattern in itertools.product((1., -1.), repeat=7):
            e = excursion(np.array(pattern))
            worst = min(guard.clearance(pose) for lvl in (-1, -.5, .5, 1)
                        for pose in kin.fk(center + e*lvl))
            if best is None or worst > best[0]:
                best = (worst, np.array(pattern))
        signs = best[1]
    e = excursion(signs)
    keyframes = [('start', start), ('center', center), ('plus', center+e), ('center', center),
                 ('minus', center-e), ('center', center), ('start', start)]
    if args.anchor:
        keyframes.insert(0, ('measured', measured))
    spec = {'schema_version': 2, 'created_at': datetime.now().astimezone().isoformat(),
            'start_snapshot': str(Path(args.start).resolve()), 'start_q': measured.tolist(),
            'anchor': str(Path(args.anchor).resolve()) if args.anchor else None, 'anchor_q': start.tolist(),
            'keyframes': [{'name': n, 'q': k.tolist()} for n, k in keyframes],
            'raise_m': args.raise_m, 'amplitude_rad': amplitude.tolist(), 'first_sign': signs.tolist(),
            'move_s': args.move_s, 'approach_s': args.approach_s, 'dwell_s': args.dwell_s,
            'hold_s': args.hold_s, 'joint_order': 'physical left 7 then right 7 (motors 15..28)',
            'profile': str(Path(args.profile).resolve()) if args.profile else 'config/robot.json',
            'urdf': str(MODEL_PATH)}
    traj = Trajectory(spec)
    lower = np.r_[kin.arms[0].lower, kin.arms[1].lower] + args.limit_margin
    upper = np.r_[kin.arms[0].upper, kin.arms[1].upper] - args.limit_margin
    times = np.arange(0., traj.duration, .01)
    samples = np.array([traj(t) for t in times])
    violations = []
    if np.any(samples < lower) or np.any(samples > upper):
        bad = np.unique(np.where((samples < lower) | (samples > upper))[1])
        violations.append(f'joint limit margin violated: motors {[ARM[i] for i in bad]}')
    clear = {'left': [], 'right': []}
    tcp = {'left': [], 'right': []}
    distance = []
    for row in samples[::5]:
        poses = kin.fk(row)
        for side, pose in zip(('left', 'right'), poses):
            clear[side].append(guard.clearance(pose))
            tcp[side].append(pose[:3, 3])
        distance.append(float(np.linalg.norm(poses[0][:3, 3]-poses[1][:3, 3])))
    sampled = times[::5]
    moves = [seg for seg in traj.segments if seg[0] == 'move']
    into_center = next(seg for seg in moves if traj.names[seg[4]] == 'center')
    out_of_center = [seg for seg in moves if traj.names[seg[3]] == 'center'][-1]
    excursion_mask = (sampled >= into_center[2]) & (sampled < out_of_center[1])
    report = {}
    for side in ('left', 'right'):
        p = np.array(tcp[side])
        c = np.array(clear[side])
        report[side] = {'start_tcp_m': p[0].tolist(), 'tcp_range_m': (p.max(0)-p.min(0)).tolist(),
                        'max_tcp_displacement_m': float(np.max(np.linalg.norm(p-p[0], axis=1))),
                        'start_tool_clearance_m': float(c[0]), 'min_tool_clearance_m': float(c.min()),
                        'min_tool_clearance_excursion_m': float(c[excursion_mask].min()),
                        'min_y_m': float(p[:, 1].min()), 'max_y_m': float(p[:, 1].max())}
        # Start/end may be the measured resting pose; elsewhere require margin and
        # never go below the starting clearance.
        need = min(args.min_clearance, float(c[0]))
        if c.min() < need - 1e-6:
            violations.append(f'{side} tool clearance {c.min():.3f} m < {need:.3f} m')
        if report[side]['min_tool_clearance_excursion_m'] < args.min_clearance:
            violations.append(f'{side} clearance during excursions '
                              f'{report[side]["min_tool_clearance_excursion_m"]:.3f} m < {args.min_clearance} m')
    if report['left']['min_y_m'] < .1 or report['right']['max_y_m'] > -.1:
        violations.append('a wrist crosses toward the midline (|y| < 0.10 m)')
    if min(distance) < args.min_arm_distance:
        violations.append(f'wrist distance {min(distance):.3f} m < {args.min_arm_distance} m')
    speed = np.abs(np.diff(samples, axis=0)).max(0) / .01
    if speed.max() > args.max_speed:
        violations.append(f'peak joint speed {speed.max():.3f} rad/s > {args.max_speed}')
    spec['checks'] = {'violations': violations, 'arms': report, 'min_wrist_distance_m': min(distance),
                      'peak_joint_speed_rad_s': speed.tolist(), 'duration_s': traj.duration,
                      'joint_travel_rad': (samples.max(0)-samples.min(0)).tolist(),
                      'scene': profile.get('scene'), 'min_clearance_required_m': args.min_clearance}
    spec['sha256'] = hashlib.sha256(json.dumps(spec['keyframes'], sort_keys=True).encode()).hexdigest()
    jdump(args.output, spec)
    print(json.dumps({'output': args.output, 'duration_s': traj.duration, 'violations': violations,
                      'first_sign': signs.tolist(), 'arms': report,
                      'joint_travel_rad': np.round(spec['checks']['joint_travel_rad'], 3).tolist(),
                      'peak_joint_speed_rad_s': np.round(speed, 3).tolist()}, indent=2))
    if violations:
        sys.exit(2)


# ---------------------------------------------------------------- recording

class Recorder:
    def __init__(self):
        self.rows = []
        self.events = []

    def add(self, t, q_des, q_sent, msg, lowcmd, qtau, terms=None):
        motors = msg.motor_state
        row = {'t': t, 'q_des': q_des.tolist(), 'q_sent': np.asarray(q_sent).tolist(),
               'q': [float(motors[i].q) for i in ARM], 'dq': [float(motors[i].dq) for i in ARM],
               'tau_est': [float(motors[i].tau_est) for i in ARM],
               'temp': [int(max(motors[i].temperature)) for i in ARM],
               'qtau': np.asarray(qtau).tolist(), 'tick': int(msg.tick)}
        if lowcmd is not None:
            row['wire_q'] = [float(lowcmd.motor_cmd[i].q) for i in ARM]
            row['wire_tau'] = [float(lowcmd.motor_cmd[i].tau) for i in ARM]
            row['wire_kp'] = [float(lowcmd.motor_cmd[i].kp) for i in ARM]
        if terms is not None:
            row['ff_terms'] = {k: np.asarray(v).tolist() for k, v in terms.items()}
        self.rows.append(row)

    def event(self, name, **data):
        self.events.append({'t_wall': datetime.now().astimezone().isoformat(), 'event': name, **data})


def health(msg, q_sent):
    motors = msg.motor_state
    q = np.array([motors[i].q for i in ARM])
    error = q - np.asarray(q_sent)
    if np.max(np.abs(error)) > ABORT_TRACKING_RAD:
        worst = int(np.argmax(np.abs(error)))
        raise RuntimeError(f'tracking error {error[worst]:+.3f} rad on motor {ARM[worst]} exceeds abort limit')
    for i in ARM + GRIP + HEAD_WAIST:
        if motors[i].motorstate or max(motors[i].temperature) > MAX_TEMPERATURE_C:
            raise RuntimeError(f'motor health/temperature fault: {i}')


def preflight(traj, q, dq, lowcmd_age, client_state):
    if client_state.control_active or client_state.runtime_error or client_state.reset_pose_active:
        raise RuntimeError('Runtime busy/error; refuse to take control')
    if client_state.thermal_protection_active:
        raise RuntimeError('Runtime thermal protection active')
    if lowcmd_age is not None and lowcmd_age < .5:
        raise RuntimeError('another LowCmd publisher is active')
    if np.max(np.abs(dq[ARM + GRIP + HEAD_WAIST])) > .1:
        raise RuntimeError('joints moving before benchmark')
    start_error = np.max(np.abs(q[ARM] - traj.start))
    if start_error > .03:
        raise RuntimeError(f'measured arms differ from trajectory start by {start_error:.3f} rad; re-plan from a fresh snapshot')


class Stop(Exception):
    pass


def run_direct(traj, ff, rec, stop):
    from r1d_chess.robot.r1d_runtime import load_experiment, R1DRuntimeClient
    load_experiment()
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
    box = {'cmd_at': None}
    lock = threading.Lock()

    def on_state(m):
        with lock:
            box['state'], box['state_at'] = m, time.monotonic()

    def on_cmd(m):
        with lock:
            box['cmd'], box['cmd_at'] = m, time.monotonic()

    ChannelFactoryInitialize(0, RUNTIME_CONFIG['network_interface'])
    subs = [ChannelSubscriber('rt/lowstate', LowState_), ChannelSubscriber('rt/lowcmd', LowCmd_)]
    subs[0].Init(on_state, 1)
    subs[1].Init(on_cmd, 1)
    time.sleep(1.)

    def feedback():
        with lock:
            m, at, cmd = box.get('state'), box.get('state_at', 0.), box.get('cmd')
        if m is None or time.monotonic() - at > .15:
            raise RuntimeError('lowstate missing/stale')
        q = np.array([x.q for x in m.motor_state])
        dq = np.array([x.dq for x in m.motor_state])
        return q, dq, m, cmd

    client = R1DRuntimeClient()
    client.connect()
    q, dq, msg, _ = feedback()
    preflight(traj, q, dq, None if box['cmd_at'] is None else time.monotonic()-box['cmd_at'], client.state())
    hold = {'head': tuple(q[[29, 30]]), 'waist': (float(q[13]),),
            'left_ee': (float(q[31]),), 'right_ee': (float(q[33]),)}
    period = 1. / 250.
    client.begin()
    rec.event('session_started', path='direct', hold=hold)
    q_sent = q[ARM].copy()
    previous = q_sent.copy()
    qtau = np.zeros(14)

    def send(q_arm, tau):
        client.send_joint_targets({'left_arm': tuple(q_arm[:7]), 'right_arm': tuple(q_arm[7:]), **hold},
                                  {'left_arm': tuple(tau[:7]), 'right_arm': tuple(tau[7:])})

    t0 = time.monotonic()
    try:
        while True:
            started = time.monotonic()
            t = started - t0
            if t >= traj.duration:
                break
            if stop.is_set():
                raise Stop('operator stop')
            q, dq, msg, cmd = feedback()
            health(msg, q_sent)
            q_sent = traj(t)
            qtau = ff.compute(q[ARM], q_sent, (q_sent-previous)/period, period)
            previous = q_sent.copy()
            send(q_sent, qtau)
            rec.add(t, q_sent, q_sent, msg, cmd, qtau, ff.last_terms)
            time.sleep(max(0., period-(time.monotonic()-started)))
        rec.event('trajectory_complete')
    except Exception as exc:
        rec.event('aborted', error=str(exc))
        q, _, _, _ = feedback()
        q_sent = q[ARM].copy()
        raise
    finally:
        # Ramp feedforward to zero holding the final/measured target, then end.
        try:
            tau = qtau.copy()
            while np.any(tau):
                tau = tau - np.clip(tau, -30*.02, 30*.02)
                send(q_sent, tau)
                time.sleep(.02)
        finally:
            client.end()
            client.close()
            rec.event('session_ended', path='direct')
            for sub in subs:
                sub.Close()


def run_adapter(traj, ff_config, rec, stop):
    from r1d_chess.robot.r1d_runtime import R1DRuntimeClient
    from r1d_chess.robot.r1d_transport import InternalDDS
    dds = InternalDDS(RUNTIME_CONFIG['network_interface'], allow_motion=True, torque_ff_config=ff_config)
    try:
        q, dq, _, _ = dds.state()
        with R1DRuntimeClient() as probe:
            state = probe.state()
        lowcmd_age = None if dds._last_lowcmd is None else time.monotonic()-dds._last_command_at
        preflight(traj, q, dq, lowcmd_age, state)
        dds.enable()
        rec.event('session_started', path='adapter', head_waist_reference=dds.head_waist_sent.tolist())
        period = .01
        t0 = time.monotonic()
        try:
            while True:
                started = time.monotonic()
                t = started - t0
                if t >= traj.duration:
                    break
                if stop.is_set():
                    raise Stop('operator stop')
                if dds.fault:
                    raise RuntimeError(f'adapter fault: {dds.fault}')
                q_des = traj(t)
                dds.command_arm(q_des, np.zeros(14))
                _, _, msg, _ = dds.state()
                with dds.lock:
                    q_sent, qtau = dds.arm_sent.copy(), dds.arm_tau_sent.copy()
                    terms = dict(dds.torque_ff.last_terms)
                rec.add(t, q_des, q_sent, msg, dds._last_lowcmd, qtau, terms)
                time.sleep(max(0., period-(time.monotonic()-started)))
            rec.event('trajectory_complete')
        except Exception as exc:
            rec.event('aborted', error=str(exc), adapter_fault=dds.fault,
                      tracking_fault_snapshot=dds.tracking_fault_snapshot)
            if not dds.fault:
                dds.hold()
            raise
        finally:
            if dds.session_active:
                deadline = time.monotonic() + 3.
                while time.monotonic() < deadline and np.max(np.abs(dds.arm_target-dds.arm_sent)) > 1e-4:
                    time.sleep(.02)
                dds.release()
                dds.release_waist()
            rec.event('session_ended', path='adapter', fault=dds.fault,
                      tau_ramp_out_error=dds.tau_ramp_out_error)
    finally:
        dds.close(unload_arms=False)   # keep holding: with no table the arms would drop from init


def run(args):
    if not args.confirm_motion:
        sys.exit('refusing: run moves both arms; pass --confirm-motion after checking the workspace')
    spec = json.loads(Path(args.trajectory).read_text())
    if spec['checks']['violations']:
        sys.exit(f'refusing: trajectory has violations {spec["checks"]["violations"]}')
    traj = Trajectory(spec)
    base = load_ff(args.ff_config)
    ff_config = with_mode(base, args.ff)
    out = Path(args.output_dir) / f'{datetime.now():%Y%m%d_%H%M%S}_{args.path}_{args.ff}'
    out.mkdir(parents=True, exist_ok=False)
    jdump(out / 'trajectory.json', spec)
    rec = Recorder()
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    meta = {'path': args.path, 'ff': args.ff, 'ff_config': ff_config, 'trajectory': str((out / 'trajectory.json').resolve()),
            'trajectory_sha256': spec['sha256'], 'profile': os.environ.get('R1D_CHESS_PROFILE', 'config/robot.json'),
            'model_urdf': str(MODEL_PATH),
            'started_at': datetime.now().astimezone().isoformat()}
    error = None
    try:
        if args.path == 'direct':
            run_direct(traj, TorqueFeedforward(ff_config), rec, stop)
        else:
            run_adapter(traj, ff_config, rec, stop)
    except Exception as exc:
        error = f'{type(exc).__name__}: {exc}'
    meta.update(finished_at=datetime.now().astimezone().isoformat(), error=error,
                completed=error is None, samples=len(rec.rows))
    jdump(out / 'meta.json', meta)
    jdump(out / 'events.json', rec.events)
    jdump(out / 'samples.json', rec.rows)
    summary = summarize(out)
    print(json.dumps({'output_dir': str(out), 'error': error, 'summary': summary['overall']}, indent=2))
    if error:
        sys.exit(1)


# ---------------------------------------------------------------- analysis

def summarize(directory):
    from r1d_chess.robot.r1d_kinematics import R1DKinematics
    directory = Path(directory)
    meta = json.loads((directory / 'meta.json').read_text())
    rows = json.loads((directory / 'samples.json').read_text())
    spec = json.loads(Path(meta['trajectory']).read_text())
    traj = Trajectory(spec)
    if not rows:
        summary = {'meta': meta, 'overall': {'samples': 0}}
        jdump(directory / 'summary.json', summary)
        return summary
    t = np.array([r['t'] for r in rows])
    q_des = np.array([r['q_des'] for r in rows])
    q_sent = np.array([r['q_sent'] for r in rows])
    q = np.array([r['q'] for r in rows])
    qtau = np.array([r['qtau'] for r in rows])
    tau_est = np.array([r['tau_est'] for r in rows])
    err = q_des - q
    # Archived runs may name a profile that no longer exists; FK here only needs the TCP.
    profile_path = spec['profile'] if spec['profile'] != 'config/robot.json' and Path(spec['profile']).exists() else None
    kin = R1DKinematics(load_profile(profile_path))
    tcp = []
    for a, b in zip(q_des[::5], q[::5]):
        pa, pb = kin.fk(a), kin.fk(b)
        tcp.append([float(np.linalg.norm(pa[k][:3, 3]-pb[k][:3, 3])) for k in (0, 1)])
    tcp = np.array(tcp)
    windows = {}
    for name, t0, t1, _ in traj.windows():
        mask = (t >= t0) & (t < t1)
        if mask.any():
            windows[name] = {'mean_error_rad': err[mask].mean(0).tolist(),
                             'tau_load_nm_est': tau_est[mask].mean(0).tolist(),
                             'qtau_nm': qtau[mask].mean(0).tolist()}
    joints = []
    for j in range(14):
        joints.append({'motor': ARM[j], 'joint': ('left_' if j < 7 else 'right_') + JOINTS[j % 7],
                       'rms_error_rad': float(np.sqrt(np.mean(err[:, j]**2))),
                       'max_abs_error_rad': float(np.max(np.abs(err[:, j]))),
                       'rms_command_lag_rad': float(np.sqrt(np.mean((q_des[:, j]-q_sent[:, j])**2))),
                       'static_error_rad': {k: v['mean_error_rad'][j] for k, v in windows.items()},
                       'max_abs_qtau_nm': float(np.max(np.abs(qtau[:, j])))})
    wire = None
    if 'wire_tau' in rows[-1]:
        wired = [r for r in rows if 'wire_tau' in r]
        wire = {'mean_abs_wire_tau_minus_qtau_nm': float(np.mean(np.abs(
            np.array([r['wire_tau'] for r in wired]) - np.array([r['qtau'] for r in wired])))),
                'wire_kp_first': wired[0]['wire_kp']}
    static = np.array([[w['mean_error_rad'][j] for w in windows.values()] for j in range(14)])
    overall = {'samples': len(rows), 'completed': meta['completed'], 'error': meta['error'],
               'duration_s': float(t[-1]),
               'rms_error_rad_max_joint': float(max(j['rms_error_rad'] for j in joints)),
               'max_abs_error_rad': float(np.max(np.abs(err))),
               'max_abs_static_error_rad': float(np.max(np.abs(static))) if static.size else None,
               'tcp_error_m': {side: {'rms': float(np.sqrt(np.mean(tcp[:, k]**2))), 'max': float(tcp[:, k].max())}
                               for k, side in enumerate(('left', 'right'))},
               'max_temperature_c': int(max(max(r['temp']) for r in rows)),
               'wire': wire}
    summary = {'meta': meta, 'overall': overall, 'joints': joints, 'windows': windows}
    jdump(directory / 'summary.json', summary)
    return summary


def report(args):
    runs = [summarize(d) for d in args.runs]
    lines = ['| joint | ' + ' | '.join(f"{r['meta']['path']}/{r['meta']['ff']} rms / max static (mrad)" for r in runs) + ' |',
             '|---|' + '---|'*len(runs)]
    for j in range(14):
        cells = []
        for r in runs:
            joint = r['joints'][j] if 'joints' in r else None
            if joint is None:
                cells.append('—')
                continue
            static = max(abs(v) for v in joint['static_error_rad'].values()) if joint['static_error_rad'] else float('nan')
            cells.append(f"{joint['rms_error_rad']*1e3:.1f} / {static*1e3:.1f}")
        lines.append(f"| {ARM[j]} {('L ' if j < 7 else 'R ') + JOINTS[j % 7]} | " + ' | '.join(cells) + ' |')
    lines.append('| TCP rms/max L (mm) | ' + ' | '.join(
        f"{r['overall']['tcp_error_m']['left']['rms']*1e3:.1f} / {r['overall']['tcp_error_m']['left']['max']*1e3:.1f}"
        if 'tcp_error_m' in r['overall'] else '—' for r in runs) + ' |')
    lines.append('| TCP rms/max R (mm) | ' + ' | '.join(
        f"{r['overall']['tcp_error_m']['right']['rms']*1e3:.1f} / {r['overall']['tcp_error_m']['right']['max']*1e3:.1f}"
        if 'tcp_error_m' in r['overall'] else '—' for r in runs) + ' |')
    lines.append('| completed | ' + ' | '.join(str(r['overall'].get('completed')) for r in runs) + ' |')
    text = '\n'.join(lines)
    if args.output:
        Path(args.output).write_text(text + '\n')
        jdump(Path(args.output).with_suffix('.json'), [r['overall'] | {'path': r['meta']['path'], 'ff': r['meta']['ff'],
                                                                     'dir': str(d)} for r, d in zip(runs, args.runs)])
    print(text)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('snapshot')
    s.add_argument('--output', required=True)
    s = sub.add_parser('plan')
    s.add_argument('--start', required=True)
    s.add_argument('--output', required=True)
    s.add_argument('--profile')
    s.add_argument('--amplitudes', type=float, nargs=7)
    s.add_argument('--first-sign', type=float, nargs=7, help='+1/-1 per joint for the first excursion (left arm)')
    s.add_argument('--scale', type=float, default=1.)
    s.add_argument('--no-scene-ok', action='store_true', help='new robot without scene: skip the table clearance check')
    s.add_argument('--anchor', help='json with q[35]; trajectory goes measured -> anchor and ends at anchor')
    s.add_argument('--max-anchor-distance', type=float, default=.2,
                   help='rad; arms sag about 0.25 rad from init once a session ends without feedforward')
    s.add_argument('--raise-m', type=float, default=.08, help='lift of the excursion center above the start TCP')
    s.add_argument('--approach-s', type=float, default=4.)
    s.add_argument('--max-speed', type=float, default=.3)
    s.add_argument('--move-s', type=float, default=2.5)
    s.add_argument('--dwell-s', type=float, default=2.)
    s.add_argument('--hold-s', type=float, default=2.)
    s.add_argument('--limit-margin', type=float, default=.03)
    s.add_argument('--min-clearance', type=float, default=.05)
    s.add_argument('--min-arm-distance', type=float, default=.25)
    s = sub.add_parser('run')
    s.add_argument('--trajectory', required=True)
    s.add_argument('--path', choices=['adapter', 'direct'], required=True)
    s.add_argument('--ff', choices=MODES, required=True)
    s.add_argument('--ff-config')
    s.add_argument('--output-dir', required=True)
    s.add_argument('--confirm-motion', action='store_true')
    s = sub.add_parser('report')
    s.add_argument('runs', nargs='+')
    s.add_argument('--output')
    a = p.parse_args()
    if a.cmd == 'snapshot':
        snap = read_snapshot()
        jdump(a.output, snap)
        print(json.dumps({k: snap[k] for k in ('captured_at', 'tick', 'lowcmd_publisher_seen')}))
    elif a.cmd == 'plan':
        plan(a)
    elif a.cmd == 'run':
        run(a)
    else:
        report(a)


if __name__ == '__main__':
    main()
