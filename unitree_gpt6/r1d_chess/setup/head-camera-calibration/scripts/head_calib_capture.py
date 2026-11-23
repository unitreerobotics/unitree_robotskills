#!/usr/bin/env python3
"""Eye-on-head extrinsic capture: move head and waist through poses, record stereo frames + joints.

MOVES THE ROBOT (requires --confirm-motion). Base and lift are never selected. Arms and grippers hold
their snapshot with the torque feedforward from config/arm_torque_ff.json; the arms swing with the waist.
Stops on arm tracking > 0.15 rad, head/waist error > 0.15 rad, motor fault, overtemperature or stale
feedback; the feedforward is ramped to zero before the session ends in every case.
"""
import argparse
import json
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
import cv2
import numpy as np
import zmq

sys.path.insert(0, str(Path(__file__).resolve().parents[2])); import _common  # noqa: E402  (adds src/ to sys.path)
ROOT = _common.ROOT
from r1d_chess.robot.r1d_config import load_profile, head_waist_reference
from r1d_chess.robot.r1d_torque_ff import TorqueFeedforward, load_config as load_ff

ARM, GRIP, HEAD, WAIST = list(range(15, 29)), [31, 33], [29, 30], [13]
RATE = .2
PERIOD = .01
RUNTIME = json.loads((ROOT/'config/runtime.json').read_text())
MAX_TEMPERATURE_C = 75


def poses(reference):
    """Serpentine grid plus repeated reference poses for hysteresis and drift checks."""
    ref = {'name': 'ref', 'head': reference[:2], 'waist': reference[2]}
    out = [dict(ref, name='ref_start')]
    grid = []
    for wi, w in enumerate((-.05, .03, .11)):
        for pi, p in enumerate((.20, .33, .50)):
            yaws = (-.6, -.3, 0., .25, .45)
            for y in (yaws if (wi*3+pi) % 2 == 0 else yaws[::-1]):
                grid.append({'name': f'w{w:+.2f}_p{p:.2f}_y{y:+.2f}', 'head': [p, y], 'waist': w})
    half = len(grid)//2
    return out+grid[:half]+[dict(ref, name='ref_mid')]+grid[half:]+[dict(ref, name='ref_end')]


def head_only_poses(reference, waist):
    """Denser head grid; the waist and arms stay put."""
    ref = {'name': 'ref', 'head': reference[:2], 'waist': waist}
    grid = []
    yaws = (-.7, -.5, -.3, -.1, .1, .3, .5, .65)
    for i, p in enumerate((.12, .20, .28, .36, .44, .52, .60)):
        for y in (yaws if i % 2 == 0 else yaws[::-1]):
            grid.append({'name': f'h_p{p:.2f}_y{y:+.2f}', 'head': [p, y], 'waist': waist})
    half = len(grid)//2
    return [dict(ref, name='ref_start')]+grid[:half]+[dict(ref, name='ref_mid')]+grid[half:]+[dict(ref, name='ref_end')]


class Controller(threading.Thread):
    def __init__(self, client, feedback, ff):
        super().__init__(daemon=True)
        self.client, self.feedback, self.ff = client, feedback, ff
        q = feedback()[0]
        self.arm = q[ARM].copy()
        self.cmd = q[HEAD+WAIST].copy()
        self.target = self.cmd.copy()
        self.grip = q[GRIP].copy()
        self.lock = threading.Lock()
        self.running, self.fault = True, None
        self.tau = np.zeros(14)

    def set_target(self, head, waist):
        with self.lock:
            self.target = np.array([*head, waist], float)

    def arrived(self):
        with self.lock:
            return bool(np.max(np.abs(self.cmd-self.target)) < 1e-6)

    def send(self, tau):
        c = self.cmd
        self.client.send_joint_targets(
            {'left_arm': tuple(self.arm[:7]), 'right_arm': tuple(self.arm[7:]), 'head': tuple(c[:2]), 'waist': (float(c[2]),),
             'left_ee': (float(self.grip[0]),), 'right_ee': (float(self.grip[1]),)},
            {'left_arm': tuple(tau[:7]), 'right_arm': tuple(tau[7:])})

    def run(self):
        while self.running:
            started = time.monotonic()
            try:
                q, _, msg = self.feedback()
                for i in ARM+GRIP+HEAD+WAIST:
                    if msg.motor_state[i].motorstate or max(msg.motor_state[i].temperature) > MAX_TEMPERATURE_C:
                        raise RuntimeError(f'motor health/temperature fault: {i}')
                with self.lock:
                    step = RATE*PERIOD
                    self.cmd = self.cmd+np.clip(self.target-self.cmd, -step, step)
                arm_error = np.max(np.abs(q[ARM]-self.arm))
                if arm_error > .15:
                    raise RuntimeError(f'arm tracking {arm_error:.3f} rad exceeded 0.15')
                hw_error = np.max(np.abs(q[HEAD+WAIST]-self.cmd))
                if hw_error > .15:
                    raise RuntimeError(f'head/waist error {hw_error:.3f} rad exceeded 0.15')
                self.tau = self.ff.compute(q[ARM], self.arm, np.zeros(14), PERIOD)
                self.send(self.tau)
            except Exception as exc:
                self.fault = str(exc)
                return
            time.sleep(max(0., PERIOD-(time.monotonic()-started)))

    def ramp_out(self):
        tau = self.tau.copy()
        while np.any(tau):
            tau = tau-np.clip(tau, -30*.02, 30*.02)
            self.send(tau)
            time.sleep(.02)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', required=True)
    p.add_argument('--dry-run', action='store_true')
    p.add_argument('--confirm-motion', action='store_true')
    p.add_argument('--head-only', action='store_true', help='denser head grid; waist holds its measured angle')
    p.add_argument('--single-pose', nargs=2, type=float, metavar=('PITCH', 'YAW'),
                   help='visit one head pose and return to the reference; waist holds its measured angle')
    a = p.parse_args()
    head, waist = head_waist_reference(load_profile())
    reference = [float(head[0]), float(head[1]), float(waist[0])]
    plan = poses(reference)
    if a.dry_run:
        print(json.dumps({'poses': len(plan), 'first': plan[:3], 'reference': reference}, indent=1))
        return
    if not a.confirm_motion:
        sys.exit('refusing: this moves head and waist; pass --confirm-motion')
    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=False)
    from r1d_chess.robot.r1d_runtime import load_experiment, R1DRuntimeClient
    load_experiment()
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
    box, lock = {}, threading.Lock()

    def on_state(m):
        with lock:
            box['state'], box['at'] = m, time.monotonic()

    def on_cmd(_):
        box['cmd_at'] = time.monotonic()

    ChannelFactoryInitialize(0, RUNTIME['network_interface'])
    subs = [ChannelSubscriber('rt/lowstate', LowState_), ChannelSubscriber('rt/lowcmd', LowCmd_)]
    subs[0].Init(on_state, 1)
    subs[1].Init(on_cmd, 1)
    time.sleep(1.)

    def feedback():
        with lock:
            m, at = box.get('state'), box.get('at', 0.)
        if m is None or time.monotonic()-at > .15:
            raise RuntimeError('lowstate missing/stale')
        return np.array([x.q for x in m.motor_state]), np.array([x.dq for x in m.motor_state]), m

    context = zmq.Context()
    sock = context.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.SUBSCRIBE, b'')
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt(zmq.RCVTIMEO, 3000)
    sock.connect(f"tcp://{RUNTIME['camera_host']}:55555")
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    detector = cv2.aruco.CharucoDetector(cv2.aruco.CharucoBoard((5, 7), .035, .026, d))
    client = R1DRuntimeClient()
    client.connect()
    state = client.state()
    if state.control_active or state.runtime_error or state.reset_pose_active or state.thermal_protection_active:
        sys.exit('Runtime busy/error; refusing to take control')
    if 'cmd_at' in box and time.monotonic()-box['cmd_at'] < .5:
        sys.exit('another LowCmd publisher is active')
    q, dq, _ = feedback()
    if np.max(np.abs(dq[ARM+GRIP+HEAD+WAIST])) > .1:
        sys.exit('joints moving before capture')
    if a.head_only:
        plan = head_only_poses(reference, float(q[WAIST[0]]))
    if a.single_pose:
        pitch, yaw = a.single_pose
        if not (-.62 <= pitch <= .62 and -2. <= yaw <= 2.):
            sys.exit('single pose outside head limits (pitch +-0.62, yaw +-2.0)')
        w = float(q[WAIST[0]])
        plan = [{'name': 'ref_start', 'head': reference[:2], 'waist': w},
                {'name': f'single_p{pitch:.2f}_y{yaw:+.2f}', 'head': [pitch, yaw], 'waist': w},
                {'name': 'ref_end', 'head': reference[:2], 'waist': w}]
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    manifest = {'started_at': datetime.now().astimezone().isoformat(), 'reference': reference,
                'start_q': q.tolist(), 'poses': [], 'error': None}
    client.begin()
    controller = Controller(client, feedback, TorqueFeedforward(load_ff()))
    controller.start()

    def wait_motion():
        while not controller.arrived():
            if controller.fault or stop.is_set():
                raise RuntimeError(controller.fault or 'operator stop')
            time.sleep(.02)
        still = None
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline:
            if controller.fault or stop.is_set():
                raise RuntimeError(controller.fault or 'operator stop')
            moving = np.max(np.abs(feedback()[1][HEAD+WAIST]))
            still = (still or time.monotonic()) if moving < .03 else None
            if still and time.monotonic()-still >= 1.:
                return True
            time.sleep(.01)
        return False

    try:
        for index, pose in enumerate(plan):
            controller.set_target(pose['head'], pose['waist'])
            settled = wait_motion()
            directory = out/f'{index:03d}_{pose["name"]}'
            directory.mkdir()
            sock.recv()
            samples, corners = [], []
            for k in range(6):
                raw = sock.recv()
                (directory/f'{k}.jpg').write_bytes(raw)
                f = feedback()
                samples.append(f[0])
                gray = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)
                ids = [detector.detectBoard(gray[:, :640])[1], detector.detectBoard(gray[:, 640:])[1]]
                corners.append([0 if i is None else len(i) for i in ids])
            samples = np.array(samples)
            record = {'index': index, **pose, 'directory': directory.name, 'settled': settled,
                      'head_q': samples[:, HEAD].mean(0).tolist(), 'waist_q': float(samples[:, WAIST].mean()),
                      'hw_q_std': samples[:, HEAD+WAIST].std(0).tolist(), 'arm_q': samples[:, ARM].mean(0).tolist(),
                      'corners_lr': corners, 'cmd': controller.cmd.tolist()}
            manifest['poses'].append(record)
            (out/'manifest.json').write_text(json.dumps(manifest, indent=1))
            print(f'{index+1}/{len(plan)} {pose["name"]} corners L{max(c[0] for c in corners)} R{max(c[1] for c in corners)} settled={settled}', flush=True)
        controller.set_target(reference[:2], plan[-1]['waist'])
        wait_motion()
    except Exception as exc:
        manifest['error'] = str(exc)
        print('ABORT', exc, flush=True)
    finally:
        controller.running = False
        controller.join(1.)
        if controller.fault:
            manifest['error'] = manifest['error'] or controller.fault
            q = feedback()[0]
            controller.arm, controller.cmd = q[ARM].copy(), q[HEAD+WAIST].copy()
        try:
            controller.ramp_out()
        finally:
            client.end()
            client.close()
            for sub in subs:
                sub.Close()
            sock.close()
            context.term()
        manifest['finished_at'] = datetime.now().astimezone().isoformat()
        (out/'manifest.json').write_text(json.dumps(manifest, indent=1))
    print(json.dumps({'poses': len(manifest['poses']), 'error': manifest['error']}))
    sys.exit(1 if manifest['error'] else 0)


if __name__ == '__main__':
    main()
