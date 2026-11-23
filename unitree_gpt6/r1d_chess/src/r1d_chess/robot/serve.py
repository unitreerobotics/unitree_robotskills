#!/usr/bin/env python3
"""R1-D chess transaction service. Startup is read-only unless explicitly enabled."""
import argparse
import hashlib
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
from r1d_chess.robot.r1d_config import ROOT, MODEL_PATH, load_profile, require_calibration, head_waist_reference, geometry_source, check_kinematic_model

from r1d_chess.robot import internal_service as engine
from r1d_chess.robot.arm_roles import LeftActorDDS, LeftActorMotion, physical_motor
from r1d_chess.robot.async_event_log import AsyncEventLog
from r1d_chess.robot.board_guard import BoardGuard
from r1d_chess.robot.camera_policy import planning_cameras
from r1d_chess.robot.motion_harness import MotionHarness
from r1d_chess.robot.motion_harness import matrix, waypoint
from r1d_chess.robot.r1d_transport import InternalDDS, SimulatedDDS, ARM, GRIP, HEAD, WAIST


def profile_digest(profile):
    return hashlib.sha256(json.dumps(profile, sort_keys=True, allow_nan=False).encode()).hexdigest()


class R1DService(engine.RobotService):
    def __init__(self, interface, camera_host, log_dir, *, arm='right', simulate=False, allow_motion=False):
        self.profile = load_profile()
        # Fail at startup, before any DDS/IPC connection, on a profile from another URDF.
        check_kinematic_model(self.profile)
        self.profile_sha256 = profile_digest(self.profile)
        self.model_sha256 = hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest()
        self.simulate = simulate
        self.physical_motion = MotionHarness()
        self.physical_dds = (SimulatedDDS() if simulate else
                             InternalDDS(interface, gripper_open_sign=1, allow_motion=allow_motion))
        self.motion, self.dds = self.physical_motion, self.physical_dds
        self.camera_host, self.camera = camera_host, None
        self.log_dir = Path(log_dir or ROOT/'runs'/datetime.now().strftime('%Y%m%d_%H%M%S_service'))
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = self.log_dir/'events.jsonl'
        self.event_log = AsyncEventLog(self.log_path)
        self.cancelled = threading.Event()
        self.guard = BoardGuard.from_dict(self.profile['scene']) if self.profile['scene'] else None
        self.cached_plans, self.cached_sequences = {}, {}
        self.set_actor(arm)

    def set_actor(self, arm):
        if arm not in ('left', 'right') or self.physical_dds.enabled:
            raise ValueError('select a physical arm only while transaction control is disabled')
        self.cached_plans.clear()
        self.cached_sequences.clear()
        self.retract_trace = None          # role-space joint samples; meaningless after a role swap
        self.active_arm = arm
        self.dds = LeftActorDDS(self.physical_dds) if arm == 'left' else self.physical_dds
        self.motion = LeftActorMotion(self.physical_motion) if arm == 'left' else self.physical_motion

    def state(self):
        result = super().state()
        physical_q = self.physical_dds.state()[0]
        head_reference, waist_reference = head_waist_reference(self.profile)
        try:
            source = geometry_source(require_calibration(self.profile))
            geometry_ready = True
        except (KeyError, TypeError, ValueError):
            source = self.profile.get('geometry_source', 'missing')
            geometry_ready = False
        result.update(robot='R1-D', simulated=self.simulate, backend='R1-D Runtime IPC',
                      profile_sha256=self.profile_sha256,
                      xy_error_context='r1d_'+self.profile_sha256[:16],
                      motion_allowed=(self.simulate or self.physical_dds.allow_motion),
                      calibration_verified=self.profile['calibration_verified'],
                      geometry_ready=geometry_ready, geometry_source=source,
                      coordinate_frame='waist_yaw_link',
                      tcp_definition=('calibrated wrist-to-TCP' if source == 'verified' else
                                      'estimated wrist-to-TCP' if geometry_ready else
                                      'UNCALIBRATED wrist frame preview; not a grasp TCP'),
                      head_q=physical_q[HEAD].tolist(), waist_q=physical_q[WAIST].tolist(),
                      physical_full_q=physical_q.tolist(),   # motor order, independent of the active arm
                      hold_ff=(self.physical_dds.hold_ff.status() if hasattr(self.physical_dds, 'hold_ff') else None),
                      head_reference_q=head_reference.tolist(), waist_reference_q=waist_reference.tolist(),
                      head_waist_reference_verified=bool(
                          self.physical_dds.session_active and np.max(np.abs(
                              physical_q[HEAD+WAIST]-np.concatenate((head_reference, waist_reference))
                          )) <= self.profile['head_waist_limit_rad']),
                      head_waist_session_active=self.physical_dds.session_active,
                      arm_field_frame='transaction_roles',
                      arm_role_mapping={'right': f'physical_{self.active_arm}_actor',
                                        'left': 'physical_right_observer' if self.active_arm == 'left' else 'physical_left_observer'},
                      controlled_motors=ARM+GRIP+HEAD+WAIST if self.physical_dds.session_active else [],
                      runtime_error_checks_enabled=not self.simulate,
                      runtime_joint_tracking_rejection_enabled=not self.simulate,
                      tracking_fault_snapshot=getattr(
                          self.physical_dds, 'tracking_fault_snapshot', None),
                      torque_feedforward=(
                          'arm qtau from config/arm_torque_ff.json beside Runtime PD gains'
                          if getattr(getattr(self.physical_dds, 'torque_ff', None), 'enabled', False)
                          else 'none; Runtime configured R1-D PD gains'),
                      arm_torque_ff=(self.physical_dds.torque_ff.status()
                                     if hasattr(self.physical_dds, 'torque_ff') else None),
                      full_arm_and_neighbor_collision_checked=False)
        if self.active_arm == 'left':
            for gripper in result['grippers'].values():
                gripper['motor_index'] = physical_motor(gripper['motor_index'])
            result['motor_diagnostics'] = {str(physical_motor(int(k))): v for k, v in result['motor_diagnostics'].items()}
        return result

    def capture(self, cameras=None, output_dir=None):
        cameras = list(planning_cameras() if cameras is None else cameras)
        directory = Path(output_dir or self.log_dir/'images').resolve()
        if not any(directory.is_relative_to(ROOT/name) for name in ('runs',)):
            raise ValueError('camera output must be inside r1d_chess/observation(s)')
        if self.simulate:
            from r1d_chess.cameras.r1d_cameras import PORTS
            if not cameras or not set(cameras) <= set(PORTS):
                raise ValueError('invalid simulated cameras')
            paths = {}
            for name in cameras:
                path = directory/name/(datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.jpg')
                path.parent.mkdir(parents=True, exist_ok=True)
                image = np.zeros((480, 640, 3), np.uint8)
                cv2.putText(image, 'SIMULATION ONLY - '+name, (15, 230), cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2)
                if not cv2.imwrite(str(path), image):
                    raise OSError('simulation image write failed')
                paths[name] = str(path)
            result = {'timestamp': datetime.now().astimezone().isoformat(), 'paths': paths,
                      'camera_order': cameras, 'simulated': True}
        else:
            from r1d_chess.cameras.r1d_cameras import capture
            result = capture(self.camera_host, cameras, directory, profile=self.profile)
        result.update(state=self.state(), physical_actor=self.active_arm)
        self.log({'event': 'observation', **result})
        return result

    def _execute_cached_bundle(self, bundle, *args, **kwargs):
        result = super()._execute_cached_bundle(bundle, *args, **kwargs)
        result.update(robot='R1-D', simulated=self.simulate, profile_sha256=self.profile_sha256,
                      r1d_profile=self.profile, r1d_model_sha256=self.model_sha256)
        self._atomic_json(bundle['directory']/(bundle['token']+'_result.json'), result)
        return result

    def prepare_init(self, archive_dir):
        if self.physical_dds.enabled:
            raise ValueError('prepare initialization while transaction control is disabled')
        q,_,_,_=self.dds.state()
        if np.max(np.abs(q[GRIP]))>=.15:
            raise ValueError('init requires both grippers confirmed empty/closed')
        poses=self.motion.fk(q[ARM])
        physical_order=['right','left'] if self.active_arm=='left' else ['left','right']
        arms={};distances={}
        for key,side,start in zip(['left_arm','right_arm'],physical_order,poses):
            goal=dict(zip(('x','y','z','roll','pitch','yaw'),self.profile['init_xyz_rpy'][side]),gripper='hold')
            target=matrix(goal)
            distance=float(np.linalg.norm(start[:3,3]-target[:3,3]))
            distances[side]=distance
            current=waypoint(start)
            if distance<=.10+1e-9:
                arms[key]=[current]
                continue
            origin_z=[self.guard.clearance(p)-p[2,3] for p in (start,target)]
            high=max(start[2,3],target[2,3],self.guard.travel_clearance-min(origin_z)+.01)
            # end exactly on init_joint_rad. Rotate while travelling to above init (init
            # roll/yaw, pitch 0.4, as round_tools), then a joint_q step: configuration change at that
            # height and vertical descent along the IK chain seeded from the init joints.
            ret={'roll':goal['roll'],'pitch':.4,'yaw':goal['yaw']}
            arms[key]=[{**current,'z':high},
                       {**goal,**ret,'z':high},
                       {**goal,'joint_q':[float(v) for v in self.profile['init_joint_rad'][side]]}]
        if all(distance<=.10+1e-9 for distance in distances.values()):
            return {'init_required':False,'distances_m':distances,'robot_requests_for_motion':0}
        prepared=self.prepare_cached({'archive_dir':archive_dir,'plan':{
            **arms,'speed_scale':.5,'skip_preflight':False}})
        self.cached_plans[prepared['token']]['r1d_init_only']=True
        return {**prepared,'init_required':True,'distances_m':distances,'kind':'r1d_init_only'}

    RETRACT_SLOWDOWN = 1.5
    RETRACT_MAX_SPEED_RAD_S = .3

    # Outcomes after which the actor is back at its init/start pose (unload
    # the arms after every return so the motors do not keep working and overheat). Outcomes that stop the
    # arm elsewhere (stopped_after_fault, sequence_stopped, alignment_pending_high, not_started) keep holding:
    # the arm would fall onto the board, possibly holding a piece.
    UNLOAD_AFTER_OUTCOMES = ('placed', 'completed', 'alignment_failed_returned', 'auto_grasp_failed_returned',
                             'empty_grasp_limit_returned', 'unverified_grasp_returned',
                             'motion_failed_returned_to_start')

    SETTLE_INIT_LIMIT_RAD = .35

    def settle_init(self):
        """Bring both arms from where the handover left them back onto init_joint_rad.

        goinit ramps its feedforward out when it stops, and under the Runtime PD gains alone the arms sag
        about 0.2 rad before this service holds them. Joint-space ramp at the
        transport's 0.3 rad/s with feedforward; refused when an arm is further than 0.35 rad from init."""
        dds = self.physical_dds
        if not dds.enabled or dds.fault:
            raise RuntimeError('settle_init needs enabled, fault-free control')
        init = np.r_[self.profile['init_joint_rad']['left'], self.profile['init_joint_rad']['right']].astype(float)
        q = dds.state()[0][ARM]
        before = float(np.max(np.abs(q-init)))
        if before > self.SETTLE_INIT_LIMIT_RAD:
            raise RuntimeError(f'arms are {before:.3f} rad from init; run goinit instead')
        with dds.lock:
            if not np.allclose(dds.arm_target, dds.arm_sent, atol=1e-6):
                raise RuntimeError('an arm motion is in progress')
            dds.arm_target = init.copy()
        deadline = time.monotonic()+before/.3+4.
        while time.monotonic() < deadline:
            time.sleep(.1)
            if dds.fault:
                raise RuntimeError(dds.fault)
            with dds.lock:
                arrived = np.allclose(dds.arm_sent, init, atol=1e-6)
            if arrived:
                break
        time.sleep(1.5)
        after = float(np.max(np.abs(dds.state()[0][ARM]-init)))
        self.log({'event': 'settle_init', 'before_rad': before, 'after_rad': after})
        return {'settled': True, 'before_rad': before, 'after_rad': after}

    def unload_arms_now(self, reason):
        """End the Runtime session (feedforward ramped to zero) and send zero-gain arm frames.

        Both arms go limp and fall under gravity; head/waist/grippers keep their last LowCmd. The service keeps
        running; the next round takes over again (enable_control or handover after goinit)."""
        dds = self.physical_dds
        if dds.enabled:
            raise RuntimeError('arm execution is enabled; finish or release_control before unloading')
        if dds.fault:
            raise RuntimeError(f'latched fault {dds.fault!r}; not unloading automatically')
        if dds.session_active:
            dds.release_waist()
        result = dict(dds.unload_arms(), reason=reason)
        # Proof that the motors are no longer working: limp arms resting on the table do
        # not visibly drop, so report the estimated arm motor torques ~1 s after the zero-gain frames.
        if result.get('unloaded'):
            time.sleep(1.)
            msg = dds.state()[2]
            tau = [float(msg.motor_state[i].tau_est) for i in ARM]
            result.update(arm_tau_est_after_nm=[round(t, 3) for t in tau],
                          arm_tau_max_abs_after_nm=round(max(abs(t) for t in tau), 3))
        self.log({'event': 'arms_unloaded_in_service', **result})
        return result

    def retract(self):
        """Replay the executed joint samples of the last execute in reverse, back to where it started.

        For an uncaught error that stopped the arm, or an explicit operator request to return. No new
        preflight: every sample was preflighted (or executed safely) on the way out. The actor gripper is
        closed (without waiting, the arm keeps moving) as soon as a reversed sample reaches transport
        clearance, as in the G1 return: open rise -> close high -> init. If no sample is that high it is
        closed at the end. A latched fault is cleared because the operator asked for this retract; Runtime
        entry checks, motor faults, feedback timeouts and the 0.20 rad tracking limit still stop it.
        """
        trace = list(self.retract_trace or [])
        if len(trace) < 2:
            raise ValueError('nothing executed since the last execute; no retract path')
        dds = self.physical_dds
        if dds.fault:
            self.log({'event': 'retract_clears_fault', 'fault': dds.fault})
            dds.clear_fault()
        if not dds.enabled:
            dds.enable()
        self.cancelled.clear()
        self.retract_trace = None           # the retract itself is not recorded
        q = self.dds.state()[0][ARM]
        targets = [trace[-1][0]]+[trace[i-1][0] for i in range(len(trace)-1, 0, -1)]
        durations = [None]+[trace[i][1] for i in range(len(trace)-1, 0, -1)]
        self.log({'event': 'retract_start', 'samples': len(targets), 'start_q': q.tolist()})
        previous = q.copy()
        done = 0
        closed = False

        def close_gripper(reason):
            self.dds.command_gripper('right', 0.)     # role-space actor gripper; non-blocking
            self.log({'event': 'retract_gripper_close', 'reason': reason, 'segment': done})
            return True
        try:
            for target, duration in zip(targets, durations):
                span = float(np.max(np.abs(target-previous)))
                if span < 1e-5:
                    continue
                t = max(duration or 0., span/self.RETRACT_MAX_SPEED_RAD_S, .05)*self.RETRACT_SLOWDOWN
                start = time.monotonic()
                while True:
                    if self.cancelled.is_set():
                        raise RuntimeError('retract cancelled')
                    self._cached_health()
                    alpha = min(1., (time.monotonic()-start)/t)
                    blend = alpha*alpha*(3-2*alpha)
                    self.dds.command_arm(previous+(target-previous)*blend, np.zeros(14))
                    if alpha >= 1:
                        break
                    time.sleep(.01)
                previous = target
                done += 1
                if (not closed and self.guard is not None and
                        self.guard.clearance(self.motion.fk(target)[1]) >= self.guard.travel_clearance):
                    closed = close_gripper('transport_clearance')
            if not closed:
                closed = close_gripper('retract_end')
            time.sleep(.5)
            result = {'retracted': True, 'segments': done, 'gripper_closed': closed}
        except Exception as exc:
            try:
                self.dds.hold()
            except Exception:
                pass
            result = {'retracted': False, 'segments': done, 'error': str(exc)}
        finally:
            with self.physical_dds.lock:
                self.physical_dds.enabled = False     # session keeps holding the final pose
        self.log({'event': 'retract_end', **result})
        if result.get('retracted'):
            try:
                result['arm_unload'] = self.unload_arms_now('after_retract')
            except Exception as exc:
                result['arm_unload'] = {'unloaded': False, 'error': str(exc)}
        try:
            result['state'] = self.state()
        except Exception as exc:
            result['state_error'] = str(exc)
        return result

    def dispatch(self, request):
        command = request.get('cmd')
        if command == 'retract':
            return self.retract()
        if command == 'unload_arms':
            return {'arm_unload': self.unload_arms_now('operator_request'), **self.state()}
        if command in ('execute_transaction', 'execute_sequence', 'execute_init'):
            q0 = self.dds.state()[0][ARM]
            self.retract_trace = [(np.asarray(q0, float).copy(), 0.)]
        if command == 'settle_init':
            return self.settle_init()
        if command == 'ping':
            return {'pong':True,'robot':'R1-D','backend':'R1-D Runtime IPC',
                    'active_arm':self.active_arm,'profile_sha256':self.profile_sha256,'simulated':self.simulate}
        if command == 'set_active_arm':
            self.set_actor(request['arm'])
            return self.state()
        if command == 'lock_waist_zero':
            raise ValueError('R1-D head/waist share Runtime ownership; establish the saved chess initial reference before handover')
        planning = {'prepare_plan', 'prepare_cached_plan', 'prepare_transaction', 'prepare_sequence','prepare_init'}
        motion = {'enable_control', 'set_gripper', 'execute_transaction', 'execute_sequence','execute_init'}
        if command in planning | motion:
            require_calibration(self.profile, hardware=not self.simulate)
            if request.get('expected_active_arm') != self.active_arm:
                raise ValueError(f'expected_active_arm={self.active_arm} required')
            if profile_digest(load_profile()) != self.profile_sha256:
                raise ValueError('profile changed; restart an idle service and create fresh plans/tokens')
        if command in {'execute_plan', 'execute_cached_plan', 'restore_cached_transaction'}:
            raise ValueError('use freshly prepared complete transaction/sequence; generic replay is disabled')
        if command=='prepare_init':
            return self.prepare_init(request['archive_dir'])
        if command=='execute_init':
            bundle=self.cached_plans.get(request['token'])
            if not bundle or not bundle.get('r1d_init_only'):
                raise ValueError('execute_init requires a fresh R1-D initialization token')
            return self.execute_cached(request['token'])
        if command == 'set_scene':
            if request.get('scene') != self.profile['scene']:
                raise ValueError('scene must match calibrated profile; edit/review profile before restarting idle service')
        if command == 'shutdown' and self.physical_dds.session_active:
            raise RuntimeError('head/waist session remains active; explicit release_waist ends all six groups')
        result = super().dispatch(request)
        if (command in ('execute_transaction', 'execute_sequence') and isinstance(result, dict)
                and result.get('outcome') in self.UNLOAD_AFTER_OUTCOMES):
            try:
                result['arm_unload'] = self.unload_arms_now('after_return_' + result['outcome'])
            except Exception as exc:
                result['arm_unload'] = {'unloaded': False, 'error': str(exc)}
        return {**result, 'robot': 'R1-D', 'active_arm': self.active_arm,
                'simulated': self.simulate, 'profile_sha256': self.profile_sha256}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile')
    parser.add_argument('--endpoint', default='ipc:///tmp/astra_r1d_chess.sock')
    parser.add_argument('--arm', choices=['left', 'right'], default='right')
    parser.add_argument('--simulate', action='store_true')
    parser.add_argument('--allow-motion', action='store_true')
    parser.add_argument('--keep-arms-loaded', action='store_true',
                        help='on exit keep the arms holding (default: unload them so they can be hand-guided)')
    parser.add_argument('--log-dir')
    args = parser.parse_args()
    if args.profile:
        import os
        os.environ['R1D_CHESS_PROFILE'] = str(Path(args.profile).resolve())
    if not args.endpoint.startswith(('ipc://', 'tcp://127.0.0.1:')):
        parser.error('endpoint must be local')
    # flock prevents a second process from unlinking/rebinding an active IPC endpoint.
    import fcntl
    lock = open('/tmp/astra_r1d_chess_'+hashlib.sha256(args.endpoint.encode()).hexdigest()[:16]+'.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    runtime = json.loads((ROOT/'config/runtime.json').read_text())
    service = R1DService(runtime['network_interface'], runtime['camera_host'], args.log_dir,
                         arm=args.arm, simulate=args.simulate, allow_motion=args.allow_motion)
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.setsockopt(zmq.LINGER, 0)
    socket.bind(args.endpoint)
    print(json.dumps({'ready': True, 'endpoint': args.endpoint, 'state': service.state()}), flush=True)

    def interrupt(*_):
        # Stopping the chess service (Ctrl-C / SIGTERM): cancel motion and hold, then exit; the transport ramps
        # feedforward to zero, ends the session and unloads the arms (unless --keep-arms-loaded).
        if service.physical_dds.session_active:
            service.cancelled.set()
            try:
                service.dds.hold()
            except Exception as exc:
                service.log({'event': 'interrupt_hold_failed', 'error': str(exc)})
            print('Stopping: motion cancelled, ending session and unloading arms.', flush=True)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        while True:
            request = socket.recv_json()
            try:
                response = {'ok': True, **service.dispatch(request)}
            except Exception as exc:
                response = {'ok': False, 'error': str(exc)}
                service.log({'event': 'request_failed', 'cmd': request.get('cmd'), 'error': str(exc)})
            socket.send_json(response)
            if response.get('shutdown'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        service.physical_dds.close(unload_arms=not args.keep_arms_loaded)
        unload = getattr(service.physical_dds, 'unload_result', None)
        if unload is not None:
            service.log({'event': 'arms_unloaded' if unload.get('unloaded') else 'arm_unload_failed', **unload})
            print(json.dumps({'arm_unload': unload}), flush=True)
        service.event_log.close()
        socket.close()
        context.term()
        lock.close()


if __name__ == '__main__':
    main()
