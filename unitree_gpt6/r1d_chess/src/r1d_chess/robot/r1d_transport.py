"""One R1-D Runtime IPC owner; raw DDS subscribers never publish LowCmd."""
import queue
import threading
import time
from types import SimpleNamespace
import numpy as np
from r1d_chess.robot.r1d_config import load_profile, require_calibration, head_waist_reference
from r1d_chess.robot.r1d_torque_ff import TorqueFeedforward, load_config as load_torque_ff
from r1d_chess.robot.r1d_hold_ff import HoldFeedforward

ARM = list(range(15, 29))
GRIP = [31, 33]
WAIST = [13]
HEAD = [29, 30]
HEAD_WAIST_RATE = .2      # rad/s ramp from measured head/waist to the saved reference
TAU_RAMP_OUT_RATE = 30.   # Nm/s when ending a session or after a latched fault
STILL_LINEAR = .01          # normalised rt/state_vel linear.x / linear.z treated as moving above this
STILL_ANGULAR = .025        # normalised rt/state_vel angular.z
HEAD_ENTRY_LIMIT_RAD = .35  # head may start this far from its reference; it is ramped there


class InternalDDS:
    def __init__(self, interface, timeout=8., gripper_open_sign=1, *, allow_motion=False,
                 torque_ff_config=None):
        from r1d_chess.robot.r1d_runtime import load_experiment
        load_experiment()
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
        self.profile = load_profile()
        from r1d_chess.robot.r1d_kinematics import R1DKinematics
        self.kinematics = R1DKinematics(self.profile)
        if gripper_open_sign != 1:
            raise ValueError('R1-D verified command convention is positive aperture')
        self.gripper_open_sign = 1
        self.allow_motion = allow_motion
        self.lock = threading.RLock()
        self.msg = None
        self.received_at = 0.
        self._last_command_at = 0.
        self._last_lowcmd = None
        self._requests = queue.Queue()
        self._ready = threading.Event()
        self._startup_error = None
        self.enabled = self.waist_enabled = self.session_active = False
        self.fault = None
        self.tracking_fault_snapshot = None
        self.fault_hold_active = False
        self.fault_hold_publish_error = None
        self.fault_hold_last_published_at = None
        self.published = 0
        self.running = True
        self.arm_target = self.arm_sent = np.zeros(14)
        self.gripper_target = self.gripper_sent = np.zeros(2)
        self.tau = np.zeros(14)
        _, waist = head_waist_reference(self.profile)
        self.waist_target = waist.copy()
        self.waist_sent = waist.copy()
        self.head_waist_sent = None
        self.torque_ff = TorqueFeedforward(torque_ff_config or load_torque_ff())
        self.arm_tau_sent = np.zeros(14)
        self.hold_ff = HoldFeedforward()
        self.tau_ramp_out_error = None
        self.ever_session = False
        self.unload_result = None
        ChannelFactoryInitialize(0, interface)
        self.subscriber = ChannelSubscriber('rt/lowstate', LowState_)
        self.subscriber.Init(self._receive, 1)
        self.command_subscriber = ChannelSubscriber('rt/lowcmd', LowCmd_)
        self.command_subscriber.Init(self._receive_command, 1)
        end = time.monotonic() + timeout
        while self.msg is None and time.monotonic() < end:
            time.sleep(.01)
        try:
            self.state()
        except Exception:
            self.subscriber.Close()
            self.command_subscriber.Close()
            raise
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        if not self._ready.wait(timeout + 2):
            self.close()
            raise TimeoutError('R1-D Runtime IPC connection timed out')
        if self._startup_error:
            self.close()
            raise RuntimeError(self._startup_error)

    def _receive(self, message):
        with self.lock:
            if self.msg is None or message.tick != self.msg.tick:
                self.msg = message
                self.received_at = time.monotonic()

    def _receive_command(self, message):
        with self.lock:
            self._last_lowcmd = message
            self._last_command_at = time.monotonic()

    def state(self):
        with self.lock:
            age = time.monotonic() - self.received_at
            if self.msg is None or age > self.profile['feedback_timeout_s']:
                raise RuntimeError(f'advancing R1-D lowstate missing/stale ({age:.3f}s)')
            q = np.array([m.q for m in self.msg.motor_state], dtype=float)
            dq = np.array([m.dq for m in self.msg.motor_state], dtype=float)
            if not np.isfinite(q[ARM+GRIP+WAIST+HEAD]).all() or not np.isfinite(dq[ARM+GRIP+WAIST+HEAD]).all():
                raise RuntimeError('non-finite R1-D feedback')
            return q, dq, self.msg, age

    def _rpc(self, command):
        event = threading.Event()
        result = {}
        self._requests.put((command, event, result))
        if not event.wait(20):
            raise TimeoutError('IPC-owner operation timeout; inspect status before retry')
        if 'error' in result:
            raise RuntimeError(result['error'])

    def enable(self):
        if not self.allow_motion:
            raise RuntimeError('service is read-only; start with --allow-motion for explicit control')
        require_calibration(self.profile, hardware=True)
        if self.fault:
            raise RuntimeError(self.fault)
        self._rpc('enable')

    def _begin(self, client):
        from robot_runtime.drivers.unitree_sdk2.robot.motion.switcher import MotionServiceSwitcher
        if self.session_active:
            with self.lock:
                if self.enabled:
                    return
                q, _, _, _ = self.state()
                self.arm_target = self.arm_sent = q[ARM].copy()
                self.gripper_target = self.gripper_sent = q[GRIP].copy()
                self.enabled = True
            return
        state = client.state()
        if state.control_active or state.runtime_error or state.reset_pose_active:
            raise RuntimeError('Runtime busy/error; existing r1d-hold-initial owner must be handed over first')
        q, dq, _, _ = self.state()
        if np.max(np.abs(dq[ARM+GRIP+WAIST+HEAD])) > .1:
            raise RuntimeError('joints moving before handover')
        self._check_reference(state, q, entry=True)
        switch = MotionServiceSwitcher(restore_motion_mode='ai')
        switch.initialize()
        entered = False
        try:
            mode, _ = switch.service_state()
            if mode == 'ai':
                switch.release_control_service(timeout=6.)
            elif mode != '':
                raise RuntimeError(f'unsupported motion owner {mode!r}')
            quiet_start = time.monotonic()
            time.sleep(.3)
            if self._last_command_at >= quiet_start:
                raise RuntimeError('another LowCmd publisher remains after AI release')
            q, _, _, _ = self.state()
            self._check_reference(client.state(), q, entry=True)
            client.begin()
            entered = True
            switch.restore_required = False
            with self.lock:
                self.arm_target = self.arm_sent = q[ARM].copy()
                self.gripper_target = self.gripper_sent = q[GRIP].copy()
                # Ramp head/waist from where they are instead of stepping to the reference.
                self.head_waist_sent = q[HEAD+WAIST].copy()
                self.torque_ff.request_zero(False)
                self.torque_ff.reset()
                self.torque_ff.output[:] = 0.
                self.hold_ff.reset()
                self.arm_tau_sent = np.zeros(14)
                self.enabled = self.waist_enabled = self.session_active = True
                self.ever_session = True
                self.session_started_at = time.monotonic()
        finally:
            if not entered:
                switch.restore_service()
            switch.shutdown()

    def _check_reference(self, state, q, target=None, *, entry=False):
        """At entry the head may be up to HEAD_ENTRY_LIMIT_RAD from its reference (it is ramped
        there at 0.2 rad/s); the waist must already be within head_waist_limit_rad because turning it
        swings both arms. During a session feedback must follow the ramped command target."""
        if state.runtime_error or state.thermal_protection_active:
            raise RuntimeError('Runtime error/thermal protection')
        head, waist = head_waist_reference(self.profile)
        limit = self.profile['head_waist_limit_rad']
        if entry:
            if np.max(np.abs(q[HEAD]-head)) > HEAD_ENTRY_LIMIT_RAD or np.max(np.abs(q[WAIST]-waist)) > limit:
                raise RuntimeError('head/waist differ from saved chess initial reference')
        else:
            goal = np.concatenate((head, waist)) if target is None else np.asarray(target)
            if np.max(np.abs(q[HEAD+WAIST]-goal)) > limit:
                raise RuntimeError('head/waist differ from saved chess initial reference')
        base = state.observation.base_group
        if len(base) != 1 or base[0].lift_height is None:
            raise RuntimeError('missing lift feedback')
        if abs(base[0].lift_height-self.profile['lift_height_m']) > self.profile['lift_tolerance_m']:
            raise RuntimeError('lift height differs from calibration')
        # rt/state_vel is normalised [-1, 1] measured velocity with a publish-side deadband (0.005 linear,
        # 0.02 angular); docs/r1d_twist_dds_topics.md recommends treating |linear| > 0.01 and |angular| > 0.025
        # as real motion, since standing-still spikes reach about 0.012 on the angular axis.
        for value, limit in ((base[0].base_linear_vel_x, STILL_LINEAR), (base[0].base_linear_vel_y, STILL_LINEAR),
                             (base[0].base_angular_vel_z, STILL_ANGULAR), (base[0].lift_linear_vel_z, STILL_LINEAR)):
            if value is None or not np.isfinite(value) or abs(value) > limit:
                raise RuntimeError('base/lift must remain stationary')

    def _publish_once(self, client, dt):
        q, dq, msg, age = self.state()
        state = client.state_receiver.latest_state(max_age=.15)
        if not state.control_active and time.monotonic()-self.session_started_at > .5:
            raise RuntimeError('Runtime control session ended unexpectedly')
        # head/waist offset, lift height and base stillness are entry conditions only
        # (_begin / handover); during a session only Runtime error/thermal protection is re-checked here.
        if state.runtime_error or state.thermal_protection_active:
            raise RuntimeError('Runtime error/thermal protection')
        for i in ARM+GRIP+WAIST+HEAD:
            m = msg.motor_state[i]
            if m.motorstate or max(m.temperature) > self.profile['max_motor_temperature_c']:
                raise RuntimeError(f'motor health/temperature fault: {i}')
        with self.lock:
            errors = q[ARM] - self.arm_sent
            if np.max(np.abs(errors)) > self.profile['arm_tracking_limit_rad']:
                worst = int(np.argmax(np.abs(errors)))
                last_command = getattr(self, '_last_lowcmd', None)
                # Preserve physical motor indices and both command layers before
                # disabling the session hides its live command fields in state().
                self.tracking_fault_snapshot = {
                    'captured_at_unix_s': time.time(),
                    'feedback_tick': int(getattr(msg, 'tick', 0)),
                    'feedback_age_s': float(age),
                    'frame': 'physical_motor_indices',
                    'motor_indices': ARM.copy(),
                    'threshold_rad': float(self.profile['arm_tracking_limit_rad']),
                    'worst_motor_index': ARM[worst],
                    'signed_error_rad': float(errors[worst]),
                    'error_definition': 'feedback minus last submitted arm setpoint',
                    'feedback_q_rad': q[ARM].tolist(),
                    'feedback_dq_rad_s': dq[ARM].tolist(),
                    'arm_target_rad': self.arm_target.tolist(),
                    'arm_sent_rad': self.arm_sent.tolist(),
                    'arm_tau_sent_nm': np.asarray(getattr(self, 'arm_tau_sent', np.zeros(14))).tolist(),
                    'torque_ff': (self.torque_ff.status() if hasattr(self, 'torque_ff') else None),
                    'hold_ff': (self.hold_ff.status() if hasattr(self, 'hold_ff') else None),
                    'published_commands': self.published,
                    'lowcmd_age_s': (float(time.monotonic()-self._last_command_at)
                                     if last_command is not None else None),
                    'observed_lowcmd': ({str(i): {
                        key: float(getattr(last_command.motor_cmd[i], key))
                        for key in ('q', 'dq', 'kp', 'kd', 'tau')}
                        for i in ARM} if last_command is not None else None),
                }
                raise RuntimeError('arm tracking error exceeded R1-D limit')
            previous = self.arm_sent.copy()
            if self.enabled:
                self.arm_sent = self.arm_sent + np.clip(self.arm_target-self.arm_sent, -.3*dt, .3*dt)
                self.gripper_sent = self.gripper_sent + np.clip(self.gripper_target-self.gripper_sent, -.8*dt, .8*dt)
            qtau = self.torque_ff.compute(q[ARM], self.arm_sent, (self.arm_sent-previous)/dt, dt)
            # Even between rounds, a single owner holds all six groups, including head/waist.
            targets = self._targets(q)
            # Waist/head: head gravity, friction while the reference ramps, small integral at rest.
            hold = self.hold_ff.compute(q[HEAD+WAIST], dq[HEAD+WAIST], self.head_waist_sent, dt)
            client.send_joint_targets(targets, {'left_arm': tuple(qtau[:7]), 'right_arm': tuple(qtau[7:]),
                                                'head': tuple(hold[:2]), 'waist': tuple(hold[2:])})
            self.arm_tau_sent = qtau
            self.published += 1

    def _targets(self, q, dt=.02):
        head, waist = head_waist_reference(self.profile)
        reference = np.concatenate((head, waist))
        if self.head_waist_sent is None:
            self.head_waist_sent = reference.copy()
        step = HEAD_WAIST_RATE*dt
        self.head_waist_sent = self.head_waist_sent + np.clip(reference-self.head_waist_sent, -step, step)
        return {'left_arm': tuple(self.arm_sent[:7]), 'right_arm': tuple(self.arm_sent[7:]),
                'head': tuple(self.head_waist_sent[:2]), 'waist': tuple(self.head_waist_sent[2:]),
                'left_ee': (float(np.clip(self.gripper_sent[0], q[31]-.18, q[31]+.18)),),
                'right_ee': (float(np.clip(self.gripper_sent[1], q[33]-.18, q[33]+.18)),)}

    def _ramp_out_torque(self, client, *, hold_feedback):
        """Bring arm feedforward to zero before Runtime stops publishing.

        R1-A7-D Runtime exit keeps the last LowCmd in the motors, so a nonzero
        tau must not be left behind. Only the feedforward changes; position
        targets stay at the last command, or at measured feedback after a fault.
        """
        if not np.any(self.arm_tau_sent) and not np.any(self.hold_ff.output):
            return
        dt = .02
        if hold_feedback:
            q, _, _, _ = self.state()
            with self.lock:
                self.arm_target = self.arm_sent = q[ARM].copy()
                self.gripper_target = self.gripper_sent = q[GRIP].copy()
        self.torque_ff.request_zero(True)
        self.hold_ff.request_zero(True)
        tau = self.arm_tau_sent.copy()
        deadline = time.monotonic() + 3.
        while (np.any(tau) or np.any(self.hold_ff.output)) and time.monotonic() < deadline:
            started = time.monotonic()
            tau = tau - np.clip(tau, -TAU_RAMP_OUT_RATE*dt, TAU_RAMP_OUT_RATE*dt)
            q, dq, _, _ = self.state()
            targets = self._targets(q)
            hold = self.hold_ff.compute(q[HEAD+WAIST], dq[HEAD+WAIST], self.head_waist_sent, dt)
            client.send_joint_targets(targets, {'left_arm': tuple(tau[:7]), 'right_arm': tuple(tau[7:]),
                                                'head': tuple(hold[:2]), 'waist': tuple(hold[2:])})
            self.arm_tau_sent = tau
            self.torque_ff.output = tau.copy()
            time.sleep(max(0., dt-(time.monotonic()-started)))

    def _run(self):
        # All command / request sockets are created, used and closed on this thread.
        client = None
        try:
            from r1d_chess.robot.r1d_runtime import R1DRuntimeClient
            client = R1DRuntimeClient()
            client.connect()
            self._ready.set()
            while self.running:
                started = time.monotonic()
                try:
                    op, event, result = self._requests.get_nowait()
                except queue.Empty:
                    op = None
                if op is not None:
                    try:
                        if op == 'enable':
                            self._begin(client)
                        elif op == 'release_all':
                            self._ramp_out_torque(client, hold_feedback=False)
                            client.end()
                            self.enabled = self.waist_enabled = self.session_active = False
                    except Exception as exc:
                        result['error'] = str(exc)
                    finally:
                        event.set()
                if self.session_active and not self.fault:
                    try:
                        self._publish_once(client, .02)
                    except Exception as exc:
                        self.fault = str(exc)
                        self.enabled = False
                        try:
                            self._ramp_out_torque(client, hold_feedback=True)
                        except Exception as ramp_error:
                            self.tau_ramp_out_error = str(ramp_error)
                        try:
                            client.end()
                        except Exception as stop_error:
                            self.fault_hold_publish_error = str(stop_error)
                        self.waist_enabled = self.session_active = False
                        # No automatic resume, return path, AI restore or hardware-disable claim.
                time.sleep(max(0., .02-(time.monotonic()-started)))
        except Exception as exc:
            self._startup_error = self.fault = str(exc)
            self._ready.set()
        finally:
            if client is not None:
                # Service shutdown (close() / SIGTERM) with a live session: R1-A7-D motors keep the last LowCmd,
                # so never leave feedforward torque behind. Position targets and gains stay (arms keep holding);
                # unloading the arms is a separate, explicit step (client unload_arms).
                if self.session_active and np.any(self.arm_tau_sent):
                    try:
                        self._ramp_out_torque(client, hold_feedback=True)
                    except Exception as ramp_error:
                        self.tau_ramp_out_error = str(ramp_error)
                client.close()

    def clear_fault(self):
        """Only for an explicit operator retract: forget a latched fault so a new session can be entered.
        The entry checks of _begin still run; a still-present hardware problem faults again immediately."""
        if self.session_active:
            raise RuntimeError('a session is still active')
        with self.lock:
            self.fault = None
            self.fault_hold_active = False

    def assert_enabled(self):
        self.state()
        if self.fault or not self.enabled:
            raise RuntimeError(self.fault or 'R1-D arm control not enabled')

    def command_arm(self, q, tau):
        # Commands are interpolated from measured feedback; clip small limit excursions instead of faulting.
        q = self.kinematics.clip_joints(q)
        if np.asarray(tau).shape != (14,) or np.any(np.asarray(tau) != 0):
            raise ValueError('this R1-D adapter uses Runtime PD; external torque is unsupported')
        with self.lock:
            self.assert_enabled()
            self.arm_target = q.copy()

    def command_gripper(self, side, q):
        if side not in ('left', 'right') or not np.isfinite(q) or not 0 <= q <= 5.4:
            raise ValueError('invalid R1-D gripper command')
        with self.lock:
            self.assert_enabled()
            measured, _, _, _ = self.state()
            j = int(side == 'right')
            self.gripper_sent[j] = measured[GRIP[j]]
            self.gripper_target[j] = q

    def stop_gripper(self, side):
        with self.lock:
            j = int(side == 'right')
            q, _, _, _ = self.state()
            self.gripper_sent[j] = self.gripper_target[j] = q[GRIP[j]]

    def hold(self):
        with self.lock:
            q, _, _, _ = self.state()
            self.arm_target = self.arm_sent = q[ARM].copy()
            self.gripper_target = self.gripper_sent.copy()

    def release(self):
        # Finish arm execution, retaining a single owner holding head, waist and final arm targets.
        with self.lock:
            self.enabled = False

    def release_waist(self):
        if self.enabled:
            raise RuntimeError('release arm execution before releasing the whole-body session')
        self._rpc('release_all')

    def close(self, unload_arms=True):
        """Stop the owner thread (ramps feedforward to zero, ends the Runtime session), then - if this service
        ever held a session - unload both arms so they can be hand-guided."""
        held = self.ever_session
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join(timeout=12)
        try:
            if held and unload_arms and self.allow_motion:
                self.unload_result = self.unload_arms()
        finally:
            self.subscriber.Close()
            self.command_subscriber.Close()

    UNLOAD_RAMP_S = 1.5     # unload slowly over about 1-2 s

    def unload_arms(self, frames=25, ramp_s=None):
        """Ramp the arm gains to zero directly on rt/lowcmd, then publish zero-gain frames.

        R1-A7-D motors keep the last LowCmd after the Runtime stops publishing, so arms stay stiff until a frame
        with kp = kd = tau = 0 arrives. kp/kd/tau go linearly from the last commanded values to
        zero over ramp_s (default 1.5 s, 50 Hz) at the current joint angles, so the arms settle slowly instead of
        dropping at once. Only arm slots change; head, waist and grippers keep the last command.
        Refuses if another publisher is still active.
        """
        import copy
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.utils.crc import CRC
        ramp_s = self.UNLOAD_RAMP_S if ramp_s is None else float(ramp_s)
        quiet = time.monotonic()
        time.sleep(.4)
        if self._last_command_at >= quiet:
            return {'unloaded': False, 'error': 'another LowCmd publisher is active; arms not unloaded'}
        with self.lock:
            base = self._last_lowcmd
        if base is None:
            return {'unloaded': False, 'error': 'no previous LowCmd observed to keep head/waist/gripper commands'}
        q, _, msg, _ = self.state()
        start = {i: (float(base.motor_cmd[i].kp), float(base.motor_cmd[i].kd), float(base.motor_cmd[i].tau))
                 for i in ARM}
        crc = CRC()

        def packet_at(scale):
            packet = copy.deepcopy(base)
            packet.mode_machine = msg.mode_machine
            for i in ARM:
                m = packet.motor_cmd[i]
                kp, kd, tau = start[i]
                m.mode = 1
                m.q = float(q[i])
                m.dq = 0.
                m.kp, m.kd, m.tau = kp*scale, kd*scale, tau*scale
            packet.crc = crc.Crc(packet)
            return packet
        steps = max(0, int(round(ramp_s/.02)))
        publisher = ChannelPublisher('rt/lowcmd', LowCmd_)
        publisher.Init()
        try:
            for k in range(1, steps+1):
                publisher.Write(packet_at(1.-k/steps))
                time.sleep(.02)
            zero = packet_at(0.)
            for _ in range(frames):
                publisher.Write(zero)
                time.sleep(.02)
        finally:
            publisher.Close()
        return {'unloaded': True, 'frames': frames, 'ramp_s': ramp_s,
                'start_kp_max': max(v[0] for v in start.values()),
                'arm_q_at_unload': q[ARM].tolist()}


class SimulatedDDS:
    """Explicit offline transport. No Unitree imports, sockets or real camera images."""
    def __init__(self, *args, **kwargs):
        self.lock = threading.RLock()
        self.enabled = self.waist_enabled = self.session_active = False
        self.fault = None
        self.gripper_open_sign = 1
        self.published = 0
        self._q = np.zeros(35)
        self.arm_target = self.arm_sent = np.zeros(14)
        self.gripper_target = self.gripper_sent = np.zeros(2)
        self.waist_target = self.waist_sent = np.zeros(1)
        self.tau = np.zeros(14)
        self.tick = 0
        self._contact = np.zeros(2, dtype=bool)
        self.arm_tau_sent = np.zeros(14)

    def state(self):
        self.tick += 1
        if self.enabled:
            self._q[ARM] = self.arm_target
            # Simulated successful contact at q=.2, never presented as actual hardware.
            self._q[GRIP] = np.where((self.gripper_target == 0) & self._contact, .2, self.gripper_target)
            self.arm_sent = self.arm_target.copy()
            self.gripper_sent = self._q[GRIP].copy()
            self.published += 1
        motors = [SimpleNamespace(q=q, dq=0., tau_est=0., motorstate=0, temperature=[25, 25]) for q in self._q]
        msg = SimpleNamespace(tick=self.tick, mode_machine=0, motor_state=motors,
                              imu_state=SimpleNamespace(rpy=[0., 0., 0.]))
        return self._q.copy(), np.zeros(35), msg, 0.

    def enable(self):
        self.enabled = self.waist_enabled = self.session_active = True

    def clear_fault(self):
        self.fault = None

    def assert_enabled(self):
        if not self.enabled:
            raise RuntimeError('simulation control disabled')

    def command_arm(self, q, tau):
        self.assert_enabled()
        self.arm_target = np.asarray(q).copy()

    def command_gripper(self, side, q):
        self.assert_enabled()
        index = int(side == 'right')
        self.gripper_target[index] = q
        self._contact[index] = q == 0

    def stop_gripper(self, side):
        self.gripper_target[int(side == 'right')] = self._q[GRIP[int(side == 'right')]]

    def hold(self):
        self.arm_target = self.arm_sent.copy()

    def release(self):
        self.enabled = False

    def release_waist(self):
        self.waist_enabled = self.session_active = False

    def unload_arms(self, frames=25):
        return {'unloaded': True, 'simulated': True, 'frames': 0}

    def close(self, unload_arms=True):
        pass
