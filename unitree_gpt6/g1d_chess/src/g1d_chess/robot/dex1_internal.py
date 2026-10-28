"""G1 HG DDS transport for internally wired Dex1 (motors 31 and 33).

Protocol reference: xr_teleoperate/teleop/robot_control/robot_arm.py,
G1_29_Arm_Internal_Dex1_Controller. Gains/arm indices follow unitree-deploy.
Connection is read-only; enable() starts holding the measured configuration.
Both internal Dex1 joints default to +1 opening direction: positive q opens,
q=0 closes. The sign can be overridden for another joint convention.
"""
import threading
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

ARM = list(range(15, 29))
GRIP = [31, 33]
WAIST = [12, 13, 14]  # yaw, roll, pitch; legs always remain disabled
WRIST = {19, 20, 21, 26, 27, 28}


class InternalDDS:
    def __init__(self, interface, timeout=5.0, gripper_open_sign=1,
                 waist_kp=300.0, waist_kd=3.0):
        self.lock = threading.RLock()
        self.msg = None
        self.received_at = 0.0
        self.last_tick = None
        self.enabled = False
        self.waist_enabled = False
        # G1_29_Arm_Internal_Dex1_Controller uses these gains for all three
        # waist motors (none is a weak motor). Do not command any leg motor.
        if not np.isfinite([waist_kp, waist_kd]).all() or not 0 < waist_kp <= 300 or not 0 < waist_kd <= 3:
            raise ValueError("waist gains must be finite: 0 < kp <= 300, 0 < kd <= 3")
        self.waist_kp, self.waist_kd = float(waist_kp), float(waist_kd)
        self.waist_target = np.zeros(3)
        self.waist_sent = np.zeros(3)
        self.fault = None
        self.fault_hold_active = False
        self.fault_hold_publish_error = None
        self.fault_hold_last_published_at = None
        self._last_published_targets = None
        self._last_published_modes = None
        self._fault_hold_targets = None
        self.running = True
        self.publisher = None
        self.published = 0
        if gripper_open_sign not in (-1, 1):
            raise ValueError("gripper open sign must be -1 or +1")
        self.gripper_open_sign = gripper_open_sign
        ChannelFactoryInitialize(0, interface)
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self._receive, 1)
        deadline = time.monotonic() + timeout
        while self.msg is None and time.monotonic() < deadline:
            time.sleep(.01)
        self.state()
        self.thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.thread.start()

    def _receive(self, msg):
        with self.lock:
            if msg.tick != self.last_tick:
                self.msg = msg
                self.last_tick = msg.tick
                self.received_at = time.monotonic()

    def state(self):
        with self.lock:
            age = time.monotonic() - self.received_at
            if self.msg is None or age > .3:
                raise RuntimeError(f"lowstate missing/stale ({age:.3f}s)")
            q = np.array([m.q for m in self.msg.motor_state])
            dq = np.array([m.dq for m in self.msg.motor_state])
            controlled = ARM + GRIP + (WAIST if self.waist_enabled else [])
            if not np.all(np.isfinite(q[controlled])) or not np.all(np.isfinite(dq[controlled])):
                raise RuntimeError("non-finite motor state")
            if self.waist_enabled:
                self._check_waist_feedback(q, dq, self.msg)
            return q, dq, self.msg, age

    @staticmethod
    def _check_waist_feedback(q, dq, state):
        if not np.isfinite(q[WAIST]).all() or not np.isfinite(dq[WAIST]).all():
            raise RuntimeError("non-finite waist motor state")
        bad = {i: int(state.motor_state[i].motorstate) for i in WAIST
               if int(state.motor_state[i].motorstate) != 0}
        if bad:
            raise RuntimeError(f"waist motorstate is nonzero: {bad}")

    def _ensure_publisher(self, state):
        if self.publisher is not None:
            return
        self.command = unitree_hg_msg_dds__LowCmd_()
        self.command.mode_pr = 0
        self.command.mode_machine = state.mode_machine
        self.crc = CRC()
        self.publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.publisher.Init()

    def _set_motor_modes(self):
        """Rebuild ownership every tick, including immediately after release."""
        for i, motor in enumerate(self.command.motor_cmd):
            arm_active = self.enabled and i in ARM + GRIP
            waist_active = self.waist_enabled and i in WAIST
            motor.mode = int(arm_active or waist_active)
            motor.dq = 0.0
            if waist_active:
                motor.kp, motor.kd = self.waist_kp, self.waist_kd
            elif arm_active:
                if i in GRIP:
                    motor.kp, motor.kd = 5.0, .05
                elif i in WRIST:
                    motor.kp, motor.kd = 40.0, 1.5
                else:
                    motor.kp, motor.kd = 80.0, 3.0
            else:
                motor.q = motor.tau = motor.kp = motor.kd = 0.0

    def _write_command(self):
        self._set_motor_modes()
        self.command.crc = self.crc.Crc(self.command)
        if self.publisher.Write(self.command) is False:
            raise RuntimeError("DDS command publication failed")
        self.published += 1
        targets = np.array([(m.q, m.tau) for m in self.command.motor_cmd], dtype=float)
        if np.isfinite(targets).all():
            self._last_published_targets = targets
            self._last_published_modes = np.array([bool(m.mode) for m in self.command.motor_cmd])

    def _freeze_fault_targets(self, feedback=None):
        """Latch one hold target; recovered feedback must not restart old ramps."""
        self.fault_hold_active = True
        active = (ARM + GRIP if self.enabled else []) + (WAIST if self.waist_enabled else [])
        if (self._last_published_targets is not None and
                self._last_published_modes[active].all()):
            self._fault_hold_targets = self._last_published_targets.copy()
        elif feedback is not None:
            # This feedback passed state() during the failed publication tick.
            # No prior published packet exists, so hold that measured position.
            targets = np.zeros((len(self.command.motor_cmd), 2))
            targets[active, 0] = feedback[active]
            if self._last_published_targets is not None:
                previous = [i for i in active if self._last_published_modes[i]]
                targets[previous] = self._last_published_targets[previous]
            self._fault_hold_targets = targets
        if self._fault_hold_targets is None:
            self.fault_hold_publish_error = "no finite published target or valid feedback available for hold"

    def _publish_fault_hold(self):
        """Retry only the latched hold packet, with current explicit ownership."""
        targets = self._fault_hold_targets
        if targets is None:
            return
        if self.enabled:
            self.arm_sent = targets[ARM, 0].copy()
            self.arm_target = self.arm_sent.copy()
            self.gripper_sent = targets[GRIP, 0].copy()
            self.gripper_target = self.gripper_sent.copy()
            self.tau = targets[ARM, 1].copy()
        if self.waist_enabled:
            self.waist_sent = targets[WAIST, 0].copy()
            self.waist_target = self.waist_sent.copy()
        for i in (ARM + GRIP if self.enabled else []) + (WAIST if self.waist_enabled else []):
            self.command.motor_cmd[i].q = float(targets[i, 0])
            self.command.motor_cmd[i].tau = float(targets[i, 1])
        try:
            self._write_command()
        except Exception as exc:
            self.fault_hold_publish_error = str(exc)
        else:
            # A successful write is not proof that the hardware held position.
            self.fault_hold_publish_error = None
            self.fault_hold_last_published_at = time.monotonic()

    def enable_waist_zero(self):
        with self.lock:
            if self.fault:
                raise RuntimeError(self.fault)
            if self.waist_enabled:
                return
            q, dq, state, _ = self.state()
            self._check_waist_feedback(q, dq, state)
            self.waist_sent = q[WAIST].copy()
            self.waist_target = np.zeros(3)
            self._ensure_publisher(state)
            self.waist_enabled = True
            for j, i in enumerate(WAIST):
                self.command.motor_cmd[i].q = float(self.waist_sent[j])
                self.command.motor_cmd[i].tau = 0.0
            self._set_motor_modes()

    def release_waist(self):
        with self.lock:
            self.waist_enabled = False
            if not self.enabled:
                self.fault_hold_active = False
            if self.publisher is not None:
                self._write_command()

    def release(self):
        """Release arms/grippers while an explicitly enabled waist hold persists."""
        with self.lock:
            self.enabled = False
            if not self.waist_enabled:
                self.fault_hold_active = False
            if self.publisher is not None:
                self._write_command()

    def enable(self):
        with self.lock:
            if self.fault:
                raise RuntimeError(self.fault)
            if self.enabled:
                return
            q, _, state, _ = self.state()
            self.arm_target = q[ARM].copy()
            self.gripper_target = q[GRIP].copy()
            self.arm_sent = self.arm_target.copy()
            self.gripper_sent = self.gripper_target.copy()
            self.tau = np.zeros(14)
            self._ensure_publisher(state)
            for i in ARM + GRIP:
                self.command.motor_cmd[i].q = float(q[i])
                self.command.motor_cmd[i].tau = 0.0
            self.enabled = True
            self._set_motor_modes()

    def command_arm(self, q, tau):
        q, tau = np.asarray(q, dtype=float), np.asarray(tau, dtype=float)
        if q.shape != (14,) or tau.shape != (14,) or not np.all(np.isfinite([q, tau])):
            raise ValueError("arm command requires finite 14-dimensional q and tau")
        with self.lock:
            self.assert_enabled()
            self.arm_target = np.asarray(q, dtype=float).copy()
            self.tau = np.asarray(tau, dtype=float).copy()

    def command_gripper(self, side, q):
        if side not in ("left", "right") or not np.isfinite(q) or not 0 <= q*self.gripper_open_sign <= 5.4:
            raise ValueError(f"gripper q must be between 0 and {5.4*self.gripper_open_sign}")
        with self.lock:
            self.assert_enabled()
            measured, _, _, _ = self.state()
            j = 0 if side == "left" else 1
            # Contact can hold the measured aperture far from the internal
            # ramp. Start an explicit new action at feedback so reopening
            # does not spend seconds unwinding a blocked closing command.
            # The publish loop still applies its velocity and feedback clamps.
            self.gripper_sent[j] = measured[GRIP[j]]
            self.gripper_target[j] = q

    def stop_gripper(self, side):
        j = 0 if side == "left" else 1
        with self.lock:
            q, _, _, _ = self.state()
            self.gripper_sent[j] = self.gripper_target[j] = q[GRIP[j]]

    def assert_enabled(self):
        self.state()
        if self.fault or not self.enabled:
            raise RuntimeError(self.fault or "control not enabled")

    def hold(self):
        with self.lock:
            q, _, _, _ = self.state()
            if self.enabled:
                self.arm_target = q[ARM].copy()
                self.arm_sent = self.arm_target.copy()
                self.gripper_target = self.gripper_sent.copy()
                # Keep gripper force and aperture: do not drop an object on abort.

    def _publish_once(self, dt=.004):
        with self.lock:
            if self.enabled or self.waist_enabled:
                if self.fault:
                    if not self.fault_hold_active:
                        self._freeze_fault_targets()
                    self._publish_fault_hold()
                    return
                q = None
                try:
                    q, _, _, _ = self.state()
                    if self.waist_enabled:
                        self.waist_sent += np.clip(self.waist_target - self.waist_sent, -.1*dt, .1*dt)
                        for j, i in enumerate(WAIST):
                            self.command.motor_cmd[i].q = float(self.waist_sent[j])
                            self.command.motor_cmd[i].tau = 0.0
                    if self.enabled:
                        # Bounded commanded velocity, including each first command.
                        # Tracking offsets alone do not fault or release control;
                        # state freshness/finite and hardware health checks remain.
                        self.arm_sent += np.clip(self.arm_target - self.arm_sent, -.3*dt, .3*dt)
                        self.gripper_sent += np.clip(self.gripper_target - self.gripper_sent, -.8*dt, .8*dt)
                        for j, i in enumerate(ARM):
                            self.command.motor_cmd[i].q = float(self.arm_sent[j])
                            self.command.motor_cmd[i].tau = float(self.tau[j])
                        for j, i in enumerate(GRIP):
                            self.command.motor_cmd[i].q = float(np.clip(self.gripper_sent[j], q[i]-.18, q[i]+.18))
                    self._write_command()
                except Exception as exc:
                    self.fault = str(exc)
                    self._freeze_fault_targets(q)
                    self._publish_fault_hold()

    def _publish_loop(self):
        dt = .004
        while self.running:
            start = time.monotonic()
            self._publish_once(dt)
            time.sleep(max(0, dt - (time.monotonic() - start)))

    def close(self):
        self.running = False
        self.thread.join(timeout=1)
        self.subscriber.Close()
        if self.publisher is not None:
            self.publisher.Close()
