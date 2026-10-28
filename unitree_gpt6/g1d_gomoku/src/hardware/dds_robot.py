"""G1 lowcmd/Dex1 transport. Connecting subscribes only; publishing is explicit."""

import threading
import time

import numpy as np

from .common import positive, vector


class StateSubscriber:
    """Own a nonblocking DDS reader and stop its worker before releasing it.

    The SDK's listener retains callbacks during reader destruction. Avoid that
    callback lifecycle entirely; do not hide SDK errors or patch SDK internals.
    """

    def __init__(self, topic, message_type, participant):
        from cyclonedds.sub import DataReader
        from cyclonedds.topic import Topic

        self._topic = Topic(participant, topic, message_type)
        self._reader = DataReader(participant, self._topic)
        self._stop = threading.Event()
        self._thread = None
        self.error = None

    def Init(self, callback, queue=1):
        if self._thread is not None:
            raise RuntimeError("State subscriber already initialized")
        self._thread = threading.Thread(
            target=self._receive, args=(callback,), name="g1-state-reader", daemon=True
        )
        self._thread.start()

    def _receive(self, callback):
        from cyclonedds.internal import InvalidSample

        try:
            while not self._stop.is_set():
                # take() is nonblocking, unlike the SDK's Read()/take_one().
                samples = self._reader.take(32)
                for sample in reversed(samples):
                    if not isinstance(sample, InvalidSample):
                        callback(sample)
                        break
                self._stop.wait(0.002)
        except Exception as exc:
            self.error = exc

    def Close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        # CycloneDDS entity lifetime follows Python references. No listener or
        # receiving thread can still touch these entities at this point.
        self._reader = None
        self._topic = None


class DDSRobot:
    WAIST_INDICES = (12, 13, 14)  # yaw, roll, pitch (mode_pr=0)

    def __init__(self, config):
        self.config = config
        interface = config.get("network_interface")
        if not isinstance(interface, str) or not interface.strip():
            raise ValueError("Set robot.network_interface to the DDS network interface")
        try:
            from unitree_gpt6.g1d_gomoku.src.unitree_sdk2py.core.channel import (
                ChannelFactoryInitialize,
                ChannelPublisher,
            )
            from unitree_gpt6.g1d_gomoku.src.unitree_sdk2py.idl.default import (
                unitree_go_msg_dds__MotorCmd_,
                unitree_hg_msg_dds__LowCmd_,
            )
            from unitree_gpt6.g1d_gomoku.src.unitree_sdk2py.idl.unitree_go.msg.dds_ import (
                MotorCmds_,
                MotorStates_,
            )
            from unitree_gpt6.g1d_gomoku.src.unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
                LowCmd_,
                LowState_,
            )
            from unitree_gpt6.g1d_gomoku.src.unitree_sdk2py.utils.crc import CRC
        except ImportError as exc:
            raise RuntimeError(
                "Install this project with its gomoku-hardware extra to load the bundled DDS SDK"
            ) from exc
        from unitree_gpt6.g1d_gomoku.models.g1.config import G1_29_JointArmIndex, G1ArmConfig

        self.arm_indices = [int(i) for i in G1_29_JointArmIndex]
        self.gains = G1ArmConfig()
        self._Publisher, self._Subscriber = ChannelPublisher, self._make_subscriber
        self._participant = None
        self._receivers = {}
        self._LowCmd, self._LowState = LowCmd_, LowState_
        self._MotorCmds, self._MotorStates = MotorCmds_, MotorStates_
        self._new_lowcmd, self._new_motorcmd = (
            unitree_hg_msg_dds__LowCmd_,
            unitree_go_msg_dds__MotorCmd_,
        )
        self._crc = CRC()
        self._lock = threading.Lock()
        self._state = {}
        self._channels = []
        self._publishers = {}
        self._subscribed = set()
        self._message = None
        self._waist_reference = None
        self._last_gripper = {}
        self.timeout = positive(config["state_timeout"], "state_timeout")
        self.connect_timeout = positive(config["connect_timeout"], "connect_timeout")
        self.rate_hz = positive(config["rate_hz"], "rate_hz")
        self.gripper_speed = positive(config["gripper_speed"], "gripper_speed")
        ChannelFactoryInitialize(int(config.get("domain_id", 0)), interface)

    def _store(self, key, value):
        with self._lock:
            self._state[key] = (value, time.monotonic())

    def _get(self, key):
        error = getattr(self._receivers.get(key), "error", None)
        if error is not None:
            raise RuntimeError(f"DDS receive failed for {key}: {error}") from error
        with self._lock:
            entry = self._state.get(key)
        if entry is None or time.monotonic() - entry[1] > self.timeout:
            raise TimeoutError(f"Missing or stale DDS state: {key}")
        return entry[0]

    def _make_subscriber(self, topic, message_type):
        from cyclonedds.domain import DomainParticipant

        if self._participant is None:
            # Share the domain/interface already configured by the SDK factory.
            self._participant = DomainParticipant(int(self.config.get("domain_id", 0)))
        return StateSubscriber(topic, message_type, self._participant)

    def _subscribe(self, key, topic, message_type):
        if key in self._subscribed:
            return
        channel = self._Subscriber(topic, message_type)
        self._channels.append(channel)
        self._receivers[key] = channel
        channel.Init(lambda msg: self._store(key, msg), 1)
        self._subscribed.add(key)

    def connect(self, grippers=()):
        self._subscribe("arm", self.config["topic_low_state"], self._LowState)
        for side in grippers:
            self._subscribe(side, self.config["grippers"][side]["state_topic"], self._MotorStates)
        deadline = time.monotonic() + self.connect_timeout
        for key in ("arm", *grippers):
            while True:
                try:
                    self._get(key)
                    break
                except TimeoutError:
                    if time.monotonic() >= deadline:
                        raise TimeoutError(
                            f"No DDS state for {key}; check interface and topic"
                        ) from None
                    time.sleep(0.01)
        return self

    def arm_q(self):
        message = self._get("arm")
        return vector([message.motor_state[i].q for i in self.arm_indices], 14, "measured arm q")

    def waist_q(self):
        message = self._get("arm")
        return vector([message.motor_state[i].q for i in self.WAIST_INDICES], 3, "waist q")

    def lock_waist(self, reference=None):
        """Latch once per connection lifetime; never follow later feedback drift."""
        if reference is None:
            reference = self.waist_q() if self._waist_reference is None else self._waist_reference
        reference = vector(reference, 3, "locked waist yaw/roll/pitch")
        if self._waist_reference is not None and not np.array_equal(
            reference, self._waist_reference
        ):
            raise ValueError("Cannot change locked waist reference during a session")
        self._waist_reference = reference.copy()
        return self._waist_reference.copy()

    def _apply_waist_lock(self, message):
        for index, angle in zip(self.WAIST_INDICES, self._waist_reference, strict=False):
            motor = message.motor_cmd[index]
            motor.mode, motor.q = 1, float(angle)
            motor.dq, motor.tau = 0.0, 0.0
            motor.kp, motor.kd = self.gains.kp_high, self.gains.kd_high

    def gripper_q(self, side):
        return float(vector([self._get(side).states[0].q], 1, f"{side} gripper")[0])

    def motor_health(self):
        state = self._get("arm")
        return [
            {
                "index": i,
                "temperature": list(state.motor_state[i].temperature),
                "motorstate": int(state.motor_state[i].motorstate),
            }
            for i in (*self.WAIST_INDICES, *self.arm_indices)
        ]

    def check_motor_health(self):
        # Temperature is telemetry only. Device-reported faults remain distinct.
        for motor in self.motor_health():
            if motor["motorstate"]:
                raise RuntimeError(f"Motor fault protection: {motor}")

    def _publisher(self, key, topic, message_type):
        if key not in self._publishers:
            channel = self._Publisher(topic, message_type)
            self._channels.append(channel)
            channel.Init()
            self._publishers[key] = channel
        return self._publishers[key]

    def begin_motion(self, grippers=()):
        self.connect(grippers)
        self.check_motor_health()
        self.lock_waist()
        self._publisher("arm", self.config["topic_low_command"], self._LowCmd)
        for side in grippers:
            self._publisher(side, self.config["grippers"][side]["command_topic"], self._MotorCmds)
        state = self._get("arm")
        msg = self._new_lowcmd()
        msg.mode_pr = 0
        msg.mode_machine = state.mode_machine
        wrists = {19, 20, 21, 26, 27, 28}
        weak = {4, 10, 15, 16, 17, 18, 22, 23, 24, 25}
        # Match original pure upper-body mode: hold legs/waist at measured positions.
        for i in range(29):
            motor = msg.motor_cmd[i]
            motor.mode = 1
            motor.q = float(state.motor_state[i].q)
            motor.dq = 0.0
            motor.tau = 0.0
            if i in wrists:
                motor.kp, motor.kd = self.gains.kp_wrist, self.gains.kd_wrist
            elif i in weak:
                motor.kp, motor.kd = self.gains.kp_low, self.gains.kd_low
            else:
                motor.kp, motor.kd = self.gains.kp_high, self.gains.kd_high
        self._message = msg
        self._apply_waist_lock(msg)
        self._last_gripper = {side: self.gripper_q(side) for side in grippers}

    def send(self, q, tau, grippers=None):
        if self._message is None:
            raise RuntimeError("begin_motion() required")
        state = self._get("arm")
        if state.mode_machine != self._message.mode_machine:
            raise RuntimeError("Robot mode_machine changed during motion")
        self.check_motor_health()
        q, tau = vector(q, 14, "arm q"), vector(tau, 14, "arm tau")
        # Reject every invalid gripper command before publishing the arm frame.
        # DDS transport remains non-atomic, but bad input must not partially act.
        pending_grippers = {}
        for side, target in (grippers or {}).items():
            if side not in self._last_gripper or side not in self._publishers:
                raise ValueError(f"begin_motion() did not enable the {side} gripper")
            target = float(vector([target], 1, f"{side} gripper target")[0])
            settings = self.config["grippers"][side]
            lower, upper = vector([settings["min"], settings["max"]], 2, "gripper limits")
            if not 0 <= lower < upper <= 5.45 or not lower <= target <= upper:
                raise ValueError(f"{side} gripper target is outside configured Dex1 limits")
            current = self.gripper_q(side)
            delta = self.gripper_speed / self.rate_hz
            limited = float(
                np.clip(target, self._last_gripper[side] - delta, self._last_gripper[side] + delta)
            )
            limited = float(np.clip(limited, current - 0.18, current + 0.18))
            msg = self._MotorCmds()
            motor = self._new_motorcmd()
            motor.q, motor.dq, motor.tau = limited, 0.0, 0.0
            motor.kp, motor.kd = 10.0, 0.05
            msg.cmds = [motor]
            pending_grippers[side] = (msg, limited)
        for j, i in enumerate(self.arm_indices):
            self._message.motor_cmd[i].q = float(q[j])
            self._message.motor_cmd[i].tau = float(tau[j])
        # Reinstate all waist fields on every frame, including holds/recovery.
        self._apply_waist_lock(self._message)
        self._message.crc = self._crc.Crc(self._message)
        if self._publishers["arm"].Write(self._message) is False:
            raise RuntimeError("DDS arm command write failed")
        for side, (msg, limited) in pending_grippers.items():
            if self._publishers[side].Write(msg) is False:
                raise RuntimeError(f"DDS {side} gripper command write failed")
            self._last_gripper[side] = limited

    def close(self):
        for channel in reversed(self._channels):
            channel.Close()
        self._channels.clear()
        self._publishers.clear()
        self._subscribed.clear()
        self._receivers.clear()
        self._participant = None
        with self._lock:
            self._state.clear()
        self._message = None
        self._waist_reference = None
