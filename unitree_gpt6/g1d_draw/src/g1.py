"""G1/Dex1 DDS transport with explicit acquisition and a terminal hold latch.

Construction and ``connect`` never publish commands. ``close`` only releases
channels; it does not move home or provide a powered hold after release.
"""

from __future__ import annotations

import copy
import threading
import time
from types import SimpleNamespace

import numpy as np

from .config import waist_vector

ARM_SLOTS = tuple(range(15, 29))
WAIST_SLOTS = (12, 13)
WAIST_Q3_SLOTS = (12, None, 13)
WAIST_Q3_INDICES = (0, 2)
GRIP_SLOTS = (31, 33)
SIDES = ("left", "right")


def _vector(value, size, label):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{label} must contain {size} finite values")
    return result.copy()


class G1DDSBackend:
    """One command publisher owns both arms, the two-axis waist and internal Dex1.

    Joint vectors use radians and the order left7, right7. Gripper positions
    use raw motor units. Limits and model validation belong to the caller.
    Waist vectors use [yaw, 0, pitch]; only slots 12 and 13 are actuated.
    A stopped instance cannot resume; construct a new instance to reacquire.
    """

    is_simulation = False

    def __init__(self, config: dict):
        self.config = copy.deepcopy(config)
        self.device = self.config["device"]
        self.motion = self.config["motion"]
        self.gains = self.config["gains"]
        self._internal = self.device["gripper_interface"] == "internal"
        self._period = 1.0 / float(self.motion["publish_hz"])
        self._timeout = float(self.motion["state_timeout_s"])
        self._lock = threading.RLock()
        self._wake = threading.Event()
        self._shutdown = threading.Event()
        self._channels = []
        self._samples = {}
        self._received = {}
        self._connected = self._acquired = self._closed = False
        self._acquiring = False
        self._stopped = False
        self._fault = None
        self._thread = None
        self._last_publish = None
        self._last_sent_q = self._last_sent_tau = None
        self._target_q = self._target_tau = None
        self._initial_q = self._initial_tau = None
        self._grip_target = self._waist_target = None
        self._tracking_since = {}
        self._acquisition = None
        self._close_errors = []

    def _fault_locked(self, error):
        if self._fault is None:
            self._fault = str(error)
        self._stop_locked()

    def _stop_locked(self):
        self._stopped = True
        if self._initial_q is not None:
            q = self._initial_q if self._last_sent_q is None else self._last_sent_q
            tau = self._initial_tau if self._last_sent_tau is None else self._last_sent_tau
            self._target_q, self._target_tau = q.copy(), tau.copy()

    def _receive(self, name, message):
        # These timestamps advance only on a valid DDS callback, never on Read().
        try:
            if name == "arm":
                motors = message.motor_state
                sample = {
                    "q14": _vector([motors[i].q for i in ARM_SLOTS], 14, "arm q"),
                    "dq14": _vector([motors[i].dq for i in ARM_SLOTS], 14, "arm dq"),
                    "waist_q3": _vector(
                        [0.0 if slot is None else motors[slot].q for slot in WAIST_Q3_SLOTS],
                        3,
                        "waist q",
                    ),
                    "waist_dq3": _vector(
                        [0.0 if slot is None else motors[slot].dq for slot in WAIST_Q3_SLOTS],
                        3,
                        "waist dq",
                    ),
                    "mode_machine": int(message.mode_machine),
                }
                if self._internal:
                    sample["grip_q2"] = _vector([motors[i].q for i in GRIP_SLOTS], 2, "grip q")
                    sample["grip_dq2"] = _vector([motors[i].dq for i in GRIP_SLOTS], 2, "grip dq")
            else:
                motor = message.states[0]
                sample = _vector([motor.q, motor.dq], 2, f"{name} grip state")
            received = time.monotonic()
            with self._lock:
                if self._closed:
                    return
                self._samples[name] = sample
                self._received[name] = received
                if (
                    name == "arm"
                    and self._acquired
                    and sample["mode_machine"] != self._mode_machine
                ):
                    self._fault_locked("DDS mode_machine changed after acquisition")
        except BaseException as exc:
            with self._lock:
                if self._acquired and not self._closed:
                    self._fault_locked(f"invalid {name} callback: {exc}")
        finally:
            self._wake.set()

    def _fresh_locked(self, now=None):
        now = time.monotonic() if now is None else now
        names = ("arm",) if self._internal else ("arm", *SIDES)
        return all(now - self._received.get(name, -float("inf")) <= self._timeout for name in names)

    def _snapshot_locked(self):
        if not self._fresh_locked():
            raise RuntimeError("missing or stale DDS state callbacks")
        arm = self._samples["arm"]
        result = {key: arm[key].copy() for key in ("q14", "dq14", "waist_q3", "waist_dq3")}
        if self._internal:
            result.update({key: arm[key].copy() for key in ("grip_q2", "grip_dq2")})
        else:
            result["grip_q2"] = np.array([self._samples[side][0] for side in SIDES])
            result["grip_dq2"] = np.array([self._samples[side][1] for side in SIDES])
        return result

    def connect(self):
        """Subscribe and await fresh state; do not create command publishers."""
        with self._lock:
            if self._connected or self._closed or self._channels:
                raise RuntimeError("backend connections cannot be reused")
        try:
            # SDK imports are deliberately confined to the explicit connection.
            from unitree_sdk2py.core.channel import (
                ChannelFactoryInitialize,
                ChannelPublisher,
                ChannelSubscriber,
            )
            from unitree_sdk2py.idl.default import (
                unitree_go_msg_dds__MotorCmd_,
                unitree_hg_msg_dds__LowCmd_,
            )
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
            from unitree_sdk2py.utils.crc import CRC

            self._sdk = SimpleNamespace(
                Publisher=ChannelPublisher,
                Subscriber=ChannelSubscriber,
                LowCmd=LowCmd_,
                LowState=LowState_,
                MotorCmds=MotorCmds_,
                MotorStates=MotorStates_,
                lowcmd=unitree_hg_msg_dds__LowCmd_,
                motorcmd=unitree_go_msg_dds__MotorCmd_,
                CRC=CRC,
            )
            ChannelFactoryInitialize(int(self.device["domain_id"]), self.device["interface"])
            self._subscribe(self.device["lowstate_topic"], LowState_, "arm")
            if not self._internal:
                for side in SIDES:
                    self._subscribe(
                        self.device["external_grippers"][side]["state"], MotorStates_, side
                    )
            deadline = time.monotonic() + float(self.motion["connect_timeout_s"])
            while True:
                with self._lock:
                    if self._closed:
                        raise RuntimeError("connection was closed")
                    if self._fresh_locked():
                        self._connected = True
                        return
                if time.monotonic() >= deadline:
                    raise TimeoutError("timed out awaiting fresh DDS callbacks")
                self._wake.wait(0.01)
                self._wake.clear()
        except BaseException:
            self.close()
            raise

    def _subscribe(self, topic, message_type, name):
        channel = self._sdk.Subscriber(topic, message_type)
        with self._lock:
            if self._closed:
                channel.Close()
                raise RuntimeError("backend was closed during connection")
            self._channels.append(channel)
        channel.Init(lambda message: self._receive(name, message), 10)
        with self._lock:
            if self._closed:
                channel.Close()
                raise RuntimeError("backend was closed during connection")

    def _publisher(self, topic, message_type):
        channel = self._sdk.Publisher(topic, message_type)
        with self._lock:
            if self._closed:
                channel.Close()
                raise RuntimeError("backend was closed during acquisition")
            self._channels.append(channel)
        channel.Init()
        with self._lock:
            if self._closed:
                channel.Close()
                raise RuntimeError("backend was closed during acquisition")
        return channel

    def read(self):
        """Return measured state; a healthy terminal user stop still permits reads."""
        with self._lock:
            if not self._connected or self._closed:
                raise RuntimeError("backend is not connected")
            if self._fault is not None:
                raise RuntimeError(self._fault)
            try:
                return self._snapshot_locked()
            except RuntimeError as exc:
                if self._acquired:
                    self._fault_locked(exc)
                raise

    def acquire(self, grip_q2, waist_q3, tau14):
        """Acquire current arm posture after checking fixed grip and waist goals."""
        grip = _vector(grip_q2, 2, "gripper target")
        waist = waist_vector(waist_q3, "waist target").copy()
        tau = _vector(tau14, 14, "arm feedforward torque")
        with self._lock:
            if self._acquired or self._acquiring or self._stopped or self._closed:
                raise RuntimeError("backend is already acquired, acquiring, closed or stopped")
            self._acquiring = True
        try:
            with self._lock:
                state = self.read()
                if np.max(np.abs(state["grip_q2"] - grip)) > self.motion["grip_tolerance"]:
                    raise ValueError("measured grippers differ from acquisition targets")
                if np.max(np.abs(state["waist_q3"] - waist)) > self.motion["waist_tolerance_rad"]:
                    raise ValueError("measured waist differs from acquisition target")
                self._grip_target, self._waist_target = grip, waist
                self._initial_q, self._initial_tau = state["q14"].copy(), tau.copy()
                acquisition_q = state["q14"].copy()
                self._target_q, self._target_tau = self._initial_q.copy(), tau.copy()
                self._mode_machine = self._samples["arm"]["mode_machine"]
                self._message = self._sdk.lowcmd()
                self._message.mode_pr, self._message.mode_machine = 0, self._mode_machine
                for motor in self._message.motor_cmd:
                    motor.mode = 0
                    motor.q = motor.dq = motor.tau = motor.kp = motor.kd = 0.0
                self._enable_slots(ARM_SLOTS, "arm", self._initial_q)
                self._enable_slots(WAIST_SLOTS, "waist", waist)
                if self._internal:
                    self._enable_slots(GRIP_SLOTS, "gripper", grip)
                self._crc = self._sdk.CRC()
            # Init may block; leave the callback lock available during setup.
            self._arm_publisher = self._publisher(self.device["lowcmd_topic"], self._sdk.LowCmd)
            self._grip_publishers, self._grip_messages = {}, {}
            if not self._internal:
                for index, side in enumerate(SIDES):
                    motor = self._sdk.motorcmd()
                    motor.mode, motor.q, motor.dq, motor.tau = 1, float(grip[index]), 0.0, 0.0
                    motor.kp = float(self.gains["gripper_kp"][index])
                    motor.kd = float(self.gains["gripper_kd"][index])
                    self._grip_messages[side] = self._sdk.MotorCmds([motor])
                    topic = self.device["external_grippers"][side]["command"]
                    self._grip_publishers[side] = self._publisher(topic, self._sdk.MotorCmds)
            with self._lock:
                if self._closed or self._stopped:
                    raise RuntimeError("acquisition was closed or stopped")
                state = self.read()
                if (
                    np.max(np.abs(state["grip_q2"] - grip)) > self.motion["grip_tolerance"]
                    or np.max(np.abs(state["waist_q3"] - waist))
                    > self.motion["waist_tolerance_rad"]
                    or self._samples["arm"]["mode_machine"] != self._mode_machine
                ):
                    raise RuntimeError("gripper, waist or control mode changed during acquisition")
                delta = state["q14"] - acquisition_q
                maximum_delta = float(np.max(np.abs(delta)))
                drift_limit = float(self.motion["tracking_tolerance_rad"])
                self._acquisition = {
                    "initial_q14": acquisition_q.tolist(),
                    "latest_q14": state["q14"].tolist(),
                    "arm_delta_q14": delta.tolist(),
                    "max_arm_delta_rad": maximum_delta,
                    "drift_limit_rad": drift_limit,
                    "rebase_accepted": maximum_delta <= drift_limit,
                }
                if maximum_delta > drift_limit:
                    raise RuntimeError(
                        "arms moved while preparing control acquisition: "
                        f"{maximum_delta:.6f} rad exceeds {drift_limit} rad"
                    )
                # Publisher setup may take time. Start from the latest measured
                # posture rather than commanding the earlier acquisition pose.
                self._initial_q = state["q14"].copy()
                self._target_q = self._initial_q.copy()
                self._acquired, self._acquiring = True, False
                self._thread = threading.Thread(
                    target=self._publish_loop, name="g1-dds-publisher", daemon=True
                )
                self._thread.start()
        except BaseException:
            # No SDK disconnect/home helpers are involved in failure cleanup.
            self.close()
            raise

    def _enable_slots(self, slots, group, q):
        for index, slot in enumerate(slots):
            source_index = WAIST_Q3_INDICES[index] if group == "waist" else index
            motor = self._message.motor_cmd[slot]
            motor.mode, motor.q = 1, float(q[source_index])
            motor.kp = float(self.gains[group + "_kp"][source_index])
            motor.kd = float(self.gains[group + "_kd"][source_index])

    def command(self, q14, tau14):
        q, tau = _vector(q14, 14, "arm target"), _vector(tau14, 14, "arm feedforward torque")
        with self._lock:
            if not self._acquired or self._closed or self._stopped or self._fault:
                raise RuntimeError(self._fault or "backend is not acquired or has stopped")
            if self._thread is None or not self._thread.is_alive():
                self._fault_locked("command publisher is unavailable")
                raise RuntimeError(self._fault)
            self.read()
            self._target_q, self._target_tau = q, tau

    def _check_tracking_locked(self, state, now):
        checks = {
            "waist": (state["waist_q3"], self._waist_target, self.motion["waist_tolerance_rad"]),
            "gripper": (state["grip_q2"], self._grip_target, self.motion["grip_tolerance"]),
        }
        if self._last_sent_q is not None:
            checks["arm"] = (state["q14"], self._last_sent_q, self.motion["tracking_tolerance_rad"])
        for name, (actual, target, tolerance) in checks.items():
            error = float(np.max(np.abs(actual - target)))
            if error > float(tolerance):
                since = self._tracking_since.setdefault(name, now)
                if now - since >= float(self.motion["tracking_window_s"]):
                    raise RuntimeError(f"{name} tracking error {error:.6f} exceeds {tolerance}")
            else:
                self._tracking_since.pop(name, None)

    def _publish_once(self):
        with self._lock:
            if self._shutdown.is_set():
                return
            try:
                state = self._snapshot_locked()
                self._check_tracking_locked(state, time.monotonic())
            except BaseException as exc:
                self._fault_locked(exc)
            if self._last_sent_q is None and self._fault is not None:
                return  # No confirmed published command exists to maintain.
            if self._last_sent_q is None:
                q, tau = self._initial_q.copy(), self._initial_tau.copy()
            elif self._stopped:
                q, tau = self._last_sent_q.copy(), self._last_sent_tau.copy()
            else:
                # A delayed tick never creates a catch-up position jump.
                increment = float(self.motion["max_speed_rad_s"]) * self._period
                delta = self._target_q - self._last_sent_q
                scale = max(1.0, float(np.max(np.abs(delta))) / increment)
                q = self._last_sent_q + delta / scale
                tau = self._target_tau.copy()
            for index, slot in enumerate(ARM_SLOTS):
                motor = self._message.motor_cmd[slot]
                motor.q, motor.dq, motor.tau = float(q[index]), 0.0, float(tau[index])
            self._message.crc = self._crc.Crc(self._message)
            if self._arm_publisher.Write(self._message) is False:
                raise RuntimeError("DDS arm publisher rejected a command")
            # Record immediately: a later external-gripper write may fail.
            self._last_sent_q, self._last_sent_tau = q.copy(), tau.copy()
            for side, publisher in self._grip_publishers.items():
                if publisher.Write(self._grip_messages[side]) is False:
                    raise RuntimeError(f"DDS {side} gripper publisher rejected a command")
            self._last_publish = time.monotonic()

    def _publish_loop(self):
        while not self._shutdown.is_set():
            started = time.monotonic()
            try:
                self._publish_once()
            except BaseException as exc:
                # Keep retrying the same last-sent hold, including after DDS
                # failure. A transport failure must never kill this thread.
                with self._lock:
                    self._fault_locked(f"publisher: {type(exc).__name__}: {exc}")
            self._shutdown.wait(max(0.0, self._period - (time.monotonic() - started)))

    def hold(self):
        """Atomically cancel all unsent motion and latch a terminal user stop."""
        with self._lock:
            self._stop_locked()
        return self.health()

    def health(self):
        with self._lock:
            now = time.monotonic()
            fresh = self._fresh_locked(now)
            publishing = bool(
                self._acquired
                and not self._closed
                and self._thread
                and self._thread.is_alive()
                and self._last_publish is not None
                and now - self._last_publish <= self._timeout
            )
            published = bool(
                self._last_sent_q is not None
                and self._target_q is not None
                and np.allclose(self._last_sent_q, self._target_q, atol=1e-9, rtol=0)
                and np.allclose(self._last_sent_tau, self._target_tau, atol=1e-9, rtol=0)
            )
            holding = False
            if publishing and fresh and self._fault is None and published:
                state = self._snapshot_locked()
                holding = bool(
                    np.max(np.abs(state["q14"] - self._last_sent_q))
                    <= self.motion["tracking_tolerance_rad"]
                    and np.max(np.abs(state["waist_q3"] - self._waist_target))
                    <= self.motion["waist_tolerance_rad"]
                    and np.max(np.abs(state["grip_q2"] - self._grip_target))
                    <= self.motion["grip_tolerance"]
                )
            return {
                "publishing": publishing,
                "telemetry_fresh": fresh,
                "fault": self._fault,
                "stopped": self._stopped,
                "last_sent_q14": None if self._last_sent_q is None else self._last_sent_q.tolist(),
                "published_target": published,
                "holding": holding,
                "connected": self._connected and not self._closed,
                "telemetry_age_s": {name: now - stamp for name, stamp in self._received.items()},
                "acquisition": copy.deepcopy(self._acquisition),
                "close_errors": list(self._close_errors),
            }

    def close(self):
        """Release DDS only. No home trajectory and no powered hold after close."""
        with self._lock:
            if self._closed:
                return
            self._stop_locked()
            self._closed, self._connected = True, False
            self._shutdown.set()
            self._wake.set()
        if (
            self._thread is not None
            and self._thread.ident is not None
            and self._thread is not threading.current_thread()
        ):
            self._thread.join(timeout=2.0)
        for channel in reversed(self._channels):
            try:
                channel.Close()
            except BaseException as exc:
                self._close_errors.append(f"{type(exc).__name__}: {exc}")
