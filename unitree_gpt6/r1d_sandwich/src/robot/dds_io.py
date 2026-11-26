# Copied from r1-ik deploy/dds_io.py (90285d2); only imports and paths changed. Keep in sync by copying; do not restyle.
"""Real R1-D IO for ik_executor (official unitree_sdk2 channels over INTERFACE).

Nothing happens at import. Constructing DdsRobotIO subscribes rt/lowstate and rt/height_state
ONLY; no publisher exists. A publisher is created solely by _arm(capability), which only
Executor.run_real calls after every gate passed (fresh, unexpired, single-use shadow token from a
read-only IO with the same interface/domain and identical parameters, plus human confirmation).
publish() refuses unless given that exact capability; _disarm() revokes it.

Timing: 250 Hz deadlines on time.perf_counter(); sleep until ~1.5 ms before, then spin.
Height: rt/height_state.z is converted as millimetres (z / 1000). The raw value is kept in
height_raw so the shadow report can be checked against a known height (checked: 329.2 -> 0.329 m).
"""
from __future__ import annotations

import hmac
import sys
import threading
import time

import numpy as np


from robot.ik_executor import (  # noqa: E402
    GROUPS, GRIPPER_MOTORS, HEAD_MOTORS, I_LIFT, I_WAIST, INTERFACE, L_SL, R_SL, WAIST_MOTOR, LIFT_VMAX,
)

SPIN_WINDOW = 0.0015


def precise_wait(deadline):
    while True:
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            return
        if remaining > SPIN_WINDOW:
            time.sleep(remaining - SPIN_WINDOW)


class DdsRobotIO:
    def __init__(self, interface=INTERFACE, domain=0, start_timeout=3.0):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_

        self.interface, self.domain = interface, int(domain)
        self._init_state()
        ChannelFactoryInitialize(self.domain, interface)
        self._sub = ChannelSubscriber("rt/lowstate", LowState_)
        self._sub.Init(self._on_state, 10)
        self._hsub = ChannelSubscriber("rt/height_state", Point32_)
        self._hsub.Init(self._on_height, 10)
        end = time.perf_counter() + start_timeout
        while (self._state is None or self.height_raw is None) and time.perf_counter() < end:
            time.sleep(0.01)
        if self._state is None or self.height_raw is None:
            raise TimeoutError("no rt/lowstate or rt/height_state within start_timeout")

    def _init_state(self):
        self._lock = threading.Lock()
        self._state, self._stamp = None, None
        self.height_raw = None
        self._pub = None
        self._lift_pub = None
        self._cap = None
        self.published = 0

    # --- capability gate (only Executor.run_real calls these) ---
    @property
    def armed(self):
        return self._cap is not None

    def _make_publisher(self):
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        pub.Init()
        return pub

    def _make_lift_publisher(self):
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Twist_
        pub = ChannelPublisher("rt/cmd_vel", Twist_)
        pub.Init()
        return pub

    def _arm(self, capability):
        if not isinstance(capability, bytes) or len(capability) < 16:
            raise PermissionError("invalid capability")
        if self._pub is None:
            self._pub = self._make_publisher()
        if self._lift_pub is None:
            self._lift_pub = self._make_lift_publisher()
        self._cap = capability

    def _disarm(self):
        self._cap = None

    def publish(self, msg, capability=None):
        if self._cap is None or not isinstance(capability, bytes) \
                or not hmac.compare_digest(capability, self._cap):
            raise PermissionError("publish refused: IO not armed by Executor.run_real")
        self._pub.Write(msg)
        self.published += 1

    def publish_lift(self, vnorm, capability=None):
        """Lift velocity on rt/cmd_vel (normalised, |v| <= ik_executor.LIFT_VMAX, chassis zero).

        Same message as r1_d lift_command (Twist, only linear.z set) but bounded by LIFT_VMAX: r1_d
        hard-caps at 0.1 (~4.4 mm/s), which is too slow; r1_d itself is left unchanged."""
        if self._cap is None or not isinstance(capability, bytes) \
                or not hmac.compare_digest(capability, self._cap):
            raise PermissionError("publish refused: IO not armed by Executor.run_real")
        from unitree_sdk2py.idl.default import geometry_msgs_msg_dds__Twist_
        vnorm = float(vnorm)
        if not np.isfinite(vnorm) or abs(vnorm) > LIFT_VMAX + 1e-9:
            raise ValueError(f"lift velocity {vnorm} outside [-{LIFT_VMAX}, {LIFT_VMAX}]")
        msg = geometry_msgs_msg_dds__Twist_()
        msg.linear.z = vnorm  # all chassis components remain zero
        self._lift_pub.Write(msg)
        self.published_lift = getattr(self, "published_lift", 0) + 1

    # --- feedback ---
    def _on_state(self, msg):
        with self._lock:
            self._state, self._stamp = msg, time.perf_counter()

    def _on_height(self, msg):
        with self._lock:
            self.height_raw = float(msg.z)

    def identity(self):
        return ("dds", self.interface, self.domain)

    def now(self):
        return time.perf_counter()

    def read(self):
        with self._lock:
            return self._state, time.perf_counter() - self._stamp

    def measured_q16(self):
        with self._lock:
            ms, h = self._state.motor_state, self.height_raw / 1000.0   # mm -> m
        q = np.zeros(16)
        q[I_LIFT], q[I_WAIST] = h, ms[WAIST_MOTOR].q
        q[L_SL] = [ms[i].q for i in GROUPS["left_arm"]]
        q[R_SL] = [ms[i].q for i in GROUPS["right_arm"]]
        return q

    def hold_extra(self):
        with self._lock:
            ms = self._state.motor_state
            return {i: float(ms[i].q) for i in HEAD_MOTORS + GRIPPER_MOTORS}

    def wait_until(self, deadline):
        precise_wait(deadline)
