# -*- coding: utf-8 -*-
"""g1d.py -- G1D wheeled chassis motion controller (agv RPC + open-loop integrated odometry).

RobotController: real-robot mode, drives the G1D chassis via the unitree_sdk2py agv RPC
service (AgvClient.Move velocity stream).

G1D is a wheeled chassis (not the G1 humanoid):
  - Move(vx, 0, vyaw): velocity stream, vx in [-1.5, 1.5] m/s (forward +),
    vyaw in [-0.6, 0.6] rad/s (ccw +); vy is always 0 (no strafing). Waist/arms/hands use
    low-level interfaces and are out of scope here.
  - No measured localization is used -> position/heading are integrated open-loop;
    services.navigation can subscribe to SLAM pose/velocity separately.
  - rt/hispeed_state (column height) is subscribed only for connection detection; the height
    value does not feed odometry.
"""

import math
import threading
import time
from typing import Any

import config

# Unitree SDK + agv RPC client (only needed in real-robot mode)
try:
    from unitree_sdk2py.core.channel import (  # type: ignore[reportMissingImports]
        ChannelFactoryInitialize,
        ChannelSubscriber,
    )
    from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_  # type: ignore[reportMissingImports]

    from services.g1_agv_client import (  # type: ignore[reportMissingImports]
        AGV_MAX_VX,
        AGV_MAX_VYAW,
        AgvClient,
    )
    SDK_AVAILABLE = True
except ImportError:
    ChannelFactoryInitialize: Any = None
    ChannelSubscriber: Any = None
    Point32_: Any = None
    AgvClient: Any = None
    AGV_MAX_VX = 1.5
    AGV_MAX_VYAW = 0.6
    SDK_AVAILABLE = False


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class RobotController:
    """Controls the G1D chassis via the unitree_sdk2py agv RPC service.

    Responsibilities:
    - 20 Hz loop streaming the current velocity target (vx, vyaw) via AgvClient.Move
    - drop to zero speed when there is no command / the watchdog times out
    - maintain open-loop odometry self.odom (position[x,y,z] + yaw; this manual path never
      reads measured odom)
    - compatible send_command(vx, vy, ang_z, ...) interface (G1D ignores vy/waist_*)"""

    model = "g1d"

    def __init__(self, net=None, domain_id=0):
        # Present whether or not the SDK is available (used by tests / watchdog)
        self._control_lock = threading.RLock()
        self._navigation_owned = False
        self.cmd_lock = threading.Lock()            # guards current_cmd / _last_cmd_time
        self.current_cmd = None                     # pending command (vx, vyaw)
        self.publish_running = False
        self._last_cmd_time = None                  # monotonic time of last send_command (watchdog)

        # Open-loop odometry: position/yaw integrated from velocity x time
        self.odom: dict[str, Any] = {"position": [0.0, 0.0, 0.0], "yaw": 0.0}
        self._odom_lock = threading.Lock()

        if not SDK_AVAILABLE:
            self._available = False
            self._connected = False
            self.dt = 1.0 / config.RATE
            return

        self._available = True
        ChannelFactoryInitialize(domain_id, net or config.NET)

        self.agv = AgvClient()
        self.agv.SetTimeout(0.3)
        self.agv.Init()

        # Column height topic (rt/hispeed_state, Point32 y = height in m), connection detection only
        self._height = None
        self._height_sub = ChannelSubscriber("rt/hispeed_state", Point32_)
        self._height_sub.Init(self._height_cb, 1)

        self.dt = 1.0 / config.RATE                 # control period (s); 20 Hz recommended for G1D Move RPC
        self._connected = False

    def _height_cb(self, m):
        """Column height callback: receiving data only marks the robot as connected."""
        self._height = float(m.y)
        self._connected = True

    @property
    def connected(self):
        return self._connected

    @property
    def available(self):
        return self._available

    def wait_for_data(self):
        """Block until the first robot data (column height) or first successful Move (connection confirmed)."""
        if not self._available:
            raise RuntimeError("unitree_sdk2py 不可用")
        while not self._connected:
            time.sleep(0.05)
        self._connected = True

    def send_command(self, vx=0.0, vy=0.0, ang_z=0.0,
                     waist_yaw=0.0, waist_pitch=0.0, waist_roll=0.0, height=0.74):
        """Set the chassis velocity command.

        G1D supports only vx (forward linear speed) and ang_z (yaw rate -> vyaw).
        vy / waist_* / height are kept for the common send_command signature and ignored."""
        if not self._available:
            return
        vx = _clamp(float(vx), -AGV_MAX_VX, AGV_MAX_VX)
        vyaw = _clamp(float(ang_z), -AGV_MAX_VYAW, AGV_MAX_VYAW)
        with self._control_lock:
            if self._navigation_owned:
                raise RuntimeError("地图导航占用底盘，禁止并行速度流")
            with self.cmd_lock:
                self.current_cmd = (vx, vyaw)
                self._last_cmd_time = time.monotonic()          # watchdog: command freshness

    # Watchdog: stale command -> zero speed -> release
    def _resolve_payload(self, now: float):
        """Per-tick publish decision: returns (vx, vyaw) to publish, None = skip this tick.

        Watchdog: more than config.CMD_WATCHDOG_S since the last send_command -> publish zero
        speed, so a crashed/hung upper layer cannot leave the robot executing the last command.
        Release: after another config.ZERO_RELEASE_S of zero speed, stop sending. agv Move is a
        non-latching velocity servo; a continuous Move(0,0,0) stream actively holds the chassis
        (cannot be pushed by hand), while after sending stops the chassis frees within ~100 ms.
        The next send_command resumes publishing."""
        with self.cmd_lock:
            cmd = self.current_cmd
            last = self._last_cmd_time
        if cmd is None or last is None:
            return None
        idle = now - last
        if idle > config.CMD_WATCHDOG_S + config.ZERO_RELEASE_S:
            return None
        if idle > config.CMD_WATCHDOG_S:
            return (0.0, 0.0)
        return cmd

    def _integrate_odom(self, vx: float, vyaw: float, dt: float):
        """Open-loop integration: world x/y advance along the current heading; yaw += vyaw*dt."""
        with self._odom_lock:
            yaw = self.odom["yaw"] + vyaw * dt
            self.odom["yaw"] = yaw
            self.odom["position"][0] += vx * dt * math.cos(yaw)
            self.odom["position"][1] += vx * dt * math.sin(yaw)

    def _publish_loop(self):
        """Background loop: publish the current command via AgvClient.Move every self.dt (with watchdog fallback)."""
        while self.publish_running:
            # Same lock guards the entire RPC and navigation handoff: no old payload
            # can leak AFTER SLAMWARE has acquired control.
            with self._control_lock:
                payload = None if self._navigation_owned else self._resolve_payload(time.monotonic())
                if payload is not None:
                    vx, vyaw = payload
                    try:
                        code, _ = self.agv.Move(vx, 0.0, vyaw)
                        if code == 0:
                            self._connected = True
                            self._integrate_odom(vx, vyaw, self.dt)
                    except Exception:
                        pass
            time.sleep(self.dt)

    def acquire_navigation(self):
        """Stop/discard manual stream before an onboard navigation goal is sent."""
        with self._control_lock:
            if not self._available:
                raise RuntimeError("机器人 SDK 不可用")
            self._navigation_owned = True
            with self.cmd_lock:
                self.current_cmd = self._last_cmd_time = None
            self.navigation_zero()

    def navigation_zero(self):
        with self._control_lock:
            code, _ = self.agv.Move(0.0, 0.0, 0.0)
            if code != 0:
                raise RuntimeError(f"停车 RPC 失败: {code}")

    def release_navigation(self):
        with self._control_lock:
            with self.cmd_lock:
                self.current_cmd = self._last_cmd_time = None
            self._navigation_owned = False

    def start_publisher(self):
        """Start the command publisher thread."""
        if not self._available or self.publish_running:
            return
        self.publish_running = True
        self._publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._publish_thread.start()

    def stop_publisher(self):
        """Stop the publisher thread, then send zero speed 5 times. Join first so an in-flight tick
        cannot emit a non-zero command after the "final" zeros; RPC success does not mean the
        robot has physically stopped."""
        self.publish_running = False
        if hasattr(self, '_publish_thread'):
            self._publish_thread.join(timeout=1.0)
        if self._available:
            for _ in range(5):
                try:
                    self.agv.Move(0.0, 0.0, 0.0)
                except Exception:
                    pass
                time.sleep(self.dt)
