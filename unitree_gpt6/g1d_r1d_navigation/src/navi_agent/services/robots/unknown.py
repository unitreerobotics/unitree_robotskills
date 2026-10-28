# -*- coding: utf-8 -*-
"""unknown.py -- placeholder robot for unrecognised/ambiguous detection: no DDS init, refuses all motion."""
from __future__ import annotations


class UnknownRobot:
    model = "unknown"
    dt = 0.05
    available = False
    connected = False

    def __init__(self, reason: str):
        self.reason = reason
        self.odom = {"position": [0.0, 0.0, 0.0], "yaw": 0.0}

    def _refuse(self, *_a, **_k):
        raise RuntimeError(f"robot_model_unknown: {self.reason}")

    def send_command(self, vx=0.0, vy=0.0, ang_z=0.0, **_k):
        # Zero speed means "stop" (sent by MotionExecutor finally / stop), so let it pass silently.
        if vx == 0.0 and vy == 0.0 and ang_z == 0.0:
            return
        self._refuse()

    acquire_navigation = navigation_zero = _refuse

    def release_navigation(self):
        pass

    def wait_for_data(self):
        self._refuse()

    def start_publisher(self):
        pass

    def stop_publisher(self):
        pass
