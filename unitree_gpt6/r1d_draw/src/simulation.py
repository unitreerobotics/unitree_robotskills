"""Offline stand-ins for the robot and its kinematics; never opens a connection.

``SimulatedRobot`` offers the same functions as the real robot and follows
every command exactly. ``SimulatedKinematics`` is a toy model in which the
first three joints of an arm are the x/y/z of its hand: it lets the whole
workflow run without a URDF, and says nothing about the real arm.
"""

import struct
import zlib

import numpy as np

from .onboard.controller import MODELED, checked_route
from .robot import Robot
from .schema import (
    GRIP_SLOTS,
    HANDS,
    HEAD_SLOTS,
    LIFT_SLOT,
    MOTOR_COUNT,
    WAIST_YAW_SLOT,
    arm_slots,
    vector,
)


class SimulatedKinematics:
    is_simulation = True
    lower = np.full(14, -3.0)
    upper = np.full(14, 3.0)

    @staticmethod
    def arm_slice(hand):
        if hand not in HANDS:
            raise ValueError("hand must be left or right")
        return slice(0, 7) if hand == "left" else slice(7, 14)

    def checked_q14(self, q14):
        result = vector(q14, 14, "synthetic q14")
        if np.any(result < self.lower) or np.any(result > self.upper):
            raise ValueError("synthetic joint limits exceeded")
        return result

    def fk(self, q14, hand, *, check_limits=True):
        result = np.eye(4)
        start = self.arm_slice(hand).start
        result[:3, 3] = np.asarray(q14)[start : start + 3]
        return result

    def solve(self, pose, seed14, hand):
        result = self.checked_q14(seed14)
        start = self.arm_slice(hand).start
        result[start : start + 3] = [pose[k] for k in ("x", "y", "z")]
        result = self.checked_q14(result)
        self.check_pose(result, pose, hand)
        return result

    def check_pose(self, q14, pose, hand):
        actual = self.fk(q14, hand)[:3, 3]
        if not np.allclose(actual, [pose[k] for k in ("x", "y", "z")], atol=1e-9) or any(
            abs(pose[k]) > 1e-9 for k in ("roll", "pitch", "yaw")
        ):
            raise ValueError("synthetic model target mismatch")


def _png(width=4, height=4):
    def chunk(kind, data):
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))

    rows = b"".join(b"\x00" + b"\xff" * (3 * width) for _ in range(height))
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(rows))
        + chunk(b"IEND", b"")
    )


class SimulatedRobot(Robot):
    """Follows commands exactly and instantly; records every route it was given."""

    is_simulation = True

    def __init__(self, q14, *, waist_yaw=0.0, grip_q2=(0.0, 0.0)):
        self.q = [0.0] * MOTOR_COUNT
        for hand, values in zip(HANDS, (q14[:7], q14[7:]), strict=True):
            for slot, value in zip(arm_slots(hand), values, strict=True):
                self.q[slot] = float(value)
        self.q[WAIST_YAW_SLOT] = float(waist_yaw)
        self.q[GRIP_SLOTS[0]], self.q[GRIP_SLOTS[1]] = (float(v) for v in grip_q2)
        self.holding = False
        self.routes, self.tick = [], 0
        self.fail_on_route = None  # set to a route number to simulate a fault there
        self.fail_at_point = None  # point of that route where it stops (default: the middle)

    def state(self):
        self.tick += 1
        q = list(self.q)
        return {
            "time_unix": 0.0,
            "tick": self.tick,
            "q": q,
            "dq": [0.0] * MOTOR_COUNT,
            "tau": [0.0] * MOTOR_COUNT,
            "temperature_c": [30.0] * MOTOR_COUNT,
            "arms": {hand: [q[s] for s in arm_slots(hand)] for hand in HANDS},
            "waist_yaw": q[WAIST_YAW_SLOT],
            "head": [q[s] for s in HEAD_SLOTS],
            "grip": [q[s] for s in GRIP_SLOTS],
            "lift": q[LIFT_SLOT],
            "commanded": self.holding,
            "command_frames": {"lowcmd": 0, "arm_sdk": 0},
            "motor_faults": [],
        }

    def photo(self, camera):
        return _png()

    def status(self):
        return {
            "state": "holding" if self.holding else "closed",
            "alive": self.holding,
            "error": None,
            "target": list(self.q),
        }

    def take_over(self):
        self.holding = True
        return self.status()

    def move(self, route):
        if not self.holding:
            raise RuntimeError("move needs a holding controller; call take_over first")
        limits = [-np.inf] * MOTOR_COUNT, [np.inf] * MOTOR_COUNT
        for slot in MODELED:
            limits[0][slot], limits[1][slot] = -3.2, 3.2
        points = checked_route(route, self.q, *limits)
        number = len(self.routes)
        self.routes.append(route)
        if self.fail_on_route == number:
            # Stop part way through the route, as a controller fault would.
            index = len(points) // 2 if self.fail_at_point is None else self.fail_at_point
            self.q = list(points[index][0])
            self.holding = False
            raise RuntimeError("the controller stopped with a fault: simulated")
        self.q = list(points[-1][0])
        return {"status": "completed", "points": len(points), "settled": True, "end_error_rad": 0.0}

    def stop(self):
        pass

    def relax(self):
        self.holding = False

    def release(self):
        self.holding = False
