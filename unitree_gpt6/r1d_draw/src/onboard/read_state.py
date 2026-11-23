"""Runs on the robot (Python 3.8). Read-only: prints one LowState sample as JSON.

Creates subscribers only. No publisher is created and no command is sent.
Usage on the robot: python3 - [window_seconds] < read_state.py
"""
# The robot runs Python 3.8: no zip(strict=) and percent formatting stays.
# ruff: noqa: UP031, B905

import json
import sys
import time

from unitree_dds_wrapper.robots import g1, r1

WINDOW_S = float(sys.argv[1]) if len(sys.argv) > 1 else 0.4

state = g1.sub.LowState(autospin=False)
commands = {"lowcmd": g1.sub.LowCmd(autospin=False), "arm_sdk": r1.sub.ArmSdk(autospin=False)}

deadline = time.monotonic() + 4.0
while state.msg is None and time.monotonic() < deadline:
    state.take_one()
    time.sleep(0.01)
if state.msg is None:
    raise RuntimeError("no LowState received on the robot")

ticks = set()
frames = dict.fromkeys(commands, 0)
end = time.monotonic() + WINDOW_S
while time.monotonic() < end:
    state.take_one()
    ticks.add(int(state.msg.tick))
    for name, subscriber in commands.items():
        before = getattr(subscriber, "_last_recv_time", None)
        subscriber.take_one()
        frames[name] += int(getattr(subscriber, "_last_recv_time", None) != before)
    time.sleep(0.004)

message = state.msg
motors = message.motor_state
sample = {
    "time_unix": time.time(),
    "tick": int(message.tick),
    "ticks_seen": len(ticks),
    "window_s": WINDOW_S,
    "mode_machine": int(message.mode_machine),
    "q": [float(m.q) for m in motors],
    "dq": [float(m.dq) for m in motors],
    "tau": [float(m.tau_est) for m in motors],
    "temperature_c": [float(max(m.temperature)) for m in motors],
    "motor_mode": [int(m.mode) for m in motors],
    "motor_state": [int(m.motorstate) for m in motors],
    "command_frames": frames,
}
print("R1D_STATE=" + json.dumps(sample, allow_nan=False), flush=True)
