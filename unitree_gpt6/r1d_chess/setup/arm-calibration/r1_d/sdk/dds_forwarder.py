"""SSH-stdio DDS transport endpoint. No trajectories, pose holding, or model loading.

Executed from source text by SSH, not installed on the robot. Commands are
forwarded once, only before their deadline. A transport watchdog stops lift
velocity after 150 ms without a fresh lift command or on disconnect.
"""

import json
import math
import struct
import sys
import threading
import time


HEADER = struct.Struct("!BI")
STAMP = struct.Struct("!d")
MAX_FRAME = 16384


def read_exact(stream, size):
    result = bytearray()
    while len(result) < size:
        part = stream.read(size - len(result))
        if not part:
            raise EOFError("SSH transport closed")
        result.extend(part)
    return bytes(result)


def read_frame(stream):
    kind, length = HEADER.unpack(read_exact(stream, HEADER.size))
    if length > MAX_FRAME:
        raise ValueError("Oversized transport frame")
    return kind, read_exact(stream, length)


def write_frame(stream, kind, payload):
    if len(payload) > MAX_FRAME:
        raise ValueError("Oversized transport frame")
    data = memoryview(HEADER.pack(kind, len(payload)) + payload)
    while data:
        written = stream.write(data)
        if not written:
            raise BrokenPipeError("Transport write failed")
        data = data[written:]
    stream.flush()


def validate_wire_command(kind, message):
    if kind == 11:
        values = (
            message.linear.x,
            message.linear.y,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        if (
            any(v != 0.0 for v in values)
            or not math.isfinite(message.linear.z)
            or abs(message.linear.z) > 0.100001
        ):
            raise ValueError("Only bounded lift commands are permitted")
    elif kind == 10:
        allowed = {13, *range(15, 32), 33}
        if message.mode_pr != 0:
            raise ValueError("Invalid LowCmd mode_pr")
        for i, m in enumerate(message.motor_cmd):
            if i not in allowed and any((m.mode, m.q, m.dq, m.tau, m.kp, m.kd)):
                raise ValueError("Unsupported motor slot")
            if not all(math.isfinite(v) for v in (m.q, m.dq, m.tau, m.kp, m.kd)):
                raise ValueError("Non-finite motor command")
            if abs(m.tau) > 20.0:
                raise ValueError("Feedforward torque out of range")


def deadline_valid(deadline, now):
    return math.isfinite(deadline) and now <= deadline <= now + 0.5


class LatestFeedback:
    """One pending sample per topic; DDS callbacks never wait for SSH output."""

    def __init__(self):
        self.lock = threading.Lock()
        self.pending = {}

    def put(self, kind, stamp, payload):
        with self.lock:
            self.pending[kind] = STAMP.pack(stamp) + payload

    def take(self):
        with self.lock:
            pending, self.pending = self.pending, {}
        return pending


def main():
    # Keep SDK diagnostic output away from the binary protocol.
    output = sys.stdout.buffer
    sys.stdout = sys.stderr
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
    from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_, Twist_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import Error_
    from unitree_sdk2py.idl.default import geometry_msgs_msg_dds__Twist_
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

    ChannelFactoryInitialize(0, sys.argv[1])
    write_lock = threading.Lock()
    lift_lock = threading.Lock()
    quit_event = threading.Event()
    feedback = LatestFeedback()
    subscribers, publishers = [], {}
    lift = {"last": 0.0, "active": False, "touched": False}

    def send(kind, payload):
        with write_lock:
            write_frame(output, kind, payload)

    def callback(kind, period):
        last = [0.0]

        def receive(message):
            now = time.monotonic()
            if now - last[0] < period:
                return
            last[0] = now
            feedback.put(kind, now, message.serialize())

        return receive

    def forward_feedback():
        while not quit_event.wait(0.002):
            try:
                for kind, payload in feedback.take().items():
                    send(kind, payload)
            except (BrokenPipeError, OSError):
                quit_event.set()

    for kind, name, cls, period in (
        (0, "rt/lowstate", LowState_, 0.009),
        (1, "rt/state_vel", Twist_, 0.018),
        (2, "rt/height_state", Point32_, 0.018),
        (3, "rt/height_alarm", Error_, 0.018),
    ):
        sub = ChannelSubscriber(name, cls)
        sub.Init(callback(kind, period), queueLen=1)
        subscribers.append(sub)
    for kind, name, cls in ((10, "rt/lowcmd", LowCmd_), (11, "rt/cmd_vel", Twist_)):
        pub = ChannelPublisher(name, cls)
        pub.Init()
        publishers[kind] = pub
    client = MotionSwitcherClient()
    client.SetTimeout(3.0)
    client.Init()

    def zero_lift():
        for _ in range(3):
            publishers[11].Write(geometry_msgs_msg_dds__Twist_())
        lift["active"] = False

    def watchdog():
        while not quit_event.wait(0.02):
            with lift_lock:
                if lift["active"] and time.monotonic() - lift["last"] > 0.15:
                    zero_lift()
        with lift_lock:
            if lift["touched"]:
                zero_lift()

    worker = threading.Thread(target=watchdog, daemon=True)
    worker.start()
    send(250, b"R1D DDS forwarder v1")
    feedback_worker = threading.Thread(target=forward_feedback, daemon=True)
    feedback_worker.start()
    try:
        while not quit_event.is_set():
            kind, payload = read_frame(sys.stdin.buffer)
            now = time.monotonic()
            if kind == 249:
                send(249, payload + STAMP.pack(now))
            elif kind in (10, 11):
                deadline = STAMP.unpack(payload[:8])[0]
                # Late buffered commands must never move the robot on reconnect.
                if not deadline_valid(deadline, now):
                    continue
                cls = LowCmd_ if kind == 10 else Twist_
                message = cls.deserialize(payload[8:])
                validate_wire_command(kind, message)
                if kind == 11:
                    with lift_lock:
                        if not publishers[kind].Write(message):
                            raise RuntimeError("DDS lift write failed")
                        lift.update(last=now, active=message.linear.z != 0.0, touched=True)
                elif not publishers[kind].Write(message):
                    raise RuntimeError("DDS arm write failed")
            elif kind == 20:
                request = json.loads(payload)
                if request["operation"] == "check_mode":
                    code, data = client.CheckMode()
                elif request["operation"] == "release_mode":
                    deadline = request.get("deadline", 0.0)
                    if not now <= deadline <= now + 5.0:
                        raise RuntimeError("Expired release-mode request")
                    code, state = client.CheckMode()
                    if (
                        code == 0
                        and isinstance(state, dict)
                        and state.get("name") in ("ai", "normal")
                    ):
                        code, data = client.ReleaseMode()
                    elif code == 0 and isinstance(state, dict) and state.get("name") == "":
                        data = None
                    else:
                        raise RuntimeError("Refusing release of an unexpected service")
                else:
                    raise ValueError("Unsupported RPC")
                send(20, json.dumps({"id": request["id"], "code": code, "data": data}).encode())
            elif kind == 251:
                break
            else:
                raise ValueError("Unknown transport frame")
    except EOFError:
        pass
    except Exception as exc:
        send(255, str(exc).encode())
        raise
    finally:
        quit_event.set()
        worker.join(timeout=1.0)
        # No arm torque unloading, trajectory, or repeated arm command here.


if __name__ == "__main__":
    main()
