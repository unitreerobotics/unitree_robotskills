"""Offline framing, isolation, expiry and disconnect/watchdog tests."""

import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock

from r1_d.sdk.dds_forwarder import (
    HEADER,
    MAX_FRAME,
    STAMP,
    read_frame,
    write_frame,
    validate_wire_command,
    deadline_valid,
    LatestFeedback,
)
from r1_d.sdk import R1D
from r1_d.sdk.model import GROUPS, HOLD
from r1_d.sdk.protocol import arm_unload_command, lift_command, low_command
from r1_d.sdk.ssh_transport import SSHTransport
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_


class TransportTests(unittest.TestCase):
    def test_feedback_backpressure_coalesces_without_blocking_dds_callback(self):
        pending = LatestFeedback()
        pending.put(0, 1.0, b"first")
        blocked_batch = pending.take()  # A sender may block holding this batch.
        for n in range(1000):
            pending.put(0, float(n + 2), b"latest arm")
            pending.put(2, float(n + 2), b"latest height")
        batch = pending.take()
        self.assertEqual(len(batch), 2)
        self.assertEqual(batch[2], STAMP.pack(1001.0) + b"latest height")
        self.assertEqual(blocked_batch[0], STAMP.pack(1.0) + b"first")
        self.assertEqual(pending.take(), {})

    def test_fragmented_stream_and_short_writes(self):
        class ShortWriter(io.BytesIO):
            def write(self, data):
                return super().write(data[:3])

        class ShortReader(io.BytesIO):
            def read(self, size=-1):
                return super().read(min(size, 2))

        writer = ShortWriter()
        write_frame(writer, 10, b"123456789")
        self.assertEqual(read_frame(ShortReader(writer.getvalue())), (10, b"123456789"))
        with self.assertRaises(EOFError):
            read_frame(io.BytesIO(writer.getvalue()[:-1]))
        with self.assertRaises(ValueError):
            read_frame(io.BytesIO(HEADER.pack(1, MAX_FRAME + 1)))

    def test_command_expiry_and_scope(self):
        self.assertTrue(deadline_valid(10.15, 10.0))
        for value in (9.99, 11.0, float("nan"), float("inf")):
            self.assertFalse(deadline_valid(value, 10.0))
        message = lift_command(0.1)
        validate_wire_command(11, message)
        message.linear.x = 0.1
        with self.assertRaises(ValueError):
            validate_wire_command(11, message)
        state = unitree_hg_msg_dds__LowState_()
        cmd = low_command(
            state, {name: [0.0] * len(ids) for name, ids in GROUPS.items()}, {i: 0.0 for i in HOLD}
        )
        validate_wire_command(10, cmd)
        validate_wire_command(10, arm_unload_command(state))
        cmd.motor_cmd[0].mode = 1
        with self.assertRaises(ValueError):
            validate_wire_command(10, cmd)

    def test_buffered_feedback_preserves_original_age(self):
        relay = object.__new__(SSHTransport)
        relay.error, relay.closed = None, False
        relay.lock = threading.Lock()
        relay.offset = 100.0
        relay.latest = {0: b"data"}
        relay.stamps = {0: (110.0, 20.0)}
        relay.counts = {0: 1}
        _, stamps, _ = relay.snapshot()
        self.assertEqual(stamps[0], 10.0)  # Delayed arrival must not refresh old state.

    def test_transport_failure_blocks_publish(self):
        robot = object.__new__(R1D)
        robot.relay = Mock()
        robot.relay.publish.side_effect = RuntimeError("disconnected")
        robot.sent = {"rt/cmd_vel": 0}
        with self.assertRaises(RuntimeError):
            robot.publish("rt/cmd_vel", lift_command(0.1))
        self.assertEqual(robot.sent["rt/cmd_vel"], 0)

    def test_congested_pipe_times_out(self):
        reader, writer = os.pipe()
        try:
            os.set_blocking(writer, False)
            while True:
                try:
                    os.write(writer, b"x" * 4096)
                except BlockingIOError:
                    break
            relay = object.__new__(SSHTransport)
            relay.error, relay.closed = None, False
            relay.write_lock = threading.Lock()
            relay.process = Mock()
            relay.process.stdin.fileno.return_value = writer
            started = time.monotonic()
            with self.assertRaises(TimeoutError):
                relay.send(11, b"data")
            self.assertLess(time.monotonic() - started, 0.5)
        finally:
            os.close(reader)
            os.close(writer)

    def test_forwarder_does_not_replay_and_stops_lift_on_loss(self):
        # Run the actual endpoint with fake DDS publishers; no sockets or robot.
        bootstrap = """
import sys,json
sys.path.insert(0,sys.argv[1]);import r1_d.sdk
import unitree_sdk2py.core.channel as ch
import unitree_sdk2py.comm.motion_switcher.motion_switcher_client as ms
output=sys.argv[2]
class Publisher:
 def __init__(self,name,cls): self.name=name
 def Init(self): pass
 def Write(self,msg):
  with open(output,"a") as f: f.write(json.dumps({"topic":self.name,"z":getattr(getattr(msg,"linear",None),"z",None)})+"\\n")
  return True
class Subscriber:
 def __init__(self,*a): pass
 def Init(self,*a,**k): pass
class Client:
 def SetTimeout(self,*a): pass
 def Init(self): pass
ch.ChannelFactoryInitialize=lambda *a: None
ch.ChannelPublisher=Publisher;ch.ChannelSubscriber=Subscriber;ms.MotionSwitcherClient=Client
from r1_d.sdk import dds_forwarder
sys.argv=["forwarder","eth10"]
dds_forwarder.main()
"""
        for disconnect in (False, True):
            with self.subTest(disconnect=disconnect), tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp) / "writes.jsonl"
                proc = subprocess.Popen(
                    [
                        sys.executable,
                        "-B",
                        "-u",
                        "-c",
                        bootstrap,
                        str(Path(__file__).resolve().parents[2]),
                        str(path),
                    ],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )
                try:
                    self.assertEqual(read_frame(proc.stdout)[0], 250)
                    # Expired arm frame gets discarded even if its payload is invalid.
                    write_frame(proc.stdin, 10, STAMP.pack(time.monotonic() - 1.0) + b"expired")
                    write_frame(
                        proc.stdin,
                        11,
                        STAMP.pack(time.monotonic() + 0.15) + lift_command(0.1).serialize(),
                    )
                    if not disconnect:
                        time.sleep(0.25)
                    proc.stdin.close()
                    proc.wait(timeout=3.0)
                    self.assertEqual(proc.returncode, 0, proc.stderr.read().decode())
                    records = [json.loads(x) for x in path.read_text().splitlines()]
                    self.assertTrue(all(x["topic"] == "rt/cmd_vel" for x in records))
                    self.assertEqual(sum(abs(x["z"] - 0.1) < 1e-6 for x in records), 1)
                    self.assertEqual(records[-1]["z"], 0.0)
                    self.assertGreaterEqual(sum(x["z"] == 0.0 for x in records), 3)
                finally:
                    if proc.poll() is None:
                        proc.kill()
                        proc.wait()
                    proc.stdout.close()
                    proc.stderr.close()


if __name__ == "__main__":
    unittest.main()
