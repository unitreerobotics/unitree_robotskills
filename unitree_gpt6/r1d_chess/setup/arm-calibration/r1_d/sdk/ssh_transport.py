"""Local DDS transport through an SSH stream; all control targets originate here."""

import json
import os
from pathlib import Path
import shlex
import select
import subprocess
import tempfile
import threading
import time

from r1_d.config import RUNTIME_DIR

from .dds_forwarder import HEADER, MAX_FRAME, STAMP, read_frame


class SSHTransport:
    def __init__(
        self,
        ip,
        user="unitree",
        remote_interface="eth10",
        local_interface="wlo1",
        remote_python="/home/unitree/agentlib_runtime/venv/bin/python",
    ):
        self.directory = tempfile.TemporaryDirectory(prefix="r1d_ssh_")
        self.socket = str(Path(self.directory.name) / "control")
        self.host = user + "@" + ip
        self.process = None
        self.lock, self.write_lock = threading.Lock(), threading.Lock()
        self.ready, self.reply = threading.Event(), threading.Event()
        self.latest, self.stamps, self.counts = {}, {}, {}
        self.responses = {}
        self.error = None
        self.closed = False
        self.offset = None
        self.last_rtt = None
        known_hosts = RUNTIME_DIR / "ssh_known_hosts"
        known_hosts.parent.mkdir(parents=True, exist_ok=True)
        # OpenSSH owns authentication; password is never put in arguments/files.
        self.options = [
            "-B",
            local_interface,
            "-o",
            f"UserKnownHostsFile={known_hosts}",
            "-o",
            "StrictHostKeyChecking=accept-new",
            "-o",
            "ConnectTimeout=5",
            "-o",
            "ServerAliveInterval=2",
            "-o",
            "ServerAliveCountMax=2",
            "-S",
            self.socket,
        ]
        try:
            subprocess.run(["ssh", *self.options, "-M", "-fN", self.host], check=True)
            source = Path(__file__).with_name("dds_forwarder.py").read_text()
            remote = shlex.join([remote_python, "-B", "-u", "-c", source, remote_interface])
            self.process = subprocess.Popen(
                ["ssh", *self.options, "-T", self.host, remote],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                bufsize=0,
            )
            os.set_blocking(self.process.stdin.fileno(), False)
            threading.Thread(target=self.receive, daemon=True).start()
            if not self.ready.wait(12.0):
                raise TimeoutError("SSH DDS forwarder did not become ready")
            self.check()
            # Choose the fastest of three round trips to estimate clock offset.
            measurements = []
            for _ in range(3):
                started = time.monotonic()
                key = str(started)
                self.send(249, key.encode())
                response = self.wait_reply(key, 3.0)
                finished = time.monotonic()
                measurements.append((finished - started, response - (started + finished) / 2.0))
            self.last_rtt, self.offset = min(measurements)
            if self.last_rtt > 0.15:
                raise RuntimeError(f"SSH RTT too high for control: {self.last_rtt:.3f}s")
        except BaseException:
            self.close()
            raise

    def receive(self):
        try:
            while True:
                kind, payload = read_frame(self.process.stdout)
                received = time.monotonic()
                with self.lock:
                    if kind in (0, 1, 2, 3):
                        remote_time = STAMP.unpack(payload[:8])[0]
                        self.latest[kind] = payload[8:]
                        self.stamps[kind] = (remote_time, received)
                        self.counts[kind] = self.counts.get(kind, 0) + 1
                    elif kind == 250:
                        self.ready.set()
                    elif kind == 249:
                        self.responses[payload[:-8].decode()] = STAMP.unpack(payload[-8:])[0]
                        self.reply.set()
                    elif kind == 20:
                        data = json.loads(payload)
                        self.responses[data["id"]] = (data["code"], data["data"])
                        self.reply.set()
                    elif kind == 255:
                        raise RuntimeError(payload.decode())
        except Exception as exc:
            if not self.closed:
                self.error = str(exc)
                self.ready.set()
                self.reply.set()

    def check(self):
        if self.error:
            raise RuntimeError("SSH DDS transport: " + self.error)
        if self.closed:
            raise RuntimeError("SSH DDS transport is closed")

    def send(self, kind, payload):
        self.check()
        if len(payload) > MAX_FRAME:
            raise ValueError("Oversized transport frame")
        with self.write_lock:
            # A congested SSH pipe must not block the local control loop forever.
            packet = memoryview(HEADER.pack(kind, len(payload)) + payload)
            deadline = time.monotonic() + 0.1
            fd = self.process.stdin.fileno()
            while packet:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not select.select([], [fd], [], remaining)[1]:
                    self.error = "SSH write blocked for 100 ms"
                    raise TimeoutError(self.error)
                try:
                    written = os.write(fd, packet)
                except BlockingIOError:
                    continue
                packet = packet[written:]

    def wait_reply(self, key, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.check()
            with self.lock:
                if key in self.responses:
                    return self.responses.pop(key)
                self.reply.clear()
            self.reply.wait(min(0.1, max(0.0, end - time.monotonic())))
        raise TimeoutError("SSH DDS response timed out")

    def rpc(self, operation):
        key = str(time.monotonic_ns())
        request = {
            "operation": operation,
            "id": key,
            "deadline": time.monotonic() + self.offset + 4.0,
        }
        self.send(20, json.dumps(request).encode())
        return self.wait_reply(key, 8.0)

    def CheckMode(self):
        return self.rpc("check_mode")

    def ReleaseMode(self):
        return self.rpc("release_mode")

    def snapshot(self):
        self.check()
        with self.lock:
            return (
                self.latest.copy(),
                {
                    k: min(remote - self.offset, received)
                    for k, (remote, received) in self.stamps.items()
                },
                self.counts.copy(),
            )

    def publish(self, kind, message):
        # Local trajectory loop is the sole source of arm/lift targets.
        self.send(kind, STAMP.pack(time.monotonic() + self.offset + 0.15) + message.serialize())

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.process is not None:
            try:
                self.process.stdin.close()  # EOF also triggers remote lift stop.
                self.process.wait(timeout=2.0)
            except (OSError, subprocess.TimeoutExpired):
                self.process.terminate()
        subprocess.run(
            ["ssh", *self.options, "-O", "exit", self.host],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.directory.cleanup()
