"""One verified SSH connection to a robot-side computer over the wired network.

The R1-D's DDS requires the robot's own credentials, so state is read and
commands are sent by programs that run on the robot. This module only opens
the connection, checks that the host is the expected machine, and runs a
command there.
"""

from __future__ import annotations

import os
import shlex
import socket
import time

# Environment of the vendor DDS wrapper on the robot's computer. In a shell
# command these assignments must come first, before ``timeout`` or ``python3``.
ONBOARD_ENV = (
    "CYCLONEDDS_URI=/unitree/etc/cyclonedds_noshm.xml "
    "LD_LIBRARY_PATH=/unitree/module/r1_arm_example/lib "
    "PYTHONPATH=/unitree/module/r1_arm_example/lib/python3.8/site-packages "
    "PYTHONDONTWRITEBYTECODE=1"
)


class SshLink:
    """Runs commands on one host after checking its wired MAC address."""

    def __init__(self, *, host, user, password_env, interface, mac, source_ip=None, timeout=6.0):
        try:
            import paramiko
        except ImportError as exc:
            raise ImportError("the wired connection requires the 'paramiko' package") from exc
        password = os.environ.get(password_env)
        if not password:
            raise RuntimeError(f"set the environment variable {password_env} to the SSH password")
        self.host = host
        self._socket = socket.socket()
        self._socket.settimeout(timeout)
        self._ssh = paramiko.SSHClient()
        try:
            if source_ip:
                self._socket.bind((source_ip, 0))
            self._socket.connect((host, 22))
            self._ssh.load_system_host_keys()
            self._ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self._ssh.connect(
                host,
                username=user,
                password=password,
                sock=self._socket,
                look_for_keys=False,
                allow_agent=False,
                timeout=timeout,
            )
            code, out, _ = self.run("cat " + shlex.quote(f"/sys/class/net/{interface}/address"))
            found = out.decode(errors="replace").strip().lower()
            if code != 0 or found != mac:
                raise RuntimeError(
                    f"{host} is not the configured machine: {interface} has MAC "
                    f"{found or 'unknown'}, expected {mac}"
                )
        except BaseException:
            self.close()
            raise

    def run(self, command, *, stdin=None, timeout=20.0):
        """Run a shell command; return (exit code, stdout bytes, stderr text)."""
        channel_in, channel_out, channel_err = self._ssh.exec_command(command, timeout=timeout)
        if stdin is not None:
            channel_in.write(stdin)
            channel_in.flush()
        channel_in.channel.shutdown_write()
        channel = channel_out.channel
        out, err = bytearray(), bytearray()
        deadline = time.monotonic() + timeout
        while (
            not channel.exit_status_ready() or channel.recv_ready() or channel.recv_stderr_ready()
        ):
            if time.monotonic() > deadline:
                channel.close()
                raise TimeoutError(f"command on {self.host} did not finish in {timeout} s")
            if channel.recv_ready():
                out.extend(channel.recv(65536))
            if channel.recv_stderr_ready():
                err.extend(channel.recv_stderr(65536))
            time.sleep(0.005)
        return channel.recv_exit_status(), bytes(out), err.decode(errors="replace")

    def close(self):
        self._ssh.close()
        self._socket.close()
