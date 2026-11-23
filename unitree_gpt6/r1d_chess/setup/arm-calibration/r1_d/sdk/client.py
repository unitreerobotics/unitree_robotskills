"""R1D connection, feedback and bounded low-level operations."""

import dataclasses
import ipaddress
import json
import math
import subprocess
import time
from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_, Twist_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import Error_
from .model import GROUPS, HOLD, finite
from .protocol import lift_command
from r1_d.config import DEFAULTS


class FeedbackUnavailable(RuntimeError):
    """A required topic has no sufficiently recent sample; distinct from motor/alarm faults."""

    def __init__(self, topic, age, limit):
        self.topic, self.age, self.limit = topic, age, limit
        age_text = "missing" if age is None else f"{age:.3f}s"
        super().__init__(f"Missing/stale feedback: {topic} (age={age_text}, limit={limit:.3f}s)")


# motorstate bit 31: the joint board missed frames from this motor. Seen in bursts
# on single motors; the motor keeps its last command. Other bits are real faults.
MOTOR_LINK_LOST = 0x80000000
MOTOR_LINK_SECONDS = 1.0  # longest uninterrupted loss that is tolerated


class R1D:
    """One robot per process. Connect/read never sends motor commands.

    Wi-Fi defaults to SSH message forwarding. Direct DDS remains available
    with transport='dds' when the host can reach the robot's DDS network.
    """

    def __init__(
        self,
        ip=DEFAULTS["ip"],
        interface=DEFAULTS["interface"],
        domain=0,
        multicast=False,
        transport=DEFAULTS["transport"],
        ssh_user=DEFAULTS["ssh_user"],
        remote_interface=DEFAULTS["remote_interface"],
        remote_python=DEFAULTS["remote_python"],
    ):
        self.ip = str(ipaddress.IPv4Address(ip))
        if interface is None:
            route = json.loads(
                subprocess.check_output(["ip", "-j", "route", "get", self.ip], text=True)
            )
            interface = route[0]["dev"]
        if not interface or any(
            c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.:-"
            for c in interface
        ):
            raise ValueError("Invalid interface")
        self.interface = interface
        self.transport = transport
        self.closed = False
        self.relay = None
        self.readers, self.latest, self.stamps, self.counts, self.writers = {}, {}, {}, {}, {}
        self.sent = {"rt/lowcmd": 0, "rt/cmd_vel": 0}
        self.link_lost_since, self.link_lost_samples = {}, {}
        self.link_seconds = MOTOR_LINK_SECONDS
        self.link_ignored = ()  # motors whose link loss never ends the session
        if transport == "ssh":
            if domain != 0:
                raise ValueError("SSH transport currently supports domain 0 only")
            from .ssh_transport import SSHTransport

            self.relay = SSHTransport(
                self.ip,
                user=ssh_user,
                remote_interface=remote_interface,
                local_interface=interface,
                remote_python=remote_python,
            )
            return
        if transport != "dds":
            raise ValueError("transport must be dds or ssh")
        config = (
            f'<CycloneDDS><Domain Id="{domain}"><General><Interfaces>'
            f'<NetworkInterface name="{interface}"/></Interfaces>'
            f"<AllowMulticast>{str(multicast).lower()}</AllowMulticast></General><Discovery>"
            f'<Peers><Peer Address="{self.ip}"/></Peers>'
            "<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>40</MaxAutoParticipantIndex>"
            "</Discovery></Domain></CycloneDDS>"
        )
        self.domain = Domain(domain, config)
        self.participant = DomainParticipant(domain)
        qos = Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(1))
        for name, cls in (
            ("rt/lowstate", LowState_),
            ("rt/state_vel", Twist_),
            ("rt/height_state", Point32_),
            ("rt/height_alarm", Error_),
        ):
            self.readers[name] = DataReader(
                self.participant, Topic(self.participant, name, cls), qos=qos
            )

    def rpc(self, api_id):
        """MotionSwitcher query/release on the existing transport; no second DDS domain."""
        if api_id not in (1001, 1003):
            raise ValueError("Only check/release motion mode is supported")
        if self.relay is not None:
            return self.relay.CheckMode() if api_id == 1001 else self.relay.ReleaseMode()
        from unitree_sdk2py.idl.unitree_api.msg.dds_ import (
            Request_,
            Response_,
            RequestHeader_,
            RequestIdentity_,
            RequestLease_,
            RequestPolicy_,
        )

        if not hasattr(self, "rpc_reader"):
            self.rpc_reader = DataReader(
                self.participant,
                Topic(self.participant, "rt/api/motion_switcher/response", Response_),
            )
            self.rpc_writer = DataWriter(
                self.participant,
                Topic(self.participant, "rt/api/motion_switcher/request", Request_),
            )
            time.sleep(0.3)
        request_id = time.monotonic_ns()
        msg = Request_(
            RequestHeader_(
                RequestIdentity_(request_id, api_id), RequestLease_(0), RequestPolicy_(0, False)
            ),
            "{}",
            [],
        )
        self.rpc_writer.write(msg)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            for response in self.rpc_reader.take(10):
                if not isinstance(response, Response_):
                    continue
                if (
                    response.header.identity.id == request_id
                    and response.header.identity.api_id == api_id
                ):
                    code = response.header.status.code
                    return code, json.loads(response.data) if code == 0 and api_id == 1001 else None
            time.sleep(0.01)
        raise TimeoutError("MotionSwitcher response timed out")

    def CheckMode(self):
        return self.rpc(1001)

    def ReleaseMode(self):
        return self.rpc(1003)

    def prepare_control(self, release=False):
        """Validate current state/mode and initialize writers before timing starts."""
        from .motion_mode import prepare

        self.arm_state()
        self.height()
        prepare(self, release=release)
        if self.relay is None:
            for name, cls in (("rt/lowcmd", LowCmd_), ("rt/cmd_vel", Twist_)):
                if name not in self.writers:
                    self.writers[name] = DataWriter(
                        self.participant, Topic(self.participant, name, cls)
                    )
            time.sleep(0.3)

    def poll(self):
        if self.relay is not None:
            messages, stamps, counts = self.relay.snapshot()
            mapping = (
                ("rt/lowstate", LowState_),
                ("rt/state_vel", Twist_),
                ("rt/height_state", Point32_),
                ("rt/height_alarm", Error_),
            )
            for kind, payload in messages.items():
                name, cls = mapping[kind]
                if self.stamps.get(name) != stamps[kind]:
                    self.latest[name] = cls.deserialize(payload)
                    self.stamps[name] = stamps[kind]
                    self.counts[name] = counts[kind]
            return
        for name, reader in self.readers.items():
            samples = reader.take(100)
            valid = [m for m in samples if dataclasses.is_dataclass(m)]
            if valid:
                self.latest[name] = valid[-1]
                self.stamps[name] = time.monotonic()
                self.counts[name] = self.counts.get(name, 0) + len(valid)

    def fresh(self, name, max_age=0.3):
        self.poll()
        age = time.monotonic() - self.stamps[name] if name in self.stamps else None
        if name not in self.latest or age is None or age > max_age:
            raise FeedbackUnavailable(name, age, max_age)
        return self.latest[name]

    def feedback_status(self):
        """Cached diagnostics only: usable even while the transport is failing."""
        now = time.monotonic()
        return {
            "age_seconds": {k: now - v for k, v in self.stamps.items()},
            "received_samples": self.counts.copy(),
            "transport_error": self.relay.error if self.relay else None,
            "motor_link_lost_samples": self.link_lost_samples.copy(),
        }

    def inspect(self, seconds=5.0):
        seconds = finite(seconds, "seconds")
        if not 0 <= seconds <= 60:
            raise ValueError("Inspection duration must be 0..60 seconds")
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.poll()
            time.sleep(0.01)
        return {
            "robot_ip": self.ip,
            "interface": self.interface,
            "transport": self.transport,
            "received_samples": self.counts.copy(),
            "sent_commands": self.sent.copy(),
            "age_seconds": {k: time.monotonic() - v for k, v in self.stamps.items()},
            "states": {k: dataclasses.asdict(v) for k, v in self.latest.items()},
        }

    def arm_state(self):
        state = self.fresh("rt/lowstate")
        indices = HOLD + tuple(i for values in GROUPS.values() for i in values)
        for i in indices:
            m = state.motor_state[i]
            if not all(math.isfinite(x) for x in (m.q, m.dq, m.tau_est)):
                raise RuntimeError(f"Non-finite feedback at motor {i}")
            if (
                m.motorstate & ~MOTOR_LINK_LOST
                or m.temperature[0] >= 85
                or m.temperature[1] >= 120
            ):
                raise RuntimeError(f"Motor {i} fault/temperature: {dataclasses.asdict(m)}")
            if not m.motorstate:
                self.link_lost_since.pop(i, None)
            elif self.stamps["rt/lowstate"] != self.link_lost_since.get(i, (None, None))[1]:
                # Count each received sample once; time the uninterrupted loss.
                now = time.monotonic()
                since = self.link_lost_since.get(i, (now, None))[0]
                self.link_lost_since[i] = (since, self.stamps["rt/lowstate"])
                self.link_lost_samples[i] = self.link_lost_samples.get(i, 0) + 1
                if now - since > self.link_seconds and i not in self.link_ignored:
                    raise RuntimeError(
                        f"Motor {i} link lost for {now - since:.1f}s: {dataclasses.asdict(m)}"
                    )
        return state

    def measured_targets(self, state):
        return {k: [state.motor_state[i].q for i in ids] for k, ids in GROUPS.items()}

    def height(self):
        alarm = self.fresh("rt/height_alarm", 1.0)
        if alarm.source != 200 or alarm.state != 0:
            raise RuntimeError(f"Lift not homed or alarm present: {dataclasses.asdict(alarm)}")
        h = finite(self.fresh("rt/height_state").z, "height") / 1000.0
        if not 0 <= h <= 0.84:
            raise RuntimeError(f"Lift height out of range: {h}")
        return h

    def publish(self, topic, message):
        if getattr(self, "closed", False):
            raise RuntimeError("R1D connection is closed")
        if self.relay is not None:
            self.relay.publish({"rt/lowcmd": 10, "rt/cmd_vel": 11}[topic], message)
            self.sent[topic] += 1
            return
        if topic not in self.writers:
            self.writers[topic] = DataWriter(
                self.participant, Topic(self.participant, topic, type(message))
            )
            time.sleep(0.3)
        self.writers[topic].write(message)
        self.sent[topic] += 1

    def stop_lift(self):
        if "rt/cmd_vel" in self.writers or self.sent.get("rt/cmd_vel", 0):
            for _ in range(3):
                self.publish("rt/cmd_vel", lift_command(0.0))
                time.sleep(0.02)

    def close(self):
        if getattr(self, "closed", False):
            return
        try:
            self.stop_lift()
        finally:
            self.closed = True
            try:
                if self.relay is not None:
                    self.relay.close()
            finally:
                # R1-D lifecycle: no arm unloading or automatic controller switch.
                self.writers.clear()
                self.readers.clear()
