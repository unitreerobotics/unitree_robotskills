"""The functions the painting code uses to reach an R1-D.

    robot = connect("device.json")
    robot.state()          # read joints, temperatures and whether a controller is active
    robot.photo("head")    # one PNG frame
    robot.take_over()      # hold the arms where they are
    robot.move(route)      # run one joint route and wait until it has finished
    robot.stop()           # cancel the running route, keep holding
    robot.relax()          # arms go limp slowly, grippers keep holding
    robot.release()        # stop commanding; does not relax or park the arms

Planning and execution code depends only on this interface. ``WiredRobot`` is
the implementation for the real robot; a different controller can be used by
providing the same five methods.
"""

from __future__ import annotations

import json
import shlex
import struct
import time
import uuid
from importlib import resources
from pathlib import Path

from . import schema
from .config import load_config, normalize_config
from .storage import atomic_json, read_json
from .wired import ONBOARD_ENV, SshLink

ONBOARD_ROOT = "/tmp/r1d_draw"

_PNG = b"\x89PNG\r\n\x1a\n"


class Robot:
    """Contract of the robot interface. Angles are radians, slots follow ``schema``."""

    def state(self) -> dict:
        """Return fresh measured feedback. Never sends a command."""
        raise NotImplementedError

    def photo(self, camera: str) -> bytes:
        """Return one PNG frame of a configured camera. Never sends a command."""
        raise NotImplementedError

    def take_over(self) -> dict:
        """Start holding the arms at their current posture and return the new state."""
        raise NotImplementedError

    def move(self, route: dict) -> dict:
        """Run one joint route; return after the arms have stopped. Raise on a fault."""
        raise NotImplementedError

    def stop(self) -> None:
        """Cancel the running route and keep holding where the arms are."""
        raise NotImplementedError

    def relax(self) -> None:
        """Let the arms go limp slowly (damping only); the grippers keep their grip."""
        raise NotImplementedError

    def release(self) -> None:
        """Stop commanding. The arms are neither relaxed nor parked."""
        raise NotImplementedError

    def close(self) -> None:
        """Close connections. Does not change what the robot is doing."""


class WiredRobot(Robot):
    """Real R1-D, reached over wired SSH; programs on the robot do the DDS I/O."""

    def __init__(self, config, link_factory=SshLink):
        self.config = config
        self._link_factory = link_factory
        self._links = {}

    def _link(self, *, host, user, password_env, interface, mac):
        if host not in self._links:
            self._links[host] = self._link_factory(
                host=host,
                user=user,
                password_env=password_env,
                interface=interface,
                mac=mac,
                source_ip=self.config["device"]["source_ip"],
            )
        return self._links[host]

    def _robot_link(self):
        device = self.config["device"]
        return self._link(
            **{k: device[k] for k in ("host", "user", "password_env", "interface", "mac")}
        )

    def state(self, window_s=0.4):
        source = resources.files(__package__).joinpath("onboard/read_state.py").read_text()
        window = shlex.quote(str(schema.number(window_s, "window_s", positive=True)))
        command = f"{ONBOARD_ENV} timeout --signal=INT --kill-after=3s 15s python3 -B - {window}"
        code, out, err = self._robot_link().run(command, stdin=source, timeout=20.0)
        lines = [x for x in out.decode(errors="replace").splitlines() if x.startswith("R1D_STATE=")]
        if code != 0 or len(lines) != 1:
            raise RuntimeError(f"reading the robot state failed (exit {code}): {err[-600:]}")
        return _checked_state(json.loads(lines[0][len("R1D_STATE=") :]))

    def photo(self, camera):
        cameras = self.config.get("cameras", {})
        if camera not in cameras:
            raise ValueError(f"unknown camera {camera!r}; configured: {sorted(cameras)}")
        settings = cameras[camera]
        link = self._link(
            **{k: settings[k] for k in ("host", "user", "password_env", "interface", "mac")}
        )
        code, out, err = link.run(_capture_command(settings), timeout=25.0)
        if code != 0 or out[:8] != _PNG or len(out) < 24:
            raise RuntimeError(f"camera {camera} capture failed (exit {code}): {err[-600:]}")
        return out

    # ---- motion: an onboard controller process owns the joints -------------------------
    def _session_file(self):
        return Path(self.config["storage"]["directory"]) / "session.json"

    def _session_directory(self):
        try:
            return read_json(self._session_file())["directory"]
        except (OSError, ValueError, KeyError):
            return None

    def status(self):
        """Status written by the onboard controller, or None when there is no session."""
        directory = self._session_directory()
        if directory is None:
            return None
        path = shlex.quote(f"{directory}/status.json")
        # The controller records its own process id in the status file.
        pid = f"$(sed -n 's/.*\"pid\": *\\([0-9][0-9]*\\).*/\\1/p' {path})"
        command = f"cat {path}; echo; kill -0 {pid} 2>/dev/null && echo R1D_ALIVE"
        _, out, _ = self._robot_link().run(command)
        text = out.decode(errors="replace")
        try:
            status = json.loads(text[: text.rindex("}") + 1])
        except ValueError:
            return None
        status["alive"] = text.rstrip().endswith("R1D_ALIVE")
        return status

    def _wait(self, done, timeout, what):
        deadline = time.monotonic() + timeout
        while True:
            status = self.status()
            if status is not None:
                if status["state"] == "fault":
                    raise RuntimeError(f"the controller stopped with a fault: {status['error']}")
                if not status["alive"] and status["state"] not in ("closed", "relaxed"):
                    raise RuntimeError(f"the controller process ended while {status['state']}")
                if done(status):
                    return status
            if time.monotonic() >= deadline:
                # The onboard controller is not cancelled by this timeout.
                raise TimeoutError(f"{what} was not confirmed within {timeout:.0f} s; query status")
            time.sleep(0.2)

    def take_over(self):
        status = self.status()
        if status is not None and status["alive"] and status["state"] in ("holding", "moving"):
            return status
        if "model" not in self.config:
            raise ValueError("take_over needs model.urdf in the configuration")
        before = self.state()
        if before["commanded"]:
            raise RuntimeError("another program is publishing joint commands; stop it first")
        if before["motor_faults"]:
            raise RuntimeError(f"motors report faults: {before['motor_faults']}")
        session = time.strftime("r1d_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8]
        directory = f"{ONBOARD_ROOT}/{session}"
        link, quoted = self._robot_link(), shlex.quote(directory)
        package = resources.files(__package__)
        files = {
            "controller.py": package.joinpath("onboard/controller.py").read_text(),
            "model.urdf": Path(self.config["model"]["urdf"]).read_text(encoding="utf-8"),
        }
        for name, content in files.items():
            code, _, err = link.run(f"mkdir -p {quoted} && cat > {quoted}/{name}", stdin=content)
            if code != 0:
                raise RuntimeError(f"could not copy {name} to the robot: {err[-300:]}")
        atomic_json(self._session_file(), {"session_id": session, "directory": directory})
        launch = f"cd {quoted} && {ONBOARD_ENV} nohup setsid python3 -B controller.py {quoted} "
        launch += "> controller.log 2>&1 < /dev/null &"
        code, _, err = link.run(launch)
        if code != 0:
            raise RuntimeError(f"could not start the controller: {err[-300:]}")
        try:
            return self._wait(lambda s: s["state"] == "holding", 40.0, "take-over")
        except RuntimeError as exc:
            _, log, _ = link.run(f"tail -c 1500 {quoted}/controller.log")
            raise RuntimeError(f"{exc}\n{log.decode(errors='replace')}") from exc

    def move(self, route):
        status = self.status()
        if status is None or not status["alive"] or status["state"] != "holding":
            raise RuntimeError("move needs a holding controller; call take_over first")
        if not isinstance(route, dict) or not isinstance(route.get("points"), list):
            raise ValueError("route must be an object with a list of points")
        duration = sum(p.get("duration_s", 0) + p.get("dwell_s", 0) for p in route["points"])
        request = {"id": uuid.uuid4().hex, "action": "move", "route": route}
        self._request(request)
        result = self._wait(
            lambda s: (s["last_route"] or {}).get("id") == request["id"], duration + 20.0, "route"
        )["last_route"]
        if result["status"] != "completed":
            raise RuntimeError(f"route {result['status']}: {result.get('error')}")
        return result

    def stop(self):
        status = self.status()
        if status is not None and status["alive"]:
            self._request({"id": uuid.uuid4().hex, "action": "stop"})
            self._wait(lambda s: s["state"] == "holding", 10.0, "stop")

    def relax(self):
        self.take_over()  # the fade starts from a controller that holds the arms
        self._request({"id": uuid.uuid4().hex, "action": "relax"})
        self._wait(lambda s: s["state"] == "relaxed", 20.0, "relax")

    def release(self):
        status = self.status()
        if status is None or not status["alive"]:
            return
        self._request({"id": uuid.uuid4().hex, "action": "close"})
        self._wait(lambda s: s["state"] == "closed", 10.0, "release")

    def _request(self, request):
        path = shlex.quote(self._session_directory() + "/request.json")
        code, _, err = self._robot_link().run(
            f"cat > {path}.tmp && mv {path}.tmp {path}", stdin=json.dumps(request, allow_nan=False)
        )
        if code != 0:
            raise RuntimeError(f"could not send the request: {err[-300:]}")

    def close(self):
        for link in self._links.values():
            link.close()
        self._links.clear()


def connect(config) -> Robot:
    """Return the robot described by a configuration object or a device.json path."""
    config = normalize_config(config) if isinstance(config, dict) else load_config(config)
    return WiredRobot(config)


def png_size(png):
    """(width, height) of PNG bytes."""
    if png[:8] != _PNG or len(png) < 24:
        raise ValueError("not a PNG image")
    return struct.unpack(">II", png[16:24])


def _capture_command(settings):
    command = ["timeout", "--signal=INT", "--kill-after=2s", "15s"]
    command += ["ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error"]
    if settings["kind"] == "rtsp":
        command += ["-rtsp_transport", "tcp", "-i", settings["source"]]
        # The stream is decoded on the robot's control computer: run it at idle
        # priority so it cannot delay the control loop.
        command = ["ionice", "-c3", "nice", "-n", "19", *command]
    else:
        command += ["-f", "v4l2", "-input_format", "mjpeg", "-framerate", "30"]
        command += ["-video_size", settings["size"], "-i", settings["source"]]
        command += ["-vf", r"select=gte(n\,5)"]  # skip the first frames: exposure settles
    command += ["-frames:v", "1", "-threads", "1", "-c:v", "png", "-f", "image2pipe", "pipe:1"]
    return shlex.join(command)


def _checked_state(sample):
    """Validate one onboard sample and add named views of the 35 slots."""
    count = schema.MOTOR_COUNT
    q = schema.vector(sample.get("q"), count, "state q")
    dq = schema.vector(sample.get("dq"), count, "state dq")
    tau = schema.vector(sample.get("tau"), count, "state tau")
    temperature = schema.vector(sample.get("temperature_c"), count, "state temperature")
    if sample.get("mode_machine") != schema.MODE_MACHINE:
        raise RuntimeError(f"unexpected mode_machine {sample.get('mode_machine')}: not an R1-D")
    if sample.get("ticks_seen", 0) < 2:
        raise RuntimeError("robot feedback is not advancing")
    frames = sample["command_frames"]
    faults = [slot for slot, code in enumerate(sample["motor_state"]) if code != 0]
    return {
        "time_unix": float(sample["time_unix"]),
        "tick": int(sample["tick"]),
        "q": q.tolist(),
        "dq": dq.tolist(),
        "tau": tau.tolist(),
        "temperature_c": temperature.tolist(),
        "arms": {hand: q[list(schema.arm_slots(hand))].tolist() for hand in schema.HANDS},
        "waist_yaw": float(q[schema.WAIST_YAW_SLOT]),
        "head": q[list(schema.HEAD_SLOTS)].tolist(),
        "grip": q[list(schema.GRIP_SLOTS)].tolist(),
        "lift": float(q[schema.LIFT_SLOT]),
        # True when some program was publishing joint commands while the sample was taken.
        "commanded": frames["lowcmd"] > 0 or frames["arm_sdk"] > 0,
        "command_frames": dict(frames),
        "motor_faults": faults,
    }
