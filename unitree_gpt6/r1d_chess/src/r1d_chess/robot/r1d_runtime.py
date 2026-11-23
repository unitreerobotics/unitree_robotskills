"""Run the bundled robot runtime (third_party/robot_runtime) with local R1-D settings and use its IPC."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
import uuid
from collections.abc import Mapping


from r1d_chess.paths import ROOT
DEFAULT_CONFIG = ROOT / "config/runtime.json"
R1D_JOINT_GROUPS = (
    "left_arm", "right_arm", "waist", "head", "left_ee", "right_ee",
)
FEEDBACK_LIMIT_TOLERANCE = 0.005


def bounded_qpos(robot_spec, schema_key: str, values):
    """Keep tiny encoder overshoot inside the configured command limits."""
    import numpy as np

    spec = robot_spec.joint_group_spec(schema_key)
    qpos = np.asarray(values, dtype=np.float32).reshape(-1)
    bounds = np.asarray(spec.qpos_limits[schema_key], dtype=np.float32)
    if (
        qpos.shape != (len(bounds),)
        or not np.all(np.isfinite(qpos))
        or np.any(qpos < bounds[:, 0] - FEEDBACK_LIMIT_TOLERANCE)
        or np.any(qpos > bounds[:, 1] + FEEDBACK_LIMIT_TOLERANCE)
    ):
        raise ValueError(f"{schema_key} feedback is outside configured limits.")
    return np.clip(qpos, bounds[:, 0], bounds[:, 1])


def load_experiment(config_path: str | Path = DEFAULT_CONFIG):
    """Resolve one experiment for both server and client IPC fingerprints."""
    local = json.loads(Path(config_path).read_text(encoding="utf-8"))
    if not local.get("network_interface"):
        raise ValueError("config/runtime.json: fill network_interface for this machine (setup step 0)")
    # Bundled minimal Runtime: third_party/robot_runtime (a relative path is resolved against the package root).
    root = (ROOT / Path(local.get("runtime_root") or "third_party/robot_runtime").expanduser()).resolve()
    source = root / "src"
    if not source.is_dir():
        raise FileNotFoundError(f"robot runtime source is missing: {source}")
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

    from robot_runtime.configs.experiment import ExperimentConfig
    from robot_runtime.configs.resolver import resolve_experiment

    resolved = resolve_experiment(root / local["experiment"])
    connections = resolved["robot"]["connections"]
    connections["unitree_sdk2"]["network_interface"] = local[
        "network_interface"
    ]
    if local["camera_enabled"]:
        connections["teleimager"]["host"] = local["camera_host"]
    else:
        connections.pop("teleimager", None)
        resolved["robot"]["spec"]["sensors"].pop("color_camera", None)
        resolved["robot"]["drivers"] = [
            driver for driver in resolved["robot"]["drivers"]
            if driver["type"] != "teleimager.color_camera"
        ]
    # Waist/head motor gains fitted on this robot (hold_joints.json); the source values
    # (50/50/10) left the waist up to 85 mrad from its target after a restart.
    gains = local.get("joint_gains") or {}
    for group in resolved["robot"]["spec"]["body"]["joint_groups"]:
        override = gains.get(group["schema_key"])
        if override:
            for key in ("kp", "kd"):
                if len(override[key]) != len(group[key]):
                    raise ValueError(f"joint_gains {group['schema_key']} {key}: wrong length")
                group[key] = [float(value) for value in override[key]]
    body_driver = next(
        driver for driver in resolved["robot"]["drivers"]
        if driver["component_id"] == "body.r1_a7_d"
    )
    body_driver["restore_motion_mode"] = local["restore_motion_mode"]
    return ExperimentConfig(resolved)


def serve(experiment) -> None:
    """Own the hardware channels; movement starts only on START_CONTROL."""
    import logging_mp

    logging_mp.basicConfig(level=logging_mp.INFO)
    from robot_runtime.drivers.unitree_sdk2.driver import UnitreeSDK2Driver
    from robot_runtime.runtime.ipc.cmd import RuntimeCmdReceiver
    from robot_runtime.runtime.ipc.request import RuntimeRequestReceiver
    from robot_runtime.runtime.ipc.state import RuntimeStateSender
    from robot_runtime.runtime.robot_hardware import RobotHardware
    from robot_runtime.runtime.server import RuntimeServer

    class R1DServer(RuntimeServer):
        def read_joint_group_qpos(self, schema_key: str):
            return bounded_qpos(
                self.robot_spec,
                schema_key,
                super().read_joint_group_qpos(schema_key),
            )

    drivers = {}
    sdk_config = experiment.robot_driver_configs["unitree_sdk2"]
    drivers["unitree_sdk2"] = UnitreeSDK2Driver(
        robot_spec=experiment.robot_spec,
        dds_domain_id=sdk_config["dds_domain_id"],
        network_interface=sdk_config.get("network_interface"),
    )
    if "teleimager" in experiment.robot_driver_configs:
        # Not part of the bundled Runtime: the chess service reads the cameras itself (r1d_cameras).
        # camera_enabled=true needs a full robot runtime checkout and the external teleimager client.
        from robot_runtime.drivers.teleimager.driver import TeleImagerDriver
        camera_config = experiment.robot_driver_configs["teleimager"]
        drivers["teleimager"] = TeleImagerDriver(
            host=camera_config["host"],
            request_port=camera_config["request_port"],
        )
    endpoints = experiment.runtime_endpoints
    server = R1DServer(
        runtime_config=experiment.runtime_config,
        robot_spec=experiment.robot_spec,
        robot_hardware=RobotHardware(
            channel_configs=experiment.robot_channel_configs,
            drivers=drivers,
        ),
        simulation_channel=None,
        state_sender=RuntimeStateSender(endpoint=endpoints["runtime_state"]),
        cmd_receiver=RuntimeCmdReceiver(endpoint=endpoints["runtime_cmd"]),
        request_receiver=RuntimeRequestReceiver(
            endpoint=endpoints["runtime_request"]
        ),
    )
    try:
        server.start()
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


class R1DRuntimeClient:
    """Reusable local IPC client; importing or connecting never takes control."""

    def __init__(self, config_path: str | Path = DEFAULT_CONFIG) -> None:
        experiment = load_experiment(config_path)
        from robot_runtime.runtime.ipc.cmd import RuntimeCmdSender
        from robot_runtime.runtime.ipc.request import RuntimeRequestSender
        from robot_runtime.runtime.ipc.state import RuntimeStateReceiver

        endpoints = experiment.runtime_endpoints
        self.robot_spec = experiment.robot_spec
        self.state_receiver = RuntimeStateReceiver(
            endpoint=endpoints["runtime_state"],
            max_state_age=0.5,
            hydrate_images=False,
        )
        self.cmd_sender = RuntimeCmdSender(endpoint=endpoints["runtime_cmd"])
        self.request_sender = RuntimeRequestSender(
            endpoint=endpoints["runtime_request"]
        )
        self.session_id: str | None = None
        self.sequence = 0
        self.include_base = False

    def connect(self, *, timeout: float = 10.0) -> None:
        self.state_receiver.start(timeout=timeout)

    def state(self):
        return self.state_receiver.latest_state()

    def current_joint_targets(self) -> dict[str, tuple[float, ...]]:
        observation = self.state().observation
        return {
            key: tuple(float(value) for value in bounded_qpos(
                self.robot_spec,
                key,
                observation.joint_group_qpos(key),
            ))
            for key in R1D_JOINT_GROUPS
        }

    def begin(self, *, include_base: bool = False) -> None:
        if self.session_id is not None:
            raise RuntimeError("A control session is already active.")
        state = self.state()
        if state.runtime_error:
            raise RuntimeError(state.runtime_error)
        self.current_joint_targets()
        self.cmd_sender.start()
        session_id = uuid.uuid4().hex
        components = {"joint_group": R1D_JOINT_GROUPS}
        if include_base:
            components["base_group"] = ("base_lift",)
        try:
            self.request_sender.start_control(
                cmd_session_id=session_id,
                cmd_components=components,
            )
        except Exception:
            self.cmd_sender.stop()
            raise
        self.session_id = session_id
        self.sequence = 0
        self.include_base = include_base

    def send_action(self, action) -> None:
        if self.session_id is None:
            raise RuntimeError("Call begin() before sending commands.")
        from robot_runtime.runtime.ipc.cmd import RuntimeCmd

        self.sequence += 1
        self.cmd_sender.send(RuntimeCmd(
            cmd_session_id=self.session_id,
            cmd_sequence=self.sequence,
            timestamp=time.monotonic(),
            action=action,
        ))

    def send_joint_targets(
        self,
        targets: Mapping[str, tuple[float, ...]],
        qtau: Mapping[str, tuple[float, ...]] | None = None,
    ) -> None:
        """Send all six joint groups; caller must stream before cmd_timeout.

        Optional ``qtau`` adds motor feedforward (N·m) for the arm, head and waist groups.
        The Runtime forwards it unchanged to LowCmd ``tau`` beside its PD gains.
        """
        if self.include_base:
            raise RuntimeError("Base control requires send_action() with base_group.")
        if set(targets) != set(R1D_JOINT_GROUPS):
            raise ValueError(f"Joint targets must contain {R1D_JOINT_GROUPS!r}.")
        qtau = dict(qtau or {})
        if not set(qtau) <= {"left_arm", "right_arm", "head", "waist"}:
            raise ValueError("qtau is supported for the arm, head and waist groups only.")
        for key, values in qtau.items():
            if len(values) != len(targets[key]) or not all(
                    math.isfinite(float(value)) for value in values):
                raise ValueError(f"qtau for {key} must be finite and match qpos length.")
        from robot_runtime.schema.frame import ActionData
        from robot_runtime.schema.joint_group import JointGroupData

        self.send_action(ActionData(joint_groups=[
            JointGroupData(
                schema_key=key,
                qpos=[float(value) for value in targets[key]],
                qtau=([float(value) for value in qtau[key]] if key in qtau else None),
            )
            for key in R1D_JOINT_GROUPS
        ]))

    def end(self) -> None:
        session_id = self.session_id
        self.session_id = None
        self.sequence = 0
        self.include_base = False
        try:
            if session_id is not None:
                self.request_sender.stop_control(cmd_session_id=session_id)
        finally:
            self.cmd_sender.stop()

    def close(self) -> None:
        try:
            self.end()
        finally:
            self.request_sender.stop()
            self.state_receiver.stop()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("config", "serve", "status"))
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    experiment = load_experiment(args.config)
    if args.command == "config":
        print(json.dumps({
            "network_interface": experiment.robot_driver_configs[
                "unitree_sdk2"
            ]["network_interface"],
            "camera_enabled": "teleimager" in experiment.robot_driver_configs,
            "endpoints": experiment.runtime_endpoints,
        }, indent=2))
    elif args.command == "serve":
        serve(experiment)
    else:
        with R1DRuntimeClient(args.config) as client:
            state = client.state()
            print(json.dumps({
                "control_active": state.control_active,
                "runtime_error": state.runtime_error,
                "available_cmd_components": state.available_cmd_components,
                "joint_targets": client.current_joint_targets(),
            }, indent=2))


if __name__ == "__main__":
    main()
