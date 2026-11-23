"""Persistent runtime server that owns robot and simulation resources."""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING, cast

import logging_mp
import numpy as np
import zmq

from robot_runtime.configs.sections.robot.composition import RobotSpec
from robot_runtime.configs.sections.runtime import (
    RuntimeControlMode,
    RuntimeConfig,
    resolve_motion_schema_keys,
)
from robot_runtime.interfaces.robot.cmd.base_group import BaseGroupCmd
from robot_runtime.interfaces.robot.cmd.joint_group import JointGroupCmd
from robot_runtime.interfaces.robot.cmd.pose_82d import Pose82dCmd
from robot_runtime.interfaces.robot.component import ComponentCmdValue
from robot_runtime.interfaces.simulation.channel import (
    SimulationChannel,
    SimulationResetScope,
)
from robot_runtime.runtime.cmd_loop.base import RobotCmdLoop
from robot_runtime.runtime.cmd_loop.joint_group import JointCmdLoop
from robot_runtime.runtime.cmd_loop.pose_82d import Pose82dCmdLoop
from robot_runtime.runtime.ipc.cmd import RuntimeCmd, RuntimeCmdReceiver
from robot_runtime.runtime.ipc.request import (
    RuntimeRequest,
    RuntimeRequestOperation,
    RuntimeRequestReceiver,
)
from robot_runtime.runtime.ipc.shared_image import ImageFrameRef, SharedImageWriter
from robot_runtime.runtime.ipc.state import RuntimeState, RuntimeStateSender
from robot_runtime.runtime.robot_hardware import RobotHardware
from robot_runtime.runtime.safety.thermal import ThermalSafety
from robot_runtime.schema.base_group import BaseGroupData
from robot_runtime.schema.frame import ActionData, ObservationData
from robot_runtime.schema.joint_group import JointGroupData
from robot_runtime.schema.sensor import IMUData, ImageData

if TYPE_CHECKING:
    from robot_runtime.multibody.dynamics.pinocchio_gravity import (
        PinocchioGravityCompensator,
    )


logger_mp = logging_mp.getLogger(__name__)

RESET_POSE_HEIGHT_TOLERANCE = 0.01
RESET_POSE_HEIGHT_GAIN = 2.0
RESET_POSE_HEIGHT_VELOCITY_LIMIT = 1.0
RESET_POSE_SETTLING_TIME = 10.0


class RuntimeServer:
    """Coordinate injected runtime resources and serve control over local IPC."""

    def __init__(
        self,
        *,
        runtime_config: RuntimeConfig,
        robot_spec: RobotSpec,
        robot_hardware: RobotHardware,
        simulation_channel: SimulationChannel | None,
        state_sender: RuntimeStateSender,
        cmd_receiver: RuntimeCmdReceiver,
        request_receiver: RuntimeRequestReceiver,
        display: bool = False,
        display_delay: bool = False,
    ) -> None:
        # Configuration and derived timing limits.
        self.runtime_config = runtime_config
        self.robot_spec = robot_spec
        self.cmd_timeout = self.runtime_config.cmd_timeout
        self.control_timeout = max(0.5, self.cmd_timeout * 3.0)
        self.state_period = (
            1.0 / self.runtime_config.state_frequency
        )
        self.display = display
        self.display_delay = display_delay

        # Connected channels and state publication resources.
        self.robot_hardware = robot_hardware
        self.simulation_channel = simulation_channel
        self.state_sender = state_sender
        self.cmd_receiver = cmd_receiver
        self.request_receiver = request_receiver
        self.control_poller: zmq.Poller | None = None
        self.shared_image_writer = SharedImageWriter()
        self.image_lock = threading.Lock()
        self.latest_image_data: tuple[ImageData, ...] = ()
        self.latest_image_refs: tuple[ImageFrameRef, ...] = ()
        self.next_image_warning_time = 0.0
        self.state_bindings: dict[str, list[str]] = {}
        self.available_cmd_components: dict[str, list[str]] = {}
        self.display_streams: dict[str, dict[str, object]] = {}

        # Active control session and the latest published runtime status.
        # Lifecycle operations may block; keep them off the state lock.
        self.control_lock = threading.Lock()
        self.control_lifecycle_lock = threading.Lock()
        self._reset_control_state()
        self.reset_pose_active = False
        self.reset_pose_generation = 0
        self.reset_pose_thread: threading.Thread | None = None
        self.reset_pose_cancel = threading.Event()
        self.reset_pose_loop: RobotCmdLoop | None = None
        self.last_runtime_error: str | None = None
        self.stop_event = threading.Event()
        self.threads: list[threading.Thread] = []
        self.is_running = False

    def start(self) -> None:
        if self.is_running:
            return
        try:
            self.robot_hardware.connect()
            if self.simulation_channel is not None:
                self.simulation_channel.start()
            self.state_bindings = {
                section: list(schema_keys)
                for section, schema_keys in self.robot_hardware.state_routes.items()
            }
            self.available_cmd_components = {
                section: list(schema_keys)
                for section, schema_keys in self.robot_hardware.cmd_routes.items()
            }
            if self.runtime_config.control_mode in {
                RuntimeControlMode.DUAL_ARM, RuntimeControlMode.UPPER_BODY,
            }:
                body_keys = self.robot_spec.joint_group_specs[0].schema_keys
                action_keys = resolve_motion_schema_keys(
                    self.runtime_config.control_mode, body_keys,
                )["joint_group"]
                self.available_cmd_components.pop("base_group", None)
                self.available_cmd_components["joint_group"] = [
                    key
                    for key in self.available_cmd_components.get(
                        "joint_group", ()
                    )
                    if key not in body_keys or key in action_keys
                ]
            camera_channel = self.robot_hardware.color_camera_channel()
            if camera_channel is not None:
                stream_bindings = getattr(
                    camera_channel,
                    "stream_bindings",
                    {},
                )
                display_stream_config = getattr(
                    camera_channel,
                    "display_stream_config",
                    None,
                )
                if callable(display_stream_config):
                    self.display_streams = {
                        stream_key: display_stream_config(stream_key)
                        for stream_key in stream_bindings.values()
                    }
            self.build_observation()
            self.state_sender.start()
            self.cmd_receiver.start()
            self.request_receiver.start()
            self.control_poller = zmq.Poller()
            self.control_poller.register(
                self.cmd_receiver.socket,
                zmq.POLLIN,
            )
            self.control_poller.register(
                self.request_receiver.socket,
                zmq.POLLIN,
            )

            self.stop_event.clear()
            self.is_running = True
            thread_targets = []
            if self.state_bindings.get("color_camera"):
                thread_targets.append(
                    (self.image_loop, "robot-runtime-image")
                )
            thread_targets.extend(
                (
                    (self.state_loop, "robot-runtime-state"),
                    (self.control_loop, "robot-runtime-control"),
                )
            )
            for target, name in thread_targets:
                thread = threading.Thread(
                    target=target,
                    name=name,
                    daemon=True,
                )
                thread.start()
                self.threads.append(thread)
        except Exception:
            try:
                self.stop()
            except Exception:
                logger_mp.exception("Failed to stop runtime after startup error.")
            raise
        logger_mp.info("Runtime server ready.")

    def stop(self) -> None:
        self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=1.0)
        self.threads.clear()
        self.control_poller = None
        self.is_running = False

        try:
            self.request_receiver.stop()
            self.cmd_receiver.stop()
            self.state_sender.stop()
        finally:
            self.stop_reset_pose()
            self.stop_control(
                cmd_session_id=self.cmd_session_id,
                force=True,
            )
            self.state_bindings.clear()
            self.available_cmd_components.clear()
            self.display_streams.clear()
            with self.image_lock:
                self.latest_image_data = ()
                self.latest_image_refs = ()
            self.shared_image_writer.close()
            if self.simulation_channel is not None:
                try:
                    self.simulation_channel.stop()
                except Exception:
                    logger_mp.exception("Failed to stop simulation channel.")
            self.robot_hardware.disconnect()

    def state_loop(self) -> None:
        sequence = 0
        joint_output_line_count = 0
        while not self.stop_event.is_set():
            started_at = time.monotonic()
            sequence += 1
            try:
                runtime_state = self.build_runtime_state(sequence)
                if self.display:
                    joint_output_lines = [
                        "Joint qpos:",
                        *(
                            f"  {group.schema_key}: {group.qpos}"
                            for group in (
                                runtime_state.observation.joint_groups or ()
                            )
                        ),
                    ]
                    if joint_output_line_count:
                        print(f"\033[{joint_output_line_count}F", end="")
                    print(
                        "\n".join(
                            f"\033[2K{line}" for line in joint_output_lines
                        ),
                        flush=True,
                    )
                    joint_output_line_count = len(joint_output_lines)
                self.state_sender.send(runtime_state)
            except Exception as exc:
                self.set_runtime_error(f"state publication failed: {exc}")
                logger_mp.exception("Runtime state publication failed.")
            elapsed = time.monotonic() - started_at
            self.stop_event.wait(max(0.0, self.state_period - elapsed))

    def image_loop(self) -> None:
        color_camera_keys = tuple(
            self.robot_hardware.state_routes.get("color_camera", {})
        )
        while not self.stop_event.is_set():
            started_at = time.monotonic()
            try:
                image_data, image_refs = self.build_image_snapshot(
                    color_camera_keys
                )
                with self.image_lock:
                    self.latest_image_data = tuple(image_data)
                    self.latest_image_refs = tuple(image_refs)
                self.next_image_warning_time = 0.0
            except Exception:
                now = time.monotonic()
                if now >= self.next_image_warning_time:
                    logger_mp.exception(
                        "Runtime image acquisition failed; retaining the "
                        "latest image snapshot."
                    )
                    self.next_image_warning_time = now + 1.0
            elapsed = time.monotonic() - started_at
            self.stop_event.wait(max(0.0, self.state_period - elapsed))

    def control_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                poller = self.control_poller
                if poller is None:
                    break
                # Failed commands must not prevent an expired session from stopping.
                self.stop_expired_control()
                events = dict(poller.poll(20))
                if self.request_receiver.socket in events:
                    try:
                        error_message = self.handle_request(
                            self.request_receiver.receive()
                        )
                    except Exception as exc:
                        error_message = str(exc)
                        logger_mp.exception("Runtime request handling failed.")
                    self.request_receiver.reply(error_message)

                if self.cmd_receiver.socket in events:
                    self.accept_cmd(self.cmd_receiver.receive())
            except Exception as exc:
                if self.stop_event.is_set():
                    break
                self.set_runtime_error(f"control handling failed: {exc}")
                logger_mp.exception("Runtime control handling failed.")

    def build_observation(self) -> tuple[ObservationData, list[ImageFrameRef]]:
        state_routes = self.robot_hardware.state_routes
        joint_group_keys = tuple(state_routes.get("joint_group", {}))
        base_group_keys = tuple(state_routes.get("base_group", {}))
        imu_keys = tuple(state_routes.get("imu", {}))

        joint_groups = []
        if joint_group_keys:
            joint_states = self.robot_hardware.read_states(
                "joint_group",
                joint_group_keys,
            )
            for schema_key in joint_group_keys:
                state = joint_states[schema_key]
                joint_group = JointGroupData(schema_key=schema_key).update(
                    qpos=state.qpos,
                    qvel=state.qvel,
                    qtau=state.qtau,
                )
                for field_name in ("casing_temp", "winding_temp"):
                    value = getattr(state, field_name)
                    if value is not None and np.all(np.isfinite(value)):
                        joint_group.update(**{field_name: value})
                joint_groups.append(joint_group)

        base_groups = []
        if base_group_keys:
            base_states = self.robot_hardware.read_states(
                "base_group",
                base_group_keys,
            )
            for schema_key in base_group_keys:
                state = base_states[schema_key]
                base_groups.append(
                    BaseGroupData(schema_key=schema_key).update(**vars(state))
                )

        imu_data = []
        if imu_keys:
            imu_states = self.robot_hardware.read_states("imu", imu_keys)
            for schema_key in imu_keys:
                state = imu_states[schema_key]
                imu_data.append(
                    IMUData(schema_key=schema_key).update(
                        linear_acceleration=state.linear_acceleration,
                        angular_velocity=state.angular_velocity,
                        orientation_quat_wxyz=state.orientation_quat_wxyz,
                    )
                )

        with self.image_lock:
            image_data = list(self.latest_image_data)
            image_refs = list(self.latest_image_refs)

        return (
            ObservationData(
                joint_groups=joint_groups or None,
                base_group=base_groups or None,
                color=image_data or None,
                imu=imu_data or None,
            ),
            image_refs,
        )

    def build_image_snapshot(
        self,
        schema_keys: tuple[str, ...],
    ) -> tuple[list[ImageData], list[ImageFrameRef]]:
        image_states = self.robot_hardware.read_states(
            "color_camera",
            schema_keys,
        )
        image_data = []
        image_refs = []
        for schema_key in schema_keys:
            state = image_states[schema_key]
            raw = bytes(state.raw)
            image_refs.append(
                self.shared_image_writer.write(
                    stream_key=f"jpg:{schema_key}",
                    schema_key=schema_key,
                    data=raw,
                    shape=state.shape,
                    format=state.format,
                    is_padded=state.is_padded,
                )
            )
            image_data.append(
                ImageData(schema_key=schema_key).update(
                    raw=None,
                    shape=state.shape,
                    format=state.format,
                    is_padded=state.is_padded,
                )
            )
        image_refs.extend(self.build_bgr_image_refs(schema_keys))
        return image_data, image_refs

    def build_bgr_image_refs(
        self,
        schema_keys: tuple[str, ...],
    ) -> list[ImageFrameRef]:
        camera_channel = self.robot_hardware.color_camera_channel()
        if camera_channel is None:
            return []
        read_bgr_frame = getattr(camera_channel, "read_bgr_frame", None)
        if not callable(read_bgr_frame):
            return []

        frame_refs = []
        for schema_key in schema_keys:
            try:
                frame = np.asarray(read_bgr_frame(schema_key))
            except RuntimeError:
                continue
            frame_refs.append(
                self.shared_image_writer.write(
                    stream_key=f"bgr:{schema_key}",
                    schema_key=schema_key,
                    data=frame.tobytes(order="C"),
                    shape=frame.shape,
                    format="bgr",
                )
            )
        return frame_refs

    def build_runtime_state(self, sequence: int) -> RuntimeState:
        with self.control_lock:
            accepted_cmd = self.last_accepted_cmd
            control_active = self.cmd_session_id is not None
            reset_pose_active = self.reset_pose_active
            reset_pose_generation = self.reset_pose_generation
            cmd_loop = self.cmd_loop
            reset_pose_loop = self.reset_pose_loop
            cmd_error = None if cmd_loop is None else cmd_loop.cmd_error
            reset_error = (
                None if reset_pose_loop is None else reset_pose_loop.cmd_error
            )
            thermal_protection_active = any(
                loop.thermal_safety is not None and loop.thermal_safety.active
                for loop in (cmd_loop, reset_pose_loop)
                if loop is not None
            )
            runtime_error = self.last_runtime_error
        if cmd_error is not None:
            runtime_error = str(cmd_error)
        if reset_error is not None:
            runtime_error = str(reset_error)
        simulation_state = (
            None
            if self.simulation_channel is None
            else self.simulation_channel.read_state()
        )
        observation, image_refs = self.build_observation()
        return RuntimeState(
            state_bindings=self.state_bindings,
            available_cmd_components=self.available_cmd_components,
            display_streams=self.display_streams,
            sequence=sequence,
            observation=observation,
            image_refs=image_refs,
            accepted_cmd=accepted_cmd,
            control_active=control_active,
            reset_pose_active=reset_pose_active,
            reset_pose_generation=reset_pose_generation,
            thermal_protection_active=thermal_protection_active,
            simulation_state=simulation_state,
            runtime_error=runtime_error,
        )

    def handle_request(self, request: RuntimeRequest) -> str | None:
        if request.operation is RuntimeRequestOperation.PREPARE_CONTROL:
            return self.prepare_control(request)
        if request.operation is RuntimeRequestOperation.START_CONTROL:
            return self.start_control(request)
        if request.operation is RuntimeRequestOperation.STOP_CONTROL:
            stopped = self.stop_control(
                cmd_session_id=request.cmd_session_id,
                restore_standing=request.restore_standing,
            )
            return None if stopped else "invalid control session"
        if request.operation in {
            RuntimeRequestOperation.RESET_SIMULATION_OBJECTS,
            RuntimeRequestOperation.RESET_SIMULATION_SCENE,
        }:
            if self.simulation_channel is None:
                return "simulation channel is unavailable"
            self.simulation_channel.request_reset(
                SimulationResetScope.OBJECTS
                if request.operation
                is RuntimeRequestOperation.RESET_SIMULATION_OBJECTS
                else SimulationResetScope.SCENE
            )
            return None
        if request.operation is RuntimeRequestOperation.RESET_POSE:
            return self.start_reset_pose(cmd_session_id=request.cmd_session_id)
        if request.operation is RuntimeRequestOperation.STOP_RESET_POSE:
            if self.prepared_cmd_session_id is not None:
                return "use stop_control to stop the prepared session"
            self.stop_reset_pose()
            return None
        return f"unsupported runtime operation {request.operation.value!r}"

    def prepare_control(self, request: RuntimeRequest) -> str | None:
        """Start the body service without constructing or publishing commands."""
        if (
            self.runtime_config.control_mode
            is not RuntimeControlMode.DECOUPLED_WBC
        ):
            return "service preparation requires decoupled_wbc"
        cmd_session_id = request.cmd_session_id
        if not cmd_session_id:
            return "command session id is required"
        with self.control_lifecycle_lock:
            with self.control_lock:
                if self.cmd_session_id is not None:
                    return "a control session is active"
                if self.prepared_cmd_session_id is not None:
                    if self.prepared_cmd_session_id != cmd_session_id:
                        return "another control session is prepared"
                    if self.last_runtime_error is not None:
                        return self.last_runtime_error
                    self.control_deadline = (
                        time.monotonic() + self.control_timeout
                    )
                    return None
                if request.renew_only:
                    return "prepared control session expired or was stopped"
                if self.reset_pose_loop is not None or self.reset_pose_active:
                    return "reset_pose owns robot control"
            body_channel = self.robot_hardware.channel("body")
            enter_control = getattr(body_channel, "enter_control", None)
            exit_control = getattr(body_channel, "exit_control", None)
            if not callable(enter_control) or not callable(exit_control):
                return "body channel does not support service preparation"
            with self.control_lock:
                self.prepared_cmd_session_id = cmd_session_id
                self.last_runtime_error = None
            try:
                enter_control()
            except Exception as exc:
                message = f"failed to prepare control: {exc}"
                self.set_runtime_error(message)
                return message
            finally:
                with self.control_lock:
                    self.control_deadline = (
                        time.monotonic() + self.control_timeout
                    )
        return None

    def start_control(self, request: RuntimeRequest) -> str | None:
        cmd_session_id = request.cmd_session_id
        if not cmd_session_id:
            return "command session id is required"
        try:
            (
                joint_group_schema_keys,
                base_group_schema_key,
                pose_82d_schema_key,
            ) = self.resolve_cmd_components(request)
        except Exception as exc:
            return str(exc)

        with self.control_lifecycle_lock:
            with self.control_lock:
                if self.control_exit_pending:
                    return "standing exit is pending; Runtime is holding control"
                if self.reset_pose_active:
                    return "reset_pose is active"
                if self.prepared_cmd_session_id is not None:
                    if self.prepared_cmd_session_id != cmd_session_id:
                        return "another control session is prepared"
                    if self.last_runtime_error is not None:
                        return self.last_runtime_error
                    if self.reset_pose_loop is None:
                        return "prepared control requires a completed reset_pose"
                if self.cmd_session_id is not None:
                    if (
                        self.cmd_session_id != cmd_session_id
                        or self.control_joint_group_schema_keys
                        != joint_group_schema_keys
                        or self.control_base_group_schema_key
                        != base_group_schema_key
                        or self.control_pose_82d_schema_key
                        != pose_82d_schema_key
                    ):
                        return "another control session is active"
                    self.control_deadline = (
                        time.monotonic() + self.control_timeout
                    )
                    return None
                reset_pose_loop = self.reset_pose_loop

            try:
                cmd_loop = self.build_cmd_loop(
                    joint_group_schema_keys=joint_group_schema_keys,
                    base_group_schema_key=base_group_schema_key,
                    pose_82d_schema_key=pose_82d_schema_key,
                )
                if reset_pose_loop is None:
                    cmd_loop.start()
                else:
                    if reset_pose_loop.cmd_error is not None:
                        raise RuntimeError(
                            "reset_pose hold command loop failed"
                        ) from reset_pose_loop.cmd_error
                    control_already_entered = (
                        reset_pose_loop.control_active
                    )
                    reset_pose_loop.stop(release_control=False)
                    with self.control_lock:
                        if self.reset_pose_loop is reset_pose_loop:
                            self.reset_pose_loop = None
                    cmd_loop.start(
                        control_already_entered=control_already_entered
                    )
                    logger_mp.info(
                        "Runtime control adopted reset_pose hold without "
                        "switching motion services."
                    )
            except Exception as exc:
                message = f"failed to start control: {exc}"
                if self.prepared_cmd_session_id is not None:
                    self.set_runtime_error(message)
                return message

            with self.control_lock:
                self.cmd_loop = cmd_loop
                self.cmd_session_id = cmd_session_id
                self.prepared_cmd_session_id = None
                self.control_joint_group_schema_keys = joint_group_schema_keys
                self.control_base_group_schema_key = base_group_schema_key
                self.control_pose_82d_schema_key = pose_82d_schema_key
                self.control_deadline = time.monotonic() + self.control_timeout
                self.last_cmd_sequence = -1
                self.last_accepted_cmd = None
                self.last_runtime_error = None
        return None

    def resolve_cmd_components(
        self,
        request: RuntimeRequest,
    ) -> tuple[tuple[str, ...], str | None, str | None]:
        mode = self.runtime_config.control_mode
        motion_schema_keys = resolve_motion_schema_keys(
            mode, self.robot_spec.joint_group_specs[0].schema_keys,
        )
        requested_cmd_components = {
            section: tuple(component_names)
            for section, component_names in (
                request.cmd_components or {}
            ).items()
        }
        unsupported_sections = set(requested_cmd_components) - {
            "joint_group",
            "base_group",
            "pose_82d",
        }
        if unsupported_sections:
            raise ValueError(
                "control request contains unsupported command sections "
                f"{tuple(sorted(unsupported_sections))!r}"
            )
        for section, component_names in requested_cmd_components.items():
            if len(component_names) != len(set(component_names)):
                raise ValueError(
                    f"control request contains duplicate {section} components"
                )

        required_joint_group_schema_keys = tuple(
            motion_schema_keys.get("joint_group", ())
        )
        requested_joint_group_schema_keys = requested_cmd_components.get(
            "joint_group",
            (),
        )
        missing_joint_group_schema_keys = set(
            required_joint_group_schema_keys
        ) - set(requested_joint_group_schema_keys)
        if missing_joint_group_schema_keys:
            raise ValueError(
                "control request is missing required joint groups "
                f"{tuple(sorted(missing_joint_group_schema_keys))!r}"
            )

        declared_end_effector_keys = set(
            self.robot_spec.schema_keys("end_effector")
        )
        optional_joint_group_schema_keys = (
            set(requested_joint_group_schema_keys)
            - set(required_joint_group_schema_keys)
        )
        unsupported_joint_group_schema_keys = (
            optional_joint_group_schema_keys
            - declared_end_effector_keys
            - {"head"}
        )
        if unsupported_joint_group_schema_keys:
            raise ValueError(
                "control request contains unsupported joint groups "
                f"{tuple(sorted(unsupported_joint_group_schema_keys))!r}"
            )
        if "head" in optional_joint_group_schema_keys:
            if mode is not RuntimeControlMode.MOBILE_WBC:
                raise ValueError(
                    f"runtime.control_mode={mode.value!r} does not support "
                    "head control"
                )
            if "head" not in self.robot_spec.joint_group_specs[0].schema_keys:
                raise ValueError(
                    "robot body does not declare a head joint group"
                )

        requested_head_schema_keys = (
            ("head",)
            if "head" in optional_joint_group_schema_keys
            else ()
        )
        joint_group_schema_keys = (
            *required_joint_group_schema_keys,
            *requested_head_schema_keys,
            *(
                schema_key
                for schema_key in requested_joint_group_schema_keys
                if schema_key in declared_end_effector_keys
            ),
        )
        for route_name, route_keys in (
            (
                "unreadable",
                set(joint_group_schema_keys)
                - set(self.robot_hardware.state_routes.get("joint_group", {})),
            ),
            (
                "unwritable",
                set(joint_group_schema_keys)
                - set(self.robot_hardware.cmd_routes.get("joint_group", {})),
            ),
        ):
            if route_keys:
                raise ValueError(
                    f"control request contains {route_name} joint groups "
                    f"{tuple(sorted(route_keys))!r}"
                )

        base_group_schema_key = None
        requested_base_group_schema_keys = requested_cmd_components.get(
            "base_group",
            (),
        )
        if requested_base_group_schema_keys:
            if mode not in {
                RuntimeControlMode.MOBILE_WBC,
                RuntimeControlMode.DECOUPLED_WBC,
                RuntimeControlMode.HYBRID_WBC,
            }:
                raise ValueError(
                    f"runtime.control_mode={mode.value!r} does not support "
                    "base_group control"
                )
            base_group_schema_keys = self.robot_spec.schema_keys("base_group")
            if len(base_group_schema_keys) != 1:
                raise ValueError(
                    "base_group control requires exactly one robot component"
                )
            base_group_schema_key = base_group_schema_keys[0]
            if requested_base_group_schema_keys != (base_group_schema_key,):
                raise ValueError(
                    "control request base_group must be "
                    f"{base_group_schema_key!r}"
                )
            if (
                mode is not RuntimeControlMode.HYBRID_WBC
                and base_group_schema_key not in self.robot_hardware.state_routes.get(
                    "base_group", {}
                )
            ):
                raise ValueError(
                    f"base_group {base_group_schema_key!r} is unreadable"
                )
            if base_group_schema_key not in self.robot_hardware.cmd_routes.get(
                "base_group", {}
            ):
                raise ValueError(
                    f"base_group {base_group_schema_key!r} is unwritable"
                )
        elif mode in {RuntimeControlMode.DECOUPLED_WBC, RuntimeControlMode.HYBRID_WBC}:
            raise ValueError(f"{mode.value} control requires a base_group")

        pose_82d_schema_keys = tuple(
            motion_schema_keys.get("pose_82d", ())
        )
        requested_pose_82d_schema_keys = requested_cmd_components.get(
            "pose_82d",
            (),
        )
        if requested_pose_82d_schema_keys != pose_82d_schema_keys:
            raise ValueError(
                "control request pose_82d components must be "
                f"{pose_82d_schema_keys!r}"
            )
        pose_82d_schema_key = (
            pose_82d_schema_keys[0] if pose_82d_schema_keys else None
        )
        if (
            pose_82d_schema_key is not None
            and pose_82d_schema_key
            not in self.robot_hardware.cmd_routes.get("pose_82d", {})
        ):
            raise ValueError(
                f"pose_82d {pose_82d_schema_key!r} is unwritable"
            )
        return (
            joint_group_schema_keys,
            base_group_schema_key,
            pose_82d_schema_key,
        )

    def stop_control(
        self,
        *,
        cmd_session_id: str | None,
        force: bool = False,
        restore_standing: bool = False,
    ) -> bool:
        if restore_standing and self.simulation_channel is not None:
            restore_standing = False
        with self.control_lifecycle_lock:
            with self.control_lock:
                owner = self.cmd_session_id or self.prepared_cmd_session_id
                if owner is None:
                    return True
                if not force and cmd_session_id != owner:
                    return False
                if (
                    self.control_exit_pending
                    and not force
                    and not restore_standing
                ):
                    raise RuntimeError(
                        "Standing exit is pending; keep Runtime running until "
                        "the robot has been secured and control is ended with R3."
                    )
                prepared = self.prepared_cmd_session_id is not None
                cmd_loop = self.cmd_loop
                if restore_standing and not force:
                    if (
                        self.runtime_config.control_mode
                        is not RuntimeControlMode.DECOUPLED_WBC
                        or prepared
                        or not isinstance(cmd_loop, JointCmdLoop)
                    ):
                        raise RuntimeError(
                            "Standing exit requires active decoupled WBC control."
                        )
                    self.control_exit_pending = True
                    # Runtime owns the hold after Policy exits; do not expire it.
                    self.control_deadline = float("inf")
                elif prepared:
                    self.reset_pose_cancel.set()
                    if self.last_runtime_error is None:
                        self.last_runtime_error = "Prepared control was stopped."
                else:
                    self._reset_control_state()
            if restore_standing and not force:
                try:
                    cmd_loop.hold_position()
                    self.robot_hardware.channel("body").exit_control(
                        restore_standing=True,
                    )
                    cmd_loop.stop(release_control=False)
                except Exception as exc:
                    message = (
                        f"Standing exit incomplete: {exc} Keep Runtime running."
                    )
                    self.set_runtime_error(message)
                    raise RuntimeError(message) from exc
                with self.control_lock:
                    self._reset_control_state()
                return True
            try:
                if cmd_loop is not None:
                    cmd_loop.stop()
            except Exception:
                logger_mp.exception("Failed to stop runtime command loop.")
        if prepared:
            # The reset worker may be restoring the service. Join it before
            # retrying restoration, outside the lifecycle lock it needs.
            self.stop_reset_pose()
            with self.control_lifecycle_lock:
                self.robot_hardware.channel("body").exit_control()
                with self.control_lock:
                    self._reset_control_state()
        return True

    def start_reset_pose(self, *, cmd_session_id: str | None = None) -> str | None:
        reset_pose = self.robot_spec.reset_pose
        if reset_pose is None:
            return "robot reset_pose is not configured"
        reset_joint_velocity_limit = cast(
            float,
            reset_pose["joint_velocity_limit"],
        )
        reset_joint_tolerance = cast(
            float,
            reset_pose["joint_tolerance"],
        )
        reset_joint_positions = cast(
            dict[str, tuple[float, ...]],
            reset_pose["joint_groups"],
        )
        reset_base_height = cast(float | None, reset_pose["base_height"])

        cmd_loop = None
        with self.control_lifecycle_lock:
            with self.control_lock:
                if cmd_session_id != self.prepared_cmd_session_id:
                    return "reset_pose does not own the prepared control session"
                if (
                    cmd_session_id is not None
                    and self.last_runtime_error is not None
                ):
                    return self.last_runtime_error
                if self.cmd_session_id is not None:
                    return "a control session is active"
                if self.reset_pose_active:
                    return "reset_pose is already active"
                reset_pose_loop = self.reset_pose_loop
                if (
                    reset_pose_loop is not None
                    and reset_pose_loop.cmd_error is None
                ):
                    self.reset_pose_generation += 1
                    self.last_runtime_error = None
                    logger_mp.info(
                        "Runtime reset_pose is already reached and holding."
                    )
                    return None

            if reset_pose_loop is not None:
                try:
                    reset_pose_loop.stop()
                except Exception:
                    logger_mp.exception(
                        "Failed to stop failed reset_pose hold loop."
                    )
                with self.control_lock:
                    if self.reset_pose_loop is reset_pose_loop:
                        self.reset_pose_loop = None

            try:
                (
                    joint_group_schema_keys,
                    base_group_schema_key,
                ) = self.resolve_reset_pose_schema_keys()
                runtime_velocity_limit = (
                    self.runtime_config.cmd_joint_velocity_limit
                )
                if runtime_velocity_limit is None:
                    raise RuntimeError(
                        "reset_pose requires a runtime joint velocity limit"
                    )
                # Reset motion may be slower, but never exceed the runtime
                # command limit configured for the experiment.
                velocity_limit = min(
                    reset_joint_velocity_limit,
                    runtime_velocity_limit,
                )
                cmd_loop = self.build_cmd_loop(
                    joint_group_schema_keys=joint_group_schema_keys,
                    base_group_schema_key=base_group_schema_key,
                    pose_82d_schema_key=None,
                    velocity_limit=velocity_limit,
                    cmd_joint_group_order=joint_group_schema_keys,
                )
                joint_qpos = {
                    schema_key: self.read_joint_group_qpos(schema_key)
                    for schema_key in joint_group_schema_keys
                }
                arm_gravity_compensator = None
                joint_qtau = None
                if (
                    self.runtime_config.control_mode
                    is RuntimeControlMode.DECOUPLED_WBC
                    or (
                        self.runtime_config.control_mode in {
                            RuntimeControlMode.DUAL_ARM, RuntimeControlMode.UPPER_BODY,
                        }
                        and self.robot_spec.dual_arm_kinematics is not None
                    )
                ) and not (
                    isinstance(cmd_loop, JointCmdLoop)
                    and cmd_loop.compute_gravity_qtau is not None
                ):
                    # Load the arm model only for resets using gravity feedforward.
                    from robot_runtime.multibody.dynamics.pinocchio_gravity import (
                        PinocchioGravityCompensator,
                    )
                    from robot_runtime.multibody.pinocchio_model import (
                        load_reduced_model,
                    )

                    kinematics = self.robot_spec.dual_arm_kinematics
                    if kinematics is None:
                        raise RuntimeError(
                            "Decoupled WBC reset_pose requires dual-arm kinematics"
                        )
                    model = load_reduced_model(
                        kinematics.urdf_path,
                        (
                            *kinematics.left_arm_joint_names,
                            *kinematics.right_arm_joint_names,
                        ),
                        cache_namespace="dual_arm",
                    )
                    arm_gravity_compensator = PinocchioGravityCompensator(model)
                    arm_qtau = arm_gravity_compensator.compute(
                        np.concatenate(
                            (joint_qpos["left_arm"], joint_qpos["right_arm"])
                        )
                    )
                    left_arm_size = joint_qpos["left_arm"].size
                    joint_qtau = {
                        "left_arm": arm_qtau[:left_arm_size],
                        "right_arm": arm_qtau[left_arm_size:],
                    }
                reset_cmd, targets = self.build_reset_pose_cmd(
                    joint_group_schema_keys,
                    base_group_schema_key,
                    reset_joint_positions,
                    joint_qtau=joint_qtau,
                )
                if joint_qtau is not None:
                    # The initial hold is published before the reset target.
                    joint_cmd_loop = cast(JointCmdLoop, cmd_loop)
                    for schema_key, qtau in joint_qtau.items():
                        joint_cmd_loop.joint_group_cmd(
                            joint_cmd_loop.hold_component_cmds,
                            schema_key,
                        ).qtau = qtau
                max_delta = max(
                    np.max(np.abs(target - joint_qpos[schema_key]))
                    for schema_key, target in targets.items()
                )
                height_delta = 0.0
                if base_group_schema_key is not None and reset_base_height is not None:
                    base_state = self.robot_hardware.read_states(
                        "base_group",
                        (base_group_schema_key,),
                    )[base_group_schema_key]
                    actual_height = (
                        base_state.body_height
                        if self.robot_spec.base_group_spec.height_control_mode == "position"
                        else base_state.lift_height
                    )
                    height_delta = abs(reset_base_height - actual_height)
                    log_height = (
                        logger_mp.warning
                        if height_delta > RESET_POSE_HEIGHT_TOLERANCE
                        else logger_mp.info
                    )
                    log_height(
                        "Runtime reset_pose height: phase=start target=%.4f "
                        "actual=%.4f signed_error=%+.4f",
                        reset_base_height,
                        actual_height,
                        reset_base_height - actual_height,
                    )
                timeout = (
                    max(
                        max_delta / velocity_limit,
                        height_delta / RESET_POSE_HEIGHT_VELOCITY_LIMIT,
                    )
                    + RESET_POSE_SETTLING_TIME
                )
                logger_mp.info(
                    "Runtime reset_pose starting. max_joint_delta=%.4f "
                    "height_delta=%.4f joint_velocity_limit=%.2f "
                    "timeout=%.1fs",
                    max_delta,
                    height_delta,
                    velocity_limit,
                    timeout,
                )
                if self.prepared_cmd_session_id is not None:
                    cmd_loop.start(control_already_entered=True)
                else:
                    cmd_loop.start()
                cmd_loop.update_cmd(reset_cmd, timeout_enabled=False)
            except Exception as exc:
                if cmd_loop is not None:
                    try:
                        cmd_loop.stop()
                    except Exception:
                        logger_mp.exception(
                            "Failed to stop reset_pose command loop."
                        )
                message = f"failed to start reset_pose: {exc}"
                if self.prepared_cmd_session_id is not None:
                    self.set_runtime_error(message)
                return message

            self.reset_pose_cancel.clear()
            with self.control_lock:
                self.reset_pose_active = True
                self.reset_pose_generation += 1
                self.reset_pose_loop = cmd_loop
                self.last_runtime_error = None
                if self.prepared_cmd_session_id is not None:
                    self.control_deadline = (
                        time.monotonic() + self.control_timeout
                    )
            reset_pose_thread = threading.Thread(
                target=self.run_reset_pose,
                args=(
                    cmd_loop,
                    base_group_schema_key,
                    reset_cmd,
                    targets,
                    reset_joint_tolerance,
                    timeout,
                ),
                kwargs={"arm_gravity_compensator": arm_gravity_compensator},
                name="robot-runtime-reset-pose",
                daemon=True,
            )
            self.reset_pose_thread = reset_pose_thread
            reset_pose_thread.start()
        return None

    def stop_reset_pose(self) -> None:
        self.reset_pose_cancel.set()
        reset_pose_thread = self.reset_pose_thread
        if (
            reset_pose_thread is not None
            and reset_pose_thread is not threading.current_thread()
        ):
            timeout = 60.0 if self.prepared_cmd_session_id is not None else 2.0
            reset_pose_thread.join(timeout=timeout)
            if (
                self.prepared_cmd_session_id is not None
                and reset_pose_thread.is_alive()
            ):
                raise RuntimeError("Prepared reset_pose worker has not stopped.")
        with self.control_lifecycle_lock:
            with self.control_lock:
                reset_pose_loop = self.reset_pose_loop
                self.reset_pose_active = False
                self.reset_pose_loop = None
                self.reset_pose_thread = None
            if reset_pose_loop is not None:
                try:
                    reset_pose_loop.stop()
                except Exception:
                    logger_mp.exception(
                        "Failed to stop reset_pose command loop."
                    )

    def run_reset_pose(
        self,
        cmd_loop: RobotCmdLoop,
        base_group_schema_key: str | None,
        reset_cmd: dict[str, dict[str, ComponentCmdValue]],
        targets: dict[str, np.ndarray],
        joint_tolerance: float,
        timeout: float,
        *,
        arm_gravity_compensator: PinocchioGravityCompensator | None = None,
    ) -> None:
        deadline = time.monotonic() + timeout
        next_height_log_time = deadline - timeout + 1.0
        error_message = None
        pose_reached = False
        try:
            if arm_gravity_compensator is not None:
                # Arrival targets omit torque-only groups; commands must keep them.
                reset_joint_group_schema_keys = tuple(
                    schema_key
                    for component_cmds in reset_cmd.values()
                    for schema_key, cmd in component_cmds.items()
                    if isinstance(cmd, JointGroupCmd)
                )
            # PC1 body-height feedback does not accurately track this target.
            # Keep sending the target, but do not claim measured height arrival.
            skip_height_check = (
                base_group_schema_key is not None
                and self.robot_spec.base_group_spec.component_id
                == "base_group.g1_edu_pseudo_base"
            )
            while not self.reset_pose_cancel.wait(0.02):
                if cmd_loop.cmd_error is not None:
                    raise RuntimeError("reset_pose command loop failed") from (
                        cmd_loop.cmd_error
                    )
                joint_qpos = {
                    schema_key: self.read_joint_group_qpos(schema_key)
                    for schema_key in targets
                }
                if arm_gravity_compensator is not None:
                    arm_qtau = arm_gravity_compensator.compute(
                        np.concatenate(
                            (joint_qpos["left_arm"], joint_qpos["right_arm"])
                        )
                    )
                    left_arm_size = joint_qpos["left_arm"].size
                    reset_cmd, _ = self.build_reset_pose_cmd(
                        reset_joint_group_schema_keys,
                        base_group_schema_key,
                        self.robot_spec.reset_pose["joint_groups"],
                        joint_qtau={
                            "left_arm": arm_qtau[:left_arm_size],
                            "right_arm": arm_qtau[left_arm_size:],
                        },
                    )
                    cmd_loop.update_cmd(reset_cmd, timeout_enabled=False)
                joint_abs_errors = {
                    schema_key: np.abs(joint_qpos[schema_key] - target)
                    for schema_key, target in targets.items()
                }
                joint_errors = {
                    schema_key: float(np.max(abs_errors))
                    for schema_key, abs_errors in joint_abs_errors.items()
                }
                joints_reached = all(
                    error <= joint_tolerance
                    for error in joint_errors.values()
                )
                height_reached = skip_height_check or self.update_reset_pose_height(
                    cmd_loop,
                    base_group_schema_key,
                    reset_cmd,
                )
                now = time.monotonic()
                if (
                    base_group_schema_key is not None
                    and self.robot_spec.reset_pose["base_height"] is not None
                    and (
                        now >= next_height_log_time
                        or now >= deadline
                        or (joints_reached and height_reached)
                    )
                ):
                    base_state = self.robot_hardware.read_states(
                        "base_group",
                        (base_group_schema_key,),
                    )[base_group_schema_key]
                    actual_height = (
                        base_state.body_height
                        if self.robot_spec.base_group_spec.height_control_mode == "position"
                        else base_state.lift_height
                    )
                    target_height = self.robot_spec.reset_pose["base_height"]
                    phase = "progress"
                    if skip_height_check:
                        phase = "skipped"
                    elif joints_reached and height_reached:
                        phase = "reached"
                    elif now >= deadline:
                        phase = "timeout"
                    log_height = (
                        logger_mp.info if height_reached else logger_mp.warning
                    )
                    log_height(
                        "Runtime reset_pose height: phase=%s target=%.4f "
                        "actual=%.4f signed_error=%+.4f",
                        phase,
                        target_height,
                        actual_height,
                        target_height - actual_height,
                    )
                    next_height_log_time = now + 1.0
                if joints_reached and height_reached:
                    pose_reached = True
                    logger_mp.info(
                        "Runtime reset_pose joints reached; holding until control starts.%s",
                        " Body height arrival is not checked." if skip_height_check else "",
                    )
                    break
                if now >= deadline:
                    unreached = []
                    for schema_key, error in joint_errors.items():
                        if error <= joint_tolerance:
                            continue
                        spec = self.robot_spec.joint_group_spec(schema_key)
                        joint_details = ", ".join(
                            f"{joint_name}(slot={motor_index}, "
                            f"target={target:.4f}, actual={actual:.4f}, "
                            f"abs_error={abs_error:.4f})"
                            for (
                                joint_name,
                                motor_index,
                                target,
                                actual,
                                abs_error,
                            ) in zip(
                                spec.joint_names[schema_key],
                                spec.motor_indices[schema_key],
                                targets[schema_key],
                                joint_qpos[schema_key],
                                joint_abs_errors[schema_key],
                            )
                            if abs_error > joint_tolerance
                        )
                        unreached.append(
                            f"{schema_key}(max_abs_error={error:.4f}): "
                            f"{joint_details}"
                        )
                    if not height_reached and base_group_schema_key is not None:
                        reset_base_height = self.robot_spec.reset_pose[
                            "base_height"
                        ]
                        base_state = self.robot_hardware.read_states(
                            "base_group",
                            (base_group_schema_key,),
                        )[base_group_schema_key]
                        actual_height = (
                            base_state.body_height
                            if self.robot_spec.base_group_spec.height_control_mode == "position"
                            else base_state.lift_height
                        )
                        height_error = abs(reset_base_height - actual_height)
                        unreached.append(
                            f"{base_group_schema_key}"
                            f"(height_error={height_error:.4f}, "
                            f"target={reset_base_height:.4f}, "
                            f"actual={actual_height:.4f}, "
                            f"signed_error="
                            f"{reset_base_height - actual_height:+.4f})"
                        )
                    raise TimeoutError(
                        f"reset_pose did not finish within {timeout:.1f}s; "
                        f"unreached: {', '.join(unreached)}"
                    )
        except Exception as exc:
            error_message = str(exc)
            logger_mp.exception("Runtime reset_pose failed.")
        finally:
            if not pose_reached:
                try:
                    cmd_loop.stop()
                except Exception as exc:
                    logger_mp.exception(
                        "Failed to stop reset_pose command loop."
                    )
                    if error_message is None:
                        error_message = str(exc)
            with self.control_lock:
                if self.reset_pose_loop is cmd_loop:
                    self.reset_pose_active = False
                    self.reset_pose_thread = None
                    if not pose_reached:
                        self.reset_pose_loop = None
                    if error_message is not None:
                        self.last_runtime_error = error_message

    def update_reset_pose_height(
        self,
        cmd_loop: RobotCmdLoop,
        base_group_schema_key: str | None,
        reset_cmd: dict[str, dict[str, ComponentCmdValue]],
    ) -> bool:
        schema_key = base_group_schema_key
        if schema_key is None:
            return True
        reset_pose = self.robot_spec.reset_pose
        if reset_pose is None:
            raise RuntimeError("reset_pose is not configured")
        target_height = cast(float | None, reset_pose["base_height"])
        if target_height is None:
            # The reset command already keeps the base and lift at zero speed.
            return True
        state = self.robot_hardware.read_states(
            "base_group",
            (schema_key,),
        )[schema_key]
        actual_height = (
            state.body_height
            if self.robot_spec.base_group_spec.height_control_mode == "position"
            else state.lift_height
        )
        height_error = target_height - actual_height
        height_reached = abs(height_error) <= RESET_POSE_HEIGHT_TOLERANCE
        if self.robot_spec.base_group_spec.height_control_mode == "position":
            return height_reached

        component_id = self.robot_hardware.cmd_component_id(
            "base_group",
            schema_key,
        )
        height_velocity = 0.0
        if not height_reached:
            height_velocity = float(
                np.clip(
                    height_error * RESET_POSE_HEIGHT_GAIN,
                    -RESET_POSE_HEIGHT_VELOCITY_LIMIT,
                    RESET_POSE_HEIGHT_VELOCITY_LIMIT,
                )
            )
        updated_cmd = {
            target_component_id: dict(component_cmds)
            for target_component_id, component_cmds in reset_cmd.items()
        }
        updated_cmd[component_id][schema_key] = BaseGroupCmd(
            base_linear_vel_x=0.0,
            base_linear_vel_y=0.0,
            base_angular_vel_z=0.0,
            lift_linear_vel_z=height_velocity,
        )
        cmd_loop.update_cmd(updated_cmd, timeout_enabled=False)
        return height_reached

    def resolve_reset_pose_schema_keys(
        self,
    ) -> tuple[tuple[str, ...], str | None]:
        mode = self.runtime_config.control_mode
        if mode is RuntimeControlMode.HYBRID_WBC:
            raise ValueError("hybrid_wbc does not support reset_pose.")
        body_joint_group_schema_keys = resolve_motion_schema_keys(
            mode, self.robot_spec.joint_group_specs[0].schema_keys,
        ).get("joint_group", ())
        if not body_joint_group_schema_keys:
            raise RuntimeError(
                "reset_pose does not support motion control mode "
                f"{mode.value!r}"
            )
        cmd_joint_keys = self.robot_hardware.cmd_routes.get(
            "joint_group",
            {},
        )
        reset_pose = self.robot_spec.reset_pose
        reset_joint_positions = (
            {} if reset_pose is None else reset_pose["joint_groups"]
        )
        joint_group_schema_keys = (
            *body_joint_group_schema_keys,
            *(
                key for key in ("waist", "head")
                if (
                    mode is RuntimeControlMode.DUAL_ARM
                    and self.robot_spec.joint_group_specs[0].component_id in {
                        "body.r1_a5", "body.r1_a7", "body.r1_a7_d",
                    }
                    and key not in body_joint_group_schema_keys
                    and key in reset_joint_positions
                    and key in cmd_joint_keys
                )
            ),
            *(
                ("head",)
                if (
                    mode is RuntimeControlMode.MOBILE_WBC
                    and "head" in reset_joint_positions
                    and "head" in cmd_joint_keys
                )
                else ()
            ),
            *(
                schema_key
                for schema_key in self.robot_spec.schema_keys("end_effector")
                if schema_key in cmd_joint_keys
            ),
        )
        unwritable_body_keys = tuple(
            schema_key
            for schema_key in body_joint_group_schema_keys
            if schema_key not in cmd_joint_keys
        )
        if unwritable_body_keys:
            raise RuntimeError(
                "reset_pose body joint groups are not writable: "
                f"{unwritable_body_keys!r}"
            )
        base_group_schema_key = None
        if mode in {
            RuntimeControlMode.MOBILE_WBC,
            RuntimeControlMode.DECOUPLED_WBC,
        }:
            writable_base_keys = [
                schema_key
                for schema_key in self.robot_spec.schema_keys("base_group")
                if schema_key
                in self.robot_hardware.cmd_routes.get("base_group", {})
            ]
            if len(writable_base_keys) != 1:
                raise RuntimeError(
                    "reset_pose requires exactly one writable base_group"
                )
            base_group_schema_key = writable_base_keys[0]
        return joint_group_schema_keys, base_group_schema_key

    def build_reset_pose_cmd(
        self,
        joint_group_schema_keys: tuple[str, ...],
        base_group_schema_key: str | None,
        qpos_reset: dict[str, tuple[float, ...]],
        *,
        joint_qtau: dict[str, np.ndarray] | None = None,
    ) -> tuple[
        dict[str, dict[str, ComponentCmdValue]],
        dict[str, np.ndarray],
    ]:
        targets = {}
        component_cmds: dict[str, dict[str, ComponentCmdValue]] = {}
        for schema_key in joint_group_schema_keys:
            spec = self.robot_spec.joint_group_spec(schema_key)
            if schema_key in spec.torque_schema_keys:
                component_id = self.robot_hardware.cmd_component_id("joint_group", schema_key)
                component_cmds.setdefault(component_id, {})[schema_key] = JointGroupCmd(
                    num_joints=len(spec.motor_indices[schema_key]),
                    qtau=np.zeros(len(spec.motor_indices[schema_key]), dtype=np.float32),
                )
                continue
            try:
                target_config = qpos_reset[schema_key]
            except KeyError as exc:
                raise KeyError(
                    f"reset_pose has no target for joint group {schema_key!r}"
                ) from exc
            target_qpos = np.asarray(target_config, dtype=np.float32)
            targets[schema_key] = target_qpos
            component_id = self.robot_hardware.cmd_component_id(
                "joint_group",
                schema_key,
            )
            component_cmds.setdefault(component_id, {})[
                schema_key
            ] = JointGroupCmd(
                num_joints=target_qpos.size,
                qpos=target_qpos,
                qtau=(
                    joint_qtau[schema_key]
                    if joint_qtau is not None and schema_key in joint_qtau
                    else np.zeros_like(target_qpos)
                ),
            )

        if base_group_schema_key is not None:
            schema_key = base_group_schema_key
            reset_pose = self.robot_spec.reset_pose
            if reset_pose is None:
                raise RuntimeError("reset_pose is not configured")
            target_height = cast(float | None, reset_pose["base_height"])
            component_id = self.robot_hardware.cmd_component_id(
                "base_group",
                schema_key,
            )
            if self.robot_spec.base_group_spec.height_control_mode == "position":
                if target_height is None:
                    raise RuntimeError("reset_pose has no base height target")
                base_cmd = BaseGroupCmd(
                    body_linear_vel_x=0.0, body_linear_vel_y=0.0, body_angular_vel_z=0.0,
                    body_height=target_height,
                    waist_roll_qpos=0.0, waist_pitch_qpos=0.0, waist_yaw_qpos=0.0,
                )
            else:
                base_cmd = BaseGroupCmd(
                    base_linear_vel_x=0.0, base_linear_vel_y=0.0, base_angular_vel_z=0.0,
                    lift_linear_vel_z=0.0,
                )
            component_cmds.setdefault(component_id, {})[schema_key] = base_cmd
        return component_cmds, targets

    def stop_expired_control(self) -> None:
        with self.control_lock:
            cmd_session_id = self.cmd_session_id or self.prepared_cmd_session_id
            expired = (
                cmd_session_id is not None
                and time.monotonic() > self.control_deadline
            )
        if expired:
            logger_mp.warning("Control timed out; stopping robot control.")
            self.stop_control(
                cmd_session_id=cmd_session_id,
                force=True,
            )

    def component_cmds_from_action_data(
        self,
        action: ActionData,
    ) -> dict[str, dict[str, ComponentCmdValue]]:
        hybrid_control = (
            self.runtime_config.control_mode is RuntimeControlMode.HYBRID_WBC
        )
        if hybrid_control:
            if type(action.hybrid_mode) is not int or action.hybrid_mode not in (0, 1):
                raise ValueError("hybrid_wbc requires hybrid_mode=0 or 1.")
        elif action.hybrid_mode is not None:
            raise ValueError("hybrid_mode is only valid for hybrid_wbc control.")
        joint_group_schema_keys = self.control_joint_group_schema_keys
        if joint_group_schema_keys is None:
            raise RuntimeError("Control has not been started.")

        allowed_joint_groups = set(joint_group_schema_keys)
        joint_groups = action.joint_groups or ()
        groups_by_schema_key = {
            group.schema_key: group for group in joint_groups
        }
        if len(groups_by_schema_key) != len(joint_groups):
            raise ValueError("ActionData contains a duplicate joint group.")
        received_joint_groups = set(groups_by_schema_key)
        unexpected_joint_groups = received_joint_groups - allowed_joint_groups
        if unexpected_joint_groups:
            raise ValueError(
                "ActionData contains unselected joint groups: "
                f"{tuple(sorted(unexpected_joint_groups))!r}."
            )
        missing_joint_groups = allowed_joint_groups - received_joint_groups
        if missing_joint_groups:
            raise ValueError(
                "ActionData omits controlled joint groups "
                f"{tuple(sorted(missing_joint_groups))!r}."
            )

        component_cmds: dict[str, dict[str, ComponentCmdValue]] = {}
        for schema_key, group in groups_by_schema_key.items():
            spec = self.robot_spec.joint_group_spec(schema_key)
            if schema_key in spec.torque_schema_keys:
                if group.qpos is not None or group.qtau is None:
                    raise ValueError(f"ActionData joint group {schema_key!r} requires qtau only.")
            elif group.qpos is None:
                raise ValueError(
                    f"ActionData joint group {schema_key!r} requires qpos."
                )
            qpos = None if group.qpos is None else np.asarray(group.qpos, dtype=np.float32)
            expected_dof = len(
                self.robot_spec.joint_group_spec(schema_key).motor_indices[
                    schema_key
                ]
            )
            qtau = (
                None
                if group.qtau is None
                else np.asarray(group.qtau, dtype=np.float32)
            )
            cmd = JointGroupCmd(
                num_joints=expected_dof,
                qpos=qpos,
                qtau=qtau,
            )
            if schema_key in spec.torque_schema_keys:
                limits = np.asarray(spec.qtau_limits[schema_key])
                if np.any(qtau < limits[:, 0] - 1e-7) or np.any(qtau > limits[:, 1] + 1e-7):
                    raise ValueError(f"ActionData joint group {schema_key!r} exceeds qtau_limits.")
            component_id = self.robot_hardware.cmd_component_id(
                "joint_group",
                schema_key,
            )
            component_cmds.setdefault(component_id, {})[schema_key] = cmd

        base_groups = action.base_group or ()
        if len(base_groups) > 1:
            raise ValueError("ActionData accepts at most one base_group action.")
        expected_schema_key = self.control_base_group_schema_key
        if expected_schema_key is not None and len(base_groups) != 1:
            raise ValueError(
                "ActionData must contain the controlled base_group "
                f"{expected_schema_key!r}."
            )
        if base_groups:
            base = base_groups[0]
            if expected_schema_key is None:
                raise ValueError(
                    "ActionData contains an unselected base_group."
                )
            if base.schema_key != expected_schema_key:
                raise ValueError(
                    f"ActionData base group {base.schema_key!r} does not match "
                    f"the controlled base group {expected_schema_key!r}."
                )
            values = self.robot_spec.base_group_spec.command_values(
                base, planar_only=hybrid_control,
            )
            if hybrid_control and action.hybrid_mode == 1 and any(values.values()):
                raise ValueError("hybrid_mode=1 requires zero base velocity.")
            cmd = BaseGroupCmd(**values)
            component_id = self.robot_hardware.cmd_component_id(
                "base_group",
                base.schema_key,
            )
            component_cmds.setdefault(component_id, {})[base.schema_key] = cmd

        pose_actions = action.pose_82d or ()
        if len(pose_actions) > 1:
            raise ValueError("ActionData accepts at most one pose_82d action.")
        expected_schema_key = self.control_pose_82d_schema_key
        # Validate the submode's command shape at the ActionData boundary.
        # Hybrid drivers encode mode 1 exactly when pose_82d is present.
        if hybrid_control and action.hybrid_mode == 0:
            expected_schema_key = None
        if expected_schema_key is not None and len(pose_actions) != 1:
            raise ValueError(
                "ActionData must contain the controlled pose_82d "
                f"{expected_schema_key!r}."
            )
        if pose_actions:
            pose = pose_actions[0]
            if expected_schema_key is None:
                raise ValueError(
                    "ActionData contains an unselected pose_82d."
                )
            if pose.schema_key != expected_schema_key:
                raise ValueError(
                    f"ActionData pose {pose.schema_key!r} does not match "
                    f"the controlled pose {expected_schema_key!r}."
                )
            if (
                pose.smpl_joints is None
                or pose.smpl_root_quat_wxyz is None
                or pose.wrist_qpos is None
            ):
                raise ValueError(
                    f"ActionData pose {pose.schema_key!r} requires all pose "
                    "fields."
                )
            smpl_joints = np.asarray(pose.smpl_joints, dtype=np.float32)
            smpl_root_quat_wxyz = np.asarray(
                pose.smpl_root_quat_wxyz,
                dtype=np.float32,
            )
            wrist_qpos = np.asarray(pose.wrist_qpos, dtype=np.float32)
            cmd = Pose82dCmd(
                smpl_joints=smpl_joints,
                smpl_root_quat_wxyz=smpl_root_quat_wxyz,
                wrist_qpos=wrist_qpos,
            )
            component_id = self.robot_hardware.cmd_component_id(
                "pose_82d",
                pose.schema_key,
            )
            component_cmds.setdefault(component_id, {})[pose.schema_key] = cmd

        return component_cmds

    def accept_cmd(self, runtime_cmd: RuntimeCmd) -> None:
        with self.control_lock:
            cmd_loop = self.cmd_loop
            if (
                self.control_exit_pending
                or runtime_cmd.cmd_session_id != self.cmd_session_id
                or cmd_loop is None
                or self.control_joint_group_schema_keys is None
            ):
                return
            if runtime_cmd.cmd_sequence <= self.last_cmd_sequence:
                return
            cmd_age = time.monotonic() - runtime_cmd.timestamp
            if cmd_age < 0.0 or cmd_age > self.cmd_timeout:
                return
            component_cmds = self.component_cmds_from_action_data(
                runtime_cmd.action
            )
            cmd_loop.update_cmd(component_cmds)
            self.last_cmd_sequence = runtime_cmd.cmd_sequence
            self.last_accepted_cmd = runtime_cmd.action
            self.control_deadline = time.monotonic() + self.control_timeout
            self.last_runtime_error = None

    def build_cmd_loop(
        self,
        *,
        joint_group_schema_keys: tuple[str, ...],
        base_group_schema_key: str | None,
        pose_82d_schema_key: str | None,
        velocity_limit: float | None = None,
        cmd_joint_group_order: tuple[str, ...] | None = None,
    ) -> RobotCmdLoop:
        robot_hardware = self.robot_hardware
        runtime_config = self.runtime_config
        body_channel = robot_hardware.channel("body")
        enter_control = getattr(body_channel, "enter_control", None)
        exit_control = getattr(body_channel, "exit_control", None)
        common_cmd_loop_kwargs = {
            "write_component_cmds": robot_hardware.write_component_cmds,
            "qpos_limits": {
                (spec.component_id, schema_key): limits
                for spec in self.robot_spec.joint_group_specs
                for schema_key, limits in spec.qpos_limits.items()
                if schema_key not in spec.torque_schema_keys
            },
            "frequency": runtime_config.cmd_frequency,
            "enter_control": enter_control,
            "exit_control": exit_control,
            "cmd_timeout": runtime_config.cmd_timeout,
            "thermal_safety": self.build_thermal_safety(base_group_schema_key),
        }

        if (
            pose_82d_schema_key is not None
            and runtime_config.control_mode is not RuntimeControlMode.HYBRID_WBC
        ):
            if not callable(enter_control) or not callable(exit_control):
                raise RuntimeError(
                    "humanoid WBC body channel must provide control lifecycle"
                )
            cmd_loop = Pose82dCmdLoop(**common_cmd_loop_kwargs)
            cmd_loop.display_delay = getattr(self, "display_delay", False)
            return cmd_loop

        fixed_joint_group_schema_keys = ()
        if runtime_config.control_mode in {
            RuntimeControlMode.DUAL_ARM, RuntimeControlMode.UPPER_BODY,
        }:
            fixed_joint_group_schema_keys = tuple(
                key
                for key in robot_hardware.cmd_routes.get("joint_group", {})
                if key in {"waist", "left_leg", "right_leg", "head"}
                and key not in joint_group_schema_keys
            )
        joint_group_schema_keys = (
            *joint_group_schema_keys,
            *fixed_joint_group_schema_keys,
        )
        joint_group_components = {
            schema_key: robot_hardware.cmd_component_id(
                "joint_group",
                schema_key,
            )
            for schema_key in joint_group_schema_keys
        }
        hold_component_cmds: dict[str, dict[str, ComponentCmdValue]] = {}
        torque_rate_limits = {}
        for schema_key in joint_group_schema_keys:
            spec = self.robot_spec.joint_group_spec(schema_key)
            if schema_key in spec.torque_schema_keys:
                component_id = joint_group_components[schema_key]
                hold_component_cmds.setdefault(component_id, {})[schema_key] = JointGroupCmd(
                    num_joints=len(spec.motor_indices[schema_key]),
                    qtau=np.zeros(len(spec.motor_indices[schema_key]), dtype=np.float32),
                )
                torque_rate_limits[schema_key] = spec.qtau_rate_limit[schema_key]
                continue
            qpos = self.read_joint_group_qpos(schema_key)
            component_id = joint_group_components[schema_key]
            if (
                schema_key in fixed_joint_group_schema_keys
                and self.reset_pose_loop is not None
            ):
                if (
                    schema_key
                    in self.reset_pose_loop.fixed_joint_group_schema_keys
                ):
                    qpos = self.reset_pose_loop.hold_component_cmds[
                        component_id
                    ][schema_key].qpos.copy()
                elif self.reset_pose_loop.last_component_cmds is not None:
                    # R1 explicitly resets its waist/head. Preserve that target
                    # rather than anchoring again to residual tracking error.
                    qpos = self.reset_pose_loop.last_component_cmds[
                        component_id
                    ][schema_key].qpos.copy()
            hold_component_cmds.setdefault(component_id, {})[
                schema_key
            ] = JointGroupCmd(
                num_joints=qpos.size,
                qpos=qpos,
                qtau=np.zeros_like(qpos),
            )

        compute_gravity_qtau = None
        if (
            runtime_config.control_mode in {
                RuntimeControlMode.DUAL_ARM, RuntimeControlMode.UPPER_BODY,
                RuntimeControlMode.MOBILE_WBC,
            }
            and joint_group_components.get("waist") == "body.g1_d_flagship_unlocked"
        ):
            from robot_runtime.multibody.dynamics.pinocchio_gravity import (
                PinocchioGravityCompensator,
            )
            from robot_runtime.multibody.pinocchio_model import load_reduced_model

            kinematics = self.robot_spec.dual_arm_kinematics
            # This URDF names its Y-axis pitch joint Yaw_Joint; torso_Joint
            # is the Z-axis yaw joint. Keep this binding local to this body.
            waist_model_names = {
                "waist_pitch_joint": "Yaw_Joint",
                "waist_yaw_joint": "torso_Joint",
            }
            gravity_joint_names = {
                "waist": tuple(
                    waist_model_names[name]
                    for name in self.robot_spec.joint_group_spec("waist").joint_names[
                        "waist"
                    ]
                ),
                "left_arm": kinematics.left_arm_joint_names,
                "right_arm": kinematics.right_arm_joint_names,
            }
            model = load_reduced_model(
                kinematics.urdf_path,
                (
                    "Yaw_Joint", "torso_Joint",
                    *gravity_joint_names["left_arm"], *gravity_joint_names["right_arm"],
                ),
                cache_namespace="gravity",
            )
            compensator = PinocchioGravityCompensator(model)
            # All selected joints are single-DoF revolute joints (nq == nv).
            gravity_indices = {
                key: np.asarray([
                    model.joints[model.getJointId(name)].idx_q for name in names
                ])
                for key, names in gravity_joint_names.items()
            }

            def compute_gravity_qtau() -> dict[str, np.ndarray]:
                states = robot_hardware.read_states(
                    "joint_group", tuple(gravity_indices),
                )
                qpos = np.empty(model.nq, dtype=np.float64)
                for key, indices in gravity_indices.items():
                    qpos[indices] = states[key].qpos
                qtau = compensator.compute(qpos)
                return {
                    key: qtau[indices].copy() for key, indices in gravity_indices.items()
                }

        base_group_component = None
        if base_group_schema_key is not None:
            schema_key = base_group_schema_key
            base_group_component = robot_hardware.cmd_component_id(
                "base_group",
                schema_key,
            )
            if runtime_config.control_mode is RuntimeControlMode.HYBRID_WBC:
                base_cmd = BaseGroupCmd(
                    body_linear_vel_x=0.0, body_linear_vel_y=0.0, body_angular_vel_z=0.0,
                )
            elif runtime_config.control_mode is RuntimeControlMode.DECOUPLED_WBC:
                previous = (
                    None if self.reset_pose_loop is None
                    else self.reset_pose_loop.last_component_cmds
                )
                if previous is not None:
                    base_cmd = JointCmdLoop.zero_base_group_cmd(
                        previous[base_group_component][schema_key]
                    )
                else:
                    state = robot_hardware.read_states("base_group", (schema_key,))[schema_key]
                    waist_spec = self.robot_spec.joint_group_spec("waist")
                    waist_qpos = dict(zip(
                        waist_spec.joint_names["waist"], self.read_joint_group_qpos("waist"),
                        strict=True,
                    ))
                    base_cmd = BaseGroupCmd(
                        body_linear_vel_x=0.0, body_linear_vel_y=0.0, body_angular_vel_z=0.0,
                        body_height=float(np.clip(state.body_height, *self.robot_spec.base_group_spec.height_limits)),
                        waist_roll_qpos=waist_qpos["waist_roll_joint"],
                        waist_pitch_qpos=waist_qpos["waist_pitch_joint"],
                        waist_yaw_qpos=waist_qpos["waist_yaw_joint"],
                    )
            else:
                base_cmd = BaseGroupCmd(
                    base_linear_vel_x=0.0, base_linear_vel_y=0.0, base_angular_vel_z=0.0,
                    lift_linear_vel_z=0.0,
                )
            hold_component_cmds.setdefault(base_group_component, {})[schema_key] = base_cmd
        if runtime_config.control_mode is RuntimeControlMode.HYBRID_WBC:
            if not callable(enter_control) or not callable(exit_control):
                raise RuntimeError("hybrid WBC body channel requires control lifecycle")
        if velocity_limit is None:
            velocity_limit = runtime_config.cmd_joint_velocity_limit
        if velocity_limit is None:
            raise RuntimeError("joint control has no runtime velocity limit.")
        if cmd_joint_group_order is None:
            cmd_joint_group_order = tuple(
                schema_key
                for schema_key in joint_group_schema_keys
                if schema_key
                not in self.robot_spec.schema_keys("end_effector")
            )
        else:
            cmd_joint_group_order = (
                *cmd_joint_group_order,
                *(
                    key
                    for key in fixed_joint_group_schema_keys
                    if key not in cmd_joint_group_order
                ),
            )
        cmd_joint_group_order = tuple(
            key for key in cmd_joint_group_order if key not in torque_rate_limits
        )
        cmd_loop = JointCmdLoop(
            torque_rate_limits=torque_rate_limits,
            joint_group_components=joint_group_components,
            hold_component_cmds=hold_component_cmds,
            fixed_joint_group_schema_keys=fixed_joint_group_schema_keys,
            compute_gravity_qtau=compute_gravity_qtau,
            cmd_joint_group_order=cmd_joint_group_order,
            read_joint_group_qpos=self.read_joint_group_qpos,
            velocity_limit=velocity_limit,
            body_height_rate_limit=runtime_config.cmd_body_height_rate_limit,
            base_group_component=base_group_component,
            base_group_schema_key=base_group_schema_key,
            zero_base_group_on_stop=(
                runtime_config.control_mode is RuntimeControlMode.MOBILE_WBC
            ),
            **common_cmd_loop_kwargs,
        )
        cmd_loop.display_delay = getattr(self, "display_delay", False)
        return cmd_loop

    def read_joint_group_qpos(self, schema_key: str) -> np.ndarray:
        state = self.robot_hardware.read_states(
            "joint_group",
            (schema_key,),
        )[schema_key]
        return np.asarray(state.qpos, dtype=np.float32).copy()

    def build_thermal_safety(
        self,
        base_group_schema_key: str | None,
    ) -> ThermalSafety | None:
        state_joint_keys = self.robot_hardware.state_routes.get("joint_group", {})
        thresholds_by_schema_key = {}
        for schema_key in state_joint_keys:
            thresholds = self.robot_spec.joint_group_spec(
                schema_key
            ).temperature_thresholds
            if thresholds:
                thresholds_by_schema_key[schema_key] = thresholds
        if not thresholds_by_schema_key:
            return None

        joint_schema_keys = tuple(state_joint_keys)
        return ThermalSafety(
            thresholds_by_schema_key=thresholds_by_schema_key,
            protection_enabled=self.runtime_config.thermal_protection_enabled,
            read_joint_states=lambda: self.robot_hardware.read_states(
                "joint_group",
                joint_schema_keys,
            ),
            waist_joint_names=(
                self.robot_spec.joint_group_spec("waist").joint_names["waist"]
                if "waist" in joint_schema_keys else ()
            ),
        )

    def set_runtime_error(self, message: str) -> None:
        with self.control_lock:
            self.last_runtime_error = message

    def _reset_control_state(self) -> None:
        self.cmd_loop: RobotCmdLoop | None = None
        self.cmd_session_id: str | None = None
        self.prepared_cmd_session_id: str | None = None
        self.control_joint_group_schema_keys: tuple[str, ...] | None = None
        self.control_base_group_schema_key: str | None = None
        self.control_pose_82d_schema_key: str | None = None
        self.control_deadline = 0.0
        self.control_exit_pending = False
        self.last_cmd_sequence = -1
        self.last_accepted_cmd = None
