"""Resolve and validate one canonical experiment configuration."""

import hashlib
import json

from robot_runtime.configs.sections.policy.replay import ReplayConfig
from robot_runtime.configs.sections.policy.teleop import TeleopConfig
from robot_runtime.configs.sections.recording import RecordingConfig
from robot_runtime.configs.sections.robot.composition import RobotSpec
from robot_runtime.configs.sections.robot.drivers.resolver import resolve_robot_drivers
from robot_runtime.configs.sections.runtime import (
    RuntimeControlMode,
    RuntimeConfig,
    resolve_motion_schema_keys,
)


SUPPORTED_MOTION_MODES = {
    "body.g1_edu": frozenset(
        {
            RuntimeControlMode.DUAL_ARM,
            RuntimeControlMode.UPPER_BODY,
            RuntimeControlMode.DECOUPLED_WBC,
            RuntimeControlMode.HUMANOID_WBC,
            RuntimeControlMode.HYBRID_WBC,
        }
    ),
    "body.g1_d_flagship": frozenset(
        {
            RuntimeControlMode.DUAL_ARM,
            RuntimeControlMode.UPPER_BODY,
            RuntimeControlMode.MOBILE_WBC,
        }
    ),
    "body.g1_d_flagship_unlocked": frozenset(
        {
            RuntimeControlMode.DUAL_ARM,
            RuntimeControlMode.UPPER_BODY,
            RuntimeControlMode.MOBILE_WBC,
        }
    ),
    "body.r1_edu": frozenset(
        {RuntimeControlMode.DUAL_ARM, RuntimeControlMode.UPPER_BODY}
    ),
    "body.r1_a7_d": frozenset(
        {
            RuntimeControlMode.DUAL_ARM,
            RuntimeControlMode.UPPER_BODY,
            RuntimeControlMode.MOBILE_WBC,
        }
    ),
    "body.r1_a7": frozenset(
        {RuntimeControlMode.DUAL_ARM, RuntimeControlMode.UPPER_BODY}
    ),
    "body.r1_a5": frozenset(
        {RuntimeControlMode.DUAL_ARM, RuntimeControlMode.UPPER_BODY}
    ),
    "body.h2_edu": frozenset(
        {
            RuntimeControlMode.DUAL_ARM,
            RuntimeControlMode.UPPER_BODY,
            RuntimeControlMode.HUMANOID_WBC,
        }
    ),
}

SIMULATION_MOTION_MODES = {
    "body.g1_edu": frozenset(
        {RuntimeControlMode.DUAL_ARM, RuntimeControlMode.UPPER_BODY}
    ),
    "body.g1_d_flagship": frozenset(
        {
            RuntimeControlMode.DUAL_ARM,
            RuntimeControlMode.UPPER_BODY,
            RuntimeControlMode.MOBILE_WBC,
        }
    ),
    "body.h2_edu": frozenset(
        {RuntimeControlMode.DUAL_ARM, RuntimeControlMode.UPPER_BODY}
    ),
}

RETARGETER_CONTROL_MODES = {
    "xr_body": frozenset(
        {
            RuntimeControlMode.DUAL_ARM,
            RuntimeControlMode.UPPER_BODY,
            RuntimeControlMode.MOBILE_WBC,
            RuntimeControlMode.HYBRID_WBC,
            RuntimeControlMode.DECOUPLED_WBC,
        }
    ),
    "xr_base_lift": frozenset({RuntimeControlMode.MOBILE_WBC}),
    "xr_pseudo_base_lift": frozenset({RuntimeControlMode.DECOUPLED_WBC}),
    "xr_pose_82d": frozenset({RuntimeControlMode.HUMANOID_WBC}),
    "xr_hybrid_wbc": frozenset({RuntimeControlMode.HYBRID_WBC}),
    "xr_gripper": frozenset(RuntimeControlMode),
    "xr_dexterous_hand": frozenset(RuntimeControlMode),
}

REQUIRED_RETARGETER_TYPES = {
    RuntimeControlMode.HYBRID_WBC: frozenset({"xr_body", "xr_hybrid_wbc"}),
    RuntimeControlMode.DUAL_ARM: frozenset({"xr_body"}),
    RuntimeControlMode.UPPER_BODY: frozenset({"xr_body"}),
    RuntimeControlMode.MOBILE_WBC: frozenset(
        {"xr_body", "xr_base_lift"}
    ),
    RuntimeControlMode.DECOUPLED_WBC: frozenset(
        {"xr_body", "xr_pseudo_base_lift"}
    ),
    RuntimeControlMode.HUMANOID_WBC: frozenset({"xr_pose_82d"}),
}


class ExperimentConfig:
    """Build runtime configuration objects from a resolved experiment."""

    def __init__(self, experiment_cfg: dict[str, object]):
        self.source_path = experiment_cfg["source_path"]
        self.environment = experiment_cfg["environment"]

        robot_cfg = experiment_cfg["robot"]
        self.robot_config = robot_cfg["spec"]
        self.robot_spec = RobotSpec(self.robot_config)

        self.runtime_config = RuntimeConfig(experiment_cfg["runtime"])
        control_mode = self.runtime_config.control_mode
        self._validate_control_mode()

        (
            self.robot_driver_configs,
            self.robot_channel_configs,
        ) = resolve_robot_drivers(
            robot_cfg["drivers"],
            robot_cfg["connections"],
            robot_spec=self.robot_spec,
            control_mode=control_mode,
            label=f"{self.source_path}.robot.drivers",
        )
        if (
            self.environment == "real"
            and control_mode in {
                RuntimeControlMode.DUAL_ARM, RuntimeControlMode.UPPER_BODY,
            }
            and not {"left_leg", "right_leg"}.isdisjoint(
                self.robot_spec.joint_group_specs[0].schema_keys
            )
            and self.robot_channel_configs["body"]["driver"]["cmd_topic"]
            != "rt/arm_sdk"
        ):
            raise ValueError(
                "Biped real dual_arm/upper_body requires rt/arm_sdk "
                "to keep the motion service in control of the legs."
            )

        policy_cfg = experiment_cfg["policy"]
        policy_id = policy_cfg["policy_id"]
        if policy_id == "teleop":
            self.policy_config = TeleopConfig(policy_cfg)
        elif policy_id == "replay":
            self.policy_config = ReplayConfig(policy_cfg)
        else:
            raise ValueError(
                f"Unsupported policy.policy_id={policy_id!r}."
            )
        self._validate_policy_compatibility()

        recording_config = experiment_cfg.get("recording")
        self.recording_config = (
            None
            if recording_config is None
            else RecordingConfig(recording_config)
        )
        self._validate_frequency_relationships()

        fingerprint_payload = {
            "environment": self.environment,
            "robot": self.robot_config,
            "drivers": self.robot_driver_configs,
            "channels": self.robot_channel_configs,
            "runtime": self.runtime_config.to_dict(),
        }
        fingerprint_json = json.dumps(
            fingerprint_payload,
            sort_keys=True,
            separators=(",", ":"),
        )
        self.runtime_fingerprint = hashlib.sha256(
            fingerprint_json.encode("utf-8")
        ).hexdigest()
        self.runtime_endpoints = {
            endpoint_name: (
                "ipc://@robot_runtime_"
                f"{self.runtime_fingerprint}_{endpoint_name}.ipc"
            )
            for endpoint_name in (
                "runtime_state",
                "runtime_cmd",
                "runtime_request",
            )
        }

    def _validate_control_mode(self) -> None:
        body_component_id = self.robot_spec.joint_group_specs[0].component_id
        control_mode = self.runtime_config.control_mode
        try:
            supported_modes = SUPPORTED_MOTION_MODES[body_component_id]
        except KeyError as exc:
            known = ", ".join(sorted(SUPPORTED_MOTION_MODES))
            raise ValueError(
                f"Unsupported body component_id={body_component_id!r}. "
                f"Known bodies: {known}."
            ) from exc

        if control_mode not in supported_modes:
            supported = ", ".join(
                sorted(mode.value for mode in supported_modes)
            )
            raise ValueError(
                f"{body_component_id} does not support "
                "runtime.control_mode="
                f"{control_mode.value!r}. "
                f"Supported: {supported}."
            )

        simulation_modes = SIMULATION_MOTION_MODES.get(
            body_component_id,
            frozenset(),
        )
        if (
            self.environment == "sim"
            and control_mode not in simulation_modes
        ):
            raise NotImplementedError(
                f"{body_component_id} "
                "runtime.control_mode="
                f"{control_mode.value!r} does not support "
                "sim yet."
            )

    def _validate_policy_compatibility(self) -> None:
        retargeter_configs = getattr(
            self.policy_config,
            "retargeter_configs",
            None,
        )
        if retargeter_configs is None:
            if self.runtime_config.control_mode is RuntimeControlMode.HYBRID_WBC:
                raise ValueError("hybrid_wbc currently requires a teleop policy.")
            return

        control_mode = self.runtime_config.control_mode
        retargeter_types = {
            retargeter_config["type"]
            for retargeter_config in retargeter_configs.values()
        }
        if (
            control_mode is RuntimeControlMode.HYBRID_WBC
            and self.policy_config.reset_pose_on_start
        ):
            raise ValueError("hybrid_wbc does not support reset_pose_on_start.")
        required_retargeter_types = REQUIRED_RETARGETER_TYPES[control_mode]
        body_spec = self.robot_spec.joint_group_specs[0]
        missing_retargeter_types = (
            required_retargeter_types - retargeter_types
        )
        if missing_retargeter_types:
            raise ValueError(
                f"runtime.control_mode={control_mode.value!r} requires "
                "policy retargeter types "
                f"{tuple(sorted(missing_retargeter_types))!r}."
            )
        if (
            "xr_body" in retargeter_types
            and self.robot_spec.dual_arm_kinematics is None
        ):
            raise ValueError(
                "Dual-arm policy retargeter requires robot "
                "kinematics.dual_arm."
            )
        action_joint_keys = resolve_motion_schema_keys(
            control_mode, body_spec.schema_keys,
        ).get("joint_group", ())
        if (
            "xr_body" in retargeter_types
            and "head" in body_spec.schema_keys
            and (
                "head" in action_joint_keys
                or control_mode is RuntimeControlMode.MOBILE_WBC
            )
        ):
            head_joint_names = body_spec.joint_names.get("head", ())
            if set(head_joint_names) != {
                "head_pitch_joint",
                "head_yaw_joint",
            }:
                raise ValueError(
                    "xr_body requires head_pitch_joint and "
                    "head_yaw_joint."
                )
            if len(body_spec.qpos_limits.get("head", ())) != len(
                head_joint_names
            ):
                raise ValueError(
                    "xr_body requires two body head qpos_limits."
                )
        for retargeter_type in retargeter_types:
            supported_modes = RETARGETER_CONTROL_MODES.get(retargeter_type)
            if supported_modes is None or control_mode in supported_modes:
                continue
            raise ValueError(
                f"Policy retargeter {retargeter_type!r} is not valid for "
                f"runtime.control_mode={control_mode.value!r}."
            )
        input_mode = self.policy_config.hmi_config.input_mode
        for retargeter_type, required_input_mode in (
            ("xr_gripper", "controller"),
            ("xr_dexterous_hand", "hand"),
        ):
            if (
                retargeter_type in retargeter_types
                and input_mode != required_input_mode
            ):
                raise ValueError(
                    f"{retargeter_type} requires policy.hmi.input_mode="
                    f"{required_input_mode!r}."
                )
        base_group_spec = self.robot_spec.base_group_spec
        for retargeter_type, expected_height_mode in (
            ("xr_base_lift", "velocity"),
            ("xr_pseudo_base_lift", "position"),
        ):
            if retargeter_type not in retargeter_types:
                continue
            if base_group_spec is None:
                raise ValueError(
                    f"{retargeter_type} requires a base_group component."
                )
            if base_group_spec.height_control_mode != expected_height_mode:
                raise ValueError(
                    f"{retargeter_type} requires base_group.height_control_mode="
                    f"{expected_height_mode!r}."
                )

        if not retargeter_types.isdisjoint(
            {"xr_base_lift", "xr_pseudo_base_lift"}
        ):
            xr_config = self.policy_config.hmi_config
            if xr_config.input_mode == "hand":
                raise ValueError(
                    "XR base-group retargeters require controller input mode."
                )

        if control_mode is RuntimeControlMode.DECOUPLED_WBC:
            if set(body_spec.joint_names.get("waist", ())) != {
                "waist_roll_joint", "waist_pitch_joint", "waist_yaw_joint",
            }:
                raise ValueError(
                    "xr_pseudo_base_lift requires waist roll, pitch, and yaw "
                    "joints."
                )

        if "waist" in action_joint_keys:
            waist_joint_names = body_spec.joint_names.get("waist", ())
            unsupported_waist_joints = set(waist_joint_names) - {
                "waist_roll_joint",
                "waist_pitch_joint",
                "waist_yaw_joint",
            }
            if unsupported_waist_joints:
                raise ValueError(
                    "xr_body does not support waist joints "
                    f"{tuple(sorted(unsupported_waist_joints))!r}."
                )
            if len(body_spec.qpos_limits.get("waist", ())) != len(
                waist_joint_names
            ):
                raise ValueError(
                    "XR waist retargeting requires qpos_limits for every "
                    "configured waist joint."
                )

    def _validate_frequency_relationships(self) -> None:
        state_frequency = self.runtime_config.state_frequency
        policy_update_frequency = self.policy_config.update_frequency
        if policy_update_frequency > state_frequency:
            raise ValueError(
                "policy.update_frequency must not exceed "
                "runtime.state_frequency."
            )
        if self.runtime_config.cmd_frequency < policy_update_frequency:
            raise ValueError(
                "runtime.cmd_frequency must not be lower than "
                "policy.update_frequency."
            )
        recording_config = self.recording_config
        if (
            recording_config is not None
            and recording_config.fps > state_frequency
        ):
            raise ValueError(
                "recording.fps must not exceed "
                "runtime.state_frequency."
            )
