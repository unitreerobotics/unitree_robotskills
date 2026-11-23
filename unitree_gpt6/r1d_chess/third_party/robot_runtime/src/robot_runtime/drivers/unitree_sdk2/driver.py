"""Build Unitree SDK2 robot channels from resolved robot runtime configuration.

``UnitreeSDK2Driver`` initializes the SDK's DDS channel factory and converts
robot specifications plus per-channel configuration into concrete robot runtime
``RobotChannel`` implementations.
"""

from typing import Any

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from robot_runtime.configs.sections.robot.composition import RobotSpec
from robot_runtime.interfaces.robot.channel import RobotChannel
from robot_runtime.interfaces.robot.component import ComponentInterface
from robot_runtime.drivers.unitree_sdk2.robot.component.base_group.g1_d_base import (
    G1DBaseInterface,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.base_group.g1_edu_pseudo_base import (
    G1EduPseudoBaseInterface,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.base_group.r1_d_base import (
    R1DBaseInterface,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.end_effector.brainco.revo2 import (
    Revo2Interface,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.end_effector.inspire.rh56dftp import (
    InspireRH56DFTPInterface,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.end_effector.unitree.dex1_1 import (
    Dex1_1Interface,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.end_effector.unitree.dex2_5 import (
    Dex2_5Interface,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.end_effector.unitree.dex3_1 import (
    Dex3_1Interface,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.end_effector.unitree.dex5_1 import (
    Dex5_1Interface,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.body.hg_body import (
    HGBodyInterface,
)
from robot_runtime.drivers.unitree_sdk2.robot.component.sensor.imu import (
    IMUInterface,
)
from robot_runtime.drivers.unitree_sdk2.robot.channel.low_state.cmd.low_cmd_robot_channel import (
    LowCmdRobotChannel,
)
from robot_runtime.drivers.unitree_sdk2.robot.channel.dds_robot_channel import (
    DDSRobotChannel,
)
from robot_runtime.drivers.unitree_sdk2.robot.channel.low_state.cmd.whole_body_sdk_robot_channel import (
    WholeBodySdkRobotChannel,
)
from robot_runtime.drivers.unitree_sdk2.robot.channel.low_state.cmd.hybrid_wbc_robot_channel import (
    HybridWBCRobotChannel,
)
from robot_runtime.drivers.unitree_sdk2.robot.channel.low_state.cmd.pose_82d_robot_channel import (
    Pose82dRobotChannel,
)


class UnitreeSDK2Driver:
    """Construct Unitree SDK2 component interfaces and robot channels.

    The driver uses ``robot_spec`` for device geometry, gains, limits, and
    schema mappings. Channel configuration supplies transport topics and the
    concrete Unitree protocol selected for each robot component.

    Args:
        robot_spec: Resolved robot specification used to build interfaces.
        dds_domain_id: CycloneDDS domain passed to the Unitree SDK.
        network_interface: Optional network interface used by CycloneDDS.

    Attributes:
        robot_spec: Resolved robot component specifications.
        dds_domain_id: Configured CycloneDDS domain ID.
        network_interface: Configured network interface, if any.

    Example:
        driver = UnitreeSDK2Driver(
            robot_spec=robot_spec,
            dds_domain_id=0,
            network_interface="eth0",
        )
        driver.initialize()
        channel = driver.build_channel(
            channel_key="body",
            channel_cfg=body_channel_cfg,
        )
    """

    def __init__(
        self,
        *,
        robot_spec: RobotSpec,
        dds_domain_id: int = 0,
        network_interface: str | None = None,
    ) -> None:
        self.robot_spec = robot_spec
        self.dds_domain_id = dds_domain_id
        self.network_interface = network_interface

    def initialize(self) -> None:
        """Initialize the Unitree DDS factory with the configured domain."""
        ChannelFactoryInitialize(
            self.dds_domain_id,
            networkInterface=self.network_interface,
        )

    def build_channel(
        self,
        *,
        channel_key: str,
        channel_cfg: dict[str, Any],
    ) -> RobotChannel:
        """Build one robot channel selected by its configured driver type."""
        driver_cfg = channel_cfg["driver"]
        driver_type = driver_cfg["type"]

        if driver_type == "unitree_sdk2.low_level":
            return self.build_low_cmd_channel(
                channel_cfg=channel_cfg,
            )

        if driver_type == "unitree_sdk2.whole_body_sdk":
            return self.build_whole_body_sdk_channel(
                channel_cfg=channel_cfg,
            )

        if driver_type in {"unitree_sdk2.pose_82d", "unitree_sdk2.hybrid_wbc"}:
            return self.build_pose_82d_channel(
                channel_cfg=channel_cfg,
            )

        if driver_type == "unitree_sdk2.base_lift":
            return self.build_base_lift_channel(
                channel_cfg=channel_cfg,
            )

        if driver_type == "unitree_sdk2.cmd_vel":
            return self.build_cmd_vel_channel(
                channel_cfg=channel_cfg,
            )

        if driver_type == "unitree_sdk2.brainco_revo2":
            return self.build_revo2_channel(
                channel_cfg=channel_cfg,
            )

        if driver_type == "unitree_sdk2.dex1_1":
            return self.build_dex1_1_channel(
                channel_cfg=channel_cfg,
            )

        if driver_type == "unitree_sdk2.dex3_1":
            return self.build_dex3_1_channel(
                channel_cfg=channel_cfg,
            )

        if driver_type == "unitree_sdk2.dex5_1":
            return self.build_dex5_1_channel(
                channel_cfg=channel_cfg,
            )

        if driver_type == "unitree_sdk2.inspire_rh56dftp":
            return self.build_inspire_rh56dftp_channel(
                channel_cfg=channel_cfg,
            )

        raise KeyError(
            f"Unsupported Unitree SDK2 channel {channel_key!r}: "
            f"{driver_type!r}."
        )

    def build_hg_body_interface(
        self,
        *,
        component_config: dict[str, Any],
    ) -> HGBodyInterface:
        """Build the HG body mapping from the robot body specification."""
        body_spec = self.robot_spec.joint_group_specs[0]
        kp, kd = body_spec.build_dense_gains()
        return HGBodyInterface(
            component_id=component_config["component_id"],
            motor_slots=body_spec.motor_slots,
            motor_indices={
                schema_key: body_spec.motor_indices[schema_key]
                for schema_key in body_spec.schema_keys
            },
            kp=kp,
            kd=kd,
        )

    def build_internal_dex_interface(
        self,
        *,
        component_config: dict[str, Any],
    ) -> Dex1_1Interface | Dex2_5Interface:
        """Build an internal Dex mapping for shared HG motor messages."""
        state_schema_keys = tuple(
            component_config["schema_bindings"]["state"]["joint_group"]
        )
        end_effector_spec = self.robot_spec.joint_group_spec(
            state_schema_keys[0]
        )
        motor_indices = {
            schema_key: end_effector_spec.motor_indices[schema_key]
            for schema_key in state_schema_keys
        }
        shared_indices = component_config["shared_motor_indices"]
        shared_motor_indices = {}
        offset = 0
        for schema_key in state_schema_keys:
            next_offset = offset + len(motor_indices[schema_key])
            shared_motor_indices[schema_key] = shared_indices[
                offset:next_offset
            ]
            offset = next_offset
        if component_config["interface_type"] == "dex2_5":
            return Dex2_5Interface(
                component_id=component_config["component_id"],
                motor_indices=motor_indices,
                shared_motor_indices=shared_motor_indices,
                qtau_limits=end_effector_spec.qtau_limits,
            )
        kp, kd = end_effector_spec.build_dense_gains()
        return Dex1_1Interface(
            component_id=component_config["component_id"],
            wiring=component_config["wiring"],
            motor_indices=motor_indices,
            motor_slots=end_effector_spec.motor_slots,
            kp=kp,
            kd=kd,
            shared_motor_indices=shared_motor_indices,
        )

    def build_low_state_component_interfaces(
        self,
        *,
        channel_cfg: dict[str, Any],
    ) -> tuple[
        HGBodyInterface,
        IMUInterface,
        tuple[ComponentInterface, ...],
    ]:
        """Build all component mappings carried by one LowState channel."""
        body_config, *embedded_configs = channel_cfg["components"]
        body_interface = self.build_hg_body_interface(
            component_config=body_config,
        )
        imu_interface: IMUInterface | None = None
        embedded_interfaces: list[ComponentInterface] = []
        for component_config in embedded_configs:
            interface_type = component_config["interface_type"]
            if interface_type in {"dex1_1", "dex2_5"}:
                component_interface = self.build_internal_dex_interface(
                    component_config=component_config,
                )
                embedded_interfaces.append(component_interface)
                continue
            if interface_type == "imu":
                if imu_interface is not None:
                    raise ValueError(
                        "Unitree SDK2 body channel received multiple "
                        "IMU component interfaces."
                    )
                imu_interface = IMUInterface(
                    component_id=component_config["component_id"],
                    schema_key=component_config["schema_bindings"]["state"]["imu"][0],
                )
                continue
            if interface_type == "g1_edu_pseudo_base":
                component_interface = G1EduPseudoBaseInterface(
                    component_id=component_config["component_id"],
                    schema_key=self.robot_spec.base_group_spec.schema_keys[0],
                )
                embedded_interfaces.append(component_interface)
                continue
            raise KeyError(
                "Unsupported Unitree SDK2 component interface "
                f"{component_config.get('component_id')!r}: "
                f"{interface_type!r}."
            )
        if imu_interface is None:
            raise ValueError(
                "Unitree SDK2 body channel requires an IMU interface."
            )
        return (
            body_interface,
            imu_interface,
            tuple(embedded_interfaces),
        )

    def build_low_cmd_channel(
        self,
        *,
        channel_cfg: dict[str, Any],
    ) -> LowCmdRobotChannel:
        """Build the shared HG LowState and LowCmd channel."""
        (
            body_interface,
            imu_interface,
            embedded_interfaces,
        ) = self.build_low_state_component_interfaces(
            channel_cfg=channel_cfg,
        )
        driver_cfg = channel_cfg["driver"]
        writable_component_ids = {
            component_config["component_id"]
            for component_config in channel_cfg["components"][1:]
            if (
                component_config.get("schema_bindings", {})
                .get("cmd", {})
                .get("joint_group", ())
            )
        }
        writable_embedded_interfaces = tuple(
            interface
            for interface in embedded_interfaces
            if interface.component_id in writable_component_ids
        )
        body_config = channel_cfg["components"][0]
        body_cmd_schema_keys = tuple(
            body_config["schema_bindings"]["cmd"]["joint_group"]
        )
        motor_topics = {}
        if driver_cfg.get("motor_topics"):
            body_spec = self.robot_spec.joint_group_specs[0]
            indices_by_name = {
                name: index
                for key in body_spec.schema_keys
                for name, index in zip(
                    body_spec.joint_names[key], body_spec.motor_indices[key], strict=True,
                )
            }
            for joint_name, topics in driver_cfg["motor_topics"].items():
                if joint_name not in indices_by_name:
                    raise ValueError(f"motor_topics names unknown body joint {joint_name!r}.")
                motor_topics[indices_by_name[joint_name]] = (
                    topics["state_topic"], topics["cmd_topic"],
                )
        return LowCmdRobotChannel(
            body_interface=body_interface,
            imu_interface=imu_interface,
            cmd_topic=driver_cfg["cmd_topic"],
            body_cmd_schema_keys=body_cmd_schema_keys,
            embedded_interfaces=embedded_interfaces,
            writable_embedded_interfaces=writable_embedded_interfaces,
            control_weight_motor_index=driver_cfg.get(
                "control_weight_motor_index"
            ),
            restore_motion_mode=driver_cfg.get("restore_motion_mode"),
            motor_topics=motor_topics,
            start_timeout=channel_cfg.get("start_timeout", 10.0),
        )

    def build_whole_body_sdk_channel(
        self,
        *,
        channel_cfg: dict[str, Any],
    ) -> WholeBodySdkRobotChannel:
        """Build the HG LowState and whole-body JSON command channel."""
        (
            body_interface,
            imu_interface,
            embedded_interfaces,
        ) = self.build_low_state_component_interfaces(
            channel_cfg=channel_cfg,
        )
        body_config = channel_cfg["components"][0]
        body_cmd_bindings = body_config["schema_bindings"]["cmd"]
        body_cmd_schema_keys = tuple(body_cmd_bindings["joint_group"])
        base_group_interfaces = tuple(
            interface
            for interface in embedded_interfaces
            if isinstance(interface, G1EduPseudoBaseInterface)
        )
        if len(base_group_interfaces) != 1:
            raise ValueError(
                "whole_body_sdk requires one G1 EDU pseudo-base interface."
            )
        (base_group_interface,) = base_group_interfaces
        embedded_interfaces = tuple(
            interface
            for interface in embedded_interfaces
            if interface is not base_group_interface
        )
        driver_cfg = channel_cfg["driver"]
        return WholeBodySdkRobotChannel(
            body_interface=body_interface,
            imu_interface=imu_interface,
            base_group_interface=base_group_interface,
            body_cmd_schema_keys=body_cmd_schema_keys,
            base_group_schema_key=base_group_interface.schema_key,
            control_motion_mode=driver_cfg["control_motion_mode"],
            restore_motion_mode=driver_cfg["restore_motion_mode"],
            embedded_interfaces=embedded_interfaces,
            start_timeout=channel_cfg.get("start_timeout", 10.0),
        )

    def build_pose_82d_channel(
        self,
        *,
        channel_cfg: dict[str, Any],
    ) -> Pose82dRobotChannel:
        """Build the HG LowState and FSM pose-command channel."""
        (
            body_interface,
            imu_interface,
            embedded_interfaces,
        ) = self.build_low_state_component_interfaces(
            channel_cfg=channel_cfg,
        )
        driver_cfg = channel_cfg["driver"]
        if driver_cfg["type"] == "unitree_sdk2.hybrid_wbc":
            if len(embedded_interfaces) != 1 or not isinstance(
                embedded_interfaces[0], G1EduPseudoBaseInterface,
            ):
                raise ValueError("hybrid_wbc requires one pseudo-base interface.")
            return HybridWBCRobotChannel(
                body_interface=body_interface,
                imu_interface=imu_interface,
                base_group_interface=embedded_interfaces[0],
                control_fsm_id=driver_cfg["control_fsm_id"],
                restore_fsm_id=driver_cfg["restore_fsm_id"],
                start_timeout=channel_cfg.get("start_timeout", 10.0),
            )
        if embedded_interfaces:
            raise ValueError(
                "pose_82d does not support embedded component interfaces."
            )
        return Pose82dRobotChannel(
            body_interface=body_interface,
            imu_interface=imu_interface,
            control_fsm_id=driver_cfg["control_fsm_id"],
            restore_fsm_id=driver_cfg["restore_fsm_id"],
            include_frame_id=driver_cfg.get("include_frame_id", False),
            start_timeout=channel_cfg.get("start_timeout", 10.0),
        )

    def build_base_lift_channel(
        self,
        *,
        channel_cfg: dict[str, Any],
    ) -> DDSRobotChannel:
        """Build the G1-D base height and planar-motion DDS channel."""
        driver_cfg = channel_cfg["driver"]
        component_config = channel_cfg["components"][0]
        component_interface = G1DBaseInterface(
            component_id=component_config["component_id"],
            schema_key=component_config["schema_bindings"]["state"][
                "base_group"
            ][0],
        )
        return DDSRobotChannel(
            state_topics={
                driver_cfg["height_state_topic"]: (
                    component_interface.HEIGHT_STATE_MSG_TYPE
                ),
                driver_cfg["move_state_topic"]: (
                    component_interface.MOVE_STATE_MSG_TYPE
                ),
            },
            cmd_topics={
                driver_cfg["height_cmd_topic"]: (
                    component_interface.HEIGHT_CMD_MSG_TYPE,
                    component_interface.HEIGHT_CMD_MSG_FACTORY,
                ),
                driver_cfg["move_cmd_topic"]: (
                    component_interface.MOVE_CMD_MSG_TYPE,
                    component_interface.MOVE_CMD_MSG_FACTORY,
                ),
            },
            components={
                component_interface.component_id: component_interface,
            },
            writable_components=(component_interface.component_id,),
            start_timeout=channel_cfg.get("start_timeout", 10.0),
        )

    def build_cmd_vel_channel(
        self,
        *,
        channel_cfg: dict[str, Any],
    ) -> DDSRobotChannel:
        """Build the R1-D command and measured base-state DDS channel."""
        driver_cfg = channel_cfg["driver"]
        component_config = channel_cfg["components"][0]
        component_interface = R1DBaseInterface(
            component_id=component_config["component_id"],
            schema_key=component_config["schema_bindings"]["state"][
                "base_group"
            ][0],
        )
        return DDSRobotChannel(
            state_topics={
                driver_cfg["move_state_topic"]: (
                    component_interface.MOVE_STATE_MSG_TYPE
                ),
                driver_cfg["height_state_topic"]: (
                    component_interface.HEIGHT_STATE_MSG_TYPE
                ),
                driver_cfg["height_alarm_topic"]: (
                    component_interface.HEIGHT_ALARM_MSG_TYPE
                ),
            },
            cmd_topics={
                driver_cfg["cmd_topic"]: (
                    component_interface.CMD_MSG_TYPE,
                    component_interface.CMD_MSG_FACTORY,
                ),
            },
            components={component_interface.component_id: component_interface},
            writable_components=(component_interface.component_id,),
            start_timeout=channel_cfg.get("start_timeout", 10.0),
        )

    def build_single_component_dds_channel(
        self,
        *,
        channel_cfg: dict[str, Any],
        component_interface: ComponentInterface,
    ) -> DDSRobotChannel:
        """Wrap one standalone component mapping in a DDS robot channel."""
        driver_cfg = channel_cfg["driver"]
        return DDSRobotChannel(
            state_topics={
                driver_cfg["state_topic"]: component_interface.STATE_MSG_TYPE,
            },
            cmd_topics={
                driver_cfg["cmd_topic"]: (
                    component_interface.CMD_MSG_TYPE,
                    component_interface.CMD_MSG_FACTORY,
                ),
            },
            components={
                component_interface.component_id: component_interface,
            },
            writable_components=(component_interface.component_id,),
            start_timeout=channel_cfg.get("start_timeout", 10.0),
        )

    def build_revo2_channel(
        self,
        *,
        channel_cfg: dict[str, Any],
    ) -> DDSRobotChannel:
        """Build one standalone BrainCo Revo2 DDS channel."""
        component_config = channel_cfg["components"][0]
        schema_keys = tuple(
            component_config["schema_bindings"]["cmd"]["joint_group"]
        )
        end_effector_spec = self.robot_spec.joint_group_spec(schema_keys[0])
        kp, kd = end_effector_spec.build_dense_gains()
        component_interface = Revo2Interface(
            component_id=component_config["component_id"],
            motor_indices={
                schema_key: end_effector_spec.motor_indices[schema_key]
                for schema_key in schema_keys
            },
            motor_slots=end_effector_spec.motor_slots,
            qpos_limits={
                schema_key: end_effector_spec.qpos_limits[schema_key]
                for schema_key in schema_keys
            },
            kp=kp,
            kd=kd,
        )
        return self.build_single_component_dds_channel(
            channel_cfg=channel_cfg,
            component_interface=component_interface,
        )

    def build_dex1_1_channel(
        self,
        *,
        channel_cfg: dict[str, Any],
    ) -> DDSRobotChannel:
        """Build one standalone Unitree Dex1-1 DDS channel."""
        component_config = channel_cfg["components"][0]
        schema_keys = tuple(
            component_config["schema_bindings"]["cmd"]["joint_group"]
        )
        end_effector_spec = self.robot_spec.joint_group_spec(schema_keys[0])
        kp, kd = end_effector_spec.build_dense_gains()
        driver_cfg = channel_cfg["driver"]
        component_interface = Dex1_1Interface(
            component_id=component_config["component_id"],
            wiring=driver_cfg["wiring"],
            motor_indices={
                schema_key: end_effector_spec.motor_indices[schema_key]
                for schema_key in schema_keys
            },
            motor_slots=end_effector_spec.motor_slots,
            kp=kp,
            kd=kd,
        )
        return self.build_single_component_dds_channel(
            channel_cfg=channel_cfg,
            component_interface=component_interface,
        )

    def build_dex3_1_channel(
        self,
        *,
        channel_cfg: dict[str, Any],
    ) -> DDSRobotChannel:
        """Build one standalone Unitree Dex3-1 DDS channel."""
        component_config = channel_cfg["components"][0]
        schema_keys = tuple(
            component_config["schema_bindings"]["cmd"]["joint_group"]
        )
        end_effector_spec = self.robot_spec.joint_group_spec(schema_keys[0])
        kp, kd = end_effector_spec.build_dense_gains()
        component_interface = Dex3_1Interface(
            component_id=component_config["component_id"],
            motor_indices={
                schema_key: end_effector_spec.motor_indices[schema_key]
                for schema_key in schema_keys
            },
            motor_slots=end_effector_spec.motor_slots,
            kp=kp,
            kd=kd,
        )
        return self.build_single_component_dds_channel(
            channel_cfg=channel_cfg,
            component_interface=component_interface,
        )

    def build_dex5_1_channel(
        self,
        *,
        channel_cfg: dict[str, Any],
    ) -> DDSRobotChannel:
        """Build one standalone Unitree Dex5-1 DDS channel."""
        component_config = channel_cfg["components"][0]
        schema_keys = tuple(
            component_config["schema_bindings"]["cmd"]["joint_group"]
        )
        end_effector_spec = self.robot_spec.joint_group_spec(schema_keys[0])
        kp, kd = end_effector_spec.build_dense_gains()
        component_interface = Dex5_1Interface(
            component_id=component_config["component_id"],
            motor_indices={
                schema_key: end_effector_spec.motor_indices[schema_key]
                for schema_key in schema_keys
            },
            motor_slots=end_effector_spec.motor_slots,
            kp=kp,
            kd=kd,
        )
        return self.build_single_component_dds_channel(
            channel_cfg=channel_cfg,
            component_interface=component_interface,
        )

    def build_inspire_rh56dftp_channel(
        self,
        *,
        channel_cfg: dict[str, Any],
    ) -> DDSRobotChannel:
        """Build one standalone Inspire RH56DFTP DDS channel."""
        component_config = channel_cfg["components"][0]
        schema_keys = tuple(
            component_config["schema_bindings"]["cmd"]["joint_group"]
        )
        end_effector_spec = self.robot_spec.joint_group_spec(schema_keys[0])
        component_interface = InspireRH56DFTPInterface(
            component_id=component_config["component_id"],
            motor_indices={
                schema_key: end_effector_spec.motor_indices[schema_key]
                for schema_key in schema_keys
            },
            motor_slots=end_effector_spec.motor_slots,
            qpos_limits={
                schema_key: end_effector_spec.qpos_limits[schema_key]
                for schema_key in schema_keys
            },
            invert={
                schema_key: end_effector_spec.invert[schema_key]
                for schema_key in schema_keys
            },
            scale={
                schema_key: end_effector_spec.scale[schema_key]
                for schema_key in schema_keys
            },
        )
        return self.build_single_component_dds_channel(
            channel_cfg=channel_cfg,
            component_interface=component_interface,
        )
