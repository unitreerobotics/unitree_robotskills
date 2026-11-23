# Documentation index

English | [中文](README.zh.md)

| Document | Content |
|---|---|
| [operation_manual.md](operation_manual.md) | Main specification: environment, interfaces and coordinates, start and takeover, per-round planning, execution and gates, review, fault handling |
| [round_script_inventory.md](round_script_inventory.md) | Mapping between subcommands, entry scripts, and implementation modules |
| [robot_service_architecture.md](robot_service_architecture.md) | How the chess service is set up and how motion is sent to the R1-D |
| [r1d_scene_parameters.md](r1d_scene_parameters.md) | Meaning of the TCP, grip point, board, and scene parameters and how to determine them |
| [r1d_joint_tracking_feedforward.md](r1d_joint_tracking_feedforward.md) | Role of the arm torque feedforward, the control law, and the tracking limits |
| [r1d_twist_dds_topics.md](r1d_twist_dds_topics.md) | Chassis and lift feedback used by the chess service, and the criteria |

The bring-up procedure for a new robot or a new site is in [../setup/README.md](../setup/README.md): local environment, camera check, arm accuracy calibration, head camera calibration, chess scene parameters, and board calibration. For the built-in robot runtime (the only process that sends commands to the robot), see [../third_party/robot_runtime/README.md](../third_party/robot_runtime/README.md). The execution order of each round is in [../prompts/round_plan.md](../prompts/round_plan.md), board reading and board re-measurement are in [../board_position/README.md](../board_position/README.md), and the configuration file description is in [../config/README.md](../config/README.md).

Values marked as "reference" in the documents come from measurements on one reference robot; they only indicate the order of magnitude and cannot be taken as measured values for a new robot or a new site.
