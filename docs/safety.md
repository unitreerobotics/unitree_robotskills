# Robot execution safety

English | [中文](safety.zh.md)

## Current support boundary

The unified navigation preview CLI entry point provides only a local no-motion preview; it is not a physics simulation. This entry point has no implementation of connection checks, inverse kinematics, obstacle avoidance, contact force control or emergency stop. `--mode hardware` and any non-`preview` configuration are rejected. The robot model in the configuration only verifies that the declarations are consistent; it cannot confirm the actual device.

Exception: `g1d_r1d_navigation/src/navi_agent/` is a separate real-robot runtime that drives base motion. It has lidar reflex-layer speed limiting, a command watchdog and a software stop, but none of these can replace a hardware emergency stop; before use, read the safety layers and the on-site checklist in that skill's README.

Chinese chess (Xiangqi) includes real robot SDK interfaces, inverse kinematics, services and execution scripts; its configuration and capability boundaries must be checked against its [SKILL.md](../unitree_gpt6/g1d_chess/SKILL.md) and [runtime guide](../unitree_gpt6/g1d_chess/docs/runtime.md). The preview-mode limitations above do not apply to the chess entry point.

All model outputs and plans should be treated as inputs that still need to be verified. Real-robot interface integration must be designed separately; execution must not be started merely by removing the mode check.

Painting provides a G1-D / Dex1 hardware backend, connection and feedback checks, URDF / IK compilation, execution stop and control authority management. Compilation and runs still require models, tools and on-site measurements that match the actual device.

Painting currently retains checks for stale feedback, publish failures, control mode changes, sustained tracking errors, contact-point arrival, minimum lift-off height and offline joint limits; the position and orientation of in-air (off-paper) targets and the joint deviation of the final rest pose are recorded to the log and are not used as strict arrival thresholds. A program stop is not the same as an on-site hardware emergency stop, and there is currently no automatic collision detection, closed-loop contact force control or temperature / current threshold protection.

Gomoku includes camera, SDK, FK/IK and persistent control tools; the private configuration, calibration, publisher health and motion evidence must be checked first. Releasing the gripper requires a recorded explicit user confirmation; after release, the robot must retreat continuously and the return home must be verified; the model must not bypass these conditions. For interfaces and known boundaries, see the [autonomous round guide](../unitree_gpt6/g1d_gomoku/docs/autonomous_play.md) and [hardware integration](../unitree_gpt6/g1d_gomoku/docs/hardware_integration.md).

## Requirements before hardware integration

1. Verify the robot model, SDK/firmware, connection, control authority and device state.
2. Calibrate the camera, end effector, board, painting plane or navigation map; define coordinate frames and units explicitly.
3. Set explicit limits on position, joints, velocity, acceleration, contact force and workspace.
4. Validate actions again after planning and before every execution; the model cannot bypass the adapter's constraints.
5. Set task and per-step timeouts, an independent execution watchdog and handling for communication loss.
6. Provide mechanisms for task cancellation, stopping and entering a safe state, and first verify that the on-site emergency stop works.
7. Perform no-motion checks and appropriate simulation first, then proceed step by step with real-robot validation in an isolated area under personnel supervision.

These are the acceptance requirements for hardware integration; the capabilities, differences and limitations already implemented by each skill are as stated in its runtime documentation, and offline validation cannot replace on-site acceptance.

## Scenario requirements

- **Painting**: check pen-tip contact, table height, range of motion and force limits, and avoid collisions of the tool or the robot body.
- **Board games**: re-check board state, game rules, pick and place targets and grasp state; stop when vision is uncertain and do not continue on assumptions.
- **Navigation**: handle localization failure, obstacles, dynamic scenes, map changes and task timeouts; a goal point is not the same as a safely traversable route.

## Records and data

Each skill's documentation should distinguish local preview, simulation validation and real-robot validation, and record the hardware model, software version, configuration and known limitations. Publish on-site images, maps and logs only after authorization. Secrets and robot access credentials are provided by the local environment or a secrets manager and are never committed to Git.

When an anomaly occurs, follow the on-site emergency stop and isolation procedure; automatic retries must have a limited count and must not repeat motion while the environment state is uncertain.
