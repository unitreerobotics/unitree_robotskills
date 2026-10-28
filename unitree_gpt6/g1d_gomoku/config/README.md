# Configuration and On-Site Calibration

English | [中文](README.zh.md)

`default.yaml` only defines the autonomous mode, the 19×19 board and the game rules; it contains no on-site experience.
Copy `hardware.example.yaml` to `config/hardware.local.yaml` in the workspace.

| Field | Configuration requirement |
| --- | --- |
| `camera.host` | Required: current camera service address; no default address |
| `robot.network_interface` | Required: network interface connected to the G1; no default interface |
| Camera ports | Template: head 55555, left wrist 55556, right wrist 55557; adjust according to the service configuration |
| DDS domain / topics | Template: 0, `rt/lowcmd`, `rt/lowstate` |
| Control frequency / max joint speed | Template: 100 Hz / 0.4 rad/s; these are project control settings |
| Gripper speed / range | Template: 0.15 rad/s / 0–5.45 rad; these are control settings and device range |
| `runtime.directory` | `../runtime` |
| `runtime.scene_calibration` | `../calibration/scene.json`; first measured and reviewed by the agent |
| `runtime.reset_state` | `../calibration/reset_right_state.json`; must first be captured read-only |

Relative paths are resolved against the directory containing the YAML; all control processes share the same configuration and runtime lock.

The project ships no rest-pose reference values. After confirming that the current device is in a suitable resting posture, the agent captures them with the read-only command in the README:
`joint_names` and `joint_positions` are the seven left-arm joints and seven right-arm joints; `waist_q` is the three waist joints;
`poses.left/right` are `[x, y, z, roll, pitch, yaw]` in pelvis; `grippers` is the left and right gripper feedback.
Positions are in meters, angles in radians, and the joint order must match the model. Zeros must not be filled in place of feedback.

Return home in a continuous session restores the right arm from the reference while keeping the current left arm and the task-start waist angle;
a standalone `reset` restores both arms, the reference waist angle and the left gripper, while the right gripper keeps its current opening.
The rest-pose file is a pre-execution reference and does not mean pick-and-place has already succeeded.

Board dimensions, pick-and-place heights, arm configuration and gripper experience are initially empty; after success they are saved to the workspace according to [Experience data conventions](../docs/experience.md).
Hardware limits, control frequency and game rules are not learned data and cannot be overridden by experience.

## Scene geometry data

The structure of `calibration/scene.json` is defined in [scene.schema.json](scene.schema.json); there are no numerical defaults.
The agent builds this file from the current images, model, feedback and scale basis, and saves the measurement method and image basis.

| Field | Meaning |
| --- | --- |
| `frame` | `pelvis` |
| `tool_tip_offset_m` | 3D offset in the TCP local frame to the fingertip checkpoint, in meters |
| `high_route_min_tcp_z_m` | Minimum TCP Z allowed in the reviewed high-position corridor, in meters |
| `high_route_min_tip_z_m` | Minimum fingertip-checkpoint Z allowed in the reviewed high-position corridor, in meters |
| `rest_corridor.left/right.min_m/max_m` | 3D range of the left/right arm end-effector rest-pose corridor; per axis the lower bound is less than the upper bound |
| `measurements` | Other actually measured grid spacing, board extent, table and board-surface heights, etc.; field names indicate units |
| `evidence` | Workspace-relative paths of calibration images and feedback |

These data are used for FK sampling and corridor checks; they are not a complete collision model; the gripper, held stone, arm and surrounding obstacles must still be checked.
The pre-check only validates structure and finite numbers and does not claim that on-site calibration is already accurate.
