# Experience Data Conventions

English | [中文](experience.zh.md)

Experience is stored as JSON; dimensions, poses, arm-configuration joints, gripper openings or old trajectories are not pre-filled in Markdown.
For the general assessment methods see [Manipulation methods](manipulation.md); for the data structure see [schema.json](../experience/schema.json).

| Location | Purpose |
| --- | --- |
| Project `experience/template.json` | Empty experience library, shipped with the package, read-only at runtime |
| Project `experience/schema.json` | Data structure for records and candidates, for tools and editors |
| Workspace `experience/context.json` | Digest of the current robot, scene and configuration, rest-pose reference, calibration and model |
| Workspace `experience/binding.json` | Workspace-relative paths of the current configuration and calibration |
| Workspace `experience/records.json` | Measured experience appended after success; old records are not overwritten |
| Round `experience_candidate.json` | Data to be validated, compiled by the agent after a successful return home |

## First execution

Initialization only creates an empty library; it does not inject old experience and does not overwrite existing data:

```bash
python -m unitree_gpt6.g1d_gomoku.src.experience init \
  --workspace "$GOMOKU_WORKSPACE"
```

First capture the rest-pose reference of the current device, and establish the scene calibration based on the four-way images, current feedback and the model.
The agent writes the calibration results and their basis to the workspace `calibration/scene.json`; when scale or coordinate relationships are missing, continue observing and calibrating; do not generate them out of thin air.
The rest-pose reference is calibration data prior to execution, not successful experience, and need not wait until a move is finished to be captured.

The agent chooses a stable `robot-id` and `scene-id` for the current device and layout, and establishes the binding:

```bash
python -m unitree_gpt6.g1d_gomoku.src.experience context \
  --workspace "$GOMOKU_WORKSPACE" \
  --robot-id ROBOT_ID --scene-id SCENE_ID \
  --config "$GOMOKU_HARDWARE_CONFIG" \
  --calibration "$GOMOKU_WORKSPACE/calibration/scene.json"

python -m unitree_gpt6.g1d_gomoku.src.experience show \
  --workspace "$GOMOKU_WORKSPACE"
```

When the robot or the board, supply area or camera layout changes, update the corresponding ID and calibration file and rebuild the binding.
`show` returns only records under the current binding whose evidence digests are still consistent; the first time it returns an empty list.
If the configuration, rest-pose reference, calibration or model changes without rebuilding the binding, reading is refused; it does not automatically fall back to old values.
Do not import legacy default rest-pose files or experience. Solve independently on the first execution; zero values or example values must not stand in for measurements.

## Writing after success

After returning home, the agent compiles the candidate file from this round's images, feedback, plans and results. The fields are as follows:

| Field | Requirement |
| --- | --- |
| `context` | Full copy of the current `experience/context.json` |
| `observations` | Non-empty list of observations; fill in only items actually verified |
| `observations[].kind` | `geometry`, `grasp`, `alignment`, `release`, `return` or `placement` |
| `target` | Target-value object; `{}` when there is no corresponding target |
| `measured` | Non-empty measured-value object; command targets must not pose as feedback; omit unknown fields |
| `verification` | The corresponding check is true, with a non-empty `method` describing the verification method |
| `evidence` | List of relative file paths under this round's directory, pointing to the images, feedback and results actually read |

Field names in `measured` and `target` must indicate units, for example `tcp_pose_m_rad`, `joint_positions_rad`,
`gripper_rad`, `grid_spacing_m`, `duration_s`; when recording poses also state the `frame` and joint order.
When recording the grasp arm configuration, save the measured shoulder/elbow/wrist joints and the gripper orientation, not only the TCP.
Do not save fixed game records, landing points, user authorizations or motion scripts that could be blindly replayed.

| kind | Check item in verification | Evidence requirement |
| --- | --- | --- |
| `geometry` | `calibration_verified` | Current calibration, raw images and scale/coordinate verification |
| `grasp` | `single_stone_held` | Short-lift images, empty original spot and current joint/gripper feedback |
| `alignment` | `selected_intersection_verified` | Target intersection before release, neighboring-stone grid and low-position feedback |
| `release` | `release_completed` | Gripper feedback and this round's release result; does not indicate landing-point accuracy |
| `return` | `reset_joint_verified` | This round's rest-pose feedback and result |
| `placement` | `physical_placement_verified` | Independent post-placement visual verification and result |

The current flow does not analyze images after placement and `physical_placement_verified` is false, so `placement` success experience must not be published.
Successful alignment before release can be recorded separately as `alignment`, but must not be relabeled as actual on-board placement accuracy.

```bash
python -m unitree_gpt6.g1d_gomoku.src.experience record \
  --workspace "$GOMOKU_WORKSPACE" \
  --round "$GOMOKU_ROUND" \
  --input "$GOMOKU_ROUND/experience_candidate.json"
```

The tool requires this round to be `returned` with the rest-pose verification passed, checks the deployment binding, finite numbers, each kind of verification claim and the evidence files,
computes the evidence digests, then locks and appends atomically; the same round can be submitted only once, and all observations are submitted together.
Candidate data and evidence must be located in the workspace, with evidence further restricted to this round's directory; external private directories must not be referenced.
Failed, unverified or plan-only data stays only in the runtime logs and does not enter the successful-experience library.

The tool is responsible for structure and traceability validation; it does not inspect images on the agent's behalf, nor does it independently prove visual or physical success.
All learning writes happen after returning home and do not prolong low-position waiting. Subsequent experience only assists candidate solving; current feedback, images, limits and clearance checks are still required.
