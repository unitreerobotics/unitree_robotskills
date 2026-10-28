# How to Configure a Brand-New G1D Before It Starts Playing Chinese Chess (Xiangqi)

English | [中文](first_game_setup.zh.md)

First install the requirements and the Xiangqi engine according to the [environment setup guide](environment_setup.md), then complete the on-site configuration on this page, and only then use [round_plan](../prompts/round_plan.md). This package targets G1 with Dex1 and the `assets/models/g1/` model; on first deployment, archived images, init, scene, gripper direction or template poses must not be treated as measurements of the new device. `config/` only provides templates and notes; it does not automatically detect hardware or apply parameters.

## 1. Determine the install location and interpreter (offline)

You can copy `.env.example` to `.env`, fill in its existing fields, and then explicitly run `set -a; source .env; set +a` in every new shell. The project does not load `.env` automatically, and configuration JSON does not set environment variables automatically either. The example below shows the equivalent explicit-variable approach.

On the target machine, choose the package root and a compatible Python. All `<…>` below must be replaced; the example IPs and paths in the docs only illustrate the format. Shell variables are not kept across independent tool calls; set them again in every new call or use the actual absolute paths.

```bash
cd /path/to/g1d_chess
source /path/to/robot-env/bin/activate
export CHESS_ENGINE=/path/to/fairy-stockfish
CHESS_ROOT="$PWD"
CHESS_PYTHON="$(command -v python)"
CHESS_CLIENT="$CHESS_ROOT/scripts/robot_client.py"
CHESS_SERIAL="$CHESS_ROOT/scripts/serial_checkpoint.py"
"$CHESS_PYTHON" scripts/check_environment.py
"$CHESS_PYTHON" scripts/verify_package.py
```

Use a Python environment with the dependencies of the `chess` group of `pyproject.toml` installed; after activating it you can run `python` directly, or explicitly call the absolute path of that environment's interpreter. The SDK and rules library are imported from this environment; the model is read from this package's `assets/models/g1/`; no third-party source paths are added, and there is no dependency on the development machine's Conda path or `unitree_deploy`. Every independent tool call must re-activate the environment or use the confirmed absolute interpreter path.

Following the `_field_help` of `config/environment.example.json`, fill in the actual deployment record `config/package_environment.json`. `package_root` records the actual absolute path; `python_executable` records the absolute path of the interpreter with the dependencies installed; `engine` records the external engine path or PATH command name. The actual interpreter is selected by the currently activated environment or an explicit command; these fields are only records. Neither JSON automatically exports variables or passes arguments to the launcher.

The target machine must be compatible with the installed SDK, rules library, engine and in-package model. Changing the Python ABI, system or architecture requires installing matching versions and re-verifying the native libraries and the model. `scripts/check_environment.py` verifies the actual imports, model meshes, engine UCI and rules library; on failure, resolve the specific dependency first, and do not merely change the version record to claim a pass. Read versions with the actual interpreter's `--version` and `importlib.metadata.version()` and fill them into `config/python_packages.json`; it is not responsible for installing or pinning dependency versions. Switching to a compatible environment does not mean hardware connectivity has been confirmed.

## 2. Fill in the network, endpoint and gripper direction

Fill in the following fields from the on-site network information and keep their sources:

| Field | How to confirm | Where it is passed |
| --- | --- | --- |
| `interface` | Use `ip -brief address` to identify the interface connected to the robot's DDS subnet and verify the connection | Launcher `--interface` |
| `camera_host` | The actual host/IP of the robot image server and network reachability | Launcher `--camera-host`; standalone capture `--host` |
| `endpoint` | Choose a writable local IPC path, or a local loopback TCP endpoint supported by the current service | Pass `--endpoint` explicitly to the launcher and to every robot_client call |
| `gripper_open_sign` | Built-in Dex1 left and right grippers both default to `+1`; override with justification only if the hardware/firmware uses a different joint direction convention | Launcher `--gripper-open-sign`, or `CHESS_GRIPPER_SIGN` |

The built-in Dex1 physical left gripper (motor 31) and physical right gripper (motor 33) both open at positive angles and close at 0; the default allowed motor target range is `[0,5.4]rad`. For example, planning `gripper_open_q=2.3` actually sends `+2.3rad`; the magnitude is a motor angle, not an opening in millimeters. This direction follows the built-in Dex1 default convention; changing the host, Python, network interface or deployment directory does not change the sign, and there is no need to test-open each time or ask the user to confirm again. When `CHESS_GRIPPER_SIGN` is unset or empty, both the launcher and the service use `+1`, and the CLI overrides the environment variable; only when the actual hardware/firmware joint convention differs is a justified `-1` override used, and it is not mirror-flipped for left versus right hands. The service's `get_state.gripper_open_sign` returns the sign actually in effect.

The launch script and service require the network interface and camera server, which can be passed explicitly on the CLI or via the corresponding environment variables in `.env.example`; if these two are missing, no connection is started. The gripper direction has the default above, and configuration JSON does not override launch arguments automatically. The image configuration request port and stream ports must match the on-site server; ImageClient currently defaults to request port 60000, and the service entry point does not expose this port parameter; if the site uses another port, first adapt and verify the image client entry point. After the camera configuration request succeeds, it is written to `config/cam_config_client.yaml` (not under version control); if a local fallback is needed, fill in the real ports and other parameters according to `config/cam_config_client.example.yaml` and save it as that file. The release package does not provide a fallback configuration that can connect directly to an old device.

For example, after the actual values are filled in, set:

```bash
CHESS_INTERFACE='<verified-dds-interface>'
CHESS_CAMERA_HOST='<verified-image-server>'
CHESS_ENDPOINT='<selected-local-service-endpoint>'
CHESS_GRIPPER_SIGN=1
```

## 3. First confirm the physical camera mapping

The current code convention uses the logical key `cam_left_high` for the head image, and the logical keys `cam_left_wrist` and `cam_right_wrist` for the wrist streams. The names do not prove the physical side; it must be confirmed from this session's real-robot images. After obtaining capture authorization for the task, you can obtain the head and both wrist streams with a standalone tool without enabling arm motion:

```bash
"$CHESS_PYTHON" scripts/capture_cameras.py \
  --host "$CHESS_CAMERA_HOST" --cameras cam_left_high cam_left_wrist cam_right_wrist \
  --output-dir "$CHESS_ROOT/runs/site_setup/camera_check"
```

Look at the photos and fill in `config/chess_camera_policy.json.wrist_stream_by_physical_arm.left/right`; the two entries must be two different wrist streams. Fill in `mapping_evidence`, recording the image paths, the basis for confirmation and the time. The `_field_help.example` in the JSON only demonstrates one same-name mapping and must not be taken as a confirmed result. Both entries in the current template are empty, and the planner and service will refuse to use them.

The service caches the policy per process. Fill it in before the first start; an already running process does not automatically load changes on disk, and must be handled in an orderly way once on-site conditions such as idle and empty grippers are met; old tokens become invalid and the preflight must be redone. Actual move planning uses only the head image, grasp/release checkpoints use only the head image plus the physical executing-hand wrist image, and images are read according to the `camera_selection/camera_order` bound to the new token.

## 4. Set up the service and record this session's state

This step operates the robot connection; perform it only when an actual connection is authorized and the site is suitable. First check existing services and decide whether to reuse or start; do not use the launcher to probe the environment, as it cleans up matching old processes. A service that is holding a piece or executing must first be recovered through its original transaction.

`runs/` stores data newly generated in the current environment; nothing needs to be copied from an old environment. After creating a new `runs/…` directory for this configuration/first round, when a start is actually required, pass the confirmed parameters explicitly (the physical executing hand must also be chosen explicitly):

```bash
CHESS_RUN="$CHESS_ROOT/runs/site_setup"
mkdir -p "$CHESS_RUN/tmp"
bash scripts/start_service.sh --foreground \
  --arm '<left-or-right>' --python "$CHESS_PYTHON" \
  --interface "$CHESS_INTERFACE" --camera-host "$CHESS_CAMERA_HOST" \
  --endpoint "$CHESS_ENDPOINT" --gripper-open-sign "$CHESS_GRIPPER_SIGN" \
  --log-dir "$CHESS_RUN/service" --log-file "$CHESS_RUN/service.log"
```

The start keeps a foreground tool session; other tool calls set the required variables again. On the idle service, save `ping/get_state` using the same endpoint, and check the real device, control state, faults, waist pose and physical hand roles:

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" ping \
  --save-response "$CHESS_RUN/tmp/ping.json"
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" get_state \
  --save-response "$CHESS_RUN/tmp/initial_state.json"
```

Following operation manual 0.1.3, lock the waist at zero and re-verify it when the conditions are met; do not treat the zero waist in the template as a measurement. Check the logical arm fields of the left-hand entry point and their mapping to the physical hand according to the current service notes; do not interpret the left-hand state directly with the field meanings of the right-hand entry point.

## 5. Configure on-site geometry, both arms' init and motion parameters

**Source of scene values.** The user may give all or some of the on-site values. Prefer data given by the user that applies to the current scene, first checking units, the base/EE coordinate frames, the robot's waist pose, the tool and the open-gripper state; when there is a conflict, state the specific difference and do not overwrite silently. Even if the user provides no on-site information at all, the model estimates all scene fields itself; when the user gives only some, the model fills in the remaining fields. The model may combine existing observations, robot and tool models, dimension data and reasonable assumptions to complete the estimate, and does not make user-provided values, item-by-item supplementary data or extra confirmation a precondition for setting the scene. Field definitions are in the [scene template](../config/scene.example.json); obtain the basis as follows:

| Field | How the model evaluates it when the user does not provide it |
| --- | --- |
| `surface_z` | Combining the current table/board images, known dimensions, the robot base and waist pose, FK from the same state batch, and available camera/coordinate transform data, estimate the highest Z of obstacle surfaces in the motion area. A table height above the floor provided by the user must be converted into the model base frame; piece top surfaces also must not be taken directly as the board surface. You may actively combine other views, model scale, robot geometry and pose relations to estimate; when direct scale or base localization data is lacking, clearly record the scale/localization assumptions used and the larger estimation error, and do not call a visual inference a metric measurement. |
| `finger_end`, `half_width`, `half_thickness` | Combining the current tool's model/dimension data and images of the real object, estimate the envelope in the model's EE local frame. `half_width` includes this round's planned open-gripper range and must not take only the closed-gripper width; check that the model corresponds to the actual Dex1 and attachments. |
| `uncertainty` | From the errors of surface localization, tool dimensions, coordinate transforms and image estimation, evaluate a non-negative equivalent vertical margin. Record the error ranges used and the basis for combining them; do not shrink the margin to make a path pass. |
| `travel_clearance`, `minimum_clearance` | Choose policy margins based on the on-site surfaces, tool and path needs, and check that they satisfy the current code ranges. You may start the evaluation from the code defaults 0.040m and 0.003m; they are default policy values whose applicability must be checked, not on-site measurements. |
| `xy_tolerance`, `rotation_tolerance` | Check the runtime "same XY/same orientation" decision needs; you may use the code defaults 0.001m and 0.008rad and note the source; they are not grasp precision or visual estimation error. |

The model first uses images, states and data already within the task's scope, and may also actively collect available evidence; when capturing images or reading hardware state is needed, it follows the current task's hardware authorization. The user not providing data is in itself not a reason to wait or refuse to set the scene; the model should combine model geometry, available observations, code defaults and reasonable assumptions to estimate the missing parameters, reflect unknowns in the assumptions and uncertainty, and does not need to request extra confirmation from the user. The estimates are used directly for subsequent setup and the full plan preflight; record which are user-given, model-estimated or code defaults, and do not call it on-site calibration just because the service accepted it.

Save the per-field results in this run's `tmp/scene_assessment.json`, recording value, unit, coordinate frame, source type (user-given/model-estimated/code default), image or state paths, model version, estimation method and error, and applicable conditions. When some fields are given by the user, keep those fields and only fill in the missing parts. When using the highest board-surface Z and `uncertainty`, explain how the margin is included; the lowest point of the model envelope after it changes with the pose is the basis for the clearance calculation, and comparing only EE Z is not enough. The current envelope back end is fixed at −0.040m along EE local X, and the attached model's EE is offset 0.050m along the wrist-yaw local X; this must be checked against the tool and the site.

**Scene setup and verification.** From the results above, generate this run's `tmp/scene_request.json`; the request contains only `scene` and its finite numeric fields; the assessment record is saved separately, and `_field_help`, unfilled nulls or explanatory text are not sent as part of the request. The current [BoardGuard](../src/g1d_chess/robot/board_guard.py) requires `finger_end` to be 0.10–0.30m, `half_width` 0.02–0.09m, `half_thickness`>0, `uncertainty`≥0, `travel_clearance`≥0.030m and `minimum_clearance`≥0.002m; having all fields filled does not mean they fit the current tool or site. When a parameter is outside the implementation range, check the geometry and the applicability of the implementation; do not modify estimates just to pass.

Only when the service is idle and arm control is off, call `set_scene` according to [Section 4 of the operation manual](operation_manual.md#4-create-this-rounds-directory-and-observation) and save the setup receipt; then use `get_state` to check that `scene_guard` matches the request, and record the corresponding assessment file. When an existing scene is compatible with the current environment, prefer reusing it, and do not re-estimate or overwrite it every round; when the board, tool, waist pose or coordinate premises change, re-evaluate, and after the scene changes, run the full preflight again and do not reuse old tokens.

**Both arms' init.** `config/init_state.json` is empty on release and must be filled with measurements of the current device. Determine the standby poses of the physical left and right hands through the authorized initialization flow, and replace the currently applicable record with the same-frame measured joints, FK XYZ/RPY from the same model, gripper and waist pose, time and source. It is actually read by the planner; do not just add new values in config. Check the 3D distance from the new state to each init with the current 10cm rule; within range, do not return home again; do not substitute a configuration change for a real state.

**Motion candidates.** Record in `config/manipulation_profile.json` this session's confirmed source/target orientations, source high/low positions, travel, staging, release height and open-gripper q; check source and target, and the left and right hands, separately. This file is not written into the plan automatically; the actual values must be filled into this round's spec:

| Configuration field | Corresponding location in this round's spec |
| --- | --- |
| source/target orientation and height | `source.z_m/rpy_rad`, `target.z_m/rpy_rad`, estimated independently |
| source high position, transport height | `gate_z_m`, `travel_z_m` |
| Retreat/linking area XY | `staging_xy_m`, according to the actual executing hand |
| Open-gripper target | `gripper_open_q`, shared by all open in the same transaction |
| Round tray target | XY/Z/RPY of `capture.offboard_target`, the same-side tray identifier and on-site basis |

The G1/Dex1 flow candidates/constraints specified by `assets/templates/`, the operation manual, round_plan and the runtime code include board release Z=.139, tray Z=.159, contact detection thresholds and the 5mm lowering rule; they are not calibration of a new device. In particular, the current planner has a fixed check on tray release Z=.159, the close action uses 0, and some contact detection parameters are written in the generator: **if the new site cannot meet these conventions, filling in config alone will not adapt it successfully**; first adjust the affected templates, planner/service constraints and this round's notes accordingly and validate them, then execute motion. Correction counts and execution rules follow round_plan and the current task instructions.

## 6. Set up the new board and error context

Using this session's head image, generate and visually re-check the 90 points according to the [board_position notes](../board_position/README.md) and the [model marking task](../board_position/prompts/mark_board.md). Re-localize after the board/camera/waist pose changes. The dimensions in `config/board_parameters.json` are empty on release and must be filled with measurements of the current physical board; board-surface pixels, piece top surfaces, wrist image pixels and base end-effector coordinates are not interchangeable. This tool does not provide hand-eye calibration.

Fill in `config/xy_error_learning.json.context_id` for this session's compatibility conditions; for example, `g1d_site_A_board_A_tool_A_pose_v1` only demonstrates the naming. You can also fill in `xy_error_context` in this round's spec or pass `--context-id` to the error tool; the current priority is explicit CLI/context argument > spec > config. When there is truly no history journal the first time, do not pass `--previous-journal`; afterwards, keep and carry over compatible journals, and create a new context when conditions change, without directly copying the identity of a history group. The JSONL is not automatic coordinate compensation.

## 7. Enter the first round

Create a new run according to Section 4 of the operation manual, and confirm this game's piece placement, which side we play, the current turn and the FEN/legal moves; a brand-new game counts from our round 1. Fill in the spec with the new state and images, check the new init, scene, physical camera mapping and the complete grasp/release/failure branches, obtain a new preflight token, and then execute. The first round still goes through the source high- and low-position judgments, the target low-position judgment and the final two review items; the visual checks are not skipped because configuration is complete.

`verify_package.py` verifies the release files and links; editing manifest-managed configuration on site produces expected hash changes; it is not a check that "on-site parameters are correct". Keep the original release manifest and a record of local modifications, and do not falsely claim on-site calibration is complete just to make hashes pass. Use `check_environment.py` to check the environment, and new images/states plus the full motion preflight to check the actual runtime conditions. Completing this page only means the preparation process has a basis; the robot's current state follows this session's receipts.
