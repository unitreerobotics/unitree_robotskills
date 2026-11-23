# Script Inventory

English | [中文](round_script_inventory.zh.md)

`bash r1d <subcommand>` dispatches to `scripts/<entry>.py`. Every entry is a 5-line thin wrapper that, through `scripts/_bootstrap.py`, adds `src/` to `sys.path` and then runs the corresponding module with `runpy`; no pip installation is needed. All modules locate the package root with `r1d_chess.paths.ROOT`.

## Subcommands and entries

| Subcommand | Entry | Module | Purpose | Connects to the robot |
|---|---|---|---|---|
| `serve` | `scripts/serve.py` | `robot.serve` | Chess REP service (preflight, token, execution, checkpoints) | Yes |
| `runtime` | `scripts/r1d_runtime.py` | `robot.r1d_runtime` | Bundled robot runtime (`third_party/robot_runtime`): `serve` (the systemd unit runs it), `config`, `status` | Yes |
| `client` | `scripts/robot_client.py` | `robot.robot_client` | Service client: `get_state`, `capture_cameras`, `set_active_arm`, `prepare_*`, `enable_control`, `release_control`, `execute_*`, `unload_arms` and others | Through the service |
| `start` | `scripts/start.py` | `robot.start` | One command: start the Runtime and the service, goinit, handover, verify in position | Yes |
| `goinit` | `scripts/go_init.py` | `robot.go_init` | Return to the reference pose after a restart (no preflight, needs confirmation by a person) | Yes |
| `handover` | `scripts/handover.py` | `robot.handover` | Stop the hold unit and hand over to the chess service | Yes |
| `retract` | `scripts/retract.py` | `robot.retract` | Replay the travelled path in reverse order back to the starting point | Through the service |
| `serial` | `scripts/serial_checkpoint.py` | `robot.serial_checkpoint` | `next`/`submit`: handle checkpoints through local files only | No |
| `gate` | `scripts/gate_submit.py` | `chess.gate_submit` | Generate the evaluation for the stage and immediately run `serial submit` | No |
| `round` | `scripts/round_tools.py` | `chess.round_tools` | `init`/`build`/`tokens`/`report` | No |
| `engine` | `scripts/xiangqi.py` | `chess.xiangqi` | `analyze`/`match`/`validate` (Fairy-Stockfish + pyffish) | No |
| `pose` | `scripts/grip_pose.py` | `chess.grip_pose` | Generate TCP poses from square names; `check` checks a spec | No |
| `candidates` | `scripts/preflight_candidates.py` | `chess.preflight_candidates` | Batch preflight of candidates, no execution | Through the service |
| `review` | `scripts/review_pose_audit.py` | `chess.review_pose_audit` | The three event FK errors | No |
| `history` | `scripts/xy_error_history.py` | `chess.xy_error_history` | `show`/`finish`: error accumulation and named review | No |
| `boardview` | `scripts/board_view.py` | `board.board_view` | Top view, per-square enlarged images, undistorted image | No |
| `fencheck` | `scripts/fen_check.py` | `board.fen_check` | Draw the FEN back onto the top view for checking | No |
| `boardpoints` | `scripts/board_points.py` | `board.board_points` | Measurement of the 90 intersections, can be written back to the geometry | No |
| `boardcal` | `scripts/board_calibrate.py` | `board.board_calibrate` | Capture images, register, refine and write the board geometry | Through the service when capturing images |
| `board` | `scripts/board_registration.py` | `board.board_registration` | Fit the board plane from measured intersections | No |
| — | `scripts/head_board_measure.py` | `board.head_board_measure` | Coarse board measurement (including the board surface height) | No |
| — | `scripts/check_environment.py` | — | Offline environment check | No |
| — | `scripts/install_runtime_service.py` | — | Generate the `r1d-runtime.service` user unit | No |
| — | `scripts/setup_status.py` | — | Whether each configuration item is a reference value or has been set up on this robot | No |

## Setup tools (`setup/`)

Used when bringing up a new robot or a new site; for the procedure see [setup/README.md](../setup/README.md). The scripts are run directly (`python -B setup/<directory>/scripts/<script>.py`) and load `src/` through `setup/_common.py`.

| Directory | Script | Purpose | Moves the robot |
|---|---|---|---|
| `camera-check/` | No script; use `client capture_cameras` | Mapping between camera streams and the physical left and right arms | No |
| `chess-scene-parameters/` | `capture_initial_pose.py` | Record the initial posture and the head, waist and lift references | Read-only |
| | `dex1_envelope.py`, `reach_check.py`, `view_resolution.py` | Gripper dimension check, IK reachability spot check, view resolution | No |
| | `write_scene.py` | Generate the scene in `robot.json` and `board_geometry.json` from the board measurement results | No |
| `head-camera-calibration/` | `stage0_check.py`, `head_calib_record.py`, `head_calib_intrinsics_capture.py` | Read-only check, recording of the calibration video | No |
| | `head_calib_detect.py`, `head_calib_intrinsics_solve.py`, `stereo_fit.py` | Corner detection, fitting of intrinsics and stereo extrinsics | No |
| | `head_calib_capture.py` | Multi-pose head capture, single-pose image capture | Head |
| | `capture_check.py`, `head_calib_solve.py`, `arm_overlay.py` | Capture quality check, extrinsics solving and acceptance, visual inspection | No |
| `joint-tracking/` | `bench.sh`, `arm_tracking_benchmark.py` | Tracking comparison on the Runtime path: snapshot, planning, execution, summary | Both arms |
| | `joint_report.py`, `diagnose.py`, `model_fit.py`, `friction_fit.py`, `ilc_coulomb.py` | Result analysis and fine-tuning of the feedforward parameters | No |
| `arm-calibration/` | `r1_d/calibration/explore.py`, `fit.py`, `heldout.py`, `compare.py` | Exploration with both arms, dynamics fitting, held-out evaluation | Both arms (explore run) |
| | `r1_d/tools/run.py` (`joint`, `hold`, `height`), `unload_arms.py` | Joint test, waist and head calibration, height test, unloading the arms | Yes |
| | `scripts/install_results.py` | Install the fitting results into the chess configuration | No |

## Implementation modules (`src/r1d_chess/`)

| Subpackage | Module | Content |
|---|---|---|
| `robot/` | `serve` | `R1DService`: the R1-D constructor, inherits the generic transaction engine |
| | `internal_service` | Generic transaction engine `RobotService`: preflight, token, execution, checkpoints, empty-grasp retry, return and unload |
| | `motion_harness`, `fast_correction` | Cartesian interpolation, IK paths, in-execution correction paths |
| | `board_guard` | Table and tool envelope check |
| | `release_alignment`, `grasp_vision` | Validation and correction of the release gate evaluation, grasp determination |
| | `checkpoint_runner`, `serial_checkpoint` | Checkpoint file protocol and submission |
| | `arm_roles` | Role/joint mapping when the left arm acts |
| | `camera_policy` | Planning/execution camera selection |
| | `pose_audit`, `async_event_log` | Event snapshots and JSONL log |
| | `r1d_transport`, `r1d_runtime` | Runtime IPC session, DDS subscription, simulated transport |
| | `r1d_kinematics`, `r1d_urdf` | URDF FK/IK (mainly damped least squares, scipy fallback) |
| | `r1d_gravity`, `r1d_torque_ff`, `r1d_hold_ff` | Arm gravity and torque feedforward, waist and head hold feedforward |
| | `r1d_config` | Profile loading, calibration and model hash validation |
| | `go_init`, `handover`, `start`, `retract`, `robot_client` | See the table above |
| `chess/` | `round_tools`, `grip_pose`, `xiangqi`, `move_constraints` | This round's plan, poses, chess rules and engine, arm selection policy |
| | `gate_submit`, `preflight_candidates`, `review_pose_audit`, `xy_error_history` | Gate submission, candidate preflight, review |
| `cameras/` | `r1d_cameras` | TeleImager image fetching (head stereo cropped to one eye, wrist streams) |
| `board/` | `board_view`, `fen_check`, `board_points`, `board_register`, `board_calibrate`, `board_registration`, `head_board_measure` | Board-reading aid images and board geometry measurement |

## Runtime files (`runs/<this round>/`)

| Path | Produced by |
|---|---|
| `round_spec.json`, `resume.json`, `tmp/`, `images/`, `docs/` | `round init` |
| `board.json` | Written by the agent |
| `tmp/request.json`, `tmp/plan_summary.json` | `round build` |
| `tmp/prepared.json` | `client prepare_*` |
| `tmp/execute_request.json`, `tmp/stage_N_prepared.json` | `round tokens` |
| `tmp/checkpoints/…`, `tmp/eval_*.json` | The service, `gate` |
| `tmp/result.json` | `client execute_*` |
| `tmp/visual_review.json`, `docs/pose_review.json`, `docs/第N轮_….json` (round N, Chinese move notation, Beijing time), `docs/execution_xy_errors.jsonl` | The agent, `review`, `history finish` |

The service log is in `runs/<time>_service/events.jsonl`. Both the service and the `round` tools reject run directories outside `runs/`.
