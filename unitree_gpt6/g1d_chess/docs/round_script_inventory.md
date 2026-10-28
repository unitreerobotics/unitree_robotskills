# Scripts and Modules for a Generic Move Round

English | [中文](round_script_inventory.zh.md)

Corresponds to the [skill](../SKILL.md), the [Operation Manual](operation_manual.md) and [round_plan](../prompts/round_plan.md). All paths are relative to the repository root. `scripts/` holds the actual callable entry points, and `src/g1d_chess/` is the implementation those entry points load; the two do not maintain duplicate copies of the same business code. Python entry points locate the src of the current checkout via `_bootstrap.py`; no separate working copy needs to be installed.

## Entry points used directly

| Entry point | Purpose and input/output |
| --- | --- |
| `scripts/start_service.sh` | `--status` only queries the process; when startup is truly needed, choose `--arm left/right` and start the service with the on-site NIC, cameras and gripper orientation. Startup cleans up matching old services and is not used for fault recovery. |
| `scripts/robot_client.py` | Single service-request CLI: ping, get_state, image capture, waist hold, scene, full pre-check, enable and execute, and on-demand release; returns real receipts. |
| `scripts/round_tools.py` | Right-hand init/build/tokens: creates the run and templates, reads this round's spec / new observations, generates the full branches and submission files. The init command does not move the robot. |
| `scripts/round_tools_left.py` | Corresponding left-hand entry point; requires new observations from the left service and binds the physical acting hand and expected_active_arm. |
| `scripts/move_constraints.py` | Checks square names and the source preferred hand; generates searchmoves from an externally confirmed list of legal UCI moves; does not recognize images, does not generate rule-legal moves, and does not do IK. |
| `scripts/xy_error_history.py` | Before planning, show reads the reference for the same hand's compatible context; in review, finish merges real FK / visual conclusions with the JSONL, without automatically compensating coordinates. |
| `scripts/serial_checkpoint.py` | next/submit during execution; submits the current checkpoint evaluation, or a user-specified pre-grasp failure. Must not wait until execute returns to handle it. |
| `scripts/review_pose_audit.py` | After the end, computes three kinds of FK error offline for the two event snapshots in the real result. |

## On-demand entry points and environment checks

| Entry point | When to use |
| --- | --- |
| `scripts/mark_board.py` | Before planning, init/build/query the 90 points from a new image; does not capture images and does not output robot XY. |
| `scripts/capture_cameras.py` | Independent image capture during on-site setup or camera troubleshooting; for a normal single round use robot_client, and do not interrupt the transaction with independent requests during execute. |
| `scripts/start_left_service.sh` | Convenience wrapper for start_service --arm left. |
| `scripts/internal_service.py`, `scripts/internal_service_left.py` | Actual process entry points called by the launcher; manual invocation also requires the on-site environment parameters. |
| `scripts/check_environment.py` | Offline check of installed dependencies, models, engine and rules library; does not connect to hardware. |
| `scripts/verify_package.py` | Offline check of release files, syntax, skill/document links and the local import closure. |

## Existing implementation modules

| Directory | Files and roles |
| --- | --- |
| `src/g1d_chess/chess/` | round_tools, round_tools_left, move_constraints, xy_error_history, review_pose_audit: plan building and post-hoc analysis. |
| `src/g1d_chess/robot/` | internal_service and left entry point, dex1_internal, arm_roles: transactions, DDS, dual-arm roles and waist hold. |
| Same as above | g1_model, motion_harness, board_guard: model, FK/IK, trajectories and scene checks. |
| Same as above | fast_correction, release_alignment, grasp_vision: grasp correction, release gate and the compatibility vision config that must still be shipped. |
| Same as above | serial_checkpoint, checkpoint_runner, camera_policy: serial evaluation/schema and physical camera mapping; the current flow does not start the old background worker in checkpoint_runner. |
| Same as above | pose_audit, async_event_log, robot_client: raw snapshots, non-blocking logs and the service client. |
| `src/g1d_chess/cameras/` | imageclient, capture_cameras: camera connection and on-demand capture; local config is in config, and successful requests update the git-ignored fallback file. |
| `src/g1d_chess/board/` | mark_board: image point fitting, validation and annotation. |
| `src/g1d_chess/paths.py` | Locates the repository root from the current source location; each subdirectory's __init__.py is only for Python package import. |

The complete file list and local import relations are in the [JSON index](round_script_inventory.json): 46 entry points, modules and package support files in total (including package init files). After moving directories, run the tests and package check; do not treat passing imports as real-robot verification.

## Call order and data in one round

1. Check state and waist hold against the actual service and current authorization; check each arm's distance from init and the physical acting hand.
2. The corresponding hand's round_tools init creates a unique runs directory; capture a new head image and the same-batch state, and confirm the complete position and Black's actual move.
3. Match this game's forecast, calling CHESS_ENGINE and pyffish if necessary; if needed, first complete point marking on the current image with mark_board.
4. Read the same hand's context reference, independently estimate source/target XYZ/RPY and neighbors, and fill in the spec, scene and on-site basis.
5. build the full branches; prepare_transaction for an ordinary move, prepare_sequence joining both segments for a capture; tokens generates one execute submission package and the prepared data for each stage.
6. After enable, one execute; in another session do next / look at this step's two images / submit; after the first capture segment releases, continue directly into the second segment, without an intermediate pre-check or return to init.
7. After completion, only capture a new head image to judge the placement, compute the three kinds of FK error for the two events, and use finish to merge them into this round's docs and the shared JSONL. A normal ending keeps the waist hold.

Before motion, have ready `config/init_state.json`, the camera policy / connection parameters, scene, current state and images, spec, asset templates, the new pre-check token and the evaluation structure. Run results go only into this round's `runs/`; no historical positions, coordinates or tokens are preset. The flow candidate parameters in templates must be checked on site. The main session is responsible for image judgment and engine/forecast orchestration; the repository has no fully automatic single-round controller.

XY correction counts, graspable-is-enough and release rules follow round_plan; for specific differences between the current implementation and the operating requirements see [runtime](runtime.md#implementation-verification-boundary). Normal review does not rebuild points, run tests or shut down the service.
