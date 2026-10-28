# Agent Autonomously Executes One Move

English | [中文](autonomous_play.zh.md)

This page is for the host agent to operate. The user only needs to say "next move"; the user is not required to write any stage files. All files below are internal interfaces that the agent saves itself after observing and computing.

## 1. Initialization and continuous control

Copy the hardware configuration template as described in the README, fill in the current network, and capture the rest-pose reference read-only from the current device. Follow [Hardware integration](hardware_integration.md) to run the offline pre-check, read-only status and camera checks. The experience library starts empty; dimensions, heights and arm configuration are solved from the current scene and recorded after success according to [Experience data conventions](experience.md). First verify control ownership and the entire motion path; do not copy old round XY directly, and do not treat records as real-time feedback.

Start the continuous publisher only once, running in a separate long-lived process:

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main --config config.local.yaml \
  control --root game --execute owner
```

`game` is relative to `runtime.directory`; afterwards use its absolute path as `--owner-root`. The status is in `game/session/status.json`; use `control --root game status` for read-only diagnostics. If not in the rest pose, the agent first plans and verifies the retreat corridor, then calls `control --root ROUND --owner-root GAME --execute reset reset_start --duration D --review REVIEW`; this command interpolates from the measured start point to the local reference and has no obstacle-avoidance capability. A low position or a stone-holding state must not return home directly through obstacles; the agent must first plan a lift and exit.

After startup keep the same publisher; do not repeatedly switch control processes while observing and thinking. The current configuration, feedback and SDK fault checks are retained by the tool; on a protective exit, stop subsequent tasks and determine the state.

## 2. Observe a new round

Choose a directory that does not yet exist for this move, and run in the rest pose:

```bash
python -m unitree_gpt6.g1d_gomoku.src.main play \
  --config config.local.yaml --owner-root GAME_ABSOLUTE --round ROUND_ABSOLUTE
```

The workflow first checks the rest-pose feedback, restores four-way image capture, saves the `board_capture/` raw images and state plus `capture.json`, and enters `awaiting_board_review`. It returns only a status hint; raw images and detailed paths are read from this round's files.

The host agent opens the four raw images with an image-viewing tool, cross-checks the complete grid, black and white stones and the round with the left and right head images, and uses the wrist images to help check the stone supply. The left head image is authoritative: rows go down, columns go right, 19×19 both numbered from 1, and the outer frame does not count as part of the grid. If something is unclear, recapture in the rest pose and create a new round directory; do not guess empty points or ask the user to mark them.

## 3. Recognition result and automatic move selection

The agent writes `ROUND/board_observation.json`, containing exactly the following fields:

| Field | Content |
| --- | --- |
| `capture_id` | Read from this round's `capture.json` |
| `recognition_source` | `agent_visual_review` |
| `board` | Complete 19-row × 19-column integer matrix; 0 empty, 1 black, 2 white |
| `views_agree` | true when the left and right head images have been cross-checked |
| `uncertain_intersections` | Empty list after review is complete; if doubts remain, do not continue |
| `grid_corners_px` | Pixels of the four outermost **grid intersections** in the left head raw image, in order top-left, top-right, bottom-right, bottom-left; not the board's outer frame |
| `notes` | The agent's recognition basis, grid orientation and cross-check conclusion |

These are visual measurement results; do not add target or suggested landing-point fields. Do not copy the matrix from an old game. The engine requires that it be Black's turn (equal numbers of black and white stones) and that nobody has already made five. If recognition conflicts, look at the images again first.

```bash
python -m unitree_gpt6.g1d_gomoku.src.main decide --round ROUND_ABSOLUTE
```

The engine selects the move automatically: making five with this move has priority, then handling White's direct five-in-a-row threat, then comparing continuous and gapped shapes. When multiple threats cannot be defended simultaneously this is recorded explicitly, without fabricating a forced win. The result is written to `position_and_move.json`; a copy of the raw image circles the single intersection using an SVG-embedded image, and the state enters `target_selected`. The projection is used only for image annotation; **do not treat it as robot calibration**.

## 4. Grasp, transfer and low-position alignment

Before planning, first read [Zigzag arm-shape prior for grasp alignment](manipulation.md#zigzag-arm-shape-prior-for-grasp-alignment); use the current feedback to verify the shoulder/elbow/wrist branch and the limit margins for the subsequent descent and short lift. There are no posture defaults; even matching successful experience cannot be used directly as a motion command.

The host agent reads the row/column and annotation selected by the engine, and estimates the target pose using the current four-way images, TCP/joint feedback and on-site calibration. Before moving, check the entire path and the held-stone offset, and submit stages with the low-level `control` tool. The parameters below are computed by the agent; the user does not fill them in:

```text
control --root ROUND --owner-root GAME --execute move STAGE
        --pose X Y Z ROLL PITCH YAW --duration D --review REVIEW --quiet
control --root ROUND --owner-root GAME --execute grip STAGE
        --gripper OPENING --review REVIEW --quiet
```

The full invocation prefix is always `python -m unitree_gpt6.g1d_gomoku.src.hardware.main --config CONFIG`. Use a different name for each stage; the actual paths are recorded in this round's directory. End-effector poses are in pelvis, meters and radians. The tool solves IK from the current feedback and checks speed, joints, left-arm hold and plan connection; `--review` records the agent's check of this round's images and the clearance of the whole segment; numerical sampling by itself is not collision detection.

Tabletop stone picking: go from a safe high position to nearby, correct at the preview position, at the low position verify the inner edges of both fingers / stone center / contact line, then keep the arm fixed and close slowly. On the short lift verify that a single stone follows the gripper, the original spot is empty and neighboring stones have not moved; on failure correct based on the current image. After reaching the board at a high position, use the right wrist image with the rows/columns of neighboring stones and two grid lines to confirm the same empty intersection; the main XY correction is completed at the high position; descend vertically in segments. Do not judge the landing point solely from the image center, the projection of the stone's top surface or the TCP error.

## 5. Prepare the return before confirmation

After the final low-position XY is determined, generate the complete return path from this round's outbound route and save it fixed as `ROUND/return_plan.json`:

```text
control --root ROUND --owner-root GAME prepare_return return
        --pose LIFT_X LIFT_Y LIFT_Z ROLL PITCH YAW
        --outbound-plan THIS_ROUND_HIGH_PLAN --review REVIEW --quiet
```

`--pose` is the vertical lift target that keeps the current XY and orientation; `--outbound-plan` must be a high-position plan from this round that has been completed and checked for clearance; alternatively use a reviewed `--via-pose`. The tool reverse-connects the high-position outbound route and joins the local rest-pose reference. If the final target changes, rebuild the corresponding connection and verify the complete return path, all before user confirmation.

Based on the current images and gripper feedback, the agent computes the release opening and first writes `release_parameters.json`, with fields `right_gripper_rad` (a finite radian value within the configured gripper range) and `review` (a non-empty basis for the solution and checks). This file is bound together with the return path to the release confirmation; if it changes, the confirmation must be prepared again.

After viewing the four-way `after/` raw images of the last low-position stage, the agent writes `alignment_observation.json`:

| Field | Content |
| --- | --- |
| `decision_sha256` | SHA256 of this round's `position_and_move.json` |
| `matches_selected_intersection` | true when verified to match the engine's selected point |
| `target_empty_before_descent` | true when the target intersection has been verified empty |
| `single_black_stone_held` | true when images such as the short lift have verified that a single black stone follows the gripper |
| `grid_neighbor_review` | Actual visual conclusion about neighboring stone rows/columns, two grid lines and the held-stone offset |
| `stage_directory` | Absolute path of this round's last low-position alignment stage directory |

```bash
python -m unitree_gpt6.g1d_gomoku.src.main prepare-release \
  --config config.local.yaml --owner-root GAME_ABSOLUTE --round ROUND_ABSOLUTE
```

The workflow verifies that the low-position stage, images, current holding pose, target and return path are consistent, records `release_checkpoint.json`, and enters `awaiting_release_confirmation`. Now explain to the user the actual landing point, the release and the continuous return, and wait for confirmation; the stone picking and fine-tuning process does not require asking step by step.

## 6. Release and continuous retreat after confirmation

Only after the user actually confirms, record their exact words and execute immediately:

```text
python -m unitree_gpt6.g1d_gomoku.src.main release
       --config CONFIG --owner-root GAME --round ROUND
       --confirmation USER_ACTUAL_CONFIRMATION
```

No more ad hoc planning, no switching the publisher. The flow verifies that the saved plan is unchanged, consumes this round's confirmation once, and the low level, in the same session, slowly opens the gripper, saves the four-way images only once, and continuously lifts vertically / exits / returns home. During this, do not view or analyze post-placement images and do not capture again; runtime health checks continue.

The client waits for the full retreat to finish; `returned` additionally requires that the measured right-arm joints be within the rest-pose tolerance. `physical_placement_verified: false` indicates there is no final intersection visual acceptance. When `execution_failed` or `return_unverified` appears, check the current state; do not run release again to replay. After one move ends, wait for the opponent and the user's next-round instruction; a new round uses a new directory.

## Save experience after success

After returning home, the host agent submits this round's measured data according to [Experience data conventions](experience.md). Writing happens in the resting state and must not prolong stone holding or low-position waiting.
