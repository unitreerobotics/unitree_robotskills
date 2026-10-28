---
name: g1d-gomoku
description: Operate a Unitree G1-D to play Gomoku autonomously as Black. The agent reads four camera views, recognizes the board, selects a move, and performs closed-loop pickup, placement, and return; the user starts a round and confirms before the gripper releases.
---

# G1-D autonomous Gomoku

English | [中文](SKILL.zh.md)

Start a round when the user says “next move” or “execute the next round.” View images, recognize the board, select a move, and plan actions yourself. Do not require the user to provide board coordinates, a board matrix, pixels, or end-effector poses. Coordinates are internal decision/control records only. Do not use a fixed game sequence or a previous round's landing point.

First read the [autonomous execution flow](docs/autonomous_play.md) and [operating rules](docs/operating_guide.md). Read [hardware integration](docs/hardware_integration.md) before the first connection, consult [manipulation methods](docs/manipulation.md) when localization or grasping needs improvement, and follow the [experience data contract](docs/experience.md) for reading/writing experience. Complete the task within current authorization; writing or inspecting code does not authorize operating a physical robot.

Before planning pickup, arm lifting, and alignment in each round, read the [zigzag arm-posture prior](docs/manipulation.md#zigzag-arm-shape-prior-for-grasp-alignment). Use its shoulder/elbow/wrist coordination and gripper-orientation guidance to check the current arm posture, joint margins along the whole segment, and continuity during descent and the short lift. Do not keep raising or flipping the wrist merely to satisfy TCP constraints. Use no default experience on the first run; later records are only candidates for comparison, not directly executable motions or replacements for the live starting state. The current solver does not automatically guarantee this arm posture.

Before running, set `GOMOKU_WORKSPACE` according to the [workspace contract](ARCHITECTURE.md#4-the-gpt6-astra-standalone-workspace). It defaults to `gomoku-workspace/` under the repository root. Start there and store temporary code, private configuration, images, plans, and logs there. Write only to the workspace during operation; keep source, project configuration, and documentation read-only. The deployer initializes the workspace; do not depend on directories or files from the developer's machine.

## Closed-loop round

1. Use the site's private configuration and keep exactly one persistent control publisher. Capture four original camera views and feedback while at the rest pose. Create a separate directory for this move with `src.main play`.
2. Use your own image-viewing capability to inspect the actual images, cross-check the left and right head views, and identify the 19×19 black/white board and grid orientation. Write recognition results to this round's `board_observation.json`; do not write a target or assume an empty board. If uncertain, recapture and recheck yourself rather than asking the user to mark points.
3. Run `src.main decide` so the engine selects a currently legal empty intersection and saves the board state and single-intersection annotation. If recognition is wrong, create a new round and observe again before motion; do not manually override the engine target.
4. Call motion tools using current images and feedback, and pick one black piece from the tabletop supply area with the right arm. At the low position, check both fingers' inner edges, the piece center, and the contact line. Close slowly, then make a short lift to confirm that exactly one piece follows the gripper and its original location is empty before transferring at the high position. Perform routine fine adjustments autonomously.
5. Use neighboring piece rows/columns and two grid lines to align with the same engine-selected intersection; hold XY while descending in segments. Prepare the complete return path: lift vertically from placement, reverse the high-position segments actually executed and clearance-checked during this round, then connect to the site's rest-pose reference. Rebuild the connection if final XY changes.
6. Save the low-position visual check and run `prepare-release`. Explain the landing point, release, and return, then wait for the user's gripper-release confirmation. Only after actually receiving confirmation pass their exact words to `release --confirmation`; do not generate approval yourself.
7. Release within the same healthy control session. Save the four camera views once, without viewing or analyzing them, and immediately perform a continuous lift, exit, and return. Retain necessary feedback and health checks; do not plan on the fly after placement.
8. Report the return result from measured feedback. Do not claim post-release visual verification of the exact intersection. Wait for the opponent and the user's next-round instruction.

## Execution constraints

Maintain the startup waist angles. Do not move pieces already on the board, take pieces from a box, or collide with the table, board, or neighboring pieces. Historical heights, pixels, and routes are priors only; they do not replace site calibration and full-path checks.

Minimize waiting while the arm is raised: assess the board and do the main preparation at the rest pose, approach the area above the target briskly, then make a small number of evidence-based fine adjustments. After lost communication, a protective stop, or a control-state change, inspect feedback before rebuilding the current plan; do not replay old targets. Do not report unfinished actions as successful.

Explain the target and actual result to the user. Keep reference-consultation details in the round records; do not proactively record video. The host agent running this skill handles four-view image understanding and motion planning; the Python stage tools do not perform those tasks on its behalf.

## Experience learning

Initialize an empty experience store in the workspace on the first run. Do not use dimensions, XY positions, pickup/placement heights, joint vectors, gripper openings, or default rest poses from an old scene. Capture the rest-pose reference read-only from the current device. Solve from the current four camera views, live feedback, model, and site calibration. Do not fill unknown values with zero or turn guesses into experience.

Create a binding for the robot, scene, configuration, rest-pose reference, calibration, and model, then read matching records. Rebuild the binding after configuration, scene, or calibration changes. If no records match, treat it as a first run. Experience only assists candidate solving; it cannot directly replay trajectories or reuse authorization.

Before `prepare-release`, create this round's `release_parameters.json`, recording the currently solved release opening and its evidence, and review it with the return path. After user confirmation, release and retreat immediately rather than collecting experience on the fly.

After return has passed verification, prepare `experience_candidate.json`: record targets and measured values separately, link this round's images, feedback, and results, declare each stage's success according to actual evidence, and submit it to the workspace with `src.experience record`. Keep failed, unverified, or planned-but-unexecuted data in runtime logs, not successful experience. `returned` does not mean visual verification of placement; do not use it to record exact landing success. Do not modify project templates, configuration, documentation, or model files.
