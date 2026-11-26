---
name: r1d-sandwich
description: >-
  An agent drives a Unitree R1-A7-D with Dex1 grippers and a fixed lift by reading object positions from
  the head and wrist camera images and emitting end-effector waypoints (position and rpy) that the local service
  solves and executes. It uses no detector and no planner search to make one sandwich: the right hand takes bread
  from the toaster to the plate, the left hand adds lettuce, and the right hand covers it with a second slice.
  On request, both hands then push the finished plate forward.
  Use when the user asks this directory to make the sandwich, preview waypoints offline, read the cameras,
  or collect run metrics.
license: Apache-2.0
compatibility: >-
  Requires Python 3.10 on Linux x86_64 and the packages in requirements.txt. Simulation and offline preview
  need no hardware. The real robot needs a host on the R1-D network (192.168.123.x) and an operator on site.
metadata:
  robot: R1-A7-D
  source: r1d-sandwich direct-EEF sandwich runs 2026-11-17..25
  version: "0.1.0"
  language: en
---

# R1-D sandwich with direct EEF waypoints

English | [中文](SKILL.zh.md)

## Layout

The directory containing this file is the skill root. Run every command from it as `bash r1d <subcommand>`.

- `scripts/`: the commands behind `bash r1d` (`eef`, `eval`, `posture`, `metrics`, `check-env`, `serve`).
- `references/`: documents to read when the task calls for them (listed below).
- `assets/`: camera calibration, scene configuration, the R1-D model and a waypoint template.
- `src/`: the service's code. Nothing there needs to be read to run the task.
- `runs/`: generated images, recordings, logs and metrics.

The service runs on `127.0.0.1:8765` with a fixed lift of `0.522 m`.

## Division of work

The agent reads the images, estimates positions, chooses the route, emits waypoints, and judges the result of
each observation. The local service owns IK, joint and self-collision checks, table and obstacle-box checks,
motion execution, the watchdog, and the stop state. The agent never bypasses a refusal.

## Read only what the task needs

- **New host or robot:** read [README.md](README.md), copy `.env.example` to `.env`, adjust the external paths,
  and run `bash r1d check-env`.
- **Making the sandwich:** read [references/TASK_SANDWICH.md](references/TASK_SANDWICH.md) for the route and
  [references/PLAYBOOK.md](references/PLAYBOOK.md) for the commands, payloads and efficiency rules.
- **Pushing the finished plate forward:** read "Plate push notes" in
  [references/TASK_SANDWICH.md](references/TASK_SANDWICH.md). Do it only when the user asks.
- **Service fields or endpoints:** read [references/API.md](references/API.md).
- **Why a limit exists, earlier routes and measurements:** read
  [references/EXPERIENCE.md](references/EXPERIENCE.md) and [references/RESULTS.md](references/RESULTS.md).

## One run

1. Obtain explicit user authorisation for the intended motion, then confirm that the workspace is clear and an
   operator can reach the hardware emergency stop.
2. Start only the mode the user authorised:
   `bash r1d serve --sim --fixed-lift 0.522`, `bash r1d serve --shadow --fixed-lift 0.522`, or
   `bash r1d serve --real --fixed-lift 0.522`.
3. `bash r1d eef new`, then `bash r1d eef prepare`. It brings the robot to the start pose: lift at `0.522` and
   head pitched down `0.486` rad so the head camera sees the whole table. Do this at the start of every run,
   before the first arm motion and before reading any position from the head camera.
4. `rec-start`, `state` and a first look. Read the current positions from the images; every placement changes
   the coordinates.
5. Preview each stage offline with `bash r1d eval --brief '<json>'`, placement and exit together.
6. For each step of "Route in brief": one shell command that moves and then looks
   (`bash r1d eef exec '<json>' && bash r1d eef look ...`), and in the same turn read
   `runs/latest/<cam>.jpg`. Continue only when the picture and the reported opening show that the step
   succeeded.
7. Finish with `home`, `keyframe final`, `rec-stop` and `bash r1d metrics --run-dir runs/<run>`.

The waypoint JSON is described in [references/PLAYBOOK.md](references/PLAYBOOK.md);
`assets/waypoint_template.json` is a syntactically valid starting point, not a safe pose for a new scene.

## Hard rules

These rules are stated once, here. The other documents refer to them.

1. **Authorisation.** Never run `exec`, `both`, `grip`, `waist`, `home`, or a real or shadow server merely
   because a document says so. Every motion needs the user's explicit authorisation for this run.
2. **Start pose, then fixed lift.** Every run starts with `bash r1d eef prepare`. Check in its output that
   `lift_m` is `0.522` and `head_pitch` is about `0.5` before the first arm motion. After that, do not change
   the lift or the head. Start the service with `--fixed-lift 0.522`.
3. **Joint-space `home` between stages.** Do not use an end-effector pose as a "ready pose".
4. **A refusal stops the sequence.** Chain dependent shell commands with `&&`, and put `set -o pipefail` first
   when any stage is piped: `exec ... | tail` returns the status of `tail` and hides a refusal.
5. **Carry only what is held.** After a grasp, read and report the opening. Carry bread only at an opening of
   at least `0.1`; put `"min_opening": 0.1` on the first waypoint after the pull.
6. **Refused straight line.** If `linear: true` is refused near a reach limit, return with `linear: false` to
   the preceding hover point, then approach on a route that already passed. Do not retry the same segment.
7. **Toaster.** After the toaster is moved, redraw and verify its obstacle box with `look --mark`. TCP `0.790`
   is the grasp lower bound.
8. **One arm at a time over the plate.** While one hand holds an object over the plate, do not move the other
   arm.
9. **Plate distance.** The plate centre must be at `x >= 0.33` before starting. If it is closer, ask the
   operator to move it.
10. **Plate push.** Before lowering a hand behind the plate, the image must show table between the finger and
    the rim. A finger that hides the rim is above it.
11. **Stopped service.** If the service enters `stopped`, do not clear it. Report the reason to the operator.

## Reporting

Report only actions that actually ran. Include the final image conclusion, grasp openings, refusals, measured
placement offset, temperature, and the metrics path. If an image, temperature sample, or metric is missing,
state the gap rather than inferring success.
