# R1-7a marker sorting

English | [中文](README.zh.md)

The agent workflow is in [SKILL.md](SKILL.md).

An R1-7a arm with a Dex1-1 gripper and a wrist RGB camera puts markers scattered on a desk back into their numbered slots in a marker box (a 4×6 box in this example). The agent reads the wrist camera images and calls the tool scripts here, step by step, to locate the box, find the pens, grasp, insert and verify.

This skill is the method worked out on one real robot, published as a reference for other agents and developers; the conclusions and lessons from the experiments, failures included, are written into SKILL.

## Hardware

| Part | Notes |
| --- | --- |
| Arm | R1-7a |
| Gripper | Dex1-1 |
| Camera | Wrist USB RGB camera (JR0002), 1280×720 MJPEG, about 109° horizontal field of view |
| Scene | 4×6 marker box (slot pitch about 17.8 mm, with a grid of square holes inside); dark or white desk |

## Before you start

You need three things. For joint control you can start from the official SDK; this repository provides tools for IK and camera calibration.

**1. Joint control (bring your own).** The scripts reach the robot only through four functions. Replace them with your controller's interface:

| Function | Requirement |
| --- | --- |
| `rt.state()` | Returns a dict: `q` (7 measured joint angles, rad), `dq`, `tau`, `command_q` (last joint target sent), `armed`, `fault` (`None` when healthy), `busy` (trajectory running), and `gripper` with `q`, `dq`, `tau`, `q_min`, `q_max`, `opening_fraction` (0 closed, 1 fully open) |
| `rt.move(q, speed)` | Sends a joint target, waits until the arm is really still, returns the measured joint angles; raises on fault or timeout |
| `rt.gripper(opening)` | Moves the gripper to `opening` (0–1) of its travel, waits for the motor to settle, returns `state()['gripper']` |
| `grab.frame()` | Returns a wrist camera JPEG newer than the moment of the call |

The implementation in the repository is an HTTP client for the author's own control program (`127.0.0.1:8000`) and is only an example. A few things affect the results:

- The author's control is joint PD (Kp 50, Kd 5) plus URDF gravity feedforward with no integral term; steady-state error is 0.01–0.025 rad, and the scripts compensate with measured angles and an outer loop (see SKILL). The gravity feedforward includes the end-effector payload (Dex1-1 plus camera, 0.62 kg, centre of mass at `[0, -0.09, 0]` in Link7); without it J4 sags 0.038 rad.
- The gripper is position-controlled, so grip force comes from commanding a smaller opening than the pen allows. The close commands in SKILL (0.08, 0.13) and the opening and torque thresholds are measured on a Dex1-1 under this control; measure them again for a different gripper or control scheme.
- Stopping or disconnecting releases torque and the arm drops; the scripts never call either.

**2. IK (provided here).** `src/ik/kinematics.py` reads the right-arm model from `urdf/URDF_R1-7A.tar.gz` in the official [r1-7a_sdk](https://github.com/unitreerobotics/r1-7a_sdk) and provides FK, IK and the gravity term. The world frame is at the centre of the underside of the base plate (`src/ik/frames.py`). `src/robot/rt.py` adds the calibrated joint zero offsets, multi-seed IK (`ik_all`) and clearance checks on top. The `MODEL = …` line in `rt.py` points at the URDF; change it to where your SDK is.

**3. Camera calibration (provided here).** All scripts rely on `src/calib.json`, which holds the camera intrinsics, the hand-eye transform, the joint zero offsets and the desk plane.

`config/calib.example.json` is the calibration of the author's R1-7a and can be used directly as an empirical starting point: copy it to `src/calib.json`, then photograph the board for a few frames with `calibration.check` and look at the pixel-to-desk error. Within 1 cm you can start; beyond that, run a full calibration. How well each part carries over differs:

| Content | Depends on | Reusing the example values |
| --- | --- | --- |
| `K`, `dist` (camera intrinsics, distortion) | Camera model | Mostly usable with the same camera (JR0002) |
| `link7_from_camera` (hand-eye) | How the camera bracket is mounted | Mostly usable with the same bracket |
| `joint_zero_offset` (joint zero offsets) | Each arm's encoder zeros | Differs per arm; better recalibrate |
| `world_from_board`, `board_top_plane` (desk plane) | Desk height and tilt relative to the base | Differs per site; fingertip height when grasping depends on it, so recalibrate whenever the desk or the base position changes |

A full calibration solves all of these at once; redo it whenever the camera, the gripper mount or the resolution changes. Use a chessboard with 12×9 squares of 30 mm; for a different board, change `PATTERN, SQUARE` (inner corners and square size) in `src/robot/rt.py`. Copy `config/calib.example.json` to `src/calib.json` as a starting guess, lay the board flat on the desk in front of the arm, open the gripper, raise the arm to where it sees the board, and run in `src/`:

```bash
python3 -m calibration.plan calib.json 28 2     # plan 28 look-down poses that see the board; no motion
python3 -m calibration.capture 0 cal2           # at each pose, wait until still, photograph, record measured joints
python3 -m calibration.solve calib.json $(ls cal2/*.jpg | sed 's/.jpg//')
python3 -m calibration.check <frames not used in the calibration>    # pixel-to-desk error
```

Results on this device: 0.52 px reprojection, pixel-to-desk RMS 3.6 mm (about 1 cm worst case). Joint zero offsets reach 0.06 rad; without correcting them the desk localization error doubles. OpenCV's partial chessboard detection skips rows and links squares along diagonals with this wide-angle camera, so `rt.corners()` does its own saddle-point detection and square-by-square growth.

Requirements: Python 3.10+, `pip install -r requirements.txt` (NumPy, SciPy, OpenCV).

## Using it through the skill

Give this directory to the agent as a skill (SKILL.md is the entry point). Once the three things above are ready, the agent runs `python3 -m sorting.sort RUN <step>` in `src/` one step at a time, looking at the wrist image before deciding the next step. `RUN` is the directory for the run; images, fit results and the timestamped log `events.jsonl` are stored there. The agent does not start or restart your control program and never calls a stop that releases torque.

## Layout

| Path | Contents |
| --- | --- |
| [SKILL.md](SKILL.md), [SKILL.zh.md](SKILL.zh.md) | Agent workflow: procedure, grasp choice by how each pen lies, measured numbers |
| [docs/](docs/) | Read as needed: white desk, control and perception notes (with the reasons behind each practice) |
| `config/calib.example.json` | Calibration of the author's R1-7a: empirical values and calibration seed |
| `src/` | Tool scripts; see the table below |

Run every tool from `src/` as `python3 -m <dir>.<tool>`.

| Directory in `src/` | Files | Purpose |
| --- | --- | --- |
| `robot/` | `rt.py` | Robot interface (three of the four functions above; replace with your control); FK/IK with joint zero offsets (multi-seed `ik_all`), pixel-to-desk mapping, triangulation, clearance checks, outer-loop `servo`, partial chessboard detection |
| | `grab.py` | Grabs a wrist frame (`frame`); run on its own, saves one frame with the joint angles |
| `ik/` | `kinematics.py`, `frames.py` | Model from the URDF: FK, IK, Jacobian, gravity term; world frame definition |
| `calibration/` | `plan.py`, `capture.py`, `solve.py`, `check.py` | Camera calibration: plan poses, capture at each pose, solve intrinsics, hand-eye, joint zero offsets and board plane, held-out check |
| `perception/` | `pens.py` | Finds pens lying flat in the wrist image: end points in world coordinates and which end has the big cap |
| | `view_plan.py`, `box.py` | Choose look-down poses for the box (no motion), fit the 4×6 slot lattice from several views |
| `sorting/` | `sort.py` | Step-by-step entry point for a run: `look`, `pick`, `restage`, `putdown`, `place`, `verify`, `reseat`, `addpen` |
| | `task.py` | Implementation of the steps, plus checks such as `holding`, `hangs_straight`, `find_cap` |
| | `pick_direct.py` | Direct grasp: picks the gripper turn and grasp point from how the pen lies, with the joint-limit margin check |
| | `reseat2.py`, `reseat_marked.py` | Straighten a tilted pen, cage and jiggle it into its slot; manual point picking for dark caps |
| | `refit.sh` | Re-photographs the three look-down views and refits the box, keeping the previous fit if the new one is not sane |

## Safety

- These scripts drive a real robot. Read the repository's [docs/safety.md](../../docs/safety.md) first, clear the workspace, have someone on site, and keep the hardware emergency stop within reach.
- A stop or disconnect that releases torque, or restarting the control program, drops the arm. Support the arm by hand before doing any of them.
- Run motion scripts in the foreground one step at a time and look at the image between steps. Path checks block with `assert`.
- Do not strain for a pen that is out of reach: a grasp right at a joint limit ended in a motor error, the controller releasing torque and the arm dropping.
