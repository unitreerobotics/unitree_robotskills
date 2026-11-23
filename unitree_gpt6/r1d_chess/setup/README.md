# Setup: bringing up a new robot or a new site

English | [中文](README.zh.md)

What ships in `config/` with the package is the calibration result of one reference robot at its site (the same content is also stored in `config/reference/` for comparison). **Before starting to play in a new environment, you must first complete the setup below and update `config/` with the results of this robot.** Camera intrinsics and extrinsics, link mass and friction, and the board position all differ between robots and sites; reusing the reference values may cause pick-and-place offsets or false triggering of the tracking protection.

Check the progress at any time (offline, without connecting to the robot):

```bash
python scripts/setup_status.py            # each item: reference value / updated / confirmed to reuse the reference value
python scripts/setup_status.py --require  # exit code is 1 while required items are still not completed
```

The agent must run it before enabling motion for the first time (`serve --allow-motion`, `enable_control`, `goinit --execute`, `start --execute`); do not make moves while a required item is still a reference value, unless the user explicitly decides to reuse it (see "Reusing reference values" at the end).

## Order

| Order | Content | Motion | Configuration updated |
|---|---|---|---|
| 0 | [Local environment](#step-0-local-environment): Python, engine, `config/runtime.json` | No | `runtime.json` |
| 1 | [camera-check](camera-check/README.md): confirm the three camera streams and their correspondence to the physical left and right hands | No | `chess_camera_policy.json` |
| 2 | [chess-scene-parameters](chess-scene-parameters/README.md) step 1: record the initial pose, lock the head pose | Read-only | `robot.json`, `chess_initial_pose.json` |
| 3 | [arm-calibration](arm-calibration/README.md): on-robot identification of the dual-arm dynamics (link mass, centre of mass, friction) and the waist and head hold parameters | Large-range motion of both arms, waist and head | `arm_torque_ff.json`, `hold_joints.json`, the waist and head gains in `runtime.json`, the dynamics URDF |
| 4 | [joint-tracking](joint-tracking/README.md): verify the tracking error on the Runtime link, and fine-tune the torque feedforward if necessary | Motion of both arms | `arm_torque_ff.json` |
| 5 | [head-camera-calibration](head-camera-calibration/README.md): head stereo intrinsics, baseline, mounting extrinsics, head joint corrections | Head motion in stage 2 | `head_camera_calibration.json` |
| 6 | chess-scene-parameters steps 2–7: TCP, table surface, board, release height | Step 3 turns the head, the rest are read-only | `robot.json`, `board_geometry.json` |
| 7 | joint-tracking step 7: confirm once on the send path of the chess service | Motion of both arms | — |
| 8 | Dry-run path acceptance (no piece): pick one real move and run the full preflight, then execute it once without gripping a piece | Motion of both arms | — |

Afterwards, when the board is moved, only the board geometry needs to be re-measured: `bash r1d boardcal`, see [board_position](../board_position/README.md).

Dependencies between the items: the head camera calibration uses the locked head pose and (preferably) the tuned feedforward; the scene parameters use the head camera calibration; the joint accuracy calibration depends only on the initial pose.

## Step 0: local environment

This package contains no machine-specific paths; the following items must be filled in for this machine:

| Item | Where to set it | Description |
|---|---|---|
| Python interpreter | Environment variable `R1D_CHESS_PYTHON`, or create `.venv/` in the package root | `bash r1d` looks for `R1D_CHESS_PYTHON`, `<package root>/.venv/bin/python`, and `python3` on PATH, in that order |
| Chess engine | Environment variable `CHESS_ENGINE`, or put `fairy-stockfish` on PATH | Must be a `largeboards=yes` build |
| DDS network interface | `network_interface` in `config/runtime.json` | Name of the network interface connected to the robot |
| Camera host | `camera_host` in `config/runtime.json` | Address of the image service on the robot; the default is 192.168.123.164 on the robot's internal network |
| External hold program (optional) | `hold_program` in `config/runtime.json` | When the site has a separate systemd unit that holds the robot pose, fill in the program path from its ExecStart, so that `handover` takes it over; when left empty, only the goinit unit of this package is recognised |
| Runtime service | `python scripts/install_runtime_service.py` | Generates the systemd user unit `r1d-runtime.service`, which runs the built-in robot runtime from this package (`third_party/robot_runtime`, the only DDS writer). By default it only prints the unit content, and writes it only with `--write`; an existing unit of the same name is not overwritten. Runtime and the chess service can use the same Python environment, or a different one can be specified with `--python` |
| Connection of the joint calibration tool | `ip`, `interface`, `transport` in `setup/arm-calibration/r1_d/config.json` | Used only during arm-calibration |

After filling them in, run `python scripts/check_environment.py`, and continue only when all required items pass; then confirm with `bash r1d runtime config` that the Runtime configuration can be parsed (without connecting to the robot). When `network_interface` is left empty, the chess service and the tools that need Runtime report an error directly. Runtime itself needs no configuration: `runtime_root` in `runtime.json` points by default to `third_party/robot_runtime` in the package, and `camera_enabled` stays `false` (the chess service fetches images itself).

## General rules

- Every command that makes the robot move must first be explicitly authorised by the operator, and there must be no people or obstacles nearby or within the range of motion of the arms.
- Do not loosen protection thresholds such as tracking error, temperature or stale feedback in order to make a test pass.
- During calibration, the chess service or other control programs must not run at the same time. arm-calibration goes directly over DDS, and also requires Runtime to be stopped.
- Put raw data and intermediate results under `runs/<name>/`, and point the `evidence` field written into the configuration to it.
- All commands are run from the package root, and `python` refers to the environment with the dependencies installed (the same as `R1D_CHESS_PYTHON`).

## Directory

```text
setup/
├── README.md                      this file
├── _common.py                  common code: locate the package root, read runtime.json, read the arm PD gains
├── camera-check/               correspondence between the camera streams and the physical left and right hands
├── chess-scene-parameters/     initial pose, TCP, scene, board, release height
├── head-camera-calibration/    head camera calibration (capture, detection, intrinsics, extrinsics solve)
├── joint-tracking/             tracking error comparison and feedforward fine-tuning on the Runtime link
└── arm-calibration/            identification of dual-arm dynamics and waist and head parameters (with its own r1_d direct-connection tool)
```

## Reusing reference values

When an item, after checking, can indeed reuse the reference value (for example, the camera mapping is the same as the reference after checking), record the basis:

```bash
python scripts/setup_status.py --confirm camera_mapping --note "checked the three streams one by one by covering them; left and right wrists match the reference"
```

The confirmation record is bound to the file content, and becomes invalid automatically when the file is changed afterwards. The head camera calibration, the initial pose, the scene and the board geometry should not reuse the reference values.

## Reference values

`config/reference/` holds the results of the reference robot. It is only for comparing orders of magnitude; do not modify it, and do not iterate from it as the initial values for a new robot. The offline tests are written against the reference configuration; after `config/` is updated, individual cases that depend on specific values may need to be adjusted accordingly.
