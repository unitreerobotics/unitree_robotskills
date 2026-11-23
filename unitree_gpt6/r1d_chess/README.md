# R1-D Chess

English | [中文](README.zh.md)

Through [SKILL.md](SKILL.md), a model that can run terminal commands and view images controls a Unitree R1-D (R1-A7-D, Dex1 grippers) to carry out single-round planning, left-arm and right-arm pick and place, captures, and result review for Chinese chess (xiangqi).

The model reads the images to confirm the position, calls the chess engine to choose a move, chooses the acting arm and the pick and place poses, and judges every visual gate; the Python service handles full-plan preflight, one-time tokens, and motion execution. This package has no master CLI that recognises images automatically and plays a whole game, and it does not include the image service that runs on the robot.

The package includes the setup tools for bringing up a new robot or a new site (camera check, head camera calibration, arm accuracy calibration, chess scene parameters, and board calibration), and the robot runtime that sends the commands.

## Installation and configuration

The baseline is Linux x86_64 with Python 3.10. Keep the whole directory, enter this skill directory, and run:

```bash
python3.10 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m pip install --no-deps "unitree_sdk2py @ git+https://github.com/unitreerobotics/unitree_sdk2_python.git@814556d15970dd2ecf1c9984e845ca02ab07e206"
cp .env.example .env
```

You can also use your own Python environment that already has the dependencies installed. The unified entry point `bash r1d <subcommand>` uses, in order, `R1D_CHESS_PYTHON`, `<skill directory>/.venv/bin/python`, and the `python3` on PATH; each subcommand is equivalent to `python scripts/<entry>.py`.

Only three items need to be prepared for the local machine:

| Item | Where to set it |
|---|---|
| Chess engine Fairy-Stockfish (must be built with `largeboards=yes`) | Environment variable `CHESS_ENGINE`, or put `fairy-stockfish` on PATH |
| DDS network interface connected to the robot | `network_interface` in `config/runtime.json` |
| Address of the image service on the robot | `camera_host` in `config/runtime.json` (the default is the robot's internal network address) |

The program does not load `.env` automatically; see [.env.example](.env.example) for the supported environment variables. For the engine build and other details, see "Entry points and runtime environment" in the [operation manual](docs/operation_manual.md). After configuration, run the offline checks:

```bash
python scripts/check_environment.py     # dependencies, engine, model hashes, configuration; does not connect to the robot
bash r1d                                # list the subcommands
```

### Robot runtime

The robot runtime (Runtime) is built into [third_party/robot_runtime](third_party/robot_runtime/README.md) and does not need to be obtained separately. It is the only process that sends commands to the robot. It runs as a systemd user unit, and the chess service calls it through local IPC:

```bash
python scripts/install_runtime_service.py            # print the unit content without writing it
python scripts/install_runtime_service.py --write    # generate r1d-runtime.service
```

### Bringing up a new environment (setup)

**The files shipped in `config/` are the calibration results of one reference robot** (also stored in `config/reference/`). Camera intrinsics and extrinsics, link masses and friction, and the board position all differ between machines and sites, so before playing chess in a new environment you must first complete setup as described in [setup/README.md](setup/README.md) and update `config/` with the results of the local machine:

| Content | Procedure | Result written to |
|---|---|---|
| Camera check (head and wrist camera streams, mapping between wrist streams and physical arms) | [setup/camera-check](setup/camera-check/README.md) | `config/chess_camera_policy.json` |
| Arm accuracy calibration (dual-arm dynamics, waist and head hold parameters) | [setup/arm-calibration](setup/arm-calibration/README.md), [setup/joint-tracking](setup/joint-tracking/README.md) | `config/arm_torque_ff.json`, `config/hold_joints.json` |
| Head camera calibration (intrinsics, baseline, eye-on-head extrinsics) | [setup/head-camera-calibration](setup/head-camera-calibration/README.md) | `config/head_camera_calibration.json` |
| Chess scene parameters (initial pose, TCP, scene, release height) | [setup/chess-scene-parameters](setup/chess-scene-parameters/README.md) | `config/robot.json` |
| Board calibration (board surface height, centre, rotation angle, 90 intersections) | Steps 3–4 of setup/chess-scene-parameters; afterwards, when the board moves, use `bash r1d boardcal` ([board_position](board_position/README.md)) | `config/board_geometry.json` |

```bash
python scripts/setup_status.py            # whether each item is a reference value, updated on this machine, or confirmed to keep the reference value
python scripts/setup_status.py --require  # exit code 1 while required items are still incomplete
```

Every setup step that moves the robot requires the operator's explicit authorisation and confirmation that the surroundings are safe.

## Using it through the skill

Keep the whole directory and have the model read [SKILL.md](SKILL.md). For example:

> Read `/actual-deploy-path/r1d_chess/SKILL.md`, check the environment and the setup progress; this time do offline checks only and do not connect to the robot.

When setup is complete and you are ready to actually move pieces:

> Read `/actual-deploy-path/r1d_chess/SKILL.md`, complete this round's move for Red on the current board, and verify source/target and the actual placement.

Based on the task, the skill chooses among the flows for environment and setup, starting the service and taking over, reading the board and board calibration, single-round execution, or fault retraction, and it refers to the documents and scripts in this directory. Copying only `SKILL.md` does not bring the runtime code, models, and configuration with it.

A normal single round follows [round_plan](prompts/round_plan.md); see the [operation manual](docs/operation_manual.md) for the detailed commands. Handle the gates during execution as "`next` → look at this checkpoint's two images → submit → `next`"; do not study code or install missing dependencies while the robot is holding a piece.

## Directory

```text
r1d_chess/
├── README.md                    installation and usage entry
├── SKILL.md                     model task entry and on-demand document routing
├── r1d                       unified entry point: bash r1d <subcommand>, dispatches to scripts/
├── .env.example              template of the supported environment variables
├── requirements.txt          Python dependencies
├── config/                   site configuration; reference/ holds the reference robot's results
├── setup/                    bringing up a new robot or site: camera check, calibration, scene parameters
├── src/r1d_chess/            implementation: robot/ chess/ cameras/ board/
├── scripts/                  executable entries (thin wrappers), environment check, setup progress, runtime unit installation
├── third_party/robot_runtime/ built-in robot runtime (the only channel that sends commands)
├── assets/                   kinematic and dynamic models, JSON form templates, piece legend
├── prompts/                  single-round prompt round_plan
├── docs/                     operation manual, script inventory, and topic notes
├── board_position/           board-reading helper images, measurement of the 90 board points, and recalibration
└── tests/                    offline tests and samples
```

At run time, images, plans, checkpoints, service logs, and reviews are generated in `runs/`; this directory, `.env`, and the virtual environment are not under version control. For script names, inputs and outputs, and the mapping to modules, see the [script inventory](docs/round_script_inventory.md).

## Offline verification

```bash
python scripts/check_environment.py
python -B -m unittest discover -s tests -p 'test_*.py'
```

These checks do not connect to the cameras, do not initialise robot control, and do not operate the robot; the tests use a simulated transport. They do not replace setup, this round's full-path preflight, or the confirmation by head image after the move.

## Notes for first use in a new environment

- The built-in robot runtime and the setup procedures have both been verified offline; when connecting to a new robot for the first time, first confirm that feedback is normal with `bash r1d runtime status` and the read-only `bash r1d client get_state`, then take over.
- After setup is complete, first do one dry-run path acceptance (after the full preflight, execute one move without gripping a piece), then do a real pick and place, and confirm the landing square with the head image after the move.
- A few parameters are given for the reference robot and may need adjustment on a new robot; see "Verification and capability limits" in the [operation manual](docs/operation_manual.md).
- The offline tests are written against the reference configuration; after you update `config/` with the local results, test cases that depend on specific values may fail, which does not mean the code is wrong.
