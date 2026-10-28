# g1d_gomoku — G1-D autonomous Gomoku

English | [中文](README.zh.md)

The host agent is configured separately by the user; this project does not bundle a model API client or come with model access. Specific model names in the documentation describe the development workflow and do not mean that installing this project grants access to that model.

The user only needs to say "next move" or "run the next round". An agent with image understanding and tool-calling capabilities follows [SKILL.md](SKILL.md) to observe the board, recognize black and white stones, select a point autonomously, and then control the right arm to pick a black stone, align it, place it, and return home. The user only confirms before the gripper is released.

**There is only one task mode: autonomous play.** Specifying board coordinates, fixed landing points, preset game records, or symbolic pick-and-place previews is not supported. Row/column coordinates are internal records produced by recognition and decision-making; the end-effector pose is solved by the agent from this round's images and feedback.

## Who does what

- **Host agent**: reads the four raw camera images, recognizes the game position, re-checks the grid, fine-tunes pick and place, checks path clearance, and asks for release confirmation. It uses the agent's own image-understanding and tool capabilities and does not ask the user to fill in the position or a target pose.
- **Game engine**: checks the complete 19×19 black/white position, the turn and the end of the game; it automatically prefers making five in this move, then blocking White from making five, and then compares attack and defense by contiguous and gapped shapes. It is a deterministic local-threat strategy, not a professional-strength search engine.
- **Round workflow**: binds the raw images to the recognition result, saves the unique target and its annotation, checks the alignment record and the precomputed return path, records confirmation, and calls the same publisher to release and retreat continuously.
- **Hardware tools**: provide DDS, the four cameras, FK/IK, continuous control, feedback checks and return home.

This is a skill executed by an agent; the Python process does not invoke an unconfigured vision model in the background on its own. Running the CLI by itself reports the next state whenever the agent needs to look at images or plan motion; the host agent continues according to the skill.

## From deployment to actual play

You are responsible for the first deployment, setting up the board and confirming release; gpt6-astra is responsible for recognizing the position, calling the engine to select a point, and controlling the right arm to pick, place and return home. You do not need to fill in board coordinates, a position matrix or arm poses. Follow the steps below in order for first use.

### 1. Prepare the runtime environment

First use a Pinocchio environment with the CasADi binding. A new environment can be created from conda-forge:

```bash
conda create -n gomoku -c conda-forge python=3.10 'pinocchio>=3,<5' 'casadi>=3.6,<4' pip
conda activate gomoku
```

Then install from the repository root:

```bash
python -m pip install -r unitree_gpt6/g1d_gomoku/requirements.txt
```

The DDS communication SDK for G1 and Dex1 is bundled in [unitree_sdk2py/](src/unitree_sdk2py/VENDORED.md) and is used directly from the checkout; requirements.txt installs the numerical and communication dependencies. Pinocchio must include `pinocchio.casadi`; installing `pin` and `casadi` via pip alone does not guarantee that binding. The SDK keeps its upstream BSD 3-Clause license.

The G1 model is bundled in [unitree_gpt6/g1d_gomoku/models/g1/](models/g1/README.md), including `G1_29_ArmIK`, joint configuration and indices, the URDF and all 49 referenced meshes.

Follow [Hardware integration](docs/hardware_integration.md) to start the TeleImager camera publisher and connect the robot and camera networks. Do not modify the project or install dependencies during operation.

### 2. Create a workspace inside the repository

Still run from the repository root. By default `gomoku-workspace/` is created under the repository root; you can also set `GOMOKU_WORKSPACE` to a location of your choice beforehand. The configuration is initialized from the repository templates and does not depend on files on the developer's machine. The workspace is already covered by Git ignore rules; at runtime files are written only here, and source code, project configuration and documentation stay read-only.

```bash
export GOMOKU_SOURCE_ROOT="$(pwd -P)"
export GOMOKU_WORKSPACE="${GOMOKU_WORKSPACE:-$GOMOKU_SOURCE_ROOT/gomoku-workspace}"
python -m unitree_gpt6.g1d_gomoku.src.workspace --workspace "$GOMOKU_WORKSPACE"
export GOMOKU_WORKSPACE="$(cd "$GOMOKU_WORKSPACE" && pwd -P)"

export GOMOKU_HARDWARE_CONFIG="$GOMOKU_WORKSPACE/config/hardware.local.yaml"
export GOMOKU_OWNER_ROOT="$GOMOKU_WORKSPACE/runtime/game"
export PYTHONPATH="$GOMOKU_SOURCE_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
export TMPDIR="$GOMOKU_WORKSPACE/tmp"
export XDG_CACHE_HOME="$GOMOKU_WORKSPACE/cache"
export PIP_CACHE_DIR="$XDG_CACHE_HOME/pip"
export RUFF_CACHE_DIR="$XDG_CACHE_HOME/ruff"
export MPLCONFIGDIR="$XDG_CACHE_HOME/matplotlib"
mkdir -p "$PIP_CACHE_DIR" "$RUFF_CACHE_DIR" "$MPLCONFIGDIR"
cd "$GOMOKU_WORKSPACE"
```

All later terminals and agent tool processes must inherit these variables and use the workspace as the current directory. You can save the environment setup in your own workspace script and load it in each new process. Temporary code goes in `scratch/`, round images and plans in `runtime/rounds/`, and logs in `logs/`. See the [architecture document](ARCHITECTURE.md#4-the-gpt6-astra-standalone-workspace) for the full directory layout.

First check the actual numerical backend; no rest-pose or scene calibration file is needed and no device is connected:

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main \
  --config "$GOMOKU_HARDWARE_CONFIG" model-check
```

The result must be `model_initialized: true`. Otherwise, first fix the same environment according to the reported `python_executable` and the specific import error, and only then proceed to on-site capture.

### 3. First-time configuration and on-site capture

The hardware configuration is the workspace's `config/hardware.local.yaml`; the template is [hardware.example.yaml](config/hardware.example.yaml). Fill in `camera.host` and `robot.network_interface` for your deployment network; for ports, DDS domain, control rate and control limits see the [configuration guide](config/README.md). The project does not ship any particular machine's rest pose or pick-and-place experience.

First use your existing control method to put the robot in a rest pose suitable for the current scene; after the agent has checked it, capture the rest-pose reference read-only:

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main \
  --config "$GOMOKU_HARDWARE_CONFIG" \
  pose --output "$GOMOKU_WORKSPACE/calibration/reset_right_state.json" \
  state --grippers

python -m unitree_gpt6.g1d_gomoku.src.experience init \
  --workspace "$GOMOKU_WORKSPACE"
```

The capture command does not move the robot; it only saves the current joints of both arms, the waist angle, the TCP and gripper feedback. Do not capture in an arbitrary pose, and do not fill in zeros in place of feedback. The rest-pose reference is pre-execution calibration, not success experience. Old default files in an existing workspace should be moved out of the current configuration and captured again from the current device.

In the YAML, `runtime.directory: ../runtime` and `runtime.reset_state: ../calibration/reset_right_state.json` are both resolved relative to the directory containing the YAML. All control processes share the same configuration and control lock.

Before running the next step's preflight, the host agent first inspects the current four camera images and feedback read-only and completes the workspace's `calibration/scene.json`; for fields and requirements see [Scene geometry data](config/README.md#scene-geometry-data). The host agent can be started from the current workspace; see step 5 for how to load the skill. Do not start the motion publisher during calibration.

On first use the experience library is empty; gpt6-astra plans motion from the current four camera images, robot feedback, model and on-site calibration. Only after returning home and confirming that the corresponding stage succeeded is measured experience written through the [experience data tool](docs/experience.md); later it is used as a reference after matching robot, scene, configuration, calibration and model.

### 4. Check configuration, feedback and the four camera images

First run the fully offline preflight to confirm that the configuration is valid and all dependencies are available:

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main \
  --config "$GOMOKU_HARDWARE_CONFIG" preflight
```

Preflight does not connect to devices and does not verify on-site calibration. Then check the actual state and cameras read-only:

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main \
  --config "$GOMOKU_HARDWARE_CONFIG" state --grippers

python -m unitree_gpt6.g1d_gomoku.src.hardware.main \
  --config "$GOMOKU_HARDWARE_CONFIG" \
  camera --snapshot "$GOMOKU_WORKSPACE/calibration/camera_check"
```

Confirm that the left and right head images show the complete board clearly and that the wrist images help check the stone supply and the gripper. Fix the board in place and put graspable black stones in the stone supply area on the table.

A new scene requires establishing or re-checking the relationship between the board, the table, the supply area and robot coordinates. Image intersections cannot be used directly as robot coordinates; the project has no default board size or pick-and-place height, and the agent must establish the current scene calibration and check reachability and clearance of the entire path.

### 5. Start gpt6-astra and load the skill and the continuous controller

Start the host agent from the workspace, make sure it can read local images, run commands, write files and manage long-running background processes, and that it inherits the environment variables above. Give it the following instruction:

> Read `unitree_gpt6/g1d_gomoku/SKILL.md` under `GOMOKU_SOURCE_ROOT` and follow the autonomous Gomoku workflow. Use `GOMOKU_WORKSPACE` to store all temporary code, images, plans and logs; during operation write only to the workspace, and keep source code, project configuration and documentation read-only. Use `GOMOKU_HARDWARE_CONFIG` and `GOMOKU_OWNER_ROOT`, check for an existing controller, and start only one continuous publisher. First check communication, the four camera images, the on-site rest-pose reference and calibration, then autonomously play the next black move. You handle picking, fine-tuning and return-path preparation; wait for my confirmation before releasing the gripper.

The agent should start the single controller in a separate long-running process. The corresponding tool command is below; if you have already started it manually, tell the agent to reuse it and not to start another one:

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main \
  --config "$GOMOKU_HARDWARE_CONFIG" \
  control --root "$GOMOKU_OWNER_ROOT" --execute owner
```

**From this step on, real control commands are published.** After successful initialization it continuously holds the startup pose and does not return home automatically. Keep this process running during operation; do not restart it for every move. When the robot is not in the rest pose, the agent first reviews the current corridor and plans lifting, exit and return home.

Another terminal that inherits the same environment can check status read-only:

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main \
  --config "$GOMOKU_HARDWARE_CONFIG" \
  control --root "$GOMOKU_OWNER_ROOT" status
```

### 6. Play one move

After you say "autonomously play the next black move", the agent should complete the whole flow; you do not need to run the stage commands one by one:

```text
play → observe and recognize → decide → pick, short-lift check, high transfer, low alignment
     → prepare return → prepare-release → user confirmation → release → return home and wait
```

| Stage | What the agent does | Mechanical motion |
| --- | --- | --- |
| `play` | Creates a new round directory, checks the rest pose, captures the four raw images and feedback | Does not change the target pose |
| Observe and recognize | Recognizes the complete black/white position, cross-checks the grid, saves the recognition result | No motion |
| `decide` | The engine selects a point autonomously and generates the single-intersection annotation | No motion |
| Pick and align | Calls `move` and `grip` to pick a black stone, check with a short lift, transfer and align | Moves |
| Return preparation | Generates the complete retreat plan from the high-level approach path already executed this round | Plan only |
| `prepare-release` | Checks and binds the target, the low alignment record and the return path | Does not release |
| Awaiting confirmation | Explains the landing point, release and continuous retreat, and waits for your actual confirmation | Keeps holding |
| `release` | After confirmation, releases, saves images once, lifts continuously, exits and returns home | Moves |

You only need to answer "confirm placement" when asked for release confirmation. The recognition files, point selection, poses, motion commands and return path are generated by the agent itself; it must not approve on the user's behalf.

`play` is only an observation stage; run by itself it stops at `awaiting_board_review` and waits for the agent to look at the images. It does not start a controller, pick a stone or return home automatically. An existing controller keeps publishing pose-hold commands. `decide` refuses to select a point when recognition is missing, the left and right head images disagree, there are uncertain intersections, it is the wrong turn, or the game has ended; it does not assume an empty board.

The following is only for understanding the agent's stage calls and assumes this move's directory does not yet exist:

```bash
export GOMOKU_ROUND="$GOMOKU_WORKSPACE/runtime/rounds/round_01"
python -m unitree_gpt6.g1d_gomoku.src.main play \
  --config "$GOMOKU_HARDWARE_CONFIG" --owner-root "$GOMOKU_OWNER_ROOT" \
  --round "$GOMOKU_ROUND"

# After the agent has reviewed the four raw images and written this round's recognition result:
python -m unitree_gpt6.g1d_gomoku.src.main decide --round "$GOMOKU_ROUND"
```

See the [autonomous execution workflow](docs/autonomous_play.md) for low-level motions and stage file formats. The return path must be prepared before release confirmation is requested; after the target, configuration, holding pose or plan changes, an old confirmation cannot be reused. After release, the client returns only when the complete retreat has finished.

### 7. Continue the game and finish

After the robot returns home, place your white stone and then say:

> I have placed my stone, please run the next round.

The agent reuses the same continuous controller, creates a new round directory for each move, and observes and decides again, without reusing the old position, old target or old authorization.

To finish, tell the agent:

> End the game, confirm the robot is in the on-site rest pose, stop the publisher following the on-site control handover procedure, and report the exit result.

Stopping publishing is neither continued pose holding nor a hardware emergency stop; control must be handed over according to the on-site procedure. After a protective stop or stale feedback, check the current state before resuming control; do not directly replay the previous round's commands. `execution_failed` or `return_unverified` also cannot be retried by running `release` again.

## Round records

Each move uses a new directory that stores the four raw images, image digests, the complete position recognition result, `position_and_move.json`, the single-intersection annotation `suggested_black_move.svg`, stage feedback, the return plan and the confirmation record. The SVG embeds the full raw image and can be viewed directly in a browser.

Internally, rows and columns are 1-based and numbered from top to bottom and left to right in the left head image. Black is played by default, there are no forbidden moves, and five or more in a row wins. An existing position continues from the current turn; after each move the robot returns home and waits for the opponent and the next round's start instruction.

After placement, images are saved only once and not analyzed; no further images are captured, and the arm lifts continuously and returns home. Therefore `returned` means the rest-pose joint feedback check passed, while `physical_placement_verified` remains `false`; it must not be described as visual acceptance of the landing point.

## Documentation and tests

- [Code architecture and workspace](ARCHITECTURE.md): module relationships, execution flow, and the portable configuration of the standalone runtime directory inside the repository.
- [Skill entry point](SKILL.md): usage conventions for the host agent.
- [Autonomous execution workflow](docs/autonomous_play.md): initialization, observation and recognition, motion tools and stage file formats.
- [Operating rules](docs/operating_guide.md): constraints on tasks, game rules, poses, pick and place, and records.
- [Manipulation methods](docs/manipulation.md): positioning, grasping arm configuration, and returning home along the reversed approach path.
- [Experience data](docs/experience.md): starting from an empty library, evidence validation and writing data after success.
- [Hardware integration](docs/hardware_integration.md): optional dependencies, private configuration and device tools.

`config/default.yaml` stores only the autonomous mode and game rules; experience data lives in the workspace's `experience/`, and the empty templates in the source tree are not modified at runtime.

```bash
python -m unittest discover -s unitree_gpt6/g1d_gomoku/tests -v
python -m ruff check .
python -m ruff format --check .
```

The tests use synthetic positions, fake camera files and fake feedback to cover autonomous decisions, state transitions and failure behavior, and check the model asset digests. When the numerical dependencies are available, they also actually load the bundled model and verify FK, IK, gravity compensation and left-arm holding; none of the tests connect to the robot.

The initializer only generates configuration templates, directories and an empty experience library, preserves existing configuration, and does not generate rest-pose or scene calibration. It cannot replace on-site capture; `preflight` still reports an error when required fields or captured files are missing. Offline verification does not constitute acceptance of robot communication, cameras or actual pick and place.
