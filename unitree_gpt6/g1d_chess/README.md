# G1D Chess

English | [中文](README.zh.md)

Through the repository's [SKILL.md](SKILL.md), a model with terminal execution and image viewing capabilities can use the Unitree G1 and the built-in Dex1 gripper to complete single-round planning, left/right-hand pick and place, consecutive captures and result review for Chinese chess (Xiangqi).

The model is responsible for viewing images to confirm the position, estimating source/target, calling the chess engine and judging checkpoints; the Python service is responsible for complete plan preflight, transaction caching and motion execution. The repository has no master-control CLI that automatically recognises images and plays a complete game, and does not include the robot-side camera server.

## Installation and configuration

The baseline is Linux x86_64 and Python 3.10. First install Python, venv and Git on the system, keep the full repository, enter the skill directory from the repository root and run:

```bash
python3.10 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
source .venv/bin/activate
cp .env.example .env
```

You can also activate a Python environment the user has already installed and add the dependencies with `python -m pip install -r requirements.txt`, without creating a new `.venv`. Subsequent commands use that environment's `python` directly; the service launcher uses the `python` on PATH by default and also supports `--python /actual/env/bin/python`.

Fill in the separately installed Fairy-Stockfish, the DDS network interface and the camera server according to [.env.example](.env.example). For the built-in Dex1, both the left and right grippers default to `CHESS_GRIPPER_SIGN=1`: the gripper opens as the motor angle increases in the positive direction, and `q=0` is closed; for example, a planned opening amplitude of `2.3` corresponds to a motor target of `+2.3 rad`. This is the direction convention of the built-in gripper; changing the host, Python or network environment does not require re-entering or flipping it, while different hardware/firmware joint conventions can still be overridden explicitly. Then load it:

```bash
set -a
source .env
set +a
python scripts/check_environment.py
```

For engine installation, platform compatibility and native-library issues see the [environment setup guide](docs/environment_setup.md). The code uses this repository's model module and does not require installing `unitree_deploy`; `unitree_sdk2py` is the robot communication dependency installed through the chess optional dependency group. `.env` is not loaded automatically by the program, and the configuration JSON does not automatically export environment variables.

Then complete the [first-game setup](docs/first_game_setup.md): confirm the physical wrist-stream mapping, scene, the init poses of both arms, board dimensions and the position marking of the current image. For scene, prefer on-site values provided by the user; even if the user provides no on-site information, the model estimates all missing fields itself from available images, robot state, tool models, dimension data and reasonable assumptions, records the basis and uncertainty, and then sets them, without requiring the user to supply each item or confirm separately. For the specific method see section 5 of the first setup. The released on-site configuration is empty; model estimates and template parameters must be labelled with their source and must not be treated as measurements of the current device.

## Using it through the skill

Keep the entire repository and let the model read the root [SKILL.md](SKILL.md). For example:

> Read `/actual/deployment/path/g1d_chess/SKILL.md` and check whether the first setup is complete; this time only check the environment and do not connect to the robot.

When the configuration is complete and you are ready to play a real move:

> Read `/actual/deployment/path/g1d_chess/SKILL.md`, complete this Red round according to the current board, and check the source/target and the actual landing point.

Depending on the task, the skill chooses installation and configuration, service and waist hold, board marking, single-round execution or recovery procedures, and references this repository's documents and scripts. Copying only `SKILL.md` does not bring the runtime code and models; nothing here depends on other projects' working copies or globally installed robot skills.

A normal single round follows [round_plan](prompts/round_plan.md); detailed commands are in the [operation manual](docs/operation_manual.md). Gates during execution are handled as `next → view the two images of this step → submit → next`; do not study code or install missing dependencies while the robot is holding a piece.

## Layout

```text
g1d_chess/
├── README.md                 Installation and usage entry point
├── SKILL.md                  Model task entry point and on-demand document routing
├── .env.example              Template of supported environment variables
├── config/                   On-site configuration and blank measurement records
├── src/g1d_chess/             Chess, robot, camera and board marking implementation
├── scripts/                  Executable CLIs, launchers and offline checks
├── assets/                   Required model assets and JSON form templates
├── prompts/                  Single-round prompts and board/move conventions
├── docs/                     Operation manual and topic guides
├── board_position/           New-board position marking guide
└── tests/                    Offline tests and synthetic data
```

At runtime, images, plans, checkpoints and reviews are generated under `runs/`; that directory, `.env` and virtual environments are not put under version control. For script names, inputs/outputs and module dependencies see the [script inventory](docs/round_script_inventory.md).

## Offline validation

With the dependencies installed and the environment variables loaded, keep the full repository, enter the skill directory from the repository root and run:

```bash
python scripts/verify_package.py
python scripts/check_environment.py
python -m unittest discover -s tests -p 'test_*.py'
```

These checks do not connect to cameras, do not initialise DDS and do not operate the robot. `verify_package.py` checks the file structure, syntax, documentation links and the script inventory; when a local `manifest.json` exists it additionally verifies file hashes. That manifest is not uploaded to Git, and the other checks can still be completed when it is missing. They cannot replace on-site configuration and this round's complete path preflight. Differences remain between the operating rules and the current implementation, especially the local checks of XY correction segments; see [implementation check boundary](docs/runtime.md#implementation-verification-boundary).
