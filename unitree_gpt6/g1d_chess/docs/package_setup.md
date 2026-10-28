# Package Usage and Offline Verification

English | [中文](package_setup.zh.md)

The working root is this deployment's `g1d_chess/`. On a brand-new device, first complete [First Game Setup](first_game_setup.md) and the [Configuration Template Guide](../config/README.md), then execute per the [Operation Manual](operation_manual.md) and [round_plan](../prompts/round_plan.md). All project entry points use this package's files; the optional local `manifest.json` records file hashes and is not uploaded to Git.

## Directories and dependencies

- `docs/operation_manual.md`, `prompts/`, `docs/`: the complete specification, per-round prompts and topic guides; for their relations see the [Document Index](README.md).
- `scripts/`, `src/`: the only current planning, service, camera and review entry points. There is no need to install user-level skills or access other projects' working copies.
- `config/`: configuration templates with machine-specific values removed; on-site information must be filled in per the first-time guide. `config/init_state.json` and the board size record are empty and must be filled in on site; the flow candidate parameters in `assets/templates/` must still be checked for applicability.
- `board_position/`: instructions and general scripts for marking the 90 points on the current image; no real photos, pixels, robot XY or pose anchors are preset, and this session's results are generated when used.
- `assets/models/g1/`: the current model URDF/meshes; the `chess` group of `pyproject.toml`: Python dependencies and the official SDK pinned commit. The engine is installed separately; see the [Environment Setup Guide](environment_setup.md).
- Python interpreter: use the current environment in which the user has installed dependencies, and run `python scripts/…` directly, or use the interpreter's absolute path.
- `runs/`: created when used, holding new runs, checkpoints, images and logs for the current environment; the release package presets no run data, and old tokens cannot be replayed.

First install the `chess` group of `pyproject.toml` and an external engine supporting xiangqi per the [Environment Setup Guide](environment_setup.md), then configure the on-site network/hardware. The package already includes the model and build code; `unitree_deploy` or a working copy from an old machine is not needed. The SDK is a required externally installed dependency; `config/python_packages.json` is a version record to be filled in and is not responsible for installation. The on-site configuration JSON does not automatically install dependencies or start services.

## Offline verification

```bash
cd /path/to/g1d_chess
source /path/to/robot-env/bin/activate
export CHESS_ENGINE=/path/to/fairy-stockfish
python scripts/verify_package.py
python scripts/check_environment.py
python -m unittest discover -s tests -p 'test_*.py'
python scripts/round_tools.py --help
python scripts/round_tools_left.py --help
```

These commands do not initialize DDS, do not open cameras and do not publish robot commands. The environment check resolves the actual import locations and model resources and runs the engine UCI handshake; it cannot prove that on-site cameras, DDS or motion have been verified. `verify_package.py` checks the file structure, syntax, entry-point links and the script inventory; when a local `manifest.json` exists it additionally checks hashes, and it also runs without the manifest. When on-site config edits change hashes, keep a record of the local modifications; changing the hashes cannot replace configuration verification.

## Starting a round

Execute in stages starting from Section 4 of the Operation Manual. For each independent tool call, set this package's workdir and absolute paths, with `CHESS_PYTHON` as the absolute path of the user environment's interpreter (e.g. `CHESS_PYTHON="$(command -v python)"`); client, serial and run are all inside the package. Prefer reusing a healthy, idle service under this package's ROOT with the correct acting hand. An existing service does not automatically load disk updates; before switching, handle it according to the real control, holding, waist-hold and transaction state, and do not use the launcher to probe the environment.

Before planning, configure the physical camera mapping and the error context, and check the on-site scene, dual-arm init, actual waist hold and board images. Source and target poses are still estimated independently; the board release Z=.139m and same-side tray Z=.159m in the templates are current flow conventions; if incompatible on site, first adapt the affected templates and implementation per the first-time guide. Historical measurements must not be used as defaults for a new device.

The current main session is responsible for confirming the position from images, rules-library legality, forecast matching/saving, independent grasp/place estimation, necessary engine move selection and the six/seven-field gate evaluation. There is no controller CLI that completes a round automatically. Before motion, prepare the full branches, pre-check and submission files; do not tidy code or material during execution. For implementation boundaries and timeout behavior see [runtime](runtime.md); the end only performs the current two review items.
