# Installing the Standalone Runtime Environment

English | [中文](environment_setup.zh.md)

This project does not require installing or keeping the `unitree_deploy` project. The kinematic model is built in this package's `g1_model.py`, with URDF and meshes in `assets/models/g1/`; FK, IK and gravity compensation use Pinocchio and SciPy. `unitree_sdk2py` is still a required dependency for robot DDS communication; the official pinned commit is installed through the `chess` group of `pyproject.toml`, and the SDK source is no longer copied into the package.

## 1. Set up the Python environment

The baseline is Linux x86_64, Python 3.10. Other systems, architectures or Python versions require re-verifying native-library and wheel compatibility. First install Python 3.10 with venv and Git via the system package manager; building the engine also requires a C++ compiler and Make. Keep the complete repository and run from the `unitree_gpt6/g1d_chess/` directory (dependencies are declared at the repository root; installing after copying only the skill directory is not supported):

```bash
python3.10 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e '../..[chess]'
source .venv/bin/activate
```

You can also use an existing virtual environment directly: activate it first, then run `python -m pip install -e '../..[chess]'` to fill in the dependencies; there is no need to create an in-package `.venv`.

The `chess` group of `pyproject.toml` pins the direct Python dependencies and the SDK commit; it is not a complete transitive dependency lock for all platforms. The first installation requires access to PyPI and GitHub; for offline deployment, prepare wheels and the engine for the target platform in advance. `pin` is the Pinocchio distribution; do not install another distribution with the same name `pinocchio`. The current runtime path no longer needs CasADi, the Pinocchio CasADi bindings, Meshcat or `unitree_deploy`.

If importing OpenCV is missing `libGL.so.1` or GLib, install the runtime libraries for the target distribution, e.g. `libgl1` and `libglib2.0-0` on Debian/Ubuntu. If Cyclone DDS has no matching wheel, or reports `Could not locate cyclonedds`, install the Cyclone DDS 0.10.x native library following the [official Python SDK installation instructions](https://github.com/unitreerobotics/unitree_sdk2_python#installation), set the actual `CYCLONEDDS_HOME` and retry; the Python binding is pinned to 0.10.2. The JSON environment record does not install these libraries.

## 2. Install the Chinese chess (Xiangqi) engine separately

`pyffish` handles rule validation and does not provide an engine executable for subprocess calls. Separately install [Fairy-Stockfish](https://github.com/fairy-stockfish/Fairy-Stockfish) with `xiangqi` support, using an official build suited to the target CPU or compiling from source. For example:

```bash
git clone https://github.com/fairy-stockfish/Fairy-Stockfish.git /path/to/Fairy-Stockfish
make -C /path/to/Fairy-Stockfish/src build ARCH=x86-64 largeboards=yes
export CHESS_ENGINE=/path/to/Fairy-Stockfish/src/stockfish
```

On Linux x86_64 you can also directly download the official `fairy_sf_14` [largeboard generic build](https://github.com/fairy-stockfish/Fairy-Stockfish/releases/download/fairy_sf_14/fairy-stockfish-largeboard_x86-64), save it to a chosen path, make it executable, and set `CHESS_ENGINE` to its absolute path. This build supports the 90-point Xiangqi board; a build without largeboard/xiangqi support cannot be substituted.

The source directory should use an actual installation path that does not yet exist; `ARCH=x86-64` applies to the x86_64 baseline above, and for other CPUs choose according to the engine documentation. Record the actual source commit and build parameters, and re-verify after upgrading. The engine is not required to be inside this package, and an ordinary Stockfish that only supports international chess cannot be used instead. The engine path is selected by `CHESS_ENGINE`; when unset or empty, the environment check looks for `fairy-stockfish` on `PATH`. When the main session calls the engine it also uses this variable or the same PATH command, without concatenating an in-package binary path.

## 3. Offline verification

In the same terminal with the Python environment activated and `CHESS_ENGINE` configured, run:

```bash
python -m pip check
python scripts/check_environment.py
python scripts/verify_package.py
python -m unittest discover -s tests -p 'test_*.py'
python scripts/round_tools.py --help
python scripts/round_tools_left.py --help
python scripts/internal_service.py --help
python scripts/internal_service_left.py --help
```

The environment check reports the actual import locations, dependency versions, 87 mesh references, the 14-joint order, model FK/gravity computation, engine UCI and `xiangqi` support, and the 44 legal moves from the initial position; it does not initialize DDS, connect cameras or start the robot service. The tests additionally check FK and gravity results for synthetic joint states, and IK. The source no longer imports `unitree_deploy`, and does not need old machine project paths or user-level skills.

Release-file verification does not traverse `.venv/`; the virtual environment is not part of the release manifest. Re-run the above verification after changing Python dependencies or the engine. Whether the target device can be controlled must still be confirmed per [First Game Setup](first_game_setup.md) — network, camera mapping, gripper orientation, scene and dual-arm init — and then this round's plan is generated and pre-checked per [round_plan](../prompts/round_plan.md).

## 4. Scope of standalone operation

After the above dependencies are installed, this package's planning, service, image capture, checkpoint and review entry points can be used directly without installing the whole `unitree_deploy`. Robot communication goes through the installed official SDK; therefore "standalone" means independent of the old project directory, not that no SDK, robot hardware or on-site setup is required. Model files are static resources needed for execution and must not be deleted along with third-party source code.

The main session is still responsible for confirming the position from current images, estimating grasp and place positions, matching the forecast / selecting moves and judging checkpoints; there is currently no controller CLI that automatically recognizes images and completes a whole round. Passing offline does not mean real-robot grasp-and-place has been verified; for the motion flow and known implementation differences see [runtime](runtime.md#implementation-verification-boundary).
