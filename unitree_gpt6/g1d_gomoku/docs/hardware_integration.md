# Optional G1 control tools

English | [中文](hardware_integration.zh.md)

This directory wraps the source project's code for DDS, the four cameras, FK/IK, continuous control sessions, return-path planning and right-arm return home. Autonomous point selection does not import these modules; they are loaded only when connecting to devices. The hardware tools use a separate entry point and require an explicitly provided local configuration; publishing control commands requires `--execute`.

Camera-to-board/robot calibration, visual recognition of stones and full collision checking must still be provided by the on-site upper-level agent or the integrator.

## Installation

First install Pinocchio with the CasADi binding in a conda-forge environment:

```bash
conda create -n gomoku -c conda-forge python=3.10 'pinocchio>=3,<5' 'casadi>=3.6,<4' pip
conda activate gomoku
```

Then install the optional dependencies from the repository root:

```bash
python -m pip install '.[gomoku-hardware]'
```

[Unitree SDK2 Python](../src/unitree_sdk2py/VENDORED.md) is bundled and provides the DDS messages, publishers and CRC for G1/Dex1; the installation command above also installs its communication dependencies.

The G1 model layer is integrated in [unitree_gpt6/g1d_gomoku/models/g1/](../models/g1/README.md): it includes `G1_29_ArmIK`, `G1ArmConfig`, `G1_29_JointArmIndex`, the G1 URDF and 49 mesh files, loaded by their relative locations inside the package. The installed package includes these resources. For provenance, versions, hashes and licenses see `manifest.json` and `licenses/` in the model directory.

`pinocchio`, `pinocchio.casadi` and `casadi` must be importable in the same Python environment.
The PyPI `pin` wheel does not necessarily include the CasADi binding, and installing `casadi` separately does not add it to Pinocchio.
`preflight` only checks whether the top-level modules can be found and cannot replace a check of the actual numerical backend.

After the configuration file is initialized, run the following check; it does not require rest-pose or scene files and does not connect to DDS or cameras:

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main \
  --config "$GOMOKU_HARDWARE_CONFIG" model-check
```

The dependency versions that worked during migration are NumPy 2.2.6, SciPy 1.15.2, OpenCV 5.0.0.93, pyzmq 27.1.0, Pinocchio 4.1.0, CasADi 3.7.2 and CycloneDDS 0.10.2. This only records the packaging environment and must not be taken as a conclusion about firmware compatibility or validation of a new scene.

## Local configuration and read-only preparation

Following the [workspace initialization steps in the README](../README.md#2-create-a-workspace-inside-the-repository), copy the hardware template to the workspace's `config/hardware.local.yaml` and fill in the camera address and the DDS network interface. The project provides no on-site rest-pose or pick-and-place experience defaults.

Below, `config.local.yaml` is a placeholder for the local configuration path; in practice use `"$GOMOKU_HARDWARE_CONFIG"`. Paths are resolved relative to the directory containing the YAML, and all processes share the same deployment configuration and runtime lock.

Before the first run, put the robot in a checked rest pose and save the current rest-pose reference read-only:

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main --config "$GOMOKU_HARDWARE_CONFIG" \
  pose --output "$GOMOKU_WORKSPACE/calibration/reset_right_state.json" state --grippers
```

This only subscribes to feedback and does not publish motion commands. For the structure and control ranges see the [configuration guide](../config/README.md). Old default references in an existing workspace must not be used as current measurements. The experience library is initially empty; for writing after success see [experience data](experience.md).

The agent also needs to generate, from the current scene, the `calibration/scene.json` pointed to by `runtime.scene_calibration`; for its structure see [Scene geometry data](../config/README.md#scene-geometry-data). After capture and calibration are complete, run the fully offline preflight:

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main --config config.local.yaml preflight
```

Preflight only reads the configuration, the rest-pose reference and the scene geometry structure, checks whether dependencies can be found, and checks whether the bundled URDF and all referenced meshes are complete, returning the resource check result in `model_assets`. It does not initialize the model, does not subscribe to cameras or DDS, and does not send control commands. Missing configuration, dependencies or resources return a non-zero exit code; `configuration_valid: true` still does not mean that connectivity, calibration, joint reachability or robot state have been checked. Actual numerical library compatibility is checked with `model-check`; its result lists the current Python interpreter and the version or raw import error of each numerical module.

## Tool entry points

General form:

```text
python -m unitree_gpt6.g1d_gomoku.src.hardware.main --config CONFIG TOOL [ARGS]
```

| TOOL | Purpose |
| --- | --- |
| `model-check` | Imports the numerical libraries and the Pinocchio CasADi binding and actually loads the bundled model; fully offline |
| `preflight` | Fully offline check of configuration, rest-pose reference format and dependencies |
| `state` | Read-only state of both arms/waist; `--grippers` adds the grippers |
| `pose` | Original `arm_pose` interface: `state`, `ik`, `move`; IK with an explicit seed can run offline |
| `camera` | Snapshots of the four cameras or a local web page; keeps parameters such as `--snapshot DIR` |
| `session` | Continuous control session and stage request inbox; observe-only by default |
| `control` | Continuous session for a single move, stage requests, return preparation along the reversed approach path, and release with retreat |
| `reset` | Precomputes return home from the local reference; sends motion only with `--execute` |

Full parameters are given by each tool's `--help`. Normal play uses the `src.main` round entry point of the [autonomous execution workflow](autonomous_play.md); this page is the agent's low-level tool interface, and there is no task mode for manually specifying board coordinates.

## Control conventions for one move

First start the single continuous publisher with an explicit `--execute`; a relative `--root` is resolved under `runtime.directory`, and later commands reuse that root directory:

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main --config config.local.yaml \
  control --root game --execute owner
# In another terminal, check status read-only:
python -m unitree_gpt6.g1d_gomoku.src.hardware.main --config config.local.yaml \
  control --root game status
```

The `move`, `grip`, `reset`, `release` and `restore_initial` requests also require `--execute`; `prepare_return` only writes a plan, and `status`, `observe` and `new_round` do not send joint commands. A new round uses a separate `--root round_02 --owner-root game` and resumes image capture through `new_round`. Do not start the publisher again for a new round.

Use the current four camera images and feedback to judge the position and locate the black stones on the table and the landing point. Each motion stage is solved from the current state and submitted after checking the complete path; `--review` saves this round's notes on the visual and clearance checks. Numerical IK or path sampling cannot replace environmental collision checking.

When picking, check both fingers at the low position before closing slowly, and confirm with a short lift that exactly one stone follows the gripper before transferring. Before placing, use `prepare_return` to reuse in reverse this round's executed, clearance-checked high-level route and connect it to the local rest-pose reference; if the final XY changes, redo the connection. `release` requires the agent to explicitly provide this round's computed `--gripper OPENING` and requires this round's user confirmation; in the same control session it releases the gripper, saves the four camera images once without analyzing them, and lifts and returns home continuously. Read-only status and prepared plans are not execution.

The `--pose` of `prepare_return` is a vertical lift target at the same XY and orientation; additionally choose `--outbound-plan` (this round's completed high-level plan) or `--via-pose` (a reviewed high-level exit). First generate the return file, then obtain this round's release confirmation, after which the upper layer writes `release_authorization.json` in this round's directory with the content `{"approved": true}`. Then `release` specifies a separate stage name, `--return-plan` and `--review`, and the client waits for the continuous retreat to finish. The confirmation file must be written after the final return plan has been generated. The client consumes it once as the authorization record for that stage and binds it to the digests of the release and return files; the control session checks this binding. In normal play the autonomous workflow records authorization after the user's actual confirmation; it is not approved automatically by the point-selection or planning steps.

The dual-arm rest corridor of `restore_initial` and the high-level clearance conditions of `prepare_return` both read the current `scene.json`; there are no historical scene thresholds or fingertip offset defaults. After changing the scene, calibration must be rebuilt and the geometric conditions and the entire path re-checked. `reset_holding` in `return_result.json` only means that plan interpolation has finished and holding has begun; it does not by itself prove that the measured TCP has arrived or that the landing point on the board is correct.

The control layer keeps the checks for device faults, stale feedback, velocity, joint limits, left-arm holding, publisher conflicts and plan start points. The client must use the same configuration as the publisher; on startup it refuses to run if leftover pending requests are found, and a protective exit returns a non-zero status. Stopping publishing is neither pose holding nor a hardware emergency stop; do not reconnect in an unknown state and continue sending old targets. Stop and resume semantics need separate on-site acceptance; this package adds no collision system or independent safety controller.

## Module mapping and verification

`src/hardware/` keeps the responsibilities of the source modules: `dds_robot.py` communication, `kinematics.py` model and IK, `camera_web.py` images, `arm_pose.py` plans/arrival checks, `arm_session.py` continuous publishing, `continuous_gomoku_session.py` same-session release and retreat, `gomoku_live_control.py` single-round orchestration, `reset_right.py` return home. `common.py` unifies private configuration and runtime directories and does not write logs into the installed package directory.

Base CI does not install hardware dependencies; autonomous decision and round tests use synthetic recognition results and fake feedback, and asset tests check the URDF, meshes and manifest digests. In an environment with the numerical dependencies, the model tests also actually load the bundled model and verify FK/IK and gravity compensation, and importing external model packages or device SDKs is forbidden. None of the tests start DDS/cameras:

```bash
python -m unittest unitree_gpt6.g1d_gomoku.tests.test_hardware_contract -v
python -m unittest unitree_gpt6.g1d_gomoku.tests.test_bundled_model -v
```

For operating and record rules see [operating_guide.md](operating_guide.md); for deployment acceptance requirements see the repository's [safety requirements](../../../docs/safety.md).
