# robot_runtime: robot runtime

English | [中文](README.zh.md)

The robot runtime is **the only process that sends commands to the robot** in the chess pipeline: it owns DDS exclusively, reads motor feedback, receives joint targets, and writes LowCmd at a fixed rate; the chess service, `goinit`, and the setup tools all communicate with it through local IPC and do not send commands directly themselves.

| Content | Count | Description |
|---|---|---|
| `src/robot_runtime/` | 64 modules | Configuration parsing, runtime server and IPC, Unitree SDK2 driver, R1-A7-D body and Dex1 components, data structures |
| `configs/` | 8 YAML files | Experiment, body joint groups, gripper, chassis, and sensor definitions for R1-A7-D + Dex1 |
| `LICENSE` | — | Apache License 2.0 |

## Usage

No separate installation is needed. `runtime_root` in `config/runtime.json` points to this directory (relative to the package root):

```bash
bash r1d runtime config                              # parse the configuration without connecting to the robot
python scripts/install_runtime_service.py --write    # generate the systemd user unit r1d-runtime.service
systemctl --user start r1d-runtime.service           # start; sends no commands to the robot before a control session is established
```

For dependencies, see `requirements.txt` in the package root (`logging_mp`, `msgspec`, `PyYAML`, `pyzmq`, `cyclonedds`) and the R1-D SDK `unitree_sdk2py`.

## Scope

It contains only the parts actually used by this one configuration: R1-A7-D + Dex1, the `mobile_wbc` control mode, and cameras disabled:

- It does not include teleoperation policies, retargeting, recording, simulation, or gravity compensation specific to other robot models.
- It does not include camera drivers: the chess service fetches images directly with `r1d_chess.cameras.r1d_cameras`, so `camera_enabled` in `config/runtime.json` must stay `false`.

## Origin and changes

The code in this directory is taken from an upstream robot control project licensed under Apache License 2.0 (commit `54690e0c30cde76f95a44bde228d454846201cb7`), with three changes:

1. Only the modules and configurations within the scope above are kept.
2. The Python package name is changed to `robot_runtime`, and the package names, thread names, and IPC endpoint prefixes in the code are renamed accordingly; the logic is unchanged.
3. `src/robot_runtime/runtime/robot_hardware.py` no longer imports the camera driver at the top of the module (it is imported only for type checking instead), so it can be loaded without the camera client library installed.
