# Site configuration

English | [中文](README.zh.md)

The files shipped with the package are **the calibration results of one reference robot at its site**. They show the format and the order of magnitude and let the offline tests run; the same content is also stored in [reference/](reference/README.md). **Before playing chess in a new environment, you must first complete setup as described in [setup/README.md](../setup/README.md) and update the files here with the results of the local machine.**

```bash
python scripts/setup_status.py     # whether each item is a reference value, updated, or confirmed to keep the reference value
```

| File | Content | Read by | Setup method |
|---|---|---|---|
| `runtime.json` | Location of the built-in robot runtime (`runtime_root`, default `third_party/robot_runtime`), experiment, network interface (`network_interface`), camera host, external hold program (`hold_program`, optional), kinematic URDF (`model_urdf`, relative to the package root), waist and head gains | `r1d_config`, `r1d_runtime`, `serve`, `handover` | `network_interface` is shipped empty, fill it in for the local machine; the waist and head gains are installed by arm-calibration |
| `chess_camera_policy.json` | Which camera streams are used for planning and execution, and the mapping between wrist streams and physical arms | `camera_policy` (service, `round build`, `serial`) | `setup/camera-check` |
| `robot.json` | Geometry profile: TCP, dual-arm init, scene, release heights, head, waist, and lift references, tracking thresholds, `kinematic_model_sha256` | service, `round build`, `pose`, `goinit`, `handover`, `start` | `setup/chess-scene-parameters` |
| `chess_initial_pose.json` | Original record of the init pose; the init in `robot.json` is derived from it | Record only, not read by the code | Same as above, step 1 |
| `arm_torque_ff.json` | Arm torque feedforward: gravity, friction, damping compensation, integral; `model_urdf` is relative to the package root | service, `goinit` | `setup/arm-calibration`, then `setup/joint-tracking` |
| `hold_joints.json` | Friction, bias, and gains for holding the waist and head | service, `goinit` | `setup/arm-calibration` |
| `head_camera_calibration.json` | Head stereo intrinsics and eye-on-head extrinsics | `boardview`, `boardpoints`, `head_board_measure` | `setup/head-camera-calibration` |
| `board_geometry.json` | Board centre, rotation angle, square spacing, 90 intersections, tray positions | `pose`, `round build`, `boardview`, `boardpoints`, `boardcal` | `setup/chess-scene-parameters`; afterwards, when the board moves, use `boardcal` |
| `xy_error_learning.json` | Learning conditions for error samples (such as the 25 mm upper limit) | `history` | Can be kept as is |
| `setup_status.json` | Record of items confirmed, after verification, to keep the reference value (generated on demand) | `scripts/setup_status.py` | `setup_status.py --confirm` |

Notes:

- The content of `robot.json` is part of the profile digest, and the digest determines `xy_error_context`. After the profile is changed, the previously accumulated landing error samples no longer apply.
- After changing `robot.json` or replacing the kinematic URDF, `kinematic_model_sha256` must equal the hash of the current URDF, otherwise the service refuses to start.
- The package contains no machine-specific absolute paths: model and template paths are all relative to the package root, and evidence paths in the reference configuration start with `reference-robot:` and only indicate the origin.
- Fields such as `evidence`, `note`, and `*_previous` in the reference configuration are records of the reference robot; when the setup tools write the local results, they include the basis from the local machine.
- The offline tests are written against the reference configuration; after the configuration is updated, a few test cases that depend on specific values may need to be adjusted accordingly.
