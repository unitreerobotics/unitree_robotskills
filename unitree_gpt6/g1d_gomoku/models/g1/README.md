# Bundled Unitree G1 model resources

English | [中文](README.zh.md)

Gomoku's hardware adapter carries the G1 model layer it needs. This directory
contains the 14-DoF arm reduction, DDS arm indices and gains, the
`g1_body29_hand14.urdf`, and the 49 mesh files referenced by that URDF.

The Python model layer is exposed through `unitree_gpt6.g1d_gomoku.models.g1` and is used
directly by `g1d_gomoku`; it does not import or require an external
`unitree_deploy` installation. Pinocchio and CasADi remain normal hardware
dependencies because they perform FK, IK and gravity calculations.

The URDF and meshes are copied from Unitree Robotics' `xr_teleoperate`
`assets/g1` tree. The arm reduction, frame offsets, and consumed configuration
values are adapted from the `unitree_deploy` 0.0.5 sources listed in
`manifest.json`. That source package declares BSD 3-Clause; the model assets
carry the Apache-2.0 notice from their upstream repository. `manifest.json`
records upstream revisions and SHA256 values; `licenses/` contains the
applicable notices. Run `validate_assets()` to check the installed inventory
without connecting to a robot.

The full model's legs, waist and finger joints are locked at zero for the
14-joint arm reduction; the skill then accounts for the measured waist in
FK/IK and uses the full model for gravity compensation. `L_ee` and `R_ee`
remain 0.05 m along each wrist yaw joint's local X axis. These definitions,
joint limits, inertias, indices and gains match the previously used model.
The bundled Python layer omits unused viewers, filters, pickle caches and the
old solver's fallback behavior; the skill retains its existing strict solver
and raises on failure. No model assets are downloaded or rewritten at runtime.
