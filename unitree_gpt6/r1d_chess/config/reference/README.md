# Calibration results of the reference robot

English | [中文](README.zh.md)

This is the configuration of one reference robot at its site, identical to the initial content of the `config/*.json` files shipped with the package. It has only two purposes: comparing orders of magnitude, and letting `python scripts/setup_status.py` determine whether each item in `config/` has been updated with the results of the local machine.

- Do not modify this directory, and do not use these values as initial values for a new robot.
- After each item is completed in a new environment as described in [setup/README.md](../../setup/README.md), the results are written to `config/` one level up, and this directory stays unchanged.
- The dynamics URDF referenced by `arm_torque_ff.json` is `assets/models/r1d/r1d_arm_dynamics_reference.urdf` (the fitting result of the reference robot).
