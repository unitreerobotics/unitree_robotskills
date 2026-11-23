# R1-D models

English | [中文](README.zh.md)

| File | Source | Purpose |
|---|---|---|
| `r1d.urdf` | unitree_ros `robots/r1d_description/r1d.urdf` (commit 31d9427, includes Dex1) | FK/IK and preflight; specified by `model_urdf` in `config/runtime.json`, and its SHA-256 must equal `kinematic_model_sha256` in `config/robot.json` |
| `r1d_arm_dynamics_reference.urdf` | Arm dynamics (link mass/centre of mass) fitted on the real reference robot; re-identification is recommended for a new robot | Gravity term of the torque feedforward; specified by `model_urdf` in `config/arm_torque_ff.json` |

After `setup/arm-calibration` is completed, the fitting result of the local machine is installed as `r1d_arm_dynamics.urdf`, and `model_urdf` in `config/arm_torque_ff.json` then points to it; `r1d_arm_dynamics_reference.urdf` is kept as a reference.

Only the kinematic and inertial parameters of the URDF are parsed here, and no mesh files are needed (the meshes used for the self-collision check are in `setup/arm-calibration/r1_d/assets/r1d/meshes/`).
