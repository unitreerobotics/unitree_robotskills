# G1 Model Assets

English | [中文](README.zh.md)

`g1/` keeps the URDF and meshes required for this project's kinematics, and they must be carried with the package when migrating. The assets come from the original G1 description; the release file hashes are kept in the release manifest, and the upstream model notes are in [g1/README](g1/README.md). `g1_body29_hand14.urdf` is modified from Unitree's G1 model with hands and must not be casually replaced by a URDF with different joints or a different end-effector definition.

The construction code is `src/g1d_chess/robot/g1_model.py`: it locks the lower limbs, waist and fingers, keeps the physical left 7 plus right 7 joints, and the end effector is offset 50mm along the wrist yaw's local X. Synthetic-state FK/gravity results are checked against the original model in tests; these are not measured states of this machine. For dependencies and the runtime environment see the [environment setup guide](../../docs/environment_setup.md).
