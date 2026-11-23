# R1-D 模型

[English](README.md) | 中文

| 文件 | 来源 | 用途 |
|---|---|---|
| `r1d.urdf` | unitree_ros `robots/r1d_description/r1d.urdf`（提交 31d9427，含 Dex1） | FK/IK 和预检；由 `config/runtime.json` 的 `model_urdf` 指定，SHA-256 必须等于 `config/robot.json` 的 `kinematic_model_sha256` |
| `r1d_arm_dynamics_reference.urdf` | 参考机器人实机拟合的臂动力学（连杆质量/质心），新机器人建议重新辨识 | 力矩前馈的重力项；由 `config/arm_torque_ff.json` 的 `model_urdf` 指定 |

`setup/arm-calibration` 完成后，本机的拟合结果会安装为 `r1d_arm_dynamics.urdf`，`config/arm_torque_ff.json` 的 `model_urdf` 随之指向它；`r1d_arm_dynamics_reference.urdf` 保留作参考。

这里只解析 URDF 的运动学和惯性参数，不需要网格文件（自碰撞检查用的网格在 `setup/arm-calibration/r1_d/assets/r1d/meshes/`）。
