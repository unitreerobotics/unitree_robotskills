# 参考机器人的标定结果

[English](README.md) | 中文

这里是一台参考机器人在其场地上的配置，与随包附带的 `config/*.json` 初始内容相同。用途只有两个：对照量级，以及让 `python scripts/setup_status.py` 判断 `config/` 里的各项是否已经用本机结果更新。

- 不要修改这个目录，也不要把这些数值当作新机器人的初值。
- 新环境按 [setup/README.zh.md](../../setup/README.zh.md) 完成各项后，结果写进上一级的 `config/`，这里保持不变。
- `arm_torque_ff.json` 引用的动力学 URDF 是 `assets/models/r1d/r1d_arm_dynamics_reference.urdf`（参考机器人的拟合结果）。
