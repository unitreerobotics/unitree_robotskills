# 内置 Unitree G1 模型资源

[English](README.md) | 中文

五子棋的硬件适配层自带所需的 G1 模型层。本目录包含 14-DoF 手臂简化模型、DDS 手臂关节索引与增益、`g1_body29_hand14.urdf`，以及该 URDF 引用的 49 个网格文件。

Python 模型层通过 `unitree_gpt6.g1d_gomoku.models.g1` 提供，由 `g1d_gomoku` 直接使用；它不导入也不需要外部安装的 `unitree_deploy`。Pinocchio 和 CasADi 仍是常规硬件依赖，因为它们负责 FK、IK 和重力计算。

URDF 和网格复制自 Unitree Robotics `xr_teleoperate` 的 `assets/g1` 目录树。手臂简化模型、坐标系偏移和所使用的配置值改编自 `manifest.json` 中列出的 `unitree_deploy` 0.0.5 源码。该源码包声明 BSD 3-Clause 许可证；模型资产附带其上游仓库的 Apache-2.0 声明。`manifest.json` 记录上游修订版本和 SHA256 值；`licenses/` 包含适用的许可声明。运行 `validate_assets()` 可在不连接机器人的情况下检查已安装的资源清单。

对 14 关节手臂简化模型，完整模型的腿部、腰部和手指关节锁定为零；技能随后在 FK/IK 中计入实测腰角，并使用完整模型进行重力补偿。`L_ee` 和 `R_ee` 仍位于各腕部 yaw 关节局部 X 轴方向 0.05 m 处。这些定义、关节限位、惯量、索引和增益与此前使用的模型一致。内置 Python 层省略了未使用的可视化工具、滤波器、pickle 缓存和旧求解器的回退行为；技能保留现有的严格求解器，求解失败时抛出异常。运行时不会下载或改写任何模型资产。
