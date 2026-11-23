# 现场配置

[English](README.md) | 中文

随包附带的是**一台参考机器人在其场地上的标定结果**，用来说明格式和量级，并让离线测试能运行；同样的内容另存在 [reference/](reference/README.zh.md)。**在新环境开始下棋之前，必须先按 [setup/README.zh.md](../setup/README.zh.md) 完成 setup，用本机的结果更新这里的文件。**

```bash
python scripts/setup_status.py     # 每一项是参考值、已更新，还是已确认沿用参考值
```

| 文件 | 内容 | 读取者 | setup 方法 |
|---|---|---|---|
| `runtime.json` | 内置机器人运行时的位置（`runtime_root`，默认 `third_party/robot_runtime`）、experiment、网卡（`network_interface`）、相机主机、外部保持程序（`hold_program`，可选）、运动学 URDF（`model_urdf`，相对包根）、腰头增益 | `r1d_config`、`r1d_runtime`、`serve`、`handover` | `network_interface` 随包为空，按本机填写；腰头增益由 arm-calibration 安装 |
| `chess_camera_policy.json` | 规划和执行用哪路相机、腕流与物理手的对应 | `camera_policy`（服务、`round build`、`serial`） | `setup/camera-check` |
| `robot.json` | 几何 profile：TCP、双臂 init、scene、释放高度、头腰和升降参考、跟随阈值、`kinematic_model_sha256` | 服务、`round build`、`pose`、`goinit`、`handover`、`start` | `setup/chess-scene-parameters` |
| `chess_initial_pose.json` | init 位姿的原始记录；`robot.json` 的 init 由它派生 | 仅作记录，代码不读 | 同上，第 1 步 |
| `arm_torque_ff.json` | 臂力矩前馈：重力、摩擦、阻尼补偿、积分；`model_urdf` 相对包根 | 服务、`goinit` | `setup/arm-calibration`，再 `setup/joint-tracking` |
| `hold_joints.json` | 腰和头保持的摩擦、偏置、增益 | 服务、`goinit` | `setup/arm-calibration` |
| `head_camera_calibration.json` | 头部双目内参和眼在头外参 | `boardview`、`boardpoints`、`head_board_measure` | `setup/head-camera-calibration` |
| `board_geometry.json` | 棋盘中心、转角、格距、90 个交叉点、托盘位置 | `pose`、`round build`、`boardview`、`boardpoints`、`boardcal` | `setup/chess-scene-parameters`；之后棋盘移动用 `boardcal` |
| `xy_error_learning.json` | 误差样本的学习条件（如 25 mm 上限） | `history` | 可沿用 |
| `setup_status.json` | 经核对后确认沿用参考值的记录（按需生成） | `scripts/setup_status.py` | `setup_status.py --confirm` |

注意：

- `robot.json` 的内容参与 profile 摘要，摘要决定 `xy_error_context`。改动 profile 后，之前累计的落点误差样本不再适用。
- 改动 `robot.json` 或更换运动学 URDF 后，`kinematic_model_sha256` 必须等于当前 URDF 的哈希，否则服务拒绝启动。
- 包内没有机器相关的绝对路径：模型和模板路径都相对包根，参考配置里的证据路径以 `reference-robot:` 开头，只表示出处。
- 参考配置里的 `evidence`、`note`、`*_previous` 等字段是参考机器人的记录；setup 工具写入本机结果时会带上本机的依据。
- 离线测试按参考配置编写，更新配置后个别依赖具体数值的用例可能需要随之调整。
