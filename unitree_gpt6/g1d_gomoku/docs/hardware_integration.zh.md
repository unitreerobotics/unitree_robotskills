# 可选 G1 控制工具

[English](hardware_integration.md) | 中文

本目录封装了源项目的 DDS、四路相机、FK/IK、持续控制会话、返程规划和右臂归位代码。自主选点不导入这些模块；连接设备时才加载。硬件工具使用独立入口，并要求显式提供本地配置；发布控制命令需要 `--execute`。

相机到棋盘/机器人标定、棋子视觉识别和完整碰撞检测仍需现场上层 Agent 或集成方提供。

## 安装

先在 conda-forge 环境安装含 CasADi 绑定的 Pinocchio：

```bash
conda create -n gomoku -c conda-forge python=3.10 'pinocchio>=3,<5' 'casadi>=3.6,<4' pip
conda activate gomoku
```

再从仓库根目录安装硬件依赖：

```bash
python -m pip install -r unitree_gpt6/g1d_gomoku/requirements.txt
```

[Unitree SDK2 Python](../src/unitree_sdk2py/VENDORED.md) 已内置，提供 G1/Dex1 的 DDS 消息、发布器和 CRC；上述安装命令会一并安装其通信依赖。

G1 模型层已整合在 [unitree_gpt6/g1d_gomoku/models/g1/](../models/g1/README.zh.md)：包含 `G1_29_ArmIK`、`G1ArmConfig`、`G1_29_JointArmIndex`、G1 URDF 和 49 个网格文件，按包内相对位置加载。仓库源码已包含这些资源。来源、版本、哈希和许可证见模型目录的 `manifest.json` 与 `licenses/`。

`pinocchio`、`pinocchio.casadi` 和 `casadi` 必须在同一个 Python 环境可导入。
PyPI 的 `pin` wheel 不一定包含 CasADi 绑定，单独安装 `casadi` 也不会为 Pinocchio 补上它。
`preflight` 只检查顶层模块是否可发现，不能代替实际数值后端检查。

配置文件初始化后，运行以下检查；它不要求归位或场景文件，不连接 DDS 或相机：

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main \
  --config "$GOMOKU_HARDWARE_CONFIG" model-check
```

迁移时可用的依赖版本为 NumPy 2.2.6、SciPy 1.15.2、OpenCV 5.0.0.93、pyzmq 27.1.0、Pinocchio 4.1.0、CasADi 3.7.2、CycloneDDS 0.10.2。这里只记录封装环境，不能当作固件兼容性或新场景验证结论。

## 本地配置与只读准备

按 [README 的工作空间初始化步骤](../README.zh.md#2-创建仓库内的工作空间)，复制硬件模板到工作空间 `config/hardware.local.yaml`，填写相机地址和 DDS 网卡。工程不提供现场归位或抓放经验默认值。

后文 `config.local.yaml` 是本地配置路径占位符，实际使用 `"$GOMOKU_HARDWARE_CONFIG"`。路径相对 YAML 所在目录解析，所有进程共用同一部署配置和运行锁。

首次执行前，让机器人处于已核对的休息姿态，只读保存当前归位参考：

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main --config "$GOMOKU_HARDWARE_CONFIG" \
  pose --output "$GOMOKU_WORKSPACE/calibration/reset_right_state.json" state --grippers
```

这只订阅反馈，不发布运动命令。结构和控制范围见 [配置说明](../config/README.zh.md)。已有工作空间的旧默认参考不能作为当前测量值。经验库初始为空，成功后的写入见 [经验数据](experience.zh.md)。

Agent 还需根据当前场景生成 `runtime.scene_calibration` 指向的 `calibration/scene.json`，结构见 [场景几何数据](../config/README.zh.md#场景几何数据)。采集与标定完成后运行完全离线的预检：

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main --config config.local.yaml preflight
```

预检只读配置、归位参考和场景几何结构、依赖是否可找到，以及包内 URDF 和所有引用网格是否齐全，通过 `model_assets` 返回资源检查结果。不初始化模型、不订阅相机或 DDS、不发送控制命令。配置、依赖或资源缺失时返回非零退出码；`configuration_valid: true` 仍不代表连接、标定、关节可达性或机器人状态已经检查。实际数值库兼容性使用 `model-check` 检查；结果列出当前 Python 解释器、各数值模块的版本或原始导入错误。

## 工具入口

统一形式：

```text
python -m unitree_gpt6.g1d_gomoku.src.hardware.main --config CONFIG TOOL [参数]
```

| TOOL | 用途 |
| --- | --- |
| `model-check` | 导入数值库及 Pinocchio CasADi 绑定，实际加载内置模型，完全离线 |
| `preflight` | 完全离线检查配置、归位参考格式与依赖 |
| `state` | 只读双臂/腰部状态，`--grippers` 加夹爪 |
| `pose` | 原 `arm_pose` 接口：`state`、`ik`、`move`；显式 seed 的 IK 可离线运行 |
| `camera` | 四路相机快照或本地网页，沿用 `--snapshot DIR` 等参数 |
| `session` | 持续控制会话及阶段请求收件箱，默认只观察 |
| `control` | 单手棋的持续会话、阶段请求、反向来路返程准备和释放回撤 |
| `reset` | 按本地参考预计算归位；带 `--execute` 才发送动作 |

完整参数由各工具的 `--help` 给出。正常对弈使用 [自主执行流程](autonomous_play.zh.md) 的 `src.main` 轮次入口；本页为 Agent 底层工具接口，没有人工指定棋盘坐标的任务模式。

## 一手棋的控制约定

先以显式 `--execute` 启动唯一持续发布器；相对 `--root` 解析到 `runtime.directory` 下，后续命令复用该根目录：

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main --config config.local.yaml \
  control --root game --execute owner
# 在另一个终端只读查看状态：
python -m unitree_gpt6.g1d_gomoku.src.hardware.main --config config.local.yaml \
  control --root game status
```

`move`、`grip`、`reset`、`release` 和 `restore_initial` 请求也需要 `--execute`；`prepare_return` 只写计划，`status`、`observe`、`new_round` 不发关节命令。新一轮使用独立 `--root round_02 --owner-root game`，通过 `new_round` 恢复采图。不要为新一轮重复启动发布器。

使用当前四路图像和反馈判断棋局、定位桌面黑棋与落点。每个运动阶段由当前状态求解、核对完整路径后提交；`--review` 保存本轮视觉与净空检查说明。数值 IK 或路径采样不能代替环境碰撞检查。

抓子低位核对双指后慢收，短抬升确认单颗随爪再转运。放子前用 `prepare_return` 反向复用本轮已执行、核对过净空的高位路线，并连接本地归位参考；最终 XY 改变后重做连接。`release` 需要 Agent 显式提供本轮计算的 `--gripper OPENING`，必须有本轮用户确认，在同一控制会话中松爪、保存一次四路图像但不分析、连续抬升与归位。只读状态和准备计划不等于执行。

`prepare_return` 的 `--pose` 是同一 XY 和朝向的垂直抬升目标，另选 `--outbound-plan`（本轮已完成的高位计划）或 `--via-pose`（已审阅的高位出口）。先生成返程文件，再取得本轮松爪确认，由上层在本轮目录写入 `release_authorization.json`，内容为 `{"approved": true}`。随后 `release` 指定独立阶段名、`--return-plan` 和 `--review`，客户端会等待连续回撤结束。确认文件须在最终返程计划生成之后写入。客户端将其单次消费为该阶段的授权记录，并绑定释放与返程文件摘要；控制会话会核对该绑定。正常对弈由自主工作流在用户实际确认后记录授权，不由选点或规划步骤自动批准。

`restore_initial` 的双臂休息通道与 `prepare_return` 的高位净空条件均读取当前 `scene.json`，没有历史场景门限或指尖偏移默认值。更换场景后须重建标定、复核几何条件和整条路径。`return_result.json` 的 `reset_holding` 只表示计划插值结束并进入保持，不单独证明实测 TCP 到位或棋面落点正确。

控制层保留设备故障、过期反馈、速度、关节边界、左臂保持、发布器冲突及计划起点检查。客户端须与发布器使用同一配置；启动时遇到遗留待处理请求会拒绝运行，保护退出返回非零状态。停止发布不等于保持姿态或硬件急停，不能在未知状态重连并续发旧目标。停止和恢复语义需在现场单独验收；本包没有新增碰撞系统或独立安全控制器。

## 模块对应与验证

`src/hardware/` 保留源模块的职责：`dds_robot.py` 通信，`kinematics.py` 模型与 IK，`camera_web.py` 图像，`arm_pose.py` 计划/到位检查，`arm_session.py` 持续发布，`continuous_gomoku_session.py` 同会话释放回撤，`gomoku_live_control.py` 单轮编排，`reset_right.py` 归位。`common.py` 统一私有配置和运行目录，不在安装包目录写日志。

基础 CI 不安装硬件依赖；自主决策和轮次测试使用合成识别结果及假反馈，资产测试核对 URDF、网格与清单摘要。在装有数值依赖的环境中，模型测试还会实际加载包内模型、验证 FK/IK 和重力补偿，禁止导入外部模型包或设备 SDK。全部测试不启动 DDS/相机：

```bash
python -m unittest unitree_gpt6.g1d_gomoku.tests.test_hardware_contract -v
python -m unittest unitree_gpt6.g1d_gomoku.tests.test_bundled_model -v
```

操作及记录规则见 [operating_guide.zh.md](operating_guide.zh.md)，部署验收要求见仓库 [安全要求](../../../docs/safety.zh.md)。
