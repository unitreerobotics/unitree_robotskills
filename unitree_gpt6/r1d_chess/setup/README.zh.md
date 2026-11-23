# Setup：新机器人、新场地上线

[English](README.md) | 中文

`config/` 里随包附带的是一台参考机器人在其场地上的标定结果（同样的内容另存在 `config/reference/` 供对照）。**在新环境开始下棋之前，必须先完成下面的 setup，用本机的结果更新 `config/`。** 相机内外参、连杆质量和摩擦、棋盘位置都因机器和场地而异，沿用参考值可能导致抓放偏差或跟随保护误触发。

随时查看进度（离线，不连机器人）：

```bash
python scripts/setup_status.py            # 每一项：参考值 / 已更新 / 已确认沿用参考值
python scripts/setup_status.py --require  # 还有必做项没完成时退出码为 1
```

agent 在第一次打开运动（`serve --allow-motion`、`enable_control`、`goinit --execute`、`start --execute`）之前要先运行它；有必做项仍是参考值时不要走子，除非用户明确决定沿用（见文末“沿用参考值”）。

## 顺序

| 顺序 | 内容 | 是否运动 | 更新的配置 |
|---|---|---|---|
| 0 | [本机环境](#第-0-步本机环境)：Python、引擎、`config/runtime.json` | 否 | `runtime.json` |
| 1 | [camera-check](camera-check/README.zh.md)：确认三路相机流和物理左右手的对应 | 否 | `chess_camera_policy.json` |
| 2 | [chess-scene-parameters](chess-scene-parameters/README.zh.md) 第 1 步：记录初始姿态、锁定头姿 | 只读 | `robot.json`、`chess_initial_pose.json` |
| 3 | [arm-calibration](arm-calibration/README.zh.md)：双臂动力学（连杆质量、质心、摩擦）和腰头保持参数的实机辨识 | 双臂、腰、头大范围运动 | `arm_torque_ff.json`、`hold_joints.json`、`runtime.json` 的腰头增益、动力学 URDF |
| 4 | [joint-tracking](joint-tracking/README.zh.md)：在 Runtime 链路上验证跟随误差，必要时微调力矩前馈 | 双臂运动 | `arm_torque_ff.json` |
| 5 | [head-camera-calibration](head-camera-calibration/README.zh.md)：头部双目内参、基线、安装外参、头关节修正 | 阶段 2 头部运动 | `head_camera_calibration.json` |
| 6 | chess-scene-parameters 第 2–7 步：TCP、桌面、棋盘、释放高度 | 第 3 步转头，其余只读 | `robot.json`、`board_geometry.json` |
| 7 | joint-tracking 第 7 步：在象棋服务的发送路径上确认一次 | 双臂运动 | — |
| 8 | 空载路径验收：挑一步真实走法做完整预检，再不夹棋子执行一次 | 双臂运动 | — |

之后棋盘被移动时只需重测棋盘几何：`bash r1d boardcal`，见 [board_position](../board_position/README.zh.md)。

各项的依赖关系：头部相机标定要用到锁定的头姿和（最好）已整定的前馈；场景参数要用到头部相机标定；关节精度标定只依赖初始姿态。

## 第 0 步：本机环境

本包不含任何机器相关的路径，下面几项要按本机填写：

| 项 | 在哪里设 | 说明 |
|---|---|---|
| Python 解释器 | 环境变量 `R1D_CHESS_PYTHON`，或在包根建 `.venv/` | `bash r1d` 依次找 `R1D_CHESS_PYTHON`、`<包根>/.venv/bin/python`、PATH 上的 `python3` |
| 象棋引擎 | 环境变量 `CHESS_ENGINE`，或把 `fairy-stockfish` 放到 PATH | 必须是 `largeboards=yes` 构建 |
| DDS 网卡 | `config/runtime.json` 的 `network_interface` | 连机器人的网卡名 |
| 相机主机 | `config/runtime.json` 的 `camera_host` | 机器人上图像服务的地址，默认是机器人内网的 192.168.123.164 |
| 外部保持程序（可选） | `config/runtime.json` 的 `hold_program` | 现场另有保持机器人姿态的 systemd 单元时，填它 ExecStart 里的程序路径，`handover` 才会接管它；留空则只认本包的 goinit 单元 |
| Runtime 服务 | `python scripts/install_runtime_service.py` | 生成 systemd 用户单元 `r1d-runtime.service`，从本包运行内置的机器人运行时（`third_party/robot_runtime`，唯一的 DDS 写入者）。默认只打印单元内容，`--write` 才写入；已有同名单元时不会覆盖。Runtime 与象棋服务用同一个 Python 环境即可，也可用 `--python` 另行指定 |
| 关节标定工具的连接 | `setup/arm-calibration/r1_d/config.json` 的 `ip`、`interface`、`transport` | 只在 arm-calibration 时用 |

填好后运行 `python scripts/check_environment.py`，必需项全部通过再继续；再用 `bash r1d runtime config` 确认 Runtime 配置能解析（不连机器人）。`network_interface` 留空时，象棋服务和需要 Runtime 的工具会直接报错。Runtime 本身不用配置：`runtime.json` 的 `runtime_root` 默认指向包内的 `third_party/robot_runtime`，`camera_enabled` 保持 `false`（象棋服务自己取图）。

## 通用规则

- 每一次会让机器人运动的命令都要先得到操作者明确授权，并确认周围和手臂运动范围内没有人和障碍物。
- 不要为了让测试通过而放宽跟随误差、温度、反馈过期等保护阈值。
- 标定期间不能同时运行象棋服务或其他控制程序。arm-calibration 直接走 DDS，还要求 Runtime 也停掉。
- 原始数据和中间结果放在 `runs/<名称>/` 下，写进配置的 `evidence` 字段指向它。
- 命令都在包根下运行，`python` 指已安装依赖的环境（与 `R1D_CHESS_PYTHON` 相同）。

## 目录

```text
setup/
├── README.zh.md                   本文件
├── _common.py                  公共代码：定位包根、读 runtime.json、读手臂 PD 增益
├── camera-check/               相机流与物理左右手的对应
├── chess-scene-parameters/     初始姿态、TCP、场景、棋盘、释放高度
├── head-camera-calibration/    头部相机标定（采集、检测、内参、外参解算）
├── joint-tracking/             Runtime 链路上的跟随误差对比和前馈微调
└── arm-calibration/            双臂动力学与腰头参数辨识（自带 r1_d 直连工具）
```

## 沿用参考值

某一项经核对后确实可以沿用参考值时（例如相机映射核对后与参考相同），记录依据：

```bash
python scripts/setup_status.py --confirm camera_mapping --note "三路流逐一遮挡核对，左右腕与参考一致"
```

确认记录绑定文件内容，文件之后有改动会自动失效。头部相机标定、初始姿态、场景和棋盘几何不应沿用参考值。

## 参考值

`config/reference/` 是参考机器人的结果，只用于对照量级，不要修改，也不要当作新机器人的初值去迭代。离线测试按参考配置编写，更新 `config/` 后个别依赖具体数值的用例可能需要随之调整。
