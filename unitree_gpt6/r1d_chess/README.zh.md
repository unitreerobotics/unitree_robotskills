# R1-D Chess

[English](README.md) | 中文

通过 [SKILL.zh.md](SKILL.zh.md)，让具备终端执行与图片查看能力的模型，控制 Unitree R1-D（R1-A7-D，Dex1 夹爪）完成中国象棋的单轮规划、左右手抓放、吃子和结果复盘。

模型负责看图确认棋局、调用象棋引擎选着、选择执行手和抓放姿态、判断每个视觉门禁；Python 服务负责完整计划预检、一次性 token 和运动执行。本包没有自动识图并下完整盘棋的总控 CLI，也不包含机器人端的图像服务。

包内带有新机器人、新场地上线用的 setup 工具（相机检查、头部相机标定、机械臂精度标定、下棋环境参数和棋盘标定），以及下发命令用的机器人运行时。

## 安装与配置

基线为 Linux x86_64、Python 3.10。保留完整目录，进入本技能目录后执行：

```bash
python3.10 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m pip install --no-deps "unitree_sdk2py @ git+https://github.com/unitreerobotics/unitree_sdk2_python.git@814556d15970dd2ecf1c9984e845ca02ab07e206"
cp .env.example .env
```

也可以用自己已装好依赖的 Python 环境。统一入口 `bash r1d <子命令>` 依次使用 `R1D_CHESS_PYTHON`、`<技能目录>/.venv/bin/python`、PATH 上的 `python3`；每个子命令等价于 `python scripts/<入口>.py`。

需要按本机准备的只有三项：

| 项 | 在哪里设 |
|---|---|
| 象棋引擎 Fairy-Stockfish（必须用 `largeboards=yes` 构建） | 环境变量 `CHESS_ENGINE`，或把 `fairy-stockfish` 放到 PATH |
| 连机器人的 DDS 网卡 | `config/runtime.json` 的 `network_interface` |
| 机器人上图像服务的地址 | `config/runtime.json` 的 `camera_host`（默认是机器人内网地址） |

`.env` 不会被程序自动加载；支持的环境变量见 [.env.example](.env.example)。引擎构建和其余细节见 [操作手册](docs/operation_manual.zh.md)“入口和运行环境”。配置好后运行离线检查：

```bash
python scripts/check_environment.py     # 依赖、引擎、模型哈希、配置；不连机器人
bash r1d                                # 列出子命令
```

### 机器人运行时

机器人运行时（Runtime）已内置在 [third_party/robot_runtime](third_party/robot_runtime/README.zh.md)，不需要另外获取。它是唯一向机器人下发命令的进程，以 systemd 用户单元运行，象棋服务通过本机 IPC 调它：

```bash
python scripts/install_runtime_service.py            # 打印单元内容，不写入
python scripts/install_runtime_service.py --write    # 生成 r1d-runtime.service
```

### 新环境上线（setup）

**`config/` 里随包附带的是一台参考机器人的标定结果**（另存于 `config/reference/`）。相机内外参、连杆质量和摩擦、棋盘位置都因机器和场地而异，所以在新环境开始下棋之前，必须先按 [setup/README.zh.md](setup/README.zh.md) 完成 setup，用本机结果更新 `config/`：

| 内容 | 流程 | 结果写入 |
|---|---|---|
| 相机检查（头部和腕部相机流、腕流与物理手的对应） | [setup/camera-check](setup/camera-check/README.zh.md) | `config/chess_camera_policy.json` |
| 机械臂精度标定（双臂动力学、腰头保持参数） | [setup/arm-calibration](setup/arm-calibration/README.zh.md)、[setup/joint-tracking](setup/joint-tracking/README.zh.md) | `config/arm_torque_ff.json`、`config/hold_joints.json` |
| 头部相机标定（内参、基线、眼在头外参） | [setup/head-camera-calibration](setup/head-camera-calibration/README.zh.md) | `config/head_camera_calibration.json` |
| 下棋环境参数（初始位姿、TCP、场景、释放高度） | [setup/chess-scene-parameters](setup/chess-scene-parameters/README.zh.md) | `config/robot.json` |
| 棋盘标定（板面高度、中心、转角、90 个交叉点） | setup/chess-scene-parameters 第 3–4 步；之后棋盘移动用 `bash r1d boardcal`（[board_position](board_position/README.zh.md)） | `config/board_geometry.json` |

```bash
python scripts/setup_status.py            # 每一项是参考值、已在本机更新，还是已确认沿用参考值
python scripts/setup_status.py --require  # 还有必做项没完成时退出码为 1
```

setup 中会让机器人运动的步骤，都要操作者明确授权并确认周围安全。

## 通过 skill 使用

保留整个目录，让模型读取 [SKILL.zh.md](SKILL.zh.md)。例如：

> 读取 `/实际部署路径/r1d_chess/SKILL.zh.md`，检查环境和 setup 进度；本次只做离线检查，不连接机器人。

setup 完成且准备实际走棋时：

> 读取 `/实际部署路径/r1d_chess/SKILL.zh.md`，按当前棋盘完成红方这一轮走棋，核对 source/target 和实际落点。

skill 根据任务选择环境与 setup、启动服务与接管、读棋与棋盘标定、单轮执行或故障回撤流程，引用本目录的文档与脚本。仅复制 `SKILL.zh.md` 不会携带运行代码、模型和配置。

正常单轮遵循 [round_plan](prompts/round_plan.zh.md)，详细命令见 [操作手册](docs/operation_manual.zh.md)。执行中的门禁按“`next` → 看本次两图 → 提交 → `next`”处理；机器人持子时不要研究代码或补装依赖。

## 目录

```text
r1d_chess/
├── README.zh.md                 安装与使用入口
├── SKILL.zh.md                  模型任务入口与按需文档路由
├── r1d                       统一入口：bash r1d <子命令>，分发到 scripts/
├── .env.example              已支持的环境变量模板
├── requirements.txt          Python 依赖
├── config/                   现场配置；reference/ 为参考机器人的结果
├── setup/                    新机器人、新场地上线：相机检查、标定、场景参数
├── src/r1d_chess/            实现：robot/ chess/ cameras/ board/
├── scripts/                  可执行入口（薄包装）、环境检查、setup 进度、运行时单元安装
├── third_party/robot_runtime/ 内置的机器人运行时（下发命令的唯一通道）
├── assets/                   运动学和动力学模型、JSON 表单模板、棋子图例
├── prompts/                  单轮提示词 round_plan
├── docs/                     操作手册、脚本清单与专题说明
├── board_position/           读棋辅助图、棋盘 90 点实测与重新标定
└── tests/                    离线测试及样本
```

运行时在 `runs/` 中生成图像、计划、检查点、服务日志和复盘；该目录及 `.env`、虚拟环境不进入版本管理。脚本名称、输入输出和模块对应关系见 [脚本清单](docs/round_script_inventory.zh.md)。

## 离线验证

```bash
python scripts/check_environment.py
python -B -m unittest discover -s tests -p 'test_*.py'
```

这些检查不连接相机、不初始化机器人控制，也不操作机器人；测试用模拟 transport。它们不能代替 setup、本轮完整路径预检和走后头图确认。

## 在新环境中首次使用时注意

- 内置的机器人运行时和 setup 流程都做过离线验证；首次连接新机器人时，先用 `bash r1d runtime status` 和只读的 `bash r1d client get_state` 确认反馈正常，再接管。
- setup 完成后先做一次空载路径验收（完整预检后不夹棋子执行一步），再做真实抓放，并以走后头图确认落格。
- 个别参数按参考机器人给出，新机器人上可能需要调整，见 [操作手册](docs/operation_manual.zh.md)“验证与能力边界”。
- 离线测试按参考配置编写；用本机结果更新 `config/` 后，依赖具体数值的用例可能失败，这不代表代码有问题。
