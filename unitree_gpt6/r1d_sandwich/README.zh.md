# R1-D 三明治

[English](README.md) | 中文

通过 [SKILL.zh.md](SKILL.zh.md)，让具备图片查看能力的 Agent 操作 Unitree R1-A7-D 和 Dex1 夹爪制作一份
三明治。Agent 读取头部与两路腕部相机、估计物体位置、直接给出末端路点并判断每一步结果；本机服务负责
IK、碰撞与桌面检查、运动执行、看门狗和停止状态。

本技能不使用物体检测器、符号任务规划器，也不在真机上搜索位姿。支持范围是文档描述的烤面包机、白盘和
小盘生菜，不包含任意布局或其他食物任务。

## 目录名与技能名

目录名为 `r1d_sandwich/`，`SKILL.md` 中的技能标识为 `r1d-sandwich`，与仓库其他技能的惯例一致。
Agent Skills 规范要求两者相同；把技能安装到会检查这一点的地方时，请把目录改名为 `r1d-sandwich`。

## 能力

- 从本机服务读取 `head_left`、`left_wrist` 和 `right_wrist` 图像。
- 通过 `bash r1d eef` 下发明确的 TCP 位置和 `rpy_deg` 路点。
- 在支持的流程中固定升降为 `0.522` m。
- 检查关节限位、自碰撞、桌面上方间隙、已登记障碍盒和单次运动位移。
- 支持 sim、shadow、real 三种服务模式，以及无动作路点预演和运行指标采集。
- 阶段之间使用关节空间 `home` 复位。
- 用标定过的相机把图像像素换算成基座坐标（`eef project`）。这只是几何换算，不做任何检测或识别。
- 用户要求时，用双手把做好的盘子向前推出（`eef both`）。

## 运行条件

- Linux x86_64，Python 3.10，以及 `requirements.txt` 里的包。
- 软件上不需要别的东西。逆解、碰撞检查、执行器、机器人读写、仿真机器人和相机客户端都在 `src/` 里，
  R1-D 模型（URDF 与网格）在 `assets/r1d/` 里。无论仿真还是真机，都不需要其他代码目录。复制自其他工程
  （r1-ik、r1_d）的文件在第一行注明了来源。
- 连真机时：一台在 R1-D 网络内的主机，带 Dex1 夹爪和参考相机配置的 Unitree R1-A7-D；每一次运动都有用户
  明确授权，现场有操作员，硬件急停伸手可及。

仓库在 `assets/config/` 里附带参考标定 JSON，自带模型里的手臂惯性参数也是按一台机器人拟合的。它们可以
用来检查软件流程，但不能替代其他机器人或其他摆放场景的现场标定。

## 安装

在本技能目录执行：

```bash
python3.10 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
```

在仓库根目录也可以用可选依赖组安装：

```bash
python3.10 -m pip install -e ".[sandwich-r1d]"
```

`requirements.txt` 列出本技能导入的包：

| 包 | 用途 |
|---|---|
| `numpy`、`opencv-python` | 数组、图像、相机模型 |
| `pin`（Pinocchio，自带 `coal`） | 运动学、重力项和碰撞检查 |
| `robokit`、`warp-lang` | 每个路点背后的 IK 求解器；在 CPU 上运行，有 CUDA 显卡时用显卡 |
| `pyzmq` | 相机客户端 |
| `cyclonedds`、`unitree_sdk2py`（从 GitHub 安装） | 机器人状态和指令；仿真时也会导入 |

## 设置与 `.env`

不需要任何设置就能运行。要改默认值时，复制模板再修改：

```bash
cp .env.example .env
```

程序会自动读取 `.env`。Shell 环境变量优先级更高。代码里没有写死任何本机路径。

| 变量 | 用途 |
|---|---|
| `R1D_SANDWICH_PYTHON` | 可选：`bash r1d ...` 使用的解释器；默认 `.venv/bin/python`，其次 `python3` |
| `UNITREE_SDK2_ROOT` | 可选：用某个 `unitree_sdk2_python` 代码目录代替已安装的包 |
| `R1D_URDF` | 可选：相机模型使用的 R1-D URDF；默认用自带的那份 |
| `SERVER_URL` | 本机服务地址，默认 `http://127.0.0.1:8765` |
| `RUNS_DIR` | 可选运行输出目录，默认 `<包根>/runs` |

不要把 `.env`、运行数据、录像、日志和本地虚拟环境提交到版本库。

## 环境检查

运行无动作的依赖检查：

```bash
bash r1d check-env
```

它会检查自带的 IK 代码和模型、Python 导入（包括 IK 求解器用到的 `robokit` 和 `warp`），以及
`assets/config/` 下的所有 JSON。它不会启动服务、连接相机或移动机器人。

## 离线与无动作验证

在仓库根目录运行离线测试，只需要 Python 标准库：

```bash
python -m unittest discover -s unitree_gpt6/r1d_sandwich/tests -p 'test_*.py'
```

先启动一个升降高度与任务一致的仿真机器人，再对着它预演和检查路点。以下命令不会让机器人运动：

```bash
bash r1d serve --sim --fixed-lift 0.522
bash r1d eval '<候选路点 JSON>'
bash r1d eval --brief '<候选路点 JSON>'
bash r1d eef check '<路点 JSON>'
bash r1d eef both-check '<双臂 JSON>'
```

`eval` 使用与服务相同的 planner，报告可达性和姿态指标；`--brief` 每个候选只输出一行。加上
`"side": "both"` 可以预演双臂路点，`"end_at_start": true` 会顺带检查能否沿关节空间回到起始关节角。
`check` 和 `both-check` 从当前位姿检查每个路点。这些命令都不能替代现场验证。

## 启动服务

只启动用户授权的模式。命令都在本技能目录执行。

```bash
bash r1d serve --sim --fixed-lift 0.522
```

```bash
bash r1d serve --shadow --fixed-lift 0.522
```

```bash
bash r1d serve --real --fixed-lift 0.522
```

| 模式 | 读取机器人状态 | 下发运动 |
|---|---|---|
| `--sim` | 否 | 否 |
| `--shadow` | 是 | 否 |
| `--real` | 是 | 是 |

`--sim` 下仿真机器人从固定升降高度起步，所以预演路线时的高度和任务一致。
real 模式启动时要求操作员输入一次 `s`，默认还会在每次运动前再确认。启动 real 或 shadow 服务以及发送
任何运动命令，都需要用户明确授权。

连真机时，服务用自带的执行器通过 `unitree_sdk2py` 驱动手臂，并用自带的客户端从机器人的图像服务读取
相机，不需要 unitree-deploy 代码目录。2026-11-17 和 18 日的三明治运行用的就是这个执行器；2026-11-25
那次用的是 unitree-deploy 的控制进程，这部分后来已从本技能中移除。

## 运行流程

Agent 按 [SKILL.zh.md](SKILL.zh.md) 执行：其中“单次运行”是操作步骤，“硬规则”是一次运行里唯一的一份
规则清单。大致过程：

1. 用户授权本次运动；场地清空，操作员能够触达硬件急停。
2. 启动已授权的服务模式，创建运行，把机器人调到起始姿态（`prepare`：升降 `0.522`，头部低头 `0.486` rad），
   并开始录像：

```bash
bash r1d eef new
bash r1d eef prepare
bash r1d eef rec-start
bash r1d eef state
bash r1d eef look --cams head_left,left_wrist,right_wrist --raw
```

3. 从当前图像读取烤面包机、盘子和生菜的位置，每个阶段用 `bash r1d eval --brief` 预演。
4. 每一步都是一条先动作再看图的命令，并在同一轮里读图。每次 `look` 还会把各视角另存为
   `runs/latest/<相机>.jpg`，任何会让机器人运动的命令都会先删掉这些文件，所以这个路径下的图片一定是
   最后一次动作之后拍的：

```bash
bash r1d eef exec '<路点 JSON>' && bash r1d eef look --cams right_wrist --raw
```

5. 要量出位置时，先看带像素刻度的裁剪图，再把像素投影到指定高度的平面上：

```bash
bash r1d eef look --cams head_left --raw --crop 180,150,360,290 --scale 4
bash r1d eef project --cams head_left --z 0.61 "261,226;297,224"
```

6. 结束时回 `home`、保存关键帧、停止录像并采集指标：

```bash
bash r1d eef keyframe final
bash r1d eef rec-stop
bash r1d metrics --run-dir runs/<运行目录>
```

路点可以带 `"min_opening": 0.1`：执行到这个路点之前，如果这只手的夹爪开度小于该值，`exec` 就停下。
这样夹面包和搬运可以放在同一条命令里。

路线见 [references/TASK_SANDWICH.zh.md](references/TASK_SANDWICH.zh.md)；命令、载荷和效率规则见
[references/PLAYBOOK.zh.md](references/PLAYBOOK.zh.md)。

## 安全边界

Agent 必须遵守的规则是 [SKILL.zh.md](SKILL.zh.md) 里的“硬规则”。对操作员来说：

- 每一次运动都需要用户明确授权。real 模式会在每次运动前到服务终端询问，除非操作员有意用
  `--confirm none` 启动。
- 服务检查关节限位、自碰撞、桌面间隙、已登记的障碍盒和单次运动位移。它不能识别人、盘子、食物或松散
  物体，也不能确认夹持力或物体是否被夹住。
- 软件停止不能替代硬件急停。硬件急停要放在伸手可及的地方。
- 运行开始前，盘子中心必须在机器人基座前方 `x >= 0.33` 处。

## 目录结构

顶层按 [Agent Skills 规范](https://agentskills.io/specification) 组织：`SKILL.md` 加上 `scripts/`、
`references/` 和 `assets/`。

| 路径 | 内容 |
| --- | --- |
| [SKILL.md](SKILL.md)、[SKILL.zh.md](SKILL.zh.md) | Agent 流程：单次运行、硬规则、什么任务读什么文档 |
| [references/](references/) | 按需阅读：任务路线、手册、服务 API、经验、结果（中英文各一份） |
| `scripts/` | `bash r1d` 背后的命令，见下表 |
| `assets/` | `config/`：参考机器人的相机标定和场景；`r1d/`：R1-D 的 URDF 和网格；`waypoint_template.json` |
| `r1d` | 统一入口：`bash r1d {serve,eef,eval,posture,metrics,check-env,home}` |
| `src/` | 服务的代码，按功能分目录，见下表 |
| `tests/` | 离线测试，只需要 Python 标准库 |
| `requirements.txt`、`.env.example` | Python 依赖；可选设置 |

`bash r1d` 会设置解释器和 `PYTHONPATH`。要手动运行 `src/` 里的工具，在 `src/` 目录下执行
`python3 -m <目录>.<工具>`。

| `scripts/` 里的文件 | 命令 | 用途 |
| --- | --- | --- |
| `eef_cli.py` | `bash r1d eef ...` | Agent 用的命令：`new`、`prepare`、`state`、`look`、`project`、`check`、`exec`、`both`、`both-check`、`grip`、`waist`、`home`、`keyframe`、`rec-start`、`rec-stop` |
| `eef_eval.py` | `bash r1d eval` | 用服务的 planner 离线预演候选路点，单臂或双臂 |
| `arm_posture.py` | `bash r1d posture` | 当前姿态的肩外展、肘部高度和肩偏航重力力矩 |
| `collect_metrics.py` | `bash r1d metrics` | 一次运行的用时、模型调用、成本、阶段和温度 |
| `check_environment.py` | `bash r1d check-env` | 检查自带代码、Python 导入和配置文件；不接触硬件 |
| `serve.sh`、`_env.sh` | `bash r1d serve` | 启动服务；选择解释器并读取 `.env` |

| `src/` 里的目录 | 文件 | 用途 |
| --- | --- | --- |
| `service/` | `server.py` | 本机 HTTP 服务：路由、模式（`--sim`、`--shadow`、`--real`）、启动检查 |
| | `client.py` | 服务的客户端，只用标准库 |
| | `session.py` | `bash r1d eef` 的实现：执行和检查路点、状态摘要、标尺图和像素图、关键帧 |
| | `paths.py` | 来自环境变量和 `.env` 的设置；自带代码和资源的位置 |
| `robot/` | `robot.py` | 状态、TCP 运动、双臂运动、夹爪、home、腰、升降、停止；shadow 检查和操作员确认 |
| | `grip_exec.py` | 执行层：自带执行器加同一指令帧里的夹爪，以及仿真机器人 |
| | `simworld.py` | `--sim` 用的桌面场景：夹取、搬运、松手和叠放 |
| | `go_home.py` | `bash r1d home`：从垂臂姿态回 home，不从桌面下方穿过 |
| | `ik_executor.py`、`dds_io.py`、`brain.py` | 复制自 r1-ik：轨迹、指令帧、看门狗和执行器；通过 `unitree_sdk2py` 读写机器人；标准起始姿态 |
| | `model.py`、`protocol.py`、`dynamics.py` | 复制自 r1_d：关节分组与限位、`rt/lowcmd` 打包、手臂重力与摩擦前馈 |
| `ik/` | `ik_path.py` | Planner：逆解、关节限位、自碰撞、桌面与障碍检查、直线和关节空间路径、双臂路径 |
| | `kin.py`、`quality.py`、`solvers.py`、`models.py` | 复制自 r1-ik：精简模型、姿态与碰撞检查、IK 求解器、模型变体 |
| `perception/` | `image_client.py` | 机器人图像服务的只读客户端 |
| | `cameras.py` | 各相机的最新画面，在单独进程里读取；仿真相机 |
| | `camera_model.py` | 头部双目和腕部相机的标定内参与基座系位姿 |
| | `projection.py` | `/project` 用的像素到基座坐标换算（平面或头部双目） |
| | `record3.py` | `eef rec-start` / `rec-stop` 背后的三视角 MP4 录像 |

服务里没有物体检测器，也没有预先写好的任务阶段脚本。接口列表见
[references/API.zh.md](references/API.zh.md)。运行输出写到 `runs/`，已被 Git 忽略。

## 许可证

Apache-2.0，见仓库 [LICENSE](../../LICENSE)。
