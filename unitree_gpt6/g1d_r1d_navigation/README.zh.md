# g1d_r1d_navigation — AI Agent 导航（Unitree Navi Agent）

[English](README.md) | 中文

通过自然语言指令控制 Unitree R1-D / G1-D 轮式底盘，让机器人观察环境并自主移动。

目标平台：**R1-D / G1-D 轮式底盘**（共用同一套代码，启动时自动识别机型）。

当前状态：已通过 MCP 宿主接入机载传感器和控制器，并在 R1-D、G1-D 真机上完成室内测试，测试全程由人员监护。

支持图像输入的大模型通过 [MCP](https://modelcontextprotocol.io) 调用机器人工具，获取相机画面、控制平移和转向、提交连续速度序列，或返回已标定的点位。模型根据画面规划动作，在移动过程中继续观察和调整，执行「走到桌子旁边停下」「沿走廊走到尽头，保存为 B 点后返回」等任务。

```text
"走到走廊尽头的饮水机旁边停下"
        │
        ▼
                  image+text                        stdio/MCP                          DDS/RPC/ZMQ/UDP
┌───────────────┐            ┌────────────────────┐            ┌─────────────────────┐                 ┌────────────────┐
│ Vision LLM    │<──────────>│ MCP host           │<──────────>│ navi_mcp_server.py  │<───────────────>│ G1D / R1D base │
│ (GPT /        │            │ (DeepSeek-Harness  │            │ (Python side of     │                 │ camera / lidar │
│ Qwen-VL ...)  │            │ / Claude Desktop)  │            │ this project)       │                 │ SLAMWARE (G1D) │
└───────────────┘            └────────────────────┘            └─────────────────────┘                 └────────────────┘
                                        ^  polls :3082 every 1 s (read-only)  │
                                        └── navi-observer plugin <────────────┘
```

> **警告**：本软件用于驱动真实机器人。首次在实体机器人上运行时，必须有人员在现场值守、急停装置触手可及、场地已清空，且速度保持默认值 0.3 m/s。软件停止指令（`stop`）不能替代硬件急停。

---

## 目录

1. [功能概述](#1-功能概述)
2. [快速开始（离线 stub）](#2-快速开始离线-stub)
3. [真机部署](#3-真机部署)
4. [代码结构](#4-代码结构)
5. [工具清单与语义](#5-工具清单与语义)
6. [三种运动模式与互斥](#6-三种运动模式与互斥)
7. [安全分层](#7-安全分层)
8. [配置参考](#8-配置参考)
9. [系统提示词（persona）与观测推送](#9-系统提示词persona与观测推送)
10. [接入其他 MCP 宿主](#10-接入其他-mcp-宿主)
11. [移植到其他机器人](#11-移植到其他机器人)
12. [常见问题](#12-常见问题)
13. [无动作预览与测试](#13-无动作预览与测试)
14. [致谢、出处与许可证](#14-致谢出处与许可证)

---

## 1. 功能概述

| 功能 | 说明 |
|---|---|
| **视觉导航** | 模型读取相机帧 → 规划 `move` / `turn` / `submit_sequence` → 读取返回帧 → 再次规划。无需地图与定位即可运行。 |
| **运动中推理** | `submit_sequence` 提交未来数秒的速度序列，并返回运动中的画面。模型根据画面规划下一段时，机器人继续执行当前序列；下一段提交后由服务端平滑衔接，无需每步停车。 |
| **返回已标定点** | 配有机载 SLAMWARE 地图时，用 `save_place` 保存点位，再用 `navigate_to` 调用机载规划器避障返回。R1D 使用底盘雷达建图和定位，`save_place` 保存点位，`list_places` 提供距离与方位，由模型调用运动工具驶回。 |
| **雷达反射层防撞** | 雷达反射层在每个控制周期根据前方障碍物距离限制前进速度，无需等待模型响应。检测范围和盲区见第 7 节。 |
| **观测推送** | 模型推理期间，机器人停稳或被雷达拦截时，宿主插件会向对话插入「[观测]」消息，提醒模型当前状态。 |
| **离线 stub** | 无需连接机器人，即可检查宿主连接 MCP、图像传入模型上下文、模型调用工具的完整流程，方便调试提示词或更换模型。 |

**G1D 硬件要求**：Unitree G1D 轮式底盘，提供 `agv` RPC 服务；机器人上运行 image_server，提供 ZMQ 相机流；可选配 SLAMWARE 激光雷达与机载 SLAM。主机与机器人需位于同一内网，机器人默认地址为 `192.168.123.164`。

**R1D** 会被自动识别，使用 `cmd_vel` 控制底盘，头部左目相机提供 RTP-H264 视频流（UDP 5002），底盘前端的 Livox Mid-360 提供 UDP 点云（内网地址 `192.168.123.120`）。R1D 暂不具备机载 SLAMWARE，不支持 `navigate_to` 地图导航，主要依靠视觉导航。本服务也支持用 Mid-360 定位和标记点位，再由模型调用运动工具驶回（见第 5 节）。

---

## 2. 快速开始（离线 stub）

本节无需连接机器人。stub 用带编号的合成画面代替相机画面，工具名称、参数与返回字段均与真机一致。`memory_*` 拓扑记忆工具的数据存放在临时目录，不会写入真机记忆库。

### 2.1 准备 Python 环境

```bash
cd unitree_gpt6/g1d_r1d_navigation                          # 本文档中的命令均在技能目录下执行
python3 -m venv .venv && source .venv/bin/activate      # 或 conda create -n navi python=3.10
python -m pip install -e '../..[navigation]'
```

### 2.2 准备宿主（DeepSeek-Harness）

本项目默认使用 DeepSeek-Harness（下称 DSH）作为宿主。它是一个 TypeScript Agent 框架，负责管理对话、调用大模型、提供 MCP 工具和 Web UI。

```bash
git clone https://github.com/deepseek-ai/deepseek-harness.git deepseek-harness   # 可置于技能目录内或任意位置
cd deepseek-harness && pnpm install && pnpm run build && cd ..
export DSH_DIR="$PWD/deepseek-harness"                  # 位于 ./deepseek-harness 时可省略
```

> 如不使用 DSH，请参阅 [第 10 节](#10-接入其他-mcp-宿主)：任何 MCP 宿主均可直接启动 `src/navi_agent/navi_mcp_server.py`。

### 2.3 配置模型密钥并启动

默认使用 OpenAI 的 `gpt-6-astra`，配置见 `host/navi-gpt.patch.yaml`。新会话的推理强度默认为 `low`，可在 UI 中切换。DSH 所依赖的 pi-ai 模型目录需要包含该模型，旧版 DSH 请先更新；更换模型时，需修改 patch 中的两处模型名称。

```bash
export NAVI_PYTHON="$PWD/.venv/bin/python"   # 指定宿主启动 sidecar 时使用已安装依赖的解释器
export OPENAI_API_KEY=sk-xxxx                # 使用百炼时设置 DASHSCOPE_API_KEY，使用 DeepSeek 时设置 DEEPSEEK_API_KEY
chmod +x scripts/start.sh
scripts/start.sh --dry-run                 # 预览将要执行的命令
scripts/start.sh                           # 等同于 scripts/start.sh gpt stub
```

终端会输出一个带 token 的 URL（`http://127.0.0.1:3080/...`）。在浏览器中打开地址并新建会话，默认模型为 `gpt-6-astra`。如果更换了模型提供方，请在下拉列表中选择**支持图像输入**的模型，例如百炼的 `qwen3-vl-plus`。然后输入：

```text
向前走一段，然后左转 90 度，告诉我你看到了什么
```

此时可以看到模型依次调用 `mcp__robot__observe`、`mcp__robot__submit_sequence` 等工具，stub 画面中的编号随之递增。

也可以不打开 UI，直接运行一次验证：

```bash
scripts/start.sh --headless "观察一下当前画面，然后向前走 1 米"
```

### 2.4 `start.sh` 参数

| 参数 | 含义 |
|---|---|
| `gpt` / `bailian` / `deepseek` | 选择模型提供方（默认 `gpt`）：OpenAI GPT（使用网关时可在 `host/navi-gpt.patch.yaml` 中添加 `baseURL`）/ 阿里云百炼 / DeepSeek 官方 |
| `stub` / `real` | 模拟机器人 / 实体机器人 |
| `r1d` / `g1d` | 选择系统提示词（默认 `r1d`）。sidecar 启动时自动识别机型，此参数须与实际机型一致 |
| `--dry-run` | 仅输出命令，不执行 |
| `--headless "消息"` | 以 headless 方式运行一次 |

| 环境变量 | 含义 |
|---|---|
| `DSH_DIR` | DSH 仓库路径 |
| `NAVI_PYTHON` | 运行 sidecar 的 Python 解释器（默认使用技能目录下的 `.venv/bin/python`，不存在时使用 `python3`） |
| `OPENAI_API_KEY` / `DASHSCOPE_API_KEY` / `DEEPSEEK_API_KEY` | 对应 provider 的密钥；未设置时 `start.sh` 在终端提示输入（不回显、不写入磁盘） |
| `DEEPSEEK_BASE_URL` | DeepSeek 经转发网关访问时覆盖端点地址 |
| `NAVI_CAMERA_VIEW=0` | 真机模式下不启动浏览器相机画面 |
| `NAVI_OBSERVER_PORT=0` | 关闭观测推送插件 |
| `NAVI_RUN_LOG` | 运行日志路径（默认 `logs/run-<时间戳>.jsonl`；设为 `0` 时仅在相机页显示，不写入磁盘） |
| `NAVI_LOG=0` | 不写入进程输出日志（默认 `logs/navi-<时间戳>.log`，记录 DSH 与 sidecar 的 stdout/stderr） |

本机路径与开关（`DSH_DIR`、`NAVI_PYTHON`、`PATH` 及上表中的 `NAVI_*` 变量）可写入技能目录下的 `navi.env`。从 `.env.example` 复制该文件后，`start.sh` 会在启动时自动加载并导出其中的变量。该文件不纳入版本控制，请勿在其中保存模型密钥。

`start.sh` 启动时会依次执行以下操作：

1. 检查模型密钥。
2. 在 `host/generated/` 下生成 patch，配置 MCP 服务。web 模式还会读取 `host/navi-persona-<g1d|r1d>.txt`，生成包含 persona 和压缩链的 `navi` preset，并将其设为默认 preset。
3. 设置 `DSH_HOME=<仓库>/.dsh-home`，将会话数据与全局 `~/.dsh` 分开存放。
4. 以 `pnpm dsh --profile web --patch ...` 启动 DSH。

---

## 3. 真机部署

### 3.1 安装 Unitree SDK

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python
cd unitree_sdk2_python && pip install -e .     # 同时安装 cyclonedds
```

R1D 的头部相机通过 `gst-launch-1.0` 解码 RTP-H264，还需安装 GStreamer 命令行工具与插件，例如：

```bash
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-libav
```

### 3.2 编写现场配置

```bash
cp src/navi_agent/config.example.yaml src/navi_agent/config.yaml
```

至少确认以下两项：

- `net`：连接机器人内网的网卡名称（通常 `auto` 即可；多网卡主机请显式指定）。
- `camera.host`：运行 image_server 的机器人 IP。

默认关闭地图导航。首次部署建议先用视觉导航确认基本功能正常。

### 3.3 只读自检（不驱动底盘）

```bash
# 相机：在浏览器中查看实时画面、运行日志与任务看板（只读，不发送任何运动指令；日志与看板数据来自运行中的 Navi）
python src/navi_agent/tools/camera_view.py --port 3081          # 访问 http://127.0.0.1:3081/
python src/navi_agent/tools/camera_view.py --demo               # 无相机时验证 HTTP 链路
```

### 3.4 启动

```bash
scripts/start.sh real          # R1D
scripts/start.sh real g1d      # G1D（使用 G1D 版系统提示词）
```

首次运行建议依次尝试「观察一下周围，不要移动，描述你看到的东西」「原地左转 30 度」「向前走 0.5 米」。确认观察、转向和前进均正常后，再尝试实际任务。

### 3.5 启用地图导航（可选）

前提：已使用厂商工具在机器人上完成本场地的建图与定位。

1. 在 `config.yaml` 中设置 `navi.navigation.enabled: true`、`map_id`（自定义名称）以及 `mapping.backend: slamware_ros`。
2. 重启后，在会话中让模型调用 `navigation_preflight`，逐项处理 `errors` 中的问题。`frame` 须与 `rt/robot_pose` 的 `header.frame_id` 一致。刚启动时，订阅数据尚未就绪，可能出现 `*_stale` / `lidar_not_ready`，等待数秒后重新检查即可。
3. 由操作人员将机器人推动或驾驶至目标位置，让模型执行 `save_place("起点")`。
4. 执行 `navigate_to("起点")`，验证机器人能够返回。

`config.example.yaml` 末尾列出了一组在 G1D + SLAMWARE 上实测可用的配置，用于放宽部分检查或调整起步行为，包括 `odom_required: false`、`receipt_time_freshness: true`、`pre_align_deg: 60` 等。每项都附有原因说明，请确认适用于当前场地后再启用。

### 3.6 现场检查清单

- [ ] 急停装置触手可及，场地已清空，人员位于机器人侧后方
- [ ] `get_status` 中 `robot_connected`、`camera_connected` 均为 true
- [ ] `get_status` 中 `lidar.ok` 为 true，且 `front_m` 与实际前方距离一致（可在前方放置遮挡物核对读数）
- [ ] 速度保持默认值（`motion.move_speed: 0.3`；序列速度上限按机型区分：G1D 0.5 m/s、R1D 1.0 m/s，并由雷达限速包络根据前方净空自动降低）
- [ ] 已知雷达**正后方存在盲区**（G1D 实测有效回波范围约 ±133°），后退时没有雷达反射层保护
- [ ] 已知雷达**无法检测低矮的地面物体**（线缆、横杆、倒地的物体），此类障碍仅能依靠模型识别画面
- [ ] 玻璃门、玻璃隔断通常会被雷达穿透，同样仅能依靠画面识别

---

## 4. 代码结构

```text
unitree_gpt6/g1d_r1d_navigation/
├── README.md / README.zh.md / .env.example / .gitignore
├── common/                           本地预览的配置、接口、运行流程及测试
├── config/default.yaml               仓库统一 CLI 的无动作预览配置（与真机运行时无关）
├── scripts/
│   ├── start.sh                      启动脚本（DSH + patch 叠加）
│   └── preview.sh                    无动作预览（仓库统一 CLI）
├── tests/test_preview.py             无动作预览入口的 unittest
├── assets/                           可公开素材（当前为空）
├── src/
│   ├── main.py / agent.py / …        仓库统一 CLI 的无动作预览入口（见第 13 节）
│   └── navi_agent/                   ── Python：MCP 服务（sidecar）（机器人侧全部逻辑；平铺导入布局，不是 Python 包）
│       ├── navi_mcp_server.py            ★ 入口。FastMCP stdio 服务，注册 17 个工具（另由 topo_memory 注册 7 个 memory_* 工具），编排各层
│       ├── robot_tools.py                工具纯逻辑：参数校验（guard_move/turn/sequence）、结果整形，不依赖 MCP
│       ├── driver_factory.py             HeadlessDriver 进程级单例 + status/odometry/stop 基础工具
│       ├── config.py                     配置权威来源：DEFAULT_CONFIG + config.yaml 深合并 + 运行时全局量
│       ├── config.example.yaml           现场配置模板
│       ├── core/                         ── 与硬件解耦的控制逻辑
│       │   ├── driver.py                 HeadlessDriver：机器人与相机的最小门面（connect/move/turn/stop/status）
│       │   ├── motion.py                 离散运动执行器：斜坡加减速 + 时长控制 + 指令积分里程
│       │   ├── sequence.py               序列执行器：速度段合并（fresh/blend/replace）、对齐、看门狗、延迟 EMA
│       │   ├── heading_hold.py           R1D 航向闭环：雷达 IMU 陀螺修正直行偏航、按实测转角停止转向
│       │   └── places.py                 命名点位存储：绑定 map_id，原子写入 + 进程锁，文件损坏时报错而不回退
│       ├── services/                     ── 硬件/协议适配
│       │   ├── robots/                   机型层：首次连接时只读探测 G1D / R1D → profile（限速/能力位）+ 对应底盘驱动
│       │   │   ├── g1d.py                G1D RobotController：agv RPC 20Hz 速度流、指令看门狗、零速释放
│       │   │   ├── r1d.py                R1D 底盘：cmd_vel 归一化速度流 + 反馈新鲜度/告警守卫（保守限速）
│       │   │   └── probe.py / profile.py / unknown.py   探测子进程、机型参数、未识别机型时拒绝运动
│       │   ├── rtp_camera.py             R1D 左目 RTP-H264 相机（gst 解码）
│       │   ├── topo_memory.py            拓扑记忆：地点/地标/前沿/已走过的边，memory_* 工具
│       │   ├── g1_agv_client.py          AgvClient 的 Python 实现（agv 服务 API 1001 Move）
│       │   ├── camera.py                 image_server 相机：GET_DATA 协商 + ZMQ SUB 后台收帧
│       │   ├── lidar.py                  LidarGuard 雷达反射层：前方扇区/走廊净空 → 钳制前进速度
│       │   ├── livox_mid360.py           R1D 的 Livox Mid-360：UDP 点云 → 虚拟二维扫描 → LidarGuard
│       │   ├── lidar_slam.py             R1D 的雷达定位：点云 → 二维扫描-地图匹配 → 位姿 + 地图 + 重定位
│       │   ├── navigation.py             SLAMWARE 定位订阅 + goto/取消 + 到达/停滞/超时判定
│       │   ├── slamware_ros_mapping.py   只读地图健康检查：地图、定位质量、TF（无写操作）
│       │   ├── slamware_idl.py           SLAMWARE 话题的 CycloneDDS IDL 定义
│       │   ├── mapping_config.py         navi.navigation.mapping 段的类型化校验
│       │   ├── observer.py               内嵌 HTTP 只读状态接口（:3082），供宿主插件轮询
│       │   └── runlog.py                 运行日志：工具调用与结果摘要（相机页显示、logs/run-*.jsonl）
│       ├── tools/
│       │   ├── stub_mcp_server.py        离线 stub：与真机同构的工具集，返回合成画面
│       │   ├── camera_view.py            浏览器相机画面 + 运行日志 + 任务看板 / 已记录房间（只读，:3081）
│       │   ├── r1d_diag.py               R1D 只读诊断：机型探测、反馈、话题、相机服务、Livox 广播
│       │   ├── livox_diag.py             Livox Mid-360 诊断与安装外参标定（默认只读）
│       │   ├── heading_check.py          R1D 航向闭环现场对比测试（会驱动底盘，以雷达定位作为真值）
│       │   └── lidar_slam_replay.py      雷达定位离线回放（读取 NAVI_LIDAR_RECORD 录制的点云，输出轨迹与地图图像）
│       ├── tests/ + conftest.py          pytest 单元测试（不连接硬件）
│       └── ruff.toml                     继承仓库 Ruff 配置，放宽排版类规则
└── host/                             ── 宿主侧（DeepSeek-Harness 配置与插件）
    ├── navi-gpt.patch.yaml           provider：OpenAI GPT（默认；llm-pi-ai openai 路由）
    ├── navi-bailian.patch.yaml       provider：阿里云百炼
    ├── navi-deepseek.patch.yaml      provider：DeepSeek 官方
    ├── navi-persona-r1d.txt          ★ R1D 系统提示词 persona（start.sh 按机型参数选择、按模式注入）
    ├── navi-persona-g1d.txt          ★ G1D 系统提示词 persona
    ├── navi-observer.patch.yaml      挂载观测推送插件
    ├── navi-observer/                TypeScript 宿主插件：在模型推理期间推送「[观测]」
    │   ├── index.ts                  插件入口：轮询、推送、必要时中断推理
    │   ├── policy.ts                 纯函数决策：何时推送、何时升级为中断
    │   ├── policy.test.ts            上述逻辑的回归用例（`node --experimental-strip-types --test`）
    │   ├── continuation.ts           任务目标自动续行被上游故障停用后的重新启用策略
    │   ├── poll.ts                   拉取 :3082 的 /state 与 /frame.jpg
    │   ├── narration.ts              从 DSH 会话事件提取模型旁白、用户指令等，写入运行日志（POST /runlog）
    │   └── scope.ts                  判定会话是否属于 Navi（preset 或 persona 标记）
    └── generated/                    start.sh 运行时生成（不纳入版本控制）：robot 挂载 + web preset / headless 提示词
```

### 4.1 分层职责

| 层 | 文件 | 职责 | 不负责 |
|---|---|---|---|
| **系统提示词** | `navi-persona-<g1d\|r1d>.txt` | 向模型说明安全约束、各工具的适用时机、序列规划方法 | 不提供强制安全保障 |
| **工具层** | `navi_mcp_server.py` | 参数校验、模式互斥、帧登记、延迟统计、结果整形 | 不做视觉理解 |
| **执行器** | `core/sequence.py` `core/motion.py` | 将速度段转换为指令流，按底盘控制周期下发（G1D 20 Hz、R1D 50 Hz）；负责平滑合并与看门狗停车 | 不读取画面 |
| **雷达反射层** | `services/lidar.py` | 每个控制周期按雷达净空钳制前进分量 | 不钳制转向、不规划绕行 |
| **驱动** | `services/robots/` | 机型识别 + 底盘速度流、指令看门狗、零速释放伺服 | 不做定位 |
| **地图导航** | `services/navigation.py` | 将目标点交给机载 SLAMWARE，判定到达/停滞 | 不自行规划路径 |

### 4.2 一次 `submit_sequence` 的数据流

```text
模型 → submit_sequence(segments, frame_id, observe_after_s=3)
  └ robot_tools.guard_sequence       速度/时长/位移/转角硬上限，剥离零速段
  └ _motion_entry_lock               若地图导航正在执行 → 拒绝
  └ SequenceExecutor.submit          与正在执行的序列合并：同向→blend，冲突→replace，静止→fresh
       └ 控制周期循环（G1D 20 Hz / R1D 50 Hz）：取当前段速度 → LidarGuard.clamp(vx) → RobotController.send_command
  └ 等待 observe_after_s 秒后抓取一帧，登记 frame_id
  └ 返回：mode / remaining_s / suggested_horizon_s / latency_est_s / 图像
模型推理（机器人继续运动）→ 下一次 submit_sequence …
```

`suggested_horizon_s` 根据提交间隔的指数滑动平均值，估算模型每次推理所需的时间。提交的运动段应尽量覆盖这段时间，避免模型还在推理时机器人就已停下。

单次盲走距离仍受 `navi.sequence.max_blind_m` 限制，默认上限为 5 m，并按计划的平均速度折算为时长。推理较慢时（实测每步 20~30 s），应分段行驶，无需保持连续运动。返回结果中的 `note` 会提示模型每段行驶 4~5 m，停稳并观察后再继续。

运动和观察工具的返回结果均附带 `clearance`，表示抓帧时刻雷达测得的四个方向净空，单位为米。模型判断距离时无需另行调用 `get_status`。

---

## 5. 工具清单与语义

在宿主中，工具名称为 `mcp__robot__<名称>`（`serverName: robot`）。

### 感知

| 工具 | 说明 |
|---|---|
| `observe(eye=None)` | 返回当前相机帧及 `frame_id`。`eye`：`left`/`right`/`full`。过期帧（本机接收时间超过 `camera.max_frame_age_s`）将报错，而不是返回旧图像。 |
| `get_status()` | 机器人/相机连接状态、雷达反射层状态（`lidar.corridor_front_m` 为行进走廊净空、`front_m` 为 ±30° 扇区净空、`ok`）。 |
| `get_odometry()` | **基于指令积分的估计值**，并非实测定位，不能作为到达依据。 |
| `get_localization()` | SLAMWARE 地图定位、实测速度、数据新鲜度（需启用导航配置）。 |

### 运动

| 工具 | 说明 |
|---|---|
| `move(direction, distance_m)` | `forward`/`backward` 平移，单次上限 5 m，速度为 `motion.move_speed`。阻塞直至完成，返回停稳后的一帧。被雷达拦截时返回 `status="blocked"`。G1D 不支持横移。 |
| `turn(angle_deg)` | 原地转向，正值向左、负值向右，`10° ≤ \|角度\| ≤ 360°`。阻塞执行，返回停稳后的一帧。 |
| `submit_sequence(segments, frame_id, observe_after_s, alignment)` | 连续运动的首选工具。`segments=[{"vx","vyaw","duration"},…]`，线速度上限为 `capabilities.max_vx`（G1D 0.5、R1D 1.0 m/s）；角速度上限取 `capabilities.max_vyaw`（G1D 0.6、R1D 0.4 rad/s）与 `navi.sequence.max_vyaw`（默认 0.5）中的较小值。调用后立即返回。详见第 6 节。 |
| `sequence_status()` | 序列是否正在执行、剩余时长、当前下发速度、是否曾被看门狗停止。 |
| `stop()` | 清空序列、取消离散动作、取消导航，并等待底盘停稳。 |

`move` / `turn` / `submit_sequence` 共用原地打转检测。如果机器人一直未离开原位置，累计转向达到 360°，返回结果中会附带 `spin`，提醒模型选定方向驶离，避免反复原地转向搜索。R1D 通过雷达定位判断是否离开原位置，距离阈值为 ≥ 1 m。

### 任务看板

| 工具 | 说明 |
|---|---|
| `update_tasks(tasks)` | 将任务清单同步到相机页右侧（`:3081`），格式为 `[{"name","status","note"}]`，`status` 可取 `todo`/`doing`/`done`/`dropped`。该工具只更新显示，不驱动底盘。模型每开始或完成一项任务时，重新提交完整清单，并与下一个运动工具并行调用。相机页还会列出本次运行中通过 `save_place` 标记的房间；此前保存的点位不显示，但仍可用 `list_places` 查询。 |

**运行日志**：相机页下方按时间顺序滚动显示日志，同时逐行写入 `logs/run-<时间戳>.jsonl`，方便事后查看。日志来自两部分：

- sidecar 通过 `services/runlog.py` 记录工具调用的参数摘要和结果摘要，包括行驶距离、转向角度、前方净空、错误原因、原地打转提醒等。
- 宿主插件通过 `host/navi-observer/narration.ts` 提取 DSH 会话事件，记录模型输出、推理摘要、用户指令、[观测] 消息，以及本轮停止、上游出错等情况，再通过 POST 发送到状态接口 `/runlog`。

**相机页（:3081）**：R1D 的左目（即模型所见画面，由 sidecar 转发）与右目（UDP 5003，由相机页自行接收）并排铺满画面区域，下方为运行日志，右侧为任务看板；单目相机仅显示一幅画面。

### R1D 的雷达定位与标点

R1D 不具备机载 SLAMWARE，本服务使用底盘前端的 Mid-360 实现二维 SLAM，代码位于 `services/lidar_slam.py`。它将点云中离地 0.15~2.2 m 的竖直结构（墙体、柱子、家具）投影到二维平面，与行驶过程中逐步构建的 5 cm 占据栅格地图进行扫描匹配。定位不依赖里程计，因为 R1D 的速度反馈与 IMU 偏航均不可用。

| 工具 | 在 R1D 上的行为 |
|---|---|
| `get_localization()` | 雷达定位位姿（`frame=lidar_map`），仅当 `ok=true` 时可信；`state` 取值为 tracking / degraded / lost / relocalizing，异常时附带 `hint` |
| `save_place(name)` | 须在停稳且定位可信时调用；先保存地图，再保存点位 |
| `list_places()` | 每个点位附带 `distance_m`、`bearing_deg`（正值为左）、`heading_diff_deg`，模型据此手动驶回 |
| `navigate_to(name)` | 暂不支持（返回 `unsupported_on_r1d` 及手动返回方法） |

- 地图与点位保存在 `src/navi_agent/data/lidar_map/<navi.memory.venue>/`（`map.npz`、`places.json`）。地图有更新时每 60 s 自动保存一次，正常退出时再保存一次；进程被强制终止时，最近一次保存之后的地图更新会丢失。
- 重启后会自动重定位，以上次行驶轨迹中的位姿和已标定点为初始候选位置，在旧地图上匹配。未到过的区域无法完成定位，需将机器人驶回曾经过的区域，并静止约两秒。
- 目前不支持回环检测。大范围、长时间运行会积累误差，地图可能出现重影；长走廊、空旷大厅等几何特征单一的场景也会影响匹配效果。
- 更换场地或地图异常时，设置 `NAVI_SLAM_NEW_MAP=1` 启动一次。旧地图与旧点位会被重命名保留，服务随后重新建图。
- 调参时可设置 `NAVI_LIDAR_RECORD=1` 启动，将原始点云录制到技能目录下的 `logs/lidar-*.bin.gz`，再用 `cd src/navi_agent && python -m tools.lidar_slam_replay <文件>` 离线回放，查看轨迹、匹配质量、单帧耗时及地图图像。录制时绕行一圈回到出发点，可用终点位姿与起点之间的距离衡量累积误差。

### R1D 的航向闭环（雷达 IMU 陀螺仪）

R1D 的车轮起步时不一定朝向正前方。实测以 0.5 m/s 直行 1 m，车头偏航可达约 10°，而底盘没有可用的偏航反馈。因此，底盘指令发布线程接入了 Mid-360 内置 IMU 的陀螺仪数据（UDP 56401），通过 `core/heading_hold.py` 修正航向：

- **直行 / 序列**：每个控制周期将「指令偏航 − 陀螺仪实测偏航」累计为航向误差，并按比例叠加到转向分量上进行修正，使偏转的车头回正。
- **离散转向**：陀螺仪数据可信时，以**实测转角**判断是否到达目标，并提前 `turn_stop_lead_s` 秒对应的转角量收回指令，抵消停车后的滑转。陀螺仪不可用时，按指令积分判断何时停止，并补偿起步加速阶段少转的角度。若实测转角长时间未达目标，则超时停止，结果中附带 `turn_stop`。
- **结果中的实测值**：`turn` 返回 `measured_angle_rad`（`angle_source=gyro`），`move` 返回 `measured_yaw_change_deg`，`submit_sequence` 返回 `heading_track`（自上次提交以来的指令偏航与陀螺仪实测值）。运行日志中同样显示。
- **降级策略**：启动后，在底盘指令为零时估计零偏。此时机器人必须真正静止；若被人推动或遥控转动，会导致零偏估计错误，需要重启 Navi。首次转向会核对陀螺仪方向，方向相反则永久关闭闭环；IMU 数据中断或误差达到上限时，退回开环控制；停车时不修正航向。

可用 `cd src/navi_agent && python -m tools.heading_check [--distance 1.5 --speed 0.5 --repeat 1]` 进行现场对比测试。**该测试会驱动底盘，须先停止 Navi，并安排人员现场值守。** 测试分别在开环和闭环下执行一组「原地 ±90° 转向四次 + 直行后掉头 180° 两次」，以雷达定位作为真值，输出转角误差、直行航向漂移与横向偏移，并给出 `turn_stop_lead_s` 建议值。明细保存到 `logs/heading-check-<时间>.json`。

### 地图导航（`navi.navigation.enabled: true` 时可用）

| 工具 | 说明 |
|---|---|
| `mapping_capabilities()` | 只读：当前地图后端的能力。 |
| `mapping_status()` | 只读：机载地图、定位质量、TF。 |
| `navigation_preflight()` | 只读预检：配置、定位、雷达。冷启动时遇到 `*_stale` 会自动短暂重试。 |
| `list_places()` | 当前 `map_id` 下已标定的命名点位。 |
| `save_place(name, overwrite=True)` | 将当前有效定位保存为命名点位，要求底盘处于静止状态。重名时默认覆盖（结果中 `replaced=true`）；传 `overwrite=false` 时重名报错。 |
| `navigate_to(name)` | 异步导航至已标定点位，返回 `running` 仅表示已提交。目标方向偏离车头超过 `pre_align_deg` 时，先原地转向对准。 |
| `navigation_status()` | 仅当 `state=arrived` 且 `active=false` 时表示已到达；`stalled`/`timeout`/`fault` 均不表示成功。 |

### 拓扑记忆（`services/topo_memory.py`）

记忆库按机型和场地（`navi.memory.venue`）分别存储，记录经过的地点、地标、尚未探索的出口，以及地点之间走过的路线，供后续任务参考。这些记录只能提供线索，不能代替定位。离线 stub 也提供这组工具，数据存放在临时目录，不会写入真机记忆库。

| 工具 | 说明 |
|---|---|
| `memory_context(task, frame_id, max_images)` | 将当前帧与已存地点比对，返回候选地点及参考图、与 `task` 匹配的目标、从当前锚点出发的路线提示 |
| `memory_localize(place_id, frame_id, via)` | 模型比图确认位于某地点后调用：移动锚点，并自动记录上一锚点到此处的边 |
| `memory_save_place(label, evidence, frame_id, …)` | 将当前位置存为新地点，或为已有地点补充视角 |
| `memory_remember_landmark(label, evidence, frame_id)` | 在当前锚点地点下记录地标 |
| `memory_remember_frontier(label, direction, …)` | 记录尚未探索的出口或方向 |
| `memory_mark_unavailable(kind, id, reason)` | 将地点/地标/出口/边标记为失效 |
| `memory_list()` | 本机型、本场地的全部记忆概览 |

---

## 6. 三种运动模式与互斥

| 模式 | 工具 | 特点 | 适用场景 |
|---|---|---|---|
| 离散阻塞 | `move` / `turn` | 动作完成并停稳后返回画面 | < 1 m 的精确对位、狭窄空间 |
| 连续序列 | `submit_sequence` | 立即返回，后台连续执行，运动中推理 | 开阔区域接近目标、绕障、走廊 |
| 地图导航 | `navigate_to` | 机载 SLAMWARE 规划与避障 | 返回已标定点位 |

**互斥**：三种模式由同一把 `_motion_entry_lock` 锁保护。地图导航执行期间，`move`/`turn`/`submit_sequence` 将被拒绝；`move`/`turn` 会先清空正在执行的序列。`stop` 在任何模式下均可用。

### 序列合并规则

| 返回的 `mode` | 触发条件 | 行为 |
|---|---|---|
| `fresh` | 当前没有正在执行的序列 | 从静止状态起步 |
| `blend` | 新旧序列方向一致 | 在 `blend_s` 时间窗内平滑过渡 |
| `replace` | 方向冲突（如前进↔后退） | 立即接管 |

`alignment=current`（默认）表示新计划从服务端接收时刻开始执行。`alignment=continue` 表示计划以 `frame_id` 对应的时刻为起点，服务端会裁去已经执行的前缀部分。后者仅在旧计划为恒速、且能验证一致性时允许使用，否则会被拒绝。

**看门狗**：序列执行期间，若模型超过 `max(sequence.watchdog_s, 2×推理时延EMA+1)` 秒未提交新序列，机器人会逐步减速停车。

---

## 7. 安全分层

系统按以下层次检查和限制运动，越靠下越少依赖模型判断：

| 层 | 机制 | 默认值 |
|---|---|---|
| 1. 系统提示词 | persona「〇、最高约束」：禁止碰撞；人员优先（迎面接近时默认停在 ≥1 m 之外，经操作员明确允许可接近至 0.5 m）；低矮物体依靠画面识别；不得虚报到达 | — |
| 2. 参数校验 | `robot_tools.guard_*`：单次平移 ≤5 m、转向 10°~360°、序列速度/时长/位移/转角硬上限 | 5 m / 机型 max_vx / 机型 max_vyaw（≤0.5 rad/s） / 45 s |
| 3. 雷达反射层 | `LidarGuard`：前方净空 < `stop_m` 时前进速度归零；G1D 在 `stop_m`~`slow_m` 区间线性减速，R1D 使用限速包络（允许速度 = max(0.2, (路径净空 − stop_m) / 1.5 s)，仅考虑与车身同宽的行进路径，边行进边转向时沿圆弧计算，不考虑两侧墙体）；**仅钳制前进分量；纯原地转向不受约束，但前进被拦时连续序列会同时刹停转向分量**。数据源按机型区分：G1D 使用 SLAMWARE 的二维扫描；R1D 使用底盘前端 Mid-360 的三维点云（离地高于 5 cm + 1.5 cm × 距离（米）且不超过 1.5 m 的点计为障碍物，如 2 m 处阈值约 8 cm；距离自车头前缘起算） | 0.45 m / 1.2 m |
| 4. 序列看门狗 | 模型长时间无响应时斜坡减速停车 | 8 s（自适应） |
| 5. 驱动看门狗 | 距上次指令超过 `cmd_watchdog_s` 时发送零速；距上次指令超过 `cmd_watchdog_s + zero_release_s` 后停止发送，释放伺服（人员可推动机器人）。stop 本身也算一条零速指令，因此 stop 后约 1.5 s 释放 | 0.5 s / 1.0 s |
| 6. 导航门控 | 位姿/状态新鲜度、地图健康、雷达就绪、起步前静止、停滞/超时判定 | 见配置 |
| 7. **硬件急停** | 现场人员 | **必须配备** |

**已知盲区（请告知现场人员）**

- 雷达正后方存在盲区：后退时没有雷达反射层保护。
- 低矮地面物体（线缆、横杆、倒地的机器人）通常无法被雷达检测到。
- 玻璃门、玻璃隔断通常会被雷达穿透。
- 原地转向不受雷达反射层约束，转向时车身侧面可能碰到近处物体。
- G1D 默认 `navi.lidar.require: false`：扫描断流或订阅失败时，反射层不拦截前进。现场确认雷达工作正常后，建议在 `config.yaml` 中设为 `true`。

**R1D 的雷达与 G1D 不同**（安装于底盘前端，离地 20 cm，向前下方俯倾 30°）：

- 可检测近处 5 cm 以上的低矮物体（箱子、桌腿、倒地的物体；检测阈值随距离升高，2 m 处约 8 cm），G1D 的单线雷达无法检测此类物体。
- **无法检测近处高于视场上沿的悬空物体**：0.5 m 处高于 0.4 m、1 m 处高于 0.6 m 的物体（桌面、外伸台面、横杆）。底盘能够通过不代表上半身能够通过。
- 后方被机身遮挡，仅对 0.6 m 以上的物体有读数；后方无数据时，后退不受拦截。
- 低于 5 cm 的贴地物体（线缆、门槛）被视为地面；玻璃、镜面、黑色吸光表面可能没有回波。
- 雷达无数据（断流或数据过期）时前进速度归零（`robot.r1d.lidar_require`）。设置 `NAVI_NO_LIDAR=1` 关闭后，退回为「相机帧有效时才允许平移」。
- **已知缺陷**：雷达窗被遮挡（布、袋子、手），或障碍物贴在车头前缘 3 cm 以内时，行进路径上没有有效回波，反射层会按「前方无障碍」放行，而不是归零。运行前确认雷达窗无遮挡；转向后车头贴近障碍物时，不要依赖反射层。
- R1D 线速度上限 1.0 m/s、角速度上限 0.4 rad/s（`robot.r1d.max_vx` / `max_vyaw`）；实际速度由限速包络根据前方路径净空决定：以 1 m/s 行驶需要前方路径约 2 m 净空；接近障碍物时以 0.2 m/s 缓慢靠近，至 0.45 m 处停止（`robot.r1d.lidar_headway_s` / `lidar_creep_mps`，或 `navi.lidar.headway_s` / `creep_mps`）。`mps_full` 尚未经实测核对，首次高速运行前，建议使用 `get_localization` 在一段直线上核对实际速度。
- R1D 相机：左目 544×448 为双目校正后的针孔图像（直线保持笔直，非鱼眼），水平视场约 105°，画面中心物体显得较小、较远。每帧向模型提供两幅图像：中央 50% 区域放大至原尺寸的「常规视场」图像（约 66°，用于判断前方情况与距离）以及超广角全幅图像（用于观察两侧、搜索目标）。可通过 `robot.r1d.camera_crop_ratio` 调整放大比例。

---

## 8. 配置参考

配置文件为 `src/navi_agent/config.yaml`，从 `config.example.yaml` 复制，不纳入版本控制。只需填写要修改的配置项，服务会将其与 `src/navi_agent/config.py` 中的 `DEFAULT_CONFIG` 递归合并，其余项保留默认值。没有 `config.yaml` 时全部使用默认值，地图导航保持关闭。配置仅在服务启动时读取，修改后需重启。

| 段 | 关键键 | 说明 |
|---|---|---|
| `net` | | DDS 网卡；`auto` 表示自动检测 |
| `robot` | `model` `probe_timeout_s` `candidates` `g1d.*` `r1d.*` | 机型识别（`auto` 为首次连接机器人时只读探测一次）；`robot.g1d` / `robot.r1d` 可覆盖机型参数（`max_vx`、`max_vyaw`、`lidar_require`、`camera_crop_ratio` 等） |
| `motion` | `move_speed` `rate` `ramp_time` `cmd_watchdog_s` `zero_release_s` | 离散运动与驱动参数 |
| `camera` | `host` `request_port` `camera` `eye` `max_frame_age_s` | image_server 相机 |
| `navi.guard` | `max_move_distance_m` `max_turn_angle_deg` | 离散运动硬上限 |
| `navi.sequence` | `max_vx` `max_vyaw` `max_accel_mps2` `max_duration_s` `max_distance_m` `max_blind_m` `blend_s` `watchdog_s` `horizon_factor` | 序列模式 |
| `navi.lidar` | `enabled` `stop_m` `slow_m` `headway_s` `creep_mps` `require` `corridor_half_width_m` `yaw_offset_deg` | 雷达反射层 |
| `navi.observer` | `port` | 观测状态接口端口，设为 0 时关闭 |
| `navi.memory` | `dir` `venue` | 拓扑记忆与 R1D 雷达地图的存储目录及场地名（更换场地时请修改 `venue`） |
| `navi.navigation` | `enabled` `map_id` `frame` `goal_frame` `places_file` `odom_required` `receipt_time_freshness` `max_distance_m` `timeout_s` `stall_s` `pre_align_deg` … | 地图导航 |
| `navi.navigation.mapping` | `backend` `trust_slam_state` `require_map_health` `min_quality` | 地图健康门控 |

各键的具体含义参见 `src/navi_agent/config.py` 中的行内注释。

**Python 侧环境变量**

| 变量 | 作用 |
|---|---|
| `NAVI_NO_CAMERA=1` | 不连接相机（仅用于运动调试） |
| `NAVI_NO_LIDAR=1` | 不启用雷达反射层（仅限调试，实体机器人上禁止使用） |
| `NAVI_CAMERA_HOST` / `NAVI_CAMERA_REQUEST_PORT` | 覆盖相机地址 |
| `NAVI_LIDAR_RECORD=1` | R1D：将雷达原始点云录制至技能目录下的 `logs/lidar-*.bin.gz`（用于离线回放调参，约 1.5 GB/10 分钟） |
| `NAVI_SLAM_NEW_MAP=1` | R1D：将旧雷达地图与点位重命名保留，重新建图 |

**使用的 DDS 话题 / 服务**

| 话题/服务 | 方向 | 用途 |
|---|---|---|
| `agv` RPC（API 1001 Move） | 写 | 底盘速度 |
| `rt/hispeed_state` | 读 | 连接判定 |
| `rt/slamware_ros_sdk_server_node/scan` | 读 | 雷达反射层（G1D） |
| `rt/robot_pose` `rt/agv/odom` `rt/slamware_ros_sdk_server_node/state` | 读 | 定位、实测速度、SLAM 状态 |
| `rt/move_base_simple/goal` `rt/slamware_ros_sdk_server_node/cancel_action` | 写 | 导航目标 / 取消 |
| `rt/slamware_ros_sdk_server_node/map` `.../robot_basic_state` `rt/tf` | 读 | 地图健康（只读） |

**R1D 额外使用的 UDP 端口**（均位于机器人内网，不经过 DDS）

| 端口 | 方向 | 用途 |
|---|---|---|
| 5002 | 接收 | 头部左目 RTP-H264（需 `stereo_patch_pc1` 服务运行，启动时自动开启） |
| 56301 | 接收 | Livox Mid-360 点云（约 2084 包/秒） |
| 56100 ← 56101 | 发送/接收 | Livox 命令：启动及数据流中断时核对点云与 IMU 的目的地址，若非本机则修改为本机（**会写入雷达配置，且持久生效**；`livox_diag --set-host` 只能改回点云地址；可通过 `robot.r1d.lidar_auto_host: false` 关闭） |
| 56201 | 接收 | Livox 状态广播（仅供诊断工具使用） |
| 56401 | 接收 | Livox 内置 IMU（陀螺仪，供航向闭环使用） |

---

## 9. 系统提示词（persona）与观测推送

### 9.1 persona

`host/navi-persona-r1d.txt` 和 `navi-persona-g1d.txt` 分别是 R1D、G1D 的系统提示词，说明机器人如何观察、移动和处理异常。`start.sh` 根据 `r1d`/`g1d` 参数选择文件，默认使用 `r1d`；web 模式将内容加入 preset，headless 模式则使用 `personaPrefix`。

两份文件的通用规则相同，修改时须同步更新。机型相关的差异包括：

- R1D 版说明每次观测返回的两幅图像、Mid-360 雷达特性，以及雷达定位和点位保存方法，不提供 `navigate_to`。
- G1D 版说明如何以单目画面中的底盘白色弧形判断距离、如何进行 SLAMWARE 预检和调用 `navigate_to`，以及如何处理 `stalled`。速度表按 G1D 的 0.5 m/s 上限编写。

提示词分为以下几部分：

- **〇 最高约束**：如何判断能否通行（仅考察与底盘同宽的行进走廊）、人员优先、自主完成任务而不求助、不得作弊、雷达反射层的能力边界、如何处理「[观测]」消息。
- **一 感知 / 度量导航**：何时可以使用 `navigate_to`（仅限已存在的标定点）、预检时等待订阅就绪、失败后的重试和替代做法、`stalled` 的处理。
- **二 运动**：任务分解、按阶段选择工具、通过门与走廊、序列使用方法（含三条人工遥控记录的参考曲线）、速度与步长、终止条件。
- **三 拓扑记忆**：何时调用 `memory_context`、如何比图确认地点、何时保存地点 / 地标 / 未探索出口。

这些规则大多针对真机测试中遇到的问题，例如长时间贴墙行驶、未识别倒地的低矮障碍物而碰撞、目标位于身后时 goto 无响应。修改前建议先了解对应问题。

观测插件通过 preset `navi` 识别 web 会话。persona 首句中的 `Unitree Navi Agent` 则是为 headless 会话预留的识别标记。当前 DSH 不向插件提供会话的系统提示词，因此 headless 会话暂时无法被识别，观测推送和运行日志中的模型旁白也不可用。如需更改该标记，须同步修改 `host/navi-observer/scope.ts`。

**web 模式注意事项**：DSH web 会话默认使用 `standard` preset，包含编程类 persona 和 bash/edit 等工具。`start.sh` 会生成 `navi` preset，使用 `navi-persona-<g1d|r1d>.txt` 中的提示词，配置压缩链和任务目标工具，再将注册表的 `default` 设为 `navi`。机器人工具由宿主层提供，`navi` preset 不加载编程类工具。已有会话会保留创建时的 preset，因此修改提示词后需**重启并新建会话**。

### 9.2 观测推送

`navi_mcp_server.py` 在 `127.0.0.1:3082` 提供只读状态接口 `/state` 和 `/frame.jpg`。相机页也从 `/state` 获取任务看板 `task` 和已记录房间 `places`。宿主插件 `host/navi-observer/` 每秒查询一次，在模型推理期间按以下规则推送消息：

- 序列执行完毕、底盘停稳超过 `NAVI_OBSERVER_IDLE_S` 秒（默认 4）后，推送一条「[观测] 已停稳…」。同一次停稳只推送一次。如果模型持有的画面已在停稳后拍摄，则不再提醒，避免重复消息导致模型反复作答、对话无法结束。例如，`submit_sequence` 的 `observe_after_s` ≥ 运动时长时就属于这种情况。
- 序列执行中被雷达反射层拦截时，推送一条「[观测] 前方 x.xx m 被拦…」。同一障碍持续超过 8 s 后，中断当前推理，让模型根据新情况重新决策；任务目标处于自动续行状态时，只插入提醒，不中断推理（见 9.3 节）。序列停止后，反射层读数不再更新，因此不再上报拦截消息。

插件只读取状态，不调用机器人工具，仅对 Navi 会话生效，目前只能识别 web 会话（见 9.1 节）。设为 `NAVI_OBSERVER_PORT=0` 可关闭插件，`NAVI_OBSERVER_IMAGES=0` 可关闭消息附图；`NAVI_OBSERVER_INTERVAL_MS` 和 `NAVI_OBSERVER_INTERRUPT_IDLE_S` 分别用于调整轮询间隔和停稳后中断推理的等待时间。

### 9.3 任务目标与自动续行

模型有时会在任务中途回复「本轮未完成」并结束，例如遇到通道入口变窄时停下，等待用户发送「继续」。为减少这类中断，`navi` preset 配置了 DSH 的任务目标工具：模型开始任务时调用 `create_goal`；任务未完成而模型结束回复时，宿主的 `goal-round-driver` 会自动发送 `<goal_round>`，让模型继续执行；完成后由模型调用 `update_goal(complete)`。

- UI 上的停止按钮或 `/goal pause` 会暂停目标，停止自动续行。用户在对话中说「停 / 撞了 / 先别动」时，persona 要求模型先执行 `stop`，再调用 `update_goal(pause)`。**UI 停止按钮只停止模型推理，不会调用机器人的 `stop`。** 正在执行的序列会继续，直到计划走完或序列看门狗触发（默认至少 8 s）；需要立即停车时使用硬件急停。
- 上游临时故障（网关 503 / 403、流中断）会使驱动器停用自动续行。观测插件会依次在 10 s / 30 s / 60 s 后尝试重新启用，最多连续 3 次；用户主动取消的目标不会被重新启用。
- 可随时通过 `/goal` 查看或控制当前目标。

---

## 10. 接入其他 MCP 宿主

`navi_mcp_server.py` 是标准的 stdio MCP 服务，不依赖 DSH。任何支持 MCP 的宿主均可直接挂载：

```bash
python src/navi_agent/navi_mcp_server.py            # 实体机器人
python src/navi_agent/tools/stub_mcp_server.py      # 离线 stub
```

以 Claude Desktop 为例（`claude_desktop_config.json`）：

```json
{
  "mcpServers": {
    "robot": {
      "command": "/path/to/.venv/bin/python",
      "args": ["/path/to/unitree_roboskills/unitree_gpt6/g1d_r1d_navigation/src/navi_agent/tools/stub_mcp_server.py"]
    }
  }
}
```

注意事项：

- **stdout 是 MCP 协议通道**，所有日志均输出至 stderr；二次开发时请勿在 sidecar 中 `print` 到 stdout。
- 将 `host/navi-persona-<g1d|r1d>.txt`（按机型选择）的内容作为系统提示词提供给模型（工具名前缀 `mcp__robot__` 需按宿主的实际命名替换）。
- 宿主的工具调用超时建议 ≥ 60 s（`move`/`turn` 为阻塞调用；`submit_sequence` 的 `observe_after_s` 最长 20 s）。
- 宿主需将工具返回的图像块传给模型，并限制每次请求携带的历史图像数量。DSH 中设置为 `maxImagesPerRequest: 4`，以免历史图像占用过多上下文。
- 观测推送插件为 DSH 专用；其他宿主不接入该插件也能正常工作，只是模型推理期间不会收到主动提醒。

---

## 11. 移植到其他机器人

需替换的部分集中在 `services/` 层，上层工具层与执行器无需修改：

| 替换对象 | 位置 | 接口约定 |
|---|---|---|
| 底盘驱动 | `services/robots/g1d.py` 中的 `RobotController`（R1D 参见 `services/robots/r1d.py`） | `send_command(vx, vy, vyaw)`、`start()`/`stop()`、`get_pose()`、`connected` |
| 相机 | `services/camera.py` 中的 `CameraStream` | `start()`、`get_latest_frame() -> bytes(JPEG) \| None`、`connected`/`error` |
| 雷达 | `services/lidar.py` 中的扫描订阅 | 提供最新的 `ranges` 及角度参数；`clamp(vx)` 逻辑可复用 |
| 地图导航 | `services/navigation.py` 中的 `SlamwareBackend` | 发布目标、取消、定位回调 |

同时需调整 `config.py` 中的速度上限与 `navi.sequence.max_vx/max_vyaw`，以及 persona 中与底盘外观相关的描述（如「底盘白色弧形」等）。

---

## 12. 常见问题

**问：Web UI 中模型表现得像编程助手，并且可以看到 bash 等工具。**

这说明 `navi` preset 未生效，会话仍在使用 `standard`。请通过 `start.sh` 启动，并**新建会话**；手动执行 `pnpm dsh` 或继续使用旧会话，都可能保留原 preset。可运行 `pnpm dsh --profile web --patch … --dump-config`，检查 `agent-preset-registry.config.default` 是否为 `navi`，以及是否存在 `preset-navi` 条目。

**问：模型表示无法看到图像。**

所选模型不支持图像输入，或 provider patch 中该模型未配置 `inputModalities: [text, image]`。

**问：DashScope 返回 400 错误。**

`maxTokens` 超过了端点上限；`navi-bailian.patch.yaml` 中已设置为 8192。

**问：首次执行 `navigation_preflight` 时报告 `pose_stale` / `lidar_not_ready`。**

订阅刚建立时，数据可能尚未就绪，服务端会短暂重试。如仍失败，请等待数秒后重新检查；若连续多次失败，再排查连接或配置。

**问：`navigate_to` 返回 `running`，但底盘不动或进入 `stalled`。**

`stalled` 通常表示机载避障阻止了移动，例如前方有低矮物体。目标位于机器人后方时，机载 goto 也可能不响应，可启用 `pre_align_deg: 60`，让机器人先转向目标。

**问：机器人停止后，人员无法推动。**

正常情况下，距最后一条指令（包括 stop 发出的零速）超过 `cmd_watchdog_s + zero_release_s`（默认 1.5 s）后，将停止发送指令并释放伺服。若仍无法推动，请检查是否有其他程序在持续发送 `Move(0,0,0)`。

**问：R1D 的 `get_status` 中 `lidar.state` 不为 `healthy`，前进始终被拦截（`no_data`）。**

先停止服务端，再运行 `cd src/navi_agent && python -m tools.livox_diag`。工具会依次检查雷达状态广播、点云目的地址和点云包率，并通过地面拟合核对安装外参。常见原因有：

- 点云目的地址不是本机。出厂配置可能指向其他地址，服务端启动时会自动修改，也可用 `--set-host <本机IP>` 手动修改。
- 其他程序占用了 56301 端口。端口复用时不会报错，但点云会被后绑定的程序接收，因此运行 `livox_diag` 前必须先停止服务端。
- 更换过雷达或调整过安装位置。需将工具输出的实测外参写入 `config.yaml` 的 `robot.r1d`。

**问：R1D 没有画面（`observe` 返回相机未连接）。**

机器人重启后，`stereo_patch_pc1` 默认不会启动，5002 端口没有视频流。Navi 服务端启动时会自动关闭 `video_hub` 并开启 `stereo_patch_pc1`，约需 10 秒。如仍无画面，运行 `cd src/navi_agent && python -m tools.r1d_diag --net <网卡>`，查看两个服务的状态（0 = 运行，1 = 停止），并确认本机已安装 GStreamer（见 3.1 节）。

**问：多网卡主机上 DDS 无法连接。**

在 `config.yaml` 中将 `net` 显式设置为连接机器人内网的网卡名称。

---

## 13. 无动作预览与测试

### 13.1 无动作预览（仓库统一 CLI）

`src/main.py` 提供与仓库其他技能一致的 `preview` 入口。它根据输入名称精确匹配 `config/default.yaml` 中的 `waypoints`，只输出符号化计划，**不连接机器人、不调用模型**，与前述真机运行时（`src/navi_agent/`）相互独立。在仓库根目录下运行：

```bash
python -m unitree_gpt6.g1d_r1d_navigation.src.main --instruction home
bash unitree_gpt6/g1d_r1d_navigation/scripts/preview.sh
```

### 13.2 测试

使用已安装 `navigation` 和 `dev` 依赖组的导航环境。

```bash
# 仓库统一检查（在仓库根目录下执行）
for skill in g1d_r1d_navigation/common g1d_gomoku g1d_r1d_navigation; do
  python -m unittest discover -s "unitree_gpt6/$skill/tests" -p 'test_*.py'
done
python -m ruff check . && python -m ruff format --check .

# 从仓库根进入技能目录，执行以下两组检查
cd unitree_gpt6/g1d_r1d_navigation
# 真机运行时的单元测试：不连接硬件、不需要 SDK
pip install pytest
(cd src/navi_agent && python -m pytest -q)

# 宿主插件的纯函数单元测试（Node ≥ 22.6，无额外依赖）
(cd host/navi-observer && node --experimental-strip-types --test *.test.ts)
```

真机验证不在自动化测试范围内。在实体机器人上运行之前，请按照 [第 3.6 节](#36-现场检查清单) 与仓库 [安全要求](../../docs/safety.zh.md) 逐项确认。

---

## 14. 致谢、出处与许可证

- **[DeepSeek-Harness](https://github.com/deepseek-ai/deepseek-harness)**（MIT）：宿主框架。本项目通过 patch 与插件对其进行扩展，未修改其源代码。
- **Unitree SDK2 Python**（`unitree_sdk2py`）：DDS 通信与 RPC 客户端基础。`services/g1_agv_client.py` 依据 `unitree_sdk2` C++ 版 `g1/agv` 的 AgvClient 接口，以 Python 重新实现。
- **SLAMWARE ROS SDK**：`services/navigation.py` 与 `services/slamware_idl.py` 依据 SLAMWARE ROS SDK 公开的话题与消息格式定义 DDS 类型（未引入其运行时依赖）。

本技能随仓库以 [Apache-2.0](../../LICENSE) 许可证发布。DeepSeek-Harness、unitree_sdk2_python、MCP Python SDK 等第三方依赖各自遵循原有许可证，不随本仓库分发。

请勿提交现场配置（`src/navi_agent/config.yaml`、`navi.env`）、地图与点位、拓扑记忆库、相机帧或运行日志（`logs/`）；这些路径已写入技能目录的 `.gitignore`。
