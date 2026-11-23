# R1-D 机器人绘画

[English](README.md) | 中文

代理工作流见 [Skill](SKILL.zh.md)。

面向双臂各 7 轴、带夹爪的 Unitree R1-D（R1-A7-D）。完成 **规划 → 测量 → 记录 → 编译导出 → 执行**：代理根据参考图设计笔画，测量现场并保存数据，再离线生成完整关节轨迹和 `run.sh`；执行时无需模型逐笔指挥。

## 安装

需要 Linux、Python 3.10+ 和到机器人的有线网络。在**仓库根目录**执行：

```bash
python3.10 -m venv unitree_gpt6/r1d_draw/.venv
source unitree_gpt6/r1d_draw/.venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r unitree_gpt6/r1d_draw/requirements.txt
export PATH="$PWD/unitree_gpt6/r1d_draw/scripts:$PATH"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
r1d-draw --help
```

| 依赖 | 用途 | 开源许可 |
| --- | --- | --- |
| [NumPy](https://github.com/numpy/numpy) | 数值计算与轨迹处理 | BSD 3-Clause |
| [SciPy](https://github.com/scipy/scipy) | 数值 IK | BSD 3-Clause |
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio)，安装包 `pin` | URDF 与正向运动学 | BSD 2-Clause |
| [Paramiko](https://github.com/paramiko/paramiko) | 到机器人的有线 SSH | LGPL 2.1 |

### 为什么走 SSH

R1-D 的 DDS 启用了安全认证，工作站能发现参与者但收不到话题。因此读取状态、拍照和控制都由放到机器人上运行的程序完成，工作站通过有线 SSH 启动它们并读写请求/状态文件。连接前会核对配置的网卡 MAC，避免连到别的机器。

SSH 密码只从环境变量读取，配置文件里只写变量名：

```bash
export R1D_SSH_PASSWORD=...
```

### URDF

仓库不包含机器人模型。下载官方模型 [R1-D URDF](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/r1d_description/r1d.urdf)（同目录的 [meshes](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/r1d_description/meshes) 可选；仓库许可 [BSD-3-Clause](https://github.com/unitreerobotics/unitree_ros/blob/master/LICENSE)），并在 `device.json` 的 `model.urdf` 填写其路径。URDF 用于 IK、笔尖 FK 和机器人上的重力补偿，须与实际设备一致。

## 目录

| 路径 | 内容 |
| --- | --- |
| [SKILL.zh.md](SKILL.zh.md)、`SKILL.md` | 代理的规划、测量、任务交付和执行流程 |
| `src/robot.py` | 机器人接口：`connect`、`state`、`photo`、`take_over`、`move`、`stop`、`relax`、`release`、`status` |
| `src/onboard/` | 在机器人上运行的只读采样程序和控制器 |
| `src/calibration.py`、`task.py`、`job.py`、`kinematics.py` | 纸面标定、规划、作业校验、IK 与编译 |
| `src/timing.py`、`runner.py` | 轨迹定时和整幅作业执行、进度记录与继续 |
| `src/simulation.py` | 不连接机器人的模拟机器人和玩具运动学 |
| [config/device.example.json](config/device.example.json) | 设备、相机、模型和存储配置样例 |
| `examples/` | 只读示例：读取关节、保存相机画面 |
| `tests/` | 离线测试，不连接机器人 |

## 先检查连接和图像

复制配置样例并填写机器人地址、网卡、MAC 和相机：

```bash
cp unitree_gpt6/r1d_draw/config/device.example.json ./device.json
python unitree_gpt6/r1d_draw/examples/read_state.py --config ./device.json --output ./state.json
python unitree_gpt6/r1d_draw/examples/capture_images.py --config ./device.json --output ./images-1
```

两个示例都只读，不接管机械臂。`state.json` 中的数据：

| 数据 | 顺序与单位 |
| --- | --- |
| `q` / `dq` / `tau` / `temperature_c` | 35 个电机槽位；rad、rad/s、Nm、°C |
| `arms.left` / `arms.right` | 每臂 7 轴（槽位 15–21、22–28）：shoulder_pitch、shoulder_roll、shoulder_yaw、elbow、wrist_roll、wrist_pitch、wrist_yaw |
| `waist_yaw`、`head`、`lift` | 槽位 13；29/30（俯仰、偏航）；12 |
| `grip` | 左、右夹爪，设备原生单位；槽位 31、33 |
| `commanded`、`command_frames` | 采样期间是否有程序在发命令 |

相机在配置的 `cameras` 中逐个定义：`kind` 为 `rtsp`（如头部相机）或 `v4l2`（如接在相机电脑上的腕部相机），`host`/`interface`/`mac` 指明它接在哪台机器上。图像不裁剪、不镜像，多相机之间没有曝光同步。

## 在 Python 中控制机器人

```python
from unitree_gpt6.r1d_draw.src.robot import connect

robot = connect("./device.json")
print(robot.state()["arms"]["right"])  # 只读
png = robot.photo("head")  # 只读，返回 PNG 字节
robot.take_over()  # 原地接管：先重复实测姿态和力矩，3 秒内过渡到模型重力补偿
robot.move({"points": [{"q": {"28": 0.25}, "duration_s": 3.0, "dwell_s": 0.5}]})
robot.stop()  # 取消当前路线，原地保持
robot.relax()  # 双臂刚度和前馈 3 秒内降到零，只留阻尼；夹爪继续夹持
robot.release()  # 停止发命令；电机保持最后一帧，不卸力、不收臂
robot.close()
```

路线点的 `q` 以电机槽位为键、弧度为值，只能包含腰部、双臂和头部。`profile` 省略时每个点起止都静止；`ease_in`、若干 `linear`、`ease_out` 组成一段连续运动，中间点不停。控制器拒绝超出关节限位、峰值关节速度超过 0.3 rad/s 或相邻点速度跳变超过 0.01 rad/s 的路线。

接管需要：没有其他程序在发命令、机械臂静止、电机不过热（72 °C 以下开始，75 °C 停止）。出现故障时控制器停止发命令，电机保持最后一帧；用 `status()` 查看原因。

## 从零创建绘画任务

### 1. 规划

```bash
r1d-draw init-task --output ./my-task
```

输出目录必须是新目录：

| 文件 | 填写内容 |
| --- | --- |
| `device.json` | 机器人与相机连接、模型和状态存储 |
| `strokes.json` | 纸张尺寸、颜色、笔画顺序与纸面路径 |
| `measurements.json` | 纸面、笔刷、颜料、水杯、夹爪、腰部和进入/收臂路线 |

### 2. 配置设备和模型

- `device`：机器人地址、SSH 用户、网卡名和 MAC、本机源地址（可选）、密码所在的环境变量名。
- `cameras`：每路相机的主机、类型和来源。
- `model`：`urdf`；可选 `base_frame`（默认 `pelvis_link`）、`left_frame`/`right_frame`（默认 `<手>_wrist_yaw_link`）和 `left_tool_transform`/`right_tool_transform`（末端 frame 到笔尖的 4×4 变换）。
- `motion`：速度与容差，见[运动参数](#运动参数)。
- `workspace`（可选）：`lower`/`upper` 笔尖允许范围，离线检查时使用。
- `storage`：执行记录目录；同一机器人共用同一个目录。

### 3. 测量并记录

```bash
r1d-draw observe --config ./my-task/device.json --label rest --output ./my-task/rest.json
r1d-draw photo --config ./my-task/device.json --camera left_wrist --output ./my-task/wrist.png
```

`observe` 只读，保存关节和两只手的笔尖位姿。需要运动的测量用上一节的 Python 接口。在 `measurements.json` 中记录：

- `paper`：纸张尺寸、至少 3 个非共线锚点和至少 1 个独立核对点（纸面坐标 `uv_m` 与机器人坐标 `xyz_m`）。
- `brush`：落笔姿态 `rpy_rad`、笔宽、单笔最大长度、笔画之间的抬笔高度 `clearance_m`。
- `materials`：每种颜色（以及水）一个材料站，`action` 为完整的位姿序列，`contact_index` 指明接触/浸入点，`dwell_s` 为在该点的停留时间。
- `robot`：绘画手、`grip_q2`、`waist_yaw`、进入路线 `entry_q14`、收臂路线 `park_q14`（首项为 `null`，编译时替换为最后抬离点）、转移高度 `transfer_z_m` 和 `clearance_review`。

FK 反馈不代表笔刷已经接触。确认实际接触后填写 `contact_verified=true` 和 `source`。

### 4. 编译并导出

```bash
r1d-draw prepare --config ./my-task/device.json \
  --strokes ./my-task/strokes.json --measurements ./my-task/measurements.json \
  --job-id my-painting-attempt-1 --output ./ready/my-painting-attempt-1
```

离线完成纸面拟合、笔画映射、作业编排、全部 IK 和路径检查。大幅作品（上百笔）的编译需要几分钟。输出目录必须是新目录：

| 产物 | 用途 |
| --- | --- |
| `preview.svg` | 根据最终笔画和笔宽生成的覆盖预览 |
| `plan.json`、`build_plan.py` | 完整末端动作计划和复现脚本 |
| `job.json` | 全部批次和关节轨迹 |
| `calibration.json`、`device.json`、`manifest.json`、`report.json` | 标定、配置、文件校验和编译报告 |
| `run.sh` | 直接启动该作业 |

修改源输入后重新 `prepare`；直接编辑导出文件会触发校验失败。

### 5. 执行

核对预览、报告、现场布局、起点、夹持、腰部与完整通道，确认没有其他程序在控制机器人，然后运行：

```bash
bash ./ready/my-painting-attempt-1/run.sh
```

执行器检查腰部和夹爪与作业一致、双臂位于 `entry_q14` 首点，然后接管、走进入路线、逐批执行、收臂并继续保持。另一终端：

```bash
r1d-draw status  --config ./ready/my-painting-attempt-1/device.json
r1d-draw stop    --config ./ready/my-painting-attempt-1/device.json
r1d-draw relax   --config ./ready/my-painting-attempt-1/device.json
r1d-draw release --config ./ready/my-painting-attempt-1/device.json
```

中断后继续：

```bash
bash ./ready/my-painting-attempt-1/run.sh --resume          # 从未完成的批次继续
bash ./ready/my-painting-attempt-1/run.sh --resume --lift   # 先把笔竖直抬到转移高度
```

已完成的批次不会重画；被中断的那个批次会从头再做一遍。笔尖低于该批次的抬笔高度时，不加 `--lift` 会拒绝继续。

## 运动参数

在源 `device.json` 的 `motion` 中调整，修改后重新 `prepare`：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `max_speed_rad_s` | 0.09 | 笔在空中时的峰值关节速度 |
| `contact_speed_rad_s` | 0.05 | 落笔、离开、蘸料和笔画中的峰值关节速度 |
| `posture_speed_rad_s` | 0.05 | 进入和收臂路线的峰值关节速度 |
| `stroke_speed_m_s` | 0.01 | 笔画中的笔尖速度 |
| `travel_speed_m_s` | 0.05 | 空中转移的笔尖速度（受关节速度限制时自动降低） |
| `fk_position_tolerance_m` / `fk_orientation_tolerance_rad` | 0.003 / 0.08 | 编译时 IK/FK 的一致性要求 |
| `start_tolerance_rad` | 0.04 | 开始时双臂与进入路线首点的允许差 |
| `grip_tolerance` / `waist_tolerance_rad` | 0.18 / 0.03 | 开始时夹爪、腰部与作业的允许差 |
| `check_step_rad` | 0.02 | 离线路径检查的关节采样间距 |

控制器内置的保护：反馈超过 0.25 秒未更新、出现其他命令发布者、电机故障、温度 75 °C、跟随偏差 0.15 rad、关节速度 0.5 rad/s、命令间隔 0.08 秒。

## 数据格式

| 输入 | 主要字段 |
| --- | --- |
| `strokes.json`，`r1d-draw-strokes/v1` | `purpose`、`paper_size_m`、`palette`、有序 `strokes`（`id`、`color`、`points_uv_m`，可选 `width_m`）；可选 `reload_every`、`wash_station`、`stroke_sample_spacing_m` |
| `measurements.json`，`r1d-draw-measurements/v1` | `paper`、`brush`、`materials`、`robot` |
| 编译作业，`r1d-draw-job/v1` | `job_id`、`hand`、`grip_q2`、`waist_yaw`、进入/收臂路线和 `LOAD`/`PAINT`/`WASH` 批次；每批含 `action`、`joint_path_q14` 和到达该批的直线转移 `travel_q14` |

一个 `PAINT` 批次包含同一次蘸色的全部笔画，每笔用 `start`、`end` 指定 `action` 中的接触区间（含两端）。`LOAD`/`WASH` 用 `contact_index` 标明接触点。

## 离线体验与测试

```bash
r1d-draw demo-workflow --output ./offline-demo
python -m unittest discover -s unitree_gpt6/r1d_draw/tests -t .
```

`demo-workflow` 用合成数据和模拟机器人完成记录、编译、导出和执行，不连接机器人。运动学测试需要真实模型：`export R1D_DRAW_TEST_URDF=/path/to/r1d.urdf`，未设置时跳过。

## 已知限制

- 离线检查不含碰撞检测；整条通道需要人工复核并记录在 `clearance_review`。
- 只有一个转移高度；靠近机身处如需更低的转移高度，目前要通过材料站位姿自行安排。
- 增益为控制器内置的基础值，暂不可配置。
- 每个批次开始时的笔尖偏差修正需要在线求解 IK，批次之间会有几秒停顿。

代码使用仓库 [Apache-2.0 许可](../../LICENSE)。
