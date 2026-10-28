# G1-D 机器人绘画

[English](README.md) | 中文

代理工作流见 [Skill](SKILL.zh.md)。

面向固定底座、双臂各 7 轴、配备 Dex1 的 G1-D。完成 **规划 → 测量 → 记录 → 编译导出 → 执行**：代理根据参考图设计笔画，测量现场并保存数据，再离线生成完整关节轨迹和 `run.sh`；执行时无需模型逐笔指挥。

## 安装

需要 Linux（glibc ≥ 2.28）、Python 3.10–3.13 和 Git。在**仓库根目录**执行：

```bash
python3.10 -m venv unitree_gpt6/g1d_draw/.venv
source unitree_gpt6/g1d_draw/.venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r unitree_gpt6/g1d_draw/requirements.txt
export PATH="$PWD/unitree_gpt6/g1d_draw/scripts:$PATH"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
python -m pip check
robot-paint-execute --help
```

依赖由 [requirements.txt](requirements.txt) 管理，固定数值库、Pinocchio、Unitree SDK、OpenCV 和 PyZMQ 的版本；SDK 固定到具体 Git 提交。相机与绘画使用同一套依赖。需要保留完整仓库，并在每次运行命令或生成脚本的 shell 中设置 `PATH` 和 `PYTHONPATH`。

也可使用模块入口：

```bash
python -m unitree_gpt6.g1d_draw.src.main --help
```

| 依赖 | 用途 | 开源许可 |
| --- | --- | --- |
| [NumPy](https://github.com/numpy/numpy) | 数值计算与轨迹处理 | BSD 3-Clause |
| [SciPy](https://github.com/scipy/scipy) | 数值 IK | BSD 3-Clause |
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio)，安装包 `pin` | URDF、正向运动学与重力补偿 | BSD 2-Clause |
| [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python) | DDS 通信 | BSD 3-Clause |
| [CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds) | SDK 通信依赖 | EPL 2.0 / BSD 3-Clause，按上游许可 |
| [OpenCV](https://github.com/opencv/opencv-python) | JPEG 解码与 PNG 保存 | OpenCV 4.8+：Apache-2.0；二进制组件按各自许可 |
| [PyZMQ](https://github.com/zeromq/pyzmq) | 相机配置查询与图像订阅 | BSD 3-Clause；附带组件按各自许可 |

使用 `opencv-python` 提供图像处理，同一环境只安装一种 OpenCV 发行包。Python 3.11+ 或 ARM 上可能需要构建 CycloneDDS；按 [SDK 安装说明](https://github.com/unitreerobotics/unitree_sdk2_python#installation) 设置 `CYCLONEDDS_HOME` 后重新安装。机器人 URDF 自行提供。

### URDF 来源与适配

官方模型下载：

- [G1-D](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_d_description/g1_d.urdf)／[G1＋Dex1](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_mode_15_with_dex1_1.urdf)：仓库许可 [BSD-3-Clause](https://github.com/unitreerobotics/unitree_ros/blob/master/LICENSE)。

填写 `model.urdf`、全部非手臂关节的 `reference_positions`、左右末端 frame 和工具变换。当前支持固定基座、标量关节及标准 G1 腰部命名；完整 G1-D 模型的轮关节和躯干命名需适配，不能直接执行。下载时保留版本和许可，模型须匹配实际设备。

G1-D 腰部使用 12=yaw、13=pitch，14 不启用。`waist_q3` 为 `[yaw, 0, pitch]`；速度及增益按相同顺序，roll 项不使用。

## 目录

| 路径 | 内容 |
| --- | --- |
| [SKILL.zh.md](SKILL.zh.md)、`SKILL.md` | 代理的规划、测量、任务交付和执行流程 |
| `src/` | 标定、编译、任务导出、G1-D后端与执行器 |
| [config/device.example.json](config/device.example.json) | 设备、模型、增益、运动和存储配置 |
| `examples/` | 只读连通性、关节与相机示例，以及执行 API 示例 |
| [assets/golden_gate_bridge/](assets/golden_gate_bridge/) | 金门大桥参考图 |

## 先检查连接和图像

```bash
DRAW_DIR=unitree_gpt6/g1d_draw
IFACE=enp4s0
CAMERA_HOST=192.168.123.164
ip -br addr
ip route get "$CAMERA_HOST"

python "$DRAW_DIR/examples/check_connection.py" \
  --config "$DRAW_DIR/config/device.example.json" --interface "$IFACE" \
  --camera-host "$CAMERA_HOST" --timeout 5 --output ./captures/connection.json

python "$DRAW_DIR/examples/read_joints.py" \
  --config "$DRAW_DIR/config/device.example.json" --interface "$IFACE" \
  --samples 3 --interval 0.5 --output ./captures/joints.json
```

这两个示例只订阅状态或图像，不接管机械臂，不需要 URDF。连通性示例实际读取新鲜 DDS 反馈、查询相机配置并接收图像，结果写入 JSON。

| 数据 | 顺序与单位 |
| --- | --- |
| `q14` / `dq14` | 左臂 7 轴、右臂 7 轴；rad / rad/s；LowState 槽位 15–28 |
| 每臂关节顺序 | shoulder_pitch、shoulder_roll、shoulder_yaw、elbow、wrist_roll、wrist_pitch、wrist_yaw |
| `waist_q3` | `[yaw, 0, pitch]`；rad；对应槽位 `[12, null, 13]`，roll 为固定零，14 未使用 |
| `grip_q2` | 左、右；设备原生单位；内置 Dex1 槽位 31、33 |

相机使用 [TeleImager](https://github.com/unitreerobotics/teleimager) 或兼容的单消息 JPEG/ZMQ 服务。配置服务默认端口 60000，接收 `GET_DATA` 并返回每路 `zmq_port`；实际图像端口以服务配置为准。常见头部、左腕、右腕端口分别为 55555、55556、55557：

```bash
python "$DRAW_DIR/examples/capture_images.py" \
  --stream "head=tcp://$CAMERA_HOST:55555" \
  --stream "left_wrist=tcp://$CAMERA_HOST:55556" \
  --stream "right_wrist=tcp://$CAMERA_HOST:55557" \
  --warmup-frames 3 --timeout 10 --output ./captures/images-1
```

输出 PNG 原图和 `capture.json`，保留地址、尺寸、哈希与接收时间。图像不自动裁剪或镜像；多相机与关节采样没有曝光同步。相机独立于 DDS，需单独启动图像服务。没有配置服务但已知图像端口时，连通性示例可直接使用 `--stream "head=tcp://$CAMERA_HOST:55555"`。

DDS 超时时检查网卡、domain、topic/类型和机器人状态发布；相机超时时检查服务、配置端口和 `enable_zmq`。依赖安装方式与绘画相同。

## 从零创建绘画任务

### 1. 规划

```bash
robot-paint-execute init-task --output ./my-task
```

输出目录必须是新目录：

| 文件 | 填写内容 |
| --- | --- |
| `device.json` | 网卡、模型、增益和状态存储 |
| `strokes.json` | 纸张尺寸、颜色、笔画顺序与纸面路径 |
| `measurements.json` | 纸面、笔刷、颜料、水桶、夹爪、腰部和进入/收臂路线 |
| `probe.template.json` | 测量路线和采样点 |

按 [Skill](SKILL.zh.md) 分析参考图，先设计整幅画的颜色区域和笔画，再根据实测笔宽、载色笔长调整。可使用 [金门大桥参考图](assets/golden_gate_bridge/reference_gold_bridge.jpg)，也可换成自己的图片。

### 2. 配置设备和模型

填写 `device.json`：

- `device`：有线网卡、DDS domain、状态和命令 topic、内部或外部 Dex1。
- `gains`：本机适用的双臂、腰部和夹爪 PD 增益。
- `model`：URDF、左右末端 frame、笔尖工具变换及非手臂关节参考值。
- `storage`：执行记录目录和控制锁；同一机器人共用 `robot_id`、记录目录和锁。

`init-task` 从 `config/device.example.json` 读取默认增益：每臂肩部和肘部四轴为 `kp=80`、`kd=3`，腕部三轴为 `kp=40`、`kd=1.5`。可在生成的 `device.json` 中按设备调整；已有任务的配置不会自动覆盖。

URDF 用于 IK、笔尖 FK 和重力补偿。支持固定底座、活动关节为标量的模型。生成非手臂关节配置表：

```bash
robot-paint-execute model-info --urdf /path/to/g1.urdf --output model.json
```

1. 打开生成的 `model.json`，确认 URDF 路径和左右末端名称与本机模型一致。
2. 将 `reference_positions` 中的每个 `null` 填为对应关节在绘画时保持的位置。这里列出的是模型中除双臂以外的所有活动关节；旋转关节填写角度，单位为弧度。
3. 腰部参考设置为 `waist_yaw_joint = waist_q3[0]`、`waist_roll_joint = 0`、`waist_pitch_joint = waist_q3[2]`，即采用实际 yaw／pitch 反馈，模型 roll 固定为零。非零模型 roll 会被拒绝；映射正确仍不代表模型的连杆几何和负载已经匹配实际设备。
4. 把填好的字段更新到 `device.json` 的 `model` 对象中，保留原有的笔尖工具变换等其他配置，不要替换整个 `device.json`。

末端位姿的 `x/y/z` 以 URDF 根坐标系为基准，单位为米；`roll/pitch/yaw` 为 XYZ 欧拉角，单位为弧度。笔尖工具变换则描述末端坐标系到笔尖的相对位置和姿态。

内部 Dex1 使用 LowCmd/LowState 槽位 31、33。外部夹爪设置 `gripper_interface=external`，填写 `external_grippers.left/right` 的 `state`、`command` topic。

### 3. 测量并记录

先只读并保存当前关节与笔尖位置：

```bash
robot-paint-execute observe --config ./my-task/device.json \
  --label rest --output ./my-task/rest-observation.json
```

在测量模板中填写进入、采样、抬离和收臂路线，记录纸面至少 3 个非共线锚点、至少 1 个独立核对点、各色颜料和水桶接触点。试笔记录笔宽、可用笔长、接触高度和来源。保留已经接受的材料、夹持与姿态数据。

`capture_indices` 指定采样点；悬空拍照点同时列入 `lifted_capture_indices`，默认 `[]`，且必须是前者的子集。编译后执行测量：

```bash
robot-paint-execute compile-probe --config ./my-task/device.json \
  --probe ./my-task/probe.template.json --output ./my-task/probe.json
robot-paint-execute probe --config ./my-task/device.json --probe ./my-task/probe.json
```

采样写入状态目录 `attempts/<hash>/captures/`。FK 反馈不代表笔刷已经接触，记录默认 `contact_verified=false`。确认实际接触后，将数据写进 `measurements.json`，填写 `contact_verified=true` 和 `source`。

正常测量结束后沿 `park_q14` 收臂并保持。另一终端查询并释放，再进行后续作业：

```bash
robot-paint-execute status --config ./my-task/device.json
robot-paint-execute release --config ./my-task/device.json
```

再次测量时使用新的 `job_id` 并重新编译。

### 4. 编译并导出

```bash
robot-paint-execute prepare --config ./my-task/device.json \
  --strokes ./my-task/strokes.json --measurements ./my-task/measurements.json \
  --job-id my-painting-attempt-1 --output ./ready/my-painting-attempt-1
```

离线完成纸面拟合、笔画映射、作业编排、全部 IK 和路径检查。输出目录必须是新目录：

| 产物 | 用途 |
| --- | --- |
| `preview.svg` | 根据最终笔画和笔宽生成的覆盖预览 |
| `plan.json`、`build_plan.py` | 完整末端动作计划和复现脚本 |
| `job.json` | 全部批次和关节轨迹 |
| `device.json`、`manifest.json`、`report.json` | 配置、文件校验和编译报告 |
| `run.sh` | 直接启动该作业 |

输入规划、测量与标定一并保存。URDF 路径需在执行设备上有效。修改源输入后重新 `prepare`，直接编辑导出文件会触发校验失败。

### 5. 执行

核对预览、报告、现场布局、起点、夹持、腰部与完整通道，确保测量控制器已释放，然后运行：

```bash
bash ./ready/my-painting-attempt-1/run.sh
```

另一终端查询、停止与释放：

```bash
robot-paint-execute status --config ./ready/my-painting-attempt-1/device.json
robot-paint-execute stop --config ./ready/my-painting-attempt-1/device.json
# 已完成或停止后，结束本程序保持
robot-paint-execute release --config ./ready/my-painting-attempt-1/device.json
```

执行器在前台运行，不在线调用模型、求解 IK 或逐笔拍照。正常完成编译作业后继续带电保持；`job_completed=true` 且 `parked=true` 表示完成计划并收臂。运行中按 Ctrl-C 停止；已停止或完成后按 Ctrl-C 释放。停止或故障保留最后发布目标并取消后续动作；`release` 结束保持，不执行收臂。异常后的收臂需根据实际位置另行规划。

## 默认工作位、休息位和夹持

默认值定义在 [src/postures.py](src/postures.py)：

```bash
python -c 'from unitree_gpt6.g1d_draw.src.postures import default_work_q14; print(default_work_q14())'
python -c 'from unitree_gpt6.g1d_draw.src.postures import default_rest_q14; print(default_rest_q14())'
```

新任务预填 `entry_q14=[默认休息位, 默认工作位]`，`park_q14=[null, 默认休息位]`。首个入口点必须对应当前姿态；`null` 编译为最后抬离点。根据现场在路线中补充绕行点。修改工作姿态用 `entry_q14` 的终点，修改休息姿态用 `park_q14` 的终点，再编译。右臂绘画时，左臂保持入口终点中的左臂角度。

`grip_q2` 默认 `[0.0, 0.0]`，顺序为左、右，Dex1 原生单位范围 `0..5.45`。默认 `gripper_kp=[5,5]`、`gripper_kd=[0.05,0.05]`，前馈 `tau=0`；`tau_est` 是反馈值。测量与绘画模板中的夹持目标保持一致，工作和收臂期间不变。

规划或等待时可收回双臂，减少长时间悬臂。正常流程为 **进入工作位 → 测量/试笔/绘画 → 抬离 → 收臂 → 保持**；整幅绘画连续完成后统一收臂。

## 到位和运行检查

编译时用 IK/FK 检查规划关节是否对应目标位姿；运行时用实测关节的 FK 检查接触到位，两者使用各自的容差。默认行为如下，参数在源配置 `motion` 中调整，修改后重新编译导出：

| 检查 | 默认值与行为 |
| --- | --- |
| 编译 IK/FK | 位置 3 mm，姿态 0.08 rad |
| 接触点到位 | 位置 5 mm，姿态 0.08 rad，关节 0.04 rad |
| 悬空端点/拍照 | XYZ、姿态、关节目标偏差仅记录，不设到位门槛 |
| 最终收臂 | 关节偏差仅记录，等待稳定保持 0.2 秒 |
| 持续跟随异常 | 偏差超过 0.08 rad、持续 0.2 秒停止 |
| 需要停稳时 | 速度不超过 0.1；手臂/腰部为 rad/s，夹爪为原生单位/秒 |
| 接管 | 初始入口关节差不超过 0.04 rad；接管漂移在跟随容差内重新校验衔接路线 |
| 反馈 | 0.5 秒未更新视为过期；首次连接超时 10 秒 |

入口的悬空工作终点和普通批次抬离后直接衔接下一段；拍照、接触、洗笔和最终收臂按需等待稳定。最低净空、腰部/夹爪目标、限速、关节限位、配置的工作空间及停止状态检查仍生效。旧 `lifted_xy_tolerance_m`、`lifted_z_tolerance_m` 字段会被忽略。

## 重用规划，从第一笔重画

布局、笔刷和机器人配置未变时，可复用测量和笔画。每次执行使用新编号、新输出目录：

```bash
robot-paint-execute prepare --config ./my-task/device.json \
  --strokes ./my-task/strokes.json --measurements ./my-task/measurements.json \
  --job-id my-painting-attempt-2 --output ./ready/my-painting-attempt-2
bash ./ready/my-painting-attempt-2/run.sh
```

一个 `job_id` 只尝试一次，失败也可能占用编号。先查询原请求并排除原因，再创建新尝试；保留共享执行记录。已编译且尚未执行的作业可稍后启动。纸张、笔刷或材料位置变化时更新受影响测量；中断作业不自动续画。

## 数据格式与 API

| 输入 | 主要字段 |
| --- | --- |
| `strokes.json`，`astra-paint-strokes/v1` | `purpose="painting"`、`paper_size_m`、`palette`、有序 `strokes`；每笔含 `id`、`color`、`points_uv_m`，可选 `width_m` |
| `measurements.json`，`astra-paint-measurements/v1` | `paper` 锚点与核对点；`brush` 姿态、宽度和可用笔长；`materials` 各色与 `water` 的动作/接触/停留；`robot` 的夹爪、腰部、入口、收臂、转移高度与 `clearance_review` |
| 编译作业，`astra-paint-execute/v1` | `job_id`、`hand`、固定夹爪/腰部、完整入口/收臂路线和 `LOAD`/`PAINT`/`WASH` 批次 |

格式标识保留以兼容已有数据。`q14` 按左臂 7 轴、右臂 7 轴排列，单位 rad，夹爪和腰部另存。EEF 位姿中的 `x/y/z` 使用配置的 URDF 根坐标系、单位米；`roll/pitch/yaw` 为 XYZ 欧拉角、单位 rad。工具变换定义末端 frame 到笔尖的关系，腕部位置不能直接当作笔尖位置。

`width_m` 记录预计覆盖宽度，用于预览和纸面边界检查。实际线宽由笔刷、接触高度和载色量决定，应根据试笔结果填写；单独增大该字段不会自动改变落笔高度或增加填充路径。

纸面坐标 `u/v` 的方向需明确定义。标定根据纸面实测点拟合纸张的位置和倾斜程度，再将每个纸面路径点映射为机器人坐标。因此，纸面存在倾斜时，接触高度会随纸面位置变化，而不是整张纸共用同一个 Z 值。该模型适用于平整纸面；曲面需要另行适配。

编译作业的 `PAINT` 笔画用 `start`、`end` 指定 `action` 中的接触索引区间，包含首尾两点。`LOAD` 用 `contact_index` 标明蘸料接触点。

Python API 见 [examples/execute.py](examples/execute.py)，公开导入路径为 `unitree_gpt6.g1d_draw.src`。还提供 `calibrate`、`build-plan`、`compile`、`validate --model-check` 等独立命令。

## 离线体验

```bash
robot-paint-execute demo-workflow --output ./offline-demo
```

用合成数据完成测量记录、编译、导出和模拟执行，不连接机器人。查看 `offline-demo/demo_report.json` 与 `offline-demo/ready/preview.svg`。每次使用新的输出目录。

代码使用仓库 [Apache-2.0 许可](../../LICENSE)。金门大桥参考图由此前绘画项目的 `pic/gold_bridge.jpg` 复制，原文件未记录再分发授权；该图片不包含在上述软件许可的授权范围内。
