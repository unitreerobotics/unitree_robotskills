# R1-7a 马克笔整理

[English](README.md) | 中文

Agent 的做法见 [SKILL.zh.md](SKILL.zh.md)。

用 R1-7a 机械臂、Dex1-1 夹爪和腕部 RGB 相机，把桌面散落的马克笔按色号顺序放回马克笔盒（示例为4×6笔盒）。Agent 看腕部相机的图，调用这里的工具脚本，一步一步完成定盒、找笔、抓取、入槽和验证。

这份技能是在一台真机上摸索出来的做法，供其他 Agent 和开发者参考，实验里得到的结论和失败教训都写进了 SKILL。

## 硬件

| 部件 | 说明 |
| --- | --- |
| 机械臂 | R1-7a |
| 夹爪 | Dex1-1 |
| 相机 | 腕部 USB RGB 相机（JR0002），1280×720 MJPEG，水平视场约 109° |
| 场景 | 4×6 马克笔盒（槽距约 17.8 mm，盒内有方孔格栅）；深色或白色桌面 |

## 开始前的准备

需要三样东西。关节控制可以参考官方sdk；IK 和相机标定这里提供了一套工具。

**1. 关节控制（自备）。** 脚本和机器人之间只经过四个函数，把它们换成你的控制接口：

| 函数 | 要求 |
| --- | --- |
| `rt.state()` | 返回字典：`q`（实测关节角，7 个，rad）、`dq`、`tau`、`command_q`（最后下发的关节目标）、`armed`、`fault`（无故障为 `None`）、`busy`（轨迹进行中），以及 `gripper`：`q`、`dq`、`tau`、`q_min`、`q_max`、`opening_fraction`（0 合拢，1 全开） |
| `rt.move(q, speed)` | 下发关节目标，等到手臂真正静止后返回实测关节角；故障或超时抛异常 |
| `rt.gripper(opening)` | 夹爪开到行程的 `opening`（0–1），等电机停稳后返回 `state()['gripper']` |
| `grab.frame()` | 返回一帧比调用时刻更新的腕部相机 JPEG |

仓库里的实现是作者自己的控制程序的 HTTP 客户端（`127.0.0.1:8000`），只作示例。几点会影响结果：

- 作者的控制是关节 PD（Kp 50、Kd 5）加 URDF 重力前馈，没有积分项，稳态误差 0.01–0.025 rad，脚本用实测角和外环补偿（见 SKILL）。重力前馈里算上了末端负载（Dex1-1 加相机，0.62 kg，质心在 Link7 系 `[0, -0.09, 0]`）；不算时 J4 下垂 0.038 rad。
- 夹爪是位置环，夹紧力靠“闭合指令比实际开度小多少”产生。SKILL 里的闭合指令（0.08、0.13）、开度和力矩阈值都是 Dex1-1 在这种控制下的实测值，换了夹爪或控制方式要重新测。
- 停止和断开会卸力、手臂会掉；脚本从不调用它们。

**2. IK（本仓库提供）。** `src/ik/kinematics.py` 从官方 [r1-7a_sdk](https://github.com/unitreerobotics/r1-7a_sdk) 的 `urdf/URDF_R1-7A.tar.gz` 读右臂模型，做 FK、IK 和重力项；world 系在底板底面中心（`src/ik/frames.py`）。`src/robot/rt.py` 在上面加了标定得到的关节零位修正、多初值 IK（`ik_all`）和间隙检查。`rt.py` 里 `MODEL = …` 一行指向 URDF，改成你的 SDK 位置。

**3. 相机标定（本仓库提供）。** 脚本都依赖 `src/calib.json`，里面有相机内参、手眼、关节零位偏差和桌面平面。

`config/calib.example.json` 是作者这台 R1-7a 的标定结果，可以作经验值直接用：复制成 `src/calib.json`，再用 `calibration.check` 拍几帧标定板，看像素反投到桌面的误差。误差在 1 cm 以内可以先跑；超过就完整标定一次。其中各项能不能沿用不一样：

| 内容 | 取决于 | 沿用经验值 |
| --- | --- | --- |
| `K`、`dist`（相机内参、畸变） | 相机型号 | 同型号相机（JR0002）基本可用 |
| `link7_from_camera`（手眼） | 相机支架的安装 | 同样的支架基本可用 |
| `joint_zero_offset`（关节零位偏差） | 每台臂的编码器零位 | 因机而异，最好重标 |
| `world_from_board`、`board_top_plane`（桌面平面） | 桌子相对底座的高度和倾斜 | 因场地而异；抓取时的指尖高度靠它，换了桌子或底座位置必须重标 |

完整标定一次同时解出上面所有项；相机、夹爪安装或分辨率变过也要重做。标定板用 12×9 格、每格 30 mm 的棋盘格；用别的规格要改 `src/robot/rt.py` 里的 `PATTERN, SQUARE`（内角点数和格边长）。把 `config/calib.example.json` 复制成 `src/calib.json` 作为初值，标定板平放在臂前桌面，夹爪张开，臂在能看到板的高位，在 `src/` 下运行：

```bash
python3 -m calibration.plan calib.json 28 2     # 规划 28 个能看到板的俯视姿态，不动臂
python3 -m calibration.capture 0 cal2           # 逐个姿态静止后拍照，记录实测关节角
python3 -m calibration.solve calib.json $(ls cal2/*.jpg | sed 's/.jpg//')
python3 -m calibration.check <没参与标定的若干帧>  # 像素反投到桌面的误差
```

这台设备上的结果：重投影 0.52 px，像素反投到桌面 RMS 3.6 mm（最大约 1 cm）。关节零位偏差最大 0.06 rad，不修正时桌面定位误差会翻倍。OpenCV 自带的部分棋盘检测在这个广角相机上会跳行、沿对角线连格，所以 `rt.corners()` 自己做了鞍点检测加逐格生长。

依赖：Python 3.10+，`pip install -r requirements.txt`（NumPy、SciPy、OpenCV）。

## 通过技能使用

把本目录作为技能交给 Agent（SKILL.md 是入口）。三样准备好后，Agent 在 `src/` 下一步一步运行 `python3 -m sorting.sort RUN <步骤>`，每步之间看腕部图再决定下一步；`RUN` 是这一轮的目录，图片、拟合结果和带时间戳的日志 `events.jsonl` 都存在里面。Agent 不启动、不重启你的控制程序，也不调用会卸力的停止。

## 目录

| 路径 | 内容 |
| --- | --- |
| [SKILL.md](SKILL.md)、[SKILL.zh.md](SKILL.zh.md) | 给 Agent 的做法：流程、按摆法选抓法、实测数值 |
| [docs/](docs/) | 按情况查阅：白色桌面、控制与感知要点（含各项做法的依据） |
| `config/calib.example.json` | 作者这台 R1-7a 的标定结果：经验值和标定初值 |
| `src/` | 工具脚本，见下表 |

所有工具都在 `src/` 下以 `python3 -m <目录>.<脚本>` 运行。

| `src/` 里的目录 | 文件 | 作用 |
| --- | --- | --- |
| `robot/` | `rt.py` | 机器人接口（上表四个函数之三，换成你的控制）；带零位修正的 FK/IK（多初值 `ik_all`）、像素与桌面互换、三角定位、间隙检查、外环 `servo`、部分棋盘检测 |
| | `grab.py` | 取一帧腕部图（`frame`）；单独运行时拍一帧并记下关节角 |
| `ik/` | `kinematics.py`、`frames.py` | 从 URDF 建模：FK、IK、雅可比、重力项；world 系定义 |
| `calibration/` | `plan.py`、`capture.py`、`solve.py`、`check.py` | 相机标定：规划姿态、逐姿态拍照、解内参加手眼加关节零位加板平面、独立检验 |
| `perception/` | `pens.py` | 在腕部图里找平放的笔：端点世界坐标、哪端是大帽 |
| | `view_plan.py`、`box.py` | 选俯视盒子的姿态（不动臂），多视角拟合 4×6 槽格 |
| `sorting/` | `sort.py` | 一轮任务的分步入口：`look`、`pick`、`restage`、`putdown`、`place`、`verify`、`reseat`、`addpen` |
| | `task.py` | 各步的实现，以及 `holding`、`hangs_straight`、`find_cap` 等判断 |
| | `pick_direct.py` | 直接抓取：按笔的摆法自动选转角和抓取点，带关节限位余量检查 |
| | `reseat2.py`、`reseat_marked.py` | 把斜立的笔夹正、成笼晃动入槽；深色笔帽手工取点 |
| | `refit.sh` | 重拍三个俯视图并重新定盒，结果不合理时保留上一次的拟合 |

## 安全

- 这些脚本会驱动真机。先读仓库的 [docs/safety.zh.md](../../docs/safety.zh.md)，清空工作区，有人在场，硬件急停伸手可及。
- 会卸力的停止、断开或重启控制程序都会让手臂掉下来，需要时先扶住手臂。
- 运动脚本在前台一步一步跑，每步之间看图。路径检查用 `assert` 拦截。
- 够不着的笔不要硬够：贴着关节限位抓取时出过电机报错、控制器卸力、手臂落下。
