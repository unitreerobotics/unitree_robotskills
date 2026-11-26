# R1-D 三明治服务 API

[English](API.md) | 中文

另见：[手册](PLAYBOOK.zh.md)、[经验](EXPERIENCE.zh.md)。

本机 HTTP 服务负责 DDS、IK、安全检查、相机和执行。Agent 通过 `bash r1d eef` 或
`src/service/client.py` 里的 `R1DClient` 与它通信。客户端只使用 Python 标准库，并且对本机的机器人连接禁用
桌面 HTTP 代理。

## 目录

1. [进程模型](#进程模型)
2. [启动服务](#启动服务)
3. [坐标系与姿态](#坐标系与姿态)
4. [接口](#接口)
5. [运动结果与拒绝代码](#运动结果与拒绝代码)
6. [命令行对应关系](#命令行对应关系)
7. [安全边界](#安全边界)

## 进程模型

```text
agent
  -> bash r1d eef / src/service/client.py
  -> 127.0.0.1:8765
  -> src/service/server.py
       -> src/ik/ik_path.py                 IK and checks
       -> src/robot/robot.py                state and motion routing
       -> src/robot/grip_exec.py            execution and gripper frames
       -> src/perception/cameras.py         head and wrist frames
       -> src/perception/image_client.py    reads the robot's image server (ZMQ, read-only)
       -> src/perception/projection.py      image pixel to base-frame point
```

服务是唯一向机器人发送命令的进程。配置从 `assets/config/` 加载。运行时的日志、
图像和录像放在 `runs/` 下。

## 启动服务

在 `SKILL.md` 所在目录执行：

```bash
bash r1d serve --sim --fixed-lift 0.522
bash r1d serve --shadow --fixed-lift 0.522
bash r1d serve --real --fixed-lift 0.522
```

| 模式 | 读取机器人状态 | 下发运动 | 用途 |
|---|---|---|---|
| `--sim` | 否 | 否 | 用玩具场景做离线开发 |
| `--shadow` | 是 | 否 | 真实反馈、IK 和执行层检查；`executed=false` |
| `--real` | 是 | 是 | 先做 shadow 检查，随后真实执行并确认 |

默认地址是 `http://127.0.0.1:8765`。可以用环境变量或包内 `.env` 里的 `SERVER_URL` 覆盖。
只启动用户授权的模式。real 和 shadow 运行需要明确授权，每一次运动都需要新的明确授权。

`--fixed-lift 0.522` 防止任务期间升降发生变化。直出末端命令始终传 `auto_lift=false`。
参考运行用的就是这个固定值。在 `--sim` 下，仿真机器人从固定升降高度开始，所以可以在任务实际
使用的高度上预演路线；不带 `--fixed-lift` 时，它从 `assets/config/scene.json` 里的
`lift_m`（`0.665`）开始。

`--temp-guard` 让服务在 `70 C` 时拒绝开始运动、在 `80 C` 时触发停止。除非主动要求，否则它是关闭的。

real 模式下，服务在连接之前要求操作员输入一次 `s`，默认的 `--confirm all` 还会在每次运动前在服务
终端询问确认。它需要交互式终端。任何更弱的确认模式都必须是操作员有意做出的决定，Agent 不能自行假定。

连真机时，服务用自带的执行器通过 `unitree_sdk2py` 驱动手臂：250 Hz 指令循环、跟踪看门狗和软停止都
运行在服务进程里。

## 坐标系与姿态

- 坐标系：R1-D URDF 基座，单位为米，x 向前，y 向左，z 向上。TCP 是 Dex1 两个指垫的中点。
- 在标准姿态下，工具 x 轴沿手指方向，工具 y 轴是开合轴，工具 z 轴向上。
- `orient`、`rpy_deg`、`quat_wxyz` 三者最多给一个。都不给时，保持当前姿态。
- `orient="forward"` 让手指指向前方。
- `orient="down"` 让手指指向下方；`yaw_deg` 旋转开合轴。
- `orient="flat"` 让手指保持向前、开合轴竖直，适合搬运平放的面包片。
- `rpy_deg=[r,p,y]` 在基座坐标系下使用 `R = Rz(y) * Ry(p) * Rx(r)`。
- 直出末端路点通常同时给出 `pos` 和 `rpy_deg`。

## 接口

| 方法与路径 | 请求体 | 主要响应 |
|---|---|---|
| `GET /health` | 无 | 模式、忙碌/停止状态、相机、像素投影状态、确认模式、固定升降、版本和日志目录 |
| `GET /state` | 无 | 关节、TCP、夹爪开度、头部、升降、腰、温度和机器状态 |
| `GET /image/<cam>.jpg` | 无 | `head_left`、`head_right`、`left_wrist` 或 `right_wrist` 的 JPEG |
| `GET /poses` | 无 | 示教位姿和当前场景模型 |
| `GET /sim/scene` | 无 | 仿真物体、附着关系和完成标志 |
| `POST /move` | `{side, pos?, orient?/rpy_deg?/quat_wxyz?, yaw_deg?, speed?, linear?, auto_lift?, check_only?}` | 运动或检查结果 |
| `POST /move_both` | `{left:{pos, rpy_deg}, right:{pos, rpy_deg}, speed?, linear?, check_only?}` | 两个 TCP 同时运动，作为一条路径检查：默认走直线，`linear:false` 时走一条关节空间路径 |
| `POST /gripper` | `{side, opening}` 或 `{side, action:"open"\|"close"\|"grasp"}` | 开度、夹持标志和执行结果 |
| `POST /home` | `{side:"left"\|"right"\|"both", speed?}` | 关节空间 home 的结果 |
| `POST /move_joints` | `{side, q:[7], speed?}` | 带安全检查的关节空间回放 |
| `POST /waist` | `{target}` | 腰部运动；手臂保持各自的关节角 |
| `POST /head` | `{pitch?, yaw?}` | 头部运动 |
| `POST /lift` | `{height_m}` | 升降运动，固定升降任务期间禁用 |
| `POST /stop` | `{}` 或 `{clear:true}` | 锁存或清除软件停止 |
| `POST /scene` | `{table_x_min?, table_z?, obstacles?}` | 更新运行时场景模型 |
| `POST /project` | `{cam, pixel(s), z?, method?, window?}` | 用标定过的相机模型对像素做反投影 |
| `POST /resync` | 无 | shadow 模式：把孪生体拉回到实测位姿 |
| `POST /sim/reset` | `{randomize?, seed?}` | 仿真重置 |

没有新鲜帧可用时，`/image` 请求会失败。`/project` 只做几何换算：它需要真实相机的标定，
使用仿真相机时不可用。服务里没有物体检测器；Agent 直接查看 JPEG，用 `project` 测量，并用
`look --mark` 核对估计值。

## 运动结果与拒绝代码

每个运动结果至少包含：

```json
{
  "ok": false,
  "executed": false,
  "reason": "human-readable explanation",
  "code": "unreachable",
  "stopped": false
}
```

`ok=false` 不会抛出异常。要检查它，并停止依赖它的后续序列。常见代码有 `unreachable`、`too_far`、
`collision`、`joint_limit`、`table`、`lift_range`、`invalid_argument`、`operator_declined`、
`watchdog`、`stopped`、`not_ready`、`executor_refused` 和 `refused`。

成功时有用的字段包括 `final_tcp`、`planned_tcp`、`pos_err_mm`、`rot_err_deg`、`min_clearance_m`、
`min_table_gap_m`、`duration_s`，夹爪命令还有 `opening` 和 `holding`。

`linear:true` 路点被拒绝时，必须先用 `linear:false` 退回上一个悬停点，再沿已经走通的路线靠近。
不要重复同一段被拒绝的路径。

## 命令行对应关系

```bash
bash r1d eef new
bash r1d eef prepare
bash r1d eef state
bash r1d eef look --cams head_left,left_wrist,right_wrist --raw
bash r1d eef look --cams head_left --z 0.61 --mark "0.34,-0.06,0.61"
bash r1d eef look --cams head_left --raw --crop 180,150,360,290 --scale 4
bash r1d eef project --cams head_left --z 0.61 "261,226;297,224"
bash r1d eval '<candidate-json>'
bash r1d eval --brief '<candidate-json>'
bash r1d eef check '<waypoint-json>'
bash r1d eef exec '<waypoint-json>'
bash r1d eef both-check '<two-arm-json>'
bash r1d eef both '<two-arm-json>'
bash r1d eef grip right grasp
bash r1d eef home right
bash r1d eef keyframe final
bash r1d eef rec-start
bash r1d eef rec-stop
bash r1d metrics --run-dir runs/<run>
```

`check` 只从当前位姿发送 `check_only=true` 的运动请求。`exec` 在第一个被拒绝的路点停下并返回非零
状态；如果夹爪开度达不到某个路点的 `min_opening`，它也会在该路点之前停下。`eval` 使用同一个
planner，但需要可读的状态，所以本机服务必须在运行；它从不发送运动命令。`project` 调用
`POST /project`。`look` 把各视角保存到这次运行的 `looks/` 下，并另存为 `runs/latest/<cam>.jpg`；
`exec`、`both`、`grip`、`waist` 和 `home` 在让机器人运动之前会删掉 `runs/latest/` 里的文件。

管道开头使用 `set -o pipefail`，有依赖关系的阶段之间用 `&&` 连接。否则被拒绝的 `exec ... | tail`
可能被误当成成功，后面的阶段可能继续执行。

## 安全边界

planner 强制检查带余量的关节限位、自碰撞间隙、已登记的障碍盒、桌面上方间隙和单次运动位移。执行层提供
shadow 检查、看门狗和停止状态。

它不检测盘子、食物、松散物体或人。它不验证夹持力，也不验证物体是否附着。从上方接近，检查图像，
确认夹爪开度，场景或结果不确定时就停下。软件停止不是硬件急停。
