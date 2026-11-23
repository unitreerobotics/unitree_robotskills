# R1-D 场景参数：含义与确定方法

[English](r1d_scene_parameters.md) | 中文

坐标系 `waist_yaw_link`，单位米/弧度。这些参数写在 `config/robot.json`（profile）和 `config/board_geometry.json`。棋盘移动、头部参考姿态变化或升降高度变化后都要重测。

文中的数值是一台参考机器人在其场地上的模型估计（`model_estimate`），用来说明量级和算法；新机器人、新棋盘要按同样的方法得到自己的数值。

## 夹爪与 TCP

依据：Dex1 图纸和 URDF 网格。同型号夹爪可以沿用。

| 参数 | 参考值 | 依据 |
|---|---|---|
| `tcp_xyz_rpy` | 左 (0.0485, −0.003, 0.02)，右 (0.0485, 0.003, 0.02) | `dex1_base_joint` 原点，即夹爪法兰 |
| `finger_end` | 0.1432 | 图纸法兰到指尖 143.2 mm |
| `half_width` | 0.0585 | 图纸全开外宽 117 mm |
| `half_thickness` | 0.005 | 手指从根部 32 mm 收窄到尖端。**只在 pitch ≥0.2 rad 时成立**：这时最低点是指尖，手指根部更高 |

## 夹持点与 spec 位姿（固定规则）

- spec 的 `xy_m`、`z_m` 是 TCP（法兰），手指沿 TCP +x 伸出。
- 夹持点离法兰 `GRIP_DEPTH_M = 0.137`（夹爪前部）：指尖 143.2 mm，夹持中心在指尖内约 6 mm。
- 夹持点 = TCP + 0.137·(cos p·cos y, cos p·sin y, −sin p)，与 roll 无关。默认 pitch 0.8、yaw −π/2 时偏移 (0, −95.4, −98.3) mm。
- 抓取：指尖在板面上方 3 mm。
- 释放：所有走棋的棋盘 TCP z 固定 0.150 m，与 pitch 无关（`BOARD_RELEASE_TCP_Z_M`）。托盘 TCP z = 指尖高度 + 0.1432·sin p。
- pitch 0.8 时 `gate_z_m` 至少 0.175（运输净空 40 mm）。
- 工具：`bash r1d pose pose|check`（`src/r1d_chess/chess/grip_pose.py`）。`round build` 要求夹持点距所写格子交叉点 ≤12 mm（小于半格），托盘 ≤30 mm；把 TCP 直接放在交叉点上会偏 123 mm，会被拒绝。

## 棋盘（`config/board_geometry.json`）

需要的量：板厚、列距和行距、棋子直径和厚度（尺量），板面高度 `surface_z`、棋盘中心和转角、90 个交叉点（头部相机测量）。

- 测量工具：`bash r1d boardcal`（配准印刷图案并精修）、`bash r1d boardpoints`、`python scripts/head_board_measure.py`（含板面高度），见 [board_position](../board_position/README.zh.md)。前提是已有头部相机标定。
- `surface_z` 的 `uncertainty` 要覆盖不同测法之间的差异。
- 网格拟合是周期性的，没有先验时可能错一格；重测时加 `--prior config/board_geometry.json`。

> 参考：参考棋盘板厚 20 mm，格距 35.6（列）× 33.7（行）mm，棋子 ø29.7×9.9 mm；板面 z = +11.8 mm（立体测量），另一种测法得 +9 mm，相差 2.8 mm，所以取 `surface_z=0.012`、`uncertainty=0.004`。

## 释放高度

profile 的 `board_release_z_m`、`tray_release_z_m` 在 `release_pitch_rad`（0.35）下给出，其他 pitch 按指尖高度换算：TCP z = 指尖 z + 0.1432·sin(pitch)。

| 参数 | 参考值 | 指尖高度 |
|---|---|---|
| `board_release_z_m` | 0.0741 | z=25 mm，距板面约 13 mm |
| `tray_release_z_m` | 0.1091 | z=60 mm，距桌面约 71 mm |

托盘释放时指尖要高过杯沿，首次使用前看图确认。

> 参考：判断某个 pitch 是否可达，要按真实下降方式（高位 → 逐 5 mm 下降、链式种子）检查。单点 IK 会因为种子过远而误判不可达。参考机器人上左手在 pitch 0.35–1.0 都可达，腕 pitch 余量 ≥0.22 rad。

## 头部锁定姿态

头部锁定在 `config/robot.json` 的 `head_q_rad`（参考值 pitch 0.62、yaw −0.0036，接近最低头）。棋盘因此位于画面中心、畸变更小。

- 改头部参考位后要重测棋盘几何，并确认头部相机标定在该头姿下仍然可用。
- 头 yaw 有回差（参考值约 ±15 mrad）。测量时最后一次 yaw 运动的方向要固定，以后每次从同一方向回到参考位。

> 参考：参考机器人在 pitch 0.344 和 0.624 两个头姿下分别测棋盘，板面 z 差 1.3 mm、中心差 0.1/0.6 mm、转角差 0.7 mrad，说明标定在低头姿态下可用。低头后远端格子约 11.5×17.7 px，近端约 26×27.5 px。

## 桌面净空判据

净空 = 夹爪包络最低角 − 板面 − `uncertainty`。包络已经包含手指半厚。

| 检查 | 下限 | 物理含义 |
|---|---|---|
| 规划（计划位姿：路径、开合爪、撤销动作） | `minimum_clearance` = 0 | 包络不低于“板面 + 不确定度” |
| 运行（实测位姿：插值中反馈、开合爪前、释放检查点和低位 XY 修正） | −`uncertainty` | 包络不低于板面估计本身 |

运行下限比规划下限宽一个不确定度，是因为手臂静态跟随误差有几毫米，下垂方向正好向下；两者用同一个下限时，下降到释放高度容易误报故障。

抓取低位仍然低于规划下限：指尖要低于棋子顶面，所以每轮用 `ignore_source_tool_envelope`，由视觉门禁把关。

## 两级支撑面：棋盘与桌面

`scene` 有两级支撑面，因为 init 位姿停在棋盘两侧，下方是比板面低一个板厚的桌面：

- 棋盘占地范围内用板面 `surface_z`。范围是交叉点范围外扩 30 mm 边框：中心 `board_centre_xy`、朝向 `board_yaw`、半尺寸 `board_half_extent`（沿行、沿列）。
- 范围外用桌面 `table_z = surface_z − 板厚`。
- 夹爪包络在棋盘坐标系下的外接矩形只要和棋盘范围重叠，就整体按板面算（偏保守）。
- 这些字段要么都给，要么都不给：都不给时按单平面处理，只给一部分会被拒绝。单平面模型下，init 处的手会被算成低于板面而被预检拒绝。

托盘和桌上其他物体没有建模。

## 已知限制

- 抓取低位不在这组参数的净空要求内，见上文“桌面净空判据”。
- 预检不含手臂与棋子、托盘的碰撞。
- 模型估计的精度取决于头部相机标定；标定残差、头 pitch 零偏都会直接进入棋盘几何。
