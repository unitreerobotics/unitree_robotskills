# 底盘与升降反馈：象棋服务用到的 DDS topic

[English](r1d_twist_dds_topics.md) | 中文

象棋服务不控制底盘和升降，只读它们的反馈，用来确认升降高度在参考值上、底盘和立柱静止。本文说明这些反馈的含义和服务里的判据。无底盘机型不发布这些 topic。

## 用到的 topic

| topic | 类型 | 频率 | 用途 |
|---|---|---|---|
| `rt/state_vel` | `geometry_msgs::msg::dds_::Twist_` | 50 Hz | 底盘和立柱的实测速度，判断是否静止 |
| `rt/height_state` | `geometry_msgs::msg::dds_::Point32_` | 50 Hz | 立柱高度 |
| `rt/height_alarm` | `unitree_go::msg::dds_::Error_` | 50 Hz | 立柱告警，判断高度是否可信 |

这三条都来自电机实测反馈。`rt/cmd_vel`（底盘和立柱的速度命令，归一化 [-1, 1]）象棋服务不发。

## `rt/state_vel`：速度反馈

```text
Twist_
├── linear.x      实测前后线速度，归一化，相对当前档位的最大速度
├── linear.z      实测立柱升降速度，归一化，向上为正
├── angular.z     实测转向角速度，归一化，左正右负
└── 其余分量      恒 0
```

- **值是归一化的 [-1, 1]，不是 m/s 或 rad/s。**
- 发布前有死区：`linear.x`、`linear.z` 为 0.005，`angular.z` 为 0.02。角速度的死区更宽，因为差速运动学会放大左右轮速度差，静止时的读数尖峰可达约 0.012。
- 判断“真的在动”不要只看是否非零，用留有余量的阈值：`|linear| > 0.01`、`|angular.z| > 0.025`。

## `rt/height_state`：立柱高度

```text
Point32_
├── x / y     恒 0
└── z         立柱高度，单位 mm
```

- 原点在机械下限位上方 5 mm，向上为正；全行程 850 mm，软限位约 [0, 840] mm。
- **`z = 0` 无法区分“高度未知”和“确实在零点”**：立柱未回零或反馈无效时也发 0，必须配合 `rt/height_alarm` 判断。

## `rt/height_alarm`：立柱告警

```text
Error_
├── source    固定为 200（底盘模块）
└── state     告警位掩码
```

| bit | 名称 | 含义 |
|---|---|---|
| 0 | NotHomed | 立柱未回零，高度不可信 |
| 1 | Lower | 处于或低于下软限位 |
| 2 | Upper | 处于或高于上软限位 |
| 3 | Stall | 未回零状态下检测到持续堵转 |

- 高度可信的条件：`source == 200`、`(state & bit0) == 0`，且 `z` 为有限值。
- 这条告警只覆盖立柱，不是整机告警总线。电机故障看 `rt/lowstate` 的 `motor_state[i].motorstate`；底盘急停和自由推行不上 DDS。

## 在象棋服务中的使用

- **Runtime 的转换**：Runtime 把这些 topic 转成 `base_group` 状态。速度原样转发（归一化值）；`lift_height = height_state.z / 1000`（单位 m），只在高度可信（上面三个条件）时给出，否则为 `None`。
- **升降高度**：传输层（`src/r1d_chess/robot/r1d_transport.py` 的 `_check_reference`）要求 `lift_height` 不为 `None`，并且与 `config/robot.json` 的 `lift_height_m` 相差不超过 `lift_tolerance_m`（5 mm）。立柱未回零时 Runtime 给出 `None`，服务直接拒绝运行，所以不需要另外检查告警位。
- **静止判断**：`linear.x`、`linear.z` 超过 0.01 或 `angular.z` 超过 0.025（归一化值）就认为底盘或立柱在动，服务拒绝接管。
- 升降高度和静止都只在接管入口（`enable_control`、`handover`）检查，会话中不再检查；下棋过程中不要移动底盘或升降。
- 象棋服务不选底盘和升降控制组，程序不对升降做闭环保持。升降不在参考高度时，用 `bash r1d goinit` 先把升降带到 `lift_height_m`（见 [操作手册](operation_manual.zh.md)）。

## 速查

| 概念 | 数值 |
|---|---|
| `rt/state_vel` 线速度死区 / 角速度死区 | 0.005 / 0.02 |
| 静止判据（线速度 / 角速度） | 0.01 / 0.025 |
| 立柱全行程 / 软限位 | 850 mm / [0, ~840] mm |
| 立柱最大升降速度 | 0.08 m/s |
| `rt/height_alarm` 源标识 | 200 |
| 反馈频率 | 50 Hz |
