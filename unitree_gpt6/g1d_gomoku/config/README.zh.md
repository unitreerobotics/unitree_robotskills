# 配置与现场标定

[English](README.md) | 中文

`default.yaml` 只定义自主模式、19×19 棋盘和棋规，不包含现场经验。
复制 `hardware.example.yaml` 到工作空间的 `config/hardware.local.yaml`。

| 字段 | 配置要求 |
| --- | --- |
| `camera.host` | 必填：当前相机服务地址，无默认地址 |
| `robot.network_interface` | 必填：连接 G1 的网卡，无默认网卡 |
| 相机端口 | 模板为头部 55555、左腕 55556、右腕 55557，按服务配置调整 |
| DDS domain / 话题 | 模板为 0、`rt/lowcmd`、`rt/lowstate` |
| 控制频率 / 最大关节速度 | 模板为 100 Hz / 0.4 rad/s，属于工程控制设置 |
| 夹爪速度 / 范围 | 模板为 0.15 rad/s / 0–5.45 rad，属于控制设置和设备范围 |
| `runtime.directory` | `../runtime` |
| `runtime.scene_calibration` | `../calibration/scene.json`，首次由 Agent 测量并审阅 |
| `runtime.reset_state` | `../calibration/reset_right_state.json`，首次必须只读采集 |

相对路径以 YAML 所在目录为准，所有控制进程共用同一配置和 runtime 锁。

工程不附带归位参考值。确认当前设备处于合适的休息姿态后，由 Agent 按 README 的只读命令采集：
`joint_names` 与 `joint_positions` 为左臂七关节、右臂七关节；`waist_q` 为三个腰关节；
`poses.left/right` 为 pelvis 下的 `[x, y, z, roll, pitch, yaw]`；`grippers` 为左右夹爪反馈。
位置单位为米，角度为弧度，关节顺序必须与模型一致。不能填零代替反馈。

连续会话归位恢复参考中的右臂，保持当前左臂和任务启动腰角；
独立 `reset` 恢复双臂、参考腰角及左夹爪，右夹爪保持当前开度。
归位文件是执行前的参考，不代表抓放已经成功。

棋盘尺寸、抓放高度、臂形和夹爪经验首次为空，成功后按 [经验数据约定](../docs/experience.zh.md) 保存到工作空间。
硬件限位、控制频率和棋规不属于学习数据，不能由经验覆盖。

## 场景几何数据

`calibration/scene.json` 的结构见 [scene.schema.json](scene.schema.json)，没有数值默认值。
Agent 根据当前图像、模型、反馈和尺度依据建立该文件，并保存测量方法和图像依据。

| 字段 | 含义 |
| --- | --- |
| `frame` | `pelvis` |
| `tool_tip_offset_m` | TCP 局部坐标下到指尖检查点的三维偏移，单位米 |
| `high_route_min_tcp_z_m` | 已审阅高位通道允许的 TCP 最低 Z，单位米 |
| `high_route_min_tip_z_m` | 已审阅高位通道允许的指尖检查点最低 Z，单位米 |
| `rest_corridor.left/right.min_m/max_m` | 左右臂末端归位通道的三维范围，逐轴下界小于上界 |
| `measurements` | 其他实际测得的网格间距、棋盘范围、桌面与棋面高度等，字段注明单位 |
| `evidence` | 标定图像与反馈的工作空间相对路径 |

这些数据用于 FK 采样与通道核对，不是完整碰撞模型；仍须检查夹爪、持棋、手臂和周边障碍物。
预检只验证结构与有限数值，不宣称现场标定已经准确。
