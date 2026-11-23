# R1-D 象棋：机器人服务建立与运动下发链路

[English](robot_service_architecture.md) | 中文

本文说明象棋服务如何建立、运动如何下发到 R1-D。

## 一、三层服务结构

```
bash r1d client ...           (ZMQ REQ, ipc:///tmp/astra_r1d_chess.sock)
        │  JSON 命令
        ▼
bash r1d serve  →  src/r1d_chess/robot/serve.py  R1DService (ZMQ REP)
        │  继承 src/r1d_chess/robot/internal_service.py 的 RobotService
        │  IK / 插值 / 预检 / token / 检查点
        ▼
src/r1d_chess/robot/r1d_transport.py  InternalDDS
        │  订阅 rt/lowstate、rt/lowcmd（只读）
        │  后台线程每 20ms 经 Runtime IPC 发六组关节目标
        ▼
r1d-runtime.service  (bash r1d runtime serve，systemd 用户服务)
        │  RuntimeServer + UnitreeSDK2Driver
        ▼
      DDS LowCmd → R1-D 机器人
```

### 1. 底层 Runtime（`src/r1d_chess/robot/r1d_runtime.py`）

- `load_experiment()` 读取 `config/runtime.json`，把 `runtime_root` 指向的内置 Runtime（`third_party/robot_runtime`）的 `src` 加入 `sys.path`，再解析 experiment yaml（R1-A7-D mobile_wbc），并改写网卡和相机（TeleImager）配置。
- `serve()` 启动 `RuntimeServer`（子类 `R1DServer` 会把反馈里的轻微越限钳回限位内）。它直接占用 DDS，对外开放三个 IPC 端点：`runtime_state`、`runtime_cmd`、`runtime_request`。实际常驻运行的是 systemd 用户服务 `r1d-runtime.service`。
- `R1DRuntimeClient` 是 IPC 客户端，导入或连接本身不会接管控制：
  - `connect()`：订阅 Runtime 状态。
  - `begin()`：发 `start_control` 申请控制会话，申请的组件是 `left_arm, right_arm, waist, head, left_ee, right_ee`，默认不含底盘和升降。
  - `send_joint_targets()`：每次必须带齐上面六组的 qpos，否则报错；调用方必须在 cmd 超时前持续发送。
  - `end()`：发 `stop_control` 结束会话。

### 2. 中间层 InternalDDS（`src/r1d_chess/robot/r1d_transport.py`）

- 电机索引：左臂 15–21，右臂 22–28，腰 yaw 13，头 pitch/yaw 29/30，左/右夹爪 31/33。
- 只订阅原始 DDS（`rt/lowstate` 读反馈，`rt/lowcmd` 用来检测是否还有其他发布者），**不创建第二个 LowCmd publisher**。
- 所有 IPC socket 都在后台线程 `_run()` 里创建、使用和关闭；外部通过 `_rpc()` 排队提交请求（超时 20s）。
- **`enable` → `_begin()`**，按顺序做这些事：
  1. 检查 Runtime 空闲（无 `control_active`/`runtime_error`/`reset_pose_active`）、关节静止（|dq| ≤ 0.1）。
  2. `_check_reference()`：无热保护；头在 `head_q_rad` 参考位 0.35 rad 以内（接管后斜坡转过去），腰在 `waist_q_rad` 附近（≤ `head_waist_limit_rad`）；升降高度与 `lift_height_m` 相差在 ±5mm 内；底盘和升降速度（`rt/state_vel` 归一化值）线速度 ≤0.01、角速度 ≤0.025，见 [r1d_twist_dds_topics.zh.md](r1d_twist_dds_topics.zh.md)。
  3. 用 `MotionServiceSwitcher` 释放 AI 模式（`release_control_service`）。
  4. 等 0.3s，确认已经没有别的 LowCmd 在发。
  5. 再检查一次参考位姿，然后调用 `client.begin()` 建立会话。
  6. 把当前关节位置设为初始目标；若会话没建成，恢复原 motion 服务。
- **`_publish_once()`**，每 20ms 执行一次：
  1. 做健康检查：lowstate 是否过期（> `feedback_timeout_s`）、Runtime 会话是否仍有效、Runtime 错误和热保护、电机状态与温度（> `max_motor_temperature_c`）、手臂跟踪误差是否超过 `arm_tracking_limit_rad`（0.20rad；超限时保存 `tracking_fault_snapshot`）。
  2. 让 `arm_sent` 向 `arm_target` 逼近，手臂限速 0.3 rad/s，夹爪限速 0.8 rad/s（夹爪目标再钳到当前反馈 ±0.18 内）。
  3. 按 `config/arm_torque_ff.json` 计算臂部前馈 qtau（重力 + 摩擦 + 阻尼补偿 + 积分，限幅限速）；见 [r1d_joint_tracking_feedforward.zh.md](r1d_joint_tracking_feedforward.zh.md)。
  4. 头腰从会话开始时的实测位置以 0.2 rad/s 斜坡过渡到配置里的初始参考 `head_q_rad/waist_q_rad`，之后保持。
  5. 调用 `client.send_joint_targets(targets, qtau)` 发出六组目标和臂部 qtau。
  - 任何一项检查失败就锁存 `fault`，先保持实测位置、把 qtau 以 30 Nm/s 斜坡降为 0，再调用 `client.end()` 结束会话。**不会自动恢复，也不会自动回撤或恢复 AI**。
- 上层只能改目标值，不直接发命令：
  - `command_arm(q, tau)`：只改 `arm_target`；`q` 需通过 URDF 限位检查，`tau` 必须全为 0。前馈只由传输层按配置计算，事务引擎不参与。
  - `command_gripper(side, q)`：只改夹爪目标，q ∈ [0, 5.4]。
  - `hold()`：把手臂目标设为当前反馈位置。
  - `release()`：只关掉事务执行（`enabled=False`），会话照常维持，继续保持头腰与双臂最终位置。
  - `release_waist()`：执行关闭后才可调用，真正结束整个六组会话。
- `SimulatedDDS` 是离线替身，不导入 Unitree、不连 socket，供 `--simulate` 和测试使用。

### 3. 象棋服务（`src/r1d_chess/robot/serve.py`）

- 入口：`bash r1d serve [--arm right|left] [--allow-motion] [--simulate] [--endpoint ...] [--log-dir ...]`。
- 它用 flock（`/tmp/astra_r1d_chess_<hash>.lock`）保证只有一个实例，再绑定 ZMQ REP 端点 `ipc:///tmp/astra_r1d_chess.sock`（只允许 ipc 或 127.0.0.1）。
- **默认只读**：不加 `--allow-motion` 时，`enable()` 会直接拒绝。
- `R1DService` 继承通用 `RobotService`，并在 `dispatch()` 外面加了几道检查。对规划类命令（`prepare_*`）和运动类命令（`enable_control/set_gripper/execute_*`），要求：
  - 标定校验通过（`require_calibration`）；
  - 请求带上 `expected_active_arm`，且和当前执行臂一致；
  - profile 自启动以来没有改过（比对 SHA256），否则要求重启空闲服务并重新出计划和 token。
- 通用回放命令 `execute_plan / execute_cached_plan / restore_cached_transaction` 被禁用，只允许执行刚预检过的事务。
- `set_scene` 必须与 profile 中的 scene 一致；会话仍在时拒绝 `shutdown`。
- `set_active_arm` 只能在执行关闭时切换，切换后清空缓存计划。用左手执行时，`arm_roles.py` 的 `LeftActorDDS/LeftActorMotion` 会交换关节向量和夹爪索引（逻辑 right = 物理执行手），**笛卡尔坐标不镜像**。
- 额外命令 `prepare_init/execute_init`：两臂距 init TCP ≤ 0.10m 时原地保持；超限的手臂按“竖直上升 → 高位平移/转姿 → 竖直下降”生成并预检路径。
- SIGINT/SIGTERM：会话仍在时只请求取消并 `hold()`，不结束会话。

## 二、一次运动怎么下发到机器人

1. **准备阶段**：`client prepare_transaction`（吃子用 `prepare_sequence`）提交 TCP 航点，每个航点是 `x/y/z/roll/pitch/yaw`（`waist_yaw_link` 坐标系）加 `gripper: open/close/hold`。
   - `prepare_cached()` 调用 `_compile_cached()`，由 `MotionHarness.prepare` 用 unitree_ros R1-D URDF（`config/runtime.json` 的 `model_urdf`）做 IK，生成关节空间样本 `(q, duration)`。IK 在 `src/r1d_chess/robot/r1d_urdf.py` 的 `Arm.solve`：先解析雅可比阻尼最小二乘（`solve_dls`），不收敛才回退 scipy `least_squares`。抓取重试按 3 个抓取高度 × 3 次抓取重复预检整条成功/失败路径（约 9 遍），这是有意保留的安全预检，不要为提速删除。
   - 同时检查 `BoardGuard` 桌面/工具包络，并为每一段生成反向回撤路径和夹爪撤销动作。
   - 成功（success）、失败（failure）两个分支都会完整预检。最后返回一个一次性 token。
2. **启用控制**：服务有两层状态。
   - **会话**（`session_active`）：第一次 `client enable_control`（有保持单元时用 `handover --arm X --execute`）走上面的 `_begin()` 建立会话，之后保持头腰参考和双臂，直到走完回 init 后自动卸力、`retract` 完成或 `release_waist`。
   - **执行**（`enabled`，状态里的 `control_enabled`）：只表示允许执行一次走子。`release_control` 或 execute 结束会把它关掉，会话保留。会话已存在时，`enable_control` 只重新打开执行。
   - 规划、`round build`、`set_active_arm`、`prepare_init` 要求执行关闭。所以流程是：接管 → `release_control` → 规划/预检 → `enable_control` → execute → 自动关闭执行 → 回 init 后卸力并结束会话 → 下一轮重新接管。
3. **执行**：`client execute_transaction --request {token}`，进入 `_run_cached_steps()`（`src/r1d_chess/robot/internal_service.py:1662`）。
   - 每个样本内部用 smoothstep 插值：`command = prev + (target - prev) * α²(3 - 2α)`，每约 10ms 调一次 `dds.command_arm(command, tau)`；不动的手臂保持原目标。
   - 每次下发前都会比较反馈 FK 和期望 FK：低位允许 9mm 偏差，高位允许 18mm，超出即中止。
   - 抓取失败后竖直上抬时，可能在运行时重建路径：先竖直离开，确认达到运输净空后再横向连接。
   - 夹爪动作通过 `command_gripper` 下发。
4. **持续发送**：`InternalDDS` 后台线程以 50Hz 读取最新的 `arm_target`，限速后经 `send_joint_targets` 发给 Runtime，再由 Runtime 写 DDS LowCmd。
5. **视觉检查点**：执行过程中会在检查点暂停，等另一个终端用 `serial next/submit` 提交看图后的评价，再决定继续、修正（`adjust` / `adjust_pitch` / `retry_high`）还是失败回撤。
6. **收尾**：正常完成并回到 init 后，服务关闭执行、结束六组会话并给两臂卸力（结果里的 `arm_unload`）；停在半路或有故障的结果不卸力，会话保持。

## 三、关键文件索引

| 文件 | 作用 |
|---|---|
| `r1d` | 子命令启动器，分发到各脚本 |
| `src/r1d_chess/robot/serve.py` | R1-D 象棋 REP 服务 `R1DService` |
| `src/r1d_chess/robot/r1d_transport.py` | `InternalDDS`（Runtime IPC 唯一所有者）/ `SimulatedDDS` |
| `src/r1d_chess/robot/r1d_runtime.py` | Runtime 实验加载、`serve()`、`R1DRuntimeClient` |
| `src/r1d_chess/robot/handover.py` | 保持单元 → 象棋控制的交接 |
| `src/r1d_chess/robot/r1d_config.py` | profile 加载、标定校验、头腰参考 |
| `src/r1d_chess/robot/internal_service.py` | 通用事务引擎 `RobotService`（预检、token、执行、检查点） |
| `src/r1d_chess/robot/motion_harness.py` | 笛卡尔插值与 IK（不发命令） |
| `src/r1d_chess/robot/arm_roles.py` | 左手执行时的角色/关节映射 |
| `src/r1d_chess/robot/robot_client.py` | REQ 客户端 CLI |
| `config/runtime.json` | 内置 Runtime 的位置、experiment、网卡、相机主机 |
| `third_party/robot_runtime/` | 内置的最小机器人运行时（服务端、IPC、SDK2 驱动、R1-A7-D 配置） |
| `config/robot.json` | 几何 profile、init、头腰/升降参考、安全阈值 |

## 四、几何来源

`config/robot.json` 的场景和释放高度可以是实测标定，也可以是模型估计（见 [r1d_scene_parameters.zh.md](r1d_scene_parameters.zh.md)），两者都能通过预检和运动启用门禁。通过门禁只说明数值完整，不代表抓放已验收。
