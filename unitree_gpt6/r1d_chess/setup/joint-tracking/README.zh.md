# 关节跟随：Runtime 链路上的验证与力矩前馈微调

[English](README.md) | 中文

## 适用场景
- [arm-calibration](../arm-calibration/README.zh.md) 的结果安装后，在象棋实际使用的 Runtime 链路上验证跟随误差。
- 更换夹爪或负载、更换 URDF、修改 Runtime PD 增益之后。
- 象棋服务报告 `arm tracking error exceeded R1-D limit`，或者执行时 TCP 误差达到厘米级。
- 没有条件做 arm-calibration 时，也可以只用本流程从中性初值整定（重力比例、库仑摩擦、积分），精度低于前者。

## 背景
- 机器人运行时（Runtime）只给关节发位置 PD，`dq` 目标恒为 0；qtau 会原样写进电机的 `tau`。
- 不加前馈时，静态误差约等于 负载力矩 / kp；运动中还多一项阻尼滞后，约等于 kd·v / kp。
- 前馈公式和各项含义见 [docs/r1d_joint_tracking_feedforward.zh.md](../../docs/r1d_joint_tracking_feedforward.zh.md)，参数在 `config/arm_torque_ff.json`。
- PD 增益 kp/kd 由脚本从内置 Runtime 的机器人配置（`third_party/robot_runtime/configs`）读取，不要手抄数值。

## 前提
- `r1d-runtime.service` 在运行，没有其他控制会话。整定期间不能运行象棋服务或保持服务。
- `config/robot.json` 至少要有 `init_joint_rad`、`head_q_rad`、`waist_q_rad`、`lift_height_m`（chess-scene-parameters 第 1 步）。
- 双臂下方和周围没有障碍物，旁边有人可以随时接管。

## 安全与授权
- `bench.sh` 每跑一次，双臂会动约 32 秒：先抬高约 8 cm，然后 14 个关节双向摆动，最后回到起点。每次运行都要得到操作者明确授权。
- 本体保护保持开启：跟随误差超过 0.15 rad、超过 75 °C、反馈过期都会中止，结束前 qtau 斜坡降到 0。不要为了让测试通过而放宽这些阈值。
- 还没有 `scene` 时，规划无法检查桌面净空。这时必须先人工确认手臂下方是空的，再加 `PLAN_ARGS=--no-scene-ok`。

## 两条发送路径
- `direct`（路径名）：按 250 Hz 直连 Runtime，不需要完整几何。还没有场景参数时只能用它。
- `adapter`：与象棋服务完全相同的发送路径（50 Hz），需要完整几何。

## 步骤
命令都在包根下执行，`S=setup/joint-tracking/scripts`，`python` 为已安装依赖的环境（`bench.sh` 用 `R1D_CHESS_PYTHON`，未设置时用 PATH 上的 `python`）。

**A. 已完成 arm-calibration：验证**

1. 跑两次基线和两次完整前馈，估计重复运行之间的离散：
   ```bash
   export BENCH_DIR=runs/tracking_benchmark PLAN_ARGS=--no-scene-ok   # 已有 scene 时去掉 PLAN_ARGS
   $S/bench.sh direct off
   FF_CONFIG=config/arm_torque_ff.json $S/bench.sh direct full
   FF_CONFIG=config/arm_torque_ff.json $S/bench.sh direct full
   ```
2. 每次运行后看结果：
   - `python -B $S/joint_report.py <run>`：各关节的静态误差和 RMS。
   - `python -B $S/diagnose.py <run>`：误差与速度之比（e/v）远大于 kd/kp 的关节，是重力残差或摩擦问题；最大误差出现在低速末段的，是静摩擦问题。
3. 达到下面的验收标准就结束；个别关节不达标时按 B 的第 5 步做一两轮摩擦迭代学习。

**B. 从中性初值整定**

1. 从模板开始，不要沿用别的机器人调好的参数：
   ```bash
   mkdir -p runs/tracking_benchmark/ff
   cp setup/joint-tracking/templates/arm_torque_ff.initial.json runs/tracking_benchmark/ff/iter0.json
   ```
2. 跑基线：同一配置跑两次。
   ```bash
   export BENCH_DIR=runs/tracking_benchmark PLAN_ARGS=--no-scene-ok
   $S/bench.sh direct off
   $S/bench.sh direct off
   ```
3. 只加重力：
   ```bash
   FF_CONFIG=$BENCH_DIR/ff/iter0.json $S/bench.sh direct gravity
   ```
   然后用 `python -B $S/model_fit.py <run>` 看重力模型是否偏小（用误差已收敛的运行效果最好）。把最小二乘比例在 0.9–1.3 之间的关节写进 `gravity.scale`；超出这个范围的，先检查 URDF 或负载，不要直接硬改比例。
4. 辨识库仑摩擦：用只加重力的运行。
   ```bash
   python -B $S/friction_fit.py <gravity run> [<another gravity run>]
   ```
   把输出值乘以 0.8 写进 `friction.coulomb_nm`，并设 `error_epsilon_rad=0.01`。
5. 跑完整前馈并迭代：
   ```bash
   FF_CONFIG=$BENCH_DIR/ff/iter1.json $S/bench.sh direct full
   python -B $S/ilc_coulomb.py <run> iterN.json iterN+1.json --gain 0.5     # 一次摩擦迭代学习
   ```
6. 收敛判断：连续两版的改善小于重复运行的离散（约 ±1 mrad），就停止迭代。
7. **在 adapter 路径上确认一次**：等场景参数完成后，用 `$S/bench.sh adapter full` 跑一次，确认象棋服务实际使用的发送路径结果一样。
8. 写入正式配置：把最终版复制为 `config/arm_torque_ff.json`，在 `evidence` 里写明最终那次运行的目录。必须确认 `gravity`、`friction`、`integral`、`damping` 四个 `enabled` 都是 true：对比工具的 `--ff full` 会临时打开它们，象棋服务读的是文件本身。

## 验收标准
| 项 | 标准 |
|---|---|
| 各关节停留窗口的静态误差 | ≤ 0.012 rad |
| 单关节 RMS 误差 | ≤ 0.015 rad |
| 瞬时最大误差 | ≤ 0.05 rad（象棋服务的锁存故障阈值是 0.20） |
| TCP RMS 误差 | ≤ 8 mm |
| 温度 | ≤ 60 °C |
| `wire.mean_abs_wire_tau_minus_qtau_nm` | ≈ 0，证明力矩确实到达了电机 |

## 输出
- `config/arm_torque_ff.json`
- `runs/tracking_benchmark/runs/*`、`ff/iter*.json`

## 常见问题
- **连续运行时起点会漂移**：每次结束都带着残余误差停住。`bench.sh` 每次都会重新拍快照、重新规划，同一配置至少跑两次再比较。
- **积分窗口不能比初始下垂还小**：否则积分根本不启动。模板里取 0.25 rad。
- **前馈有效不代表模型正确**：重力在命令角处计算。整定阶段电机估计力矩远小于模型，通常是静摩擦撑住了，不能因此把比例调小。
- **TCP 误差依赖 TCP 定义**：`joint_report.py` 里的 TCP 误差用的是 `robot.json` 当前的 TCP，TCP 改了之后，前后结果不能直接比较。
- **对比工具用的 profile**（`BENCH_PROFILE`）只用于对比测试的净空检查，不能用于下棋。

> 参考（参考机器人）：无前馈时关节 RMS 误差 49–71 mrad、TCP 误差 27–40 mm；完整前馈后关节 RMS 约 10 mrad、TCP RMS 约 6 mm，改用 arm-calibration 拟合的动力学 URDF 后 TCP RMS 降到约 2.5 mm。
