# 关节精度标定：双臂动力学与腰头保持参数

[English](README.md) | 中文

## 标定什么

| 对象 | 辨识的量 | 结果文件 | 象棋服务怎么用 |
|---|---|---|---|
| 双臂 14 个关节 | 连杆质量和质心、库仑摩擦、随负载变化的摩擦、粘性摩擦 | `r1_d/assets/r1d/r1d_calibrated.urdf` | 力矩前馈的重力项和摩擦项（`config/arm_torque_ff.json`） |
| 腰 yaw、头 pitch、头 yaw | 干摩擦、头部重力保持力矩、电机增益 | `r1_d/assets/r1d/hold_joints.json` | 腰头保持前馈（`config/hold_joints.json`）和 Runtime 的腰头增益 |

这里标的是“关节到位精度”：让电机在指令角度上停得准。它不改运动学（FK/IK 仍用官方 `r1d.urdf`），也不标零位。

## 工具

`r1_d/` 是一套独立的直连工具，不经过机器人运行时（Runtime，`third_party/robot_runtime`）和象棋服务，自己通过 DDS 发 LowCmd：

```text
r1_d/
├── config.json        机器人 IP、网卡、传输方式（dds 直连或 ssh 转发）
├── sdk/               连接、状态、协议编码、控制会话、动力学、自碰撞检查
├── tools/             控制台、关节测试、高度测试、腰头标定、卸力
├── calibration/       双臂探索（explore）、拟合（fit）、留出评估（heldout、compare）
├── assets/r1d/        官方 r1d.urdf、拟合结果 r1d_calibrated.urdf、hold_joints.json、网格
├── tests/             离线测试
└── third_party/       自带的 unitree_sdk2_python
```

`assets/r1d/` 里的 `r1d_calibrated.urdf` 和 `hold_joints.json` 是参考机器人的结果，标定完成后会被本机的结果覆盖。

## 前提

- **停掉 Runtime 和象棋服务**（`systemctl --user stop r1d-runtime.service`，象棋服务 Ctrl-C）。两者同时发命令会互相打架。
- 按本机填写 `r1_d/config.json` 的 `ip`、`interface`、`transport`（`interface` 随包为空，必须填）。有线直连用 `"transport": "dds"`；走 Wi-Fi 时用 `ssh` 转发，需要先配好免密登录。
- 厂商运控服务需要释放时，命令加 `--release-mode`；默认只查询不切换。
- **把桌子和棋盘移开**：探索会让双臂走遍整个关节范围。
- 升降高度不低于 0.40 m。
- 依赖见 `r1_d/requirements.txt`（numpy、scipy、cyclonedds），与象棋环境兼容，可以用同一个 Python。

## 安全与授权

- 所有带运动的命令都要操作者明确授权，旁边有人可以随时断电。
- 探索路径离线规划时做过自碰撞检查（手臂与躯干、头、立柱、底盘），但不知道外部障碍物。
- 断连会自动重连并从断点继续；跟随误差超过 `--guard` 会停止当前路径点。
- 结束后手臂停在最后的位置并保持，不会自动卸力。卸力用 `python tools/unload_arms.py`（先看只读预检，再加 `--execute`），卸力前托住手臂或清出下落空间。

## 步骤

以下命令在 `setup/arm-calibration/r1_d/` 下运行，数据放 `<包根>/runs/arm_calibration/`（下文记作 `$O`）。

**0. 只读检查和离线测试**
```bash
python -B -m unittest discover -s tests -t ..     # 离线，不连机器人
python tools/run.py --check                        # 只读：URDF、状态话题、运控模式
```

**1. 双臂探索**（双臂大范围运动）

每一轮先离线规划无碰撞的路径点，再上机执行并记录每个关节的角度、速度和力矩：
```bash
python calibration/explore.py plan $O/plan1.json --count 160 --seed 1 --height <当前升降高度 m> \
    --head-pitch 0 --gravity-urdf assets/r1d/r1d.urdf
python calibration/explore.py run  $O/plan1.json $O/r1 --gravity-urdf assets/r1d/r1d.urdf --hours 1
```
- `--height`、`--waist`、`--head-pitch` 要与执行时的实际姿态一致，否则执行会拒绝。头抬平（pitch 0）时肩关节的可用范围最大。
- 第一轮用官方 `r1d.urdf` 的重力做前馈；之后各轮用上一轮拟合出的 URDF（`--gravity-urdf`）。
- 至少做三轮，每轮换 `--seed`：前两轮用来拟合，最后一轮留出不参与拟合。

**2. 拟合**
```bash
python calibration/fit.py fit $O/r1 $O/r2 --output $O/fit_a.urdf --record $O/fit_a.json
```
模型：慢速运动时 `tau = g + (c + mu·|g|)·sign(dq) + b·dq`，`g` 由连杆质量和质心决定。按 1/kp 加权，等价于最小化错误前馈造成的位置误差；数据看不到的参数保持官方 URDF 的值。

**3. 留出验证**
```bash
python calibration/heldout.py assets/r1d/r1d.urdf $O/fit_a.urdf -- $O/r3     # 各 URDF 在留出数据上的力矩残差
python calibration/fit.py evaluate $O/fit_a.urdf $O/r3 --record $O/eval_a.json
python calibration/compare.py $O/r3                                           # 各路径点的停点误差
```
留出误差不再明显下降时停止迭代；还在下降就再采一轮、把更多数据放进拟合。

**4. 安装并做关节测试**
```bash
cp $O/fit_a.urdf assets/r1d/r1d_calibrated.urdf
python tools/run.py joint                  # 14 个关节逐个走完整行程，再测两个夹爪，约 14 分钟
```
- 关节测试从基准姿态（双臂下垂、肘 80°）出发，每个关节走到行程两侧的中点和端点再返回，记录到位误差、回位误差、回差和其他关节的偏离。默认合格线 0.5°。
- **头抬平再做**：头在下棋视角时，肩 pitch 的大角度会碰到头。

**5. 腰和头**（腰、头运动，双臂保持）
```bash
python tools/run.py hold arms --pose hang                               # 双臂到下垂姿态
python tools/run.py hold sweep --joint waist_yaw  --report $O/waist_sweep.json
python tools/run.py hold sweep --joint head_pitch --report $O/head_pitch_sweep.json
python tools/run.py hold sweep --joint head_yaw   --report $O/head_yaw_sweep.json
python tools/run.py hold fit --records $O/waist_sweep.json $O/head_pitch_sweep.json $O/head_yaw_sweep.json
python tools/run.py hold verify --joint waist_yaw  --period 0.02 --report $O/waist_verify.json
python tools/run.py hold verify --joint head_pitch --period 0.02 --report $O/head_pitch_verify.json
python tools/run.py hold verify --joint head_yaw   --period 0.02 --report $O/head_yaw_verify.json
```
- `sweep`：单关节以几档恒速往返，记录力矩。`fit`：拟合干摩擦、粘性摩擦和随位置变化的保持力矩，写入 `assets/r1d/hold_joints.json`。`verify`：走随机目标并报告停点误差。
- `--period 0.02` 让前馈按象棋服务的 50 Hz 更新，验证的就是下棋时的效果。
- **先核对本机腰的实际可转范围**。`tools/hold_calib.py` 的 `RANGE` 写的是参考机器人的范围（腰约 ±0.56 rad，受机械限位），新机器人不同时要先改它，或用 `--centre`、`--amplitude` 缩小扫描范围。
- 不要用 50 Hz 的前馈去加“附加刚度”，会自激振荡并带着双臂抖；刚度只放在电机增益里。工具里有振荡保护，触发后本次验证会停止。

**6. 把结果装进象棋配置**（在包根下运行）
```bash
python setup/arm-calibration/scripts/install_results.py                 # 空跑：打印将要改动的内容
python setup/arm-calibration/scripts/install_results.py --write --evidence "runs/arm_calibration"
systemctl --user restart r1d-runtime.service                           # 重启 Runtime，腰头增益才生效
```
它把拟合的 URDF 复制为 `assets/models/r1d/r1d_arm_dynamics.urdf`，把 URDF 里的摩擦系数写进 `config/arm_torque_ff.json`（重力比例置 1，积分、阻尼、限幅保持不变），复制 `hold_joints.json`，并更新 `config/runtime.json` 的腰头增益。

**7. 在 Runtime 链路上验证**：按 [joint-tracking](../joint-tracking/README.zh.md) 的 A 部分跑对比，确认下棋实际使用的链路上跟随误差达标。

## 验收标准

| 项 | 标准 |
|---|---|
| 双臂关节测试 | 14 个关节全部 PASS（到位误差 ≤ 0.5°） |
| 留出数据 | 新 URDF 的力矩残差和停点误差明显小于官方 URDF，且再加数据不再明显下降 |
| 腰、头 `verify` | 停点误差最大值在几个毫弧度以内，没有触发振荡保护 |
| 接管后 | 象棋服务接管并稳定后，腰和头离参考位 ≤ 5 mrad，双臂离 init ≤ 20 mrad（`bash r1d start` 会核对） |

## 说明

- 标定结果随时间会变（减速器磨合、温度），到位误差明显变大时重做。
- 标定是空载、在一个升降高度下做的；带负载和其他高度下的精度没有覆盖。棋子只有 10–20 g，可以忽略。
- `--ignore-link-loss` 是为个别电机通信不稳的机器准备的开关，默认不需要。
- `r1_d/config.json` 的 `ip` 和 `remote_python` 是机器人侧的默认值，机器人配置不同时修改；`provenance/` 里是代码来源记录，与运行无关。

> 参考（参考机器人）：
> - 双臂三轮探索共约 400 个路径点、实际采集约 1.7 小时；留出数据上的停点误差均值 0.078°、最大 0.43°；关节测试最大误差 0.36°。
> - 腰静摩擦约 0.6 Nm；头 pitch 在下棋视角需要约 1 Nm 的重力保持力矩，没有前馈时会下垂约 20 mrad。电机增益取腰 150/6、头 pitch 150/4、头 yaw 40/0.4。
> - 25 个随机停点的最大误差：腰 7.5 → 3.2 mrad，头 pitch 19.4 → 1.5 mrad，头 yaw 7.5 → 4.2 mrad。象棋服务链路下多次启动（含冷启动），腰离参考位 0.6–0.8 mrad，头 0.3–1.4 mrad。
> - 一段时间后复测，个别关节的干摩擦有明显变化（左腕 pitch 从约 0.19 Nm 变为约 0.77 Nm），重新拟合后恢复精度。
