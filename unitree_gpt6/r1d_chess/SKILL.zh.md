---
name: r1d-chess
description: 由 agent 手动控制 Unitree R1-D（R1-A7-D，Dex1 夹爪）下中国象棋：启动象棋服务并接管、读棋与选着、生成抓放位姿、完整预检、一次性执行、串行视觉门禁、故障回撤和走后复盘。用户要求用本目录让 R1-D 走棋、检查服务或棋盘状态时使用。
---

# R1-D 象棋控制（agent 手动流程）

[English](SKILL.md) | 中文

本 skill 的目录就是包根。所有命令在包根下用 `bash r1d <子命令>` 运行（等价于 `python scripts/<入口>.py`，解释器依次取 `R1D_CHESS_PYTHON`、`<包根>/.venv/bin/python`、PATH 上的 `python3`）。脚本、配置、模板和模型都相对包根定位；运行产物只放在 `runs/` 下。执行中的门禁命令一条一条单独发，路径写绝对路径。

看图读棋、选着编排、选手和姿态、每个门禁的判断都由 agent 完成；Python 服务负责预检、token、运动执行和安全停机。

进程分两层：**机器人运行时**（下文简称 Runtime，代码在 [third_party/robot_runtime](third_party/robot_runtime/README.zh.md)，以 systemd 用户单元 `r1d-runtime.service` 运行）是唯一向机器人下发命令的进程；**象棋服务**（`bash r1d serve`）通过本机 IPC 调它，agent 只和象棋服务打交道。本包没有自动走完一轮的总控脚本。

## 选择任务与读取资料

- **新环境、新机器人或现场未确认**：先读 [README](README.zh.md) 和 [setup/README](setup/README.zh.md)，运行 `python scripts/check_environment.py` 和 `python scripts/setup_status.py`（都离线，不连机器人）。`config/` 里随包附带的是参考机器人的标定结果，**开始下棋前必须先完成 setup**：本机环境（[第 0 步](setup/README.zh.md)）、[相机检查](setup/camera-check/README.zh.md)、[机械臂精度标定](setup/arm-calibration/README.zh.md)及[跟随验证](setup/joint-tracking/README.zh.md)、[头部相机标定](setup/head-camera-calibration/README.zh.md)、[下棋环境参数和棋盘标定](setup/chess-scene-parameters/README.zh.md)。setup 中会让机器人运动的步骤都要用户明确授权。
- **只启动服务、接管或检查状态**：读 [操作手册](docs/operation_manual.zh.md)“开始下棋任务”，用 `goinit`、`serve`、`handover`、`client get_state`。完成指定范围后等待，不自动走棋。
- **读棋、核对棋盘位置、棋盘移动后重新标定**：读 [board_position](board_position/README.zh.md)，用 `boardview`、`fencheck`、`boardpoints`、`boardcal`。首次棋盘标定在 setup 的场景参数里完成。
- **执行一轮**：先读 [round_plan](prompts/round_plan.zh.md)，按下面流程执行；精确命令和字段查 [操作手册](docs/operation_manual.zh.md) 和 [脚本清单](docs/round_script_inventory.zh.md)，运动前备齐。
- **故障、中断或用户要求回位**：读操作手册“出错、回位和结束”，先核对实际会话和结果。execute 客户端被打断不等于机器人已取消，不盲目重发 execute、重启服务或复用旧 token。

任务范围以当前用户指令为准。读文档、整理目录或离线检查不授权启动服务和运动。

**运动前的检查**：第一次打开运动（`serve --allow-motion`、`enable_control`、`goinit --execute`、`start --execute`）之前运行 `python scripts/setup_status.py --require`。有必做项仍是“参考值，未 setup”时不要走子：告诉用户缺哪几项，按 setup 流程完成；只有用户明确决定沿用某项参考值时，才用 `--confirm <项> --note "<依据>"` 记录后继续。

## 单轮工作流程

1. **接管**。机器人重启后先 `bash r1d goinit` 预览，用户确认安全后才 `goinit --execute`（不做预检，不得自行执行）。然后启动 Runtime（`systemctl --user start r1d-runtime.service`；这个单元第一次使用前由 `python scripts/install_runtime_service.py --write` 生成）和服务（`bash r1d serve --allow-motion --arm 所选手`），用 `handover --arm X --execute`（有保持单元时）或 `client enable_control --arm X` 接管。`get_state` 确认无 fault、`head_waist_session_active=true`、头 pitch 约 0.62 后，`client release_control` 关闭执行、保留会话。上一轮回 init 后服务已卸力并结束会话，所以每轮都要重新接管。
2. **建 run、查双臂**。`round init --run <包根>/runs/<本轮>`。两臂 TCP 离各自 init ≤10 cm 直接从当前位置开始，否则先 `client prepare_init`/`execute_init`。
3. **读棋**。只拍一张 `cam_left_high` 规划图，确认完整棋局和对手实际着法，写 `board.json`。看不清时用 `boardview` 和 `fencheck`（只用于读棋，不用于估位姿）。
4. **选着**。有上一轮预测先 `engine match`，失配或耗尽再 `engine analyze`。
5. **选手和位姿**。source 在 a–d 列优先左手，e–i 列优先右手；换手用 `client set_active_arm` 并重拍。用 `bash r1d pose pose --role source|target|tray ...` 生成 TCP 位姿（不是棋子位置），填 `round_spec.json`。吃子先把对方棋子放进同侧圆盘。
6. **建计划、预检、取 token**。`round build` → `client prepare_transaction`（吃子用 `prepare_sequence`，两段联合预检）→ `round tokens`。首选手整条不可达才换另一手，两手都不行才换着法。计划、起点、执行手变化或服务重启后都要重新预检。
7. **执行**。预检后先 `get_state`、等约 3 s，再 `client enable_control`，然后只提交一次 `execute_transaction`/`execute_sequence`（长调用，返回前不发其他请求）。
8. **门禁**。另一调用里循环 `serial next` → 看本检查点的头图和执行手腕图 → `bash r1d gate --decision ...` → 立刻 `next`。门禁由 agent 自己判断并立即提交：
   - 源高位：棋子在两爪中间则 `success`，否则 `adjust`（XY 偏移，最多三次）。
   - 闭爪前：棋子中心已过爪尖、大致在两爪之间则 `success`；大半在爪尖外用 `adjust_pitch`（最多两次）；明显偏到会被单爪顶开才 `retry_high`。
   - 释放前：用腕图独立量落点，不能只看 FK；偏差 >5 mm 提交 `adjust`（最多三次），最后一次修正必须确认修正后可安全释放，否则 `failure` 停住持棋。
9. **释放与回位**由服务自动完成，回到 init 后自动卸力。
10. **复盘只做两项**：新头图判断成功或偏位；`review` 加 `history finish` 记录 FK 误差。

出错或要回位时用 `bash r1d retract`，它不代替门禁审批；不要用 `client set_gripper` 手动张爪。

## 真实能力边界

- 随包的 `config/` 是参考机器人的结果，不代表当前机器人；setup 未完成时的任何几何和跟随精度都不可信。
- 几何是模型估计，可以规划和执行，但不等于抓放准确；`placed`、小 FK 误差或命令被接受都不能代替走后头图确认。
- 预检只查 IK、关节限位、桌面和工具包络，只检查执行手；没有全机械臂、双臂、邻子和动态环境碰撞检测，邻子要按本轮图像复核。
- 执行中的调整（XY 修正、减倾角、回高位、释放修正）不做预检，只解 IK 直接下发。
- 没有自动识棋；`boardview` 只是辅助读图。
- 同一台主机上同一服务端点只能有一个象棋服务实例。

报告实际完成的动作、最终视觉结论、可用的误差和本轮产物路径；缺少图像或结果时写明缺口，不补造走棋成功。
