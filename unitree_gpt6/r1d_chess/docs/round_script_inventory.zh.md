# 脚本清单

[English](round_script_inventory.md) | 中文

`bash r1d <子命令>` 分发到 `scripts/<入口>.py`；入口都是 5 行薄包装，经 `scripts/_bootstrap.py` 把 `src/` 加入 `sys.path` 后用 `runpy` 运行对应模块，不需要 pip 安装。所有模块用 `r1d_chess.paths.ROOT` 定位包根。

## 子命令与入口

| 子命令 | 入口 | 模块 | 作用 | 连接机器人 |
|---|---|---|---|---|
| `serve` | `scripts/serve.py` | `robot.serve` | 象棋 REP 服务（预检、token、执行、检查点） | 是 |
| `runtime` | `scripts/r1d_runtime.py` | `robot.r1d_runtime` | 内置机器人运行时（`third_party/robot_runtime`）：`serve`（systemd 单元运行它）、`config`、`status` | 是 |
| `client` | `scripts/robot_client.py` | `robot.robot_client` | 服务客户端：`get_state`、`capture_cameras`、`set_active_arm`、`prepare_*`、`enable_control`、`release_control`、`execute_*`、`unload_arms` 等 | 经服务 |
| `start` | `scripts/start.py` | `robot.start` | 一条命令：启动 Runtime 和服务、goinit、handover、核对到位 | 是 |
| `goinit` | `scripts/go_init.py` | `robot.go_init` | 重启后回参考位（不预检，需人确认） | 是 |
| `handover` | `scripts/handover.py` | `robot.handover` | 停保持单元并交给象棋服务 | 是 |
| `retract` | `scripts/retract.py` | `robot.retract` | 倒序重放已走路径回到起点 | 经服务 |
| `serial` | `scripts/serial_checkpoint.py` | `robot.serial_checkpoint` | `next`/`submit`：只通过本地文件处理检查点 | 否 |
| `gate` | `scripts/gate_submit.py` | `chess.gate_submit` | 按阶段生成评价并立即 `serial submit` | 否 |
| `round` | `scripts/round_tools.py` | `chess.round_tools` | `init`/`build`/`tokens`/`report` | 否 |
| `engine` | `scripts/xiangqi.py` | `chess.xiangqi` | `analyze`/`match`/`validate`（Fairy-Stockfish + pyffish） | 否 |
| `pose` | `scripts/grip_pose.py` | `chess.grip_pose` | 由格名生成 TCP 位姿；`check` 核对 spec | 否 |
| `candidates` | `scripts/preflight_candidates.py` | `chess.preflight_candidates` | 批量预检候选，不执行 | 经服务 |
| `review` | `scripts/review_pose_audit.py` | `chess.review_pose_audit` | 事件 FK 的三种误差 | 否 |
| `history` | `scripts/xy_error_history.py` | `chess.xy_error_history` | `show`/`finish`：误差累计和命名复盘 | 否 |
| `boardview` | `scripts/board_view.py` | `board.board_view` | 俯视图、逐格放大图、去畸变图 | 否 |
| `fencheck` | `scripts/fen_check.py` | `board.fen_check` | 把 FEN 画回俯视图核对 | 否 |
| `boardpoints` | `scripts/board_points.py` | `board.board_points` | 90 个交叉点实测，可写回几何 | 否 |
| `boardcal` | `scripts/board_calibrate.py` | `board.board_calibrate` | 拍图、配准、精修并写棋盘几何 | 拍图时经服务 |
| `board` | `scripts/board_registration.py` | `board.board_registration` | 从实测交点拟合棋盘平面 | 否 |
| — | `scripts/head_board_measure.py` | `board.head_board_measure` | 棋盘粗测（含板面高度） | 否 |
| — | `scripts/check_environment.py` | — | 离线环境检查 | 否 |
| — | `scripts/install_runtime_service.py` | — | 生成 `r1d-runtime.service` 用户单元 | 否 |
| — | `scripts/setup_status.py` | — | 各项配置是参考值还是已在本机完成 setup | 否 |

## Setup 工具（`setup/`）

新机器人、新场地上线时使用，流程见 [setup/README.zh.md](../setup/README.zh.md)。脚本直接运行（`python -B setup/<目录>/scripts/<脚本>.py`），通过 `setup/_common.py` 加载 `src/`。

| 目录 | 脚本 | 作用 | 是否运动 |
|---|---|---|---|
| `camera-check/` | 无脚本，用 `client capture_cameras` | 相机流与物理左右手的对应 | 否 |
| `chess-scene-parameters/` | `capture_initial_pose.py` | 记录初始姿态和头腰升降参考 | 只读 |
| | `dex1_envelope.py`、`reach_check.py`、`view_resolution.py` | 夹爪尺寸核对、IK 可达抽查、视角分辨率 | 否 |
| | `write_scene.py` | 由棋盘测量结果生成 `robot.json` 的场景和 `board_geometry.json` | 否 |
| `head-camera-calibration/` | `stage0_check.py`、`head_calib_record.py`、`head_calib_intrinsics_capture.py` | 只读检查、录制标定视频 | 否 |
| | `head_calib_detect.py`、`head_calib_intrinsics_solve.py`、`stereo_fit.py` | 角点检测、内参和双目外参拟合 | 否 |
| | `head_calib_capture.py` | 头部多姿态采集、单姿态拍图 | 头部 |
| | `capture_check.py`、`head_calib_solve.py`、`arm_overlay.py` | 采集质量检查、外参解算与验收、目视检查 | 否 |
| `joint-tracking/` | `bench.sh`、`arm_tracking_benchmark.py` | Runtime 链路上的跟随对比：快照、规划、执行、汇总 | 双臂 |
| | `joint_report.py`、`diagnose.py`、`model_fit.py`、`friction_fit.py`、`ilc_coulomb.py` | 结果分析和前馈参数微调 | 否 |
| `arm-calibration/` | `r1_d/calibration/explore.py`、`fit.py`、`heldout.py`、`compare.py` | 双臂探索、动力学拟合、留出评估 | 双臂（explore run） |
| | `r1_d/tools/run.py`（`joint`、`hold`、`height`）、`unload_arms.py` | 关节测试、腰头标定、高度测试、卸力 | 是 |
| | `scripts/install_results.py` | 把拟合结果装进象棋配置 | 否 |

## 实现模块（`src/r1d_chess/`）

| 子包 | 模块 | 内容 |
|---|---|---|
| `robot/` | `serve` | `R1DService`：R1-D 构造器，继承通用事务引擎 |
| | `internal_service` | 通用事务引擎 `RobotService`：预检、token、执行、检查点、空抓重试、回位卸力 |
| | `motion_harness`、`fast_correction` | 笛卡尔插值、IK 路径、执行中修正路径 |
| | `board_guard` | 桌面与工具包络检查 |
| | `release_alignment`、`grasp_vision` | 释放门禁评价校验与修正、抓取判定 |
| | `checkpoint_runner`、`serial_checkpoint` | 检查点文件协议和提交 |
| | `arm_roles` | 左手执行时的角色/关节映射 |
| | `camera_policy` | 规划/执行相机选择 |
| | `pose_audit`、`async_event_log` | 事件快照和 JSONL 日志 |
| | `r1d_transport`、`r1d_runtime` | Runtime IPC 会话、DDS 订阅、模拟 transport |
| | `r1d_kinematics`、`r1d_urdf` | URDF FK/IK（阻尼最小二乘为主，scipy 回退） |
| | `r1d_gravity`、`r1d_torque_ff`、`r1d_hold_ff` | 臂重力和力矩前馈、腰头保持前馈 |
| | `r1d_config` | profile 加载、标定和模型哈希校验 |
| | `go_init`、`handover`、`start`、`retract`、`robot_client` | 见上表 |
| `chess/` | `round_tools`、`grip_pose`、`xiangqi`、`move_constraints` | 本轮计划、位姿、棋规与引擎、选手策略 |
| | `gate_submit`、`preflight_candidates`、`review_pose_audit`、`xy_error_history` | 门禁提交、候选预检、复盘 |
| `cameras/` | `r1d_cameras` | TeleImager 取图（头部双目裁单眼、腕流） |
| `board/` | `board_view`、`fen_check`、`board_points`、`board_register`、`board_calibrate`、`board_registration`、`head_board_measure` | 读棋辅助图和棋盘几何测量 |

## 运行期文件（`runs/<本轮>/`）

| 路径 | 产生者 |
|---|---|
| `round_spec.json`、`resume.json`、`tmp/`、`images/`、`docs/` | `round init` |
| `board.json` | agent 写 |
| `tmp/request.json`、`tmp/plan_summary.json` | `round build` |
| `tmp/prepared.json` | `client prepare_*` |
| `tmp/execute_request.json`、`tmp/stage_N_prepared.json` | `round tokens` |
| `tmp/checkpoints/…`、`tmp/eval_*.json` | 服务、`gate` |
| `tmp/result.json` | `client execute_*` |
| `tmp/visual_review.json`、`docs/pose_review.json`、`docs/第N轮_….json`、`docs/execution_xy_errors.jsonl` | agent、`review`、`history finish` |

服务日志在 `runs/<时间>_service/events.jsonl`。服务和 `round` 工具都拒绝 `runs/` 以外的 run 目录。
