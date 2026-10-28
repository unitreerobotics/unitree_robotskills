---
name: g1d-chess
description: 控制使用 Dex1 内置夹爪的 Unitree G1 下中国象棋，包括环境与现场配置、服务及腰锁、棋盘点位标记、单轮规划、左右手抓放、连续吃子、检查点判断和复盘。用户要求使用本仓库进行这些机器人象棋任务时使用。
---

# G1D 象棋控制

[English](SKILL.md) | 中文

本 skill 的目录就是仓库根。使用用户已安装依赖的Python环境，直接运行`python scripts/…`；独立工具调用时确认当前解释器，或使用其绝对路径。所有脚本、配置、资料与模型均相对这个目录定位；在终端调用中显式设置 workdir 和实际绝对路径，不沿用其他会话的 shell 变量、token 或机器人状态。

内置Dex1左右爪默认开爪方向均为`+1`，正电机角度张开、`q=0`闭合。普通环境迁移沿用该默认，不按左右手翻转，也不把方向列为必填现场测量；仅硬件/固件关节约定不同时按依据覆盖。规划`gripper_open_q`填正幅值，服务乘方向符号后下发；客户端`--q`接收带符号的实际电机角度。

## 选择任务与读取资料

- 新设备或环境未确认：先读 [README](README.zh.md)、[环境安装](docs/environment_setup.zh.md) 和 [首次配置](docs/first_game_setup.zh.md)。scene 数值优先采用用户提供且与当前坐标系、工具及现场兼容的数据；用户未提供任何现场信息时，由模型按首次配置第5节自行估计全部scene字段。用户仅提供部分信息时，模型补齐其余字段。结合可用观测、机器人与工具模型、尺寸资料及合理假设，记录来源、估计依据和不确定度，继续设置与核验，不把用户提供数值、逐项补充资料或额外确认作为前置条件。估计值明确标注为估计。先完成离线部分，再按当前任务授权连接硬件。
- 只建立/复用服务、锁腰或检查 init：读 [操作手册](docs/operation_manual.zh.md) 0.1.2、0.1.3 与第4节，使用 `scripts/start_service.sh` 和 `scripts/robot_client.py`。完成指定范围后等待，不自动走棋。
- 标记新棋盘：读 [board_position](board_position/README.zh.md) 及其 [标记任务](board_position/prompts/mark_board.zh.md)，使用 `scripts/mark_board.py`。输出是当前图像的90点像素资料，不是机器人坐标或手眼标定。
- 执行一轮：先读 [round_plan](prompts/round_plan.zh.md) 与 [运行接口及实现边界](docs/runtime.zh.md)，按下述流程执行。精确命令和字段从 [操作手册](docs/operation_manual.zh.md)、[脚本清单](docs/round_script_inventory.zh.md) 按需查阅，在运动前备齐。
- 中断、超时或用户要求回撤：先读操作手册第9节及 runtime，核对实际会话、事务结果和状态。execute 客户端被打断不表示机器人已取消，不盲目重发 execute、重启服务或复用旧 token。

任务范围以当前用户指令为准，沿用已有的本轮执行授权；读文档、整理仓库或离线检查不会授权启动服务和运动。

## 单轮工作流程

1. 确认依赖与现场配置已完成，核对本仓库 ROOT、服务单实例、物理执行手及腰锁。双臂当前末端到各自 init 的三维距离 ≤10cm 时免除起步前重复初始化；当前状态仍作为新计划起点。
2. 只采一张新的 `cam_left_high` 规划图与同批状态，确认完整局面、行棋方及黑方实际着法。按 [棋盘与走法规范](prompts/board_and_move_conventions.zh.md) 确认格名；有当前兼容预测先按 [预测规则](docs/move_forecast.zh.md) 匹配，无效才重新调用已安装引擎并用 pyffish 验证。
3. 按 source 选首选手（a–d 左、e–i 右）。独立估计本轮 source/target 的 XYZ、RPY 与邻子空间；高位开爪后的实际夹口须仍覆盖可抓区域，不能把 source 偏移复制给 target。先完整预检首选手，确实不可达再用另一手重新估计和预检。
4. 使用 `scripts/round_tools.py` 或 `scripts/round_tools_left.py` 建立本轮 `runs/`、build 并提取提交文件。普通走子完整预检后只发一次 execute；吃子将移走被吃棋子和移动己方棋子联合为一个 sequence 预检、一次 execute，第一段释放后连续进入第二段。
5. 保留 execute 的长调用会话，在另一调用中用 `scripts/serial_checkpoint.py next/submit` 处理检查点。每次只依据本检查点头图和物理执行手腕图判断并提交；不要求棋子百分百居中，能抓且邻子空间满足即可尝试。XY 不足回高位；XY 已可夹且仅高度接触不足时，按 round_plan 使用自动5mm `adjust_z`。source 高位和 target 低位 XY 各最多3次，第三次按当前流程放行；实现差异以 runtime 记录为准，不能把手册要求宣称成服务能力。
6. 释放时根据当前腕图核对预计棋子落点与目标交点、邻子及独立下落空间。动作结束后仅做两项复盘：新头图判断成功/偏位；用 `scripts/review_pose_audit.py` 及 `scripts/xy_error_history.py finish` 记录两个事件的FK误差并合并视觉结论。`placed`、小FK误差或命令接受均不能代替实际落格确认。

普通结束执行臂回自身 init、非执行臂保持本轮位姿，关闭手臂控制并保留腰锁。释放腰锁或结束服务按本次明确要求处理。相机流名与物理左右手不一定同侧，以现场配置和本 token 的 `camera_order/camera_selection` 为准。

## 真实能力边界

本仓库提供计划、服务和采图入口；主会话仍负责看图、选招编排、位置估计和六/七字段评价，不存在 `play_one_round.py` 等自动总控。模型、模板与离线测试不证明新设备抓放准确，也不替代现场测量、完整计划预检或实际硬件故障处理。

报告实际完成的动作、最终视觉结论、可用的误差与本轮产物路径；缺少图像、快照或执行结果时明确记录缺口，不补造走棋成功或实机验证。
