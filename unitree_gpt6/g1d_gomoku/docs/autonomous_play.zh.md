# Agent 自主执行一手棋

[English](autonomous_play.md) | 中文

本页供宿主 Agent 操作。用户给“下一手”即可，不要求用户写任何阶段文件。下列文件都是 Agent 自己观察、计算后保存的内部接口。

## 1. 初始化与持续控制

按 README 复制硬件配置模板，填写当前网络，并从当前设备只读采集归位参考。按 [硬件接入](hardware_integration.zh.md) 做离线预检、只读状态和相机检查。经验库初始为空；尺寸、高度和臂形从当前场景求解，成功后按 [经验数据约定](experience.zh.md) 记录。先核对控制权和整条运动路径，不直接照搬旧轮 XY，也不把记录当实时反馈。

只启动一次持续发布器，在独立长期进程中运行：

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main --config config.local.yaml \
  control --root game --execute owner
```

`game` 相对 `runtime.directory`，后续用其绝对路径作为 `--owner-root`。状态在 `game/session/status.json`，只读诊断用 `control --root game status`。如不在归位姿态，Agent 先规划并核对回撤通道，调用 `control --root ROUND --owner-root GAME --execute reset reset_start --duration D --review REVIEW`；该命令按实测起点到本地参考插值，不具备避障能力。低位或持棋状态不能直接归位穿过障碍，应先由 Agent 规划抬升、退出。

启动后保持同一发布器，观察和思考期间不反复换控制进程。当前配置、反馈和 SDK 故障检查由工具保留；保护退出时停止后续任务并查清状态。

## 2. 观察新一轮

为本手选择尚不存在的目录，在归位状态运行：

```bash
python -m unitree_gpt6.g1d_gomoku.src.main play \
  --config config.local.yaml --owner-root GAME_ABSOLUTE --round ROUND_ABSOLUTE
```

工作流先检查归位反馈，恢复四路采图，保存 `board_capture/` 原图与状态、`capture.json`，进入 `awaiting_board_review`。只返回状态提示；原图和详细路径从本轮文件读取。

宿主 Agent 用看图工具打开四路原图，左右头图交叉核对完整网格、黑白棋及轮次，腕图辅助核对供子。以左头图为准，行向下、列向右，19×19 都从 1 编号，外框不计入网格。看不清时在归位状态重新采图，重建新轮目录，不能猜空点或请求用户标记。

## 3. 识别结果与自动选点

Agent 写 `ROUND/board_observation.json`，恰好包含以下字段：

| 字段 | 内容 |
| --- | --- |
| `capture_id` | 从本轮 `capture.json` 读取 |
| `recognition_source` | `agent_visual_review` |
| `board` | 完整 19 行 × 19 列整数矩阵；0 空、1 黑、2 白 |
| `views_agree` | 已交叉核对左右头图时为 true |
| `uncertain_intersections` | 完成复核后为空列表；仍有疑点不能继续 |
| `grid_corners_px` | 左头原图最外四个**网格交点**的像素，顺序左上、右上、右下、左下；不是棋盘外框 |
| `notes` | Agent 的识别依据、网格方向和交叉核对结论 |

这些是视觉测量结果，不能加入目标或建议落点字段。不要从旧棋局复制矩阵。引擎要求轮到黑方（黑白数量相等），且无人已经成五。识别冲突先重新看图。

```bash
python -m unitree_gpt6.g1d_gomoku.src.main decide --round ROUND_ABSOLUTE
```

引擎自动选点：本手成五优先，其次处理白方直接成五威胁，再比较连续与跳空棋形。多处无法同时防守会明确记录，不虚构必胜。结果写 `position_and_move.json`，原图副本用 SVG 内嵌图像圈出唯一交点，进入 `target_selected`。投影仅用于图像标注，**不能把它当成机器人标定**。

## 4. 抓取、转运与低位对准

规划前先读 [抓子“之”字型臂形先验](manipulation.zh.md#抓子对准时的之字型臂形先验)，用当前反馈核对肩肘腕分支和后续下降、短抬升的限位余量；没有姿态默认值；匹配的成功经验也不能直接作为运动命令。

宿主 Agent 读取引擎选择的行列和标注，用当前四路图像、TCP/关节反馈及现场标定估计目标位姿。动作前检查整条路径和持棋偏移，用底层 `control` 工具提交阶段。下面参数由 Agent 计算，用户不填写：

```text
control --root ROUND --owner-root GAME --execute move STAGE
        --pose X Y Z ROLL PITCH YAW --duration D --review REVIEW --quiet
control --root ROUND --owner-root GAME --execute grip STAGE
        --gripper OPENING --review REVIEW --quiet
```

完整调用前缀均为 `python -m unitree_gpt6.g1d_gomoku.src.hardware.main --config CONFIG`。每阶段用不同名称，实际路径记录在本轮目录；末端位姿为 pelvis、米与弧度。工具用当前反馈求解 IK并核对速度、关节、左臂保持与计划连接；`--review` 记录 Agent 对本轮图像和整段净空的检查，数值采样本身不是碰撞检测。

桌面取子从安全高位到附近，预览位校正，低位核对双指内缘/棋心/接触线后固定臂位慢收。短抬升核对单颗随爪、原位为空、邻子未动，失败就依据当前图像修正。高位到棋盘后，右腕图用邻棋行列及两条网格线确认同一空交点，主要 XY 修正在高位完成；分段垂直下降。不要仅凭图像中心、棋子顶面投影或 TCP 误差判断落点。

## 5. 确认前准备返程

低位最终 XY 确定后，用本轮来路生成完整返程，固定保存为 `ROUND/return_plan.json`：

```text
control --root ROUND --owner-root GAME prepare_return return
        --pose LIFT_X LIFT_Y LIFT_Z ROLL PITCH YAW
        --outbound-plan THIS_ROUND_HIGH_PLAN --review REVIEW --quiet
```

`--pose` 是保持当前 XY 和朝向的垂直抬升目标；`--outbound-plan` 必须是本轮已完成且检查过净空的高位计划，也可改用已审阅的 `--via-pose`。工具会反向连接高位来路并接本地归位参考。最终目标变化则重建相应连接、核对完整返程，全部在用户确认前完成。

Agent 在当前图像与夹爪反馈基础上计算松爪开度，先写 `release_parameters.json`，字段为 `right_gripper_rad`（有限弧度数，位于配置的夹爪范围内）和 `review`（非空的求解与核对依据）。该文件与返程一起绑定到松爪确认；改变后必须重新准备确认。

Agent 看完最后低位阶段的四路 `after/` 原图后，写 `alignment_observation.json`：

| 字段 | 内容 |
| --- | --- |
| `decision_sha256` | 本轮 `position_and_move.json` 的 SHA256 |
| `matches_selected_intersection` | 已核对与引擎选点一致时 true |
| `target_empty_before_descent` | 已核对目标交点为空时 true |
| `single_black_stone_held` | 已由短抬升等图像核对单颗黑子随爪时 true |
| `grid_neighbor_review` | 邻棋行列、两条网格线与持棋偏移的实际视觉结论 |
| `stage_directory` | 本轮最后低位对准阶段目录的绝对路径 |

```bash
python -m unitree_gpt6.g1d_gomoku.src.main prepare-release \
  --config config.local.yaml --owner-root GAME_ABSOLUTE --round ROUND_ABSOLUTE
```

工作流核对低位阶段、图像、当前持有姿态、目标和返程一致，记录 `release_checkpoint.json`，进入 `awaiting_release_confirmation`。现在向用户说明实际落点、松爪和连续返程，等待确认；取子和精调过程无需逐步询问。

## 6. 确认后释放与连续回撤

只有用户实际确认后，记录其原话并立即执行：

```text
python -m unitree_gpt6.g1d_gomoku.src.main release
       --config CONFIG --owner-root GAME --round ROUND
       --confirmation USER_ACTUAL_CONFIRMATION
```

不再临时规划、不更换发布器。流程核对保存计划未变，单次消费本轮确认，低层同会话慢开夹爪、只保存一次四路图像、连续垂直抬升/退出/归位。期间不查看、不分析放子后图像，不再采图，运行时健康检查继续。

客户端等完整回撤结束，`returned` 还要求实测右臂关节在归位容差内。`physical_placement_verified: false` 表明没有最终交点视觉验收。出现 `execution_failed` 或 `return_unverified` 时查当前状态，不能再次运行 release 重放。一手结束后等对手及用户下一轮指令，新轮使用新目录。

## 成功后保存经验

完成归位后，宿主 Agent 按 [经验数据约定](experience.zh.md) 提交本轮实测数据。写入发生在休息状态，不能延长持棋或低位等待。
