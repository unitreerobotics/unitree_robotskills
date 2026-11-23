# R1-D 每轮象棋提示词

[English](round_plan.md) | 中文

先读 [SKILL.zh.md](../SKILL.zh.md) 和 [操作手册](../docs/operation_manual.zh.md)。运行产物放在 `runs/<本轮>/`，评价模板在 `assets/templates/`。

本轮若只规划，止于规划和预检；只有本轮已授权实际走棋时才启用和 execute。数值完整、依据明确的 `model_estimate` profile 足以规划、预检和执行，不需要等实物标定；来源如实记录，不得把估计标成 `calibration_verified=true`。缺 TCP、init、scene 或释放高度时只做只读观测。

## 模型、profile 与跟随设置（必读）

- **运动学和重力模型**：包内 `assets/models/r1d/r1d.urdf`（含 Dex1，腕 pitch 限位 ±0.5236），由 `config/runtime.json` 的 `model_urdf` 指定。不要用其他 URDF 计算 TCP、init 或场景，也不要设置 `R1D_MODEL_URDF` 去覆盖它。
- **profile 只能是当前 `config/robot.json` 或从它派生**：棋盘未动就直接用它；棋盘或场景变化时复制它，改 `scene`、`board_release_z_m`、`tray_release_z_m`、`estimate_evidence`，再用 `serve --profile` 加载。必须保留 `kinematic_model_sha256`、`init_xyz_rpy` 和 `arm_tracking_limit_rad`（0.20）。`kinematic_model_sha256` 缺失或与当前 URDF 不一致时，服务启动和预检都会拒绝；不得手抄哈希让它过检，几何必须按当前模型估计。
- **力矩前馈默认开启**：`config/arm_torque_ff.json` 由服务自动加载，除非用户明确要求，不要修改或关闭。`get_state` 返回的 `arm_torque_ff` 可用来核对是否生效。
- **跟随限制**：运行中关节跟随误差超过 0.20 rad 会锁存故障；执行起点要求关节偏差 ≤0.08 rad、TCP 偏差 ≤15 mm。故障后按实际状态处理，不得为了通过检查而放宽阈值。
- **场景参数**在 `config/robot.json`（TCP 在夹爪法兰、`scene`、释放高度），棋盘交叉点在 `config/board_geometry.json`，含义见 `docs/r1d_scene_parameters.zh.md`。棋盘移动后先重测（`bash r1d boardcal`，大幅移动时先用 `python scripts/head_board_measure.py --prior config/board_geometry.json` 粗测，先验可以避免网格错一格）。
- **头部锁定在 `config/robot.json` 的 `head_q_rad`**（接近最低头）。服务接管时会把头斜坡转过去，规划图都在这个视角下拍。
- **抓放姿态：pitch 0.8 rad，用夹爪前部（夹持点离法兰 137 mm）。** `bash r1d pose` 默认即此姿态：抓取指尖在板面上方 3 mm；**所有走棋的棋盘释放 TCP z 固定 0.150 m，与 pitch 无关**（托盘按 profile）；释放张爪在开爪方向两侧邻点都空时为 3.0，否则 2.3（`round build` 自动判断）；`gate_z_m` 用 0.175（0.16 不满足运输净空）。
- **spec 的 source/target/capture 位姿是 TCP（Dex1 法兰），不是棋子位置。** 夹持点 = TCP + 0.137·(cos p·cos y, cos p·sin y, −sin p)。一律用 `bash r1d pose pose --role source|target|tray --square 格名 [--fingers=-y|+y|+x] [--offset-xy dx dy]` 生成位姿块，再填本轮图像依据。`round build` 会核对夹持点在所写格子交叉点 12 mm 内，TCP 直接放在交叉点上会被拒绝；`bash r1d pose check --spec round_spec.json` 可单独核对。
- **预检**用解析雅可比阻尼最小二乘 IK，完整走子预检通常几十秒到两分钟。明显超过几分钟属于异常，先查服务日志，不要重发或重建计划。
- **预检以实时位置为准，只检查执行手**：`prepare_transaction` 开始时读实时关节，非执行手的航点替换为它的实时位姿（只保持），执行手第一个抬升航点改用实时 x/y/rpy。拍图后手臂下沉几毫米不需要重拍。非执行手的碰触不会被拦截。

## 每轮步骤

0. **故障或回位**：执行中手臂因错误停住，或用户要求回位时，直接 `bash r1d retract`（倒序重放已执行路径，不预检；回到运输净空高度时同步闭爪）。`retract` 不能代替门禁审批或成功路径；不要用 `client set_gripper` 手动张爪绕过事务。结束任务时按 Ctrl-C 停止服务，它会自动卸力（手臂会下垂）。
0. **setup 检查**：本机第一次走子之前，运行 `python scripts/setup_status.py --require`。有必做项仍是参考值时不要启用运动，先按 `setup/README.zh.md` 完成（本机环境、相机检查、机械臂精度标定、头部相机标定、下棋环境参数、棋盘标定），或由用户明确决定沿用并记录依据。
0. **机器人重启后、接管之前**：升降、头、腰、双臂、夹爪不在初始位姿时，先 `bash r1d goinit` 预览，交给用户确认安全后才 `bash r1d goinit --execute`（不做任何预检，不得自行执行）；`--status` 中 `ready_for_chess=true` 后再启动服务并用 `handover --execute` 接管。下棋中回 init 用 `prepare_init`/`execute_init`。
0. **任务开始先启动象棋服务并接管机器人**（操作手册“开始下棋任务”）：
   - `systemctl --user start r1d-runtime.service` 启动 Runtime（机器人运行时，`third_party/robot_runtime`；单元由 `python scripts/install_runtime_service.py --write` 生成），`bash r1d serve --allow-motion --arm 所选手` 后台常驻，再 `handover --execute`（有保持单元时）或 `client enable_control`。
   - 之后只通过这个服务与机器人通信：init、走子、吃子都用 `client prepare_*`/`execute_*`，检查点用 `serial next` 和 `gate`，不直接调用 Runtime 或其他运动脚本。
   - `get_state` 确认头已到参考位后，`client release_control` 关闭执行（会话保留：`control_enabled=false`、`head_waist_session_active=true`），再拍规划图和规划。`round build`、`set_active_arm`、`prepare_init` 都要求执行关闭。
   - **走完回到 init（或 `retract`）后服务自动卸力并结束会话，所以每一轮开始都要重新接管**：入口条件满足时 `client enable_control`，否则先 `goinit`（需用户确认安全）再 `handover --execute`；然后 `release_control`，再规划。
1. 核对 Runtime、外部保持单元、反馈新鲜且静止、空爪、头腰参考位（接管时头允许差 0.35 rad，会被斜坡转过去）和 profile 记录的升降高度。双臂 TCP 距各自 init ≤10 cm 不重复初始化，以本轮实测关节和 XYZ/RPY 起步；**只看 TCP 距离，关节构型与 init 不同不算问题**。卸力后超过 10 cm 先 `prepare_init`/`execute_init`。非执行臂保持当前位置。
2. 新 run 只采头部单眼 `cam_left_high` 规划图，确认完整棋局和对手实际应手；写 FEN、实际走子历史和图像证据。匹配本局预测，失配或耗尽再搜索。优先最快可靠的胜线，不假定对手照主变走。远端黑方半场看不清时，按 `board_position/prompts/board_view.zh.md` 用 `bash r1d boardview` 生成俯视图和逐格放大图来写 FEN，再用 `bash r1d fencheck` 画回图上逐格核对；这些图只用于读棋，不能用于估计 source/target 位姿。
3. 选手：source 在 a–d 列优先左手，e–i 列优先右手，允许横跨整盘。独立估计本轮该手 source/target 的 TCP XYZ/RPY，姿态分别优先横向，有干涉或不可达再试纵向。按完整分支预检的可达性决定候选：首选手失败再对另一手重估、build、预检，两手都失败才换着法。吃子先把对方棋子放进同侧圆盘，再移动己方棋子，整条 sequence 联合预检；目标姿态与源不同时 build 会安排在圆盘上方原地转向。
4. 执行前（执行状态关闭、本轮已重新接管）备齐服务 endpoint、Python、run/request/prepared/execute/result、各 stage 检查点路径。准备完定位、棋规、姿态和全分支预检后：先 `get_state` 确认 `state_age_s` 正常、等约 3 s，再 `client enable_control`，核对 `control_enabled=true`、无 fault，然后只提交一次 execute。execute 不限时等待，客户端返回前不发其他请求（不另发 `get_state`/`capture`/hold）。结束后执行自动关闭。
5. **门禁由你判断并立即提交，不逐个询问用户。** 一律用 `bash r1d gate --prepared ... --checkpoint ... --decision ... --reason "..." [--offset dx dy]`；每条门禁命令和每条 `serial next` 都单独发，路径写绝对路径。
   - 所有门禁（source 高位、闭爪前、修正后复拍、释放前）都不限时，服务在当前位置保持到收到决定为止。不限时不等于可以慢：每个门禁目标约 5 s，`next` 返回 `needs_review` 后只做“看两图 → 提交 → 立刻 `next`”，中间不读代码、不改代码、不查日志、不发 `get_state`/`capture`、不跑测试。真实耗时如实记录。
   - 每次只看本检查点的头图和执行手腕图。执行中的所有调整（XY 修正、减倾角、回高位、释放修正）都只解 IK 直接下发，不预检。
   - source 高位：棋子在两爪中间则 `success`，否则 `adjust`（最多三次）。
   - 闭爪前：棋子中心已过爪尖、大致在两爪之间就 `success` 闭爪，不因横向偏几毫米而回高位；棋子大部分在指尖外用 `adjust_pitch`（减小向下倾角 0.1 rad，TCP 位置不变，复拍，最多两次）；明显偏到会被单爪顶开才 `retry_high`。不用 `adjust_z`（默认关闭）。闭爪后判定空抓时服务自动升回高位、复拍并重新抓取，真实闭爪最多三次。
6. 抓成功后连续搬运，target 下降后张爪前复拍。**释放门禁必须用腕图独立核对落点，不能只看 FK**（FK 与棋盘几何同源，几何偏了 FK 看不出来）：沿手指方向找一个已知邻子或印刷交点，用棋子直径作局部比例尺，量被夹棋子中心到它的距离，与应有的格距比较；横向同理看两侧格线。任一方向偏差 >5 mm 就提交 `adjust`（偏差填基座系、米），保持 Z/RPY 做低位 XY 修正；最多三次，前两次复拍。最后一次 `adjust` 必须从当前两图确认修正后的预计落点可安全释放（`release_alignment_verified=true`，reason 写明依据），否则客户端和服务端都拒绝；额度用完不能代替确认。无法判断、有碰撞风险或用户终止时不得放行：释放前用 `failure` 停止并持棋；抓前明确终止用 `serial submit --failure` 回撤。
7. 释放门禁提交 `success` 后，服务自动连续完成：原位张爪 → 保持 0.5 s → 开爪竖直上撤至净空 → 开爪退 staging → 原位闭爪 → 高位回 init（最后关节等于 `init_joint_rad`）。中间没有门禁，也不需要再发命令；`next` 读到 `finished` 后再复盘。吃子两段连续衔接。执行和持棋期间不检索资料、不改代码、不跑测试、不做引擎搜索或 FK 报告。
8. 回到 init 后服务自动卸力（结果里的 `arm_unload`），下一轮需要重新接管。完成后只做两项：新头图判断成功或偏位；离线算抓成和低位释放前两个事件的三种 FK 误差，再合并成 `docs/第N轮_中文着法_北京时间.json` 和误差 JSONL。图像结论与 FK 误差并列，缺失数据如实标注。实机结果、仿真结果和准确落格不能混同。

复盘后只报告成功或偏位结论、FK 误差、文件路径，以及卸力是否成功。恢复 AI 或关闭服务需要另有明确要求并核实实际状态。

## 参考

以下来自一台参考机器人的实测，只说明可能遇到的情况，新机器人要自己确认：

- 低位手臂实际比命令高约 1 cm，闭爪前门禁常看到棋子大半在指尖外，用 `adjust_pitch` 处理。
- 只按 FK 放行释放门禁时，出现过约 2 cm 的落点偏差。
- 左手只有手指 yaw −0.8 能到达左侧圆盘（`pose --role tray --tray left --yaw=-0.8`）。
- 棋子直径 29.7 mm，相邻列 35.6 mm，相邻行 34.2 mm，相邻棋子边缘间隙约 6 mm。
- 空抓后重试上升时，手臂跟踪滞后，服务会等手臂到位（最多 2 s）再判断是否达到运输净空。
