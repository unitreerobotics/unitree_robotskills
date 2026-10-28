# 每轮走棋执行提示词

[English](round_plan.md) | 中文

**抓取门禁终止语义：** 普通对中不足需要复拍时，显式提交 `retry_high`；用户要求立即终止本轮或确认无法安全继续时，在当前抓前检查点用 `serial_checkpoint.py submit --prepared <prepared.json> --checkpoint <checkpoint.json> --failure` 直接提交，无需视觉评估文件。服务沿已缓存失败路径回 init，不再做高位复拍、抓取或释放。低位 `failure` 先保持开爪竖直升高，再连续回 init。硬件故障仍可中止运动，不能声称已回位；此时核对实时状态，不强制驱动。

全新G1D先完成 [首次开局配置](../docs/first_game_setup.zh.md)，按当前部署根的 [README](../docs/operation_manual.zh.md) 和本轮用户指令完成以下 plan。使用 README 指定的本次部署包根；历史棋局、坐标和 token 不可直接重放。从当前实测状态规划本轮走法，完整预检通过后执行；“只规划”时止于规划。

scene优先使用用户给定且适用的现场数值；即使用户未提供任何现场信息，也由模型按首次配置第5节自行估计全部scene字段，部分给定时补齐其余字段。模型结合可用观测、模型资料及合理假设，记录依据和不确定度后直接设置并核验，不以用户补充资料或额外确认为前置条件，估计值标注为估计。兼容的已设置scene直接复用；场景变化后重新评估和完整预检。

规则与代码能力的已知差异见[运行说明](../docs/runtime.zh.md#实现核对边界)，不能把文档要求当作已完成实现。

规划阶段按[单轮脚本清单](../docs/round_script_inventory.zh.md)备齐入口、运行依赖与按需工具；清单对应本提示词第1–6步，不将历史临时脚本当作通用自动入口。

**服务与腰锁准备（与操作手册 0.1.3一致）：** 在真实进程可见的环境核对单实例、工作ROOT、执行手入口，优先复用正确ROOT的健康空闲服务，串行检查 `ping`、`get_state`。按用户已授权要求保持腰部零位：若 `waist_hold_enabled=true`、`waist_zero_verified=true`、`waist_target=[0,0,0]` 且无故障，记录实际 `waist_q`，不重复归零；否则在服务空闲、双臂控制关闭、双爪空闭合且无故障时调用 `lock_waist_zero`，保存回执并重新读取状态核验。锁腰须在本轮规划和完整预检前完成；腰位变化后刷新观测和棋盘配准，旧缓存失效。普通回init和关闭手臂控制后保持腰锁；仅用户明确要求释放腰锁或结束服务时，按操作手册8.1确认手臂控制关闭且现场可释放后调用 `release_waist`，需要结束服务时再 `shutdown`。若任务只要求建立服务与锁腰，核验完成后等待命令。

提交 execute 后按第 3–5 步串行处理抓取、释放及回位检查点：`next → 看本次头图和执行手腕图 → 决策 → submit → next`。预检通过后仍须按本次图像完成复拍与修正门禁。

**执行前备齐提交包：** 在机械臂开始运动前确定本包的 Python、`robot_client.py`、`serial_checkpoint.py` 绝对路径，以及本轮 run、执行请求、各 stage 的 prepared 回执、执行结果和本地评价文件路径；独立工具进程不继承上次 shell 的变量。先完成 build、完整预检，再用 `round_tools.py tokens` 生成 `tmp/execute_request.json` 和 `tmp/stage_N_prepared.json`；不能套用旧文件。核对 JSON 可解析、token/stage/检查点目录相互匹配、所用服务接口与当前版本一致，并备好下表两种评价结构和 `next`/`submit` 命令。普通走子启用控制并提交**一次** `execute_transaction --request <本轮执行请求>`；吃子提交**一次** `execute_sequence`，提前备好两个 stage 的 prepared，不能分别执行子事务。execute 是长调用，保留其会话并在另一工具调用处理本地检查点；不得等 execute 返回才看图，也不得在长调用期间另发 `get_state`、`capture`、`hold` 等同步服务请求。

**视觉门禁提交内容（字段名必须完整，布尔值用 JSON true/false，XY 单位为米）：**

| 检查点 | 本地评价 JSON 必填字段 | 决策和提交前核对 |
| --- | --- | --- |
| source 高位及修正后高位 | `decision`, `actual_relative_offset_xy_m`, `reason`, `source_identified`, `descent_alignment_verified`, `evidence` | `success`/`adjust`/`failure`；放行时两个布尔值为 true、XY=`[0,0]`；修正时确认 source 身份，给相对当前实际 FK 的明确非零 XY，`descent_alignment_verified=false`。XY 不设单次或累计距离上限，不进行修正段运动检查，保留修正次数和视觉门禁要求。 |
| 抓前低位 `preclose` | 同上六字段 | `success`/`adjust_z`/`retry_high`/`failure`；低位不能提交 XY 修正。`adjust_z` 仅请求预检好的自动 5 mm 下调；`retry_high` 保持开爪、XY=`[0,0]` 返回高位；放行闭爪时两个布尔值为 true、XY=`[0,0]`。 |
| 释放前低位 `pre_release` | `decision`, `actual_relative_offset_xy_m`, `reason`, `target_identified`, `release_alignment_verified`, `obvious_large_offset`, `evidence` | `success`/`adjust`/`failure`；放行须目标已识别、落点可安全释放、`release_alignment_verified=true`、`obvious_large_offset=false`、XY=`[0,0]`；修正须目标已识别、`obvious_large_offset=true`，给相对当前低位实际 FK 的明确非零 XY。无单次或累计距离上限，不进行修正段运动检查。 |

每次 `reason` 写明本次图像支持的 source 身份或 target 落点、左右/前后或交点偏差、为何能下降/闭爪/释放或必须修正；`evidence` 填本检查点返回的头图和物理执行手腕图路径，不能留空、沿用模板示例或旧检查点。`failure`、`retry_high`、`adjust_z` 及其他非 XY 修正决定均用 `[0,0]`；不随意增加 schema 外字段或自选 `grasp_z_m`。提交前对照当前 `stage`、`corrections_remaining`；source 高位和 target 低位 XY 修正均不设单次或累计距离上限，不进行修正段运动检查，包括额外预检、IK可达性、路径、表面净空、关节限位及采样检查；执行前完整plan预检、视觉门禁、次数限制和实际硬件故障处理仍保留。此处为文档操作策略，不能据此声称运行服务已实现。`serial_checkpoint.py submit` 会从当前 checkpoint 自动绑定 token、worker、stage 等身份，评价 JSON 不手填这些字段。图片到位后只作视觉判断并填写已备好的结构；先激活用户已安装依赖的 Python 环境，将 `CHESS_PYTHON` 设为该解释器的绝对路径（例如 `CHESS_PYTHON="$(command -v python)"`），将 `CHESS_SERIAL` 设为本次部署包根中该脚本的绝对路径，`CHESS_PREPARED` 设为当前 stage 的 prepared 绝对路径，每次独立工具调用重新设置：

```bash
"$CHESS_PYTHON" "$CHESS_SERIAL" next --prepared "$CHESS_PREPARED" --seconds 45
```

看完本次两图并填写评价 JSON 后，`CHESS_CHECKPOINT` 取本次 `next` 返回的 `checkpoint_path`，`CHESS_EVALUATION` 指向该评价文件，再提交：

```bash
"$CHESS_PYTHON" "$CHESS_SERIAL" submit --prepared "$CHESS_PREPARED" \
  --checkpoint "$CHESS_CHECKPOINT" --evaluation "$CHESS_EVALUATION"
```

确认 `accepted=true` 再 `next`；提交超时先用 `next` 核对回执，不重复提交。source 高位/低位和 target 低位是正常等待视觉判断的门禁；抓成后的搬运、吃子两段衔接、张爪后上撤和回 init 由已提交的事务连续执行，不插入代码搜索、临时脚本修补、额外服务请求或人工等待。硬件/控制故障按现有保护处理，不为追求连续性强行运动。

**开合路点：** 开合时执行臂保持当前实测位置，路点中的 XYZ/RPY 不会同时驱动手臂。需要先移动时，在 plan 中单列 `hold` 移动路点，到位后再开合；它与其余分支一起走原有完整预检，不增加执行中的额外预检路径。释放审批后的开爪尾段按最新反馈估计开爪位姿。

规划时按 [board_position说明](../board_position/README.zh.md) 与 [新模型标记任务](../board_position/prompts/mark_board.zh.md)，从当前left_high建立并视觉复核90点，核对source/target格名（a–i左→右，0–9近→远）。目录不附真实历史坐标或图像。棋盘/相机位置变化时重新定位，不能套旧像素或旧棋子占位，不能把板面交点当棋子顶面或机械臂XY。资料及复核在execute前准备；结束后仅在第6项视觉判断内辅助验格、验偏位。

规划阶段使用 `scripts/mark_board.py init/build/query`，输入当前原图和实际可见交点，生成本次标注PNG、90点像素JSON及CSV；拟合点与留出点分开，数值通过后仍须视觉验线。释放检查点只读取预先完成的当前头图标记辅助辨认目标格及邻子，仍以本次执行手腕图里的格线、预计棋子底面中心和邻子间隔判定落点；头图与腕图不是同一投影，不把头图像素偏差直接换算成低位基座XY。机位、腰或棋盘变化后在下一轮规划阶段重新标记，门禁等待中不运行配准或检索资料。

**执行 plan 期间不可以做任何无关步骤。需要复拍的检查点到位即复拍，图片就绪即判断、即提交、即继续：`next → 看本次两图 → 决策 → submit → next`。尤其高位复拍修正须从刚采集的两图直接预测 XY，不在门禁等待时检索历史资料或其他信息。不要让机械臂持棋等待资料检索、代码阅读/修改、测试、额外模型调用、长篇解释、报告整理或 FK 复盘。所需脚本、模板和参数在 execute 前全部准备好。**

`board_position`仅输出板面像素几何，不提供分手source/target末端XY先验。两者仍结合当前图像、执行手与实际姿态独立估计；不得将历史deltaXYZ均值、其他姿态或source修正直接叠加到target。

第4项的交点判据用于棋盘内落子；吃子移除阶段将执行手同侧圆盘作为target：物理左手放左盘、右手放右盘。按本轮图像估计盘内可用位置XY，左右手释放Z均固定为0.159m；预计棋子整体落入盘内、避开盘沿和已放棋子即可释放，不要求对准盘中心，不套棋盘交点判据。`capture.offboard_target`填写`kind=round_tray`及与执行手一致的`physical_side`，圆盘`z_m=0.159`；完整规则见[圆盘吃子规则](../docs/capture_tray.zh.md)。详细判定表见[释放落点判定](../docs/release_alignment.zh.md)。

1. **起点与选棋：** 每轮读取双臂实测位姿，分别计算末端XYZ到自身init的三维距离；≤0.10m（含10cm）不重新init，只有超出范围的手臂才需初始化。直接从所选执行臂的当前实测关节及XYZ/RPY规划，非执行臂保持本轮当前位姿；左右手均不使用camera pose，完整预检及走棋后的回init尾段仍保留。只采一张新的 `cam_left_high` 规划走法，结合当前状态确认棋局。先读取本局`move_forecast.json`，匹配黑方实际应手及预期局面：一致且还有下一步红方着法才作为候选；无预测、不一致或用完则从完整合法着法搜索并保存新的未来5轮红黑着法。预测落盘和进度确认只在规划阶段做。优先按source选手：a–d列左手，e–i列右手（e归右）。**允许横跨整盘，包括右手到a–b及左手到h–i；先完整预检原执行手可达性，不可达再用另一手从本轮实测起点和独立估计的抓放位姿完整预检。两手均不可达才放弃该着法，搜索下一合法候选。** 吃子须预检整条联合sequence；单个点IK成功不代表整步可执行。`scripts/move_constraints.py`只列棋规合法候选，不再按横跨幅度过滤；详见`docs/move_constraints.md`。
2. **完整规划与预检：** 规划前读取[邻子间隙与准确落子](../docs/neighbor_clearance_and_landing.zh.md)。选定候选手后，在估计前运行`scripts/xy_error_history.py show --run <本轮run> --hand <left或right> --context-id <当前兼容配置> --previous-journal <上一轮run>/docs/execution_xy_errors.jsonl`，读取该手历史累计 ΔXYZ 均值；换手时重新读取另一手，不共用误差或坐标。**source、target XYZ/RPY 分别由本轮图像独立估计，历史均值仅作辅助，不直接加减。** 红方和黑方半场的source、target均分别优先尝试偏横向夹爪姿态；横向抓取可行不要求横向释放。若某阶段在该姿态不可达，或有明确推挤、夹入邻子风险且无法在横向候选消除，才将该阶段改为偏纵向；换释放 RPY 后按新姿态重新估计 target EE XYZ。检查整片夹爪在下降、闭合、张开、上撤时与邻子的关系，减少明显干涉，不要求完美零接近，也不凭二维投影重叠或遮挡判定碰撞；转姿在净空高位完成，不沿用不同RPY的旧EE XY。每次换手或姿态后重新build、完整预检抓取、重试、持物转姿及搬运、释放、上撤、回init和失败分支；仅预检成功的候选可执行。填写spec的误差context、参考说明、前轮日志及本局轮次着法；吃子提前联合规划为一次sequence。通过后只提交一次execute。
3. **抓取：** source 高位、低位只用本次头部 `cam_left_high` 与物理执行手腕图判断；按相机策略读取，不按流名猜物理手。高位看两个方向：左右使棋子位于两片夹爪相对的纹理接触面之间，前后使棋子落在夹爪前端的纹理可夹持段内、没有退到夹爪后方。若任一方向尚不足以夹住棋子，就根据当前两图合并估计左右和前后偏差，给出争取一次对中的明确 XY 向量并立即复拍；不分方向试探性移动，也不要求像素级居中。单次 source 高位 XY 不设距离上限。复拍后若两方向均足以使纹理面夹住棋子侧壁，且下降扫掠没有明显推挤或夹入邻子的风险，就立即下放；仍不满足时才使用剩余修正次数。source 高位 XY 最多 3 次：前两次修正后立即复拍；第三次修正完成后直接放行下降抓取，不再追加高位复拍或视觉审批，不因普通对中不足走失败回撤。下降后的既有低位闭爪门禁仍保留。遮挡或投影接近本身不拒抓；需要换 RPY 时保持高位处理，低位不转腕。

   下放后在低位新图检查左右/前后对中、棋子侧壁与两指有纹理夹持面的实际重叠，以及是否压顶、推移或碰到邻子。容易抓稳的闭爪放行判据是：目标棋子位于两指之间，两侧侧壁分别进入相对的纹理有效夹持面，指尖能够包住侧壁而非压在顶面，且没有推移棋子或夹入邻子；仅落在张口投影内不足以放行。若超过半枚棋子位于夹爪纹理接触区域之外，而 XY 已可夹持、只是高度造成有效接触面积不足，考虑 `adjust_z` 自动竖直下调 5 mm 增加抓取面积；下调一次后立即复拍，若双侧有效接触仍仅受高度限制且预检允许第二次下调，才按现有流程再下调并直接闭爪，不增加第三次低位门禁。若左右或前后偏位使侧壁未进入两侧有效夹持面，尤其棋子明显在夹爪偏后，不能用降 Z 代替修正，应保持开爪以 `decision=retry_high`、XY=[0,0] 竖直回高位复拍。真实闭爪空抓同样回高位复拍，不直接 failure 回 init；最多真实闭爪 3 次，跨重规划累计不重置。高位修正只能依据当前两图给相对实测 FK 的明确 XY，不检索历史图或资料；完整plan预检及普通执行路径的既有净空保护保留；source高位/target低位XY修正段不进行运动检查，实际硬件或控制故障仍按原流程处理。抓取成功时异步记录原始快照，随即搬运，不计算 FK 误差。
4. **下降后、张爪前复拍并立即决策：** 搬运到target高位后按计划下降至释放高度（棋盘内左右手均Z=.139m；吃子移入圆盘时均Z=.159m），保持夹持，采新的头图和执行手腕图。棋盘内落子以本次腕图为主、头图辅助核对格名：先辨认目标周围可见棋子及其所在交点，再沿画面中两组棋盘线和相邻交点延伸、相交或插值，定位可能被夹爪或棋子遮住的目标交点；再用可见棋子轮廓估计所夹棋子的中心及张爪后底面预计中心，考虑夹持偏心和顶面/板面视差，比较它与目标交点的左右、前后偏向和邻子间隔。不能用夹爪外壳、影子、单条格线或棋子边缘代替落点，也不能仅凭 FK 误差小放行。只有预计中心靠近目标交点、整枚棋子可干净下落且不与邻子重叠、张爪和上撤无明显干涉时，才提交 `success`、`obvious_large_offset=false` 原位张爪；小幅中心偏差无需反复调整。仅边缘覆盖交点、预计中心明显偏开、落在相邻交点之间或方格中间时，提交 `adjust`、`obvious_large_offset=true`；目标交点被遮住时仍利用可见格线和邻子推断，不因遮挡本身修正，也不设未经标定的毫米/像素阈值。reason/evidence 记录腕图路径、用于定位的可见格线/棋子、预计中心相对交点的偏向及最近邻子关系。XY 相对当前低位实际FK，保持当前Z/RPY直接低位调整，不抬升；此修正段豁免低位横移禁令，不设单次或累计距离上限，不进行修正段运动检查；保留视觉判断、次数限制和实际硬件故障处理。target 独立最多3次：前两次后立即在释放高度复拍；第三次修正完成后直接放行原位张爪，不再追加复拍、视觉审批或“已有充分安全依据”的放行前提。实际硬件或控制故障仍按原流程处理。真正无法判断预计落点或存在碰撞风险时，提交 `adjust`、`obvious_large_offset=true`、`release_alignment_verified=false`，说明不确定处或风险；仍须识别目标、给出有依据的明确非零XY后直接执行修正，不进行修正段运动检查，不能虚构识别或位移。放行时记录第二份原始快照，冻结回位尾段。
5. **连续回位与吃子衔接：** 普通放置：下降 → 低位两图判断/必要的直接XY微调 → 原位张爪 → 保持开爪竖直上撤至净空 → 开爪退 staging → 原位闭爪 → 高位回自身 init。吃子移除阶段确认可释放圆盘内被吃子后，原位张爪并由已有反馈核验开爪到位，随即竖直上撤、经预检高位路径直接到己方source高位复拍；staging不是独立等待点，第二段不重复开爪、重新enable、回init或另发服务请求。只有整步完成后回init；若开爪未到位或硬件/运动保护拒绝，则停止衔接并核实状态。执行中只处理本地检查点及原长调用结果；超时先查本地回执，不重复 execute 或提交决策。
6. **完成后只做两件事：** ① 采一张新的 `left_high`，判断走棋成功与否及位置偏差，准备`tmp/visual_review.json`；② 离线计算“抓取成功”和“下降后低位批准张爪释放”两时刻实际 FK 相对原始 plan、最终生效目标、当时命令 FK 的三种误差，再用`scripts/xy_error_history.py finish`在本轮`<本轮run>/docs/execution_xy_errors.jsonl`承接plan summary中`previous_journal`的全部历史行，再追加本轮一行（完整误差及左右手分别累计的source/target `count`、`sum_delta_xyz_mm`、`mean_delta_xyz_mm`），同时与图像判断合并保存到本轮 `docs/{第几轮}_{这一轮的走棋子方案}_{时间戳}.json`。日志与本轮最终复盘JSON同目录，旧日志不变，新日志供下一轮继续累计。累计只用actual减最终生效目标的完整 ΔXYZ；视觉成功/现行事件/三个分量有限且三维模长≤25mm样本可学习，旧高位、失败、缺失、吃子棋外移除仅保留证据不计入；同token不重复学习。JSONL维护属于第②项，执行门禁不读写它。命名如`第N轮_中文着法_YYYYMMDD_HHMMSS.json`：轮次为本局我方第N次行棋（新局从1开始，重试/吃子子事务不递增），方案取最终中文棋谱着法，时间戳取合并落盘时北京时间`YYYYMMDD_HHMMSS`，同名不得覆盖。规划时备好本局/轮次/着法/context，合并时创建`docs/`并记录`round_number`、`move_plan`、`review_created_at`；详细规则见操作手册第8节与`docs/xy_error_learning.md`，`pose_review.json`仅作中间明细。缺失数据如实标注。最终JSON和日志落盘即结束，只回复成功/偏位结论、FK 误差简述和文件路径，不追加其他收尾工作。
