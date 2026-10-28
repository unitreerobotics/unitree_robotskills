# 抓取与释放历史 ΔXYZ 误差的持续参考

[English](xy_error_learning.md) | 中文

每轮复盘将误差另存到一个持续追加的JSONL，按物理左手、右手及抓取/释放阶段分别累计；下一轮估计source/target时读取所选手的历史累计 ΔXYZ 均值。**source、target 的 XYZ 仍基于本轮采集图像估计，历史均值只作辅助参考，不直接加减坐标，不自动更新棋盘XY表，不自动生成补偿坐标。** 参考的是历史位移误差均值，不是`grasp_success`、`release_start`实际XYZ坐标的均值。最终候选结合本轮图像、抓放姿态、夹持位置及邻子独立判断。

## 文件与实现

命令在当前部署的g1d_chess根目录执行；新设备先按 [首次开局配置](first_game_setup.zh.md) 设置解释器与兼容context。

- 累计日志：`<本轮run>/docs/execution_xy_errors.jsonl`，与该轮最终复盘JSON同目录，文件名沿用`execution_xy_errors.jsonl`，内容为完整三维 ΔXYZ。新run通过`--previous-journal`承接上一份累计日志，finish先原样复制其历史行，再追加本轮一行；旧日志不变。每轮恰好一行，含本轮完整误差及更新后的`cumulative`。新一份日志包含承接的全部历史，可跨兼容棋局继续使用。
- 离线FK计算：`scripts/review_pose_audit.py`，从真实事务结果提取两事件，不连接硬件。输出`docs/pose_review.json`是中间明细。
- 追加、累计及最终合并：`scripts/xy_error_history.py finish`，属于完成后第二项复盘的一部分，同时生成按轮次/着法/时间戳命名的最终JSON。JSONL用文件锁串行追加并同步落盘；同一组token重复运行且输入相同不重复计数，输入冲突报错。同一`game_id`及`round_number`不能换token或context重复计入。文件末尾出现残缺JSON行时报错，不静默忽略。
- 规划参考：`scripts/xy_error_history.py show`；左右手规划器在build中再次读同一日志，将所选物理手参考保存到`tmp/plan_summary.json.xy_error_reference`，不改动任何XYZ。存在合格历史样本时，spec必须填写`xy_error_reference_review`，否则拒绝build。保留已有`xy_error_*`脚本及配置字段名，其统计内容均为三维 ΔXYZ。
- 配置：`config/xy_error_learning.json.context_id`，模板为null，不提供默认历史组；必须填写当前兼容标识，或显式传spec/CLI context。棋盘/机身/腰姿、工具、模型或抓放姿态组发生不兼容变化时使用新context（改配置或spec的`xy_error_context`）；旧数据保留，但统计只合并同context。新棋局若条件兼容无需清空经验；统计可跨局，轮次仍在新局从1开始。

本包提供[离线工具](../scripts/xy_error_history.py)、[配置](../config/xy_error_learning.json)、左右手规划参考接入及离线测试。全部从本次部署的包根运行；日志由`--run`定位在该轮docs内，不要误选其他棋局分支的历史文件。

## 计算口径

每个合格事件的三维误差定义为：

```text
delta_xyz_mm = 1000 × (actual_fk.xyz − effective_target.xyz)
count := count + 1
sum_delta_xyz_mm := sum_delta_xyz_mm + delta_xyz_mm
mean_delta_xyz_mm := sum_delta_xyz_mm / count
```

基座系x向前、y向机器人左侧、z向上，正号表示实际末端在该方向偏于最终生效目标；左手不能镜像。`cumulative.left.source/target`和`cumulative.right.source/target`各存`count`、`sum_delta_xyz_mm`及`mean_delta_xyz_mm`，向量顺序为`[ΔX, ΔY, ΔZ]`。**用历史累计位移误差均值描述偏差；sum只是可复算统计量，均值与累计和均不能自动加减到下一轮坐标。** source只累计`grasp_success`，target只累计低位`release_start`，两者不串用，不从另一手借样本。没有样本时count=0、mean=[0,0,0]表示无数据，不表示误差为零。

本轮原始计划、最终生效目标、命令FK三种误差以及actual/effective/command位姿均完整保存在`main_report.records`；学习使用完整`vs_effective_target.delta_xyz_mm`三分量，不平均actual或effective坐标，不把source视觉在线修正量累加，也不以批准释放的同帧实际位姿`placement_target`作误差参考。日志保留图像判断、实际结果文件路径、规划参考及本局/轮次/着法；缺失快照仍记本轮行，缺失阶段不增加count。

合格条件：主走棋事务`outcome=placed`、最终视觉`move_success=true`，事件分别为`source_contact_success`和`target_low_release_approval`，ΔXYZ三个分量均有限且三维模长≤25mm。失败、无法视觉确认、旧高位释放、缺失误差、超限异常样本保留原证据和排除原因，不进入均值。吃子移除/棋外放置事务放入`auxiliary_reports`，保存误差但不计入source/target累计。只有己方走棋主事务可学习，一次吃子仍只追加一行。

FK是末端跟踪估计，不是棋子中心误差；抓取点、夹持偏心、转姿、释放及撤爪可能另外造成落点偏差。累计值不能证明首抓成功率已经提高，仍须门禁和最终视觉验证。参考中提供每阶段最近合格事件的实际/生效目标位姿，仅用于追溯和核对可比性，不能作为坐标均值取代历史 ΔXYZ 均值；不可比时更换context，并在本轮估计说明中记录。

## 规划时读取

按source选物理手之后、估计source/target之前，即使棋规着法复用预测，也必须读取对应手的最新参考。例如左手：

```bash
"$CHESS_PYTHON" scripts/xy_error_history.py show \
  --run "$CHESS_RUN" --hand left --context-id "$CHESS_XY_CONTEXT" \
  --previous-journal "$CHESS_PREVIOUS_ERROR_JOURNAL" \
  --output "$CHESS_RUN/tmp/xy_error_reference.json"
```

右手改为`--hand right`。`CHESS_XY_CONTEXT`取当前配置/本轮spec明确核对的context，不能照抄旧局。只读操作不连接机器人、不追加日志。首次无日志时返回空统计；日志损坏则报错并修复，不能仅换context或把损坏伪装成无样本。查看两阶段`count`和`mean_delta_xyz_mm`的方向/量级，依据本轮唯一规划头图、执行手实际姿态及RPY/Z分别估计source、target的XYZ；board_position只给当前图的板面像素几何，不提供机器人XY先验；`latest_eligible_event`仅辅助核对条件，不替代累计误差均值，不使用`source_xyz -= mean`或`target_xyz -= mean`的机械规则。

`CHESS_PREVIOUS_ERROR_JOURNAL`指定上一已完成轮次的`docs/execution_xy_errors.jsonl`绝对路径；首次确无历史时省略整项`--previous-journal`。不要因本轮目录尚无日志而忽略上一轮历史。指定的历史文件缺失或损坏会报错，不静默清空累计。

在spec填写`game_id`、`round_number`、`move_plan`、`xy_error_context`、`xy_error_reference_review`及`xy_error_previous_journal`（与show相同的上一份累计日志绝对路径；首次无历史时为null）。`xy_error_reference_review`说明累计值的方向/量级、样本可比性、如何与当前视觉结合、最终候选的依据；即使决定不调整候选也写明依据。build记录当前日志路径、承接历史路径和估计说明，完整预检仍针对最终冻结候选。自定义规划器须显式做同样读取和保存，不能绕过参考步骤。全部在execute前完成，持棋门禁不读写JSONL。

## 复盘第2项落盘

先完成第1项新left_high视觉判断，在`tmp/visual_review.json`准备结构化结果，至少包含`move_success`（true/false/null）、原图绝对路径、采图时间、偏位描述及不确定性。用真实事务结果离线计算FK，随后一次finish完成JSONL追加和最终JSON合并：

```bash
"$CHESS_PYTHON" scripts/review_pose_audit.py \
  --result "$CHESS_RESULT" --output "$CHESS_RUN/docs/pose_review.json"

"$CHESS_PYTHON" scripts/xy_error_history.py finish \
  --run "$CHESS_RUN" --game-id "$CHESS_GAME_ID" \
  --round-number "$CHESS_ROUND_NUMBER" --move-plan "$CHESS_MOVE_PLAN" \
  --context-id "$CHESS_XY_CONTEXT" \
  --report "$CHESS_RUN/docs/pose_review.json" \
  --visual-review "$CHESS_RUN/tmp/visual_review.json" \
  --plan-summary "$CHESS_RUN/tmp/plan_summary.json"
```

`finish`从`--plan-summary`读取规划时保存的`previous_journal`，首次写入本轮日志时承接全部历史再追加本轮；不用在finish命令中重复指定。无plan summary的离线合并必须显式传`--previous-journal`才能承接历史。当前日志与最终复盘固定在同一`run/docs/`，不写回其他资料目录。

吃子：两个真实事务各计算一份FK，`--report`首先传己方主走棋报告，再重复该参数传移除事务报告；不能直接传sequence包装结果。两个事务中任一个未完成时不进行整轮成功复盘。`finish`输出最终复盘路径及最新累计统计，落盘后结束；它不采图、不调用服务、不搜索引擎。重试同一个finish时只恢复同一命名文件，不新增累计样本；不能换token或轮次来规避重复检查。

独立复盘继续用 `docs/第N轮_中文着法_YYYYMMDD_HHMMSS.json`，JSONL路径和该轮累计值关联在`xy_error_history`字段。两种文件同属复盘第2项，未增加第三项收尾。历史误差仅在确认事件、真实结果、视觉成功及context兼容后显式导入，禁止批量猜测旧缺失信息。

同一棋局轮次失败后可重试：已有记录须明确视觉失败、未placed且无可学习样本，方可用新token追加同轮记录；保留失败证据和原轮次。已有成功/不明结果或token冲突仍拒绝重复记账。
