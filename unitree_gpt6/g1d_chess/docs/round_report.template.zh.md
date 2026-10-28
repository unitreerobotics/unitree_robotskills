# 两项复盘字段提示

[English](round_report.template.md) | 中文

本页是 [操作手册第8节](operation_manual.zh.md#8-走棋完成后的复盘仅两项) 与 [round_plan第6步](../prompts/round_plan.zh.md) 的填写提示，不要求另写完整报告或增加第三项收尾。执行前备好game_id、我方轮次、中文着法、context及输出路径；执行期间不整理复盘。

## ① 新头图视觉判断

机械动作结束后只采一张新的cam_left_high，在本轮`tmp/visual_review.json`记录：

- 原图绝对路径、采图时间及本局归属。
- source是否腾空，目标棋子是否位于正确交点附近，偏向哪边，邻子是否移动或重叠。
- `move_success`为true/false/null；无法判断、手工复位、对手走棋或图像延迟等不确定性如实写明。

服务outcome与棋局验收分别陈述；placed不能代替视觉成功，小FK残差不能证明棋子中心对格。只有当前终图用于验收，不用旧图补造结果。

## ② 离线FK、累计日志与合并落盘

用`scripts/review_pose_audit.py`读取真实事务结果里的两份执行时原始快照：

| 时刻 | 事件stage | 计算 |
| --- | --- | --- |
| grasp_success | source_contact_success | 实际FK相对原始plan、最终生效目标、当时命令FK |
| release_start | target_low_release_approval | 同上，释放使用下降后低位目标 |

各记录ΔXYZ毫米、位置模长及姿态旋转角；缺失快照/计算失败写原因，不造数值。吃子分别计算两个真实事务，己方走棋报告为主，棋外移除为辅助，合并同一轮。旧高位释放事件不混入现行低位学习样本。

按 [误差闭环说明](xy_error_learning.zh.md) 用`scripts/xy_error_history.py finish`结合第①项和plan summary，一次完成：

- 承接previous_journal全部历史行，再向本轮`docs/execution_xy_errors.jsonl`追加一行，保存完整本轮误差及左右手source/target独立累计。
- 生成本轮`docs/第N轮_中文着法_YYYYMMDD_HHMMSS.json`；轮次是本局我方走棋计数，重试和吃子子事务不递增，时间取合并落盘时北京时间，同名不覆盖。
- 关联原图、真实结果、FK明细和JSONL路径；pose_review.json仅是中间明细，不替代最终命名复盘。

累计只用actual减最终生效目标的三维误差；同context、视觉成功、现行两事件、有限三分量且模长≤25mm的主事务样本可学习。同token不重复学习，失败/不明/棋外移除只保留证据。25mm是学习筛选，绝不是运动修正距离上限。下一轮基于新图独立估计XYZ，历史均值只参考，不自动加减坐标。

正常结束关闭手臂控制并保持腰锁；不因复盘而重建棋盘、搜索引擎、重启或释放服务。
