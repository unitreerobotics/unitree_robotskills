# 文档分工与阅读顺序

[English](README.md) | 中文

本目录是 [仓库入口](../README.zh.md)、[操作手册](operation_manual.zh.md) 和 [每轮 round_plan](../prompts/round_plan.zh.md) 的专题补充。操作手册提供完整操作规范、命令与异常处理；round_plan把同一规范组织成每轮执行顺序。专题页解释字段、判断依据和接口边界，不另立一套流程。当前用户的明确指令优先；发现文档与运行代码能力不符时，先核对具体实现，不能仅凭文档声称服务已支持。

新用户先读 [首次开局配置](first_game_setup.zh.md) → [包使用与依赖](package_setup.zh.md) → 操作手册 → round_plan，执行前按 [脚本清单](round_script_inventory.zh.md) 备齐材料。项目文档、脚本和模型从本次部署的 `g1d_chess/` 内取得；Python依赖按根目录 pyproject.toml 的 chess 组安装，引擎单独安装。目标机仍需兼容的Python和原生库，以及实际网络、相机和机器人；不需要原工作目录或用户级skills目录。

每轮新数据统一放在包根`runs/<本轮名称>/`，由初始化或启动过程按需创建；发布包不预置运行记录。

## 专题对应

| 文档 | 对应主规范 | 用途 |
| --- | --- | --- |
| [first_game_setup](first_game_setup.zh.md)、[package_setup](package_setup.zh.md) | 操作手册 0.1、第4节；round_plan服务准备 | 新设备配置、包内路径、依赖和版本 |
| [runtime](runtime.zh.md) | 操作手册 0.1、0.5、第5–9节 | 事务、检查点、超时与控制状态语义 |
| [round_script_inventory](round_script_inventory.zh.md) / [JSON](round_script_inventory.json) | 操作手册第4–9节；round_plan第1–6步 | 所需脚本、运行模块及调用顺序 |
| [move_constraints](move_constraints.zh.md)、[move_forecast](move_forecast.zh.md) | 操作手册 0.2；round_plan第1–2步 | 全合法着法、换手可达性、五轮预测 |
| [neighbor_clearance_and_landing](neighbor_clearance_and_landing.zh.md)、[tip_grasp_policy](tip_grasp_policy.zh.md) | 操作手册 0.3–0.5；round_plan第2–3步 | 独立抓放估计、邻子关系、能夹即可 |
| [release_alignment](release_alignment.zh.md)、[capture_tray](capture_tray.zh.md) | 操作手册 0.5.2；round_plan第4–5步 | 低位落点、同侧圆盘、连续吃子 |
| [xy_error_learning](xy_error_learning.zh.md) | 操作手册 0.4.1、第8节；round_plan第2、6步 | 两事件FK误差、同手context、JSONL与合并复盘 |
| [experience](experience.zh.md)、[round_report.template](round_report.template.zh.md) | 操作手册第8节；round_plan第6步 | 经验边界与两项复盘字段提示，非额外报告任务 |
| [validation](validation.zh.md) / [记录](package_validation.json) | README包核验入口 | 离线核验事实与尚未验证的现场条件 |

棋盘点位资料统一在 [board_position](../board_position/README.zh.md)，仅含通用说明和脚本；由新图生成本次90点，不提供历史实测数据或机器人XY。格名与走法写法见 [统一规范](../prompts/board_and_move_conventions.zh.md)。

## 关键执行规则

- 规划仅一张新头图；source高/低位与target低位均为本次头图加物理执行手腕图。相机映射现场填写，不按流名猜测。
- 实际腰锁零并核验；双臂各自距init三维≤10cm不重复初始化；非执行臂保持当前。正常结束保持腰锁。
- source与target XYZ/RPY按本轮图像分别估计，历史误差仅参考；全合法着法可跨盘，首选手不可达再试另一手。完整plan及吃子两段在execute前联合预检。
- source高位最多3次XY修正，前两次复拍，第三次直接放行下降；原有低位闭爪门禁保留。能夹即可，不要求完全居中。低位XY不足用retry_high；XY可夹且仅高度覆盖不足才考虑5mm adjust_z，第一次后复拍，第二次后直接闭爪。真实闭爪最多3次，跨重规划不重置。
- source高位、target低位XY修正无单次/累计距离上限，不进行修正段运动检查；完整plan预检、视觉门禁、次数和实际硬件故障处理保留。target前两次修正后复拍，第三次后直接原位张爪，不追加审批条件。
- 普通走子target下降后两图判断，再张爪、开爪竖直上撤、开爪退staging、原位闭爪、高位回init；吃子第一段圆盘释放并核验开爪后直接进入第二段source，不中间回init、再enable、再open或另启预检。
- execute期间只处理本地next/submit及原长调用，不同步查询服务或检索资料。结束只有新头图视觉判断和离线两事件三种FK误差/JSONL合并两项复盘。25mm是学习样本筛选阈值，不能用作修正距离上限。

操作规范与实际实现的差异见[runtime](runtime.zh.md#实现核对边界)；新设备须独立完成现场配置与验证。

环境安装方法、chess 组依赖、外部引擎及独立运行范围见[环境安装说明](environment_setup.zh.md)。
