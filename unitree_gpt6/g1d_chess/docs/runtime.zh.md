# 运行接口与事务衔接

[English](runtime.md) | 中文

本页对应 [README](operation_manual.zh.md) 的服务、执行与异常处理，以及 [round_plan](../prompts/round_plan.zh.md) 第2–6步。本页说明包内文件和接口，不代表机器人当前状态或新的运动授权。操作规范与代码能力须区分，已知差异列在文末。

## 包根与环境

只使用本次部署的 `g1d_chess/`。入口与模块见 [脚本清单](round_script_inventory.zh.md)，发布文件清单是根目录`manifest.json`。服务脚本按自身位置定位包根，相机`imageclient.py`从本包cameras包加载；无需用户级skills、旧项目目录或Codex CLI。串行提交器导入`checkpoint_runner.py`的文件/schema函数，不启动其中的历史后台评估器。

按[环境安装说明](environment_setup.zh.md)安装`pyproject.toml` 的 `chess` 组及外部引擎，再按 [首次配置](first_game_setup.zh.md) 激活用户已安装依赖的Python环境，直接用`python scripts/…`调用入口。模型从包内`assets/models/g1/`由`g1_model.py`构建，不导入`unitree_deploy`；`unitree_sdk2py`和`pyffish`从目标Python环境导入。基础Python、原生库、网络和真实硬件由目标环境提供；引擎通过`CHESS_ENGINE`选择。网卡、相机服务器、端点和夹爪方向均显式传入，配置模板不会自动向启动器传参。物理左右腕映射现场确认并填写，按进程缓存；检查点读token绑定的相机选择。

事务`checkpoint_dir`和`image_dir`必须位于本包`runs/`内，使用展开后的当前绝对路径。token绑定准备它的进程缓存、起点、场景、执行手、相机和夹爪方向，一次消费；不能复制旧token、把普通prepared当完整恢复归档，或在prepare后重启再execute。`restore_cached_transaction`需要完整轨迹归档及SHA256和当前兼容条件，不能把单事务恢复当联合sequence恢复。

启动器启动前会清理匹配的旧服务进程，不局限同端点；`--status`与确需启动分开。执行中、持物或状态不明时不以重启恢复。普通结束关闭手臂控制、保留腰锁；腰锁与手臂控制是分别管理的状态。只有用户明确要求释放/结束时才按README依次处理`release_waist`和`shutdown`。

## 普通走子

`prepare_transaction --request`接收`{"transaction": TX}`，TX由本包对应手规划器生成，含`grasp_plan/success_plan/failure_plan/grasp_retry`及全部重试、未知接触和退出分支。完整plan预检通过后启用控制、只提交一次`execute_transaction`，保留长调用。

`decision_channel={"mode":"serial","require_worker":false}`。主会话在另一工具调用中按当前stage prepared执行`next → 看本次头图+物理执行手腕图 → submit → next`。规划只用一张头图；source高/低位和target下降后低位均用头图加执行手腕图判断。

source高位先判左右在两指相对纹理间、前后在前端可夹段，能夹即可。最多3次XY修正，前两次后复拍，第三次后直接下降，既有低位闭爪门禁保留。低位XY不足用开爪`retry_high`；XY已可夹、仅高度导致纹理接触不足才考虑自动5mm `adjust_z`，第一次复拍、第二次后直接闭爪。最多真实闭爪3次，重规划不重置。用户显式抓前终止用`submit --failure`沿已缓存失败路径回撤；硬件故障可能阻止回位，不能把普通对中不足改成终止。

抓成后连续搬运到target高位、下降到计划释放高度、保持夹持拍头+腕图。按本次腕图的预计底面中心、目标格线/邻子和下落空间判断；target需修正时保持当前实际Z/RPY直接低位XY移动，不抬升。最多3次，前两次复拍，第三次后直接原位张爪，无额外审批条件。source高位和target低位XY不设单次/累计距离上限，不进行修正段运动检查；执行前完整plan预检、视觉门禁、次数和实际硬件故障处理保留。这是当前操作要求，代码覆盖情况见文末。

普通释放后开爪竖直撤至净空、开爪退staging、原位闭空爪、高位回自身init。抓成到target之前不插固定staging等待，不拆成新的放置事务。`placed`只表示动作分支完成，不能代替最终落格验收。

单事务全部`open`共用`gripper_open_q`；不存在可随意添加的per-waypoint `release_q/gripper_q/no_gate`实现。开合时手臂保持实际反馈位置，开合路点XYZ/RPY不会同时驱动手臂；需要移动则在完整plan中单列`hold`移动路点。释放批准后的尾段用最新反馈估计开爪位姿。开爪q须按新设备与本轮准备，不把历史数值作默认。

source修正只改变指定source阶段，不把其偏移复制到target/init。`actual_relative_offset_xy_m`相对当前实测FK，提交器负责换算服务参考偏移；不能与累计参考量混加。target XYZ/RPY独立估计，换姿态后重新估计工具偏置并在净空处转姿。

## 连续吃子

棋谱明确两步：`被吃方 <target> → empty`，再`我方 <source> → <target>`。`empty`是移到执行手同侧圆盘的语义，不是API坐标。盘内XY按当轮图像估计，左右手当前释放Z约定均为.159m，见 [圆盘规则](capture_tray.zh.md)。

`prepare_sequence`接收`{"transactions":[REMOVE_TX,MOVE_TX]}`，当前严格两个事务，联合预检两段及其全部失败/重试分支，并预测第一段成功末态作为第二段起点。第一段不包含回init，第二段首点为`gripper=hold`的竖直保持/上撤，再高位到己方source；同一物理手贯穿两段，观察臂保持本轮位姿。

用联合token只发一次`execute_sequence`。第一段圆盘低位判断、原位张爪并核验开爪到位后，竖直上撤并直接沿已预检高位路径进入第二段source门禁；不回init、不重新enable、不重复open、不再发prepare或其他同步服务请求。第二段仍保留自己的source高/低位和target低位判断，整步完成后才回init。第一段失败、不确定或开爪未到位时不盲目继续。

`recheck_between_transactions=false`表示内部衔接不再入口预检；正常视觉检查点及实际硬件/健康故障处理仍存在。`collision_checked=false`不能解释为全工具/邻子三维碰撞已验证。

`round_tools tokens`预先生成两个stage prepared，各自交给serial工具。第一stage本地完成后继续第二stage，不单独执行子token，两套身份不能交叉。事后从sequence结果取两个真实事务分别离线计算FK，合并同一轮复盘，移除棋子的棋外误差不进入学习累计。

## 超时与异常语义

服务是单线程同步ZMQ REP，execute/等视觉决定时不能处理另一个REQ的`get_state/capture/hold/release`。执行期间只读取本地检查点及原长调用，不能以同步hold作为即时急停通道。

| 情况 | 处理含义 |
| --- | --- |
| 客户端`request_sent=false` | 请求未成功发送，不等于已经核验机器人状态。 |
| `request_sent=true`后响应超时或断链 | 服务可能仍在执行或等决定；不盲重发execute。 |
| source串行检查点 | 按检查点`deadline_unix_s/wait_indefinitely`解释；不能把配置秒数一律当自动回撤倒计时。 |
| target释放检查点 | 当前`release_alignment.py`设置有限deadline；看本次checkpoint，不套用source无限等待语义。超时不表示自动张爪。 |
| `next --seconds 45` | 本地轮询时长，有图即返回；上限55秒，不改变机器人端等待。 |
| submit等accepted回执超时 | 决策可能已生效，先next查本地回执，不重复提交。 |
| 抓前显式failure | 按对应已预检失败路径退出，不是任意轨迹阶段急停。低位先开爪竖直升高。 |
| 服务SIGINT | 控制启用时请求cancel/hold，控制关闭时退出；仅对已核实进程处理，不等于中断客户端。 |
| release_control / shutdown | 前者停本服务手臂发布，不自动回init或放物；后者在手臂控制或腰保持仍启用时拒绝。 |

异常可能保持持物及健康控制hold；不得猜测已回位、自动释放或用SIGKILL替代受控恢复。现场硬件处置不由同步服务提供。

## 实现核对边界

包内源码提供左右手、联合sequence、两图相机选择、低位释放判断和第三次修正放行结构。实际服务能力按其加载的代码版本核对，控制与故障状态以当前回执为准。

“XY修正不做运动检查”是本流程的操作规范，**不能宣称包内实现已经完全满足**：`release_alignment.py`的`_release_correction/_checked_release_local`仍调用工具包络校验、局部编译、关节段检查及IK端点误差检查，释放尾段还有回位桥接检查；source修正实现也须按`fast_correction.py`核对。若要执行时严格满足该要求，需要先完成对应实现适配和验证。也不能根据初次完整预检承诺执行过程中绝无硬件或运行拒绝。

坐标是所附模型的基座/EE定义；模型锁零腰与实际腰锁零是两件事，须真实核验。EE工具偏置见包内模型与`motion_harness.py`，不等于现场手眼结果。头图板面点、棋子顶面、腕图像素和机械臂坐标分别处理。

离线验证范围见 [validation](validation.zh.md)。完成后只执行新头图视觉判断，以及两个事件快照的离线三种FK误差/JSONL合并复盘；没有第三项收尾。
