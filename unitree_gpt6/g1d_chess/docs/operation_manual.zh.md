# 机器人象棋单轮操作手册

[English](operation_manual.md) | 中文

**抓取门禁终止语义：** 需要继续抓取并复拍时用 `retry_high`；用户要求终止或无法安全继续时，在当前抓前检查点用 `serial_checkpoint.py submit --prepared <prepared.json> --checkpoint <checkpoint.json> --failure` 直接提交，无需视觉评估文件。服务直接走已缓存回 init 路径，不再启动复拍。低位先开爪竖直升高，再连续回位。故障保护仍可停止运动，必须核实实际状态，不能把接受 `failure` 等同已回 init。旧服务进程不会自动加载修复，重建服务后须用新 token。

**抓取重试流程：** 不满足抓取条件或空抓时，先回高位采头图+执行手腕图、修正 XY 并复拍，再下降；低位不得用降 Z 解决 XY。最多三次高位 XY 修正，前两次后复拍，第三次完成后直接放行下降抓取，不再追加高位复拍或视觉审批；下降后的既有低位闭爪门禁仍保留，仍为空抓才回 init，不把普通对中不足直接判整轮 failure。抓取 RPY 减少明显邻子干涉，不要求完美零接近。此条优先于下文旧失败回撤描述，完整入口与六字段 `retry_high` 用法见[本轮提示词](../prompts/round_plan.zh.md)。包内代码已更新新事务策略，已有服务进程不会自动加载磁盘修改，下一次执行前在空闲状态加载新版并生成新 token；不得复用旧 token。

**邻子与落点：** 新局按[邻子间隙与准确落子](neighbor_clearance_and_landing.zh.md)执行。source/target 分别选择 RPY，检查整片夹爪下降、闭合、张开和上撤的邻子扫掠；抓取不能只确认两指对中。释放同时确认预计落点在目标交点附近、整枚棋子能独立下落且不与邻子重叠。不得以历史“略偏心”、服务 placed 或小 FK 残差放行。规则已写入提示词，改进效果待新局实机验证。

**现行选招与姿态规则：** 允许任一手横跨棋盘。按source列选首选手（a–d左、e–i右），对最佳棋规着法先以该手的当前实测起点及本轮图像估计抓放位姿做完整事务预检；不可达则换另一手重新估计、build并预检。两手均不可达才放弃该着法，搜索下一合法候选。两半场的source和target均先考虑偏横向夹爪姿态；某阶段横向不可达，或有明确推挤、夹入邻子风险且无法在横向候选消除时，该阶段改试偏纵向，并按新RPY重新估计EE XYZ、复核邻子和完整路径。单点IK、历史坐标或旧预测均不能代替当前全路径预检。执行细则见[本轮提示词](../prompts/round_plan.zh.md)与[选招与机器人可达性](move_constraints.zh.md)。

适用机器人：G1、Dex1 内置夹爪。本文是单轮走棋统一入口，支持物理左手或右手执行，另一只手保持本轮确认的当前位姿。所有现行入口均在本次部署的g1d_chess包内；规则与运行实现的已知差异见[运行说明](runtime.zh.md#实现核对边界)，文档整理不等于实机验证。

以后可直接向助手提供：

每轮可直接附加精简版提示词：[prompts/round_plan.zh.md](../prompts/round_plan.zh.md)，包含完整plan及执行期间“立即复拍、立即决策，不做无关步骤”的约束。

**历史 ΔXYZ 误差参考闭环：** 每轮复盘第2项另向本轮`<本轮run>/docs/execution_xy_errors.jsonl`追加一行，保存本轮两事件误差及左右手独立累计；下一轮按source选手后、估计source/target前读取对应手的历史累计 ΔXYZ 均值。source、target 的 XYZ 仍基于本轮图像估计，历史均值仅作辅助参考，不直接加减坐标、不平均抓取/释放实际XYZ、不自动改坐标表。实现及命令见[历史 ΔXYZ 误差参考](xy_error_learning.zh.md)、0.4.1和第8节。

> 读取 `<部署后的g1d_chess根目录>/SKILL.zh.md`，按本文与当前用户指令完成下一轮象棋。先按0.1.3建立或复用服务并核验腰部锁零；正常走棋结束保持腰锁。每轮先核对双臂实测位置到各自init的距离，均≤10cm则不重新init，直接从当前位姿规划，再仅采cam_left_high确认当前棋局，并读取机器人状态；先按0.2匹配本局五轮预测，黑方应手及局面一致且着法合法才作为候选，无预测、不一致或用完则重新搜索。允许右手到a–b、左手到h–i等跨侧边缘走法；优先按source选手，完整预检不可达再换另一手，两手均不可达才放弃该候选。若本轮已授权执行，则在完整规划和预检通过后执行。抓取成功后搬运至target高位并下降至释放高度，再于张爪前复拍；执行手腕图显示预计落点在目标交点附近则释放；预计落在相邻交点之间、方格中间或明显错开目标交点时保持实际Z/RPY直接低位XY微调（不抬升，不设单次或累计距离上限，不进行修正段运动检查）；最多三次，前两次后复拍，第三次后直接张爪。随后执行臂连续上撤、回自身init pose；非执行臂全程保持本轮确认的当前位姿，双臂等待下一轮，不使用camera pose。走棋完成后只做两项复盘：采一张新left_high判断走棋成功与否及位置偏差，再计算两个事件快照的三种FK误差，与图像判断合并落盘至本轮 `docs/{第几轮}_{这一轮的走棋子方案}_{时间戳}.json`，命名细则见第8节。

历史会话及参数不是新动作授权。“只规划”“只评估、不纠正”“结束后释放服务”等本轮限定优先；已有本轮执行授权不必重复询问。只要求读取本手册或整理资料时，不运行下面的硬件命令。

## 0. 快速接手：先读这一节

**每轮主线：读取双臂当前位姿，距各自init≤10cm免归位 → 新left_high单图核对棋局 → 匹配五轮预测，失效或用完才重搜 → 按source选首选手并独立估计source/target，允许跨侧边缘 → 首选手完整分支预检，不可达再试另一手 → 一次execute → 高位两图/必要修正复拍 → 低位两图 → 原位闭爪验抓 → target下降后复拍/腕图判定明显偏离目标交点时直接低位XY微调 → 原位释放、执行臂回自身init → 双臂待机 → 新left_high单图判断成功/偏位 → 离线计算FK误差并与图像结论合并落盘。** 抓前所需资料、计算、解释和脚本全部提前准备；持物或视觉门禁中不研究旧方案。此节覆盖旧文档中“固定右手”“远端禁止纵向”“载物必经staging”“释放后立即闭爪”“观察臂移到camera pose”等过时描述；当前用户指令仍优先。

### 0.1 独立工作根与入口

本包工作根由实际安装位置确定，命令示例用`CHESS_ROOT`表示，直接使用根目录的scripts、skills、config、templates和prompts。操作规则以本包操作手册和round_plan为准，服务和规划代码也使用本包文件；不依赖其他项目目录或用户级skills。每次独立工具调用设置本包workdir和绝对路径。

| 内容 | 本包入口 |
| --- | --- |
| 执行提示词 | [prompts/round_plan.zh.md](../prompts/round_plan.zh.md) |
| 完整脚本清单 | [docs/round_script_inventory.zh.md](round_script_inventory.zh.md) |
| 运行环境与迁移说明 | [docs/package_setup.zh.md](package_setup.zh.md) |
| 右/左规划器 | [round_tools.py](../scripts/round_tools.py)、[round_tools_left.py](../scripts/round_tools_left.py) |
| 服务启动和请求 | [start_service.sh](../scripts/start_service.sh)、[robot_client.py](../scripts/robot_client.py) |
| Python解释器 | 用户已安装依赖的当前环境；直接使用 `python`，或显式指定解释器绝对路径 |
| 完整性核验 | `python scripts/verify_package.py` |

核对服务ROOT与当前仓库一致，不跨服务复用token或运行目录。已有运行进程不会随磁盘文件更新。

### 0.1.1 按阶段选择相机

完成通用一轮所需的全部脚本、服务内部依赖、按需工具及调用顺序见[单轮脚本清单](round_script_inventory.zh.md)。脚本均使用0.1指定的本次部署包根，文档分工与专题对应见[docs索引](README.zh.md)。

**决定走棋方案只采`cam_left_high`一张图；执行中的抓子和放子判断只采头部`cam_left_high`+物理执行手腕部两张图。** source高位、低位闭爪前、抓取XY修正后及target下降后张爪前均使用同一规则；不采观察手腕部，不再把三路齐全当作门禁前提。走棋规划仍需同批机器人状态，但不要求腕部图片。完成后复盘只采一张新的left_high判断成功及偏位，再按第8节离线计算FK误差并合并落盘。

| 场景 | 当前配置实际请求的流 |
| --- | --- |
| 规划走法 / 默认棋盘观测 | `cam_left_high` |
| 物理左手执行抓放 | `cam_left_high` + 本次配置确认的物理左腕流 |
| 物理右手执行抓放 | `cam_left_high` + 本次配置确认的物理右腕流 |

腕流与物理手的映射必须按现场画面核对，不能按流名或逻辑`right_arm`猜测。填写 [config/chess_camera_policy.json](../config/chess_camera_policy.json) 的左右腕映射和本次依据；模板映射为空，未填时不会生成相机选择。读取入口为 [camera_policy.py](../src/g1d_chess/robot/camera_policy.py)，按进程缓存。先按 [首次开局配置](first_game_setup.zh.md) 完成核对；新服务预检将映射绑定到新token，检查点返回`physical_actor/camera_order/camera_selection`，看图按本次路径读取。已有进程不会因磁盘配置变化自动更新。

服务`capture_cameras`和本包独立采图脚本的默认请求均为left_high；抓放门禁显式请求所选两路。规划器拒绝混入腕图的规划观测，执行入口拒绝不匹配的预检相机策略。运行进程须加载匹配的本包配置并生成新token；历史三路观测不重写。无需为此次改动移动观察手或重新拟合棋盘XY表。

### 0.1.2 init待机与每轮起步10cm免归位规则

**每轮开始分别计算双臂当前末端位置到各自`init pose`的三维距离；距离≤0.10m（包含恰好10cm）的手臂不重新init。** 双臂均在该范围时，直接按source选定执行手，以当前实测关节和完整XYZ/RPY作为新plan起点；非执行臂保持本轮确认的当前位姿。不是每个坐标轴分别允许10cm，不把init配置值当实际起点，也不为满足免归位条件先移动或转腕。此阈值只判断位置，当前姿态及整条路径仍参与原有预检。

左右臂均不使用camera pose。落子后执行臂仍沿已预检尾段返回自身init，非执行臂保持本轮起点，等待下一轮；10cm规则只免除下一轮起步前的重复归位，不省略本轮放置后的回位尾段。吃子两个子事务仍连续衔接，整步完成后回init。

init参数按**物理手**分别读取当前适用配置，不把左右手设成同一组数值，也不直接复用包含另一臂camera姿态的历史双臂文件。文中旧称“观察臂”仅代表非执行臂的服务角色，不要求其移动到拍摄姿态；抓放判断仍用头部及执行臂腕部相机。

服务ready只表示进程就绪。建立或复用服务时，在同一基座坐标系、同一末端定义下比较实测FK XYZ与相应物理手init XYZ：`distance_m = sqrt(dx² + dy² + dz²)`。只有超过0.10m的手臂需要按已授权流程规划并预检初始化；范围内的另一臂不陪同重复归位。初始化后以新的实际状态生成plan。若init配置或状态缺失，不假定已满足阈值；异常持物或故障按第9节恢复。

生成plan时核对实际输出满足当前姿态规则，运行实现差异见runtime；不能认为操作手册会自动改变进程行为。

### 0.1.3 建立服务与锁定腰部

本对弈流程按用户已明确授权建立服务并锁定腰部零位。先在可见真实进程的环境核对单实例、工作ROOT、执行手入口及 `ping`、`get_state`；已有正确ROOT的健康空闲服务优先复用，不为建立连接而重启。若当前只要求建立服务与锁腰，完成本节核验即等待命令，不自动执行走棋或双臂初始化。

腰锁与双臂控制独立。若实时状态已满足 `waist_hold_enabled=true`、`waist_zero_verified=true`、`waist_target=[0,0,0]` 且无故障，保存实际 `waist_q` 并保持，不重复归零。否则只在服务空闲、双臂控制关闭、双爪空闭合且无硬件故障时调用 `lock_waist_zero`，保存回执并重新读取状态核验上述字段；未验证成功不声称已锁零。锁腰会使既有缓存计划失效，须在建立本轮计划和完整预检前完成，并在腰位变化后刷新观测、棋盘配准及source/target估计。

普通回init及 `release_control` 后保持腰锁，不逐轮释放或重新归零。仅在用户明确要求释放腰锁或结束机器人服务时，确认已放物、手臂控制关闭且现场可释放，再调用 `release_waist`；结束服务的具体顺序见8.1。

### 0.2 引擎预测未来五轮，匹配且符合机械约束才复用

1. **先确认实际局面，再匹配预测。** 每轮只采新的left_high，与上轮复盘终图核对，识别黑方实际source→target、吃子及当前完整棋局。不能用预测猜黑方走法。读取本局`move_forecast.json`，按已确认进度核对上一红方落子、黑方实际应手、当前局面和行棋方；确认一致且仍有下一步合法红方着法，才将其作为候选，**不为补满五轮提前搜索**。按本轮新图独立估计机械臂source/target并完整预检；首选手不可达再试另一手，两手均不可达才舍弃该候选并搜索。
2. **无预测、不一致、用完或候选不可达时重新搜索。** 没有可用预测、黑方实际应手与预测不一致、预测中已无下一步红方着法时，以当前已确认局面调用象棋引擎，寻找尽快赢棋的走法并保存新的五轮预测。预测着法经两手完整预检均不可达时也从当前局面重搜；棋盘复位、己方落错格、缓存损坏/不属于本局等也视为预测失效；局面尚不清楚时先识别，不能猜测后搜索。每次保存的是从当前红方回合开始的`红1、黑1、…、红5、黑5`，**五轮最多10个半回合，不是总共5个单方着法**。若已提前终局或引擎有效主变不足10个半回合，只保留实际返回并验证的部分，不能编造对手着法；下次无剩余红方着法时再搜索。
3. **引擎以最快可确认胜线为目标。** 被将军时先筛合法应将；有强制杀棋时优先正mate中更短者，不把负mate当获胜，不把搜索深度当剩余回合。右手到a–b及左手到h–i的着法也参与胜线比较，选定后按完整预检决定能否执行；无强制杀棋时选局面分值和主变稳定性较好的走法，胜势相当才比较机械成本。主变是引擎预测，不承诺对手必然照走或全局最快必胜。已匹配的预测不因每轮重新排名而触发额外搜索。
4. **搜索和落盘只在规划阶段完成。** 使用环境变量`CHESS_ENGINE`指定的Fairy-Stockfish，设置`UCI_Variant=xiangqi`、`Threads=2`、`Hash=128`、`MultiPV=3`；使用完整合法着法搜索，或用`scripts/move_constraints.py`生成包含全部合法候选的`searchmoves`命令。无合法着法时不搜索；复杂局面可延长，稳定mate1无需反复深搜。引擎及已安装的`pyffish`须先按环境安装说明确认可用。保存选定主变、引擎原始分值/日志、逐步预期局面和进度；执行期间不搜索、不维护预测，结束复盘仍只有两项。下轮规划时根据上轮复盘和新图确认已走着法，再推进预测进度，不能在发出execute时提前标记走棋成功。
5. **坐标和合法性逐步验证。** FEN从远端rank9写到近端rank0；引擎UCI行号1–10，手册0–9，例如`b7c9`对应本盘`b6→c8`。用`pyffish.legal_moves/get_fen`核验主变，保留本局实际走子历史供重复局面规则判断；这些规则检查不是引擎搜索。预测不替代当前视觉、实际落格确认或机器人预检。

预测文件及匹配顺序见[五轮预测使用说明](move_forecast.zh.md)。使用`CHESS_ENGINE`指定引擎的UCI接口，按上述选项输入当前已确认局面，并用已安装的`pyffish`逐步核验主变。选招脚本与本包规划器均不按横跨幅度拒绝走法；预测缓存读写按操作约定由主会话处理。

### 0.3 如何选择左右手，并把换手写进同一plan

**当前规则：允许任一手横跨整盘，包括右手到a–b、左手到h–i。** source在a–d列优先选物理左手，source在e–i列优先选物理右手（e列归右手）。先从首选手本轮实测起点独立估计抓放位姿并完整预检；不可达再换另一手重新估计和完整预检，两手均不可达才放弃该着法。target所在列不直接决定执行手。一旦选定，同一只手完成该次抓取、搬运、释放及回init，非执行臂保持本轮确认的当前位姿，无需在中线交接。例如g4→d4优先右手、d8→h8优先左手；跨侧目标仍须按该执行手独立估计XY/姿态并通过全路径预检。历史右手g4→d4可作跨侧放置参考，但不沿用旧camera姿态前缀。吃子的完整联合sequence亦须预检；不能在持物过程中临时切换服务。预测和新搜索使用同一可达性规则，详见[选招与机器人可达性](move_constraints.zh.md)。

| 服务模式 | 计划/状态 `right_arm` | 计划/状态 `left_arm` | 要求 |
| --- | --- | --- | --- |
| 默认右手服务 | 物理右执行臂 | 物理左非执行臂（保持本轮当前位姿） | 使用右手规划器、同一服务的新观测和token |
| 左手服务，`active_arm=left`、`arm_field_frame=transaction_roles` | **物理左执行臂** | **物理右非执行臂（保持本轮当前位姿）** | 使用左手规划器；运动/预检/使能请求含 `expected_active_arm=left`，执行请求由左手tokens入口生成 |

左右手XYZ都采用同一基座系：x向前、y向机器人左侧、z向上，**禁止镜像XYZ或按流名交换相机**。左手适配同时处理关节、模型、夹爪31/33和反馈；不要在外层再交换一次。`cam_right_wrist`在这些轮次随物理左手运动，仍以画面核对。实腰/机身变化会影响基座与棋盘关系；按0.1.3的已授权流程核验腰锁，腰位变化后重新观测与配准，不在走棋途中临时驱动腰。

换手时，服务角色只能在旧服务空闲、无持物、控制关闭时有序shutdown后切换。每轮按0.1.2核对双臂与各自init的距离，≤10cm不重复归位；下一轮执行臂从当前实测位姿出发，在同一个走棋事务中升净空、转抓取姿态并去source，非执行臂保持本轮确认的当前位姿。不再插入camera姿态过渡，也不为换手重复归位或插入额外拍照/等待。若接手旧流程时某臂仍在camera等非init姿态，也按0.1.2的距离规则判断是否需要初始化；不能把期望init冒充实际起点。全程检查姿态与邻子包络，不能只看XY。

生成plan时检查两臂等长的保持点：全部成功/失败/重试路径中，非执行臂均保持本轮确认的当前位姿，执行臂的正常结束及已规划的空爪失败回撤以自身init为终点。清除历史camera过渡后重新做整条预检；修改request后重算本地摘要，不能绕开token的请求一致性检查。异常持物/故障不能因本条强行回init。

### 0.4 近端与远端抓放：可复用候选和明确边界

**现行姿态规则（覆盖2026-10-10按半场选初始朝向的旧建议）：** source 和 target 在两半场均分别先考虑偏横向，再根据各自可达性及明显邻子干涉风险调整 RPY；抓取横向可行不要求释放保持横向，必要时 target 可偏纵向或斜向。结合两侧邻子、张爪/闭爪扫掠、指尖及指身空间独立判断抓取和释放姿态，不把抓取时的二维遮挡或投影接近直接判为碰撞。先选各阶段姿态，再结合工具相对末端偏置分别估计 XYZ；换释放 RPY 后重新估计 target EE XYZ，不能沿用旧 EE 坐标，历史坐标只适用于其 pose_context。转姿在净空高位完成，重新 build 并完整预检持物转姿、搬运、释放、上撤及回 init；高位棋子处于开口投影内不等于指尖侧壁已可抓取。g1邻g0红相、d1邻d0仕/d2黑车的首次空抓碰移案例见20261010_round1，不能直接重放。

“红方近端/黑方远端”指这块棋盘的空间半场，近端rank0–4、远端rank5–9，并不表示按棋子颜色决定姿态。横向/纵向指夹爪前伸方向在板面内的朝向；都仍可保持pitch45°向下倾斜，不是roll翻转或pitch90°垂直扎下。以下均是**模型EE命令**，不是指尖或棋子距板面的实际高度。

| 场景 | 抓取候选 | 放置候选 | 实机依据与使用边界 |
| --- | --- | --- | --- |
| 近端、物理右手 | roll0/pitch45°/yaw约+80.68°（1.408187rad），偏横向；source Z=.105m候选 | 同姿态搬运曾用Z=.105m/q2.3；新轮按最新释放高度要求填写，不自动回退旧值 | e3→e4首抓成功；g3→g4曾需要.105→.100→.095。右手成功低位实际FK Z约.122m，不能以.105当实物净高。 |
| 远端、物理左手 | 优先斜向/偏横向，两次末轮用yaw−48°、pitch45°，source Z=.109m | 高位转成偏纵向yaw0、pitch45°；最近确认release Z=.139m/q2.3 | b7→b9与b6→c8实机完成，均有少量落点偏差；其他格点仍需独立预检与终验。−48°是局部成功姿态，不是全左手通用yaw。 |
| 同一执行手的近远端及跨侧外推 | 从执行手同姿态成功样本及当前图生成候选 | 远端可采用偏纵向以改善伸展，转换必须在净空完成并重新估计工具偏置 | 跨侧放置不换手，不把右手XY/yaw镜像成左手参数；缺少对应姿态样本时明确标候选。 |

最近两轮gate=.170m、travel=.210m，历史staging右手[.22,−.21]、左手[.22,+.21]m；这些也是候选，当前整手净空由场景及反馈核对。新棋盘记录为列距35.6mm、行距33.7mm、板厚20mm、棋子直径29.7mm/高9.9mm；不要把原包35mm等距、板厚15mm、棋高12mm覆盖到新盘。

抓取以**能夹持即可**为准，不要求棋子完全或像素级居中。高位同时看左右和前后：棋子在两片夹爪相对纹理面之间，且位于前端有效纹理可夹持段内、没有退到夹爪后方；两方向足以夹住侧壁且无明显推挤或夹入邻子的风险就下放。不足时结合本次头图和执行手腕图一次估计明确XY，争取一次调整成功；source最多3次，前两次后复拍，第三次完成后直接放行下降抓取，保留既有低位闭爪门禁。

低位核对两侧侧壁进入相对的有效纹理面、指尖能够包住侧壁且没有压顶、推移或夹入邻子，满足即可闭爪。若超过半枚棋子位于纹理接触区域之外，但XY已可夹持、只是高度造成有效接触面积不足，考虑 `adjust_z` 自动竖直下调5mm；第一次后复拍，仅高度仍不足且预检允许时第二次下调后直接闭爪。若左右/前后偏位使侧壁未进入两侧有效夹持面，尤其明显偏后，应保持开爪用 `retry_high` 回高位复拍，不能降Z代替XY修正。真实空抓同样回高位；普通对中不足不直接按failure回init。真实碰撞、翻面或控制故障按原故障分支处理，不能把撞翻当抓成。

放置采用最新顺序：**原位张开 → 保持开爪，从实际XY/RPY竖直升到净空 → 开爪退到staging → 原位闭爪 → 回init**。闭爪前先离开棋子，避免夹指再次扫到、拖动落子；不能沿用曾尝试的“松开后立刻闭爪再抬”。source抓成后原位升净空即可直接去target上方，高位有需要再转姿/用连接点，不强制载物绕staging。释放是接近板面的脱离高度，不从travel高度直接抛下；.139m是最后确认的EE候选，不能声称等于棋子离板1cm或2cm。

**source与target分别定位。** source对准当前实物和有效夹持区，target对准板面交点及其邻子。优先同手、同RPY/Z、同棋盘/机身条件的成功抓取样本和正面朝上实际释放样本；使用棋盘局部方向映射估算新候选，记录不确定度。棋子顶面与板面有视差，本轮+5px只是局部估计，不能全盘固定使用。禁止用未标定的“工具长.19m”跨yaw推导精确target，禁止把翻面弹跳后的落点当稳定标定，禁止把source在线修正全量加给target。

解释转姿误差时可用 `p_piece = p_EE + R*d`：即使EE到位，改变R或夹持偏心d也会移动棋子中心。d未标定时不能仅靠末端FK保证准确落格。两次可供核对的样本如下，**不可复制为其他轮次执行命令**：

| 样本 | source请求XYZ m / yaw | target请求XYZ m / yaw | 结果 |
| --- | --- | --- | --- |
| 10月8日右手e3→e4 | [.266446,−.094318,.105] / +80.68° | [.288554,−.095434,.105] / +80.68° | 1抓/0在线修正，e4附近；释放后FK [.294241,−.087217,.122893]。 |
| 10月8日左手b6→c8 | [.291926,.199529,.109] / −48°，随后高位相对实际FK修正[+10,−5]mm | [.350247,.096347,.139] / 0°，target未叠加source修正 | 1抓/1修正/0降Z，正面朝上覆盖c8、略偏左；释放后FK [.352485,.094825,.145476]。 |

### 0.4.1 左右手独立累计 ΔXYZ 误差，供下一轮估计参考

**目录规则：** `execution_xy_errors.jsonl`固定放在`<本轮run>/docs/`，与本轮最终复盘JSON同目录。新run规划时为下面的show命令增加`--previous-journal <上一轮run>/docs/execution_xy_errors.jsonl`，并在spec的`xy_error_previous_journal`保存同一绝对路径。首次确无历史才省略该参数并填null。finish自动从plan summary取得上一份日志，承接其全部历史行并追加本轮；旧日志保留，新日志继续作为下一轮入口，因此换run不会丢失左右手累计。规划读取只读，不提前创建本轮日志。

在最新工作根以`scripts/xy_error_history.py show --run <本轮run> --hand left|right --context-id <当前兼容配置>`读取本轮`<本轮run>/docs/execution_xy_errors.jsonl`（尚未创建时只读`--previous-journal`指定的上一轮日志），在source/target估计前保存本轮`tmp/xy_error_reference.json`。**用户明确：XYZ仍由本轮图像估计；历史累计 ΔXYZ 均值仅为辅助参考，禁止程序将均值或累计和直接加减到source/target。** source看该物理手`grasp_success`的历史位移误差均值，target看同手低位`release_start`的历史位移误差均值，不看实际XYZ坐标均值，不混手、不混阶段；无样本须写明无数据。棋规预测命中仍须做这一步。

累计口径是`1000 × (actual_fk.xyz − effective_target.xyz)`（毫米）的`count`、`sum_delta_xyz_mm`及`mean_delta_xyz_mm`，向量顺序为`[ΔX, ΔY, ΔZ]`；均值是历史偏差参考，sum不是补偿量。日志完整保留每轮三种FK误差和实际/命令/目标位姿、图像判断及来源路径。只学习视觉成功的主走棋事务、现行事件阶段及有限且三维模长≤25mm的样本；旧高位释放、失败、缺失和棋外移除事务保留证据但不计入。每个阶段计数独立。配置context限定棋盘、机身/腰、工具/模型、姿态组的可比性，变化时隔离旧统计，不从另一手借数据。

source、target的XYZ最终候选仍独立依据当前图像、同手姿态和邻子选择。在spec写明`xy_error_context`、`xy_error_reference_review`、`xy_error_previous_journal`以及本局`game_id`、`round_number`、`move_plan`；估计说明记录如何结合历史 ΔXYZ 均值与当前视觉，决定保持候选时也写理由。左右手规划器build自动读取并保存`plan_summary.xy_error_reference`，有样本而无估计说明时拒绝构建，**不改变spec的XYZ**。保留`xy_error_*`字段名以兼容既有入口，内容为三维误差。自定义build须遵循同一约定。全部参考和脚本在execute前准备好，门禁不读写历史。完整说明、JSONL结构及命令见[docs/xy_error_learning.zh.md](xy_error_learning.zh.md)。离线参考不保证抓取成功率已提升，仍以实机反馈和最终视觉为准。

### 0.5 插值解决关节跳变，同时减少落点漂移

先定位失败的是哪一臂、哪段和哪种约束。关节跳变、IK终点残差、真实硬件故障、场景包络碰撞不是同一种问题。最后一轮首版0.350rad跳变发生在**起步竖直上撤末端**，并非远端释放；盲调target无法修复它。先固定正确的source/target，再处理到达它们的姿态和高位连接路径。

包内代码已实现的 `MotionHarness.prepare` 方法：

1. 每个行为航点保留固定绝对笛卡尔终点；位置线性插值，姿态用Slerp。初始采样最大间距8mm/0.05rad，每次IK使用前一已接受关节解为seed。
2. 普通段若 `max(abs(q_new-q_seed)) > .3rad`，不直接接纳跳解；在当前alpha区间二分，先求中点并接受其连续解，再求原区间终点。最多递归5层，仍跳变则整个预检拒绝，不使能执行。现有source例外段使用 `IK_SOURCE_BRANCH_JUMP_RAD=1.0`，它与 `ignore_right_envelope_steps`关联，不能把这轮概括为全路径都受.3约束，也不能将source例外扩大到target。
3. 每个接受的关节插值段仍检查5个FK采样及现有表面包络。时长按实际细分比例和关节差计算：`max(.12, 1.6*delta/.20, position_distance*alpha_span/.025, rotation_distance*alpha_span/.2) / speed_scale`。不是用更快速度跨过跳变，也不是只检查两个端点。
4. 到下一航点时，笛卡尔参照仍取原计划target，不把上一解的近似FK误差累加进新目标。张爪航点增加离线端点位置≤5mm、旋转≤.025rad检查；首次超差只对同一个固定目标复求一次，仍超差则拒绝。当前该检查覆盖含open的航点，不是所有运动点的统一收敛保证。
5. 所有成功、失败、空抓重试及低位候选在使能前完整服务预检；source修正不改变target；释放前仅按下降后的新低位图判断是否需直接XY微调，init尾段保持冻结，见0.5.2。代码变更要离线测试，再在空爪、控制关闭、服务空闲时有序重载，恢复已复核scene，采新状态并生成新token；旧token不跨进程复用。

**内部采样会成为执行关节轨迹，但不需要变成几十个独立行为航点。** 上一轮为通过b9下降，将1mm分段写入success列表，得到84航点/208采样；本轮改为区间内按需细分，14航点/145采样，完成b6→c8。两者走法和目标也不同，这组数字不是同路径性能对照，也没有证明“求解次数多”是上一轮偏落的唯一原因。不能删掉必要轨迹采样来追求点数少。

执行端当前使用固定等待，不按末端残差设置实时到位拒绝；原位夹爪动作保留上一臂目标，不执行新一轮末端纠偏。上述open端点检查是离线名义解核验，**不是已经恢复实时到位控制**。落点还受实际跟踪、夹持偏心、转姿、松爪和退爪影响。本轮释放后FK相对target仍为[+2.239,−1.523,+6.476]mm，终图略偏心。减少航点后也必须分别记录请求、命令解、实际FK、棋子中心，最后用图像验格。

若细分仍失败，优先改变source所选执行手的净空转姿位置、连接路径，或选择棋力允许且可执行的其他着法；target允许跨侧，但不因target所在侧在持物时换手，低位保持竖直。不要靠 `skip_preflight`、扩大source例外、降低scene平面或重置重试额度来制造“通过”。普通段.3rad阈值见包内[插值实现](../src/g1d_chess/robot/motion_harness.py)；执行须使用当前完整预检回执。

#### 0.5.1 抓取成功 / 开始释放快照与执行后复盘

本包增加两类 `pose_snapshot`：现有静止验抓返回success时，保存判定成功的同帧反馈；在target下降后的低位复拍决策允许张爪释放时，保存当时反馈；第三次低位XY调整后自动放行也在低位采集，标注`xy_budget_exhausted_auto_release`。不记录高位或开爪结束快照。快照采集本身不增加等待或拍照；释放前复拍流程见0.5.2。记录实测关节角/速度、反馈tick/年龄/单调时间、物理执行手、夹爪状态、原始plan目标、最终生效目标及当时`arm_target`/`arm_sent`。新增采集不计算FK或三种误差；完整plan预检及普通执行路径的既有净空保护保留，source高位/target低位XY修正段按0.5.2豁免运动检查，实际硬件与控制故障处理仍保留。左手记录采用明确的“观察臂7关节+执行臂7关节”顺序，复盘时恢复物理顺序，不能镜像XYZ。

采集只复制小份数据并进入现有非等待异步日志；命令锁忙时保留实测反馈，命令对比标缺失。每事务至多两份快照也保存在内存，执行结束后随`<token>_result.json`的`pose_audit`落盘；日志队列满/写盘失败不等待，结果中的`missing_events`/`errors`供复核。空抓、unknown和source重试开爪不冒充这两个事件。抓取成功入口覆盖当前`grasp_retry`静止验抓流程；旧自动视觉gate流程没有同样的低位成功采样，缺失即明确标注。

执行完成后的复盘仅做第8节规定的两件事；先用新left_high判断成功与偏位，再在本次部署包根使用同一模型与Python环境计算FK误差（路径替换为本轮实际归档，输出须通过`xy_error_history.py finish`与视觉结论合并落盘，并在同一步追加共用误差JSONL）：

```bash
"$CHESS_PYTHON" scripts/review_pose_audit.py \
  --result runs/本轮/tmp/checkpoints/阶段/TOKEN_result.json \
  --output runs/本轮/docs/pose_review.json
```

该工具不连接机器人；只在复盘时用实测关节计算FK，输出相对原始plan、最终生效目标、当时命令FK的三种误差（ΔXYZ毫米、位置模长、姿态旋转角），并附最后已发送命令FK。原始source与高位XY/低位Z修正后的目标分别保留。新释放快照的`event_stage=target_low_release_approval`，`original_target`为原始低位释放目标，`effective_target`为最近一次低位命令目标（不是把同一帧实测值当误差参考），`placement_target`记录批准时用于原位张爪的实际位姿。历史`target_high_release_approval`及`before_low_open_command`保持原语义，可离线读取，不重写旧数据。FK仍是关节反馈+运动学模型的估计，不是棋子中心的视觉实测；开始释放也不是已确认棋子脱离。源码：[采集](../src/g1d_chess/robot/pose_audit.py)、[复盘工具](../scripts/review_pose_audit.py)。相关修改只作用于磁盘文件并做离线验证，未重载服务或执行机器人；后续在空闲条件重载后重新预检生成token，旧记录不能补造两个时刻的反馈。

#### 0.5.2 下降后、张爪前复拍；腕图判定明显偏离目标交点时直接低位XY微调

左右手共用本包的 [release_alignment.py](../src/g1d_chess/robot/release_alignment.py)。顺序是 **target高位 → 按计划下降到释放高度 → 保持夹持，复拍头部及执行手腕部 → 决策 → 原位张爪 → 开爪竖直上撤 → 开爪退staging → 闭爪 → 回init**。`pre_release_gate`绑定下降末端，复拍不再发生在高位。仍使用同一`serial_checkpoint.py next/submit`，不另开机器人请求。

**明显偏差的判据：以物理执行手腕部相机判断张爪后棋子的预计落点，头图辅助核对目标格名与邻子。预计落点在相邻两个交点之间、方格中间或明显错开对应目标交点，均属于需要修正的明显偏差，提交adjust。** 不能等到“非常大”才修正。预计落点已在对应目标交点附近、仅有小幅偏心且可安全释放时，提交success；尽量对准目标交点，但不要求完全精准对齐，不为轻微偏心反复调整。判断的是预计棋子落点，需考虑夹持偏心、棋子顶面与板面的视差；不能用夹爪外壳/影子或“棋子边缘覆盖交点”代替落点判断，也不能仅凭FK残差小放行。reason/evidence写明执行手腕图路径、预计落点与目标交点关系及偏向。未定义未经标定的毫米或像素阈值；棋子覆盖目标交点本身不属于看不清，不要求交点完整露出；结合可见格线延伸、邻近交点、棋子中心及视差判断预计落点，不因遮挡本身修正。真正无法判断预计落点或存在碰撞风险均按需要修正处理，提交adjust并设obvious_large_offset=true、release_alignment_verified=false，说明不确定处或风险及修正依据。提交修正仍须识别目标、给出基于本次图像的明确非零XY；不设单次或累计距离上限，不进行修正段运动检查，不能虚构目标识别或位移。

adjust以本次低位实际FK为参考，保持当时实际Z/RPY，直接做XY修正，不抬升、不重新下降。source高位与target低位XY修正均不设单次或累计距离上限，不进行修正段运动检查，包括额外预检、IK可达性、路径、表面净空、关节限位及采样检查。执行前完整plan预检、当前两图的视觉判断、修正次数限制和实际硬件故障处理仍保留。此处规定文档操作策略；运行服务是否实现须以实际代码和配置核对，本次文档修改不代表服务已改变。target独立最多3次修正：前两次后立即在低位复拍；第三次修正完成后直接放行原位张爪，不再追加复拍、视觉审批或“已有充分安全依据”的放行前提。实际硬件或控制故障仍按原流程处理。

释放evaluation仍使用**七字段**：`decision / actual_relative_offset_xy_m / reason / target_identified / release_alignment_verified / obvious_large_offset / evidence`。source仍为原六字段。保留`obvious_large_offset`字段名兼容提交格式，其含义按上述交点判据更新：两交点之间、方格中间、明显错开目标交点、真正无法判断预计落点或存在碰撞风险均为true，不仅限于“非常大”；棋子覆盖目标交点本身不算看不清。success要求`obvious_large_offset=false`、`release_alignment_verified=true`、offset=[0,0]；adjust要求`obvious_large_offset=true`及有依据的非零XY增量。`release_alignment_verified`表示腕图支持预计落点在目标交点附近且可安全释放，不表示完全精准居中。服务端与提交器都拒绝未声明需要修正（obvious_large_offset=true）的adjust；它们校验决策字段，不自行识别图像，落点判断由本轮看图决策完成。模板：[success](../assets/templates/evaluation.pre_release.success.json)、[adjust](../assets/templates/evaluation.pre_release.adjust.json)。

放行时记录低位`release_start`快照，随后从已审核实际位置原位张爪，不再插入下降。只生成必要的开爪、上撤和高位连接，衔接冻结的回位轨迹；完整结果保留实际修正和最终释放目标。相机失败、过期决策及硬件故障不属于视觉批准。

新plan及预检回执须匹配`review_pose=after_descent_before_open`、`adjustment_policy=obvious_off_target_intersection`、`alignment_criterion=actor_wrist_predicted_landing_near_target_intersection`、`correction_motion=direct_low_xy`；吃子移除阶段的criterion为`actor_wrist_predicted_landing_inside_same_side_tray`，左手放左侧圆盘、右手放右侧圆盘；预计棋子整体落入盘内可用区域即可释放，不要求对准圆盘中心，不套棋盘交点判据。规划器拒绝旧策略服务回执。判定表见[释放落点判定](release_alignment.zh.md)。此次改动为磁盘代码及离线验证，**未重载现有服务、未执行机器人**；下次执行前在空爪、控制关闭、服务空闲时有序重载，重新生成整条预检及新token，不重放历史run或使用旧token。5秒是处理目标，不是延迟保证。

### 0.6 缩短每轮耗时与保留可续接记录

- 已有空闲服务优先复用；进程检查必须与实际服务处于同一可见进程空间。沙箱内“0进程”曾是漏报，不能据此运行会杀旧服务的启动器。left服务进程名也要计入。
- 一次读当前报告/棋局/相应成功参数和新left_high单图即可建立候选；仅失败时针对性追溯。不复算已核验的棋盘模型，抓后只处理本节指定的target下降后张爪前复拍，双臂距各自init≤10cm时不为换手重复归位；非执行臂始终保持本轮确认的当前位姿。
- 提前准备serial入口、source六字段/target七字段格式；source修正不改变target，target复拍只重建放置附近局部段并衔接冻结的回位尾段。检查点到来只做两图 → 判定 → submit → next；source高位和target低位均在前两次修正后复拍；source第三次完成后直接放行下降抓取，target第三次完成后直接放行张爪，两者均不再追加复拍或视觉审批。最末轮真实耗时51.36/36.03/39.65秒，5秒目标尚未达到，不能写成已保证数秒响应。
- 每次工具调用使用正确workdir；上一调用的shell变量不会自动继承。长execute保存session_id，只轮询本地文件；返回无session_id时不要将undefined存储成有效会话，避免阻断看图。submit超时先next检查回执，不重复决定或execute。
- 最多3次真实闭爪、source累计3次高位XY修正；target另有3次XY调整，source高位和target低位XY修正均不设单次或累计距离上限、不进行修正段运动检查；每次抓取最多两次5mm低位下调，仅纯高度间隙适用。用户手动复位后采新图、新plan、新预检，不能把旧坐标/token直接重放。
- 五轮预测的保存、匹配和进度推进只在每轮规划阶段完成，见0.2；完成后仅按第8节做left_high视觉判断、FK误差计算与合并落盘。取消例行更新`resume.json`、`board_state_final.json`、长报告及项目当前索引的收尾要求；原执行结果及日志保留。下轮从本轮复盘结果和新图接手，服务placed不能代替准确落格判断，偏位不追加纠正。

### 0.7 抓放经验与验收边界

[抓放经验与边界](experience.zh.md)说明source/target独立估计、有效纹理夹持区、邻子关系和预计底面落点判断。所有坐标、姿态与当前图像对应，不以历史实测值代替新环境测量；服务placed和末端FK误差不能代替最终落格验收。

### 0.8 当前棋盘90点标记

入口为 [board_position/README.zh.md](../board_position/README.zh.md)，新模型按 [标记任务说明](../board_position/prompts/mark_board.zh.md) 从当前left_high原图重新建立90点。该目录只附文档与通用脚本，未预置真实照片、像素表、分手末端XY或历史姿态锚点；旧标定目录、数据和兼容转发入口已移除，点位标记只使用board_position。

运行 `scripts/mark_board.py init/build/query` 完成图像绑定、人工可见交点记录、全盘或分区拟合、独立留出检验与标注输出。结果放在本次 `runs/board_position/<本次标记名>/`，须视觉复核四角、河界、内部和source/target附近格线。a–i从左到右、0–9从近到远。数字残差通过仍只是待视觉复核，遮挡推算点不得称为实测。

规划时根据本轮板面交点和棋子的实际位置分别估计source与target；头图像素不等于腕图像素、棋子顶面或机械臂XY。本工具不提供机器人基座变换或毫米补偿。棋盘、机位、腰姿、裁剪发生变化后重新定位；物理尺寸未知时不能默认旧盘格距。

资料及复核在execute前完成；执行检查点只用预先准备的参考和本次头图、执行手腕图判断，不持棋重建全盘。结束后仍在第8节视觉判断内辅助验格、验偏位，不增加第三项复盘。选手、source/target姿态和运动规则继续按当前单轮流程执行。

## 1. 包内入口与适用范围

| 位置 | 用途 |
| --- | --- |
| [scripts/round_tools.py](../scripts/round_tools.py) | 离线创建单轮目录、从当前状态与人工复核坐标生成完整分支、提取新 token、汇总结果；不连接硬件 |
| [scripts/mark_board.py](../scripts/mark_board.py) | 从新图标记、验证及查询90交点；不输出机械臂坐标 |
| [prompts/board_and_move_conventions.zh.md](../prompts/board_and_move_conventions.zh.md) | 棋盘方向和走法的统一规范 |
| [config/board_parameters.json](../config/board_parameters.json) | 棋盘及棋子实际尺寸 |
| [board_position/README.zh.md](../board_position/README.zh.md) / [模型操作说明](../board_position/prompts/mark_board.zh.md) | 新图标记流程；实际PNG、JSON、CSV使用时生成 |
| [config/README.zh.md](../config/README.zh.md) / [首次配置](first_game_setup.zh.md) | 配置字段说明、示例及全新G1D开局前的必要配置 |
| [config/manipulation_profile.json](../config/manipulation_profile.json) | 通用策略与待填现场参数；不会自动配置服务 |
| [assets/templates/round_spec.json](../assets/templates/round_spec.json) / [capture_spec.json](../assets/templates/capture_spec.json) | 普通走子/吃子计划输入，source/target XY 必须根据本轮填写 |
| [assets/templates/evaluation.success.json](../assets/templates/evaluation.success.json) 等 | 四种抓前检查点的严格六字段判断格式 |
| [src/g1d_chess/robot/](../src/g1d_chess/robot) | 服务启动器、客户端、串行检查点及其运行依赖快照 |
| [src/g1d_chess/cameras/](../src/g1d_chess/cameras) | 相机客户端、配置和可选独立采图工具 |
| [docs/runtime.zh.md](runtime.zh.md) / [config/environment.example.json](../config/environment.example.json) | 精确接口语义、依赖、保护开关及超时处理 |
| [docs/experience.zh.md](experience.zh.md) | 抓放判断要点、常见问题与验收边界 |
| [docs/round_report.template.zh.md](round_report.template.zh.md) | 两项复盘字段提示，不另写完整报告或增加收尾任务，见第8节 |
| [scripts/verify_package.py](../scripts/verify_package.py) / 可选本地 `manifest.json` | 文件结构、语法和链接核验；存在本地清单时附加哈希校验 |
| [docs/validation.zh.md](validation.zh.md) | 本次离线测试内容、结果与实机验证边界 |

本包可以整体迁移，必须保留目录层级。模型构建模块及`assets/models/g1/`的URDF/mesh在包内，不需要安装`unitree_deploy`；SDK、规则库及Python依赖按`pyproject.toml` 的 `chess` 组安装，Fairy-Stockfish单独安装并设置`CHESS_ENGINE`。先按[环境安装说明](environment_setup.zh.md)完成配置和离线核验，再按首次配置指南核对现场网络与硬件。本包不会自动识别完整棋局、判断棋规、完成手眼标定或保证所有格点可达。

每轮使用新图、新状态、新计划和新 token；以当前现场数据完成规划及执行。

## 2. 一轮的阶段与交付

| 阶段 | 要做的事 | 必须留下的输出 |
| --- | --- | --- |
| 初始化与观测 | 建立/复用服务并按0.1.3核验腰部锁零；确认双臂位姿及各自init距离（≤10cm免归位）、空爪及故障；确认轮次、对手走法、己方，取得left_high单图 | 初始完整相机响应与图、状态、实腰、服务能力 |
| 决策与定位 | 核对黑方实际应手并匹配预测；一致且合法则作为候选，无预测/失效/耗尽或两手均不可达则搜索新五轮；独立定位source/target | 选定着法、规划阶段保存的五轮预测及确认进度、定位依据 |
| 完整规划 | 抓前、抓后连续尾段、重试/失败分支全部生成 | round_spec、request、plan_summary |
| 预检 | 用当前服务预检全部分支，核对真实检查覆盖范围 | prepared、阶段prepared、新的一次性execute_request |
| 抓前门禁 | 当前source高位两图；需要时XY修正；低位两图和最多两次5mm下调 | checkpoint、六字段evaluation、decision、receipt、真实耗时 |
| 搬运与释放 | 静止抓成后抬升、搬运、target下降、张爪前复拍/腕图判定明显偏离目标交点时直接低位XY微调、释放、上撤、回init | 原execute响应/本地result、抓取与运动日志 |
| 完成后复盘（仅两项） | ① 新left_high判断走棋成功及位置偏差；② 计算两时刻三种FK误差，追加本轮误差/左右手累计至本轮docs内的累计JSONL，与①合并落盘 | `<本轮run>/docs/execution_xy_errors.jsonl`及`docs/{第几轮}_{这一轮的走棋子方案}_{时间戳}.json`，关联原图和真实结果 |
| 结束服务（仅另行要求，不属默认复盘） | 用户明确要求结束时，确认已放物及控制关闭，再关闭服务 | shutdown回执和进程计数；不能只说“已回init” |

吃子是两个完整抓放事务提前联合准备：先移走被吃子，再移己方棋子。两次抓取分别有抓前门禁，每次抓成后都执行各自target下降后张爪前复拍与放置尾段。

## 3. 坐标、尺寸与必须复核的参数

从机器人视角：**左→右 a–i，近→远 0–9**。不要直接从中文记谱的“一、二、三”猜图像左右。普通走法写 `红方 i0 → h0`；吃子先写 `黑方 h0 → empty（黑方 h0 被吃）`，再写 `红方 i0 → h0`。`empty` 表示移入执行手同侧圆盘（左手左盘、右手右盘），XY须根据本轮图像确定，左右手圆盘释放Z均为0.159m，见[圆盘吃子规则](capture_tray.zh.md)。

当前棋盘图像坐标由board_position的新图标记流程生成，不附带旧盘数值标定。物理格距、河宽、板厚和棋子尺寸须按本次实物核实；既有配置或历史案例不能自动当作新棋盘测量值。图像像素因透视不等距，拟合残差只表示像素一致性；基座到棋盘变换未完成独立标定时保持null。

查询示例（全部离线）：

```bash
cd /path/to/g1d_chess
CHESS_ROOT="$PWD"
python scripts/mark_board.py --help
# 先按board_position/README.md生成本次结果，再查询：
python scripts/mark_board.py query h0 --reference /path/to/current/board_points_pixels.json
```

每轮先核对印刷格线、河沿和宫格与本轮图的一致性，再用参考点加速定位。source看实际棋子及两指前端有效夹持区；target看目标板面格线、空位和邻子，不用已碰歪的棋子充当标准格心。棋子顶面高于板面，具体高度须核实；斜视产生视差，不能直接把顶面中心当板面像素。裁图放大后必须记住缩放/平移并映射回原图坐标。

**板面点、棋子中心、模型EE、指尖是不同位置。** source修正改变夹持偏置，固定EE target也可能偏落；不把source偏移自动加到target。新棋盘标记工具不提供机器人末端锚点。

以下为模板与当前实现的流程候选和约定，不能代替新设备测量；source、gate、travel、staging与RPY按现场图像、工具几何和完整预检确定。需要改变固定实现约束时先适配并离线验证，不能只填写说明JSON。

| 项目 | 当前模板/流程约定 | 使用条件 |
| --- | --- | --- |
| 抓取/释放姿态 | 模板roll=0、pitch=45°、yaw约1.408rad | 左右手及source/target独立核对，姿态变化重新估计EE和包络 |
| 抓取低位EE Z | 模板.100m，自动下调每次5mm | 最多两次，只在XY已可夹且高度接触不足时使用 |
| source高位EE Z | 模板.165m | 与低位闭爪阶段分开 |
| 搬运高度、staging | 模板travel=.205m；staging XY必须现场填写 | 按当前手和路径核对 |
| 释放EE Z | 棋盘.139m、圆盘.159m | 是模型EE高度，不是指尖净高；新设备不兼容时须适配实现 |
| 夹爪 | close=0，模板open幅值q2.3rad，默认电机目标+2.3rad | 内置Dex1左右爪开爪方向均默认+1；同一事务所有open共用一个q |
| 静止抓成 | q≥.15、abs(dq)≤.1、稳定1.5s；接触超时2s | 仅闭爪后的原位阶段，不增加搬运途中检查 |
| 额度 | 真实抓取最多3次；source高位XY最多3次，target低位XY另有3次 | XY无距离上限，不因新token重置额度；运动检查的实现边界见runtime |

模型FK采用锁零腰坐标系，实际腰位须按0.1.3从本次状态核验。`config/init_state.json`发布时为空，由现场同帧状态确定物理左右手init，不使用历史位姿。

## 4. 建立本轮目录与观测

以下命令分阶段执行，**不是从头到尾盲跑的脚本**。路径变量都在包根设置；`RUN`必须换为本轮唯一名称。只读/离线准备可以先完成。

这些代码块假定同一个持久终端。Codex每次独立shell调用不会继承前一次的cd或变量；必须每次设置工具workdir为包根，并重新定义所需CHESS_*变量或使用展开后的绝对路径。另一个终端处理检查点时也要重设变量；export不能解决独立工具进程之间的继承。

```bash
cd /path/to/g1d_chess
CHESS_ROOT="$PWD"
CHESS_PYTHON="$(command -v python)"
CHESS_CLIENT=scripts/robot_client.py
CHESS_SERIAL=scripts/serial_checkpoint.py
CHESS_RUN="$PWD/runs/RUN"
"$CHESS_PYTHON" scripts/round_tools.py init --run "$CHESS_RUN"
```

`init`只建立tmp/images/docs、待填写spec和resume.json。每次重新规划用新run；不覆盖已有request/token/result。运行目录由当前任务创建，发布包不附带可重放的历史run。

已授权实际对弈后，先核对服务进程（此命令不启动）：

```bash
bash scripts/start_service.sh --status
```

只有确认无正在执行/持物的旧服务、并且确需启动时，使用以下入口。**启动器会SIGKILL所有匹配的旧服务，不允许用于故障恢复或“试试看”**。按 [首次配置指南](first_game_setup.zh.md) 激活已安装依赖的Python环境，并设置本次`CHESS_INTERFACE`、`CHESS_CAMERA_HOST`及`CHESS_ENDPOINT`；下面显式传参时使用`CHESS_GRIPPER_SIGN=1`，与启动器和服务的内置Dex1默认一致。方向不随主机环境或左右手改变，特殊关节约定才显式覆盖。前台启动需保留独立工具会话，待ready后继续，不要中断它来返回提示符。

```bash
bash scripts/start_service.sh --foreground \
  --python "$CHESS_PYTHON" --interface "$CHESS_INTERFACE" \
  --camera-host "$CHESS_CAMERA_HOST" --endpoint "$CHESS_ENDPOINT" \
  --gripper-open-sign "$CHESS_GRIPPER_SIGN" \
  --log-dir "$CHESS_RUN/service" --log-file "$CHESS_RUN/service.log"
```

已有正确ROOT的空闲服务可复用，不随意重启；核对实际代码版本与路径，不能因为磁盘有新文件就认为运行进程已更新。服务ROOT决定允许的runs目录，生成器与服务必须同ROOT。脚本、run与服务统一位于本次部署包根，左手使用包内左手入口，不混用其他根目录。确需切换时先有序结束旧空闲服务；执行/持物时先恢复原事务。

空闲服务上先取得完整连接及状态响应：

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" ping --timeout 5000 --save-response "$CHESS_RUN/tmp/ping.json"
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" get_state --timeout 5000 --save-response "$CHESS_RUN/tmp/initial_state.json"
```

按0.1.3核对腰锁；已锁零则只保存核验，不重复调用。仅在需要锁零且空闲、双臂控制关闭、双爪空闭合、无故障时执行：

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" lock_waist_zero --timeout 20000 --save-response "$CHESS_RUN/tmp/waist_lock.json"
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" get_state --timeout 5000 --save-response "$CHESS_RUN/tmp/waist_verified_state.json"
```

核验腰锁成功后，走棋任务再采规划图；仅建立服务锁腰的任务到此结束：

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" capture_cameras \
  --cameras cam_left_high \
  --output-dir "$CHESS_RUN/images/initial" --timeout 20000 \
  --save-response "$CHESS_RUN/tmp/initial_observation.json"
```

看本次left_high规划图并使用响应中的状态，不把另一次状态当拍图时刻。核对control=false、fault为空、双臂当前位姿、执行爪空闭合及真实图像；按0.1.2计算各自init距离，≤10cm不归位；发生初始化后刷新观测。记录实腰、左右臂、q/dq、时间、命令计数及保护能力；腕流名称与物理左右要从画面核对。初始持物或故障时不使用本包空爪起步生成器，先根据已有状态制定恢复计划。

读取服务现有scene并核对当前桌面/棋盘/工具配置。scene优先采用用户给定且适用的现场数值；用户未提供任何现场信息时，由模型按 [首次配置第5节](first_game_setup.zh.md#5-配置现场几何双臂init和动作参数) 自行估计全部scene字段，部分给定时补齐其余字段。利用可用图像、状态、模型、尺寸资料及合理假设，逐字段记录来源、依据与不确定度后继续设置，不以用户提供数值、补充资料或额外确认为前置条件。已有scene兼容时直接复用；估计值明确标注为估计。`config/scene.example.json`是待填模板，不自动加载；不要提交null、示例或`_field_help`。评估记录保存为本次run的`tmp/scene_assessment.json`，请求仅保留`scene`数值，保存为`tmp/scene_request.json`。确需设置时，仅在服务空闲、控制关闭状态调用：

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" set_scene --request "$CHESS_RUN/tmp/scene_request.json" \
  --timeout 5000 --save-response "$CHESS_RUN/tmp/scene_result.json"
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" get_state \
  --timeout 5000 --save-response "$CHESS_RUN/tmp/scene_verified_state.json"
```

核对设置响应`ok=true`及scene数值，并核对新状态的`scene_guard`与请求一致；保存回执仅表示参数已应用，不表示估计已成为实测或碰撞已完整验证。scene改变后生成新计划并重新完整预检，旧token不能继续使用。

## 5. 选棋、填spec、生成完整plan

比对上一轮已验收棋局和本轮图，确认对手动作、当前执子方、source身份、目标占用及将军应对。不能因棋盘上出现空位就默认轮次或走法合法。棋子手工复位、对手移动和镜头/棋盘变化要单独记录。

填写`$CHESS_RUN/round_spec.json`：source_square/target_square、source与target各自的EE XY/Z/RPY及证据、四项复核文字（棋规、格线配准、姿态前提、scene/邻子）。两个evidence应分别说明原图、格线位置、实际夹持/释放偏置和不确定性。source XY与target XY没有通用默认值。

普通走子用`move_kind=normal`、`capture=null`。吃子按`assets/templates/capture_spec.json`填被吃方、被吃子实际抓取位姿和执行手同侧圆盘内的释放位姿。`capture.offboard_target.kind=round_tray`，`physical_side`必须与物理执行手一致：左手left、右手right；XY及证据按本轮图像填写，左右手`z_m`均固定为0.159，规划器拒绝侧别或释放高度不匹配。`offboard_surface_review`复核盘内空位、盘沿、已放棋子、承托高度及工具净空；圆盘释放Z统一为0.159m。不要求精准居中，预计棋子整体落入盘内可用区域即可。细则见[圆盘吃子规则](capture_tray.zh.md)。

```bash
"$CHESS_PYTHON" scripts/round_tools.py build --run "$CHESS_RUN" \
  --spec "$CHESS_RUN/round_spec.json" \
  --observation "$CHESS_RUN/tmp/initial_observation.json"
```

输出`tmp/request.json`和`tmp/plan_summary.json`，**只是候选完整plan**。检查左右臂、首段竖直升高、高位转姿、source gate、源点低位、固定target、释放/上撤/init以及失败返回均正确。物理执行臂完成走棋后回自身init，物理非执行臂始终保持本轮确认的当前位姿；左右手执行均如此，计划中不包含camera pose。

抓成success路径采用最新顺序：**source低位闭爪 → 静止验抓 → 原位升净空 → 必要的高位转姿 → 直接target高位 → target下降 → 两图复拍并立即决策（腕图预计落点在目标交点附近则释放；两交点之间、方格中间或明显错开目标交点则直接低位XY微调） → 原位张开 → 按实际XY/RPY开爪竖直上撤 → 开爪退staging → 原位闭爪 → 高位转回 → init**。不强制载物绕staging。所有开合单列停稳动作；`success_source_steps=[0,1,2]`限定source修正；target修正另由`pre_release_adjustment`控制，init冻结。本包生成器已输出开爪上撤、开爪退staging再闭爪的顺序；历史run构建脚本中的手工重排不能重复套用。执行使用0.1指定的本包入口，并核对进程确已加载匹配版本。

开合时执行臂保持当前实测位置，开合路点中的 XYZ/RPY 不会同时驱动手臂。若 plan 要先移动，须显式安排独立的 `hold` 移动路点，再在到位后开合；该路点纳入原有完整 plan 预检，不增加执行中的额外预检路径。释放审批后的开爪尾段使用最新反馈估计开爪位姿，详见[运行语义](runtime.zh.md)。

生成器还提供空抓重试、失败撤回和未知接触分支。它不会验证人工evidence真伪、图像新鲜度、棋规、邻子碰撞，也不做IK；必须继续完整服务预检。配置JSON是说明/模板，修改`manipulation_profile.json`不会自动改变生成器内的额度与阈值，当前可调输入以round_spec为准；改变其他策略要同步修改生成器、预检并验证。

## 6. 预检、启用并提交一次执行

普通走子：

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" prepare_transaction \
  --request "$CHESS_RUN/tmp/request.json" --timeout 120000 \
  --save-response "$CHESS_RUN/tmp/prepared.json"
```

吃子在同位置使用`prepare_sequence`，request内必须恰好两个事务。第一条在同侧圆盘释放被吃子，开爪到位反馈须成功；随后按预检路径开爪竖直上撤至净空，经高位staging连续移往己方source高位，直接复拍并进入抓前门禁。staging只是运动路径点，不作独立视觉确认或人工等待；第二条沿用已张开的夹爪，不重复开爪、不回init、不重新enable，也不插入第二个服务请求。第二条最终放置后才回init。此流程对左右手及任意吃子通用；开爪未到位、硬件故障或路径检查失败时停止衔接并保留现场状态。不要把“抓取”和“放置”拆成两个事务；target下降后张爪前复拍在同一事务内完成。

预检须`ok=true`，审阅全部成功/重试/失败分支和scene检查结果。`collision_checked=false`必须如实保留：已有IK、关节/采样和桌面工具包络检查，不等于完整机械臂、邻子或动态碰撞验证。source的`ignore_source_tool_envelope=true`是明确例外，必须审阅；当前若几何不满足，不通过放宽其他保护来凑通过。

```bash
"$CHESS_PYTHON" scripts/round_tools.py tokens --run "$CHESS_RUN" --prepared "$CHESS_RUN/tmp/prepared.json"
```

生成`tmp/execute_request.json`和`tmp/stage_1_prepared.json`（吃子另有stage_2）。token只在当前服务进程有效、消费一次。准备后更改scene、重启、移动起始姿态或改计划，需要重新预检；不能把旧token放进通用模板。

此时完成所有解释、定位和分支审阅。若已授权本轮执行，启用后立即提交对应完整事务：

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" enable_control --timeout 5000 \
  --save-response "$CHESS_RUN/tmp/enabled.json"
```

先读取enabled.json，确认ok=true、control_enabled=true且没有fault，再提交：

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" execute_transaction \
  --request "$CHESS_RUN/tmp/execute_request.json" --timeout 900000 \
  --save-response "$CHESS_RUN/tmp/result.json"
```

吃子使用`execute_sequence`替代`execute_transaction`，只提交sequence一次，不能再独立执行两个子token。检查enable回执；若控制未成功启用，不继续。

execute是长调用：在工具中保留其运行session，让主会话能同时处理本地检查点；记录session_id到resume.json。它不是等到命令完全返回再看图。终端操作需另一个终端处理本地文件。执行期间**不发送第二个get_state/hold/capture等REQ**，同步REP会被正在执行的请求占用。

## 7. 抓前 / 释放前串行门禁与连续回位

先选第一stage，只轮询本地文件：

```bash
CHESS_PREPARED="$CHESS_RUN/tmp/stage_1_prepared.json"
"$CHESS_PYTHON" "$CHESS_SERIAL" next --prepared "$CHESS_PREPARED" --seconds 45
```

`needs_review=true`时，读取返回的**当前头部和执行手腕部两张图片**、实际FK、stage和剩余额度。检查点打开期间只做“读取 → 两图判断 → 提交（source六字段，target七字段）”，不插入历史搜索、代码阅读、测试、其他模型或报告整理。5s为目标，真实耗时必须保留，不能宣称历史已经达到。

在本轮tmp中写新的evaluation文件。source/preclose仅含：`decision`、`actual_relative_offset_xy_m`、`reason`、`source_identified`、`descent_alignment_verified`、`evidence`。`pre_release`将身份/对齐字段换为`target_identified`、`release_alignment_verified`，并增加`obvious_large_offset`，见0.5.2的专用模板。所有模板均须填入本次图像依据，不能原样提交。

| 当前阶段 | 允许决定 | 含义 |
| --- | --- | --- |
| source high | success / adjust / failure | 同时确认左右与前后有效夹持区；对中不足用有依据的XY adjust并复拍（无距离上限、不进行修正段运动检查），不因普通偏差直接failure回init；failure只用于真实异常或无法安全继续的情形 |
| preclose低位 | success / adjust_z / retry_high / failure | 对中与邻子间隙满足才闭爪；纯夹持深度不足可按预检下调5mm；前后/左右对中不足用retry_high、XY=[0,0]，保持开爪固定XY/RPY竖直升高复拍，再于高位修正XY并复拍；不提交低位XY或自选Z，不因普通偏差直接failure回init |
| pre_release低位（已下降、未张爪） | success / adjust / failure | 腕图预计落点在目标交点附近则张爪；两交点之间、方格中间、明显错开目标交点、真正无法判断落点或有碰撞风险均需修正；棋子遮住交点本身不算看不清；第3次后自动释放，无第4次复拍 |

高位offset是相对**当前实际FK**的米制增量；source由提交器换算为累计规划参考，target直接使用本次低位实际FK参考，不套用source累计偏移。不要重复累计或混用图像像素。低位第一次adjust_z后复拍，第二次后直接闭爪，没有第三次低位门禁。空抓先回高位复拍并按新图修正XY；source高位修正累计3次，前两次修正后复拍，第三次修正完成后直接放行下降抓取，不再追加高位复拍或视觉审批；既有低位闭爪门禁保留，该次仍空抓才按预检路径回init。真实抓取本轮累计最多3次，不因重新规划重置；每次真实重抓的低位额度重新按预检序列计算。target的3次XY调整独立计数，第三次后直接释放。

将`CHESS_CHECKPOINT`设为next返回的当前checkpoint_path，把`CHESS_EVALUATION`设为刚写的对应阶段新evaluation JSON：

```bash
"$CHESS_PYTHON" "$CHESS_SERIAL" submit --prepared "$CHESS_PREPARED" \
  --checkpoint "$CHESS_CHECKPOINT" --evaluation "$CHESS_EVALUATION"
```

检查accepted回执，然后继续next读取下一门禁或结束结果。submit等待回执超时不代表决策没生效；先next，禁止重复提交。

**静止抓成后服务自动搬运至target高位并下降至释放高度，张爪前复拍两图。腕图预计落点在目标交点附近则释放；两交点之间、方格中间或明显错开目标交点则直接低位XY微调；前两次后复拍，第三次后直接张爪。放行时保存低位FK快照，随后连续开爪上撤和回位。** 可读取本地进展和原长调用结果；不能用服务请求插入新动作。原位稳定q判据不扩展成搬运全程夹持监控。普通执行路径的既有健康/表面保护保留；source高位和target低位XY修正段按0.5.2不进行运动检查，实际硬件或控制故障仍可停止。运行服务能力以代码、配置和实时状态为准。

吃子第一stage的圆盘释放决策一旦被接受，服务自行执行开爪、上撤和到己方source高位的转场，不在staging等待操作者。第一stage显示`finished/placed`且整体sequence仍在执行时，立即将`CHESS_PREPARED`改为stage_2，用`next`读取己方source高位门禁并处理本次两图；提前备好stage_2文件路径，不等独立确认或重新规划。不能以第一颗已移走就宣布整步吃子完成；第二stage失败要记录棋局已部分改变。

## 8. 走棋完成后的复盘：仅两项

**用户确认：完成后的复盘只做以下两件事，左右手一致。** 执行完成及回init以原execute最终结果为前提；客户端超时不等于完成，仍按第9节等待真实结果。吃子须等两个子事务全部结束，不能在第一颗移走后提前复盘。复盘不额外读取实时FK，不补采执行事件快照。

1. **采一张新的`cam_left_high`图片，判断走棋是否成功、是否存在位置偏差。** 对照本轮source→target和目标交点，记录成功/失败/无法判断，以及偏位方向和可可靠估计的偏差；精度不足时写明未知，不把像素偏差直接当成基座XY毫米误差。保留原图路径和采集时间。
2. **此时离线计算FK误差，并与第1项结果合并落盘。** 只使用执行中已保存的“抓取成功”和“target下降后低位批准张爪释放”两时刻快照，分别计算实际FK相对原始plan、最终生效目标、当时命令FK的三种误差，包含ΔXYZ毫米、位置模长和姿态旋转角。新释放快照与对应低位目标比较；历史高位事件仍与历史高位目标比较，不混算。将图像判断、偏位描述和FK结果放入同一份本轮`docs/{第几轮}_{这一轮的走棋子方案}_{时间戳}.json`，保留原始图片、动作结果及FK明细路径。快照缺失或计算失败则如实记录原因，不补造数值；视觉偏位与机械臂FK误差并列记录，不直接断言因果。

**最终复盘文件命名：**

```text
<本轮run>/docs/{第几轮}_{这一轮的走棋子方案}_{时间戳}.json
示例：<本轮run>/docs/round_move.json
```

- **第几轮**：固定写为`第N轮`，N为本局我方走棋轮次，从1开始；下一次我方行棋递增，新局重新从1开始。不是预测数组下标、引擎半回合数、抓取次数或run序号。同轮重新规划/重试不递增；吃子的移除与落子两个子事务仍是一轮。
- **走棋子方案**：采用本轮最终选定的中文棋谱着法，如`马八进七`、`车三平六`；有前后子歧义时使用完整棋谱记法。文件内容同时保留source/target格名。名称不能包含路径分隔符、空白或下划线，禁止把整段解释放入文件名。失败/偏位时仍按本轮计划着法命名，真实结果写入内容，不能把文件名当作成功证明。
- **时间戳**：合并复盘实际落盘时的北京时间（`Asia/Shanghai`），格式`YYYYMMDD_HHMMSS`；不使用规划、预检或拍照时间。完整文件名重复时等待下一秒重新取时，使用独占创建，不能覆盖已有记录。
- 规划阶段确认并保存本局轮次、最终着法，准备合并脚本；执行期间不为命名检索历史或整理文件。复盘第2项合并时生成时间戳，并在JSON中保存`round_number`、`move_plan`、带`+08:00`时区的`review_created_at`，与文件名一致。
- 最终合并记录只按此规则保存，不再生成固定名`post_move_review.json`作为最终入口。`pose_review.json`等仅为离线FK中间明细，不替代最终复盘；吃子的两份FK明细合并至同一份本轮复盘。准备输出时创建本轮`docs/`目录。
- 续局在下一轮规划阶段读取相应run中符合此规则的复盘，按JSON的本局归属、轮次、着法及真实结果核对，不按文件名字典序或全项目最新文件推断棋局。历史固定名记录仍可读取作为证据，不要求批量改名；历史提示词/记忆中的固定文件名由本规则替代。

第1项采图沿用本轮客户端与run目录：

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" capture_cameras \
  --cameras cam_left_high \
  --output-dir "$CHESS_RUN/images/final" --timeout 20000 \
  --save-response "$CHESS_RUN/tmp/final_observation.json"
```

第2项在本次部署包根中使用0.5.1的`scripts/review_pose_audit.py`。将`CHESS_RESULT`设为含`pose_audit`的本轮真实事务结果路径，不能用超时响应或prepared替代：

```bash
"$CHESS_PYTHON" scripts/review_pose_audit.py \
  --result "$CHESS_RESULT" \
  --output "$CHESS_RUN/docs/pose_review.json"
```

吃子对两个子事务的真实结果分别计算，输出不同文件，合并到同一份按上述规则命名的最终复盘JSON并保留子事务对应关系；不要把sequence包装结果直接交给只接受事务结果的工具。现有FK工具输出的是FK明细，完成第2项时还需将其内容与第1项视觉判断合并写入上述复盘记录，不能只留下分散且未关联的文件。

**第2项同时维护独立误差JSONL，不能省略。** 规划时备好当前context、game_id、轮次和中文着法；第1项视觉判断写入`tmp/visual_review.json`（至少含`move_success`、原图路径/时间和偏位描述），随后用离线工具一次完成日志追加和命名复盘合并：

```bash
"$CHESS_PYTHON" scripts/xy_error_history.py finish \
  --run "$CHESS_RUN" --game-id "$CHESS_GAME_ID" \
  --round-number "$CHESS_ROUND_NUMBER" --move-plan "$CHESS_MOVE_PLAN" \
  --context-id "$CHESS_XY_CONTEXT" \
  --report "$CHESS_RUN/docs/pose_review.json" \
  --visual-review "$CHESS_RUN/tmp/visual_review.json" \
  --plan-summary "$CHESS_RUN/tmp/plan_summary.json"
```

该命令默认向`<本轮run>/docs/execution_xy_errors.jsonl`每轮追加一个JSON行，行内包含本轮完整误差及更新后的左右手/source-target独立累计（`count`、`sum_delta_xyz_mm`、`mean_delta_xyz_mm`）；缺失/不合格样本也保留行及原因，计数不增加。token相同且输入相同则幂等，不重复学习。吃子先传己方主走棋FK报告，再重复`--report`传移除报告；辅助报告保留但不学习棋外误差。最终复盘关联JSONL路径和本轮累计。该操作属于第2项，不追加第三项收尾。具体口径、context及下一轮只读参考流程见[历史 ΔXYZ 误差参考](xy_error_learning.zh.md)。

合并记录只包含本局/本轮标识、上述命名元数据、source→target、原图路径和时间、视觉成功及偏位结论、两时刻FK误差及缺失原因、对应结果文件路径、规划误差参考及JSONL路径/本轮累计统计。最终JSON和日志完成落盘后即结束复盘；回复仅给成功/偏位结论、FK误差简述和文件路径。不运行旧`round_tools.py report`，不填写完整报告模板，不追加棋局推演、标定更新、历史检索、索引维护、额外相机/状态采集、纠偏动作或服务关闭。下轮所需棋局核对留到下一轮规划时进行。

### 8.1 仅在用户另行明确要求时结束服务（不属于复盘）

一轮正常成功或完整预检失败回撤会关闭手臂控制，但进程可能还在；`hold`、`release_control`、`shutdown`不是同一状态。如果本轮任务要求结束服务，在确认已放物、机械臂可支撑且服务空闲后，必要时先release，再shutdown：

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" get_state --timeout 5000 --save-response "$CHESS_RUN/tmp/before_shutdown.json"
```

仅在已确认可释放且control仍启用时调用：

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" release_control --timeout 5000 --save-response "$CHESS_RUN/tmp/released.json"
```

普通走棋结束保持腰锁，不因手臂控制关闭而释放。仅在用户明确要求释放腰锁或结束服务、且现场可释放和手臂控制已关闭时，若 `waist_hold_enabled=true`，调用并保存回执：

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" release_waist --timeout 5000 --save-response "$CHESS_RUN/tmp/waist_released.json"
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" get_state --timeout 5000 --save-response "$CHESS_RUN/tmp/before_shutdown_verified.json"
```

确认 `control_enabled=false` 和 `waist_hold_enabled=false` 后，用户要求结束服务时再执行：

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" shutdown --timeout 5000 --save-response "$CHESS_RUN/tmp/shutdown.json"
bash scripts/start_service.sh --status
```

以shutdown回执和进程计数0证明关闭，不只看socket存在与否。异常持物时不自动release/shutdown，也不通过重启启动器清理。

## 9. 中断、超时与恢复

`resume.json`由操作者更新，至少记录：当前阶段、run、原execute工具session_id、是否已发送、当前stage prepared、最新checkpoint及结果路径。此文件是索引，不是实时状态机；不能单凭里面的`false`重发动作。

新会话接手时按顺序读取本轮spec、summary、prepared、本地结果/检查点及原执行会话。已发送请求后超时或用户打断**不等于取消**。服务可能仍在运动，也可能无期限等文件决策；`decision_timeout_s=300`不是串行门禁的自动退回时间，`next --seconds 45`仅控制本地轮询。

| 所见情况 | 处理 |
| --- | --- |
| 当前checkpoint尚未决策 | 读取当前两图，按本手册判断并提交一次 |
| decision已存在但无receipt | 等待/查看本地next和日志，不重复提交 |
| 抓后尾段执行中 | 保留原长调用，读取本地结果，不发新REQ或插门禁 |
| 原客户端响应超时，request_sent=true | 不重新execute；查本地token_result、原工具session和服务日志 |
| 客户端request_sent=false | 仅说明未成功发送；先确认服务/现场状态再决定下一步 |
| 本地result已完成 | 按第8节完成两项复盘，不重放token |
| 服务重启、token失效 | 先重新确认现场；旧prepared不能恢复内存缓存 |
| 抓失败或unknown/fault | 看实际走过的预检分支与持物状态，不能假定空爪或已释放 |

当前同步服务无法在execute中即时处理另一个hold/release请求，因此它不是独立急停通道。对已核实服务进程的SIGINT在控制启用时设置cancel/hold，并不等于中断客户端；SIGKILL/重启也不是受控恢复。具体分支见[运行说明](runtime.zh.md)。硬件故障或状态不明时不以猜测轨迹续跑。

## 10. 维护与离线核验

```bash
python scripts/verify_package.py
"$CHESS_PYTHON" -B -m unittest discover -s tests -v
```

核验不启动服务、不调用相机、不做运动。本包随附规划生成器及服务代码。本次打包核验不代表本包已连接机器人或完成新实机回合。每轮IK和场景预检仍由实际运行服务执行。

可选的本地文件哈希清单不上传Git，也不覆盖以后新增run；缺少清单时离线核验仍检查文件结构、语法、链接和脚本清单。运行时相机配置YAML可能被服务器改写，校验差异应按原因记录，不把变化静默当成原快照。维护后报告验证及来源；使用本地哈希清单时同步维护并保留修改记录。项目经验保留在本目录，不自动改全局AGENTS或把单次参数升级为跨项目规则。
