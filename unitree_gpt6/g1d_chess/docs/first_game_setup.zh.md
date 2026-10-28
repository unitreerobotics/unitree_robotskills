# 全新的G1D开始下象棋前如何配置

[English](first_game_setup.md) | 中文

先按[环境安装说明](environment_setup.zh.md)安装requirements及象棋引擎，再完成本页的现场配置，再使用 [round_plan](../prompts/round_plan.zh.md)。本包面向G1与Dex1及`assets/models/g1/`模型；首次部署不能把归档图像、init、scene、夹爪方向或模板位姿当成新设备的测量。`config/`只提供模板和说明，不会自动识别硬件或应用参数。

## 1. 确定安装位置与解释器（离线）

可将 `.env.example` 复制为 `.env`，填写其中现有字段，再在每个新 shell 中显式执行 `set -a; source .env; set +a`。项目不自动加载 `.env`，配置 JSON 也不会自动设置环境变量。下例展示等效的显式变量方式。

在目标机器选定包根和兼容Python。以下全部`<…>`须替换；文档示例IP和路径仅说明格式。shell变量不跨独立工具调用保存，每次新调用重新设置或使用实际绝对路径。

```bash
cd /path/to/g1d_chess
source /path/to/robot-env/bin/activate
export CHESS_ENGINE=/path/to/fairy-stockfish
CHESS_ROOT="$PWD"
CHESS_PYTHON="$(command -v python)"
CHESS_CLIENT="$CHESS_ROOT/scripts/robot_client.py"
CHESS_SERIAL="$CHESS_ROOT/scripts/serial_checkpoint.py"
"$CHESS_PYTHON" scripts/check_environment.py
"$CHESS_PYTHON" scripts/verify_package.py
```

使用按`pyproject.toml` 的 `chess` 组装好依赖的Python环境，可激活后直接运行`python`，也可显式调用该环境的解释器绝对路径。SDK和规则库从此环境导入；模型从本包`assets/models/g1/`读取，不添加第三方源码路径，不依赖开发机Conda路径或`unitree_deploy`。每个独立工具调用都须重新激活环境或使用已确认的解释器绝对路径。

按 `config/environment.example.json` 的`_field_help`填写实际部署记录 `config/package_environment.json`。`package_root`记录实际绝对路径；`python_executable`记录已安装依赖的解释器绝对路径；`engine`记录外部引擎路径或PATH命令名。实际解释器由当前激活环境或显式命令选择，这些字段仅用于记录。两个JSON都不会自动export变量或向启动器传参。

目标机需兼容已安装的SDK、棋规库、引擎和包内模型。更换Python ABI、系统或架构须安装匹配版本，并重新核验原生库与模型。`scripts/check_environment.py`验证实际导入、模型网格、引擎UCI及棋规库；失败先解决具体依赖，不能仅修改版本记录宣称通过。用实际解释器的`--version`及`importlib.metadata.version()`读取版本，填入`config/python_packages.json`；它不负责安装或固定依赖版本。更换相容环境不代表已确认硬件连通。

## 2. 填网络、端点和夹爪方向

根据现场网络信息填写以下字段，并保存来源：

| 字段 | 如何确认 | 传到哪里 |
| --- | --- | --- |
| `interface` | `ip -brief address`识别连接机器人DDS网段的接口并核对连接 | 启动器`--interface` |
| `camera_host` | 机器人图像服务器实际主机/IP及网络可达性 | 启动器`--camera-host`；独立采图`--host` |
| `endpoint` | 选择可写本地IPC路径，或当前服务支持的本地回环TCP端点 | 启动器及每一次robot_client都显式传`--endpoint` |
| `gripper_open_sign` | 内置Dex1左右爪默认均为`+1`；仅硬件/固件采用不同关节方向约定时按依据覆盖 | 启动器`--gripper-open-sign`，或`CHESS_GRIPPER_SIGN` |

内置Dex1的物理左爪（电机31）和物理右爪（电机33）均采用正角度张开、0闭合，电机目标允许范围默认是`[0,5.4]rad`。例如规划`gripper_open_q=2.3`，实际下发`+2.3rad`；幅值表示电机角度，不是开口毫米数。该方向沿用内置Dex1的默认约定，更换主机、Python、网卡或部署目录不改变符号，不需要逐次试开或让用户重新确认。`CHESS_GRIPPER_SIGN`未设置或为空时启动器及服务均采用`+1`，CLI覆盖环境变量；只有实际硬件/固件的关节约定不同时才使用有依据的`-1`覆盖，不因左右手做镜像翻转。服务`get_state.gripper_open_sign`返回实际生效符号。

启动脚本和服务要求提供网卡和相机服务器，可显式传CLI或使用`.env.example`中的对应环境变量；这两项缺失不会启动连接。夹爪方向已有上述默认，配置JSON不自动覆盖启动参数。图像配置请求端口及流端口须与现场服务器匹配；ImageClient目前默认请求端口60000，服务入口未暴露此端口参数，若现场使用其他端口，需先适配并验证图像客户端入口。相机配置请求成功后写入`config/cam_config_client.yaml`（不纳入版本管理）；如果需要本地回退，可按`config/cam_config_client.example.yaml`填写真实端口等参数后保存为该文件。发布包不提供可直接连接旧设备的回退配置。

例如在完成实际填写后设置：

```bash
CHESS_INTERFACE='<已核对的DDS网卡>'
CHESS_CAMERA_HOST='<已核对的图像服务器>'
CHESS_ENDPOINT='<选定的本地服务端点>'
CHESS_GRIPPER_SIGN=1
```

## 3. 先确认物理相机映射

当前代码约定头图逻辑键为`cam_left_high`，腕流逻辑键为`cam_left_wrist`及`cam_right_wrist`。名称不证明物理同侧；必须看本次实机画面确认。按任务取得采图授权后，可在不启用手臂运动的情况下，用独立工具取得头部和两个腕流：

```bash
"$CHESS_PYTHON" scripts/capture_cameras.py \
  --host "$CHESS_CAMERA_HOST" --cameras cam_left_high cam_left_wrist cam_right_wrist \
  --output-dir "$CHESS_ROOT/runs/site_setup/camera_check"
```

查看照片，填写`config/chess_camera_policy.json.wrist_stream_by_physical_arm.left/right`；两项必须分别为两个不同腕流。填写`mapping_evidence`，记录图片路径、确认依据和时间。JSON里的`_field_help.example`仅演示一组同名映射，不能当已确认结果。当前模板两项为空，规划器与服务会拒绝使用。

服务按进程缓存策略。应在首次启动前填写；已经运行的进程不会自动加载磁盘修改，必须在空闲、空爪等现场条件满足后有序处理，旧token作废并重新预检。实际走棋规划只用头图，抓放检查点只用头图加物理执行手腕图，按新token绑定的`camera_selection/camera_order`读图。

## 4. 建立服务并记录本次状态

这一步会操作机器人连接，仅在已授权实际连接且现场适合时执行。先检查已有服务并决定复用或启动；不要用启动器探测环境，它会清理匹配的旧进程。已有持物或执行中的服务先按原事务恢复。

`runs/`用于保存当前环境新生成的数据，无需从旧环境复制。为本次配置/首轮建立新的`runs/…`目录后，确需启动时显式传入已确认参数（物理执行手也要明确选择）：

```bash
CHESS_RUN="$CHESS_ROOT/runs/site_setup"
mkdir -p "$CHESS_RUN/tmp"
bash scripts/start_service.sh --foreground \
  --arm '<left或right>' --python "$CHESS_PYTHON" \
  --interface "$CHESS_INTERFACE" --camera-host "$CHESS_CAMERA_HOST" \
  --endpoint "$CHESS_ENDPOINT" --gripper-open-sign "$CHESS_GRIPPER_SIGN" \
  --log-dir "$CHESS_RUN/service" --log-file "$CHESS_RUN/service.log"
```

启动会保持前台工具会话；其他工具调用重新设置所需变量。在空闲服务上用同一端点保存`ping/get_state`，核对真实设备、控制状态、故障、腰姿及物理手角色：

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" ping \
  --save-response "$CHESS_RUN/tmp/ping.json"
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" get_state \
  --save-response "$CHESS_RUN/tmp/initial_state.json"
```

腰部按操作手册 0.1.3在满足条件时锁零并复核，不把模板中的零腰当实测。左手入口的逻辑臂字段与物理手映射按当前服务说明核对；不能用右手入口的字段含义直接解释左手状态。

## 5. 配置现场几何、双臂init和动作参数

**scene 数值来源。** 用户可以给定全部或部分现场数值。优先使用用户给定且适用于当前场景的数据，先核对单位、基座/EE坐标系、机器人腰姿、工具和开爪状态；遇到冲突说明具体差异，不静默覆盖。即使用户没有提供任何现场信息，也由模型自行估计全部scene字段；用户只给定部分时，模型补齐其余字段。模型可结合现有观测、机器人与工具模型、尺寸资料及合理假设完成估计，不把用户提供数值、逐项补充资料或额外确认作为设置scene的前置条件。字段定义见 [scene模板](../config/scene.example.json)，按以下方式取得依据：

| 字段 | 用户未提供时，模型如何评估 |
| --- | --- |
| `surface_z` | 结合当前桌面/棋盘图像、已知尺寸、机器人基座与腰姿、同批状态的FK及可用的相机/坐标变换资料，估计运动区域内障碍表面的最高Z。用户提供的桌面离地高度须转换到模型基座坐标系；棋子顶面也不能直接当棋盘面。可主动结合其他视图、模型尺度、机器人几何和位姿关系估计；缺少直接尺度或基座定位资料时，明确记录采用的尺度/定位假设及较大的估计误差，不将视觉推断称为公制实测。 |
| `finger_end`、`half_width`、`half_thickness` | 结合当前工具的模型/尺寸资料及实物图像，在模型EE局部坐标系中估计包络。`half_width`包含本轮计划开爪范围，不能只取闭爪宽度；核对模型是否对应实际Dex1及附件。 |
| `uncertainty` | 按表面定位、工具尺寸、坐标变换及图像估计的误差，评估非负的等效竖直裕量。记录采用的误差范围与合并依据，不能为让路径通过而缩小裕量。 |
| `travel_clearance`、`minimum_clearance` | 依据现场表面、工具和路径需求选择策略裕量，并检查是否满足当前代码范围。可从代码默认值0.040m、0.003m开始评估；它们是默认策略值，须核对适用性，不是现场测量。 |
| `xy_tolerance`、`rotation_tolerance` | 核对运行时“同XY/同姿态”的判定需求，可采用代码默认值0.001m、0.008rad并注明来源；它们不是抓取精度或视觉估计误差。 |

模型先使用任务范围内已有的图像、状态与资料，也可主动收集可用依据；需要采图或读取硬件状态时，沿用当前任务的硬件授权。用户未提供资料本身不构成等待或拒绝设置scene的理由，模型应结合模型几何、可用观测、代码默认值和合理假设完成缺失参数的估计，并将未知项反映在假设与不确定度中，无需额外请求用户确认。估计结果直接用于后续设置与完整计划预检；记录哪些是用户给定、模型估计或代码默认，不能因服务接受就称为现场标定。

将逐字段结果保存在本次run的`tmp/scene_assessment.json`，记录数值、单位、坐标系、来源类型（用户给定/模型估计/代码默认）、图像或状态路径、模型版本、估计方法及误差、适用条件。部分字段由用户给定时保留这些字段，仅补足缺失部分。使用板面最高Z和`uncertainty`时说明裕量如何计入；模型包络随姿态变化后的最低点才是净空计算依据，不能只比较EE Z。当前包络后端固定为EE局部X的−0.040m，所附模型EE在腕yaw局部X偏移0.050m，须与工具和现场核对。

**scene 设置与核验。** 从上述结果生成本次run的`tmp/scene_request.json`，请求仅包含`scene`及其有限数值字段；评估记录另存，不把`_field_help`、未填null或说明文字当请求发送。当前 [BoardGuard](../src/g1d_chess/robot/board_guard.py) 要求`finger_end`为0.10–0.30m、`half_width`为0.02–0.09m、`half_thickness`>0、`uncertainty`≥0、`travel_clearance`≥0.030m、`minimum_clearance`≥0.002m；字段全部有值也不等于符合当前工具或现场。参数超出实现范围时核对几何与实现适用性，不修改估值来凑通过。

仅在服务空闲、手臂控制关闭时按 [操作手册第4节](operation_manual.zh.md#4-建立本轮目录与观测) 调用`set_scene`，保存设置回执；再用`get_state`核对`scene_guard`与请求一致，并记录对应评估文件。已有scene与当前环境兼容时优先复用，不逐轮重新估计或覆盖；棋盘、工具、腰姿或坐标前提改变时重新评估，scene改变后重新完整预检，不复用旧token。

**双臂init。** `config/init_state.json`发布时为空，必须填写当前设备测量，需按已授权的初始化流程确定物理左右手的待机姿态，用同帧实测关节、同一模型的FK XYZ/RPY、夹爪与腰姿、时间和来源替换当前适用记录。它被规划器实际读取，不能只在config添加新值。新状态到各自init的三维距离按现行10cm规则核对，范围内不重复归位；不要以变更配置代替真实状态。

**操作候选。** 在`config/manipulation_profile.json`记录本次source/target姿态、source高位/低位、travel、staging、释放高度和开爪q等确认结果；source与target及左右手分别核对。该文件不自动写入plan，必须将实际值填入本轮spec：

| 配置字段 | 本轮spec中的对应位置 |
| --- | --- |
| source/target姿态与高度 | `source.z_m/rpy_rad`、`target.z_m/rpy_rad`，两者独立估计 |
| source高位、搬运高度 | `gate_z_m`、`travel_z_m` |
| 撤离/衔接区域XY | `staging_xy_m`，按实际执行手 |
| 开爪目标 | `gripper_open_q`，同一事务所有open共用 |
| 圆盘目标 | `capture.offboard_target`的XY/Z/RPY、同侧圆盘标识及现场依据 |

`assets/templates/`、操作手册、round_plan及运行代码规定的G1/Dex1流程候选/约束包括棋盘释放Z=.139、圆盘Z=.159、接触检测阈值和5mm下调规则；它们不是新设备标定。尤其当前规划器对圆盘释放Z=.159有固定校验，闭爪动作使用0，接触检测参数部分写在生成器中：**如果新现场无法满足这些约定，仅填写config不会适配成功**，需先同步调整受影响的模板、规划器/服务约束与本轮说明并验证，再执行动作。修正次数和执行规则按round_plan及当前任务指令执行。

## 6. 建立新棋盘与误差context

用本次头图按 [board_position说明](../board_position/README.zh.md) 和 [模型标记任务](../board_position/prompts/mark_board.zh.md) 生成并视觉复核90点。棋盘/相机/腰姿改变后重新定位。`config/board_parameters.json`中的尺寸发布时为空，须按当前实物测量填写；板面像素、棋子顶面、腕图像素和基座末端坐标不能互换。本工具不提供手眼标定。

为本次兼容条件填写`config/xy_error_learning.json.context_id`，例如`g1d_site_A_board_A_tool_A_pose_v1`仅示范命名。也可在本轮spec填写`xy_error_context`或向误差工具传`--context-id`，当前优先级为显式CLI/context参数 > spec > config。首次确无历史日志时不传`--previous-journal`；之后保留并承接兼容日志，条件改变则新建context，不直接复制历史组身份。JSONL不是自动坐标补偿。

## 7. 进入第一轮

按操作手册第4节建立新run，确认本局棋子摆放、执子方、当前回合和FEN/合法着法；全新棋局从我方第1轮计数。用新状态和图像填写spec，核对新init、scene、物理相机映射及完整抓放/失败分支，取得新预检token后再执行。首轮仍走source高位和低位判断、target低位判断及最终两项复盘，不因配置完成跳过视觉检查。

`verify_package.py`验证发布文件和链接，现场编辑受清单管理的配置会产生预期哈希变化；它不是“现场参数正确”检查。保留原发布清单与本地修改记录，不为让哈希通过而伪称现场校准完成。用`check_environment.py`核对环境，用新图/状态及完整运动预检核对实际运行条件。完成本页只代表准备过程有依据，机器人当前状态以本次回执为准。
