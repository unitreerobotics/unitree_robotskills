# g1d_gomoku — G1-D 自主五子棋

[English](README.md) | 中文

宿主 Agent 由使用者另行配置，本项目不内置模型 API 客户端或附带模型访问权限。文档中的具体模型名称描述开发工作流，不代表安装本项目即可访问该模型。

用户只需说“下一手”或“执行下一轮”。具备看图和工具调用能力的 Agent 按 [SKILL.zh.md](SKILL.zh.md) 观察棋盘、识别黑白棋，自主选点，再控制右臂抓一颗黑棋、对准、放置并归位。用户只在松爪前确认。

**只有自主下棋这一种任务模式。** 不提供指定棋盘坐标、固定落点、预设棋谱或符号取放预览。行列坐标是识别和决策产生的内部记录，末端位姿由 Agent 根据本轮图像与反馈求解。

## 谁在做什么

- **宿主 Agent**：读取四路原图、识别棋局、复核网格、抓放精调、检查路径净空和询问松爪确认。使用 Agent 自身的看图和工具能力，不让用户填写棋局或目标位姿。
- **棋局引擎**：检查完整 19×19 黑白棋局、回合与终局，自动优先本手成五、阻挡白方成五，再按连续及跳空棋形比较攻守。它是确定性的局部威胁策略，不是专业棋力搜索器。
- **轮次工作流**：绑定原图与识别结果，保存唯一目标和标注，检查对准记录与预计算返程，记录确认，调用同一发布器释放并连续回撤。
- **硬件工具**：提供 DDS、四路相机、FK/IK、持续控制、反馈检查和归位。

这是一套由 Agent 执行的技能，Python 进程不会在后台自行调用未配置的视觉模型。单独启动 CLI 会在需要 Agent 看图或动作规划时给出下一步状态；宿主 Agent 按技能继续处理。

## 从部署到实际下棋

你负责首次部署、布置棋盘和确认松爪；gpt6-astra 负责识别棋局、调用引擎选点，以及控制右臂抓放和归位。不需要你填写棋盘坐标、棋局矩阵或机械臂位姿。下面按首次使用的顺序操作。

### 1. 准备运行环境

先使用带 CasADi 绑定的 Pinocchio 环境。新建环境可用 conda-forge：

```bash
conda create -n gomoku -c conda-forge python=3.10 'pinocchio>=3,<5' 'casadi>=3.6,<4' pip
conda activate gomoku
```

然后从仓库根目录安装：

```bash
python -m pip install -r unitree_gpt6/g1d_gomoku/requirements.txt
```

G1 和 Dex1 的 DDS 通信 SDK 已内置于 [unitree_sdk2py/](src/unitree_sdk2py/VENDORED.md)，直接从仓库源码使用；requirements.txt 安装所需数值库与通信库。Pinocchio 必须包含 `pinocchio.casadi`；仅通过 pip 安装 `pin` 和 `casadi` 不保证具备该绑定。SDK 保留上游 BSD 3-Clause 许可证。

G1 模型已内置于 [unitree_gpt6/g1d_gomoku/models/g1/](models/g1/README.zh.md)，包含 `G1_29_ArmIK`、关节配置与索引、URDF 和全部 49 个引用网格。

按 [硬件接入](docs/hardware_integration.zh.md) 启动 TeleImager 相机发布端，并接通机器人和相机网络。运行期间不修改工程或安装依赖。

### 2. 创建仓库内的工作空间

仍在仓库根目录执行。默认在仓库根目录下建立 `gomoku-workspace/`，也可提前设置 `GOMOKU_WORKSPACE` 为自己选择的位置。配置从仓库模板初始化，不依赖开发者机器上的文件。该工作空间已加入 Git 忽略规则，运行时只向这里写入文件，源码、工程配置和文档保持只读。

```bash
export GOMOKU_SOURCE_ROOT="$(pwd -P)"
export GOMOKU_WORKSPACE="${GOMOKU_WORKSPACE:-$GOMOKU_SOURCE_ROOT/gomoku-workspace}"
python -m unitree_gpt6.g1d_gomoku.src.workspace --workspace "$GOMOKU_WORKSPACE"
export GOMOKU_WORKSPACE="$(cd "$GOMOKU_WORKSPACE" && pwd -P)"

export GOMOKU_HARDWARE_CONFIG="$GOMOKU_WORKSPACE/config/hardware.local.yaml"
export GOMOKU_OWNER_ROOT="$GOMOKU_WORKSPACE/runtime/game"
export PYTHONPATH="$GOMOKU_SOURCE_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
export TMPDIR="$GOMOKU_WORKSPACE/tmp"
export XDG_CACHE_HOME="$GOMOKU_WORKSPACE/cache"
export PIP_CACHE_DIR="$XDG_CACHE_HOME/pip"
export RUFF_CACHE_DIR="$XDG_CACHE_HOME/ruff"
export MPLCONFIGDIR="$XDG_CACHE_HOME/matplotlib"
mkdir -p "$PIP_CACHE_DIR" "$RUFF_CACHE_DIR" "$MPLCONFIGDIR"
cd "$GOMOKU_WORKSPACE"
```

后续终端和 Agent 工具进程都要继承这些变量，并以工作空间为当前目录。可将环境设置保存到自己的工作空间脚本中，在每个新进程加载。临时代码放 `scratch/`，轮次图像和计划放 `runtime/rounds/`，日志放 `logs/`。完整目录约定见 [架构文档](ARCHITECTURE.zh.md#4-gpt6-astra-的独立工作空间)。

先检查实际数值后端；不需要归位或场景标定文件，不连接设备：

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main \
  --config "$GOMOKU_HARDWARE_CONFIG" model-check
```

结果须为 `model_initialized: true`。否则先按输出的 `python_executable` 和具体导入错误修复同一个环境，再进行现场采集。

### 3. 首次配置与现场采集

硬件配置使用工作空间的 `config/hardware.local.yaml`，模板为 [hardware.example.yaml](config/hardware.example.yaml)。按部署网络填写 `camera.host` 和 `robot.network_interface`；端口、DDS domain、控制频率及控制限值见 [配置说明](config/README.zh.md)。工程不附带某台机器的归位姿态或抓放经验。

先通过现有控制方式让机器人处于适合当前场景的休息姿态，由 Agent 核对后只读采集归位参考：

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main \
  --config "$GOMOKU_HARDWARE_CONFIG" \
  pose --output "$GOMOKU_WORKSPACE/calibration/reset_right_state.json" \
  state --grippers

python -m unitree_gpt6.g1d_gomoku.src.experience init \
  --workspace "$GOMOKU_WORKSPACE"
```

采集命令不会移动机器人，只保存当前双臂关节、腰角、TCP 和夹爪反馈；不能在任意姿态采集，也不能填零替代反馈。归位参考属于执行前的标定，不是成功经验。已有工作空间中的旧默认文件应移出当前配置，再从当前设备重新采集。

YAML 中的 `runtime.directory: ../runtime` 和 `runtime.reset_state: ../calibration/reset_right_state.json` 均相对 YAML 所在目录解析。所有控制进程共用同一份配置和控制锁。

在运行下一步预检前，由宿主 Agent 先只读查看当前四路图像和反馈，完成工作空间的 `calibration/scene.json`，字段与要求见 [场景几何数据](config/README.zh.md#场景几何数据)。宿主 Agent 可从当前工作空间启动，技能加载方式见第 5 步；标定期间不启动运动发布器。

首次经验库为空，gpt6-astra 根据当前四路图像、机器人反馈、模型和现场标定规划动作。完成归位并确认相应阶段成功后，才通过 [经验数据工具](docs/experience.zh.md) 写入实测经验；后续按机器人、场景、配置、标定和模型匹配后参考。

### 4. 检查配置、反馈和四路图像

先做完全离线的预检，确认配置有效、依赖全部可用：

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main \
  --config "$GOMOKU_HARDWARE_CONFIG" preflight
```

预检不连接设备，也不验证现场标定。随后只读检查实际状态与相机：

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main \
  --config "$GOMOKU_HARDWARE_CONFIG" state --grippers

python -m unitree_gpt6.g1d_gomoku.src.hardware.main \
  --config "$GOMOKU_HARDWARE_CONFIG" \
  camera --snapshot "$GOMOKU_WORKSPACE/calibration/camera_check"
```

确认左右头图能看清完整棋盘，腕图能辅助检查供子与夹爪。固定棋盘，在桌面供子区放好可抓取的黑棋。

新场景需要建立或复核棋盘、桌面、供子区与机器人坐标的关系。图像交点不能直接当机器人坐标；工程没有棋盘尺寸或抓放高度默认值，Agent 必须建立当前场景标定并检查可达性和整段净空。

### 5. 启动 gpt6-astra，加载技能与持续控制器

从工作空间启动宿主 Agent，确保它能读取本地图像、执行命令、写文件和管理长期后台进程，并继承前面的环境变量。给它下面的指令：

> 读取 `GOMOKU_SOURCE_ROOT` 下的 `unitree_gpt6/g1d_gomoku/SKILL.zh.md`，按自主五子棋流程执行。使用 `GOMOKU_WORKSPACE` 保存全部临时代码、图像、计划和日志，运行期间只写工作空间，源码、工程配置和文档保持只读。使用 `GOMOKU_HARDWARE_CONFIG` 和 `GOMOKU_OWNER_ROOT`，检查已有控制器，只启动一个持续发布器。先检查通信、四路图像、现场归位参考与标定，然后自主下一手黑棋。抓取、精调和返程准备由你完成，松爪前等待我的确认。

Agent 应在独立长期进程中启动唯一控制器。下面是对应工具命令；若你已手动启动，要告诉 Agent 复用，不能再启动一份：

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main \
  --config "$GOMOKU_HARDWARE_CONFIG" \
  control --root "$GOMOKU_OWNER_ROOT" --execute owner
```

**这一步开始实际发布控制命令。** 初始化成功后持续保持启动时姿态，不会自动归位。运行期间保留该进程，不能每手重复启动；不在归位姿态时，由 Agent 先审阅当前通道，规划抬升、退出及归位。

另一个继承相同环境的终端可只读查看状态：

```bash
python -m unitree_gpt6.g1d_gomoku.src.hardware.main \
  --config "$GOMOKU_HARDWARE_CONFIG" \
  control --root "$GOMOKU_OWNER_ROOT" status
```

### 6. 执行一手棋

你说“自主下一手黑棋”后，Agent 应完成整条流程，不需要你逐条运行阶段命令：

```text
play → 看图识别 → decide → 取子、短抬升核验、高位转运、低位对准
     → 准备返程 → prepare-release → 用户确认 → release → 归位等待
```

| 阶段 | Agent 做什么 | 是否执行机械动作 |
| --- | --- | --- |
| `play` | 新建轮次目录，检查归位，采集四路原图与反馈 | 不改变目标姿态 |
| 看图识别 | 识别完整黑白棋局，交叉核对网格，保存识别结果 | 不移动 |
| `decide` | 引擎自主选点，生成唯一交点标注 | 不移动 |
| 抓取与对准 | 调用 `move`、`grip`，抓一颗黑棋、短抬升核验、转运及对准 | 会移动 |
| 返程准备 | 根据本轮已执行的高位来路生成完整回撤计划 | 只生成计划 |
| `prepare-release` | 核对并绑定目标、低位对准记录和返程 | 不松爪 |
| 等待确认 | 说明落点、释放与连续回撤，等待你的实际确认 | 持续保持 |
| `release` | 收到确认后松爪、存一次图、连续抬升、退出并归位 | 会移动 |

你只需在请求松爪确认时回答“确认放子”。识别文件、选点、位姿、动作命令和返程由 Agent 自己生成；不能自行代替用户批准。

`play` 只是观察阶段，单独执行会停在 `awaiting_board_review`，等待 Agent 看图；它不会启动控制器、抓子或自动归位。已有控制器仍在发布姿态保持命令。`decide` 遇到识别缺失、左右头图不一致、有不确定交点、回合不对或已结束时拒绝选点，不会默认空棋盘。

以下仅用于理解 Agent 的阶段调用，假定本手目录尚不存在：

```bash
export GOMOKU_ROUND="$GOMOKU_WORKSPACE/runtime/rounds/round_01"
python -m unitree_gpt6.g1d_gomoku.src.main play \
  --config "$GOMOKU_HARDWARE_CONFIG" --owner-root "$GOMOKU_OWNER_ROOT" \
  --round "$GOMOKU_ROUND"

# Agent 看完四路原图并写好本轮识别结果之后：
python -m unitree_gpt6.g1d_gomoku.src.main decide --round "$GOMOKU_ROUND"
```

底层动作及阶段文件格式见 [自主执行流程](docs/autonomous_play.zh.md)。返程必须在请求松爪确认前准备完毕；目标、配置、持有姿态或计划改变后，不能复用旧确认。释放后客户端等待完整回撤结束才返回。

### 7. 继续对弈与结束

机器人归位后，你落白棋，再说：

> 我已经下子，请执行下一轮。

Agent 复用同一个持续控制器，为每手创建新的轮次目录，重新观察和决策，不复用旧棋局、旧目标或旧授权。

结束时告诉 Agent：

> 结束对弈，确认机器人处于现场休息姿态，按现场控制权交接流程停止发布器，并报告退出结果。

停止发布不等于继续保持姿态，也不等于硬件急停，需要按现场方式交接控制权。出现保护停机或过期反馈时，先检查当前状态再恢复控制，不能直接重放上一轮命令。`execution_failed` 或 `return_unverified` 也不能通过再次执行 `release` 重试。

## 轮次记录

每手使用一个新目录，保存四路原图、图像摘要、完整棋局识别结果、`position_and_move.json`、单交点标注 `suggested_black_move.svg`、阶段反馈、返程计划和确认记录。SVG 内嵌完整原图，可以直接在浏览器查看。

内部使用 1 起始行列，按左头图从上到下、从左到右编号。默认执黑，无禁手，连续五颗及以上获胜。已有棋局继续当前回合，每手归位后等对手和下一轮启动指令。

放子后只保存一次图像、不分析，随后不再采图，连续抬升归位。因此 `returned` 表示归位关节反馈通过，`physical_placement_verified` 仍为 `false`，不能把它说成落点视觉验收。

## 文档与测试

- [代码架构与工作空间](ARCHITECTURE.zh.md)：模块关系、执行流程，以及仓库内独立运行目录的可移植配置。
- [技能入口](SKILL.zh.md)：宿主 Agent 的使用约定。
- [自主执行流程](docs/autonomous_play.zh.md)：初始化、看图识别、动作工具和阶段文件格式。
- [操作规则](docs/operating_guide.zh.md)：任务、棋规、位姿、抓放与记录约束。
- [操作方法](docs/manipulation.zh.md)：定位、抓取臂形和反向来路归位。
- [经验数据](docs/experience.zh.md)：空库启动、证据校验和成功后的数据写入。
- [硬件接入](docs/hardware_integration.zh.md)：可选依赖、私有配置和设备工具。

`config/default.yaml` 只保存自主模式与棋规；经验数据位于工作空间的 `experience/`，源码中的空模板不在运行时修改。

```bash
python -m unittest discover -s unitree_gpt6/g1d_gomoku/tests -v
python -m ruff check .
python -m ruff format --check .
```

测试用合成局面、假相机文件和假反馈覆盖自主决策、状态衔接和失败行为，并检查模型资产摘要。具备数值依赖时还实际加载内置模型，验证 FK、IK、重力补偿与左臂保持；全部测试不连接机器人。

初始化工具只生成配置模板、目录和空经验库，保留已有配置，不生成归位或场景标定。它不能代替现场采集；`preflight` 在必需字段或采集文件缺失时仍会报错。离线验证不代表机器人通信、相机和实际抓放验收。
