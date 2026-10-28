# G1D Chess

[English](README.md) | 中文

通过仓库内的 [SKILL.zh.md](SKILL.zh.md)，让具备终端执行与图片查看能力的模型，使用 Unitree G1 与 Dex1 内置夹爪完成中国象棋的单轮规划、左右手抓放、连续吃子和结果复盘。

模型负责看图确认棋局、估计 source/target、调用象棋引擎及判断检查点；Python 服务负责完整计划预检、缓存事务和运动执行。仓库没有自动识图并下完整盘棋的总控 CLI，也不包含机器人端的相机服务器。

## 安装与配置

基线为 Linux x86_64、Python 3.10。先在系统中安装 Python、venv 和 Git，保留完整仓库，从仓库根进入技能目录后执行：

```bash
python3.10 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
source .venv/bin/activate
cp .env.example .env
```

也可激活用户自己安装好的 Python 环境，用 `python -m pip install -r requirements.txt` 补齐依赖，无需新建 `.venv`。后续命令直接使用该环境的 `python`；服务启动器默认使用 PATH 中的 `python`，也支持 `--python /实际环境/bin/python`。

按 [.env.example](.env.example) 填写单独安装的 Fairy-Stockfish、DDS 网卡和相机服务器。内置 Dex1 的左右爪默认均为 `CHESS_GRIPPER_SIGN=1`：电机角度向正方向增大时张开，`q=0` 闭合；例如规划开爪幅值 `2.3` 对应电机目标 `+2.3 rad`。这是内置夹爪的方向约定，更换主机、Python或网络环境无需重新填写或翻转；不同硬件/固件关节约定仍可显式覆盖。然后加载：

```bash
set -a
source .env
set +a
python scripts/check_environment.py
```

引擎安装、平台兼容性及原生库问题见 [环境安装说明](docs/environment_setup.zh.md)。代码使用本仓库的模型模块，不需要安装 `unitree_deploy`；`unitree_sdk2py` 是通过 chess 可选依赖组安装的机器人通信依赖。`.env` 不会被程序自动加载，配置 JSON 也不会自动 export 环境变量。

随后完成 [首次开局配置](docs/first_game_setup.zh.md)：确认物理腕流映射、scene、双臂 init、棋盘尺寸和当前图像的点位标记。scene 优先使用用户提供的现场数值；即使用户未提供任何现场信息，也由模型结合可用图像、机器人状态、工具模型、尺寸资料及合理假设，自行估计全部缺失字段，记录依据与不确定度后设置，无需用户逐项给定或另行确认。具体方法见首次配置第5节。发布的现场配置为空，模型估计与模板参数须注明来源，不能当作当前设备实测。

## 通过 skill 使用

保留整个仓库，让模型读取根目录 [SKILL.zh.md](SKILL.zh.md)。例如：

> 读取 `/实际部署路径/g1d_chess/SKILL.zh.md`，检查首次配置是否齐备；本次只检查环境，不连接机器人。

配置齐备且准备实际走棋时：

> 读取 `/实际部署路径/g1d_chess/SKILL.zh.md`，按当前棋盘完成红方这一轮走棋，核对 source/target 和实际落点。

skill 根据任务选择安装配置、服务与腰锁、棋盘标记、单轮执行或恢复流程，引用本仓库的文档与脚本。仅复制 `SKILL.md` 不会携带运行代码和模型；这里不依赖其他项目工作副本或全局安装的机器人 skills。

正常单轮遵循 [round_plan](prompts/round_plan.zh.md)，详细命令见 [操作手册](docs/operation_manual.zh.md)。执行中的门禁按 `next → 看本次两图 → submit → next` 处理；机器人持物时不要研究代码或补装依赖。

## 目录

```text
g1d_chess/
├── README.md                 安装与使用入口
├── SKILL.md                  模型任务入口与按需文档路由
├── .env.example              已支持的环境变量模板
├── config/                   现场配置与空白测量记录
├── src/g1d_chess/             走棋、机器人、相机及棋盘标记实现
├── scripts/                  可执行 CLI、启动器与离线检查
├── assets/                   必需模型资源与 JSON 表单模板
├── prompts/                  单轮提示词及棋盘/走法规范
├── docs/                     操作手册与专题说明
├── board_position/           新棋盘点位标记说明
└── tests/                    离线测试及合成数据
```

运行时在 `runs/` 中生成图像、计划、检查点和复盘；该目录及 `.env`、虚拟环境不进入版本管理。脚本名称、输入输出和模块依赖见 [脚本清单](docs/round_script_inventory.zh.md)。

## 离线验证

已安装依赖并加载环境变量后，保留完整仓库，从仓库根进入技能目录后执行：

```bash
python scripts/verify_package.py
python scripts/check_environment.py
python -m unittest discover -s tests -p 'test_*.py'
```

这些检查不连接相机、不初始化 DDS，也不操作机器人。`verify_package.py` 检查文件结构、语法、文档链接与脚本清单；本地存在 `manifest.json` 时额外校验文件哈希。该清单不上传 Git，缺少时仍可完成其他检查。它们不能代替现场配置和本轮完整路径预检。操作规则与当前实现仍有差异，尤其 XY 修正段的局部检查，见 [实现核对边界](docs/runtime.zh.md#实现核对边界)。
