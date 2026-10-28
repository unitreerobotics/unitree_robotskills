# 包使用与离线核验

[English](package_setup.md) | 中文

工作根为本次部署的 `g1d_chess/`。全新设备先完成 [首次开局配置](first_game_setup.zh.md) 和 [配置模板说明](../config/README.zh.md)，再按 [操作手册](operation_manual.zh.md)、[round_plan](../prompts/round_plan.zh.md) 执行。所有项目入口均使用本包文件；可选的本地 `manifest.json` 记录文件哈希，不上传 Git。

## 目录与依赖

- `docs/operation_manual.md`、`prompts/`、`docs/`：完整规范、每轮提示词与专题说明，关系见 [文档索引](README.zh.md)。
- `scripts/`、`src/`：唯一现行规划、服务、相机与复盘入口。无需安装用户级skills或访问其他项目工作副本。
- `config/`：去除本机值后的配置模板；须按首次指南填写现场信息。`config/init_state.json`和棋盘尺寸记录为空，须现场填写；`assets/templates/`中的流程候选参数仍须核对适用性。
- `board_position/`：当前图像90点标记说明及通用脚本；没有预置真实照片、像素、机器人XY或姿态锚点，使用时生成本次结果。
- `assets/models/g1/`：当前模型URDF/网格；`pyproject.toml` 的 `chess` 组：Python依赖及官方SDK固定提交。引擎单独安装，见[环境安装说明](environment_setup.zh.md)。
- Python解释器：使用用户已安装依赖的当前环境，直接运行`python scripts/…`，或使用解释器绝对路径。
- `runs/`：使用时创建，存放当前环境的新run、检查点、图像和日志；发布包不预置运行数据，旧token不能重放。

先按[环境安装说明](environment_setup.zh.md)安装`pyproject.toml` 的 `chess` 组及支持xiangqi的外部引擎，再配置现场网络/硬件。包内已包含模型及构建代码，无需`unitree_deploy`或旧机器工作副本。SDK是外部安装的必要依赖；`config/python_packages.json`是待填版本记录，不负责安装。现场配置JSON不会自动安装依赖或启动服务。

## 离线核验

```bash
cd /path/to/g1d_chess
source /path/to/robot-env/bin/activate
export CHESS_ENGINE=/path/to/fairy-stockfish
python scripts/verify_package.py
python scripts/check_environment.py
python -m unittest discover -s tests -p 'test_*.py'
python scripts/round_tools.py --help
python scripts/round_tools_left.py --help
```

这些命令不初始化DDS、不打开相机、不发布机器人命令。环境检查解析实际导入位置、模型资源并运行引擎UCI握手；不能证明现场相机、DDS或运动已验证。`verify_package.py` 检查文件结构、语法、入口链接和脚本清单；本地存在 `manifest.json` 时额外核对哈希，缺少清单也可运行。现场编辑配置引起哈希变化时保留本地修改记录，不能以改哈希代替配置核验。

## 开始一轮

从操作手册第4节起分阶段执行。每次独立工具调用设置本包workdir和绝对路径，`CHESS_PYTHON`为用户环境的解释器绝对路径（例如`CHESS_PYTHON="$(command -v python)"`），client、serial和run均在包内。优先复用本包ROOT下健康空闲且执行手正确的服务。已有服务不自动加载磁盘更新；切换前按真实控制、持物、腰锁和事务状态处理，不能用启动器探测环境。

规划前配置物理相机映射和误差context，核对现场scene、双臂init、实际腰锁及棋盘图像。source与target位姿仍独立估计；模板中的棋盘释放Z=.139m、同侧圆盘Z=.159m为当前流程约定，若现场不兼容，先按首次指南适配受影响的模板和实现。不能把历史测量当新设备默认。

当前主会话负责看图确认棋局、规则库合法性、预测匹配/保存、独立抓放估计、必要的引擎选招和六/七字段门禁评价。没有自动完成一轮的总控CLI。运动前准备好完整分支、预检和提交文件；执行期间不整理代码或资料。实现边界与超时行为见 [runtime](runtime.zh.md)，结束只做现行两项复盘。
