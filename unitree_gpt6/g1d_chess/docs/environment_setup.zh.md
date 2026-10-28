# 安装独立运行环境

[English](environment_setup.md) | 中文

本项目不需要安装或保留`unitree_deploy`项目。运动学模型构建在本包`g1_model.py`中，URDF与网格在`assets/models/g1/`；FK、IK和重力补偿使用Pinocchio及SciPy。`unitree_sdk2py`仍是机器人DDS通信的必要依赖，通过`pyproject.toml` 的 `chess` 组安装官方固定提交，不再随包复制SDK源码。

## 1. 建立Python环境

基线为Linux x86_64、Python 3.10。其他系统、架构或Python版本需重新核验原生库与wheel兼容性。先在系统包管理器安装Python 3.10及venv、Git；编译引擎还需要C++编译器和Make。保留完整仓库，在 `unitree_gpt6/g1d_chess/` 目录执行（依赖声明在仓库根，不支持只复制技能目录后安装）：

```bash
python3.10 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e '../..[chess]'
source .venv/bin/activate
```

也可以直接使用用户已有的虚拟环境：先激活，再用`python -m pip install -e '../..[chess]'`补齐依赖，无需创建包内`.venv`。

`pyproject.toml` 的 `chess` 组固定直接Python依赖及SDK提交，不是所有平台的完整传递依赖锁。首次安装需要访问PyPI和GitHub；离线部署时应事先为目标平台准备wheel和引擎。`pin`是Pinocchio发行包，不要安装同名的其他`pinocchio`发行包。当前运行路径不再需要CasADi、Pinocchio CasADi绑定、Meshcat或`unitree_deploy`。

如果OpenCV导入缺少`libGL.so.1`或GLib，按目标发行版安装运行库，例如Debian/Ubuntu的`libgl1`、`libglib2.0-0`。如果Cyclone DDS没有匹配wheel，或报`Could not locate cyclonedds`，按[官方Python SDK安装说明](https://github.com/unitreerobotics/unitree_sdk2_python#installation)安装Cyclone DDS 0.10.x原生库，并设置实际`CYCLONEDDS_HOME`后重试；Python绑定固定为0.10.2。JSON环境记录不会安装这些库。

## 2. 单独安装象棋引擎

`pyffish`负责棋规校验，不提供供子进程调用的引擎可执行文件。另安装支持`xiangqi`的[Fairy-Stockfish](https://github.com/fairy-stockfish/Fairy-Stockfish)，使用适合目标CPU的官方构建或从源码编译。例如：

```bash
git clone https://github.com/fairy-stockfish/Fairy-Stockfish.git /path/to/Fairy-Stockfish
make -C /path/to/Fairy-Stockfish/src build ARCH=x86-64 largeboards=yes
export CHESS_ENGINE=/path/to/Fairy-Stockfish/src/stockfish
```

也可在Linux x86_64直接下载官方`fairy_sf_14`的[largeboard通用构建](https://github.com/fairy-stockfish/Fairy-Stockfish/releases/download/fairy_sf_14/fairy-stockfish-largeboard_x86-64)，保存到选定路径，赋予执行权限，再把`CHESS_ENGINE`设为其绝对路径。该构建支持象棋90点棋盘；不能用不支持largeboard/xiangqi的构建替代。

源码目录应使用尚未存在的实际安装路径；`ARCH=x86-64`适用于上述x86_64基线，其他CPU按引擎文档选择。记录实际源码提交和编译参数，升级后重新核验。引擎不要求位于本包内，也不能使用仅支持国际象棋的普通Stockfish代替。引擎路径由`CHESS_ENGINE`选择；未设置或为空时，环境检查查找`PATH`中的`fairy-stockfish`。主会话调用引擎时也使用该变量或相同的PATH命令，不拼接包内二进制路径。

## 3. 离线验证

在已激活Python环境并配置`CHESS_ENGINE`的同一终端执行：

```bash
python -m pip check
python scripts/check_environment.py
python scripts/verify_package.py
python -m unittest discover -s tests -p 'test_*.py'
python scripts/round_tools.py --help
python scripts/round_tools_left.py --help
python scripts/internal_service.py --help
python scripts/internal_service_left.py --help
```

环境检查报告实际导入位置、依赖版本、87处网格引用、14关节顺序、模型FK/重力计算、引擎UCI与`xiangqi`支持，以及初始局面的44个合法着法；不初始化DDS、不连接相机、不启动机器人服务。测试另核对合成关节状态的FK和重力结果及IK。源码不再导入`unitree_deploy`，也不需要旧机器项目路径或用户级skills。

发布文件校验不遍历`.venv/`；虚拟环境不属于发布清单。更换Python依赖或引擎后重跑以上核验。目标设备是否可控制仍需按[首次开局配置](first_game_setup.zh.md)确认网络、相机映射、夹爪方向、scene和双臂init，再按[round_plan](../prompts/round_plan.zh.md)生成并预检本轮计划。

## 4. 独立运行的范围

安装上述依赖后，本包的规划、服务、采图、检查点和复盘入口可直接使用，无需安装整个`unitree_deploy`。机器人通信通过已安装的官方SDK完成；因此“独立”表示脱离旧项目目录，并不表示无需SDK、机器人硬件或现场配置。模型文件是执行所需的静态资源，不能随着第三方源码一起删除。

主会话仍负责从当前图像确认棋局、估计抓放位置、匹配预测/选招和判断检查点；当前没有自动识图并完成整轮的总控CLI。离线通过不代表已完成实机抓放验证，运动流程及已知实现差异见[runtime](runtime.zh.md#实现核对边界)。
