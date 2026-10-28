# 配置模板与字段说明

[English](README.md) | 中文

本目录不保存打包机器的用户名、绝对路径、网卡、设备IP、版本快照、实测位姿或历史误差组。依赖安装见仓库根目录 pyproject.toml 的 chess 组及[环境安装说明](../docs/environment_setup.zh.md)；首次部署再按 [全新G1D开局配置](../docs/first_game_setup.zh.md) 收集现场信息，填写需要使用的字段。

每个JSON的`_field_help`给出说明和示例，**示例只演示格式**。`192.0.2.10`为文档专用地址；`/opt/…`为虚构安装路径。不要复制这些值作为实际配置。未知设备参数为`null`，不拿0代替。操作流程中的次数、5mm下调规则及“不设距离上限”等已确认策略仍保留。

`gripper_open_sign=1`是内置Dex1左右爪共用的默认方向：正角度张开、0闭合，普通环境迁移直接沿用。两份环境JSON保留这个默认；它们只作记录，启动器和服务也独立默认+1，支持环境变量及CLI覆盖。该字段无需作为新现场的缺失测量项。

| 文件 | 保存什么 | 当前代码如何使用 |
| --- | --- | --- |
| `environment.example.json` | 环境字段说明与空白模板 | 不自动读取；根据填写结果显式设置环境变量/CLI |
| `package_environment.json` | 同样格式的部署环境记录入口 | 不自动读取；可在现场填实际信息；它不是动态状态快照 |
| `python_packages.json` | 待填写的目标Python及依赖版本 | 不自动安装、不锁版本；用check_environment验证实际解释器 |
| `chess_camera_policy.json` | 规划头图逻辑键、待确认的腕流映射和依据 | camera_policy模块按进程缓存读取；左右映射为null时拒绝生成相机选择 |
| `manipulation_profile.json` | 通用流程、策略和待填姿态/工具参数 | 说明文件，不自动配置规划器或服务；实际参数还需同步到本轮spec/请求 |
| `init_state.json` | 双臂实测关节、模型FK、夹爪及来源记录；发布时为空 | round_tools按物理执行手读取init XYZ/RPY；未填写不能生成完整计划 |
| `board_parameters.json` | 棋盘与棋子尺寸的空白测量记录 | 说明记录，不自动应用到规划器或服务 |
| `scene.example.json` | 待填写的表面/工具包络/净空参数 | 不自动加载；生成只有`scene`键的请求再调用set_scene |
| `xy_error_learning.json` | 待填写的兼容环境context_id | 无显式spec/CLI context时读取；null会报错，不默认沿用旧组 |

scene数值优先使用用户给定且与当前坐标系及工具兼容的数据；用户未给定任何现场信息时，全部scene字段由模型自行估计，部分给定时由模型补齐其余字段。使用可用图像、机器人状态、工具模型、尺寸资料及合理假设，记录来源、方法及不确定度，不以用户补充资料或额外确认为前置条件。具体方法与设置后核验见 [首次配置第5节](../docs/first_game_setup.zh.md#5-配置现场几何双臂init和动作参数)。模型可主动收集任务范围内可用资料，并在记录假设与误差范围后完成估计；估计值与采用的默认值须标注来源，不称作实测。评估结果和请求保存在本次run，通用模板保留空白。

`null`有两种明确区分的用途：现场参数字段中表示尚未填写；`max_high_offset_m`、`max_target_offset_m`和`policy.xy_correction_distance_limit_m`中表示既有策略不设距离上限。不要用统一的“所有null都填数值”规则处理。

环境字段`interface`记录目标机器的DDS网卡。`python_executable`记录用户环境的解释器路径；运行时激活该环境或显式调用此解释器，不由配置JSON自动切换。两份环境JSON采用同一模板结构，实际部署以你填写的记录为准，不靠文件名推断已生效。

现场信息可写到本地副本或本次run记录；若本地副本在其他路径，现有相机策略和误差工具不会自动寻找它，须明确同步到上述实际读取文件或用工具已有的显式context参数。
