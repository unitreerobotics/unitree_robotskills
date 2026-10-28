# 通用一轮走棋的脚本与模块

[English](round_script_inventory.md) | 中文

对应[skill](../SKILL.zh.md)、[操作手册](operation_manual.zh.md)及[round_plan](../prompts/round_plan.zh.md)。所有路径相对仓库根。`scripts/`是实际可调用的入口，`src/g1d_chess/`是这些入口加载的实现，二者不重复维护同一份业务代码。Python入口通过`_bootstrap.py`定位当前checkout的src，不需要安装另一个工作副本。

## 直接使用的入口

| 入口 | 用途与输入输出 |
| --- | --- |
| `scripts/start_service.sh` | `--status`只查进程；确需启动时选择`--arm left/right`，以现场网卡、相机和夹爪方向启动服务。启动会清理匹配旧服务，不用于故障恢复。 |
| `scripts/robot_client.py` | 单一服务请求CLI：ping、get_state、采图、腰锁、scene、完整预检、enable与execute及按需释放；返回真实回执。 |
| `scripts/round_tools.py` | 右手init/build/tokens：创建run和模板、读取本轮spec/新观测、生成完整分支及提交文件。init命令不移动机器人。 |
| `scripts/round_tools_left.py` | 左手对应入口，要求左服务新观测，绑定物理执行手与expected_active_arm。 |
| `scripts/move_constraints.py` | 核对格名与source首选手；从外部已确认的合法UCI列表生成searchmoves，不识图、不生成棋规合法着法、不做IK。 |
| `scripts/xy_error_history.py` | 规划前show读取同手兼容context的参考；复盘finish合并真实FK/视觉结论和JSONL，不自动补偿坐标。 |
| `scripts/serial_checkpoint.py` | 执行期间next/submit；提交当前检查点评价，或用户指定的抓前failure。不能等execute返回才处理。 |
| `scripts/review_pose_audit.py` | 结束后对真实结果中的两个事件快照离线计算三种FK误差。 |

## 按需入口与环境检查

| 入口 | 使用时机 |
| --- | --- |
| `scripts/mark_board.py` | 规划前从新图init/build/query90点；不采图、不输出机器人XY。 |
| `scripts/capture_cameras.py` | 现场配置或相机排查时独立采图；正常单轮用robot_client，execute期间不用独立请求打断事务。 |
| `scripts/start_left_service.sh` | start_service --arm left的便捷包装。 |
| `scripts/internal_service.py`、`scripts/internal_service_left.py` | 启动器调用的实际进程入口；手工调用同样需要现场环境参数。 |
| `scripts/check_environment.py` | 离线检查已安装依赖、模型、引擎及棋规库，不连接硬件。 |
| `scripts/verify_package.py` | 离线检查发布文件、语法、skill/文档链接及本地导入闭包。 |

## 现有实现模块

| 目录 | 文件及作用 |
| --- | --- |
| `src/g1d_chess/chess/` | round_tools、round_tools_left、move_constraints、xy_error_history、review_pose_audit：计划构建与事后分析。 |
| `src/g1d_chess/robot/` | internal_service及left入口、dex1_internal、arm_roles：事务、DDS、双臂角色及腰锁。 |
| 同上 | g1_model、motion_harness、board_guard：模型、FK/IK、轨迹与scene检查。 |
| 同上 | fast_correction、release_alignment、grasp_vision：抓取修正、释放门禁及仍需随附的兼容视觉配置。 |
| 同上 | serial_checkpoint、checkpoint_runner、camera_policy：串行评价/schema和物理相机映射；当前流程不启动checkpoint_runner中的旧后台worker。 |
| 同上 | pose_audit、async_event_log、robot_client：原始快照、非等待日志和服务客户端。 |
| `src/g1d_chess/cameras/` | imageclient、capture_cameras：相机连接和按需采图；本地配置在config，成功请求会更新被git忽略的回退文件。 |
| `src/g1d_chess/board/` | mark_board：图像点位拟合、验证和标注。 |
| `src/g1d_chess/paths.py` | 从当前源码位置定位仓库根；各子目录的__init__.py只用于Python包导入。 |

完整文件清单及本地导入关系见[JSON索引](round_script_inventory.json)，共46个入口、模块及包支持文件（包含包初始化文件）。目录移动后运行测试和包检查，不把导入通过当作实机验证。

## 一轮调用顺序与数据

1. 根据实际服务和当前授权核对状态与腰锁；核对各自init距离和物理执行手。
2. 对应手round_tools init创建唯一runs目录；采新头图和同批状态，确认完整棋局及黑方实际着法。
3. 匹配本局预测，必要时调用CHESS_ENGINE及pyffish；按需先用mark_board完成当前图像点位标记。
4. 读取同手context参考，独立估计source/target XYZ/RPY及邻子，填写spec、scene和现场依据。
5. build完整分支；普通走子prepare_transaction，吃子prepare_sequence联合两段；tokens生成一次execute提交包及各stage prepared。
6. enable后一次execute，在另一会话next/看本次两图/submit；第一段吃子释放后连续进入第二段，不中途预检或回init。
7. 完成后仅采新头图判断落子，并计算两个事件三种FK误差，使用finish合并到本轮docs和共享JSONL。普通结束保留腰锁。

运动前备齐`config/init_state.json`、相机策略/连接参数、scene、当前状态和图像、spec、资产模板、新预检token及评价结构。运行结果只放本轮`runs/`；不预置历史局面、坐标或token。模板的流程候选参数须按现场核对。主会话负责图像判断与引擎/预测编排，仓库没有全自动单轮总控。

XY修正次数、能夹即可和释放规则按round_plan；当前实现与操作要求的具体差异见[runtime](runtime.zh.md#实现核对边界)。正常复盘不重建点位、跑测试或关闭服务。
