# 离线验证与现场边界

[English](validation.md) | 中文

离线核验命令见[包使用说明](package_setup.zh.md)，全部从当前部署包根运行。发布验证记录保存在[package_validation.json](package_validation.json)，其中的结果仅说明被核验包的情况，不能代替目标环境的实际核验。

文件核验覆盖发布文件SHA256、JSON/Python/shell语法、包内文档链接和脚本路径。离线测试覆盖规划生成、检查点提交、棋盘标记与误差日志；环境检查核对包内模型与已安装SDK的实际导入位置、URDF网格和引擎握手。棋盘标记测试使用合成图像，不证明实际棋盘精度。

离线验证不连接DDS、不调用相机、不启动机器人服务或运动。操作规范与源码实现的差异见[runtime](runtime.zh.md#实现核对边界)。新设备须完成[首次配置](first_game_setup.zh.md)，确认真实相机、腰锁、几何与当前完整plan，不能以离线通过代替现场验证。
