# 文档索引

[English](README.md) | 中文

| 文档 | 内容 |
|---|---|
| [operation_manual.zh.md](operation_manual.zh.md) | 主规范：环境、接口与坐标、启动接管、每轮规划、执行与门禁、复盘、故障处理 |
| [round_script_inventory.zh.md](round_script_inventory.zh.md) | 子命令、入口脚本和实现模块的对应关系 |
| [robot_service_architecture.zh.md](robot_service_architecture.zh.md) | 象棋服务如何建立、运动如何下发到 R1-D |
| [r1d_scene_parameters.zh.md](r1d_scene_parameters.zh.md) | TCP、夹持点、棋盘和场景参数的含义与确定方法 |
| [r1d_joint_tracking_feedforward.zh.md](r1d_joint_tracking_feedforward.zh.md) | 臂力矩前馈的作用、控制律和跟随限制 |
| [r1d_twist_dds_topics.zh.md](r1d_twist_dds_topics.zh.md) | 象棋服务用到的底盘与升降反馈及判据 |

新机器人、新场地的上线流程在 [../setup/README.zh.md](../setup/README.zh.md)：本机环境、相机检查、机械臂精度标定、头部相机标定、下棋环境参数和棋盘标定。内置的机器人运行时（唯一向机器人下发命令的进程）见 [../third_party/robot_runtime/README.zh.md](../third_party/robot_runtime/README.zh.md)。每轮的执行顺序在 [../prompts/round_plan.zh.md](../prompts/round_plan.zh.md)，读棋和棋盘重测在 [../board_position/README.zh.md](../board_position/README.zh.md)，配置文件说明在 [../config/README.zh.md](../config/README.zh.md)。

文档里标为“参考”的数值来自一台参考机器人的实测，只说明量级，不能当作新机器人或新场地的实测值。
