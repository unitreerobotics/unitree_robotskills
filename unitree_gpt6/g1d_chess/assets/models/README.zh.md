# G1模型资源

[English](README.md) | 中文

`g1/`保留本项目运动学所需的URDF与网格，迁移时必须随包携带。资源来自原有G1描述；发布文件哈希保留在发布清单，模型上游说明见[g1/README](g1/README.zh.md)。`g1_body29_hand14.urdf`由Unitree的G1带手模型修改，不能随意换成不同关节或不同末端定义的URDF。

构建代码为`src/g1d_chess/robot/g1_model.py`：锁定下肢、腰部及手指，保留物理左7加右7关节，末端为腕yaw沿局部X偏移50mm。与原模型的合成状态FK/重力结果在tests中核对；这些不是本机实测状态。依赖和运行环境见[环境安装说明](../../docs/environment_setup.zh.md)。
