# robot_runtime：机器人运行时

[English](README.md) | 中文

机器人运行时是下棋链路里**唯一向机器人下发命令的进程**：它独占 DDS，读取电机反馈，接收关节目标并以固定频率写入 LowCmd；象棋服务、`goinit` 和 setup 工具都通过本机 IPC 与它通信，自己不直接发命令。

| 内容 | 数量 | 说明 |
|---|---|---|
| `src/robot_runtime/` | 64 个模块 | 配置解析、运行时服务端与 IPC、Unitree SDK2 驱动、R1-A7-D 本体和 Dex1 组件、数据结构 |
| `configs/` | 8 个 YAML | R1-A7-D + Dex1 的 experiment、本体关节组、夹爪、底盘、传感器定义 |
| `LICENSE` | — | Apache License 2.0 |

## 使用

不需要单独安装。`config/runtime.json` 的 `runtime_root` 指向本目录（相对包根）：

```bash
bash r1d runtime config                              # 解析配置，不连机器人
python scripts/install_runtime_service.py --write    # 生成 systemd 用户单元 r1d-runtime.service
systemctl --user start r1d-runtime.service           # 启动；建立控制会话之前不给机器人发命令
```

依赖见包根 `requirements.txt`（`logging_mp`、`msgspec`、`PyYAML`、`pyzmq`、`cyclonedds`）以及 R1-D SDK `unitree_sdk2py`。

## 范围

只包含 R1-A7-D + Dex1、`mobile_wbc` 控制模式、相机关闭这一种配置实际用到的部分：

- 不含遥操作策略、重定向、录制、仿真，以及其他机型专用的重力补偿。
- 不含相机驱动：象棋服务用 `r1d_chess.cameras.r1d_cameras` 直接取图，所以 `config/runtime.json` 的 `camera_enabled` 必须保持 `false`。

## 来源与改动

本目录的代码取自一个采用 Apache License 2.0 的上游机器人控制项目（提交 `54690e0c30cde76f95a44bde228d454846201cb7`），做了三处改动：

1. 只保留上述范围内的模块和配置。
2. Python 包名改为 `robot_runtime`，代码中的包名、线程名和 IPC 端点前缀随之改名；逻辑未改。
3. `src/robot_runtime/runtime/robot_hardware.py` 不再在模块顶部导入相机驱动（改为仅类型检查时导入），这样不装相机客户端库也能加载。
