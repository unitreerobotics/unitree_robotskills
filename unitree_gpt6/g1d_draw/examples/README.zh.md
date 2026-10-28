# G1-D 接入示例

[English](README.md) | 中文

先在仓库根目录安装，再进入 `unitree_gpt6/g1d_draw` 运行示例。以下三个示例只读状态或图像，不接管机械臂，也不需要 URDF。

| 文件 | 用途 |
| --- | --- |
| [check_connection.py](check_connection.py) | 检查网卡、DDS 反馈、相机配置服务及每路图像；输出逐项结果 |
| [read_joints.py](read_joints.py) | 显示并保存双臂、腰部和 Dex1 的当前关节值 |
| [capture_images.py](capture_images.py) | 订阅相机 JPEG 流，保存 PNG 原图和来源记录 |
| [execute.py](execute.py) | 执行已编译且审核过的作业；这个示例会驱动机器人 |

## 安装和配置

```bash
# From the repository root / 在仓库根目录执行
python -m pip install -r unitree_gpt6/g1d_draw/requirements.txt
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
cd unitree_gpt6/g1d_draw
```

建议在新的 Python 3.10 虚拟环境中安装。[requirements.txt](../requirements.txt) 固定所有依赖版本和 SDK 提交，图像处理使用 `opencv-python` 与 PyZMQ。CycloneDDS 安装失败时，按 [SDK 安装说明](https://github.com/unitreerobotics/unitree_sdk2_python#installation) 构建并设置 `CYCLONEDDS_HOME`。

复制并编辑 [device.example.json](../config/device.example.json)，或直接用以下命令的 `--interface` 指定本机有线网卡。只读关节示例不使用 `model` 或增益字段。

```bash
ip -br addr
IFACE=enp4s0
CAMERA_HOST=192.168.123.164
ip route get "$CAMERA_HOST"
```

这里的网卡和相机地址是部署示例，应换成自己的值；`CAMERA_HOST` 不应填写本机地址，有线直连通常不填写网关。DDS 通过 `--interface` 绑定网卡；相机 TCP 连接按系统路由选择网络，确认 `ip route get` 中的 `dev` 符合预期。相机服务器可位于机器人或独立图像主机，其地址不必等于 DDS 设备地址。

## 1. 连通性测试

```bash
python examples/check_connection.py \
  --config config/device.example.json --interface "$IFACE" \
  --camera-host "$CAMERA_HOST" --timeout 5 \
  --output ./captures/connection.json
```

测试会实际读取新鲜关节反馈，请求相机配置，再逐路接收并解码一帧 JPEG。全部通过时退出码为 `0`，任一失败为 `1`；JSON 保留各项结果。可只提供 `--config` 测 DDS，或只提供 `--camera-host` 测相机。配置服务端口可用 `--camera-config-port` 修改。

没有配置服务、但已知图像端口时，直接测对应流：

```bash
python examples/check_connection.py \
  --stream "head=tcp://$CAMERA_HOST:55555" \
  --output ./captures/head-connection.json
```

## 2. 读取和记录关节

```bash
python examples/read_joints.py \
  --config config/device.example.json --interface "$IFACE" \
  --samples 3 --interval 0.5 --output ./captures/joints.json
```

终端显示关节名、槽位、弧度、角度与速度；JSON 保存 `q14`、`dq14`、腰部、夹爪及时间信息。采样间隔是读取间隔，不是 DDS 发布频率。

| 数据 | 顺序与单位 |
| --- | --- |
| `q14` / `dq14` | 左臂 7 轴、右臂 7 轴；rad / rad/s；LowState 槽位 15–28 |
| 每臂内部顺序 | shoulder_pitch、shoulder_roll、shoulder_yaw、elbow、wrist_roll、wrist_pitch、wrist_yaw |
| `waist_q3` | yaw、roll、pitch；rad；槽位 12、13、14 |
| `grip_q2` | 左、右；设备原生单位，不是百分比；内置 Dex1 槽位 31、33 |

外置 Dex1 在配置中填写左右状态、命令 topic；反馈从各自状态 topic 读取。读取脚本只调用 `connect/read/health/close`，不创建命令发布器。

## 3. 读取和保存相机图像

相机侧需运行 [TeleImager](https://github.com/unitreerobotics/teleimager) 或兼容服务并启用 ZMQ。读取方法是：向配置端口发送 `GET_DATA` 获取 JSON 配置，再订阅各路 `zmq_port` 的单条 JPEG 消息。此接口独立于关节 DDS；安装 SDK 不会自动启动相机服务。

以下端口是常见配置，以连通性测试返回的实际配置为准：

| 相机 | ZMQ 端口示例 | 图像说明 |
| --- | --- | --- |
| `head_camera` | 55555 | 可能是双目横向拼接；看配置中的 `binocular` 和尺寸 |
| `left_wrist_camera` | 55556 | 左腕原图 |
| `right_wrist_camera` | 55557 | 右腕原图 |
| 相机配置服务 | 60000 | REQ/REP，返回各路配置，不是图像流 |

```bash
python examples/capture_images.py \
  --stream "head=tcp://$CAMERA_HOST:55555" \
  --stream "left_wrist=tcp://$CAMERA_HOST:55556" \
  --stream "right_wrist=tcp://$CAMERA_HOST:55557" \
  --warmup-frames 3 --timeout 10 --output ./captures/images-1
```

生成 `head.png`、`left_wrist.png`、`right_wrist.png` 和 `capture.json`，其中记录地址、图像尺寸、哈希和主机接收时间。可减少 `--stream` 项，只读需要的相机。示例保留整幅原图，不自动裁剪、镜像或推断图像方向。多路没有曝光同步；关节快照和图片也不是同步数据。

## 常见问题

| 现象 | 检查项 |
| --- | --- |
| 找不到网卡 | 用 `ip -br addr` 找实际名称，检查网线和地址配置 |
| DDS 超时 | 网卡、domain、LowState topic/类型、机器人状态发布、DDS 网络配置；能 ping 通不代表 DDS 已通 |
| 相机配置请求超时 | 相机服务器地址、60000 或自定义配置端口、服务是否启动；已知流端口可单独测试 |
| 配置成功但某路没有图像 | 该路是否 `enable_zmq=true`、实际端口、相机是否连接并发布 JPEG |
| 收到数据但解码失败 | 本示例要求单消息 JPEG/ZMQ，不适用于 WebRTC、RTSP 或其他封包格式 |
| 输出已存在 | 换一个文件名或新目录，示例保留已有记录 |

## 执行动作

关节快照是反馈记录，不是可执行作业。要测量移动或绘画，先按主 [README](../README.zh.md) 准备本机模型、路线及测量数据，再使用 `compile-probe/probe` 或导出的 `run.sh`。Python API 见 [execute.py](execute.py)。

执行示例与主命令使用相同的控制流程，完成或停止后继续保持。运行中按 Ctrl-C 停止；已停止或完成后按 Ctrl-C 释放，也可从另一终端使用主命令的 `status`、`stop` 和 `release`。
