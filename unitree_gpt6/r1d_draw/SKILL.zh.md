---
name: r1d-draw
description: 为 Unitree R1-D（R1-A7-D）规划参考图绘画，测量纸面与颜料，记录标定，编译整幅作业与启动脚本并执行。用于绘画规划、试笔、现场测量、离线编译和真机绘画。
metadata:
  version: "0.1.0"
  language: zh-CN
---

# R1-D 参考图绘画

[English](SKILL.md) | 中文

安装与命令见 [README](README.zh.md)。

按照 **规划 → 测量 → 记录 → 编译导出 → 执行与检查** 完成任务。根据参考图设计笔画，测量实际布局，交付整幅编译作业和 `run.sh`。执行阶段由本地确定性代码连续运行，无需模型逐笔指挥。

使用仓库中的 `unitree_gpt6.r1d_draw.src`、`r1d-draw` 命令和[设备配置样例](config/device.example.json)。编辑和离线编译不启动硬件。后端适用于双臂各 7 轴的 R1-D；URDF 自行提供，须与本机一致。

## 与 G1-D 的不同

- **连接方式。** R1-D 的 DDS 带安全认证，工作站不能直接收发话题。所有读取和控制都由通过有线 SSH 放到机器人上运行的小程序完成。配置中填写机器人地址、网卡和 MAC；SSH 密码只从环境变量读取（默认 `R1D_SSH_PASSWORD`），不写入任何文件。
- **机器人接口。** `connect(config)` 返回的对象提供 `state()`、`photo(camera)`、`take_over()`、`move(route)`、`stop()`、`relax()`、`release()`、`status()`。真机和模拟机器人使用同一组函数。
- **关节。** 35 个电机槽位：升降 12、腰部偏航 13、左臂 15–21、右臂 22–28、头部 29/30、夹爪 31/33。不发送底盘和升降命令；腰部、头部和夹爪保持接管时的位置。
- **坐标系。** 位姿使用 `model.base_frame`（默认 `pelvis_link`），升降高度不进入坐标。腰部偏航是每个作业的固定标量 `waist_yaw`。
- **中断后继续。** 作业按批次记录进度，`run-task --resume` 从未完成的批次继续，不重画已完成的批次。

## 默认配置与已保存数据

优先级为 **用户本次明确要求 → 当前任务已接受的数据 → 新任务默认配置**。重新规划时保留已接受的纸面、材料坐标、接触高度、夹持、腰部和进入/收臂路线；用户要求修改或当前测量发现变化时，再记录修订并重新生成受影响产物。

| 项目 | 默认值 |
| --- | --- |
| 绘画臂 | 右臂；绘画批次中左臂保持 `entry_q14[-1]` 的前 7 个角度 |
| 试笔 | 使用指定试笔区域，优先选择同类纸张；未指定区域时将实物试笔列为待办 |
| 测量 | 复用仍有效的标定并独立核对；需要新测量时用干净笔刷接触测量或现场可靠工具 |
| 蘸色与洗笔 | 不内置固定规则。`strokes.json` 可选 `reload_every`（每几笔重新蘸色）和 `wash_station`（换色及结束时洗笔的材料站）；不填则每种颜色蘸一次、不洗笔 |
| 完成 | 沿 `park_q14` 收臂并继续带电保持；需要卸力时另行执行 `relax` |
| 覆盖 | 补非预期空隙，合并兼容笔画，允许少量边界重叠；保留参考图有意留白和叠色 |

## 工作流程

1. **先规划整幅画。** 分析构图、颜色区域、前后层次、连接关系和有意留白。保存完整颜色顺序、路径及覆盖范围，明确纸面 u/v 方向并保留比例。列出缺少的测量；不根据未标定照片推断机器人接触坐标或真实笔宽。

2. **测量纸面和工具。** 用 `observe` 保存当前关节与笔尖位姿，用 `photo` 保存各相机画面。采集至少 3 个分散、非共线的纸面锚点和至少 1 个独立核对点。测量动作通过 `take_over()` 和 `move(route)` 执行，每条路线先离线检查。记录笔尖工具变换，保存测量来源与接触证据。反馈 FK 不能判断笔刷是否接触，记录默认 `contact_verified=false`。

3. **测量颜料、水杯并试笔。** 保存每站的接近、接触/浸入、停留和抬离位姿。蘸料要达到可用刷毛深度，避开笔箍与容器底。记录试笔区域、实际笔宽、载色可用笔长、接触高度、夹持、腰部以及进入/收臂路线。相机画面之间没有曝光同步。

4. **记录并修正规划。** 将接受的实测值写进 `measurements.json`，已确认接触的记录填写 `contact_verified=true` 和 `source`。按笔宽和载色能力修改 `strokes.json`。用最终笔画生成 `preview.svg`；预览表示规划覆盖，实拍表示纸上结果。

5. **编译整幅作业和脚本。** 使用完整笔画、测量、设备配置、新 `job_id` 和新输出目录运行 `prepare`。它离线拟合纸面、生成完整作业、求解 IK、检查路线，导出 `job.json` 和 `run.sh`。在 `clearance_review` 中记录全程物理通道复核；离线检查只含关节限位和可选的工作空间盒，不含碰撞检测。

6. **最后执行交付脚本。** 获得真机执行授权且现场/当前姿态符合任务后，确认没有其他程序在控制机器人，单次启动 `run.sh`。全程只有一个运动控制者。照片另行采集。

7. **跟踪并检查。** 保存作业身份、文件哈希、进度记录、健康状态与实拍。客户端超时不代表取消：查询同一会话的 `status`，不重复提交。`job_completed=true` 且 `parked=true` 表示计划动作和收臂完成；纸面效果根据照片单独判断。

## 测量命令

```bash
r1d-draw init-task --output ./my-task
# 先填写 device.json。
export R1D_SSH_PASSWORD=...        # 只放在环境变量里
r1d-draw observe --config ./my-task/device.json --label rest --output ./my-task/rest.json
r1d-draw photo --config ./my-task/device.json --camera head --output ./my-task/head.png
```

需要运动的测量用 Python 接口：

```python
from unitree_gpt6.r1d_draw.src.robot import connect

robot = connect("./my-task/device.json")
robot.take_over()  # 原地接管并保持
robot.move({"points": [{"q": {"28": 0.25}, "duration_s": 3.0}]})
print(robot.state()["arms"]["right"])
robot.relax()  # 双臂缓慢卸力，夹爪继续夹持
robot.close()
```

## 作业交付与执行

```bash
r1d-draw prepare --config ./my-task/device.json \
  --strokes ./my-task/strokes.json --measurements ./my-task/measurements.json \
  --job-id painting-attempt-1 --output ./ready/painting-attempt-1
bash ./ready/painting-attempt-1/run.sh
```

交付参考图、源 `strokes.json` 与 `measurements.json`、拟合的 `calibration.json`、`device.json`、完整 `plan.json`、编译 `job.json`、`preview.svg`、`report.json`、`manifest.json` 和 `run.sh`。保留原始测量和旧画照片。执行设备上的 URDF 路径必须可用。修改源输入后重新导出，不直接编辑编译产物。

```bash
r1d-draw status  --config ./ready/painting-attempt-1/device.json
r1d-draw stop    --config ./ready/painting-attempt-1/device.json   # 取消当前路线，原地保持
r1d-draw relax   --config ./ready/painting-attempt-1/device.json   # 双臂缓慢卸力
r1d-draw release --config ./ready/painting-attempt-1/device.json   # 停止发命令，不卸力、不收臂
bash ./ready/painting-attempt-1/run.sh --resume          # 从未完成的批次继续
bash ./ready/painting-attempt-1/run.sh --resume --lift   # 笔还在纸上或材料里时，先竖直抬起
```

停止、故障或反馈丢失后控制器不再发命令，电机保持最后一帧：这不是受控保持，也不会自动撤退或重播。继续之前先看 `status` 和现场，再决定 `--resume`。每个 `job_id` 只能开始一次；从头重画时用新编号和新目录重新 `prepare`。

## 坐标与执行约定

- **坐标和笔尖。** `q14` 按 `[左7轴,右7轴]` 保存关节角，单位为弧度；夹爪 `grip_q2` 和腰部 `waist_yaw` 单独保存。位姿 `x,y,z` 单位为米，`roll,pitch,yaw` 为 XYZ 欧拉角、单位为弧度，位于 `model.base_frame`。笔尖位置由末端 frame 乘以工具变换得到，腕部位置不是笔尖位置。
- **纸面和材料。** 支持平面纸张，根据实测点确定纸面的位置和倾斜，计算各处落笔高度。材料站保存完整的实测位姿序列和 `contact_index`。
- **运动方式。** 一笔之内笔尖匀速连续走完，只在急转处停顿；空中转移沿直线加密后连续走；落笔、离开和蘸料单独慢速执行。速度在 `device.json` 的 `motion` 中调整，默认值见 [README](README.zh.md#运动参数)。
- **笔尖偏差修正。** 每个批次到达其上方第一点后读取实际关节，把测得的笔尖偏差从该批次其余目标中减去。`--no-correction` 关闭。
- **运行检查。** 控制器检查反馈新鲜度、其他命令发布者、电机故障、温度、跟随偏差、速度和发布间隔；任何一项失败都停止发命令并在 `status` 中报告。

## 输出要求

准备阶段链接源规划、测量、预览、编译作业、报告和启动脚本，说明笔画数、材料及修改参数，区分离线编译和实机执行。执行阶段说明作业身份、进度、终态、保持/卸力/释放状态和观察到的绘画结果。明确列出尚未完成的具体测量或执行步骤。
