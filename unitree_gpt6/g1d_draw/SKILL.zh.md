---
name: g1d-draw
description: 为 Unitree G1-D 规划参考图绘画，测量纸面与颜料，记录标定，编译整幅作业与启动脚本并执行。用于绘画规划、试笔、现场测量、离线编译和真机绘画。
metadata:
  version: "2.0.0"
  language: zh-CN
---

# G1-D 参考图绘画

[English](SKILL.md) | 中文

安装与命令见 [README](README.zh.md)。

按照 **规划 → 测量 → 记录 → 编译导出 → 执行与检查** 完成任务。根据参考图设计笔画，测量实际布局，交付整幅编译作业和 `run.sh`。执行阶段由本地确定性代码连续运行，无需模型逐笔指挥。

使用仓库中的 `unitree_gpt6.g1d_draw.src`、`robot-paint-execute` 命令和[设备配置样例](config/device.example.json)。按 [README.zh.md](README.zh.md) 从仓库根目录执行 `python -m pip install -r unitree_gpt6/g1d_draw/requirements.txt` 安装本技能依赖，并将 `unitree_gpt6/g1d_draw/scripts` 加入 `PATH`、仓库根目录加入 `PYTHONPATH`；连通性、相机、数据格式和默认姿态也见该说明。编辑和离线编译不启动硬件。后端适用于固定底座、双臂各 7 轴的 G1-D、URDF 与设备配置需匹配本机。

## 默认配置与已保存数据

优先级为 **用户本次明确要求 → 当前任务已接受的数据 → 新任务默认配置**。重新规划时保留已接受的纸面、材料坐标、接触高度、夹持、腰部和工作/休息姿态；用户要求修改或当前测量发现变化时，再记录修订并重新生成受影响产物。

| 项目 | 默认值 |
| --- | --- |
| 绘画臂 | 右臂；绘画批次中左臂保持 `entry_q14[-1]` 的前 7 个角度 |
| 试笔 | 使用指定试笔区域，优先选择同类纸张；未指定区域时将实物试笔列为待办 |
| 测量 | 复用仍有效的标定并独立核对；需要新测量时采用现场可靠工具或干净笔刷接触测量 |
| 工作/休息姿态 | [src/postures.py](src/postures.py) 中可编辑的 `default_work_q14()` 和 `default_rest_q14()` |
| 夹持 | `default_grip_q2()=[0.0,0.0]`，左/右、设备原生单位；已有任务保留已接受值 |
| 完成 | 沿审核后的抬离和收臂路线返回，继续带电保持 |
| 覆盖 | 补非预期空隙，合并兼容笔画，允许少量边界重叠；保留参考图有意留白和叠色 |

新任务预填 `entry_q14=[休息位,工作位]`、`park_q14=[null,休息位]`。入口首点代表预期实际起点；先读当前关节，有差异就更新完整入口路线。编译时用最后抬离点替换 `null`。默认姿态是可编辑目标，完整路线仍要检查双臂和工具净空。

## 工作流程

1. **先规划整幅画。** 分析构图、颜色区域、前后层次、连接关系和有意留白。保存完整颜色顺序、路径及覆盖范围，明确纸面 u/v 方向并保留比例。列出缺少的测量；不根据未标定照片推断机器人接触坐标或真实笔宽。

2. **测量纸面和工具。** 用 `observe` 保存当前关节与工具反馈。采集至少 3 个分散、非共线的纸面锚点和至少 1 个独立核对点。使用审核后的 `compile-probe/probe` 路线或外部工具。记录笔尖工具变换，根据实测点确定纸面的位置和倾斜程度，并保存测量来源与接触证据。反馈 FK 不能判断笔刷是否接触，采样默认 `contact_verified=false`。

3. **测量颜料、水桶并试笔。** 保存每站的接近、接触/浸入、停留和抬离路线。蘸料要达到可用刷毛深度，避开笔箍与容器底。记录试笔区域、实际笔宽、载色可用笔长、接触高度、夹持、腰部以及工作/休息路线。按现场相机选择视角，或由操作者观察；不固定相机名称和图像方向。

4. **记录并修正规划。** 将接受的实测值写进 `measurements.json`，已确认接触的记录填写 `contact_verified=true` 和 `source`。按笔宽和载色能力修改 `strokes.json`，合并可连接的同色路径、删除低效短笔、修正空隙和不当覆盖。长笔不能超出可用载色长度。用最终笔画生成 `preview.svg`；预览表示规划覆盖，实拍表示纸上结果。

5. **编译整幅作业和脚本。** 使用完整笔画、测量、设备配置、新 `job_id` 和新输出目录运行 `prepare`。它离线拟合纸面、生成完整作业、求解 IK、检查路线，导出 `job.json` 和 `run.sh`。包括完整进入、转移、接近、接触区间、抬离和收臂路线，在 `clearance_review` 中记录全程物理通道复核。

6. **最后执行交付脚本。** 获得真机执行授权且现场/当前姿态符合任务后，释放旧控制器，单次启动 `run.sh`。全程只有一个运动控制者，固定夹持和腰部，绘画批次冻结非绘画臂。运行时不调用模型、求解 IK 或自动取图；照片另行采集。如需中途检查或测量，单独规划作业。

7. **跟踪并检查。** 保存作业身份、文件哈希、尝试记录、进度、健康状态与实拍。超时后查询原尝试，不重复提交。`job_completed=true` 且 `parked=true` 表示计划动作和收臂完成，当前健康状态说明保持情况；纸面效果根据照片单独判断。

## 测量命令

在已安装的仓库环境执行：

```bash
robot-paint-execute init-task --output ./my-task
# 先填写 device.json 和审核后的 probe.template.json。
robot-paint-execute observe --config ./my-task/device.json \
  --label rest --output ./my-task/rest-observation.json
robot-paint-execute compile-probe --config ./my-task/device.json \
  --probe ./my-task/probe.template.json --output ./my-task/probe.json
robot-paint-execute probe --config ./my-task/device.json --probe ./my-task/probe.json
```

`capture_indices` 指定采样动作点；悬空拍照点同时放入 `lifted_capture_indices`，默认 `[]`，必须是前者的子集。悬空采样只记录 XYZ、姿态和关节目标偏差，不设置到位门槛；未标记点保留接触检查。记录位于状态目录的 `attempts/<hash>/captures/`，将已接受的值和来源填入测量输入。

测量正常结束后收臂并保持。查询状态、明确释放后才能交给下一控制器。重复测量使用新尝试编号并重新编译。

## 作业交付与执行

```bash
robot-paint-execute prepare --config ./my-task/device.json \
  --strokes ./my-task/strokes.json --measurements ./my-task/measurements.json \
  --job-id painting-attempt-1 --output ./ready/painting-attempt-1
bash ./ready/painting-attempt-1/run.sh
```

交付参考图、源 `strokes.json` 与 `measurements.json`、拟合的 `calibration.json`、`device.json`、`build_plan.py`、完整 `plan.json`、编译 `job.json`、`preview.svg`、`report.json`、`manifest.json` 和 `run.sh`。保留原始测量和旧画照片。执行设备上的 URDF 路径必须可用。

导出启动脚本定位同目录任务并调用 `robot-paint-execute run-task --task ...`，检查作业和 URDF 哈希后执行；不重新测量、自动编译、更换编号或重试。修改源输入后重新导出，不直接编辑编译产物。

```bash
robot-paint-execute status --config ./ready/painting-attempt-1/device.json
robot-paint-execute stop --config ./ready/painting-attempt-1/device.json
robot-paint-execute release --config ./ready/painting-attempt-1/device.json
```

前台控制器在完成或停止后继续带电保持。运行中按 Ctrl-C 停止；已停止或完成后按 Ctrl-C 释放。`release` 结束发布，不执行收臂。停止、故障或反馈丢失后取消后续运动并保留终止状态；恢复路线从实际状态重新审核，不猜测撤退或自动重播。控制请求是否生效通过 `status` 确认。

每个 `job_id` 只允许尝试一次。布局仍有效且用户要求从头重画时，复用源输入，通过 `prepare` 创建新编号和新目录；保留执行记录和共享控制存储。中断作业不自动续画。

## 坐标与执行约定

- **坐标和笔尖。** `q14` 按 `[左7轴,右7轴]` 保存关节角，单位为弧度；夹爪和腰部单独保存。末端（EEF）位姿使用配置的 URDF 根坐标系：`x,y,z` 单位为米，`roll,pitch,yaw` 为 XYZ 欧拉角、单位为弧度。通过标定后的工具变换计算笔尖位置，并核对机器人、模型和图像的方向关系。
- **纸面和材料。** 支持平面纸张，根据实测点确定纸面的位置和倾斜程度，计算各处落笔高度。曲面绘画或使用其他清洗材料时，需另行适配。
- **笔画。** `start`/`end` 指定包含两端的接触区间。数据格式标识和字段说明见 [README](README.zh.md#数据格式与-api)，沿用其中的格式标识以兼容已有数据。
- **到位和运行检查。** 悬空目标记录位置、姿态和关节偏差；入口悬空终点与普通批次抬离连续衔接。最终休息位记录关节偏差并等待稳定保持。接触到位、持续跟随、反馈、最低净空、关节限位、速度、腰部/夹持目标和终止状态按配置检查；默认值见 [README](README.zh.md#到位和运行检查)。编译检查规划轨迹，运行检查实际反馈。
- **测量和指令。** 保存实测姿态与发布命令的对应关系，避免将同一伺服偏差反复累加到后续命令。接管时若漂移处于跟随容差内，运行器会读取最新反馈，检查当前位置到预编路线的衔接段。

## 输出要求

准备阶段链接源规划、测量、预览、编译作业、报告和启动脚本，说明笔画数、材料及修改参数，区分离线编译和实机执行。执行阶段说明作业身份、进度、终态、保持/释放状态和观察到的绘画结果。明确列出尚未完成的具体测量或执行步骤。
