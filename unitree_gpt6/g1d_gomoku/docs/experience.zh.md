# 经验数据约定

[English](experience.md) | 中文

经验使用 JSON 保存，不在 Markdown 中预填尺寸、位姿、臂形关节、夹爪开度或旧轨迹。
通用判断方法见 [操作方法](manipulation.zh.md)，数据结构见 [schema.json](../experience/schema.json)。

| 位置 | 作用 |
| --- | --- |
| 工程 `experience/template.json` | 空经验库，随包发布，运行时只读 |
| 工程 `experience/schema.json` | 记录及候选数据结构，供工具和编辑器使用 |
| 工作空间 `experience/context.json` | 当前机器人、场景及配置、归位参考、标定、模型摘要 |
| 工作空间 `experience/binding.json` | 当前配置与标定的工作空间相对路径 |
| 工作空间 `experience/records.json` | 成功后追加的实测经验，不覆盖旧记录 |
| 本轮 `experience_candidate.json` | Agent 在归位成功后整理的待校验数据 |

## 首次执行

初始化只创建空库，不注入旧经验，不覆盖已有数据：

```bash
python -m unitree_gpt6.g1d_gomoku.src.experience init \
  --workspace "$GOMOKU_WORKSPACE"
```

先采集当前设备的归位参考，并基于四路图像、当前反馈和模型建立场景标定。
Agent 将标定结果和依据写入工作空间 `calibration/scene.json`；缺少尺度或坐标关系时继续观测与标定，不能凭空生成。
归位参考是执行前的标定数据，不属于成功经验，不必等下完一手才采集。

Agent 为当前设备和布置选择稳定的 `robot-id`、`scene-id`，建立绑定：

```bash
python -m unitree_gpt6.g1d_gomoku.src.experience context \
  --workspace "$GOMOKU_WORKSPACE" \
  --robot-id ROBOT_ID --scene-id SCENE_ID \
  --config "$GOMOKU_HARDWARE_CONFIG" \
  --calibration "$GOMOKU_WORKSPACE/calibration/scene.json"

python -m unitree_gpt6.g1d_gomoku.src.experience show \
  --workspace "$GOMOKU_WORKSPACE"
```

机器人或棋盘、供子区、相机布置改变时更新对应 ID 与标定文件，并重建绑定。
`show` 只返回当前绑定下、证据摘要仍一致的记录；首次返回空列表。
配置、归位参考、标定或模型变化而未重建绑定时拒绝读取，不自动降级使用旧值。
不导入旧版默认归位文件或经验。首次执行独立求解，不能使用零值或示例值充当测量。

## 成功后写入

完成归位后，Agent 从本轮图像、反馈、计划和结果整理候选文件。字段如下：

| 字段 | 要求 |
| --- | --- |
| `context` | 完整复制当前 `experience/context.json` |
| `observations` | 非空观测列表；只填写实际核验的项 |
| `observations[].kind` | `geometry`、`grasp`、`alignment`、`release`、`return` 或 `placement` |
| `target` | 目标值对象；没有对应目标时为 `{}` |
| `measured` | 非空实测值对象，不能用命令目标冒充反馈，未知字段直接省略 |
| `verification` | 对应检查为 true，并提供非空 `method` 说明核验方法 |
| `evidence` | 本轮目录下的相对文件路径列表，指向实际读取的图像、反馈和结果 |

`measured` 和 `target` 的字段名须注明单位，例如 `tcp_pose_m_rad`、`joint_positions_rad`、
`gripper_rad`、`grid_spacing_m`、`duration_s`；记录姿态时一并写明 `frame` 和关节顺序。
记录抓取臂形时保存肩肘腕实测关节和夹爪朝向，不只保存 TCP。
不保存固定棋谱、落点、用户授权或可盲目重放的动作脚本。

| kind | verification 中的检查项 | 证据要求 |
| --- | --- | --- |
| `geometry` | `calibration_verified` | 当前标定、原图与尺度/坐标核验 |
| `grasp` | `single_stone_held` | 短抬升图像、原位空缺及当前关节/夹爪反馈 |
| `alignment` | `selected_intersection_verified` | 释放前目标交点、邻棋网格及低位反馈 |
| `release` | `release_completed` | 夹爪反馈和本轮释放结果，不表示落点精度 |
| `return` | `reset_joint_verified` | 本轮归位反馈与结果 |
| `placement` | `physical_placement_verified` | 独立的放子后视觉核验及结果 |

当前流程放子后不分析图像，`physical_placement_verified` 为 false，所以不能发布 `placement` 成功经验。
释放前对准成功可以单独记为 `alignment`，不能改称实际落盘精度。

```bash
python -m unitree_gpt6.g1d_gomoku.src.experience record \
  --workspace "$GOMOKU_WORKSPACE" \
  --round "$GOMOKU_ROUND" \
  --input "$GOMOKU_ROUND/experience_candidate.json"
```

工具要求本轮为 `returned` 且归位核验通过，检查部署绑定、有限数值、各类核验声明和证据文件，
计算证据摘要后加锁、原子追加；同一轮只能提交一次，所有观测一起提交。
候选数据及证据必须位于工作空间，证据进一步限定在本轮目录，不能引用外部私有目录。
失败、未核验、只有规划的数据只留在运行日志，不进入成功经验库。

工具负责结构和可追溯性校验，不代替 Agent 看图，也不独立证明视觉或物理成功。
全部学习写入发生在归位后，不延长低位等待。后续经验只辅助候选求解，仍需当前反馈、图像、限位和净空检查。
