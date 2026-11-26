---
name: r1d-sandwich
description: >-
  让 Agent 操作带 Dex1 夹爪、固定升降的 Unitree R1-A7-D：从头部和两路腕部相机图像读物体位置，直接给出
  末端位姿路点（位置和 rpy），由本机服务求解并执行，不用检测器、不做规划搜索，完成一份三明治：右手把
  烤面包机里的面包片放到白盘，左手加生菜，右手再盖第二片；用户要求时，最后用双手把盘子向前推出。
  适用于用户要求本目录制作三明治、离线预演路点、读取相机或采集运行指标的场景。
license: Apache-2.0
compatibility: >-
  需要 Linux x86_64 上的 Python 3.10 和 requirements.txt 里的包。仿真和离线预演不需要硬件；连真机需要一台在
  R1-D 网络（192.168.123.x）内的主机，并有操作员在场。
metadata:
  robot: R1-A7-D
  source: r1d-sandwich 直出末端三明治运行 2026-11-17..25
  version: "0.1.0"
  language: zh
---

# 用直出末端路点制作 R1-D 三明治

[English](SKILL.md) | 中文

## 目录

本文件所在目录就是技能根目录。所有命令都在这里执行：`bash r1d <子命令>`。

- `scripts/`：`bash r1d` 背后的命令（`eef`、`eval`、`posture`、`metrics`、`check-env`、`serve`）。
- `references/`：任务需要时再读的文档（见下面的列表）。
- `assets/`：相机标定、场景配置、R1-D 模型和路点模板。
- `src/`：服务的代码，执行任务时不需要阅读。
- `runs/`：生成的图像、录像、日志和指标。

服务运行在 `127.0.0.1:8765`，固定升降 `0.522 m`。

## 分工

Agent 负责看图、估计位置、选择路线、给出路点，并判断每次观察后的结果。本机服务负责 IK、关节与自碰撞
检查、桌面和障碍盒检查、运动执行、看门狗和停止状态。Agent 不得绕过服务拒绝。

## 按任务选读

- **新主机或新机器人**：读 [README.zh.md](README.zh.md)，把 `.env.example` 复制为 `.env` 并修改外部路径，
  再运行 `bash r1d check-env`。
- **制作三明治**：路线读 [references/TASK_SANDWICH.zh.md](references/TASK_SANDWICH.zh.md)，命令、载荷和
  效率规则读 [references/PLAYBOOK.zh.md](references/PLAYBOOK.zh.md)。
- **把做好的盘子向前推出**：读 [references/TASK_SANDWICH.zh.md](references/TASK_SANDWICH.zh.md) 的
  “推盘说明”一节。只在用户要求时执行。
- **服务字段或接口**：读 [references/API.zh.md](references/API.zh.md)。
- **某条限制的由来、以前的路线和测量**：读 [references/EXPERIENCE.zh.md](references/EXPERIENCE.zh.md) 和
  [references/RESULTS.zh.md](references/RESULTS.zh.md)。

## 单次运行

1. 先取得用户对本次运动的明确授权，再确认场地清空、操作员能够触达硬件急停。
2. 只启动用户授权的模式：`bash r1d serve --sim --fixed-lift 0.522`、
   `bash r1d serve --shadow --fixed-lift 0.522`，或 `bash r1d serve --real --fixed-lift 0.522`。
3. 运行 `bash r1d eef new`，然后运行 `bash r1d eef prepare`。它把机器人调到起始姿态：升降 `0.522`，
   头部低头 `0.486` rad，让头部相机看到整张桌面。每次运行开始都要做这一步，在第一次手臂运动之前、
   也在从头部相机读取任何位置之前。
4. 运行 `rec-start`、`state`，并先看一次图。从当前图像读取位置；每次摆放的坐标都会变。
5. 每个阶段先用 `bash r1d eval --brief '<json>'` 离线预演，放置和退出一起预演。
6. “路线速览”的每一步都是一条先动作再看图的 shell 命令
   （`bash r1d eef exec '<json>' && bash r1d eef look ...`），并在同一轮里读取 `runs/latest/<相机>.jpg`。
   只有图片和上报的开度都表明这一步成功了才继续。
7. 结束时依次执行 `home`、`keyframe final`、`rec-stop` 和 `bash r1d metrics --run-dir runs/<运行目录>`。

路点 JSON 的格式见 [references/PLAYBOOK.zh.md](references/PLAYBOOK.zh.md)；
`assets/waypoint_template.json` 只是格式正确的起点，不是新场景中的安全位姿。

## 硬规则

这些规则只在这里写一次，其他文档都引用这里。

1. **授权。** 不能因为文档写了命令就运行 `exec`、`both`、`grip`、`waist`、`home` 或 real/shadow 服务。
   每一次运动都必须有用户对本次运行的明确授权。
2. **先到起始姿态，之后固定升降。** 每次运行都从 `bash r1d eef prepare` 开始。第一次手臂运动之前，在它的
   输出里确认 `lift_m` 是 `0.522`、`head_pitch` 约为 `0.5`。此后不能再改变升降和头部。服务以
   `--fixed-lift 0.522` 启动。
3. **阶段之间用关节空间 `home`。** 不能用末端位姿当“准备位置”。
4. **被拒绝就停下。** 有依赖关系的 shell 命令用 `&&` 连接；有管道时在开头加 `set -o pipefail`：
   `exec ... | tail` 返回的是 `tail` 的退出码，会掩盖拒绝。
5. **只搬运夹住的东西。** 夹取后读取并报告开度。面包的开度至少 `0.1` 才能搬运；在拉起后的第一个路点
   上加 `"min_opening": 0.1`。
6. **直线被拒绝。** `linear: true` 在可达边界附近被拒绝时，先用 `linear: false` 退回上一个悬停点，再沿
   已经走通的路线靠近。不要重试同一段。
7. **烤面包机。** 烤面包机挪动后，用 `look --mark` 重新画出并核对障碍盒。TCP `0.790` 是夹取下限。
8. **盘子上方一次只动一只手。** 一只手拿着物体停在盘子上方时，不能移动另一只手。
9. **盘子距离。** 开始前盘子中心必须在 `x >= 0.33`。更近时请操作员把盘子挪远。
10. **推盘。** 把手降到盘子后方之前，图像里必须能看到手指和盘沿之间的桌面。手指挡住盘沿，说明手指在
    盘沿上方。
11. **服务停止。** 服务进入 `stopped` 时不要自行清除，把原因报告给操作员。

## 汇报

只报告实际执行的动作。给出最终图像结论、夹爪开度、被拒绝的指令、实测放置偏差、温度和指标路径。缺图、
缺温度采样或缺指标时直接说明缺失，不能推断成功。
