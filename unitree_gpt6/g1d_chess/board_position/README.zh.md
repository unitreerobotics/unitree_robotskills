# 新棋盘90交点标记

[English](README.md) | 中文

本目录仅提供通用文档、空白格式和离线脚本，不附带真实棋盘照片、历史像素、机械臂XY、姿态锚点或成功样本。新模型先阅读 [标记任务说明](prompts/mark_board.zh.md)，再使用本次实际图片建立结果。

## 目录与依赖

- `scripts/mark_board.py`：新图绑定、全盘/分区拟合、留出验证、标注图/CSV/JSON输出和交点查询。
- `prompts/mark_board.md`：给接手模型的完整操作要求与交付清单。
- `requirements.txt`：独立使用的Python依赖；在走棋包内直接使用已安装依赖的 Python 环境。
- `runs/<本次标记名>/`：使用时创建，放本次原图、控制点、拟合结果和复核记录；当前不预置数据。

脚本不采图、不连接机器人、不生成动作；图片由当前工作流取得。需要 Python 3.10+、NumPy、OpenCV。独立使用时自行选用安装了依赖的Python，不需要复制机器人SDK。

## 操作步骤

以下命令从 `g1d_chess` 根目录运行。将占位路径替换为本次文件；`--max-error-px` 必须根据实际分辨率、读点误差和用途填写正数，不提供沿用旧盘的阈值。

```bash
python scripts/mark_board.py init \
  --image /path/to/current_left_high.png \
  --output runs/board_position/new_board/controls.json
```

初始化会记录原图绝对路径、SHA256和尺寸；`points`为空，`orientation_confirmed=false`。查看原图并确认机器人视角a–i左到右、0–9近到远；标准头图近端在下、远端在上。确认后把该字段改为true，依据实际可见板面格线添加记录：

```json
{
  "square": "a0",
  "pixel_xy": [null, null],
  "usage": "fit",
  "evidence": "visible_board_line_intersection",
  "note": "填写本帧可见格线依据；null必须替换为原图像素"
}
```

这只是空白格式，不代表a0一定可见。`usage`为`fit`或`holdout`；留出点不得参与拟合。同一交点只写一次，遮挡点不写入观测列表，由模型推算。每个拟合区域至少6个拟合点、2个留出点，分布于区域两侧、近远边界及内部；能读到更多真实交点时应增加覆盖。

先尝试`global`全盘单应模型；如果河两侧存在系统性偏差且重读控制点后仍存在，使用`split`，近区0–4、远区5–9分别满足控制点要求。分区模型只描述图像几何，不代表测得河宽或真实三维变换。

```bash
python scripts/mark_board.py build \
  --controls runs/board_position/new_board/controls.json \
  --mapping global --max-error-px <本次像素误差阈值> \
  --output-dir runs/board_position/new_board/result

python scripts/mark_board.py query d7 \
  --reference runs/board_position/new_board/result/board_points_pixels.json
```

输出目录必须是新目录，避免覆盖旧结果。失败时阅读具体原因；误差超阈值仍输出诊断与标注图，返回码2、状态`needs_reannotation`。通过数值检查仅表示`pending_visual_review`，还要查看标注图逐区验线，不能自动宣称已完成标定。纠正标记后使用新的输出目录重跑，不通过随意放宽阈值掩盖错误。

## 输出和复核

- `board_points_annotated.png`：保持原图分辨率。绿色为拟合观测点、蓝色为留出观测点、黄色为无直接观测的推算点；圆点为拟合位置，十字为人工读点。颜色不表示棋子占位。
- `board_points_pixels.json`：90个交点、图像哈希、原图尺寸、模型矩阵、拟合/留出残差及待视觉复核状态。`original_image_pixel_xy`均属原图板面坐标。
- `board_points.csv`：格名、格坐标、原图像素及证据类型。
- `controls.json`：此次拟合的控制点副本，记录可追溯输入。
- `visual_review.md`：由模型或人工另写，记录原图哈希、方向、四角/河界/内部检查、source/target附近格线、遮挡推算与剩余不确定处，给出通过或重做结论。脚本不会代替视觉验收。

棋盘、相机、头部/腰部姿态或裁剪改变后，应重新标记新图。即使尺寸相同，也不能直接套旧像素。板面交点、棋子顶面、腕图像素、机械臂末端XY是不同坐标；本工具不输出机器人基座变换、物理尺寸或运动补偿。物理尺寸未知保持null，不默认历史格距。

## 接回一轮走棋

规划阶段按本次标注核对source/target格名；source按棋子实际位置，target按目标板面交点，各自结合执行手、姿态与当前视觉独立估计。不得把像素差直接换成基座毫米，或把source修正套到target。头图标记不提供腕图配准。

资料在execute前准备；执行中仍遵循主 [round_plan](../prompts/round_plan.zh.md) 的检查点流程，不持棋重建全盘。棋盘点位目录不携带任何旧动作预检、修正距离或次数政策。
