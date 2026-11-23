# 棋盘读图与几何重测

[English](README.md) | 中文

独立子任务：帮助读棋局和核对棋盘位置的离线工具，不连接机器人、不发命令（`boardcal` 不带 `--observation` 时会经象棋服务拍一张头图）。实现在 `src/r1d_chess/board/`。

| 说明 | 入口 | 用途 |
|---|---|---|
| [board_view.zh.md](prompts/board_view.zh.md) | `bash r1d boardview`、`bash r1d fencheck` | 把头部鱼眼图修正为俯视图、逐格放大图和透视图，帮助读棋局、写 FEN，并把写好的 FEN 画回图上核对 |
| [board_points.zh.md](prompts/board_points.zh.md) | `bash r1d boardpoints`、`bash r1d boardcal` | 在左目原图里测出 90 个交叉点，判断棋盘是否移动，可选更新 `config/board_geometry.json` |
| — | `python scripts/head_board_measure.py` | 棋盘大幅移动后的粗测（含板面高度） |
| — | `bash r1d board` | 从实测交点拟合棋盘平面与 90 个板面点（不生成抓放 TCP） |

前提：已完成头部相机标定（`config/head_camera_calibration.json`，见 [setup/head-camera-calibration](../setup/head-camera-calibration/README.zh.md)）。新环境的第一次棋盘标定（含板面高度）在 [setup/chess-scene-parameters](../setup/chess-scene-parameters/README.zh.md) 第 3–4 步完成；这里的工具用于之后每轮读棋和棋盘移动后的重新标定。

这些图和点位只用于读棋局、确认棋盘几何，**不能用来估计 source/target 位姿**；位姿以 `bash r1d pose`、`round build` 和预检为准。
