# R1-D 头部图像读棋（boardview / fencheck）

[English](board_view.md) | 中文

## 适用场景
- 每轮写 `board.json` 的 FEN 之前，或者核对对手应手时。
- 远端黑方半场在原图里挤在一起、看不清时，尤其需要。
- 怀疑 FEN 里某个子写错或漏掉时。

## 不能用于
**这些图只用于读棋局。** 它们是重采样后的图片，**不能用来估计 source/target 的 XYZ/RPY**，也不能代替规划图作为位姿依据。位姿仍以原始规划图、`round build` 和预检为准。

## 前提
- `config/head_camera_calibration.json` 存在（眼在头标定结果，见 `setup/head-camera-calibration`）。
- `config/board_geometry.json` 与棋盘当前位置一致。棋盘被移动过时，要先用 `scripts/head_board_measure.py --prior config/board_geometry.json` 重测。
- 规划图是在头部锁定姿态下拍的，并且知道当时的头角。用 `client capture_cameras` 拍的规划图会自动带上这两样。

## 步骤
在包根目录下执行。

1. **生成参考图**（用本轮的规划图响应）：
   ```bash
   bash r1d boardview --observation <run>/tmp/<capture 响应>.json --output <run>/images/board_view
   ```
   - 也可以直接给原图：`--image <*_stereo.jpg>`（可以重复给多帧，会平均降噪）加 `--head-q <pitch> <yaw>`。
   - 头 yaw 回差约 ±15 mrad。到位方向不是负向时，加 `--yaw-direction 1`。
2. **先检查对齐。** 打开 `board_topdown.jpg`，绿色十字应该落在棋盘线的交叉点上。
   - 如果整体偏了一格，或者明显错位，说明棋盘被移动过：停止读棋，先重测棋盘几何。
3. **读棋局。** 按 FEN 的顺序读：从第 9 行（远端、黑方，图上方）读到第 0 行（近端、红方，图下方），每行从 a 读到 i（a 在机器人左手边，图左侧）。
   - 字形对照底部图例：红字是大写，黑字是小写。车 r/R、马 n/N、象/相 b/B、士/仕 a/A、将/帥 k/K、炮 c/C、卒/兵 p/P。
   - 黑红两方的“炮”是同一个字，只能靠墨色区分。“象/相”“士/仕”“将/帥”“卒/兵”字形不同。
   - 拿不准时看 `board_tiles.jpg`：**字的中心落在该格的绿色圆里，才算这一格有子**。远端棋子会露出侧面，相邻格可能看到别的棋子边缘。
   - 远端字比较糊时，对照 `undistorted_left.jpg` 和原始规划图一起判断。
4. **核对 FEN：**
   ```bash
   bash r1d fencheck "<FEN> w - - 0 1" --view <run>/images/board_view
   ```
   - 它会输出文本棋盘和双方子数，并检查格式、行列数、各子数量上限、将帥是否在九宫。
   - 打开生成的 `fen_overlay.jpg`：每个交叉点上画着你 FEN 里的字母，红框是红方、黑框是黑方，绿点表示空格。**逐格比较字母和下面的棋子**，任何一处不符都要改 FEN，再重新核对。
   - `ok: true` 只说明 FEN 格式合法，不说明它和棋盘一致；一致性只能靠叠加图逐格看。
5. **写入 `board.json`**：`evidence` 里放原始规划图路径，可以附上 `fen_overlay.jpg` 作为读棋依据。然后照常用 `bash r1d engine validate/analyze` 检查着法合法性。

## 输出（都在 `--output` 目录下）
| 文件 | 内容 |
|---|---|
| `board_topdown.jpg` | 投影在棋子顶面平面的俯视图，红方在下，四周标行列坐标，附棋子图例 |
| `board_tiles.jpg` | 90 个交叉点逐格放大，每格画棋子大小的圆 |
| `undistorted_left.jpg`、`undistorted_right.jpg` | 去掉鱼眼弯曲后的普通透视图 |
| `board_view.json` | 输入参数，以及每个交叉点在俯视图中的像素位置 |
| `fen_overlay.jpg` | 由 `fencheck --view` 生成 |

## 局限
- **远端清晰度改善有限。** 远端分辨率受相机限制，一格约 11–17 px。修正只是把几何和排版变清楚，不会增加细节。
- **棋盘线有轻微重影。** 俯视图对齐在棋子顶面，棋盘线比这个平面低一个棋子高度，两目视差会让线出现重影，不影响读字。
- **棋盘必须和几何配置一致。** 任何移动都会让网格对不上，所以第 2 步的对齐检查不能省。
