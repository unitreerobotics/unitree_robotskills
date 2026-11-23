# 头部相机标定（眼在头）

[English](README.md) | 中文

## 适用场景
- 新的 R1-D 首次上线。
- 头部相机拆装或碰撞之后。
- 头图测出的棋盘或棋子位置和手臂对不上。

## 标定什么
- **内参**：左右目鱼眼内参，以及双目外参（基线）。
- **相机安装外参**：左目相对 `head_yaw_Link` 的固定变换 `X`。
- **头关节修正项**：头 pitch/yaw 的比例误差（实际转角比编码器读数小）和 yaw 回差。

实时相机位姿按下式计算：

```
T_waist_cam = T(pitch 原点)·Ry(q_p')·T(yaw 原点)·Rz(q_y')·X
q' = pivot + (q - pivot)·(1 + scale)，yaw 再加上 backlash × 最后一次运动方向
```

以下不标：
- **头 pitch 零偏**：与板的位姿耦合，无法单独解出，只能用板面水平度给出上界。
- **腰编码器**：解算时不使用腰角。

## 标定板（固定规格，换板要同时修改所有脚本里的板定义）
ChArUco，5×7 格，方格 35 mm，标记 26 mm，`DICT_5X5_100`，OpenCV ≥4.6 新版图案。打印比例 100%，用尺子核对方格边长，贴在硬质平板上。

## 前提
- `config/runtime.json` 的 `camera_host` 和 `model_urdf` 正确，`config/robot.json` 有 `head_q_rad`。
- 阶段 2 需要 `r1d-runtime.service` 在运行，并且没有其他控制会话。`config/arm_torque_ff.json` 最好已经整定过，因为阶段 2 会带前馈保持双臂。

## 安全与授权
- 阶段 0、1、3 只读，不给机器人发命令。
- **阶段 2 会动头**，每次运行都要得到明确授权：速度 ≤0.2 rad/s，约 5 分钟，双臂保持原位，升降和底盘不动。
- 跟随误差超过 0.15 rad、过温、反馈过期会自动中止，结束前力矩斜坡降到 0。

## 步骤
命令都在包根下执行，`H=setup/head-camera-calibration/scripts`，`O=runs/head_calibration`。

**阶段 0：只读检查**
```bash
python -B $H/stage0_check.py --output $O/stage0
```
确认头部流是 1280×480（左右目拼接），记下每目有效列范围；板在视野内时，能解出多少角点（共 24 个）。

**阶段 1：内参和基线**（机器人不动，操作者手持标定板）
1. 录 150 s 原始 JPEG：
   ```bash
   python -B $H/head_calib_record.py --output $O/stage1_video --seconds 150
   ```
   - 板距相机 20–35 cm，慢慢移动，每秒不超过约 10 cm。
   - 按顺序走：中心 → 四角 → 左右边缘，每处上下左右各倾斜 30–45°。
   - 四角和边缘最重要（鱼眼畸变集中在那里）；手不要挡住格子。
   - 需要按格位引导采集时用 `head_calib_intrinsics_capture.py`。
2. 检测角点、挑帧并拟合内参（`fisheye` 模型）：
   ```bash
   python -B $H/head_calib_detect.py $O/stage1_video
   python -B $H/head_calib_intrinsics_solve.py $O/stage1_video
   ```
3. 拟合双目外参：
   ```bash
   python -B $H/stereo_fit.py $O/stage1_video
   ```
4. 把板平放、固定在下棋区域。**从这里到阶段 2 结束，板和桌子都不能动。** 放好后再跑一次阶段 0，确认两目都能解出角点。

**阶段 2：外参采集**（会动头）
```bash
python -B $H/head_calib_capture.py --output $O/stage2 --head-only --confirm-motion
python -B $H/capture_check.py $O/stage2
```
- `--head-only` 让头走 59 个姿态，腰和手臂不动。不要使用转腰模式。
- `capture_check.py` 检查：
  - 角点抖动应 ≤0.1 px；
  - `ref_start`、`ref_mid`、`ref_end` 三次回参考位的角点漂移应 ≤1 px，如果明显更大，说明板或底盘动了，这批数据作废重采。

**阶段 3：解算与验收**（离线）
```bash
python -B $H/head_calib_solve.py --stage2 $O/stage2 --intrinsics $O/stage1_video/intrinsics_fit.json \
   --stereo $O/stage1_video/stereo_fit.json --output $O/stage3_final.json --config config/head_camera_calibration.json
```
- 初值由闭式手眼标定算出；比例模型的支点取本次采集的中位头姿。两者都从数据现算，不依赖任何已有标定。
- 全部验收项通过才会写 `--config`。
- 有项目没通过而操作者决定接受时，加 `--accept-note "<理由>"`，未通过的项会和理由一起记进配置。

**可选：目视检查相机和手臂是否一致**
```bash
python -B $H/arm_overlay.py $O/stage2 --output $O/arm_overlay.jpg
```
手臂 FK 投影点应落在图中真实手臂上。这是目视检查，不是定量验收。

## 验收标准（`head_calib_solve.py` 自动判断）
| 项 | 标准 |
|---|---|
| 内参留出帧 RMS | ≤0.5 px |
| 留出姿态重投影 RMS / P90 | ≤1 / ≤1.5 px |
| 交叉拟合相机位置 / 姿态变化 | ≤2 mm / ≤2 mrad |
| 交叉拟合板中心变化 | ≤3 mm |
| 板面水平 | ≤1°（前提是桌面水平） |

建议另做独立检查：用尺子量两镜头中心距，与双目基线比较（≤2 mm）；量板面到桌面的高度，与解出的板高比较。

## 输出
- `config/head_camera_calibration.json`：K、D、`T_right_from_left`、`X_head_yaw_to_left_camera`、比例支点、`scale`、`backlash`、各验收项、URDF 哈希、接受记录。
- `$O/` 下保存全部原始数据，可以复算。

## 常见问题
- **只看拟合残差会被误导**：内参和外参一起拟合、视角又窄时，残差可以很小，但桌面误差能达到几厘米。验收必须看留出姿态和交叉拟合。
- **左右目分开解会得出看似矛盾的基线**：要用两目共用一个相机架的联合解算，即 `head_calib_solve.py` 的 Rig 模型。
- **头 yaw 有回差**：使用时固定到位方向，例如每次都从负方向回到参考位，否则桌面上会有几毫米误差。
- **头关节有比例误差**：用 PnP 测得的板转角除以编码器转角，可以独立核对。
- **离参考姿态越远，外推越不可靠**：锁定的头姿要落在阶段 2 采集过的 pitch 范围内，或者只略微超出。

> 参考（参考机器人）：头 yaw 回差约 ±15 mrad，头关节比例误差约 1–2%；腰的编码器读数与输出端转角不一致，转腰模式下腰实际几乎没有转动，所以解算不用腰角。交叉拟合姿态一项为 2.76 mrad，略超 2 mrad 的标准，经操作者接受后使用。
