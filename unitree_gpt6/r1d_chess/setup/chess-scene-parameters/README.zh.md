# 下棋场景参数

[English](README.md) | 中文

## 适用场景
- 新的 R1-D 首次上线。
- 棋盘、桌子、升降高度或头部参考姿态有变动之后。
- `round build` 或预检因为几何问题被拒绝。

## 写入哪些参数
所有参数都在 `waist_yaw_link` 坐标系下，单位为米/弧度，都标记为 `model_estimate`，模型估计可以直接用于执行。各参数的含义见 [docs/r1d_scene_parameters.zh.md](../../docs/r1d_scene_parameters.zh.md)。

| 参数 | 来源 |
|---|---|
| `init_joint_rad`、`head_q_rad`、`waist_q_rad`、`lift_height_m` | 当前机器人的只读快照 |
| `tcp_xyz_rpy` | URDF 中 `dex1_base_joint` 的原点，即夹爪法兰 |
| `init_xyz_rpy` | 用 FK 由 `init_joint_rad` 计算 |
| `scene`（工具包络 + 两级支撑面） | 夹爪图纸，加上头部相机测出的板面；棋盘范围外用桌面 = 板面 − 板厚（`--board-thickness`，范围边框用 `--board-border`） |
| `board_release_z_m`、`tray_release_z_m`、`release_pitch_rad` | 板面 / 桌面 + 指尖净空 + 指长·sin(pitch) |
| `kinematic_model_sha256` | 当前 URDF 的哈希 |
| `config/board_geometry.json` | 头部相机测出的棋盘中心、转角、90 个交叉点、托盘位置 |

## 前提
- 第 1 步只需要机器人运行时（`r1d-runtime.service`）在运行。
- 第 2 步起需要已完成 [head-camera-calibration](../head-camera-calibration/README.zh.md)，`config/head_camera_calibration.json` 的 URDF 哈希与当前模型一致。
- `config/runtime.json` 的 `model_urdf` 指向包含 Dex1 的 R1-D URDF（包内 `assets/models/r1d/r1d.urdf`）。
- 棋盘参数已知（格距、板厚、棋子尺寸，用尺量），夹爪图纸已知（法兰到指尖长度、全开宽度）。

## 安全
- 除第 3 步外都是只读或离线操作，但需要 `r1d-runtime.service` 在运行，用来读取关节和升降状态。
- 第 3 步用 `--single-pose` 会转动头部（≤0.2 rad/s，腰和手臂保持），需要授权。

## 步骤
命令都在包根下执行，`C=setup/chess-scene-parameters/scripts`，`H=setup/head-camera-calibration/scripts`，`O=runs/scene`。

1. **记录初始姿态。** 把机器人摆成每轮开局时的姿态（双臂停在棋盘两侧、夹爪闭合、指尖离桌面留出净空），保持静止：
   ```bash
   python -B $C/capture_initial_pose.py --write                         # 头参考取当前实测
   python -B $C/capture_initial_pose.py --write --head-q <pitch> <yaw>   # 或者锁定接近最低头的姿态（推荐，棋盘在画面中心）
   ```
   锁定的头姿要在头部相机标定采过的 pitch 范围内，或只略微超出。头 yaw 的到位方向要固定（例如负向）。
2. **核对夹爪尺寸。** 用 URDF 网格和图纸互相核对：
   ```bash
   python -B $C/dex1_envelope.py
   ```
   法兰到指尖的长度和全开宽度应与图纸一致（Dex1 图纸为 143.2 mm、117 mm）。不一致时，以实物或图纸为准，通过 `write_scene.py` 的参数传入。
3. **测量棋盘。** 需要一张头在锁定姿态下的立体原图，以及当时的头角。还没有完整几何时象棋服务接管不了，所以用标定采集工具把头转过去拍图。这一步**会动头**（只动头，腰和手臂保持），需要授权：
   ```bash
   python -B $H/head_calib_capture.py --output $O/locked_view --single-pose <pitch> <yaw> --confirm-motion
   ```
   它会转到该头姿、拍 6 帧，再回到 `robot.json` 的参考位。图在 `$O/locked_view/001_*/0.jpg`；头角取 `manifest.json` 中 `poses[1].head_q`；yaw 方向是这一次 yaw 运动的符号。
   如果象棋服务已经接管、头已经停在参考位，也可以改用 `client capture_cameras`（保存 `*_stereo.jpg` 原图），头角取返回状态里的 `head_q`，不需要转头。
   ```bash
   python scripts/head_board_measure.py --image <立体原图> --head-q <pitch> <yaw> --yaw-direction <-1|1> --output $O/board_measure.json
   ```
   - 板面高度用双目平面扫描求得：搜索使左右目在 z=h 平面上的俯视图一致的 h。
   - 网格用已知格距拟合。
   - **一定要看输出的 `*_grid.jpg`**：网格是周期性的，没有先验时可能整体错开一格。已有本机的 `config/board_geometry.json` 时，加 `--prior config/board_geometry.json`，只在已知位置附近搜索；参考机器人的棋盘几何不能当先验。
   - 托盘中心可以从俯视图上读出（误差约 ±10 mm）。
4. **生成参数。** 先空跑，确认后再写入：
   ```bash
   python -B $C/write_scene.py --measure $O/board_measure.json --pitch 0.35 --trays LX LY RX RY          # 空跑，打印数值和变化
   python -B $C/write_scene.py --measure $O/board_measure.json --pitch 0.35 --trays LX LY RX RY --write
   ```
   - 默认值都是相对量：板面释放时指尖高出板面 13 mm；圆盘释放时指尖高出桌面 68 mm（必须高过托盘杯沿，首次吃子前要看图确认）。
   - 板厚、格距、棋子尺寸、指长、半宽、`uncertainty` 都可以通过参数修改。
5. **检查可达性**（只是 IK 抽查）：
   ```bash
   python -B $C/reach_check.py --pitch 0.35 0.8 --yaw -0.5 0 0.5 1.0
   ```
   - 确认左右手各朝向合起来能覆盖整盘。走子实际用 pitch 0.8，profile 的释放高度在 `--pitch` 给的值（0.35）下标定，换算由 `round build` 完成。
   - 单点 IK 抽查偏悲观；按真实下降方式（高位逐步下降、链式种子）可达的范围更大，最终以完整预检为准。
   - 改了第 4 步的 `--pitch` 后要重新运行第 4 步：释放高度和 pitch 是绑在一起的。
6. **评估视角（可选）：**
   ```bash
   python -B $C/view_resolution.py --image <立体原图> --head-q <pitch> <yaw> --overlay $O/view.jpg
   ```
   比较不同头姿下格子和棋子的像素尺寸，以及 90 个交叉点的投影是否落在真实交叉点上。
7. **验证门禁：**
   ```bash
   python -B -c "import sys;sys.path.insert(0,'src');from r1d_chess.robot.r1d_config import load_profile,require_calibration;require_calibration(load_profile(),hardware=True);print('ok')"
   ```
   然后挑一步真实走法，只做规划和完整预检，不执行。

之后只是棋盘被移动时，用 `bash r1d boardcal` 重测棋盘几何即可（见 [board_position](../../board_position/README.zh.md)）；板面高度或桌子变了要重做第 3、4 步。

## 验收标准
| 项 | 标准 |
|---|---|
| 两种板面估计 | “双目平面扫描的板面”与“标定板解出的板面 − 实量板厚”相差 ≤5 mm |
| 换头姿复测 | 在第二个头姿下复测（加 `--prior`），棋盘中心和板面变化 ≤3 mm |
| 网格叠加图 | 每个棋子都落在交叉点上 |
| 门禁 | `require_calibration(hardware=True)` 通过，真实走法的完整预检通过 |

## 输出
- `config/robot.json`、`config/chess_initial_pose.json`
- `config/board_geometry.json`
- `$O/board_measure.json`、`*_grid.jpg`

## 常见问题
- **`half_thickness` 的前提**：它假设抓放时 pitch ≥0.2 rad，这时最低点是指尖、手指根部更高。
- **抓取低位低于工具净空**：抓取时指尖要低于棋子顶面，会低于工具包络的净空要求。每轮要开 `ignore_source_tool_envelope`，由视觉门禁把关。
- **场景参数对环境变化敏感**：升降高度、棋盘位置、头部参考任何一项变了都要重做第 3、4 步。接管时升降高度偏离超过 ±5 mm，服务会拒绝。
- **单轮 profile 只能从 `config/robot.json` 派生**，并保留 `kinematic_model_sha256`。缺少这个字段或与当前 URDF 不一致时，服务和预检都会拒绝。
- **改动 `robot.json` 会改变 profile 摘要**，之前累计的落点误差样本（`xy_error_context`）不再适用。
