# 相机检查：头部和腕部相机流

[English](README.md) | 中文

## 做什么

确认三路相机流都能取到图，并确认两路腕流分别对应哪只物理手，写进 `config/chess_camera_policy.json`。

腕部相机**不做内外参标定**：agent 手动流程里，腕图只由 agent 在门禁时看图判断（棋子是否在两爪之间、落点相对邻子和格线的位置），不参与几何计算。需要确认的只有“哪路流是哪只手”和画面是否清晰、不被遮挡。头部相机的几何标定见 [head-camera-calibration](../head-camera-calibration/README.zh.md)。

## 前提

`config/runtime.json` 的 `camera_host` 正确，机器人端的图像服务在运行。全程只读，不给机器人发运动命令。

## 步骤

1. 只读启动象棋服务（不加 `--allow-motion`），拍三路图：
   ```bash
   bash r1d serve --arm right
   bash r1d client capture_cameras --cameras cam_left_high cam_left_wrist cam_right_wrist \
     --output-dir <包根>/runs/camera_check/images --save-response runs/camera_check/capture.json
   ```
2. 看头图：应为锁定头姿下的单眼 640×480，同时保存了 1280×480 的双目原图（`*_stereo.jpg`）。记下规划用哪只眼（`config/robot.json` 的 `head_eye`）。
3. 确认腕流与物理手的对应：遮住或在一只手的腕部相机前放一个物体，再拍一次，看是 `cam_left_wrist` 还是 `cam_right_wrist` 的画面变了。两只手各做一次。
4. 把结果写进 `config/chess_camera_policy.json` 的 `wrist_stream_by_physical_arm`（物理手 → 流名），`note` 里写核对方法。
5. 看两路腕图的内容：两指和指尖前方的区域都在画面里，画面清晰。执行时门禁看的就是这个视角。
6. 结束只读服务（Ctrl-C）。

## 验收

- 三路流都能稳定取到图，头部流是左右拼接的 1280×480。
- 两只手的腕流对应关系各核对过一次，与配置一致。
- 核对结果与参考配置相同时，用 `python scripts/setup_status.py --confirm camera_mapping --note "<核对方法>"` 记录。

## 说明

- 流名和物理左右手不一定同侧，一律以核对结果为准。
- 原始 JPEG 没有源时间戳，各路图不保证严格同步。
- 门禁时在腕图里看到的爪尖位置、棋子大小因机器而异，没有可以沿用的像素基准。
