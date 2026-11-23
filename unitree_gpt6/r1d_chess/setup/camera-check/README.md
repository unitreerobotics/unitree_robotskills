# Camera check: head and wrist camera streams

English | [中文](README.zh.md)

## What it does

Confirm that images can be fetched from all three camera streams, confirm which physical hand each of the two wrist streams corresponds to, and write this into `config/chess_camera_policy.json`.

The wrist cameras **are not calibrated for intrinsics or extrinsics**: in the agent manual flow, the wrist images are only judged by the agent by looking at them at a gate (whether the piece is between the two jaws, and where the landing point is relative to neighbouring pieces and the grid lines), and do not take part in geometric computation. The only things to confirm are "which stream is which hand" and whether the image is sharp and unobstructed. For the geometric calibration of the head camera, see [head-camera-calibration](../head-camera-calibration/README.md).

## Prerequisites

`camera_host` in `config/runtime.json` is correct, and the image service on the robot side is running. The whole procedure is read-only and sends no motion commands to the robot.

## Steps

1. Start the chess service read-only (without `--allow-motion`) and capture the three images:
   ```bash
   bash r1d serve --arm right
   bash r1d client capture_cameras --cameras cam_left_high cam_left_wrist cam_right_wrist \
     --output-dir <package root>/runs/camera_check/images --save-response runs/camera_check/capture.json
   ```
2. Look at the head image: it should be a single-eye 640×480 image at the locked head pose, with the 1280×480 raw stereo image (`*_stereo.jpg`) saved as well. Note which eye is used for planning (`head_eye` in `config/robot.json`).
3. Confirm the correspondence between the wrist streams and the physical hands: cover the wrist camera of one hand or place an object in front of it, capture again, and see whether the image of `cam_left_wrist` or of `cam_right_wrist` changed. Do this once for each hand.
4. Write the result into `wrist_stream_by_physical_arm` in `config/chess_camera_policy.json` (physical hand → stream name), and write the checking method in `note`.
5. Look at the content of the two wrist images: both fingers and the area in front of the fingertips are in the image, and the image is sharp. This is exactly the view the gate looks at during execution.
6. Stop the read-only service (Ctrl-C).

## Acceptance

- Images can be fetched stably from all three streams, and the head stream is a left-right concatenated 1280×480.
- The wrist stream correspondence of each of the two hands has been checked once and matches the configuration.
- When the check result is the same as the reference configuration, record it with `python scripts/setup_status.py --confirm camera_mapping --note "<checking method>"`.

## Notes

- The stream name and the physical left or right hand are not necessarily on the same side; always go by the check result.
- The raw JPEGs have no source timestamp, and the images of the streams are not guaranteed to be strictly synchronised.
- The jaw tip position and the piece size seen in the wrist image at a gate differ between robots; there is no pixel baseline that can be reused.
