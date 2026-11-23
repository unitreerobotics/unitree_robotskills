# Board image reading and geometry re-measurement

English | [中文](README.zh.md)

An independent subtask: offline tools that help read the position and check the board location; they do not connect to the robot and send no commands (`boardcal` without `--observation` takes one head image through the chess service). The implementation is in `src/r1d_chess/board/`.

| Description | Entry | Purpose |
|---|---|---|
| [board_view.md](prompts/board_view.md) | `bash r1d boardview`, `bash r1d fencheck` | Rectify the head fisheye image into a top-down view, a per-square enlarged image, and perspective views, to help read the position and write the FEN, and draw the written FEN back onto the image for checking |
| [board_points.md](prompts/board_points.md) | `bash r1d boardpoints`, `bash r1d boardcal` | Measure the 90 intersections in the raw left-eye image, determine whether the board has moved, and optionally update `config/board_geometry.json` |
| — | `python scripts/head_board_measure.py` | Coarse measurement after a large board move (includes the board surface height) |
| — | `bash r1d board` | Fit the board plane and the 90 board surface points from measured intersections (does not generate pick and place TCPs) |

Prerequisite: the head camera calibration is complete (`config/head_camera_calibration.json`, see [setup/head-camera-calibration](../setup/head-camera-calibration/README.md)). The first board calibration in a new environment (including the board surface height) is done in steps 3–4 of [setup/chess-scene-parameters](../setup/chess-scene-parameters/README.md); the tools here are for reading the board every round afterwards and for recalibration after the board moves.

These images and points are only for reading the position and confirming the board geometry, and **cannot be used to estimate source/target poses**; poses are determined by `bash r1d pose`, `round build`, and preflight.
