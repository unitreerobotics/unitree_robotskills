# Measuring the 90 board points (boardpoints / boardcal)

English | [中文](board_points.zh.md)

## What it does
It measures the 10 rank lines and 9 file lines printed on the board directly in the raw left-eye image and derives the 90 intersections:
- Each line is sampled in the gaps between two adjacent intersections (all pieces sit on intersections, and a gap of about 6 mm is left between pieces).
- The sub-pixel positions of the dark lines are found on the board top surface plane, lines in the same direction share one slope, and the intercept of each line is fitted robustly.
- Intersection = the crossing of a rank line and a file line.

`config/board_geometry.json` is used only to determine the initial position and the search range; the result comes entirely from the image and is compared with the stored values.

## When to use
- When the board may have been moved, or the `board-view` grid looks misaligned: to confirm whether the board geometry needs to be re-measured.
- When you need the pixel position of a square in the raw left-eye image: for example to annotate the planning image, or to let the model locate a piece in the raw image.
- When you want the measured coordinates of each point rather than coordinates "computed from the nominal square spacing".

## Prerequisites
- The board geometry is the camera fitting result, and the board-view grid should align with the printed lines. Do not shift the geometry to compensate for landing deviation; landing deviation is corrected with XY `adjust` at the pre-release gate.

- `config/head_camera_calibration.json` already exists, along with a roughly correct `config/board_geometry.json`. The deviation must be within ±9 mm; if the board has been moved a long way, first do one coarse measurement with `scripts/head_board_measure.py`.
- **When the board moves by more than about half a square, both measurements lock onto a periodic alignment that is off by one square, and they "agree" with each other** (reference: with an actual move of about 24 mm, both reported only 3–4 mm). How to tell: `measured_points` is very small (<30) and only a few file lines are measured; before writing, you must use `boardview` to see whether the palace centre and the pieces in file e fall on the marks. When it is off by one, shift a copy of the prior by the offset seen in the top-down view (shift the centre and the 90 points as a whole) and measure again with `--geometry shifted-prior.json`: with the correct alignment the number of measured points increases clearly (reference: 69 points, all 19 lines).
- The head angles at the time the image was taken are known. They are read automatically when the response of `client capture_cameras` is used.

## Steps
Run in the package root directory; it runs offline and does not connect to the robot.

1. Measure:
   ```bash
   bash r1d boardpoints --observation <capture response json> --output <run>/images/board_points
   # or: --image <raw image *_stereo.jpg or 640x480 left-eye image> (can be repeated for multiple frames, which are averaged) --head-q <pitch> <yaw>
   ```
2. Open `left_points_x2.jpg` and check:
   - The blue lines should lie on the printed rank lines.
   - **Green points**: both nearby lines were measured; **yellow points**: the area nearby is blocked by pieces, and the point is inferred from the fitted lines.
   - When the lines clearly deviate from the printed lines, the result cannot be trusted: check the head angles, the yaw approach direction (`--yaw-direction`), and the calibration.
3. Look at `summary` in the output:
   - `deviation_from_geometry_mm`: deviation from the stored geometry. With a mean ≤3 mm and a maximum ≤6 mm, the board can be considered not to have moved. Clearly larger values mean the board has been moved.
   - `cell_m`: measured square spacing.
   - When the prior is not in `config/`, use `--geometry <json>` (read-only, as the initial position and search range).
4. (Optional) Write the measured points into the board geometry:
   ```bash
   bash r1d boardpoints ... --write-geometry
   ```
   - It updates the centre, rotation angle, square spacing, and 90 points in `config/board_geometry.json`; the old values are saved in `previous_fit`.
   - The board surface height does not change; the height must be measured with `scripts/head_board_measure.py`.
   - After writing, regenerate the top-down view with `bash r1d boardview` and confirm that the grid is aligned.

## Outputs
| File | Content |
|---|---|
| `board_points.json` | For each point: the pixel in the left-eye 640×480 image, xyz in the waist frame (on the board surface plane), deviation from the stored geometry, and whether it was measured; the fitted centre, rotation angle, and square spacing; the samples and inliers of each line |
| `left_points.jpg`, `left_points_x2.jpg` | Left-eye image annotated with the 90 points, the fitted rank lines, and the rank and file coordinates |

## Accuracy and limitations
- **The result is a model estimate**, and its accuracy depends on the head camera calibration. Reference: on the reference robot, independent measurements from two different head poses differ by about 1 mm on average and about 2 mm at most over the 90 points.
- **The far lines are less than 1 pixel wide in the image**, so the signal is weak. The threshold `--min-prominence` defaults to 3, and false points are rejected on the basis that "samples on the same line should be collinear".
- **When a line is almost entirely blocked by pieces**, it can only be inferred from the other lines in the same direction. These are yellow points, and the error is larger.
- **The points are on the board surface plane.** The piece tops are about 1 cm high, so Z must be considered separately for pick and place.
- **The measured square spacing may not match the physical board.** Reference: on the reference robot the measured file spacing is about 36.0 mm and the ruler measurement about 35.0 mm; the image is about 3% too large laterally (the rank direction is normal), and the error at the edge files is about 4 mm. Before writing back the geometry, measure the distance from file a to file i with a ruler and judge whether the image square spacing can be trusted; when it cannot, do not use `--write-geometry`.
