# Head camera calibration (eye-on-head)

English | [中文](README.zh.md)

## When to use
- A new R1-D is brought up for the first time.
- After the head camera has been removed and refitted, or after a collision.
- The board or piece positions measured from the head image do not agree with the arms.

## What is calibrated
- **Intrinsics**: the fisheye intrinsics of the left and right eyes, and the stereo extrinsics (baseline).
- **Camera mounting extrinsics**: the fixed transform `X` of the left eye relative to `head_yaw_Link`.
- **Head joint correction terms**: the scale error of head pitch/yaw (the actual rotation is smaller than the encoder reading) and the yaw backlash.

The live camera pose is computed as follows:

```
T_waist_cam = T(pitch origin)·Ry(q_p')·T(yaw origin)·Rz(q_y')·X
q' = pivot + (q - pivot)·(1 + scale), and for yaw add backlash × direction of the last motion
```

The following are not calibrated:
- **Head pitch zero offset**: it is coupled with the pose of the board and cannot be solved separately; only an upper bound can be given from the levelness of the board surface.
- **Waist encoder**: the waist angle is not used in the solve.

## Calibration board (fixed specification; changing the board requires changing the board definition in all scripts at the same time)
ChArUco, 5×7 squares, square 35 mm, marker 26 mm, `DICT_5X5_100`, the new-style pattern of OpenCV ≥4.6. Print at 100% scale, check the square side length with a ruler, and stick it on a rigid flat plate.

## Prerequisites
- `camera_host` and `model_urdf` in `config/runtime.json` are correct, and `config/robot.json` has `head_q_rad`.
- Stage 2 needs `r1d-runtime.service` to be running, with no other control session. `config/arm_torque_ff.json` should preferably already be tuned, because stage 2 holds both arms with feedforward.

## Safety and authorisation
- Stages 0, 1 and 3 are read-only and send no commands to the robot.
- **Stage 2 moves the head**, and every run must be explicitly authorised: velocity ≤0.2 rad/s, about 5 minutes, both arms held in place, lift and base not moving.
- A tracking error above 0.15 rad, over-temperature or stale feedback aborts automatically, and the torque is ramped down to 0 before ending.

## Steps
All commands are run from the package root, with `H=setup/head-camera-calibration/scripts`, `O=runs/head_calibration`.

**Stage 0: read-only check**
```bash
python -B $H/stage0_check.py --output $O/stage0
```
Confirm that the head stream is 1280×480 (left and right eyes concatenated), and note the valid column range of each eye; and, with the board in the field of view, how many corners can be solved (24 in total).

**Stage 1: intrinsics and baseline** (the robot does not move; the operator holds the calibration board by hand)
1. Record 150 s of raw JPEG:
   ```bash
   python -B $H/head_calib_record.py --output $O/stage1_video --seconds 150
   ```
   - Keep the board 20–35 cm from the camera and move it slowly, no more than about 10 cm per second.
   - Go in order: centre → four corners → left and right edges, tilting 30–45° up, down, left and right at each place.
   - The four corners and the edges matter most (the fisheye distortion is concentrated there); do not cover the squares with your hands.
   - When capture guided by grid cell is needed, use `head_calib_intrinsics_capture.py`.
2. Detect the corners, select frames and fit the intrinsics (`fisheye` model):
   ```bash
   python -B $H/head_calib_detect.py $O/stage1_video
   python -B $H/head_calib_intrinsics_solve.py $O/stage1_video
   ```
3. Fit the stereo extrinsics:
   ```bash
   python -B $H/stereo_fit.py $O/stage1_video
   ```
4. Lay the board flat and fix it in the playing area. **From here until the end of stage 2, neither the board nor the table may move.** After placing it, run stage 0 once more and confirm that corners can be solved in both eyes.

**Stage 2: extrinsics capture** (moves the head)
```bash
python -B $H/head_calib_capture.py --output $O/stage2 --head-only --confirm-motion
python -B $H/capture_check.py $O/stage2
```
- `--head-only` makes the head go through 59 poses, with the waist and arms not moving. Do not use the waist-turning mode.
- `capture_check.py` checks:
  - the corner jitter should be ≤0.1 px;
  - the corner drift over the three returns to the reference position `ref_start`, `ref_mid`, `ref_end` should be ≤1 px; if it is clearly larger, the board or the base has moved, and this batch of data is void and must be captured again.

**Stage 3: solve and acceptance** (offline)
```bash
python -B $H/head_calib_solve.py --stage2 $O/stage2 --intrinsics $O/stage1_video/intrinsics_fit.json \
   --stereo $O/stage1_video/stereo_fit.json --output $O/stage3_final.json --config config/head_camera_calibration.json
```
- The initial values are computed by closed-form hand-eye calibration; the pivot of the scale model is the median head pose of this capture. Both are computed fresh from the data and do not depend on any existing calibration.
- `--config` is written only when all acceptance items pass.
- When some items do not pass and the operator decides to accept, add `--accept-note "<reason>"`; the items that did not pass are recorded in the configuration together with the reason.

**Optional: visually check whether the camera and the arms agree**
```bash
python -B $H/arm_overlay.py $O/stage2 --output $O/arm_overlay.jpg
```
The projected arm FK points should fall on the real arms in the image. This is a visual check, not a quantitative acceptance.

## Acceptance criteria (judged automatically by `head_calib_solve.py`)
| Item | Criterion |
|---|---|
| Intrinsics held-out frame RMS | ≤0.5 px |
| Held-out pose reprojection RMS / P90 | ≤1 / ≤1.5 px |
| Cross-fit camera position / orientation change | ≤2 mm / ≤2 mrad |
| Cross-fit board centre change | ≤3 mm |
| Board surface level | ≤1° (provided the table surface is level) |

Additional independent checks are recommended: measure the distance between the two lens centres with a ruler and compare it with the stereo baseline (≤2 mm); measure the height from the board surface to the table surface and compare it with the solved board height.

## Outputs
- `config/head_camera_calibration.json`: K, D, `T_right_from_left`, `X_head_yaw_to_left_camera`, the scale pivot, `scale`, `backlash`, each acceptance item, the URDF hash, the acceptance record.
- All raw data is saved under `$O/` and can be recomputed.

## Common pitfalls
- **Looking only at the fit residual is misleading**: when intrinsics and extrinsics are fitted together and the range of views is narrow, the residual can be very small while the error on the table surface reaches several centimetres. Acceptance must look at held-out poses and the cross-fit.
- **Solving the left and right eyes separately gives seemingly contradictory baselines**: use the joint solve in which the two eyes share one camera rig, that is, the Rig model of `head_calib_solve.py`.
- **The head yaw has backlash**: fix the arrival direction in use, for example always return to the reference position from the negative direction; otherwise there is an error of a few millimetres on the table surface.
- **The head joints have a scale error**: dividing the board rotation measured by PnP by the encoder rotation gives an independent check.
- **The farther from the reference pose, the less reliable the extrapolation**: the locked head pose must fall within the pitch range captured in stage 2, or only slightly beyond it.

> Reference (reference robot): the head yaw backlash is about ±15 mrad, and the head joint scale error is about 1–2%; the waist encoder reading does not agree with the rotation at the output side, and in the waist-turning mode the waist actually hardly rotates, so the solve does not use the waist angle. The cross-fit orientation item is 2.76 mrad, slightly above the 2 mrad criterion, and is used after acceptance by the operator.
