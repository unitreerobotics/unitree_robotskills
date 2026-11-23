# Chess scene parameters

English | [中文](README.zh.md)

## When to use
- A new R1-D is brought up for the first time.
- After the board, the table, the lift height or the head reference pose has changed.
- `round build` or preflight is rejected because of a geometry problem.

## Which parameters are written
All parameters are in the `waist_yaw_link` frame, in metres/radians, and are all marked as `model_estimate`; model estimates can be used directly for execution. For the meaning of each parameter, see [docs/r1d_scene_parameters.md](../../docs/r1d_scene_parameters.md).

| Parameter | Source |
|---|---|
| `init_joint_rad`, `head_q_rad`, `waist_q_rad`, `lift_height_m` | Read-only snapshot of the current robot |
| `tcp_xyz_rpy` | Origin of `dex1_base_joint` in the URDF, that is, the gripper flange |
| `init_xyz_rpy` | Computed by FK from `init_joint_rad` |
| `scene` (tool envelope + two-level support surface) | The gripper drawing, plus the board surface measured by the head camera; outside the board footprint the table surface = board surface − board thickness is used (`--board-thickness`; the footprint border uses `--board-border`) |
| `board_release_z_m`, `tray_release_z_m`, `release_pitch_rad` | Board surface / table surface + fingertip clearance + finger length·sin(pitch) |
| `kinematic_model_sha256` | Hash of the current URDF |
| `config/board_geometry.json` | Board centre, rotation angle, 90 intersections and tray positions measured by the head camera |

## Prerequisites
- Step 1 only needs the robot runtime (`r1d-runtime.service`) to be running.
- From step 2 on, [head-camera-calibration](../head-camera-calibration/README.md) must be completed, and the URDF hash in `config/head_camera_calibration.json` must match the current model.
- `model_urdf` in `config/runtime.json` points to the R1-D URDF that includes Dex1 (`assets/models/r1d/r1d.urdf` in the package).
- The board parameters are known (pitch, board thickness, piece size, measured with a ruler), and the gripper drawing is known (flange-to-fingertip length, fully open width).

## Safety
- Everything except step 3 is a read-only or offline operation, but `r1d-runtime.service` must be running, to read the joint and lift state.
- Step 3 with `--single-pose` turns the head (≤0.2 rad/s, waist and arms held) and requires authorisation.

## Steps
All commands are run from the package root, with `C=setup/chess-scene-parameters/scripts`, `H=setup/head-camera-calibration/scripts`, `O=runs/scene`.

1. **Record the initial pose.** Put the robot in the pose it has at the start of each round (both arms resting at the two sides of the board, grippers closed, fingertips leaving clearance above the table surface), and keep it stationary:
   ```bash
   python -B $C/capture_initial_pose.py --write                         # head reference taken from the current measurement
   python -B $C/capture_initial_pose.py --write --head-q <pitch> <yaw>   # or lock a pose close to the lowest head position (recommended, the board is at the centre of the image)
   ```
   The locked head pose must be within the pitch range captured in the head camera calibration, or only slightly beyond it. The arrival direction of the head yaw must be fixed (for example, negative).
2. **Check the gripper dimensions.** Cross-check the URDF meshes and the drawing against each other:
   ```bash
   python -B $C/dex1_envelope.py
   ```
   The flange-to-fingertip length and the fully open width should match the drawing (143.2 mm and 117 mm on the Dex1 drawing). If they do not match, go by the physical part or the drawing, and pass the values in through the arguments of `write_scene.py`.
3. **Measure the board.** This needs one raw stereo image with the head at the locked pose, and the head angles at that time. The chess service cannot take over while there is no complete geometry yet, so use the calibration capture tool to turn the head there and capture the image. This step **moves the head** (only the head; waist and arms are held) and requires authorisation:
   ```bash
   python -B $H/head_calib_capture.py --output $O/locked_view --single-pose <pitch> <yaw> --confirm-motion
   ```
   It turns to that head pose, captures 6 frames, and returns to the reference position in `robot.json`. The image is at `$O/locked_view/001_*/0.jpg`; take the head angles from `poses[1].head_q` in `manifest.json`; the yaw direction is the sign of this yaw motion.
   If the chess service has already taken over and the head is already resting at the reference position, you can use `client capture_cameras` instead (it saves the raw `*_stereo.jpg` image), and take the head angles from `head_q` in the returned state, without turning the head.
   ```bash
   python scripts/head_board_measure.py --image <raw stereo image> --head-q <pitch> <yaw> --yaw-direction <-1|1> --output $O/board_measure.json
   ```
   - The board surface height is obtained by a stereo plane sweep: search for the h that makes the top-down views of the left and right eyes on the plane z=h agree.
   - The grid is fitted with the known pitch.
   - **Always look at the output `*_grid.jpg`**: the grid is periodic and may be shifted as a whole by one square without a prior. When this robot's own `config/board_geometry.json` already exists, add `--prior config/board_geometry.json` to search only near the known position; the board geometry of the reference robot cannot be used as a prior.
   - The tray centres can be read from the top-down view (error about ±10 mm).
4. **Generate the parameters.** Do a dry run first, and write only after confirming:
   ```bash
   python -B $C/write_scene.py --measure $O/board_measure.json --pitch 0.35 --trays LX LY RX RY          # dry run, prints the values and the changes
   python -B $C/write_scene.py --measure $O/board_measure.json --pitch 0.35 --trays LX LY RX RY --write
   ```
   - The defaults are all relative quantities: for a board release the fingertip is 13 mm above the board surface; for a tray release the fingertip is 68 mm above the table surface (it must be higher than the tray cup rim; confirm this from an image before the first capture).
   - Board thickness, pitch, piece size, finger length, half width and `uncertainty` can all be changed through arguments.
5. **Check reachability** (only an IK spot check):
   ```bash
   python -B $C/reach_check.py --pitch 0.35 0.8 --yaw -0.5 0 0.5 1.0
   ```
   - Confirm that the left and right hands, over their orientations combined, cover the whole board. Moves actually use pitch 0.8, the release heights of the profile are calibrated at the value given by `--pitch` (0.35), and the conversion is done by `round build`.
   - The single-point IK spot check is pessimistic; with the real descent method (stepwise descent from the high position, chained seeds) the reachable range is larger, and the full preflight is the final authority.
   - After changing `--pitch` of step 4, rerun step 4: the release height and the pitch are tied together.
6. **Evaluate the view (optional):**
   ```bash
   python -B $C/view_resolution.py --image <raw stereo image> --head-q <pitch> <yaw> --overlay $O/view.jpg
   ```
   Compare the pixel sizes of squares and pieces at different head poses, and whether the projections of the 90 intersections fall on the real intersections.
7. **Verify the gate:**
   ```bash
   python -B -c "import sys;sys.path.insert(0,'src');from r1d_chess.robot.r1d_config import load_profile,require_calibration;require_calibration(load_profile(),hardware=True);print('ok')"
   ```
   Then pick one real move and do only planning and the full preflight, without executing.

Afterwards, when only the board has been moved, re-measuring the board geometry with `bash r1d boardcal` is enough (see [board_position](../../board_position/README.md)); when the board surface height or the table has changed, redo steps 3 and 4.

## Acceptance criteria
| Item | Criterion |
|---|---|
| Two board surface estimates | "The board surface from the stereo plane sweep" and "the board surface solved from the calibration board − the measured board thickness" differ by ≤5 mm |
| Re-measurement at another head pose | Re-measure at a second head pose (with `--prior`); the board centre and the board surface change by ≤3 mm |
| Grid overlay image | Every piece falls on an intersection |
| Gate | `require_calibration(hardware=True)` passes, and the full preflight of a real move passes |

## Outputs
- `config/robot.json`, `config/chess_initial_pose.json`
- `config/board_geometry.json`
- `$O/board_measure.json`, `*_grid.jpg`

## Common pitfalls
- **Precondition of `half_thickness`**: it assumes pitch ≥0.2 rad during pick and place, in which case the lowest point is the fingertip and the finger root is higher.
- **The grasp low position is below the tool clearance**: when grasping, the fingertip must be lower than the top surface of the piece, which is below the clearance requirement of the tool envelope. Each round must enable `ignore_source_tool_envelope`, and the vision gate does the checking.
- **The scene parameters are sensitive to changes in the environment**: when any one of the lift height, the board position or the head reference changes, redo steps 3 and 4. When the lift height deviates by more than ±5 mm at take-over, the service refuses.
- **A single-round profile can only be derived from `config/robot.json`**, and must keep `kinematic_model_sha256`. When this field is missing or does not match the current URDF, both the service and preflight refuse.
- **Changing `robot.json` changes the profile digest**, and the landing error samples accumulated before (`xy_error_context`) no longer apply.
