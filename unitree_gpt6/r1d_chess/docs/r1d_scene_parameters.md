# R1-D scene parameters: meaning and how to determine them

English | [中文](r1d_scene_parameters.zh.md)

The coordinate frame is `waist_yaw_link`, and the units are metres/radians. These parameters are written in `config/robot.json` (profile) and `config/board_geometry.json`. They must be re-measured after the board is moved, the head reference pose changes, or the lift height changes.

The values in this document are model estimates (`model_estimate`) of one reference robot at its site, used to illustrate the order of magnitude and the algorithm; a new robot or a new board must obtain its own values by the same method.

## Gripper and TCP

Basis: the Dex1 drawing and the URDF meshes. Grippers of the same model can reuse these values.

| Parameter | Reference value | Basis |
|---|---|---|
| `tcp_xyz_rpy` | left (0.0485, −0.003, 0.02), right (0.0485, 0.003, 0.02) | Origin of `dex1_base_joint`, that is, the gripper flange |
| `finger_end` | 0.1432 | Flange to fingertip 143.2 mm on the drawing |
| `half_width` | 0.0585 | Fully open outer width 117 mm on the drawing |
| `half_thickness` | 0.005 | The finger narrows from 32 mm at the root to the tip. **Valid only when pitch ≥0.2 rad**: in that case the lowest point is the fingertip, and the finger root is higher |

## Grip point and spec pose (fixed rules)

- The `xy_m` and `z_m` of a spec are the TCP (flange), and the fingers extend along TCP +x.
- The grip point is `GRIP_DEPTH_M = 0.137` from the flange (front part of the gripper): the fingertip is at 143.2 mm, and the grip centre is about 6 mm inside the fingertip.
- Grip point = TCP + 0.137·(cos p·cos y, cos p·sin y, −sin p), independent of roll. With the default pitch 0.8 and yaw −π/2, the offset is (0, −95.4, −98.3) mm.
- Grasp: the fingertip is 3 mm above the board surface.
- Release: the board TCP z of all moves is fixed at 0.150 m, independent of pitch (`BOARD_RELEASE_TCP_Z_M`). Tray TCP z = fingertip height + 0.1432·sin p.
- At pitch 0.8, `gate_z_m` is at least 0.175 (transport clearance 40 mm).
- Tool: `bash r1d pose pose|check` (`src/r1d_chess/chess/grip_pose.py`). `round build` requires the grip point to be ≤12 mm from the intersection of the square written (less than half a square), and ≤30 mm for the tray; placing the TCP directly on the intersection is off by 123 mm and is rejected.

## Board (`config/board_geometry.json`)

Quantities needed: board thickness, column spacing and row spacing, piece diameter and thickness (measured with a ruler), board surface height `surface_z`, board centre and rotation angle, and the 90 intersections (measured with the head camera).

- Measurement tools: `bash r1d boardcal` (registers the printed pattern and refines it), `bash r1d boardpoints`, `python scripts/head_board_measure.py` (includes the board surface height); see [board_position](../board_position/README.md). The prerequisite is an existing head camera calibration.
- The `uncertainty` of `surface_z` must cover the differences between measurement methods.
- The grid fit is periodic and may be off by one square without a prior; add `--prior config/board_geometry.json` when re-measuring.

> Reference: the reference board is 20 mm thick, with a grid spacing of 35.6 (column) × 33.7 (row) mm and pieces of ø29.7×9.9 mm; board surface z = +11.8 mm (stereo measurement), while another method gives +9 mm, a difference of 2.8 mm, so `surface_z=0.012` and `uncertainty=0.004` are used.

## Release height

The `board_release_z_m` and `tray_release_z_m` of the profile are given at `release_pitch_rad` (0.35); for other pitch values they are converted by fingertip height: TCP z = fingertip z + 0.1432·sin(pitch).

| Parameter | Reference value | Fingertip height |
|---|---|---|
| `board_release_z_m` | 0.0741 | z=25 mm, about 13 mm from the board surface |
| `tray_release_z_m` | 0.1091 | z=60 mm, about 71 mm from the table surface |

When releasing into the tray, the fingertip must be higher than the cup rim; confirm this from an image before first use.

> Reference: to judge whether a given pitch is reachable, check it with the real descent method (high position → descend in 5 mm steps, chained seeds). Single-point IK wrongly reports unreachable because the seed is too far away. On the reference robot, the left hand is reachable over pitch 0.35–1.0, with a wrist pitch margin ≥0.22 rad.

## Head lock pose

The head is locked at `head_q_rad` in `config/robot.json` (reference value pitch 0.62, yaw −0.0036, close to the lowest head position). The board is therefore at the centre of the image, with less distortion.

- After changing the head reference position, re-measure the board geometry and confirm that the head camera calibration is still usable at that head pose.
- The head yaw has backlash (reference value about ±15 mrad). During measurement, the direction of the last yaw motion must be fixed, and afterwards the head must return to the reference position from the same direction every time.

> Reference: on the reference robot, the board was measured at the two head poses pitch 0.344 and 0.624; the board surface z differs by 1.3 mm, the centre by 0.1/0.6 mm, and the rotation angle by 0.7 mrad, which shows that the calibration is usable in the head-down pose. With the head lowered, a far-end square is about 11.5×17.7 px and a near-end square about 26×27.5 px.

## Table surface clearance criterion

Clearance = lowest corner of the gripper envelope − board surface − `uncertainty`. The envelope already includes the finger half thickness.

| Check | Lower limit | Physical meaning |
|---|---|---|
| Planning (planned poses: path, opening and closing the gripper, undo actions) | `minimum_clearance` = 0 | The envelope is not lower than "board surface + uncertainty" |
| Runtime (measured poses: feedback during interpolation, before opening and closing the gripper, release checkpoints and low-position XY correction) | −`uncertainty` | The envelope is not lower than the board surface estimate itself |

The runtime lower limit is looser than the planning lower limit by one uncertainty, because the static tracking error of the arm is a few millimetres and the sag is directed exactly downwards; with the same lower limit for both, descending to the release height easily raises a false fault.

The grasp low position is still below the planning lower limit: the fingertip must be lower than the top surface of the piece, so each round uses `ignore_source_tool_envelope`, and the vision gate does the checking.

## Two-level support surface: board and table

`scene` has a two-level support surface, because the init pose rests at the two sides of the board, above the table surface, which is lower than the board surface by one board thickness:

- Inside the board footprint, the board surface `surface_z` is used. The footprint is the intersection range extended outwards by a 30 mm border: centre `board_centre_xy`, orientation `board_yaw`, half extent `board_half_extent` (along the rows, along the columns).
- Outside the footprint, the table surface `table_z = surface_z − board thickness` is used.
- As soon as the bounding rectangle of the gripper envelope in the board frame overlaps the board footprint, the whole envelope is evaluated against the board surface (conservative).
- These fields are either all given or all omitted: when all are omitted, a single plane is used, and giving only some of them is rejected. Under the single-plane model, the hand at init is computed as being below the board surface and is rejected by preflight.

The tray and other objects on the table are not modelled.

## Known limitations

- The grasp low position is not within the clearance requirement of this set of parameters; see "Table surface clearance criterion" above.
- Preflight does not include collisions between the arm and the pieces or the tray.
- The accuracy of the model estimates depends on the head camera calibration; the calibration residual and the head pitch zero offset both go directly into the board geometry.
