# Playing Black in Gomoku: Operating Rules

English | [中文](operating_guide.zh.md)

These are the operating conventions for autonomous Gomoku, executed by the host agent together with the autonomous round tools. The agent recognizes the game from images and completes visual alignment and clearance checks, the engine selects the move automatically, and the hardware tools execute mechanical motions and provide feedback. For the detailed hand-off see [Autonomous execution flow](autonomous_play.md).

## Task

The robot plays Black. Each round it uses the right arm and right gripper to pick one black stone from the tabletop stone-supply area and place it on the currently selected empty intersection, followed by `reset_right`. Choose a single, visible and reachable stone in the supply area; do not take stones from the stone box and do not move stones already on the board. Keep the waist angle from task start; the gripper must not collide with the table, the board or other stones.

The autonomous closed loop uses the four raw images `cam_left_high`, `cam_right_high`, `cam_left_wrist`, `cam_right_wrist`, as well as dual-arm, waist and gripper feedback. The hardware tools provide these data sources; the host agent actually reads and reviews them.

## Gomoku rules

19×19 board, Black first and White second, the two sides alternate, and each move places only one stone on an empty intersection. An existing game continues from the current turn. By default there are no forbidden moves; five or more consecutive stones of the same color along a row, column or either diagonal wins. Once placed, stones are not moved and there are no captures; the tabletop supply does not count as part of the game. After each completed black move, return home and wait for White to move and for the user to start the next round.

## Game assessment and landing-point annotation

In the rest pose, cross-check the board, grid and stones with the two head raw images. The left head image orientation is authoritative: rows from top to bottom, columns from left to right, numbering starts at 1, and the outer frame does not count as a grid line. Locate intersections on the board surface; do not treat the projection of a stone's top surface as the intersection.

Submit the current complete recognition result to the automatic move-selection engine: first look for Black's chance to make five with this move, then handle White's threat of making five on the next move; in other cases balance attack and defense. A four blocked at both ends does not count as an immediate winning position. The target must be a single empty intersection in the current position.

Create an independent runtime directory for each move, saving the four raw images, the landing-point annotation and `position_and_move.json`. The annotated image circles only one intersection, labeled "Black landing point" ("黑落点"), with row/column and a brief reason; the JSON records the coordinate convention, the rows/columns of black and white stones, the target row/column and raw-image pixels, the reason and image paths. The actual placement must match the record; if the game changes, reassess. When only an assessment is requested, delivering the annotation is enough; when execution is requested, continue with the full pick-and-place flow.

## Pose and height

The TCP uses the pelvis frame `[x, y, z, roll, pitch, yaw]`, positions in meters, angles in radians, rotation `Rz(yaw) @ Ry(pitch) @ Rx(roll)`. The TCP is determined by the end-effector definition of the in-package model; it is neither the fingertip nor the stone center; planning must compensate for gripper length, orientation and held-stone offset. When pitch is close to 90°, compare actual rotations, not only Euler values.

The project provides no empirical defaults for grasp, preview, short-lift or placement heights or for gripper opening. The first time, solve them from the current images, feedback and scene calibration; after success write them into the workspace's structured experience. `target` and `measured` must be kept separate; images, measured poses and feedback determine the current final pose. Do not treat the TCP height as the surface height of the table, board or stones, and do not make the user responsible for camera-to-body calibration.

## Execution flow

First return home at the start, saving the four raw images and the arm, waist and gripper states. One move is divided into four mechanical stages:

1. **Tabletop stone picking**: from a safe height to above the stone, correct at the preview position, then at a low position verify the inner edges of both fingers, the stone center and the fingertip contact line. Keep the arm position and close slowly, lift vertically a short distance, and confirm that only one stone was lifted, the original spot is now empty and nearby supply stones are undisturbed. If the grasp misses or the stone shifts, relocate based on the current image.
2. **High-position transfer**: first estimate the target XY and held-stone offset and verify clearance, then go from a safe height to above the target vicinity. Fine-tune using the neighboring stones and the vertical and horizontal grid lines in the right wrist image, completing the main correction before going low.
3. **Prepare placement**: verify the empty intersection and descend in segments along the same XY. Prepare the complete return before user confirmation: lift vertically along the placement line, reuse in reverse this round's executed high-position route whose clearance has been reviewed, then join the rest-pose corridor. If the final XY changes, re-validate the connecting segments. Explain the landing point, release and return to the user, and obtain release confirmation.
4. **Continuous return home**: after confirmation, in the same healthy control session open the gripper slowly, then immediately and continuously lift, exit the board and return home. After placement do not plan the return ad hoc or switch the publisher; the necessary runtime health and start-point checks must still be retained.

## Points of attention

Complete the game assessment and the main route preparation while in the rest pose, to reduce waiting in mid-air. First move quickly to above the target vicinity, try to align with one or two well-founded fine adjustments, and do not think for a long time at a low position over the board.

When grasping, keep the gripper perpendicular to the table; the two fingers may lightly touch the table but must not strike it. The wrist camera is not parallel to the gripper, so do not rely only on the image center. Before the first fine adjustment, use this round's executed pose changes and the image displacement relative to the fingertips to determine the direction and ratio of X/Y adjustments; review after every correction. Appearing centered at the preview position does not mean it will still be centered after descent. Before closing slowly, let both fingers make balanced contact to avoid flicking the stone away.

Stone picking and transfer may be executed autonomously under the authorized task; the user confirmation point is after alignment and before release. After confirmation save the four-way images only once, without viewing, analyzing or waiting for visual confirmation; afterwards do not capture images again, keep only the necessary control feedback, and continuously lift and return home. Therefore the completion report must not claim that the exact post-placement intersection has been visually accepted.

Ordinary execution deviations should be recorded, corrected, and the move continued to completion. When a loss of connection, a protective stop or a collision risk occurs, first stop subsequent motion and determine the state; do not report an unfinished action as success, and do not blindly replay an old target.

## Constraints and records

The board's outer edge, grid spacing, and table and board-surface heights are determined by the current scene calibration. Existing experience serves only as a candidate reference; each round re-verify against the current game, waist angle, images, held-stone offset and clearance.

Before execution explain the action and the complete target TCP; after completion record the measured TCP. Before release confirmation save raw images, dual-arm/waist/gripper feedback, plans and results per stage; images after confirmation are saved only once. Video is not recorded by default. Externally it is enough to explain the target and result; the reference-lookup process stays in the background records.

For the general methods see [Manipulation methods](manipulation.md); successful data is saved according to [Experience data conventions](experience.md). Distinguish joint return home, TCP in position, release completed and board landing-point verification; passing one of these cannot vouch for the others.
