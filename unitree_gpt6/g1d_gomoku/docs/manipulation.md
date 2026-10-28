# Grasp, Landing Point and Return Experience

English | [中文](manipulation.zh.md)

This document describes only general operating methods; it does not provide on-site dimensions, poses, joint angles or gripper openings. On the first execution, solve them from the current images, feedback and calibration; after success save the measured results according to [Experience data conventions](experience.md).

## More accurate placement

In the rest pose first number the 19×19 grid, save the current black and white stones and the single target empty intersection, and mark it on a copy of the left head image. After the high-position transfer reaches the target vicinity, use the rows/columns of neighboring stones and the vertical and horizontal grid lines in the right wrist image to confirm the same intersection; the main XY correction is completed at the preview height, and finally descend along the same vertical line.

The grid spacing obtained from the current calibration can be used for coarse positioning and target-consistency checks, but cannot be treated as the actual placement accuracy on the board. The center of a stone's top surface, the center of the gripper in the image, and a very small TCP tracking error each cannot on their own prove that the stone is aligned with the board intersection.

## More reliable first closure when grasping

Move the alignment correction ahead of the first closure: first correct at the preview position, then at the low position re-verify the inner edges of both fingers, the stone center and the fingertip contact line, and finally close slowly.

The key is to move the decision conditions earlier, not to save a fixed XY offset. Descending changes the perspective: centered at the preview position, it may still lean against one fingertip at the low position. Correct each time based on the current image; after a failed grasp, use the current position of the disturbed stone. The gripper closing angle is not evidence of success; after the short lift you must see a single stone following the gripper, the original spot on the table empty, and nearby supply stones unmoved before transferring.

## Zigzag arm-shape prior for grasp alignment

When aligning to grasp a stone, prefer comparing the "之"-shaped (zigzag) arm-configuration branches formed by the coordination of shoulder, elbow and wrist. The "之" shape here is the spatial shape of the whole shoulder–upper arm–elbow–forearm–wrist chain; it is not merely turning the gripper to face down, and it cannot be judged from the sign of the elbow angle alone. Read this section before planning the grasp approach; verify the arm shape with the head images, the fingertips and stone with the wrist images, and then confirm the branch with joint feedback.

After approaching the supply area at a high position, descend to the preview position, keep the gripper facing down, complete small XY corrections, then descend along the same vertical line. After a secure grasp at the low position, only do a short lift for verification, then plan the high-position transfer. Do not keep increasing height, flipping the wrist, or letting the shoulder and elbow switch to another IK solution set for the sake of "lifting first", only to deal with it when the wrist joint is near its limit.

The agent handles planning in the following order:

1. Starting from the current measured joints and the task waist angle, first determine the safe corridor of the supply area, the preview position and the gripper orientation. Verify the shape of the whole right arm in the head images; avoid focusing only on the TCP or the wrist image center.
2. Compare the shoulder yaw, shoulder roll and wrist coordination of the candidate solutions, keeping the left arm fixed. Existing experience may assist the solution only when the robot, scene, configuration and calibration are consistent; it must not pose as current feedback or as the start point of the execution plan.
3. Within the safe space generate a continuous path from the actual start point to the candidate branch, and check the clearance of the whole segment, speed, joint margins, and whether the subsequent descent and short lift are reachable. Do not check only the end point, and do not jump directly from the current posture to a cached joint vector.
4. Approach, descend and short-lift segment by segment using adjacent solutions that have passed the checks. If a branch switch is needed, complete it first within the reviewed safe corridor; after approaching the stone do not flip the wrist ad hoc or raise the shoulder substantially. When adjusting height or orientation, recheck the fingertip contact line and all subsequent stages.

A similar arm shape does not mean being far from the limits. Check the joint margins of the whole path and of the subsequent descent and short lift; do not relax the limits for the sake of similar appearance. When there is no suitable solution, adjust the approach plan at a safe position.

The current `Kinematics.solve()` has only joint hard limits and a very weak seed term toward the input; it has no zigzag preference, no joint-margin soft constraint and no automatic multi-branch selection. The seed of the standard motion tool comes from the current control state, so merely recording the same TCP does not guarantee the same arm shape. The solution selection and arm-shape review above are planning requirements for the host agent; this document does not mean the solver already implements them automatically.

## Faster return-home preparation

Put the return preparation before user confirmation. At the low position go up along this round's placement vertical line; at the high position reuse in reverse this round's executed outbound route whose clearance has been verified, and join the reviewed rest-pose corridor. If the final XY changes, you only need to first rebuild the corresponding short connection, then verify the continuity of the whole path, joint bounds, waist angle and left-arm hold. Old round routes must not be replayed directly.

After confirmation, in the same control session continuously open slowly, lift, exit and return home, without solving routes ad hoc. When evaluating time consumption, record planning time, planned motion duration and actual completion time separately. At runtime still check fresh feedback, faults and the plan start point.

## Other improvements and evidence boundaries

- **Reduce hovering**: handle assessment and recording in the rest pose, and avoid long waits at low positions. When a protective exit such as stale DDS state occurs, troubleshoot based on feedback and fault logs; do not judge the cause from a single temperature reading.
- **Heights are only priors**: grasp, short-lift, placement and preview heights are solved from the current scene, recorded separately as experience after success, and approached vertically in segments. The TCP height is not a measurement of the contact surface.
- **Distinguish pose representations**: when pitch is close to 90°, Euler angles are not unique; compare rotation matrices or rotation errors. Passing joint return home does not equal the TCP being in Cartesian position.
- **Image fitting does not equal placement accuracy**: the fitting residual of a single image's grid homography describes only the image-fitting quality and cannot be used as the actual placement accuracy.
- **Re-verify after interruption**: distinguish stages such as approach, closure, secure grasp and placement; the pose and authorization state before an interruption must not be treated as the current state.
- **Itemize the completion report**: passing rest-pose feedback does not mean the exact landing point has been visually accepted. A protective exit must not be reported as a normal voluntary disconnect, nor can the real-time posture after disconnection be inferred.

This experience is used in the [Autonomous execution flow](autonomous_play.md) and the [Operating rules](operating_guide.md). The host agent actually inspects images, fine-tunes and checks clearance, and the automatic engine selects the move; for the control and capture layer see [Hardware integration](hardware_integration.md).
