# Neighbor Relations and Accurate Placement

English | [中文](neighbor_clearance_and_landing.zh.md)

This page supplements steps 2–4 of [round_plan](../prompts/round_plan.md) and is consistent with the pose, grasp and release rules in [README](operation_manual.md). Grasp-and-place judgments separately check the real gripping area, the expected landing point of the piece's bottom face and neighbor relations; service results or FK errors do not substitute for visual acceptance.

## Choose grasp and release poses separately during planning

From this round's head image, list the square names and relative positions of the neighbors near source and target. Each of the two stages prefers a lateral approach, considering the sweep of the whole finger body during descent, closing, opening and retraction; do not look only at the end-effector center. Only when the lateral approach is unreachable, or there is a clear risk of pushing/pinching a neighbor that the lateral candidate cannot eliminate, change the affected stage alone to a longitudinal approach. A lateral grasp does not require a lateral release, perfect zero proximity is not required, and collision must not be asserted from 2D overlap or occlusion.

Estimate source and target XYZ/RPY separately; after changing the release RPY, re-estimate the target tool coordinates and change pose at a clear high position. Rebuild and fully pre-check grasp, retry, pose change / transport while holding, release, retraction, return to init and failure branches; for captures, pre-check both segments jointly. Passing the surface/IK pre-check and collision_checked=false do not prove that whole-tool neighbor collisions have been checked.

## Graspable is enough when grasping

Look only at this step's head image and the physical acting hand's wrist image. Left/right puts the piece between the two fingers' opposing textured faces; front/back puts the piece within the front effective texture segment, not retreated behind the gripper; once both directions are sufficient for gripping, descend, without requiring 100% centering. When not yet graspable, give one combined explicit XY correction, aiming to get there in one go. Source high position at most 3 times: recapture after the first two, descend directly after the third, with no further high-position approval; the existing low-position closing gate is kept.

At the low position, confirm both side walls enter the two fingers' texture and the fingertips wrap the side walls rather than pressing the top, with no visible pushing or pinching of neighbors. If XY is still insufficient, keep the gripper open and retry_high to return vertically to the high position; when more than half of the piece is outside the texture while XY is already graspable and only height causes insufficient coverage, consider adjust_z with a fixed 5mm. Recapture after the first; if it is still limited only by height and the second has been pre-checked, lower again and close directly, with no third low-position gate. A real empty grasp returns to the high position for a recapture; at most 3 real gripper closes, not reset by re-planning; when the user explicitly terminates, retract via an explicit failure.

Neighbor proximity or occlusion by itself is not a reason to refuse a grasp; reason/evidence describes the specific front/back/left/right gripping relation and any obvious risk of pushing or pinching. Do not rotate the wrist at the low position, and do not compensate XY offset by lowering Z.

## Judge the expected bottom-face center when releasing

1. Use the head image to help check the square name; in the wrist image, extend/interpolate along the two sets of grid lines, nearby intersections and neighbor positions to infer the possibly occluded target intersection. Combining the piece's visible contour, gripping eccentricity and top-face/board-surface parallax, estimate the bottom-face center after the gripper opens and state clearly which side of the intersection it is biased toward.
2. Check that the whole piece can drop independently without resting on or overlapping a neighbor; check for obvious interference when the two fingers open and when retracting vertically with the gripper open. An empty target square, an edge covering the intersection, the housing/shadow pointing at the intersection or a small FK error cannot alone prove it is in place.
3. success when the expected center is near the corresponding intersection, only slightly eccentric and releasable. adjust when it is between adjacent intersections, in the middle of a square, clearly off, genuinely impossible to judge, or there is a collision risk; state the basis, and still identify the target and give a justified non-zero XY; do not fabricate identification or displacement. Occlusion by itself does not require adjustment, and no unmeasured pixel/millimeter threshold is set.
4. Keep the current actual Z/RPY and make the XY correction directly at the low position without lifting; at most 3 times: recapture after the first two, and after the third open the gripper directly in place, without adding recapture, approval or an extra "sufficient safety basis already exists" release condition. For the same-side tray, the criterion is the whole piece landing within the usable area of the tray; see [Tray Rules](capture_tray.md).

Source high-position and target low-position XY both have no per-step/cumulative distance limit and no correction-segment motion check; the full plan pre-check before execution, vision and count limits, and handling of actual hardware faults are kept. This is an operating specification; for differences in code capability see [runtime](runtime.md). Execution only submits the existing six/seven fields; material is prepared before execute, and gates do not search history or rebuild the board.

## End-of-move acceptance

From a new head image, check separately that source is vacated, the target is near the correct intersection, and whether neighbors moved/overlap; record offsets and undeterminable cases truthfully. Still only the two review items — visual and offline FK/JSONL; unqualified samples do not enter the success-error accumulation. A new game counts from our round 1; history provides no coordinates or tokens for a new game.
