# Grasp-and-Place Experience and Reusable Boundaries

English | [中文](experience.zh.md)

This page explains the grasp-and-place judgment methods used in [README](operation_manual.md) and [round_plan](../prompts/round_plan.md). Coordinates, gripper opening values, poses and waist state must be verified on the current device. On a new device, read [First-Time Setup](first_game_setup.md) first.

## Key points for each round

| Stage | Reusable success points | Commonly repeated problems |
| --- | --- | --- |
| Planning | Confirm the actual position from a new head image; estimate XYZ/RPY separately for source/target; choose the preferred hand by source, and switch hands only if the full pre-check fails | Carrying historical coordinates, source offsets or tool coordinates from another pose over to target; passing off single-point IK as whole-move reachability |
| Source high position | Left/right between the two opposing textured faces, front/back within the effective front gripping segment; give one combined XY estimate aiming to be graspable | Chasing 100% centering; probing one direction at a time; looking only at the projection of the jaw opening and ignoring the piece sitting behind the gripper |
| Source low position | Both side walls of the piece enter the effective texture of the two fingers, the fingertips can wrap the side walls, no obvious pressing on top / pinching a neighbor | Compensating XY offset by lowering Z; refusing to grasp based only on occlusion or 2D proximity |
| Grasp success and transport | Once the stationary closed-gripper feedback meets the current grasp-verification criterion, transport continuously and log the raw snapshot asynchronously | Using a non-empty gripper as proof that the specified piece was grasped, or searching material / computing FK while holding the piece |
| Target low position | In the wrist image, infer the target intersection along visible grid lines and neighbors; estimate the bottom-face center after release and the drop space for the whole piece | Approving just because an edge covers the intersection; treating the housing/shadow/top face as the landing point; letting a small FK error mask an obvious wrong square |
| Consecutive capture | Jointly pre-check both segments for the same hand, one execute; after the gripper is confirmed open at the same-side tray, go from the high position directly to our own source | Returning to init after the first segment, repeating open/enable, pre-checking again, causing extra pauses |
| Wrap-up | From a new head image verify separately that source is vacated, the target is on the square and neighbor changes; offline, merge the three errors of the two events with the JSONL | Using placed in place of position acceptance; adding an extra full report; counting unclear/failed samples in the mean |

## Correction counts and graspable is enough

Source high-position XY at most 3 times: recapture immediately after the first two, descend directly after the third; the existing low-position closing gate after descending is kept. Once both directions are sufficient to grasp, attempt it; do not chase pixel-perfect centering. When low-position XY is insufficient, keep the gripper open and retry_high; do not use ordinary insufficient centering to trigger failure back to init. A real empty grasp returns to the high position; at most 3 real gripper closes, accumulated across re-plans.

If more than half of the piece is outside the textured contact area, while XY is already graspable and only the height causes insufficient effective contact area, consider adjust_z to automatically lower vertically by 5mm. Recapture after the first; if it is still limited only by height and a second lowering has been pre-checked, lower again and close directly, with no third low-position gate. Lowering Z must not be used to correct front/back offset or obvious neighbor interference.

Target low-position XY keeps the current actual Z/RPY without lifting; recapture after the first two, and after the third open the gripper directly in place, with no added visual approval conditions. Source high-position and target low-position corrections have no per-step/cumulative distance limit and no correction-segment motion check; the full plan pre-check before execution, vision and count limits, and handling of actual hardware faults remain. For differences in code capability see [runtime](runtime.md).

## Coordinate and acceptance boundaries

`actual_relative_offset_xy_m` is relative to the actual FK at the current checkpoint; the submitter converts it into the service reference offset, and it must not be mixed with cumulative reference quantities. Source and target are independent; a change in the source gripping position changes the piece's eccentricity relative to the tool, so the same EE target does not guarantee the same landing point. After changing the release RPY, re-estimate target XYZ and change pose at a clear high position.

The requested Z is the model EE coordinate, not the fingertip clearance height or the piece's bottom-face height. There is parallax between the piece's top face and the board-surface intersection, and the head image and wrist image are not the same projection. Small local fitting residuals, small FK errors, stable non-empty contact and service placed each provide only local evidence and cannot alone prove accurate square placement or that neighbors were undisturbed.

The actual waist must be locked at zero and verified; the model's assumed zero waist cannot replace the measurement. After the board, robot position or waist pose changes, re-mark from a new image with [board_position](../board_position/README.md). init, gripper-open q, grasp height and pose must all be confirmed on site.

## Execution rhythm and review

Before motion, prepare all scripts, stages and the six/seven-field evaluation structure. Execution only does next, looks at this step's head + acting-hand wrist images, decides immediately and submits, then continues with next; 5 seconds is a response target, not a guaranteed latency. Gate reason/evidence records the current identity, the bias of the gripping area or expected landing point, and neighbor relations; do not search history or change code.

The end still has only two items: ① judge move success/offset from a new head image; ② offline, compute the three errors of the two events — grasp success and low-position approved release — relative to the original plan, the final effective target and the commanded FK at that moment, and combined with the visual result, finish takes over the JSONL and generates the named review. Historical errors are only a reference and cannot be mechanically compensated; 25mm only filters learning samples. For details see [Error Loop](xy_error_learning.md).
