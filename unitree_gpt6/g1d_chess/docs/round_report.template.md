# Prompts for the Two Review Fields

English | [中文](round_report.template.zh.md)

This page is the fill-in guide for [operation manual section 8](operation_manual.md#8-post-move-review-two-items-only) and [round_plan step 6](../prompts/round_plan.md); it does not require a separate full report or a third wrap-up item. Before execution, prepare the game_id, our round number, the Chinese move notation, the context and the output path; do not compile the review during execution.

## ① Visual judgement from a new head image

After the mechanical motion ends, capture exactly one new cam_left_high image and record in this round's `tmp/visual_review.json`:

- The original image's absolute path, capture time and the game it belongs to.
- Whether the source is vacated, whether the target piece is near the correct intersection and to which side it is offset, and whether neighbouring pieces moved or overlap.
- `move_success` as true/false/null; state truthfully any uncertainty such as being unable to judge, manual repositioning, an opponent move or image latency.

State the service outcome and the game acceptance separately; placed cannot substitute for visual success, and a small FK residual cannot prove that the piece centre is on the square. Only the current final image is used for acceptance; do not fabricate results from old images.

## ② Offline FK, cumulative log and merged write

Use `scripts/review_pose_audit.py` to read the two raw execution-time snapshots in the real transaction results:

| Moment | Event stage | Computation |
| --- | --- | --- |
| grasp_success | source_contact_success | Actual FK relative to the original plan, the final effective target, and the commanded FK at that time |
| release_start | target_low_release_approval | Same as above; release uses the lowered low-position target |

Record ΔXYZ in millimetres, the position magnitude and the orientation rotation angle for each; for missing snapshots/failed computations write the reason and do not make up values. For a capture, compute both real transactions separately; the report for our own move is primary, the off-board removal is auxiliary, and they are merged into the same round. Old high-position release events are not mixed into the current low-position learning samples.

Following the [error closed-loop guide](xy_error_learning.md), use `scripts/xy_error_history.py finish` together with item ① and the plan summary to complete in one pass:

- Carry over all historical rows of previous_journal, then append one row to this round's `docs/execution_xy_errors.jsonl`, saving the complete errors of this round and the independent left/right-hand source/target accumulations.
- Generate this round's `docs/第N轮_中文着法_YYYYMMDD_HHMMSS.json`; the round number is our move count in this game, retries and capture sub-transactions do not increment it, the time is Beijing time at the merged write, and a file with the same name is not overwritten.
- Link the original image, the real results, the FK details and the JSONL path; pose_review.json is only intermediate detail and does not replace the final named review.

Accumulation uses only the 3D error of actual minus the final effective target; main-transaction samples with the same context, visual success, the two current events, three finite components and a magnitude ≤25mm may be learned. The same token is not learned twice; failed/unclear/off-board removal transactions keep evidence only. 25mm is a learning filter and is never a distance limit for motion corrections. The next round estimates XYZ independently from a new image; historical means are for reference only and are not automatically added to or subtracted from coordinates.

On a normal finish, close arm control and keep the waist hold; do not rebuild the board, search the engine, restart or release the service because of the review.
