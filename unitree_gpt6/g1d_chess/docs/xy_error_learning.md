# Ongoing Reference of Historical ΔXYZ Errors for Grasp and Release

English | [中文](xy_error_learning.zh.md)

In each round's review the errors are also saved to a continuously appended JSONL, accumulated separately for the physical left hand, the right hand, and the grasp/release stages; in the next round, when estimating source/target, the historical cumulative ΔXYZ mean of the selected hand is read. **The source and target XYZ are still estimated from the images captured this round; the historical mean is only an auxiliary reference: it is not added to or subtracted from coordinates directly, it does not automatically update the board XY table, and it does not automatically generate compensated coordinates.** What is referenced is the mean of historical displacement errors, not the mean of the actual XYZ coordinates of `grasp_success` or `release_start`. The final candidate is judged independently from this round's images, grasp/release orientations, grip position and neighboring pieces.

## Files and implementation

Commands are run in the g1d_chess root directory of the current deployment; on a new device, first set the interpreter and compatible context according to the [first game setup](first_game_setup.md).

- Cumulative journal: `<this-round-run>/docs/execution_xy_errors.jsonl` (this round's run), in the same directory as that round's final review JSON; the file name stays `execution_xy_errors.jsonl` and the content is the full 3D ΔXYZ. A new run carries over the previous cumulative journal via `--previous-journal`; finish first copies its historical lines verbatim, then appends one line for this round; the old journal is unchanged. Each round has exactly one line, containing this round's full errors and the updated `cumulative`. The new journal contains all carried-over history and can continue to be used across compatible games.
- Offline FK computation: `scripts/review_pose_audit.py` extracts the two events from the real transaction result and does not connect to hardware. The output `docs/pose_review.json` is intermediate detail.
- Appending, accumulation and final merge: `scripts/xy_error_history.py finish`, part of the second review item after completion, which also generates the final JSON named by round/move/timestamp. The JSONL is appended serially under a file lock and synced to disk; repeated runs with the same set of tokens and identical input do not count twice, and conflicting input raises an error. The same `game_id` and `round_number` cannot be counted again with a different token or context. A truncated JSON line at the end of the file raises an error and is not silently ignored.
- Planning reference: `scripts/xy_error_history.py show`; the left/right hand planners read the same journal again during build and save the selected physical hand's reference to `tmp/plan_summary.json.xy_error_reference`, without changing any XYZ. When eligible historical samples exist, the spec must fill in `xy_error_reference_review`, otherwise build is refused. The existing `xy_error_*` script and configuration field names are kept; their statistics are all 3D ΔXYZ.
- Configuration: `config/xy_error_learning.json.context_id`; the template is null and no default history group is provided. The current compatibility identifier must be filled in, or a spec/CLI context passed explicitly. When the board/body/waist pose, tool, model or grasp/release orientation group changes incompatibly, use a new context (change the configuration or the spec's `xy_error_context`); old data is kept, but statistics only merge the same context. A new game with compatible conditions does not need to clear the experience; statistics can span games, while rounds still start from 1 in a new game.

This package provides the [offline tool](../scripts/xy_error_history.py), the [configuration](../config/xy_error_learning.json), the planning reference integration for both hands, and offline tests. Run everything from the package root of this deployment; the journal is located by `--run` inside that round's docs, so do not pick history files from another game branch by mistake.

## Computation method

The 3D error of each eligible event is defined as:

```text
delta_xyz_mm = 1000 × (actual_fk.xyz − effective_target.xyz)
count := count + 1
sum_delta_xyz_mm := sum_delta_xyz_mm + delta_xyz_mm
mean_delta_xyz_mm := sum_delta_xyz_mm / count
```

In the base frame x points forward, y to the robot's left and z up; a positive sign means the actual end effector is offset in that direction relative to the final effective target; the left hand must not be mirrored. `cumulative.left.source/target` and `cumulative.right.source/target` each store `count`, `sum_delta_xyz_mm` and `mean_delta_xyz_mm`, with vector order `[ΔX, ΔY, ΔZ]`. **Describe the offset with the historical cumulative displacement error mean; sum is only a recomputable statistic, and neither the mean nor the cumulative sum may be automatically added to or subtracted from the next round's coordinates.** source only accumulates `grasp_success`, target only accumulates the low-position `release_start`; the two are not mixed, and samples are not borrowed from the other hand. When there are no samples, count=0 and mean=[0,0,0] mean no data, not zero error.

This round's three errors relative to the original plan, the final effective target and the commanded FK, as well as the actual/effective/command poses, are all saved in full in `main_report.records`; learning uses the full three components of `vs_effective_target.delta_xyz_mm`, does not average actual or effective coordinates, does not accumulate the source online visual correction amount, and does not use the same-frame actual pose at release approval, `placement_target`, as the error reference. The journal keeps the image judgment, actual result file paths, planning reference and this game/round/move; a missing snapshot still records this round's line, and a missing stage does not increase count.

Eligibility conditions: the main move transaction has `outcome=placed`, the final visual `move_success=true`, the events are `source_contact_success` and `target_low_release_approval` respectively, all three ΔXYZ components are finite and the 3D magnitude is ≤25mm. Failed, visually unconfirmable, old high-position release, missing-error and over-limit anomalous samples keep the original evidence and exclusion reason and do not enter the mean. Capture removal/off-board placement transactions go into `auxiliary_reports`; their errors are saved but not counted in the source/target accumulation. Only our own main move transaction can be learned, and one capture still appends only one line.

FK is an end-effector tracking estimate, not a piece center error; the grasp point, grip eccentricity, orientation change, release and gripper withdrawal may cause additional landing point offsets. The cumulative value cannot prove that the first-grasp success rate has improved; gates and final visual verification are still required. The reference provides the actual/effective target poses of the latest eligible event of each stage, only for traceability and checking comparability; they must not be used as coordinate means in place of the historical ΔXYZ mean. If not comparable, change the context and record it in this round's estimation notes.

## Reading during planning

After selecting the physical hand by source and before estimating source/target, the latest reference for that hand must be read, even if the rule move reuses the forecast. For example, the left hand:

```bash
"$CHESS_PYTHON" scripts/xy_error_history.py show \
  --run "$CHESS_RUN" --hand left --context-id "$CHESS_XY_CONTEXT" \
  --previous-journal "$CHESS_PREVIOUS_ERROR_JOURNAL" \
  --output "$CHESS_RUN/tmp/xy_error_reference.json"
```

For the right hand use `--hand right`. `CHESS_XY_CONTEXT` takes the context explicitly checked in the current configuration/this round's spec, and must not be copied from an old game. This read-only operation does not connect to the robot or append to the journal. With no journal the first time, it returns empty statistics; if the journal is corrupted it raises an error and must be repaired — do not just switch the context or disguise corruption as having no samples. Look at the direction/magnitude of `count` and `mean_delta_xyz_mm` for both stages, and estimate the source and target XYZ separately from this round's single planning head image, the executing hand's actual pose and the RPY/Z; board_position only gives the board-surface pixel geometry of the current image and provides no robot XY prior; `latest_eligible_event` only helps check conditions, does not replace the cumulative error mean, and the mechanical rules `source_xyz -= mean` or `target_xyz -= mean` are not used.

`CHESS_PREVIOUS_ERROR_JOURNAL` specifies the absolute path of the `docs/execution_xy_errors.jsonl` of the previous completed round; when there is truly no history the first time, omit the whole `--previous-journal` option. Do not ignore the previous round's history just because this round's directory has no journal yet. A specified history file that is missing or corrupted raises an error, and the accumulation is not silently cleared.

In the spec, fill in `game_id`, `round_number`, `move_plan`, `xy_error_context`, `xy_error_reference_review` and `xy_error_previous_journal` (the absolute path of the same previous cumulative journal as for show; null when there is no history the first time). `xy_error_reference_review` explains the direction/magnitude of the cumulative values, the comparability of the samples, how they are combined with the current vision, and the basis of the final candidate; even if you decide not to adjust the candidate, state the basis. build records the current journal path, the carried-over history path and the estimation notes; the full preflight still targets the final frozen candidate. A custom planner must explicitly perform the same reading and saving and must not bypass the reference step. Everything is completed before execute; the gates while holding a piece do not read or write the JSONL.

## Saving review item 2

First complete item 1, the new left_high visual judgment, and prepare the structured result in `tmp/visual_review.json`, containing at least `move_success` (true/false/null), the absolute path of the original image, the capture time, an offset description and the uncertainty. Compute FK offline from the real transaction result, then complete the JSONL append and final JSON merge with one finish:

```bash
"$CHESS_PYTHON" scripts/review_pose_audit.py \
  --result "$CHESS_RESULT" --output "$CHESS_RUN/docs/pose_review.json"

"$CHESS_PYTHON" scripts/xy_error_history.py finish \
  --run "$CHESS_RUN" --game-id "$CHESS_GAME_ID" \
  --round-number "$CHESS_ROUND_NUMBER" --move-plan "$CHESS_MOVE_PLAN" \
  --context-id "$CHESS_XY_CONTEXT" \
  --report "$CHESS_RUN/docs/pose_review.json" \
  --visual-review "$CHESS_RUN/tmp/visual_review.json" \
  --plan-summary "$CHESS_RUN/tmp/plan_summary.json"
```

`finish` reads the `previous_journal` saved during planning from `--plan-summary`, and when first writing this round's journal carries over all history and then appends this round; it does not need to be specified again in the finish command. An offline merge without a plan summary must explicitly pass `--previous-journal` to carry over history. The current journal and the final review are fixed in the same `run/docs/` and are not written back to other data directories.

Capture: compute one FK for each of the two real transactions; pass our own main move report first to `--report`, then repeat that parameter for the removal transaction report; do not pass the sequence wrapper result directly. If either of the two transactions is not completed, no whole-round success review is performed. `finish` outputs the final review path and the latest cumulative statistics and ends once saved; it does not capture images, call the service or search the engine. Retrying the same finish only restores the same named file and adds no cumulative samples; do not switch tokens or rounds to evade the duplicate check.

Standalone reviews continue to use `docs/第N轮_中文着法_YYYYMMDD_HHMMSS.json` (round N, Chinese move notation, timestamp); the JSONL path and that round's cumulative values are linked in the `xy_error_history` field. Both kinds of files belong to review item 2; no third wrap-up item is added. Historical errors are imported explicitly only after confirming the events, real results, visual success and context compatibility; batch guessing of old missing information is prohibited.

After a failure in the same game round, a retry is allowed: only when the existing record clearly shows visual failure, not placed and no learnable sample, may a new token append a record for the same round; the failure evidence and the original round number are kept. An existing success/unclear result or a token conflict still refuses duplicate bookkeeping.
