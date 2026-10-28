# Five-Round Forecast for Each Round's Decision

English | [中文](move_forecast.zh.md)

Moves that hit the forecast may cross the board and are not filtered by "the two outermost columns on the opposite side". Following [Move Selection and Robot Reachability](move_constraints.md), pre-check the preferred hand first; if it is unreachable, pre-check the other hand. Only when both hands are unreachable is the candidate discarded and a new search run.

Applies to the current flow with the robot as Red and the opponent as Black. After each necessary search, keep the selected principal variation, ordered from the current Red turn as `红1、黑1、红2、黑2、…、红5、黑5`, up to 10 plies. If the game ends early or the valid principal variation is shorter, save it at its actual length; do not fabricate moves to fill it. The engine prefers shorter positive mate lines; when no forced win is found, save the best available principal variation with its actual score, and do not claim a guaranteed fastest win.

## Planning order for each round

1. Capture the single planning left_high image for this round, use the previous round's review to confirm the square where Red's previous move actually landed, then identify Black's actual move and the current position.
2. Read this game's forecast and check forward from the confirmed progress: the previous Red move succeeded, Black's source/target and capture match the expectation, and the current piece layout, side to move and actual move history are also consistent.
3. If consistent and there is still a next Red move: after verifying it is legal with the rules library, use it as the candidate; do not re-search because it crosses the board, and do not top up the forecast every round. Estimate grasp and place coordinates from this round's new image, fully pre-check the preferred hand first, and if unreachable try the other hand; only when both hands are unreachable discard it and re-search.
4. If there is no forecast, Black's move does not match, the remaining forecast has no next Red move, or the current candidate is unreachable after full pre-checks with both hands: re-search from the current real position and save a new five-round forecast. A reset, a new game, our own piece landing on the wrong square, a corrupted forecast file, an illegal move or an inconsistent position all invalidate the old forecast. When the actual position is unclear, identify it first; do not infer the physical state from the old forecast.
5. Save this round's selection and forecast before execute. Sending a motion request does not mean the move is complete; in the next round's planning, confirm progress from the existing review and the new image. When an old client times out or the same round is re-planned, also judge by the real result; never consume the forecast twice or move a piece twice.

After the move and the source acting hand are selected, and before estimating source/target, follow [Historical ΔXYZ Error Reference](xy_error_learning.md) to read, from this round's `<this-round-run>/docs/execution_xy_errors.jsonl`, the historical cumulative ΔXYZ mean of that physical hand under the current compatible context. A forecast hit does not skip the error reference; source and target each look at the historical displacement-error mean for grasp / low release respectively. XYZ is still estimated from this round's images; the mean is only an auxiliary reference: do not average actual grasp/place coordinates, do not add the error directly, and do not change the coordinate table. The finish in review item 2 is responsible for appending this round's error and the left/right-hand cumulative values; the move pending execution is not counted in advance during planning.

For example, with `红1→黑1→红2→黑2…` saved: if Red 1 actually succeeded and Black actually played Black 1, play Red 2 directly; if Black played another move, search from the new position. When only Red 5 remains, it can be played directly; re-search only when there is no remaining move at the next Red turn. If the current position is already terminal, stop moving; do not keep searching for actions because the forecast is exhausted.

## Storage location and contents

The previous round's final review is at its run's `docs/{第几轮}_{这一轮的走棋子方案}_{时间戳}.json`; following [Operation Manual Section 8](operation_manual.md#8-post-move-review-two-items-only), check the game ownership, round, move and real result in its content. The round is our move count in this game and is not reset when the forecast is re-searched.

Within the same game, use `runs/<this-game-directory>/move_forecast.json` under this deployment's package root as the current forecast entry. A new game uses a new directory, and ownership is confirmed from the game identifier and the current actual position. Each planning pass keeps a snapshot in this round's `tmp/move_forecast.json` so the next round can locate the source; both are saved during planning. Different runs share the same game entry; no new index maintenance is added to the post-execution review.

A forecast record contains at least:

- `game_id`, generation time, source run, engine parameters / raw score and log path.
- `base_fen` and the actual move history before the search start; replay step by step according to the rules, not just compare the two source/target squares.
- `rounds`: up to 5 groups, saving Red and Black moves separately; each step records the side to move, engine UCI, manual source/target, capture info, `fen_before`, `fen_after`. When the principal variation ends early, the missing reply is null, with an explanation of game end or insufficient principal variation.
- `confirmed_ply_count`: counts only plies already confirmed by real observation / the previous round's review; the forecast position and the physical position are stored separately.
- `selected_move` and `selected_ply_index`: the Red move pending execution this round, marked as pending confirmation; `decision_source` is `forecast_match` or `engine_search`, and the reason is recorded when re-searching.

When comparing FEN, construct the counter information from the rules history; images only provide the visible layout, and counters that are invisible in images are not treated as visual measurements. If the cache cannot be matched consistently with the real position, discard the forecast and re-search from the confirmed position.

## Boundary with execution and review

What is reused is the rule-level move, not old arm coordinates, trajectories or tokens. Each round still selects the hand only by source, and uses the currently measured start state, current images, independent source/target estimates and a full pre-check. If the current move cannot be executed, do not bypass checks on the grounds of a forecast hit; when a different move must be chosen, invalidate the old forecast and search during planning.

**During execute, only handle the current checkpoint: recapture immediately, decide immediately, submit immediately; do not call the engine or maintain the forecast.** After the move ends, still only judge success/offset from a new left_high image and compute and save the merged FK error. Forecast progress confirmation is deferred to the next round's planning; no third review item is added.

Before searching, `scripts/move_constraints.py` can be used to generate `searchmoves` from all rule-legal moves and to verify the returned bestmove. The current script does not filter candidates by cross-board distance, and future principal variations are not truncated by it; the opponent's replies are kept as-is according to the rules. When there are no legal moves, do not send an unconstrained search command. Actual reachability before moving is determined by this round's full pre-check of both hands. Forecast cache reads and writes are done by the main session during planning; there is currently no general CLI that maintains the cache automatically.
