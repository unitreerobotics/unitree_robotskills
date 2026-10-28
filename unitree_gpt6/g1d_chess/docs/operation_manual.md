# Robot Chinese Chess (Xiangqi) Single-Round Operation Manual

English | [中文](operation_manual.zh.md)

**Grasp gate termination semantics:** Use `retry_high` when the grasp should continue and be re-captured; when the user asks to terminate or it is not safe to continue, submit directly at the current pre-grasp checkpoint with `serial_checkpoint.py submit --prepared <prepared.json> --checkpoint <checkpoint.json> --failure`, without a visual evaluation file. The service goes straight along the already-cached return-to-init path and does not start another re-capture. At the low position it first opens the gripper and rises vertically, then returns home continuously. Fault protection can still stop the motion; the actual state must be verified, and an accepted `failure` must not be treated as having returned to init. An old service process does not load fixes automatically; after rebuilding the service a new token must be used.

**Grasp retry flow:** When grasp conditions are not met or the grasp is empty, first return to the high position, capture the head image + actor wrist image, correct XY and re-capture, then descend; at the low position, never use lowering Z to solve XY. At most three high-position XY corrections: re-capture after the first two; after the third is completed, descent and grasp are released directly, with no further high-position re-capture or visual approval; the existing low-position close-gripper gate after descent still applies, and only if the grasp is still empty does it return to init — ordinary insufficient centering is not directly judged as a whole-round failure. The grasp RPY should reduce obvious interference with neighboring pieces; a perfect zero-approach is not required. This item takes precedence over the older failure-retreat descriptions below; for the full entry point and the six-field `retry_high` usage see the [round prompt](../prompts/round_plan.md). The in-package code has been updated with the new transaction policy; an existing service process does not load on-disk changes automatically, so before the next execution load the new version in the idle state and generate a new token; never reuse an old token.

**Neighboring pieces and landing point:** New games follow [Neighbor clearance and accurate landing](neighbor_clearance_and_landing.md). Choose RPY separately for source and target, and check the neighbor sweep of the whole gripper jaw during descent, closing, opening and retreat; a grasp must not be confirmed only by the two fingers being centered. On release, also confirm that the expected landing point is near the target intersection, that the whole piece can drop on its own, and that it does not overlap neighboring pieces. Do not pass on the basis of a historical "slightly off-center", the service reporting placed, or a small FK residual. The rules have been written into the prompt; the improvement still needs to be verified on the real robot in new games.

**Current move-selection and pose rules:** Either hand may reach across the board. Pick the preferred hand by the source file (a–d left, e–i right); for the best move by chess rules, first run a full transaction preflight with that hand's current measured start point and the grasp/place poses estimated from this round's images; if unreachable, switch to the other hand, re-estimate, build and preflight. Only if both hands are unreachable is the move abandoned and the next legal candidate searched. In both halves of the board, source and target first consider a more lateral gripper pose; if lateral is unreachable for a stage, or there is a clear risk of pushing or pinching a neighboring piece that cannot be eliminated among lateral candidates, that stage tries a more longitudinal pose instead, re-estimates EE XYZ for the new RPY, and re-checks neighbors and the full path. Single-point IK, historical coordinates or old predictions cannot substitute for a current full-path preflight. For execution details see the [round prompt](../prompts/round_plan.md) and [Move selection and robot reachability](move_constraints.md).

Applicable robot: G1 with the built-in Dex1 gripper. This document is the unified entry point for a single chess-move round; execution by the physical left or right hand is supported, and the other hand keeps the current pose confirmed for this round. All current entry points are inside the g1d_chess package of this deployment; for known differences between the rules and the runtime implementation see [Runtime notes](runtime.md#implementation-verification-boundary). Reorganizing the documentation is not real-robot validation.

From now on you can provide the assistant directly with:

Each round you can directly attach the condensed prompt: [prompts/round_plan.md](../prompts/round_plan.md), which contains the full plan and the constraint during execution to "re-capture immediately, decide immediately, do no unrelated steps".

**Historical ΔXYZ error reference loop:** Item 2 of each round's review additionally appends one line to this round's `<this-round-run>/docs/execution_xy_errors.jsonl`, saving this round's errors for the two events and the independent left/right-hand cumulative statistics; the next round, after choosing the hand by source and before estimating source/target, reads the historical cumulative ΔXYZ mean for the corresponding hand. The XYZ of source and target is still estimated from this round's images; the historical mean is only an auxiliary reference — it is not directly added to or subtracted from coordinates, the actual grasp/release XYZ are not averaged, and the coordinate table is not changed automatically. For implementation and commands see [Historical ΔXYZ error reference](xy_error_learning.md), 0.4.1 and Section 8.

> Read `<deployed-g1d_chess-root>/SKILL.md`, and complete the next round of Chinese chess according to this document and the current user instructions. First establish or reuse the service per 0.1.3 and verify the waist is locked at zero; keep the waist hold at the end of a normal move. Each round, first check the distance from each arm's measured position to its own init; if both are ≤10cm, do not re-init — plan directly from the current pose, then capture only cam_left_high to confirm the current position and read the robot state; first match this game's five-round forecast per 0.2 — only if Black's reply and the position agree and the move is legal is it used as a candidate; with no forecast, a mismatch, or the forecast used up, search again. Cross-side edge moves such as right hand to a–b and left hand to h–i are allowed; prefer choosing the hand by source, switch to the other hand if the full preflight is unreachable, and abandon the candidate only if both hands are unreachable. If execution has been authorized for this round, execute after full planning and preflight pass. After a successful grasp, carry the piece to the target high position and descend to the release height, then re-capture before opening the gripper; if the actor wrist image shows the expected landing point near the target intersection, release; if the expected landing is between adjacent intersections, in the middle of a square, or clearly offset from the target intersection, keep the actual Z/RPY and make a direct low-position XY fine adjustment (no lifting, no per-step or cumulative distance limit, no motion check on the correction segment); at most three times — re-capture after the first two, open the gripper directly after the third. Then the actor arm retreats upward continuously and returns to its own init pose; the non-actor arm keeps the current pose confirmed for this round throughout, both arms wait for the next round, and the camera pose is not used. After the move is complete, do only two review items: capture one new left_high to judge whether the move succeeded and any position deviation, then compute the three FK errors of the two event snapshots, merge them with the image judgment and write them to this round's `docs/{第几轮}_{这一轮的走棋子方案}_{时间戳}.json`; naming details are in Section 8.

Historical sessions and parameters are not new motion authorization. Round-specific limits such as "plan only", "evaluate only, do not correct" or "release the service when done" take precedence; if execution authorization for this round already exists, do not ask again. When you are only asked to read this manual or organize material, do not run the hardware commands below.

## 0. Quick handover: read this section first

**Main line of each round: read both arms' current poses; within ≤10cm of their own init no return home is needed → check the position with a single new left_high image → match the five-round forecast; search again only if it is invalid or used up → choose the preferred hand by source and estimate source/target independently; cross-side edges are allowed → full branch preflight with the preferred hand; if unreachable, try the other hand → one execute → two high-position images / re-capture after any necessary correction → two low-position images → close the gripper in place and verify the grasp → after target descent, re-capture; if the wrist image judges a clear offset from the target intersection, make a direct low-position XY fine adjustment → release in place, actor arm returns to its own init → both arms stand by → judge success/offset from a single new left_high image → compute the FK errors offline, merge with the image conclusion and write to disk.** All material, computations, explanations and scripts needed before the grasp are prepared in advance; do not study old plans while holding a piece or inside a visual gate. This section overrides outdated descriptions in older documents such as "fixed right hand", "longitudinal forbidden at the far end", "loaded moves must pass through staging", "close the gripper immediately after release" and "the observation arm moves to the camera pose"; current user instructions still take precedence.

### 0.1 Independent working root and entry points

The working root of this package is determined by its actual installation location; command examples use `CHESS_ROOT` for it, and the root's scripts, skills, config, templates and prompts are used directly. Operating rules follow this package's operation manual and round_plan, and the service and planning code also use this package's files; there is no dependency on other project directories or user-level skills. Each independent tool call sets this package's workdir and absolute paths.

| Content | Entry point in this package |
| --- | --- |
| Execution prompt | [prompts/round_plan.md](../prompts/round_plan.md) |
| Full script inventory | [docs/round_script_inventory.md](round_script_inventory.md) |
| Runtime environment and migration notes | [docs/package_setup.md](package_setup.md) |
| Right/left planners | [round_tools.py](../scripts/round_tools.py), [round_tools_left.py](../scripts/round_tools_left.py) |
| Service startup and requests | [start_service.sh](../scripts/start_service.sh), [robot_client.py](../scripts/robot_client.py) |
| Python interpreter | The current environment in which the user has installed the dependencies; use `python` directly, or specify the interpreter's absolute path explicitly |
| Integrity check | `python scripts/verify_package.py` |

Verify that the service ROOT matches the current repository; do not reuse tokens or run directories across services. A running process does not update along with files on disk.

### 0.1.1 Select cameras by stage

All scripts, internal service dependencies, on-demand tools and the calling order needed for a generic round are listed in the [Single-round script inventory](round_script_inventory.md). All scripts use the package root of this deployment specified in 0.1; for the division of labor among documents and topic mapping see the [docs index](README.md).

**Deciding the move uses only one `cam_left_high` image; grasp and place judgments during execution use only two images: the head `cam_left_high` + the physical actor hand's wrist.** The same rule applies at source high position, before closing at the low position, after a grasp XY correction, and before opening after target descent; the observation hand's wrist is not captured, and having all three streams is no longer a gate prerequisite. Move planning still needs a robot state from the same batch, but does not require wrist images. The post-move review captures only one new left_high to judge success and offset, then computes the FK errors offline per Section 8 and merges them to disk.

| Scenario | Streams actually requested by the current configuration |
| --- | --- |
| Move planning / default board observation | `cam_left_high` |
| Grasp/place executed by the physical left hand | `cam_left_high` + the physical left wrist stream confirmed by this configuration |
| Grasp/place executed by the physical right hand | `cam_left_high` + the physical right wrist stream confirmed by this configuration |

The mapping between wrist streams and physical hands must be checked against the live image; do not guess from stream names or the logical `right_arm`. Fill in the left/right wrist mapping and the basis for it in [config/chess_camera_policy.json](../config/chess_camera_policy.json); the template mapping is empty, and no camera selection is generated while it is unfilled. The reading entry is [camera_policy.py](../src/g1d_chess/robot/camera_policy.py), cached per process. First complete the check per [First game setup](first_game_setup.md); the new service preflight binds the mapping to the new token, the checkpoint returns `physical_actor/camera_order/camera_selection`, and images are read from this run's paths. An existing process is not updated automatically by changes to the configuration on disk.

The default requests of the service `capture_cameras` and of this package's standalone capture script are left_high; grasp/place gates explicitly request the two selected streams. The planner rejects planning observations mixed with wrist images, and the execution entry rejects preflight camera policies that do not match. The running process must load the matching configuration of this package and generate a new token; historical three-stream observations are not rewritten. There is no need to move the observation hand or re-fit the board XY table for this change.

### 0.1.2 init standby and the 10cm no-return-home rule at the start of each round

**At the start of each round, compute the 3D distance from each arm's current end-effector position to its own `init pose`; an arm whose distance is ≤0.10m (including exactly 10cm) is not re-initialized.** When both arms are within this range, select the actor hand directly by source and use the current measured joints and full XYZ/RPY as the new plan's start point; the non-actor arm keeps the current pose confirmed for this round. It is not 10cm allowed separately on each coordinate axis; do not treat the init configuration value as the actual start point, and do not move or rotate the wrist first to satisfy the no-return-home condition. This threshold only judges position; the current pose and the whole path still go through the original preflight.

Neither arm uses the camera pose. After placing, the actor arm still returns to its own init along the preflighted tail segment, and the non-actor arm keeps this round's start pose and waits for the next round; the 10cm rule only exempts the repeated return home before the next round starts, and does not omit this round's post-placement return segment. The two sub-transactions of a capture are still chained continuously, and the arm returns to init after the whole move is complete.

init parameters are read separately per **physical hand** from the currently applicable configuration; do not set the left and right hands to the same values, and do not directly reuse historical dual-arm files that contain the other arm's camera pose. The older term "observation arm" in the text only denotes the service role of the non-actor arm; it is not required to move to a capture pose; grasp/place judgments still use the head camera and the actor arm's wrist camera.

Service ready only means the process is ready. When establishing or reusing the service, compare the measured FK XYZ with the corresponding physical hand's init XYZ in the same base frame and with the same end-effector definition: `distance_m = sqrt(dx² + dy² + dz²)`. Only an arm exceeding 0.10m needs initialization planned and preflighted via the authorized flow; the other arm within range does not repeat the return home along with it. After initialization, generate the plan from the new actual state. If the init configuration or state is missing, do not assume the threshold is satisfied; recover abnormal piece holding or faults per Section 9.

When generating the plan, verify that the actual output satisfies the current pose rules; for runtime implementation differences see runtime; do not assume that the operation manual automatically changes process behavior.

### 0.1.3 Establish the service and lock the waist

This game flow establishes the service and locks the waist at zero as explicitly authorized by the user. First, in an environment where the real processes are visible, verify a single instance, the working ROOT, the actor-hand entry, and `ping`, `get_state`; a healthy idle service with the correct ROOT is preferably reused and not restarted just to establish a connection. If the current request is only to establish the service and lock the waist, after the checks in this section wait for commands, and do not automatically execute a move or dual-arm initialization.

The waist hold is independent of dual-arm control. If the real-time state already satisfies `waist_hold_enabled=true`, `waist_zero_verified=true`, `waist_target=[0,0,0]` with no fault, save the actual `waist_q` and keep it; do not re-zero. Otherwise, only when the service is idle, dual-arm control is off, both grippers are empty and closed, and there is no hardware fault, call `lock_waist_zero`, save the receipt and re-read the state to verify the fields above; do not claim the waist is locked at zero unless verified. Locking the waist invalidates existing cached plans; it must be completed before building this round's plan and full preflight, and after any waist position change, refresh the observation, board registration and source/target estimates.

After an ordinary return to init and `release_control`, keep the waist hold; do not release or re-zero it every round. Only when the user explicitly asks to release the waist hold or end the robot service, confirm the piece has been placed, arm control is off and the site allows release, then call `release_waist`; for the specific order of ending the service see 8.1.

### 0.2 The engine forecasts five rounds ahead; reuse only if it matches and satisfies mechanical constraints

1. **Confirm the actual position first, then match the forecast.** Each round, capture only a new left_high, check it against the previous round's final review image, and identify Black's actual source→target, any capture, and the current full position. Do not use the forecast to guess Black's move. Read this game's `move_forecast.json`, and against the confirmed progress check the previous Red move, Black's actual reply, the current position and the side to move; only if they agree and there is still a next legal Red move is it used as a candidate — **do not search ahead just to fill up five rounds**. Estimate the arm's source/target independently from this round's new image and run a full preflight; if the preferred hand is unreachable, try the other hand; only if both hands are unreachable is the candidate discarded and a search run.
2. **Search again when there is no forecast, a mismatch, the forecast is used up, or the candidate is unreachable.** When there is no usable forecast, Black's actual reply differs from the forecast, or the forecast has no next Red move left, call the chess engine on the currently confirmed position, look for the move that wins fastest, and save a new five-round forecast. If the forecast move is unreachable in full preflight for both hands, also re-search from the current position; a board reset, our own piece landing in the wrong square, or a damaged cache / one not belonging to this game also count as forecast invalidation; when the position is still unclear, identify it first — do not guess and then search. Each save is `红1、黑1、…、红5、黑5` starting from the current Red turn — **five rounds is at most 10 half-moves, not 5 single-side moves in total**. If the game has already ended early or the engine's valid principal variation has fewer than 10 half-moves, keep only the part actually returned and verified; do not invent opponent moves; search again next time there is no remaining Red move.
3. **The engine targets the fastest confirmable winning line.** When in check, first filter legal check responses; when there is a forced mate, prefer the shorter positive mate, do not treat a negative mate as a win, and do not treat search depth as remaining rounds. Moves with the right hand to a–b and the left hand to h–i also take part in comparing winning lines; once selected, the full preflight decides whether it can be executed; when there is no forced mate, choose the move with better position score and principal-variation stability, and compare mechanical cost only when the winning chances are comparable. The principal variation is an engine prediction; it does not promise the opponent will follow it or that it is the globally fastest forced win. A matched forecast does not trigger an extra search because of re-ranking each round.
4. **Search and saving happen only in the planning phase.** Use the Fairy-Stockfish specified by the environment variable `CHESS_ENGINE`, set `UCI_Variant=xiangqi`, `Threads=2`, `Hash=128`, `MultiPV=3`; search with the full set of legal moves, or use `scripts/move_constraints.py` to generate a `searchmoves` command that contains all legal candidates. Do not search when there are no legal moves; complex positions may be searched longer, and a stable mate1 does not need repeated deep search. The engine and the installed `pyffish` must first be confirmed usable per the environment installation notes. Save the selected principal variation, the engine's raw score/log, the expected position step by step and the progress; during execution do not search or maintain the forecast, and the post-move review still has only two items. In the next round's planning, confirm the moves played from the previous round's review and the new image before advancing the forecast progress; do not mark the move as successful in advance when sending execute.
5. **Verify coordinates and legality step by step.** The FEN is written from the far rank9 to the near rank0; the engine's UCI ranks are 1–10 and the manual's are 0–9, e.g. `b7c9` corresponds to `b6→c8` on this board. Verify the principal variation with `pyffish.legal_moves/get_fen`, and keep this game's actual move history for repetition-rule judgments; these rule checks are not engine search. The forecast does not replace current vision, confirmation of the actual landing square, or robot preflight.

For the forecast file and the matching order see [Five-round forecast usage](move_forecast.md). Use the UCI interface of the engine specified by `CHESS_ENGINE`, input the currently confirmed position with the options above, and verify the principal variation step by step with the installed `pyffish`. Neither the move-selection script nor this package's planners reject moves by how far they cross the board; reading and writing the forecast cache is handled by the main session by operating convention.

### 0.3 How to choose the left or right hand, and write a hand switch into the same plan

**Current rule: either hand may reach across the whole board, including the right hand to a–b and the left hand to h–i.** With source in files a–d prefer the physical left hand; with source in files e–i prefer the physical right hand (file e belongs to the right hand). First estimate the grasp/place poses independently from the preferred hand's measured start point for this round and run a full preflight; if unreachable, switch to the other hand, re-estimate and run a full preflight; only if both hands are unreachable is the move abandoned. The target's file does not directly decide the actor hand. Once chosen, the same hand completes that grasp, carrying, release and return to init; the non-actor arm keeps the current pose confirmed for this round, and no handover at the center line is needed. For example g4→d4 prefers the right hand and d8→h8 prefers the left hand; a cross-side target must still have its XY/pose estimated independently for that actor hand and pass the full-path preflight. The historical right-hand g4→d4 can serve as a cross-side placement reference, but its old camera-pose prefix is not reused. The full joint sequence of a capture must also be preflighted; the service must not be switched on the fly while holding a piece. Forecasts and new searches use the same reachability rules; see [Move selection and robot reachability](move_constraints.md).

| Service mode | Plan/state `right_arm` | Plan/state `left_arm` | Requirements |
| --- | --- | --- | --- |
| Default right-hand service | Physical right actor arm | Physical left non-actor arm (keeps this round's current pose) | Use the right-hand planner, a new observation and token from the same service |
| Left-hand service, `active_arm=left`, `arm_field_frame=transaction_roles` | **Physical left actor arm** | **Physical right non-actor arm (keeps this round's current pose)** | Use the left-hand planner; motion/preflight/enable requests contain `expected_active_arm=left`, and the execute request is generated by the left-hand tokens entry |

Left- and right-hand XYZ both use the same base frame: x forward, y toward the robot's left, z up — **do not mirror XYZ or swap cameras by stream name**. The left-hand adaptation handles joints, model, grippers 31/33 and feedback at the same time; do not swap again in an outer layer. In these rounds `cam_right_wrist` moves with the physical left hand; still check against the image. Real waist/torso changes affect the relation between base and board; verify the waist hold via the authorized flow in 0.1.3, re-observe and re-register after a waist position change, and do not drive the waist on the fly during a move.

When switching hands, the service role may only be switched after an orderly shutdown of the old service while it is idle, holding nothing, with control off. Each round, check both arms' distances to their own init per 0.1.2; ≤10cm means no repeated return home; in the next round the actor arm starts from its current measured pose and, within the same move transaction, rises to clearance, rotates to the grasp pose and goes to source, while the non-actor arm keeps the current pose confirmed for this round. No camera-pose transition is inserted any more, and there is no repeated return home or extra capture/wait for switching hands. If, when taking over an old flow, an arm is still in a non-init pose such as camera, also use the distance rule in 0.1.2 to decide whether initialization is needed; do not pass off the expected init as the actual start point. Check the pose and the neighbor envelope throughout; do not look only at XY.

When generating the plan, check the equal-length hold points of both arms: across all success/failure/retry paths, the non-actor arm keeps the current pose confirmed for this round, and the actor arm's normal end and its planned empty-gripper failure retreat end at its own init. After clearing the historical camera transition, redo the whole preflight; after modifying the request recompute the local digest — the token's request-consistency check must not be bypassed. Abnormal piece holding/faults must not be forced back to init because of this item.

### 0.4 Near-end and far-end grasp/place: reusable candidates and clear boundaries

**Current pose rule (overrides the 2026-10-10 suggestion of choosing the initial orientation by board half):** source and target, in both halves, each first consider a more lateral pose, then adjust RPY according to their own reachability and the risk of obvious neighbor interference; a lateral grasp being feasible does not require the release to stay lateral — if necessary the target may be more longitudinal or diagonal. Judge the grasp and release poses independently, taking into account neighbors on both sides, the open/close gripper sweep, and the space for fingertips and finger bodies; do not directly judge a 2D occlusion or projected proximity during grasp as a collision. Choose each stage's pose first, then estimate XYZ separately using the tool-to-end-effector offset; after changing the release RPY, re-estimate the target EE XYZ — the old EE coordinates must not be reused, and historical coordinates apply only to their pose_context. Perform pose rotation at a clearance height, re-build and fully preflight the loaded rotation, carrying, release, retreat and return to init; a piece lying within the opening's projection at the high position does not mean the fingertip side walls can already grasp it. For the first-time empty grasp and knocked-piece cases of g1 next to Red elephant g0, and d1 next to advisor d0 / Black chariot d2, see 20261010_round1; they must not be replayed directly.

"Red near end / Black far end" refers to the spatial halves of this board — near end rank0–4, far end rank5–9 — and does not mean the pose is decided by piece color. Lateral/longitudinal refers to the in-plane orientation of the gripper's forward direction on the board surface; both can still keep pitch 45° tilted downward — this is not a roll flip or a pitch 90° vertical plunge. All of the following are **model EE commands**, not the actual height of the fingertips or the piece above the board surface.

| Scenario | Grasp candidate | Place candidate | Real-robot basis and usage boundary |
| --- | --- | --- | --- |
| Near end, physical right hand | roll0/pitch45°/yaw about +80.68° (1.408187rad), more lateral; source Z=.105m candidate | Carrying with the same pose once used Z=.105m/q2.3; new rounds are filled in per the latest release-height requirement and do not automatically fall back to old values | e3→e4 succeeded on the first grasp; g3→g4 once needed .105→.100→.095. The right hand's successful low-position actual FK Z was about .122m; .105 must not be taken as the physical clearance height. |
| Far end, physical left hand | Prefer diagonal/more lateral; the last two rounds used yaw−48°, pitch45°, source Z=.109m | Rotate at height to more longitudinal yaw0, pitch45°; the most recently confirmed release Z=.139m/q2.3 | b7→b9 and b6→c8 were completed on the real robot, both with a small landing deviation; other squares still need independent preflight and final verification. −48° is a locally successful pose, not a universal left-hand yaw. |
| Near/far ends and cross-side extrapolation with the same actor hand | Generate candidates from successful samples of the actor hand with the same pose and the current image | The far end may use a more longitudinal pose to improve reach; the transition must be done at clearance and the tool offset re-estimated | Cross-side placement does not switch hands; do not mirror right-hand XY/yaw into left-hand parameters; when samples for the corresponding pose are missing, clearly mark it as a candidate. |

In the last two rounds gate=.170m, travel=.210m, and historical staging was right hand [.22,−.21] and left hand [.22,+.21]m; these are also candidates, and the current whole-hand clearance is checked against the scene and feedback. The new board is recorded as file spacing 35.6mm, rank spacing 33.7mm, board thickness 20mm, piece diameter 29.7mm/height 9.9mm; do not overwrite the new board with the original package's 35mm uniform spacing, 15mm board thickness and 12mm piece height.

The grasp standard is **graspable is enough**; the piece does not need to be fully or pixel-perfectly centered. At the high position check both left-right and front-back: the piece is between the two jaws' opposing textured surfaces, and within the graspable segment of the front effective texture, not retreated behind the gripper; descend when both directions suffice to grip the side walls with no obvious risk of pushing or pinching a neighboring piece. If not, combine this run's head image and actor wrist image to estimate a definite XY once, aiming to succeed in a single adjustment; source at most 3 times — re-capture after the first two; after the third is completed, descent and grasp are released directly, and the existing low-position close-gripper gate is kept.

At the low position, verify that both side walls enter the opposing effective textured surfaces, the fingertips can wrap the side walls, and there is no pressing on the top, pushing, or pinching of neighboring pieces; if satisfied, close the gripper. If more than half of the piece lies outside the textured contact area but XY is already graspable and only the height leaves insufficient effective contact area, consider `adjust_z` to automatically lower vertically by 5mm; re-capture after the first, and only if the height is still insufficient and the preflight allows, lower a second time and close directly. If a left-right/front-back offset keeps the side walls from entering the effective grip surfaces on both sides — especially a clear offset backward — keep the gripper open and use `retry_high` to return to the high position and re-capture; lowering Z must not substitute for an XY correction. A real empty grasp also returns to the high position; ordinary insufficient centering does not directly return to init as failure. Real collisions, a flipped piece or control faults follow the original fault branch; a knocked-over piece must not be treated as a successful grasp.

Placement uses the latest order: **open in place → keep open, rise vertically from the actual XY/RPY to clearance → retreat to staging with the gripper open → close in place → return to init**. Leave the piece before closing, so the fingers do not sweep and drag the placed piece again; do not reuse the once-tried "close immediately after releasing, then lift". After a successful source grasp, rising in place to clearance is enough to go directly above the target; at height, rotate pose / use a connection point if needed — the loaded move is not forced to detour through staging. Release is at a disengagement height close to the board surface, not dropped directly from travel height; .139m is the last confirmed EE candidate and cannot be claimed to equal the piece being 1cm or 2cm above the board.

**Locate source and target separately.** Source is aimed at the current physical piece and the effective grip area; target is aimed at the board-surface intersection and its neighbors. Prefer successful grasp samples and face-up actual release samples with the same hand, same RPY/Z and same board/torso conditions; estimate new candidates using the board's local direction mapping, and record the uncertainty. There is parallax between the piece's top surface and the board surface; this round's +5px is only a local estimate and must not be applied across the whole board. Do not derive a precise target across yaw from an uncalibrated "tool length .19m", do not take the landing point after a flip-and-bounce as a stable calibration, and do not add the full online source correction to the target.

When explaining pose-rotation errors, `p_piece = p_EE + R*d` can be used: even if the EE is in position, changing R or the grip eccentricity d moves the piece center. When d is uncalibrated, end-effector FK alone cannot guarantee an accurate landing square. The two samples available for checking are below; **they must not be copied as execution commands for other rounds**:

| Sample | Source requested XYZ m / yaw | Target requested XYZ m / yaw | Result |
| --- | --- | --- | --- |
| Oct 8 right hand e3→e4 | [.266446,−.094318,.105] / +80.68° | [.288554,−.095434,.105] / +80.68° | 1 grasp / 0 online corrections, near e4; post-release FK [.294241,−.087217,.122893]. |
| Oct 8 left hand b6→c8 | [.291926,.199529,.109] / −48°, then a high-position correction relative to actual FK of [+10,−5]mm | [.350247,.096347,.139] / 0°, target did not stack the source correction | 1 grasp / 1 correction / 0 Z lowering, face up covering c8, slightly to the left; post-release FK [.352485,.094825,.145476]. |

### 0.4.1 Independent cumulative ΔXYZ errors per hand, as a reference for the next round's estimates

**Directory rule:** `execution_xy_errors.jsonl` is always placed in `<this-round-run>/docs/`, in the same directory as this round's final review JSON. When planning a new run, add `--previous-journal <previous-round-run>/docs/execution_xy_errors.jsonl` to the show command below, and save the same absolute path in the spec's `xy_error_previous_journal`. Only when there is genuinely no history for the first time, omit this argument and fill in null. finish automatically obtains the previous log from the plan summary, carries over all its historical lines and appends this round; the old log is kept and the new log becomes the next round's entry, so switching runs does not lose the left/right-hand cumulative statistics. Planning reads are read-only; this round's log is not created in advance.

In the latest working root, use `scripts/xy_error_history.py show --run <this-round-run> --hand left|right --context-id <current-compatible-config>` to read this round's `<this-round-run>/docs/execution_xy_errors.jsonl` (if not yet created, only read the previous round's log specified by `--previous-journal`), and save this round's `tmp/xy_error_reference.json` before estimating source/target. **The user has made it explicit: XYZ is still estimated from this round's images; the historical cumulative ΔXYZ mean is only an auxiliary reference, and programs must not add or subtract the mean or cumulative sum directly to/from source/target.** For source look at this physical hand's historical displacement error mean for `grasp_success`; for target look at the same hand's historical displacement error mean for low-position `release_start`; do not look at means of actual XYZ coordinates, do not mix hands, do not mix stages; with no samples, state explicitly that there is no data. This step is still required when the rule-based forecast hits.

The cumulative measure is `count`, `sum_delta_xyz_mm` and `mean_delta_xyz_mm` of `1000 × (actual_fk.xyz − effective_target.xyz)` (millimeters), with vector order `[ΔX, ΔY, ΔZ]`; the mean is a historical deviation reference, and the sum is not a compensation amount. The log fully keeps each round's three FK errors, actual/commanded/target poses, image judgment and source paths. Only visually successful main move transactions, current event stages, and finite samples with a 3D magnitude ≤25mm are learned; old high-position releases, failures, missing data and off-board removal transactions keep their evidence but are not counted. Each stage is counted independently. The configuration context limits comparability by board, torso/waist, tool/model and pose group; when it changes, old statistics are isolated, and data is not borrowed from the other hand.

The final XYZ candidates for source and target are still chosen independently based on the current image, the same hand's pose and the neighbors. In the spec, write `xy_error_context`, `xy_error_reference_review`, `xy_error_previous_journal` as well as this game's `game_id`, `round_number`, `move_plan`; the estimation notes record how the historical ΔXYZ mean was combined with the current vision, and also give the reason when deciding to keep a candidate. The left/right-hand planners' build automatically reads and saves `plan_summary.xy_error_reference`, refuses to build when there are samples but no estimation notes, and **does not change the spec's XYZ**. The `xy_error_*` field names are kept for compatibility with existing entry points; their contents are 3D errors. A custom build must follow the same convention. All references and scripts are prepared before execute; gates do not read or write the history. For the complete description, JSONL structure and commands see [docs/xy_error_learning.md](xy_error_learning.md). The offline reference does not guarantee that the grasp success rate has improved; real-robot feedback and the final vision still decide.

### 0.5 Interpolation to resolve joint jumps while reducing landing drift

First locate which arm, which segment and which constraint failed. Joint jumps, IK endpoint residuals, real hardware faults and scene-envelope collisions are not the same problem. In the last round, the first version's 0.350rad jump occurred at **the end of the initial vertical retreat**, not at the far-end release; blindly tuning the target cannot fix it. First fix the correct source/target, then handle the pose and the high-position connection path that reach them.

The `MotionHarness.prepare` method already implemented in the in-package code:

1. Each behavior waypoint keeps a fixed absolute Cartesian endpoint; position is linearly interpolated and orientation uses Slerp. The initial maximum sampling spacing is 8mm/0.05rad, and each IK uses the previously accepted joint solution as the seed.
2. For an ordinary segment, if `max(abs(q_new-q_seed)) > .3rad`, the jump solution is not accepted directly; bisect within the current alpha interval, first solve the midpoint and accept its continuous solution, then solve the endpoint of the original interval. Recurse at most 5 levels; if it still jumps, the whole preflight is rejected and execution is not enabled. The existing source exception segment uses `IK_SOURCE_BRANCH_JUMP_RAD=1.0`, which is tied to `ignore_right_envelope_steps`; this round must not be summarized as the whole path being subject to the .3 constraint, and the source exception must not be extended to the target.
3. Each accepted joint-interpolation segment still checks 5 FK samples and the existing surface envelope. Duration is computed from the actual subdivision ratio and joint difference: `max(.12, 1.6*delta/.20, position_distance*alpha_span/.025, rotation_distance*alpha_span/.2) / speed_scale`. It does not use a faster speed to jump across the discontinuity, nor does it check only the two endpoints.
4. When reaching the next waypoint, the Cartesian reference is still the originally planned target; the approximate FK error of the previous solution is not accumulated into the new target. Open-gripper waypoints add an offline endpoint check of position ≤5mm and rotation ≤.025rad; on the first violation, re-solve only once for the same fixed target, and if it still exceeds, reject. Currently this check covers waypoints containing open; it is not a uniform convergence guarantee for all motion points.
5. All success, failure, empty-grasp retry and low-position candidates are fully preflighted by the service before enabling; source corrections do not change the target; before release, only the new low-position image after descent decides whether a direct XY fine adjustment is needed, and the init tail segment stays frozen, see 0.5.2. Code changes must be tested offline, then reloaded in an orderly way with an empty gripper, control off and the service idle; restore the reviewed scene, take a new state and generate a new token; old tokens are not reused across processes.

**Internal samples become the executed joint trajectory, but they do not need to become dozens of independent behavior waypoints.** In the previous round, to pass the b9 descent, 1mm segments were written into the success list, giving 84 waypoints/208 samples; this round switched to on-demand subdivision within intervals, 14 waypoints/145 samples, completing b6→c8. The moves and targets of the two also differ; these numbers are not a same-path performance comparison, and they do not prove that "many solver calls" was the only cause of the previous round's off-target landing. Necessary trajectory samples must not be removed just to have fewer points.

The execution side currently uses fixed waits and does not set real-time in-position rejection based on end-effector residuals; in-place gripper actions keep the previous arm target and do not perform a new round of end-effector correction. The open endpoint check above is an offline verification of the nominal solution, **not a restored real-time in-position control**. The landing point is also affected by actual tracking, grip eccentricity, pose rotation, gripper release and gripper retreat. In this round, the post-release FK relative to the target was still [+2.239,−1.523,+6.476]mm, and the final image was slightly off-center. Even with fewer waypoints, the request, commanded solution, actual FK and piece center must each be recorded, and the square finally verified from the image.

If subdivision still fails, prefer changing the clearance pose-rotation position and connection path of the actor hand selected for source, or choose another move that the chess strength allows and that can be executed; the target may be cross-side, but do not switch hands while holding a piece because of which side the target is on, and keep the low position vertical. Do not manufacture a "pass" by using `skip_preflight`, enlarging the source exception, lowering the scene plane or resetting the retry budget. The ordinary-segment .3rad threshold is in the in-package [interpolation implementation](../src/g1d_chess/robot/motion_harness.py); execution must use the current full preflight receipt.

#### 0.5.1 Grasp-success / release-start snapshots and post-execution review

This package adds two kinds of `pose_snapshot`: when the existing static grasp verification returns success, it saves the same-frame feedback that judged success; when the low-position re-capture decision after target descent allows opening the gripper to release, it saves the feedback at that moment; the automatic release after the third low-position XY adjustment is also captured at the low position, labeled `xy_budget_exhausted_auto_release`. High-position or end-of-opening snapshots are not recorded. Snapshot capture itself adds no waiting or image capture; for the pre-release re-capture flow see 0.5.2. It records measured joint angles/velocities, feedback tick/age/monotonic time, the physical actor hand, gripper state, the original plan target, the final effective target, and the `arm_target`/`arm_sent` at that moment. The new capture does not compute FK or the three errors; full plan preflight and the existing clearance protection of the ordinary execution path are kept; the source high-position / target low-position XY correction segments are exempt from motion checks per 0.5.2, and handling of actual hardware and control faults is kept. The left-hand record uses an explicit "observation arm 7 joints + actor arm 7 joints" order; the review restores the physical order, and XYZ must not be mirrored.

Capture copies only a small amount of data into the existing non-waiting asynchronous log; when the command lock is busy, the measured feedback is kept and the command comparison is marked missing. At most two snapshots per transaction are also kept in memory and written to disk with the `pose_audit` of `<token>_result.json` after execution ends; if the log queue is full or writing fails it does not wait, and `missing_events`/`errors` in the result are available for review. Empty grasps, unknown, and source-retry gripper opening do not impersonate these two events. The grasp-success entry covers the current `grasp_retry` static grasp verification flow; the old automatic visual gate flow has no equivalent low-position success sample, and its absence is explicitly labeled.

The post-execution review does only the two things specified in Section 8; first use a new left_high to judge success and offset, then compute the FK errors in this deployment's package root using the same model and Python environment (replace paths with this round's actual archive; the output must be merged with the visual conclusion and written to disk via `xy_error_history.py finish`, which in the same step appends to the shared error JSONL):

```bash
"$CHESS_PYTHON" scripts/review_pose_audit.py \
  --result runs/本轮/tmp/checkpoints/阶段/TOKEN_result.json \
  --output runs/本轮/docs/pose_review.json
```

This tool does not connect to the robot; only at review time does it compute FK from measured joints, and it outputs the three errors (ΔXYZ in millimeters, position magnitude, orientation rotation angle) relative to the original plan, the final effective target and the commanded FK at that moment, plus the FK of the last sent command. The original source and the targets after high-position XY / low-position Z corrections are kept separately. For the new release snapshot, `event_stage=target_low_release_approval`, `original_target` is the original low-position release target, `effective_target` is the most recent low-position command target (not the measured value of the same frame used as an error reference), and `placement_target` records the actual pose used for opening the gripper in place at approval. The historical `target_high_release_approval` and `before_low_open_command` keep their original semantics, can be read offline, and old data is not rewritten. FK is still an estimate from joint feedback + kinematic model, not a visual measurement of the piece center; release start is also not a confirmation that the piece has detached. Source code: [capture](../src/g1d_chess/robot/pose_audit.py), [review tool](../scripts/review_pose_audit.py). The related changes act only on files on disk and were verified offline; the service was not reloaded and the robot was not run; afterwards, after reloading under idle conditions, re-run the preflight to generate a token — old records cannot back-fill feedback for the two moments.

#### 0.5.2 Re-capture after descent, before opening; direct low-position XY fine adjustment when the wrist image judges a clear offset from the target intersection

Left and right hands share this package's [release_alignment.py](../src/g1d_chess/robot/release_alignment.py). The order is **target high position → descend to the release height as planned → keep holding, re-capture the head and actor wrist → decide → open in place → rise vertically with the gripper open → retreat to staging with the gripper open → close → return to init**. `pre_release_gate` is bound to the end of the descent; re-capture no longer happens at the high position. The same `serial_checkpoint.py next/submit` is still used, without opening a separate robot request.

**Criterion for a clear offset: use the physical actor hand's wrist camera to judge the piece's expected landing point after opening, with the head image assisting to check the target square name and neighbors. An expected landing point between two adjacent intersections, in the middle of a square, or clearly offset from the corresponding target intersection is a clear offset that needs correction — submit adjust.** Do not wait until it is "very large" to correct. When the expected landing point is already near the corresponding target intersection, with only small eccentricity and safe to release, submit success; aim at the target intersection as well as possible, but exact alignment is not required, and do not adjust repeatedly for slight eccentricity. What is judged is the expected landing point of the piece, taking into account grip eccentricity and the parallax between the piece's top surface and the board surface; do not use the gripper housing/shadow or "the piece edge covers the intersection" in place of the landing judgment, and do not pass solely on a small FK residual. reason/evidence state the actor wrist image path, the relation between the expected landing point and the target intersection, and the direction of the offset. No uncalibrated millimeter or pixel threshold is defined; the piece covering the target intersection itself does not count as unclear, and the intersection does not need to be fully visible; judge the expected landing point by combining visible grid-line extensions, nearby intersections, the piece center and parallax, and do not correct because of occlusion itself. When the expected landing point genuinely cannot be judged, or there is a collision risk, treat it as needing correction: submit adjust with obvious_large_offset=true, release_alignment_verified=false, and explain the uncertainty or risk and the basis for the correction. A submitted correction must still identify the target and give an explicit non-zero XY based on this run's images; there is no per-step or cumulative distance limit and no motion check on the correction segment; target identification or displacement must not be fabricated.

adjust uses this run's low-position actual FK as reference, keeps the actual Z/RPY at that time, and directly makes the XY correction, without lifting and without descending again. Neither source high-position nor target low-position XY corrections have a per-step or cumulative distance limit, and no motion check is performed on the correction segment, including extra preflight, IK reachability, path, surface clearance, joint limits and sampling checks. The full plan preflight before execution, the visual judgment of the current two images, the correction count limit, and actual hardware fault handling are still kept. This specifies the documented operating policy; whether the running service implements it must be checked against the actual code and configuration — this documentation change does not mean the service has changed. Target has at most 3 independent corrections: re-capture immediately at the low position after the first two; after the third correction is completed, opening in place is released directly, with no further re-capture, visual approval, or "sufficient safety basis already exists" release precondition. Actual hardware or control faults are still handled per the original flow.

The release evaluation still uses **seven fields**: `decision / actual_relative_offset_xy_m / reason / target_identified / release_alignment_verified / obvious_large_offset / evidence`. Source still uses the original six fields. The field name `obvious_large_offset` is kept for submission-format compatibility; its meaning is updated per the intersection criterion above: between two intersections, in the middle of a square, clearly offset from the target intersection, the expected landing point genuinely not judgeable, or a collision risk are all true, not limited to "very large"; the piece covering the target intersection itself does not count as unclear. success requires `obvious_large_offset=false`, `release_alignment_verified=true`, offset=[0,0]; adjust requires `obvious_large_offset=true` and a justified non-zero XY increment. `release_alignment_verified` means the wrist image supports that the expected landing point is near the target intersection and release is safe; it does not mean perfectly centered. Both the service and the submitter reject an adjust that does not declare that correction is needed (obvious_large_offset=true); they validate the decision fields and do not recognize images themselves — the landing judgment is made by this round's image-based decision. Templates: [success](../assets/templates/evaluation.pre_release.success.json), [adjust](../assets/templates/evaluation.pre_release.adjust.json).

On release, record the low-position `release_start` snapshot, then open in place from the reviewed actual position, with no further descent inserted. Only the necessary opening, retreat and high-position connection are generated, chained to the frozen return trajectory; the full result keeps the actual corrections and the final release target. Camera failures, expired decisions and hardware faults are not visual approval.

The new plan and preflight receipt must match `review_pose=after_descent_before_open`, `adjustment_policy=obvious_off_target_intersection`, `alignment_criterion=actor_wrist_predicted_landing_near_target_intersection`, `correction_motion=direct_low_xy`; for the capture-removal stage the criterion is `actor_wrist_predicted_landing_inside_same_side_tray`, the left hand places into the left tray and the right hand into the right tray; release is allowed when the whole piece is expected to land inside the tray's usable area — aiming at the tray center is not required, and the board-intersection criterion does not apply. The planner rejects service receipts of the old policy. For the decision table see [Release landing judgment](release_alignment.md). This change consists of on-disk code and offline verification; **the existing service was not reloaded and the robot was not run**; before the next execution, reload in an orderly way with an empty gripper, control off and the service idle, regenerate the whole preflight and a new token; do not replay historical runs or use old tokens. 5 seconds is a processing target, not a latency guarantee.

### 0.6 Shortening each round and keeping resumable records

- Prefer reusing an existing idle service; process checks must be in the same visible process space as the actual service. A "0 processes" inside the sandbox was once a false negative; it must not be used as grounds to run the launcher that kills old services. The left-service process name must also be counted.
- Reading the current report/position/corresponding success parameters and a single new left_high image once is enough to build a candidate; trace back specifically only on failure. Do not recompute an already verified board model; after grasping, handle only the re-capture after target descent and before opening specified in this section; when both arms are within ≤10cm of their own init, do not repeat the return home for a hand switch; the non-actor arm always keeps the current pose confirmed for this round.
- Prepare the serial entry and the source six-field / target seven-field formats in advance; source corrections do not change the target, and a target re-capture only rebuilds the local segment near placement and chains it to the frozen return tail segment. When a checkpoint arrives, do only two images → judge → submit → next; both source high position and target low position re-capture after the first two corrections; after the third source correction descent and grasp are released directly, and after the third target correction opening is released directly — neither adds a further re-capture or visual approval. The last round's real durations were 51.36/36.03/39.65 seconds; the 5-second target has not been reached and must not be written as a guaranteed response within seconds.
- Each tool call uses the correct workdir; shell variables of the previous call are not inherited automatically. For a long execute, save the session_id and poll only local files; if no session_id is returned, do not store undefined as a valid session, to avoid blocking image review. On submit timeout, first check the receipt with next; do not re-decide or re-execute.
- At most 3 real gripper closings, and source has a cumulative 3 high-position XY corrections; target has another 3 XY adjustments; neither source high-position nor target low-position XY corrections have a per-step or cumulative distance limit or a motion check on the correction segment; each grasp has at most two 5mm low-position lowerings, applicable only to a pure height gap. After a manual reset by the user, take a new image, new plan and new preflight; old coordinates/tokens must not be replayed directly.
- Saving, matching and advancing the five-round forecast are done only in each round's planning phase, see 0.2; after completion, do only the left_high visual judgment, FK error computation and merged write per Section 8. The wrap-up requirements of routinely updating `resume.json`, `board_state_final.json`, long reports and the project's current index are cancelled; the original execution results and logs are kept. The next round takes over from this round's review result and a new image; the service reporting placed cannot replace an accurate landing-square judgment, and an offset does not trigger an additional correction.

### 0.7 Grasp/place experience and acceptance boundaries

[Grasp/place experience and boundaries](experience.md) explains independent source/target estimation, the effective textured grip area, neighbor relations and judging the expected bottom-surface landing point. All coordinates and poses correspond to the current images; historical measured values do not replace measurements of the new environment; the service reporting placed and the end-effector FK error cannot replace the final landing-square acceptance.

### 0.8 90-point marking of the current board

The entry is [board_position/README.md](../board_position/README.md); the new model rebuilds the 90 points from the current raw left_high image per the [marking task description](../board_position/prompts/mark_board.md). That directory only contains documentation and generic scripts, with no preloaded real photos, pixel tables, per-hand end-effector XY or historical pose anchors; the old calibration directory, data and compatibility forwarding entries have been removed, and point marking uses only board_position.

Run `scripts/mark_board.py init/build/query` to complete image binding, recording of manually visible intersections, whole-board or per-region fitting, independent hold-out checking and annotated output. Results go into this run's `runs/board_position/<this-marking-name>/`, and the four corners, the river, the interior and the grid lines near source/target must be visually reviewed. a–i from left to right, 0–9 from near to far. A passing numerical residual still only means pending visual review; points inferred under occlusion must not be called measured.

When planning, estimate source and target separately from the actual positions of this round's board-surface intersections and pieces; head-image pixels are not wrist-image pixels, the piece top surface or arm XY. This tool does not provide a robot base transform or millimeter compensation. Re-locate after the board, camera position, waist pose or cropping changes; when the physical dimensions are unknown, do not default to the old board's grid spacing.

Material and reviews are completed before execute; execution checkpoints use only the prepared references and this run's head image and actor wrist image for judgment, and the whole board is not rebuilt while holding a piece. After completion, it still assists in verifying the square and offset within the Section 8 visual judgment, without adding a third review item. Hand selection, source/target poses and motion rules continue to follow the current single-round flow.

## 1. In-package entry points and scope

| Location | Purpose |
| --- | --- |
| [scripts/round_tools.py](../scripts/round_tools.py) | Offline: create the single-round directory, generate full branches from the current state and manually reviewed coordinates, extract new tokens, summarize results; does not connect to hardware |
| [scripts/mark_board.py](../scripts/mark_board.py) | Mark, validate and query the 90 intersections from a new image; does not output arm coordinates |
| [prompts/board_and_move_conventions.md](../prompts/board_and_move_conventions.md) | Unified conventions for board orientation and moves |
| [config/board_parameters.json](../config/board_parameters.json) | Actual dimensions of the board and pieces |
| [board_position/README.md](../board_position/README.md) / [model operating instructions](../board_position/prompts/mark_board.md) | New-image marking flow; actual PNG, JSON, CSV are generated at use time |
| [config/README.md](../config/README.md) / [first-time setup](first_game_setup.md) | Configuration field descriptions, examples, and the necessary configuration before the first game on a brand-new G1D |
| [config/manipulation_profile.json](../config/manipulation_profile.json) | Generic policy and on-site parameters to be filled in; does not configure the service automatically |
| [assets/templates/round_spec.json](../assets/templates/round_spec.json) / [capture_spec.json](../assets/templates/capture_spec.json) | Plan input for a normal move / capture; source/target XY must be filled in for this round |
| [assets/templates/evaluation.success.json](../assets/templates/evaluation.success.json) etc. | Strict six-field judgment format for the four kinds of pre-grasp checkpoint |
| [src/g1d_chess/robot/](../src/g1d_chess/robot) | Service launcher, client, serial checkpoint and a snapshot of their runtime dependencies |
| [src/g1d_chess/cameras/](../src/g1d_chess/cameras) | Camera client, configuration and an optional standalone capture tool |
| [docs/runtime.md](runtime.md) / [config/environment.example.json](../config/environment.example.json) | Exact interface semantics, dependencies, protection switches and timeout handling |
| [docs/experience.md](experience.md) | Key points of grasp/place judgment, common issues and acceptance boundaries |
| [docs/round_report.template.md](round_report.template.md) | Field hints for the two review items; no separate full report and no added wrap-up tasks, see Section 8 |
| [scripts/verify_package.py](../scripts/verify_package.py) / optional local `manifest.json` | File structure, syntax and link checks; additional hash checks when a local manifest exists |
| [docs/validation.md](validation.md) | This offline test's contents, results and the real-robot validation boundary |

This package can be migrated as a whole; the directory hierarchy must be kept. The model-building module and the URDF/mesh in `assets/models/g1/` are inside the package, so `unitree_deploy` does not need to be installed; the SDK, rules library and Python dependencies are installed via the `chess` group in `pyproject.toml`, and Fairy-Stockfish is installed separately with `CHESS_ENGINE` set. First complete configuration and offline checks per the [environment installation notes](environment_setup.md), then check the on-site network and hardware per the first-time setup guide. This package does not automatically recognize the full position, judge chess rules, complete hand-eye calibration, or guarantee that all squares are reachable.

Each round uses a new image, new state, new plan and new token; planning and execution are completed with current on-site data.

## 2. Stages and deliverables of a round

| Stage | What to do | Required output |
| --- | --- | --- |
| Initialization and observation | Establish/reuse the service and verify the waist is locked at zero per 0.1.3; confirm both arms' poses and their distances to their own init (≤10cm no return home), empty grippers and faults; confirm the round, opponent's move and own side; obtain a single left_high image | Initial full camera response and image, state, real waist, service capabilities |
| Decision and localization | Check Black's actual reply and match the forecast; if consistent and legal use it as the candidate; with no forecast / invalid / exhausted / both hands unreachable, search a new five rounds; locate source/target independently | Selected move, the five-round forecast saved in the planning phase and confirmed progress, localization basis |
| Full planning | Generate everything: pre-grasp, continuous post-grasp tail segments, retry/failure branches | round_spec, request, plan_summary |
| Preflight | Preflight all branches with the current service and check the coverage of the real checks | prepared, stage prepared, a new one-time execute_request |
| Pre-grasp gate | Two images at the current source high position; XY correction if needed; two low-position images and at most two 5mm lowerings | checkpoint, six-field evaluation, decision, receipt, real duration |
| Carrying and release | After a static successful grasp, lift, carry, descend at target, re-capture before opening / direct low-position XY fine adjustment when the wrist image judges a clear offset from the target intersection, release, retreat, return to init | Original execute response / local result, grasp and motion logs |
| Post-move review (two items only) | ① Judge move success and position deviation from a new left_high; ② compute the three FK errors at two moments, append this round's errors / left-right-hand cumulative statistics to the cumulative JSONL in this round's docs, and merge with ① to disk | `<this-round-run>/docs/execution_xy_errors.jsonl` and `docs/{第几轮}_{这一轮的走棋子方案}_{时间戳}.json`, linked to the original images and real results |
| End the service (only when separately requested, not part of the default review) | When the user explicitly asks to end, confirm the piece has been placed and control is off, then shut down the service | shutdown receipt and process count; do not just say "returned to init" |

A capture is two complete grasp/place transactions prepared jointly in advance: first remove the captured piece, then move our own piece. Each of the two grasps has its own pre-grasp gate, and after each successful grasp the respective re-capture after target descent and before opening and the placement tail segment are executed.

## 3. Coordinates, dimensions and parameters that must be reviewed

From the robot's perspective: **left→right a–i, near→far 0–9**. Do not guess image left/right directly from "一、二、三" in Chinese notation. Write a normal move as `红方 i0 → h0`; for a capture, first write `黑方 h0 → empty（黑方 h0 被吃）`, then `红方 i0 → h0`. `empty` means moving into the tray on the actor hand's side (left hand left tray, right hand right tray); XY must be determined from this round's image, and the tray release Z for both left and right hands is 0.159m, see [Tray capture rules](capture_tray.md).

The current board image coordinates are generated by the new-image marking flow in board_position, with no numerical calibration of the old board attached. Physical grid spacing, river width, board thickness and piece dimensions must be verified against the actual objects this time; existing configurations or historical cases cannot automatically be taken as measurements of the new board. Image pixels are not equally spaced due to perspective, and fitting residuals only indicate pixel consistency; while the base-to-board transform has not been independently calibrated, keep it null.

Query example (all offline):

```bash
cd /path/to/g1d_chess
CHESS_ROOT="$PWD"
python scripts/mark_board.py --help
# First generate this run's results per board_position/README.md, then query:
python scripts/mark_board.py query h0 --reference /path/to/current/board_points_pixels.json
```

Each round, first check that the printed grid lines, river edges and palace match this round's image, then use reference points to speed up localization. For source look at the actual piece and the effective grip area at the front of the two fingers; for target look at the target board-surface grid lines, empty spots and neighbors, and do not use an already knocked-askew piece as the standard square center. The piece top surface is above the board surface and its exact height must be verified; oblique viewing produces parallax, so the top-surface center cannot be taken directly as a board-surface pixel. After cropping and zooming, remember the scale/translation and map back to the original image coordinates.

**Board-surface point, piece center, model EE and fingertip are different positions.** A source correction changes the grip offset, and a fixed EE target may also land off; do not automatically add the source offset to the target. The new board-marking tool does not provide robot end-effector anchors.

The following are flow candidates and conventions of the templates and current implementation; they cannot replace measurements on new equipment; source, gate, travel, staging and RPY are determined from on-site images, tool geometry and full preflight. When a fixed implementation constraint needs to change, adapt and verify offline first; do not just fill in a descriptive JSON.

| Item | Current template / flow convention | Conditions of use |
| --- | --- | --- |
| Grasp/release pose | Template roll=0, pitch=45°, yaw about 1.408rad | Check left/right hands and source/target independently; on pose change re-estimate EE and envelope |
| Grasp low-position EE Z | Template .100m, automatic lowering 5mm each time | At most twice, only when XY is already graspable and height contact is insufficient |
| Source high-position EE Z | Template .165m | Separate from the low-position closing stage |
| Carrying height, staging | Template travel=.205m; staging XY must be filled in on site | Check for the current hand and path |
| Release EE Z | Board .139m, tray .159m | This is the model EE height, not the fingertip clearance; if new equipment is incompatible the implementation must be adapted |
| Gripper | close=0, template open magnitude q2.3rad, default motor target +2.3rad | Built-in Dex1 left and right grippers both default to open direction +1; all opens in the same transaction share one q |
| Static grasp success | q≥.15, abs(dq)≤.1, stable 1.5s; contact timeout 2s | Only the in-place stage after closing; no checks added during carrying |
| Budget | Real grasps at most 3; source high-position XY at most 3, target low-position XY another 3 | XY has no distance limit; budgets are not reset by a new token; for the implementation boundary of motion checks see runtime |

Model FK uses the zero-locked waist frame; the actual waist position must be verified from this run's state per 0.1.3. `config/init_state.json` is empty when released; the physical left/right hands' init are determined from on-site same-frame state, and historical poses are not used.

## 4. Create this round's directory and observation

The following commands are executed stage by stage — **this is not a script to run blindly from start to finish**. Path variables are all set in the package root; `RUN` must be replaced with this round's unique name. Read-only/offline preparation can be done first.

These code blocks assume a single persistent terminal. Each independent Codex shell call does not inherit the previous cd or variables; each time, the tool workdir must be set to the package root and the needed CHESS_* variables redefined, or expanded absolute paths used. Another terminal handling checkpoints must also reset the variables; export does not solve inheritance between independent tool processes.

```bash
cd /path/to/g1d_chess
CHESS_ROOT="$PWD"
CHESS_PYTHON="$(command -v python)"
CHESS_CLIENT=scripts/robot_client.py
CHESS_SERIAL=scripts/serial_checkpoint.py
CHESS_RUN="$PWD/runs/RUN"
"$CHESS_PYTHON" scripts/round_tools.py init --run "$CHESS_RUN"
```

`init` only creates tmp/images/docs, the spec to be filled in and resume.json. Each re-plan uses a new run; existing request/token/result are not overwritten. The run directory is created by the current task; the released package does not include replayable historical runs.

After an actual game has been authorized, first check the service process (this command does not start it):

```bash
bash scripts/start_service.sh --status
```

Only after confirming that there is no old service currently executing/holding a piece, and that a start is really needed, use the entry below. **The launcher SIGKILLs all matching old services and must not be used for fault recovery or "just trying"**. Per the [first-time setup guide](first_game_setup.md), activate the Python environment with the dependencies installed, and set this run's `CHESS_INTERFACE`, `CHESS_CAMERA_HOST` and `CHESS_ENDPOINT`; when passing arguments explicitly below use `CHESS_GRIPPER_SIGN=1`, consistent with the built-in Dex1 defaults of the launcher and service. The direction does not change with the host environment or with left/right hand; override explicitly only for special joint conventions. A foreground start needs its own tool session kept open; continue once ready, and do not interrupt it to get back to the prompt.

```bash
bash scripts/start_service.sh --foreground \
  --python "$CHESS_PYTHON" --interface "$CHESS_INTERFACE" \
  --camera-host "$CHESS_CAMERA_HOST" --endpoint "$CHESS_ENDPOINT" \
  --gripper-open-sign "$CHESS_GRIPPER_SIGN" \
  --log-dir "$CHESS_RUN/service" --log-file "$CHESS_RUN/service.log"
```

An existing idle service with the correct ROOT can be reused and should not be restarted casually; check the actual code version and path — new files on disk do not mean the running process has been updated. The service ROOT determines the allowed runs directory, and the generator and service must share the same ROOT. Scripts, runs and the service are all located in this deployment's package root; the left hand uses the in-package left-hand entry, and other root directories are not mixed in. When a switch is really needed, first end the old idle service in an orderly way; while executing/holding a piece, first recover the original transaction.

On an idle service, first obtain full connection and state responses:

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" ping --timeout 5000 --save-response "$CHESS_RUN/tmp/ping.json"
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" get_state --timeout 5000 --save-response "$CHESS_RUN/tmp/initial_state.json"
```

Check the waist hold per 0.1.3; if already locked at zero, only save the verification and do not call it again. Only when zero-locking is needed and the service is idle, dual-arm control is off, both grippers are empty and closed, and there is no fault, execute:

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" lock_waist_zero --timeout 20000 --save-response "$CHESS_RUN/tmp/waist_lock.json"
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" get_state --timeout 5000 --save-response "$CHESS_RUN/tmp/waist_verified_state.json"
```

After the waist hold is verified successfully, a move task then captures the planning image; a task that only establishes the service and locks the waist ends here:

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" capture_cameras \
  --cameras cam_left_high \
  --output-dir "$CHESS_RUN/images/initial" --timeout 20000 \
  --save-response "$CHESS_RUN/tmp/initial_observation.json"
```

Look at this run's left_high planning image and use the state in the response; do not treat a state from another call as the capture moment. Check control=false, fault empty, both arms' current poses, the actor gripper empty and closed, and the real image; compute each arm's init distance per 0.1.2, ≤10cm no return home; if initialization happens, refresh the observation. Record the real waist, left/right arms, q/dq, time, command counts and protection capabilities; wrist stream names versus physical left/right must be checked from the image. If initially holding a piece or in a fault, do not use this package's empty-gripper start generator; first draw up a recovery plan from the existing state.

Read the service's existing scene and check the current table/board/tool configuration. The scene preferably uses applicable on-site values given by the user; when the user provides no on-site information at all, the model estimates all scene fields itself per [First-time setup Section 5](first_game_setup.md#5-configure-on-site-geometry-both-arms-init-and-motion-parameters), and when some are given it fills in the remaining fields. Using available images, state, model, dimension material and reasonable assumptions, record the source, basis and uncertainty field by field, then continue with the setup; user-provided values, supplementary material or extra confirmation are not prerequisites. If the existing scene is compatible, reuse it directly; clearly label estimated values as estimates. `config/scene.example.json` is a template to be filled in and is not loaded automatically; do not submit null, examples or `_field_help`. Save the assessment record as this run's `tmp/scene_assessment.json`; the request keeps only the `scene` values and is saved as `tmp/scene_request.json`. When setting is really needed, call it only with the service idle and control off:

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" set_scene --request "$CHESS_RUN/tmp/scene_request.json" \
  --timeout 5000 --save-response "$CHESS_RUN/tmp/scene_result.json"
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" get_state \
  --timeout 5000 --save-response "$CHESS_RUN/tmp/scene_verified_state.json"
```

Check that the set response has `ok=true` and the scene values, and that the new state's `scene_guard` matches the request; the saved receipt only means the parameters have been applied, not that estimates have become measurements or that collisions have been fully verified. After the scene changes, generate a new plan and re-run the full preflight; old tokens can no longer be used.

## 5. Choose the move, fill in the spec, generate the full plan

Compare the previous round's accepted position with this round's image, and confirm the opponent's action, the current side to move, the source's identity, target occupancy and check responses. Do not assume the round or that a move is legal just because an empty spot appears on the board. Manual piece resets, opponent moves and camera/board changes must be recorded separately.

Fill in `$CHESS_RUN/round_spec.json`: source_square/target_square, the EE XY/Z/RPY and evidence for source and target separately, and four review texts (chess rules, grid registration, pose prerequisites, scene/neighbors). The two evidence entries should separately describe the original image, grid-line positions, the actual grip/release offset and uncertainty. Source XY and target XY have no universal default values.

A normal move uses `move_kind=normal`, `capture=null`. A capture fills in, per `assets/templates/capture_spec.json`, the captured side, the actual grasp pose of the captured piece and the release pose inside the tray on the actor hand's side. `capture.offboard_target.kind=round_tray`, and `physical_side` must match the physical actor hand: left hand left, right hand right; XY and evidence are filled in from this round's image, `z_m` is fixed at 0.159 for both hands, and the planner rejects a mismatched side or release height. `offboard_surface_review` reviews empty spots inside the tray, the tray rim, already placed pieces, support height and tool clearance; tray release Z is uniformly 0.159m. Exact centering is not required; release is fine when the whole piece is expected to land inside the tray's usable area. For details see [Tray capture rules](capture_tray.md).

```bash
"$CHESS_PYTHON" scripts/round_tools.py build --run "$CHESS_RUN" \
  --spec "$CHESS_RUN/round_spec.json" \
  --observation "$CHESS_RUN/tmp/initial_observation.json"
```

It outputs `tmp/request.json` and `tmp/plan_summary.json`, **which are only a candidate full plan**. Check that the left/right arms, the first vertical rise, the high-position pose rotation, the source gate, the source low position, the fixed target, release/retreat/init and the failure return are all correct. The physical actor arm returns to its own init after the move, and the physical non-actor arm always keeps the current pose confirmed for this round; this holds for both left- and right-hand execution, and the plan contains no camera pose.

The grasp-success path uses the latest order: **close at source low position → static grasp verification → rise in place to clearance → necessary high-position pose rotation → directly to target high position → descend at target → re-capture two images and decide immediately (release if the wrist image shows the expected landing point near the target intersection; direct low-position XY fine adjustment if between two intersections, in the middle of a square, or clearly offset from the target intersection) → open in place → rise vertically with the gripper open from the actual XY/RPY → retreat to staging with the gripper open → close in place → rotate back at height → init**. The loaded move is not forced to detour through staging. Every open/close is listed as its own settle action; `success_source_steps=[0,1,2]` limits source corrections; target corrections are controlled separately by `pre_release_adjustment`, and init is frozen. This package's generator already outputs the order of rising with the gripper open, retreating to staging with the gripper open, then closing; manual reordering in historical run build scripts must not be applied again. Execution uses this package's entry specified in 0.1, and verify that the process has indeed loaded the matching version.

During open/close the actor arm stays at the current measured position; the XYZ/RPY in an open/close waypoint does not drive the arm at the same time. If the plan needs to move first, an independent `hold` motion waypoint must be arranged explicitly, with the open/close after arriving; that waypoint is included in the original full plan preflight and adds no extra preflight path during execution. The opening tail segment after release approval uses the latest feedback to estimate the opening pose; see [Runtime semantics](runtime.md) for details.

The generator also provides empty-grasp retry, failure retreat and unknown-contact branches. It does not verify the authenticity of manual evidence, image freshness, chess rules or neighbor collisions, nor does it do IK; the full service preflight must still follow. Configuration JSON is a description/template; modifying `manipulation_profile.json` does not automatically change the budgets and thresholds inside the generator, and the currently adjustable inputs are those in round_spec; changing other policies requires modifying the generator and preflight in sync and verifying.

## 6. Preflight, enable and submit one execution

Normal move:

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" prepare_transaction \
  --request "$CHESS_RUN/tmp/request.json" --timeout 120000 \
  --save-response "$CHESS_RUN/tmp/prepared.json"
```

A capture uses `prepare_sequence` in the same place, and the request must contain exactly two transactions. The first releases the captured piece into the same-side tray, and the open-in-position feedback must succeed; then, along the preflighted path, it rises vertically with the gripper open to clearance, moves continuously via the high-position staging to our own source high position, re-captures directly and enters the pre-grasp gate. Staging is only a motion waypoint, with no independent visual confirmation or manual waiting; the second transaction uses the already opened gripper — no repeated opening, no return to init, no re-enable, and no second service request inserted. Only after the second transaction's final placement does it return to init. This flow is generic for left and right hands and any capture; if the gripper fails to open fully, a hardware fault occurs or a path check fails, stop the chaining and preserve the on-site state. Do not split "grasp" and "place" into two transactions; the re-capture after target descent and before opening is completed within the same transaction.

The preflight must be `ok=true`; review all success/retry/failure branches and the scene check results. `collision_checked=false` must be kept truthfully: the existing IK, joint/sampling and table tool-envelope checks are not a complete arm, neighbor or dynamic collision verification. The source's `ignore_source_tool_envelope=true` is an explicit exception and must be reviewed; if the geometry currently does not satisfy it, do not loosen other protections to force a pass.

```bash
"$CHESS_PYTHON" scripts/round_tools.py tokens --run "$CHESS_RUN" --prepared "$CHESS_RUN/tmp/prepared.json"
```

This generates `tmp/execute_request.json` and `tmp/stage_1_prepared.json` (a capture also has stage_2). A token is valid only in the current service process and is consumed once. After preparation, changing the scene, restarting, moving the start pose or changing the plan requires a new preflight; old tokens must not be placed into generic templates.

At this point all explanation, localization and branch review is complete. If execution of this round has been authorized, submit the corresponding full transaction immediately after enabling:

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" enable_control --timeout 5000 \
  --save-response "$CHESS_RUN/tmp/enabled.json"
```

First read enabled.json and confirm ok=true, control_enabled=true and no fault, then submit:

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" execute_transaction \
  --request "$CHESS_RUN/tmp/execute_request.json" --timeout 900000 \
  --save-response "$CHESS_RUN/tmp/result.json"
```

A capture uses `execute_sequence` instead of `execute_transaction`, submitting the sequence only once; the two sub-tokens must not be executed independently. Check the enable receipt; if control was not enabled successfully, do not continue.

execute is a long call: keep its running session in the tool so that the main session can handle local checkpoints at the same time; record the session_id in resume.json. It does not wait until the command fully returns before reviewing images. For terminal operation, another terminal is needed to handle local files. During execution **do not send a second REQ such as get_state/hold/capture**; the synchronous REP is occupied by the executing request.

## 7. Pre-grasp / pre-release serial gates and continuous return home

First select the first stage and poll only local files:

```bash
CHESS_PREPARED="$CHESS_RUN/tmp/stage_1_prepared.json"
"$CHESS_PYTHON" "$CHESS_SERIAL" next --prepared "$CHESS_PREPARED" --seconds 45
```

When `needs_review=true`, read the returned **current head and actor wrist images**, the actual FK, stage and remaining budget. While a checkpoint is open, do only "read → judge from two images → submit (source six fields, target seven fields)"; do not insert history searches, code reading, tests, other models or report writing. 5s is a target; real durations must be kept, and it must not be claimed that history has already achieved it.

Write a new evaluation file in this round's tmp. source/preclose contains only: `decision`, `actual_relative_offset_xy_m`, `reason`, `source_identified`, `descent_alignment_verified`, `evidence`. `pre_release` replaces the identity/alignment fields with `target_identified`, `release_alignment_verified`, and adds `obvious_large_offset`; see the dedicated templates in 0.5.2. All templates must be filled with this run's image basis and must not be submitted as-is.

| Current stage | Allowed decisions | Meaning |
| --- | --- | --- |
| source high | success / adjust / failure | Confirm the effective grip area both left-right and front-back; for insufficient centering use a justified XY adjust and re-capture (no distance limit, no motion check on the correction segment); ordinary deviation does not go straight to failure and return to init; failure is only for real anomalies or when it is not safe to continue |
| preclose low position | success / adjust_z / retry_high / failure | Close only when centering and neighbor clearance are satisfied; pure insufficient grip depth may lower 5mm per the preflight; for insufficient front-back/left-right centering use retry_high, XY=[0,0], keep the gripper open with fixed XY/RPY, rise vertically and re-capture, then correct XY at the high position and re-capture; do not submit a low-position XY or a self-chosen Z, and ordinary deviation does not go straight to failure and return to init |
| pre_release low position (descended, gripper not yet open) | success / adjust / failure | Open if the wrist image shows the expected landing point near the target intersection; between two intersections, in the middle of a square, clearly offset from the target intersection, landing point genuinely not judgeable, or collision risk all need correction; the piece covering the intersection itself does not count as unclear; automatic release after the 3rd, with no 4th re-capture |

The high-position offset is a metric increment relative to the **current actual FK**; for source the submitter converts it into a cumulative planning reference, while target directly uses this run's low-position actual FK as reference, without applying the source's cumulative offset. Do not accumulate twice or mix in image pixels. After the first low-position adjust_z, re-capture; after the second, close directly — there is no third low-position gate. An empty grasp first returns to the high position, re-captures and corrects XY from the new image; source high-position corrections total 3, re-capture after the first two corrections; after the third correction is completed, descent and grasp are released directly, with no further high-position re-capture or visual approval; the existing low-position close-gripper gate is kept, and only if that attempt is still empty does it return to init along the preflighted path. Real grasps total at most 3 in this round and are not reset by re-planning; each real re-grasp recomputes the low-position budget per the preflighted sequence. The target's 3 XY adjustments are counted independently, with direct release after the third.

Set `CHESS_CHECKPOINT` to the current checkpoint_path returned by next, and `CHESS_EVALUATION` to the new evaluation JSON just written for the corresponding stage:

```bash
"$CHESS_PYTHON" "$CHESS_SERIAL" submit --prepared "$CHESS_PREPARED" \
  --checkpoint "$CHESS_CHECKPOINT" --evaluation "$CHESS_EVALUATION"
```

Check the accepted receipt, then continue with next to read the next gate or the final result. A submit timeout while waiting for the receipt does not mean the decision did not take effect; run next first, and never submit twice.

**After a static successful grasp, the service automatically carries the piece to the target high position and descends to the release height, and re-captures two images before opening. Release if the wrist image shows the expected landing point near the target intersection; direct low-position XY fine adjustment if between two intersections, in the middle of a square, or clearly offset from the target intersection; re-capture after the first two, open directly after the third. On release, save the low-position FK snapshot, then continuously open, retreat and return home.** Local progress and the original long call's result can be read; new actions must not be inserted via service requests. The in-place stable-q criterion is not extended to grip monitoring throughout carrying. The existing health/surface protection of the ordinary execution path is kept; the source high-position and target low-position XY correction segments have no motion check per 0.5.2, and actual hardware or control faults can still stop it. Service runtime capabilities are defined by the code, configuration and real-time state.

Once the tray release decision of a capture's first stage is accepted, the service itself performs the opening, retreat and transition to our own source high position, without waiting for the operator at staging. When the first stage shows `finished/placed` while the overall sequence is still executing, immediately switch `CHESS_PREPARED` to stage_2, use `next` to read our own source high-position gate and handle this run's two images; prepare the stage_2 file path in advance, without waiting for separate confirmation or re-planning. Do not declare the whole capture complete just because the first piece has been removed; a second-stage failure must be recorded as the position having partially changed.

## 8. Post-move review: two items only

**Confirmed by the user: the post-move review does only the following two things, identically for left and right hands.** Completion of execution and return to init are premised on the original execute's final result; a client timeout does not equal completion — still wait for the real result per Section 9. A capture must wait until both sub-transactions have finished; do not review early after the first piece is removed. The review does not read additional real-time FK and does not back-fill execution event snapshots.

1. **Capture one new `cam_left_high` image and judge whether the move succeeded and whether there is a position deviation.** Against this round's source→target and the target intersection, record success/failure/undeterminable, as well as the offset direction and any deviation that can be reliably estimated; when precision is insufficient, state unknown, and do not take the pixel deviation directly as a base-frame XY error in millimeters. Keep the original image path and capture time.
2. **Now compute the FK errors offline and merge them with the result of item 1 to disk.** Use only the two moment snapshots saved during execution — "grasp success" and "low-position approval to open and release after target descent" — and compute for each the three errors of the actual FK relative to the original plan, the final effective target and the commanded FK at that moment, including ΔXYZ in millimeters, position magnitude and orientation rotation angle. The new release snapshot is compared with the corresponding low-position target; historical high-position events are still compared with historical high-position targets, without mixing. Put the image judgment, offset description and FK results into one file for this round, `docs/{第几轮}_{这一轮的走棋子方案}_{时间戳}.json`, keeping the paths of the original images, action results and FK details. If a snapshot is missing or the computation fails, record the reason truthfully and do not fabricate values; visual offset and arm FK error are recorded side by side, without asserting causation directly.

**Final review file naming:**

```text
<this-round-run>/docs/{第几轮}_{这一轮的走棋子方案}_{时间戳}.json
示例：<this-round-run>/docs/round_move.json
```

- **第几轮 (round number)**: always written as `第N轮`, where N is our side's move round in this game, starting from 1; it increments with our next move, and a new game restarts from 1. It is not the forecast array index, the engine half-move count, the number of grasps or the run sequence number. Re-planning/retries in the same round do not increment it; the removal and placement sub-transactions of a capture are still one round.
- **走棋子方案 (move plan)**: use the final selected move of this round in Chinese notation, such as `马八进七`, `车三平六`; when there is front/rear piece ambiguity use the full notation. The file content also keeps the source/target square names. The name must not contain path separators, whitespace or underscores, and putting a whole explanation in the file name is forbidden. On failure/offset, still name by this round's planned move and write the real result in the content; the file name must not be treated as proof of success.
- **时间戳 (timestamp)**: Beijing time (`Asia/Shanghai`) at the moment the merged review is actually written to disk, format `YYYYMMDD_HHMMSS`; do not use planning, preflight or capture time. If the full file name already exists, wait for the next second and take the time again, using exclusive creation; existing records must not be overwritten.
- In the planning phase confirm and save this game's round number and final move, and prepare the merge script; during execution do not search history or organize files for naming. When merging review item 2, generate the timestamp and save in the JSON `round_number`, `move_plan` and a `review_created_at` with the `+08:00` time zone, consistent with the file name.
- The final merged record is saved only per this rule; the fixed name `post_move_review.json` is no longer generated as the final entry. `pose_review.json` and similar are only offline FK intermediate details and do not replace the final review; a capture's two FK details are merged into the same review for this round. Create this round's `docs/` directory when preparing the output.
- When continuing a game, the next round's planning phase reads the review in the corresponding run that follows this rule, and checks it by the JSON's game ownership, round number, move and real result; do not infer the position from file-name lexicographic order or the newest file across the whole project. Historical fixed-name records can still be read as evidence and do not need bulk renaming; fixed file names in historical prompts/memory are superseded by this rule.

Item 1's capture reuses this round's client and run directory:

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" capture_cameras \
  --cameras cam_left_high \
  --output-dir "$CHESS_RUN/images/final" --timeout 20000 \
  --save-response "$CHESS_RUN/tmp/final_observation.json"
```

Item 2 uses the `scripts/review_pose_audit.py` of 0.5.1 in this deployment's package root. Set `CHESS_RESULT` to the path of this round's real transaction result that contains `pose_audit`; a timeout response or prepared must not be used instead:

```bash
"$CHESS_PYTHON" scripts/review_pose_audit.py \
  --result "$CHESS_RESULT" \
  --output "$CHESS_RUN/docs/pose_review.json"
```

For a capture, compute separately for the real results of the two sub-transactions, output different files, and merge them into the same final review JSON named per the rule above while keeping the sub-transaction correspondence; do not pass the sequence wrapper result directly to a tool that only accepts transaction results. The existing FK tool outputs FK details; completing item 2 also requires merging its content with item 1's visual judgment into the review record above — do not leave only scattered, unlinked files.

**Item 2 also maintains the independent error JSONL; this must not be omitted.** In planning, prepare the current context, game_id, round number and Chinese-notation move; write item 1's visual judgment to `tmp/visual_review.json` (containing at least `move_success`, the original image path/time and the offset description), then use the offline tool to complete both the log append and the named review merge in one go:

```bash
"$CHESS_PYTHON" scripts/xy_error_history.py finish \
  --run "$CHESS_RUN" --game-id "$CHESS_GAME_ID" \
  --round-number "$CHESS_ROUND_NUMBER" --move-plan "$CHESS_MOVE_PLAN" \
  --context-id "$CHESS_XY_CONTEXT" \
  --report "$CHESS_RUN/docs/pose_review.json" \
  --visual-review "$CHESS_RUN/tmp/visual_review.json" \
  --plan-summary "$CHESS_RUN/tmp/plan_summary.json"
```

By default this command appends one JSON line per round to `<this-round-run>/docs/execution_xy_errors.jsonl`; the line contains this round's full errors and the updated independent left/right-hand and source/target cumulative statistics (`count`, `sum_delta_xyz_mm`, `mean_delta_xyz_mm`); missing/unqualified samples still keep their line and reason, and the count does not increase. With the same token and the same inputs it is idempotent and does not learn twice. For a capture, first pass our own main-move FK report, then repeat `--report` to pass the removal report; the auxiliary report is kept but off-board errors are not learned. The final review links the JSONL path and this round's cumulative statistics. This operation belongs to item 2 and does not add a third wrap-up item. For the specific measure, context and the next round's read-only reference flow see [Historical ΔXYZ error reference](xy_error_learning.md).

The merged record contains only this game/round identifiers, the naming metadata above, source→target, original image path and time, the visual success and offset conclusion, the FK errors at the two moments and any missing reasons, the corresponding result file paths, the planning error reference, and the JSONL path / this round's cumulative statistics. Once the final JSON and log are written to disk, the review ends; the reply gives only the success/offset conclusion, a brief FK error summary and file paths. Do not run the old `round_tools.py report`, do not fill in the full report template, and do not add position analysis, calibration updates, history searches, index maintenance, extra camera/state captures, corrective motions or service shutdown. The position check needed for the next round is left to the next round's planning.

### 8.1 End the service only when the user separately and explicitly requests it (not part of the review)

A normally successful round or a full-preflight failure retreat turns off arm control, but the process may still be running; `hold`, `release_control` and `shutdown` are not the same state. If this round's task requires ending the service, after confirming the piece has been placed, the arms can be supported and the service is idle, release first if necessary, then shutdown:

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" get_state --timeout 5000 --save-response "$CHESS_RUN/tmp/before_shutdown.json"
```

Call only when release has been confirmed safe and control is still enabled:

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" release_control --timeout 5000 --save-response "$CHESS_RUN/tmp/released.json"
```

At the end of a normal move keep the waist hold; do not release it because arm control is off. Only when the user explicitly asks to release the waist hold or end the service, and the site allows release and arm control is off, if `waist_hold_enabled=true`, call and save the receipt:

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" release_waist --timeout 5000 --save-response "$CHESS_RUN/tmp/waist_released.json"
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" get_state --timeout 5000 --save-response "$CHESS_RUN/tmp/before_shutdown_verified.json"
```

After confirming `control_enabled=false` and `waist_hold_enabled=false`, execute when the user asks to end the service:

```bash
"$CHESS_PYTHON" "$CHESS_CLIENT" --endpoint "$CHESS_ENDPOINT" shutdown --timeout 5000 --save-response "$CHESS_RUN/tmp/shutdown.json"
bash scripts/start_service.sh --status
```

Prove shutdown by the shutdown receipt and a process count of 0, not just by whether the socket exists. When abnormally holding a piece, do not automatically release/shutdown, and do not clean up by restarting the launcher.

## 9. Interruption, timeout and recovery

`resume.json` is updated by the operator and records at least: the current stage, run, the original execute tool session_id, whether it has been sent, the current stage prepared, and the latest checkpoint and result paths. This file is an index, not a real-time state machine; do not resend an action based solely on a `false` in it.

When a new session takes over, read in order this round's spec, summary, prepared, local results/checkpoints and the original execution session. A timeout or user interruption after a request has been sent **does not mean cancellation**. The service may still be moving, or may wait indefinitely for a file decision; `decision_timeout_s=300` is not an automatic fallback time for the serial gate, and `next --seconds 45` only controls local polling.

| Observed situation | Handling |
| --- | --- |
| Current checkpoint not yet decided | Read the current two images, judge per this manual and submit once |
| decision exists but no receipt | Wait / check local next and logs; do not submit again |
| Post-grasp tail segment executing | Keep the original long call, read the local result; do not send a new REQ or insert a gate |
| Original client response timed out, request_sent=true | Do not execute again; check the local token_result, the original tool session and the service log |
| Client request_sent=false | Only means it was not sent successfully; first confirm the service/site state before deciding the next step |
| Local result completed | Complete the two review items per Section 8; do not replay the token |
| Service restarted, token invalid | First reconfirm the site; old prepared cannot restore the in-memory cache |
| Grasp failure or unknown/fault | Look at the preflighted branch actually taken and the holding state; do not assume an empty gripper or that the piece has been released |

The current synchronous service cannot immediately handle another hold/release request during execute, so it is not an independent emergency stop channel. A SIGINT to a verified service process sets cancel/hold when control is enabled, which is not the same as interrupting the client; SIGKILL/restart is also not a controlled recovery. For the specific branches see [Runtime notes](runtime.md). On hardware faults or unclear state, do not continue on a guessed trajectory.

## 10. Maintenance and offline verification

```bash
python scripts/verify_package.py
"$CHESS_PYTHON" -B -m unittest discover -s tests -v
```

Verification does not start the service, call cameras or perform motion. This package ships with the planning generator and service code. This packaging verification does not mean the package has connected to the robot or completed a new real-robot round. Each round's IK and scene preflight are still performed by the actually running service.

The optional local file hash manifest is not uploaded to Git and does not cover runs added later; when the manifest is missing, offline verification still checks file structure, syntax, links and the script inventory. The runtime camera configuration YAML may be rewritten by the server; checksum differences should be recorded with their cause, and changes must not be silently treated as the original snapshot. After maintenance, report the verification and its source; when using the local hash manifest, maintain it in sync and keep a change record. Project experience is kept in this directory; do not automatically change the global AGENTS or promote single-run parameters to cross-project rules.
