# Document Roles and Reading Order

English | [中文](README.zh.md)

This directory is a topical supplement to the [repository entry](../README.md), the [Operation Manual](operation_manual.md) and the [per-round round_plan](../prompts/round_plan.md). The Operation Manual provides the complete operating specification, commands and exception handling; round_plan organizes the same specification into the execution order for each round. Topic pages explain fields, judgment criteria and interface boundaries, and do not set up a separate flow. The current user's explicit instructions take precedence; when the documentation is found to disagree with the capabilities of the running code, check the specific implementation first, and do not claim the service supports something based on documentation alone.

New users read [First Game Setup](first_game_setup.md) → [Package Usage and Dependencies](package_setup.md) → the Operation Manual → round_plan, and before execution gather materials according to the [Script Inventory](round_script_inventory.md). Project documents, scripts and models are taken from within this deployment's `g1d_chess/`; Python dependencies are installed from the chess group of the root pyproject.toml, and the engine is installed separately. The target machine still needs a compatible Python and native libraries, plus the actual network, cameras and robot; the original working directory or a user-level skills directory is not needed.

New data for each round goes under the package root `runs/<this-round-name>/`, created on demand by initialization or startup; the release package presets no run records.

## Topic mapping

| Document | Corresponding main specification | Purpose |
| --- | --- | --- |
| [first_game_setup](first_game_setup.md), [package_setup](package_setup.md) | Operation Manual 0.1, Section 4; round_plan service preparation | New device setup, in-package paths, dependencies and versions |
| [runtime](runtime.md) | Operation Manual 0.1, 0.5, Sections 5–9 | Semantics of transactions, checkpoints, timeouts and control state |
| [round_script_inventory](round_script_inventory.md) / [JSON](round_script_inventory.json) | Operation Manual Sections 4–9; round_plan steps 1–6 | Required scripts, runtime modules and call order |
| [move_constraints](move_constraints.md), [move_forecast](move_forecast.md) | Operation Manual 0.2; round_plan steps 1–2 | All legal moves, hand-switch reachability, five-round forecast |
| [neighbor_clearance_and_landing](neighbor_clearance_and_landing.md), [tip_grasp_policy](tip_grasp_policy.md) | Operation Manual 0.3–0.5; round_plan steps 2–3 | Independent grasp/place estimation, neighbor relations, graspable is enough |
| [release_alignment](release_alignment.md), [capture_tray](capture_tray.md) | Operation Manual 0.5.2; round_plan steps 4–5 | Low-position landing point, same-side tray, consecutive capture |
| [xy_error_learning](xy_error_learning.md) | Operation Manual 0.4.1, Section 8; round_plan steps 2 and 6 | Two-event FK error, same-hand context, JSONL and merged review |
| [experience](experience.md), [round_report.template](round_report.template.md) | Operation Manual Section 8; round_plan step 6 | Experience boundaries and hints for the two review fields; not an extra reporting task |
| [validation](validation.md) / [record](package_validation.json) | README package verification entry | Offline verification facts and on-site conditions not yet verified |

Board point material is consolidated in [board_position](../board_position/README.md), containing only general instructions and scripts; this session's 90 points are generated from a new image, and no historical measured data or robot XY is provided. For square names and move notation see the [Unified Conventions](../prompts/board_and_move_conventions.md).

## Key execution rules

- Planning uses only one new head image; source high/low position and target low position each use this step's head image plus the physical acting hand's wrist image. Camera mapping is filled in on site, not guessed from stream names.
- The actual waist is locked at zero and verified; if each arm is within ≤10cm in 3D of init, do not re-initialize; the non-acting arm stays where it is. A normal ending keeps the waist hold.
- Source and target XYZ/RPY are estimated separately from this round's images, with historical errors only as a reference; all legal moves may cross the board, and if the preferred hand is unreachable try the other hand. The full plan and both capture segments are jointly pre-checked before execute.
- Source high position at most 3 XY corrections: recapture after the first two, release for descent directly after the third; the existing low-position closing gate is kept. Graspable is enough; full centering is not required. Use retry_high when low-position XY is insufficient; only consider a 5mm adjust_z when XY is graspable and only height coverage is insufficient: recapture after the first, close directly after the second. At most 3 real gripper closes, not reset across re-plans.
- Source high-position and target low-position XY corrections have no per-step/cumulative distance limit and no correction-segment motion check; full plan pre-check, visual gates, count limits and handling of actual hardware faults are kept. For target, recapture after the first two corrections, and after the third open the gripper directly in place, with no added approval conditions.
- For an ordinary move, after the target descent judge from the two images, then open the gripper, retract vertically with the gripper open, back off to staging with the gripper open, close in place, and return to init from the high position; for a capture, after the first segment's tray release and verification that the gripper is open, go directly into the second segment's source, without returning to init in between, re-enabling, re-opening or starting another pre-check.
- During execute, only handle local next/submit and the original long-running call; do not synchronously query the service or search material. The end has only two review items: visual judgment from a new head image, and offline merging of the three kinds of FK error for the two events with the JSONL. 25mm is the learning-sample filtering threshold and must not be used as a correction distance limit.

For differences between the operating specification and the actual implementation see [runtime](runtime.md#implementation-verification-boundary); a new device must independently complete on-site setup and verification.

For environment installation, chess group dependencies, the external engine and the scope of standalone operation see the [Environment Setup Guide](environment_setup.md).
