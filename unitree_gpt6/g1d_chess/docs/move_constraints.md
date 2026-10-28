# Move Selection and Robot Reachability

English | [中文](move_constraints.zh.md)

Current rule: legal moves are not excluded in advance by cross-board distance or target file. Right hand to a-b and left hand to h-i can both be candidates; crossing the board does not mean the real robot can reach it. `scripts/move_constraints.py` keeps all legal moves from its input and marks `policy_id=reachability_gated_cross_board_v2`; whether a move can actually be executed is decided jointly by this round's complete path preflight and an explicit judgement of the risk of physical interference with neighbouring pieces. 2D occlusion must not be treated as a collision that has already happened.

## Planning order

1. Obtain the complete set of legal moves from the currently confirmed position; when in check, consider only legal check responses. A predicted next move that hits is likewise not discarded for being at the edge across the board.
2. Choose the preferred executing hand by source file: a-d prefers the left hand, e-i prefers the right hand. Using this round's new image and the measured start pose of the preferred hand, estimate the source and target XYZ/RPY separately; each of the two phases first considers a more lateral approach. A feasible lateral grasp does not require a lateral release; if a phase cannot be reached laterally, or has an explicit risk of pushing or pinching a neighbouring piece that cannot be eliminated among the lateral candidates, try a more longitudinal approach for that phase. During grasping, occlusion, 2D projection overlap or proximity by themselves are not collisions and are not on their own grounds to refuse a grasp.
3. Determine the source and target RPY separately; after changing the release RPY, re-estimate the target EE XYZ rather than reusing the grasp pose's EE coordinates, change orientation at a clearance high position and build again. The complete preflight covers grasp, retries, re-orientation while holding the piece and transport, release, upward retreat and return to init; for a capture, preflight the entire joint sequence. Only when none of the preferred hand's candidates passes the complete preflight, or all have an explicit actual interference risk, re-estimate from the other hand's measured start pose, read that hand's error reference and run a complete preflight.
4. Abandon the move and continue searching the next legal candidate only when neither hand has a candidate that passes the complete preflight without an explicit actual interference risk. Do not substitute single-point IK, old coordinates or relaxed scene protection for the complete preflight.

At the package root, activate the Python environment with the dependencies installed and set `CHESS_PYTHON="$(command -v python)"` (see [first setup](first_game_setup.md)). The search helper script accepts the list of legal UCI moves returned by the current rules engine:

```bash
"$CHESS_PYTHON" scripts/move_constraints.py --legal-moves <this-round-run>/tmp/legal_moves.json
```

The output contains `policy_id`, `allowed_legal_moves`, `excluded_moves` and `go_command`. Except when there is no legal move, `searchmoves` contains all legal candidates; the script does not call the engine or the robot, and does not guarantee that motions are reachable. Every round must still verify that the engine `bestmove` belongs to the current legal list, estimate grasp and place poses from real images, and run the complete service preflight. Future moves in the predicted principal variation are only predictions and are not truncated in advance by how far they cross the board; reachability is preflighted from the actual situation when the corresponding turn arrives.
