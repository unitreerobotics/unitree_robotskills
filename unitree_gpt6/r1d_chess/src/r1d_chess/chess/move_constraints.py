"""Offline robot move filter; input legal moves must come from current chess rules."""
import argparse
import json
from pathlib import Path
import re

POLICY_ID = "reachability_gated_cross_board_v2"


def review_move(source, target):
    if not all(isinstance(s, str) and re.fullmatch(r"[a-i][0-9]", s) for s in (source, target)):
        raise ValueError("board squares must be a0..i9")
    hand = "left" if source[0] in "abcd" else "right"
    return {"policy_id": POLICY_ID, "source": source, "target": target,
            "physical_actor": hand, "allowed": True,
            "reason": "requires_full_reachability_preflight"}


def review_uci(move):
    match = re.fullmatch(r"([a-i])(10|[1-9])([a-i])(10|[1-9])", move) if isinstance(move, str) else None
    if not match:
        raise ValueError("expected xiangqi UCI move with ranks 1..10")
    f1, r1, f2, r2 = match.groups()
    return {"uci": move, **review_move(f1 + str(int(r1)-1), f2 + str(int(r2)-1))}


def search_options(legal_moves, movetime_ms=10000):
    if not isinstance(legal_moves, list) or not all(isinstance(x, str) for x in legal_moves):
        raise ValueError("legal_moves must be a JSON list from current rules")
    if type(movetime_ms) is not int or movetime_ms <= 0:
        raise ValueError("movetime_ms must be a positive integer")
    reviews = [review_uci(move) for move in dict.fromkeys(legal_moves)]
    allowed = [r["uci"] for r in reviews if r["allowed"]]
    return {"policy_id": POLICY_ID, "allowed_legal_moves": allowed,
            "excluded_moves": [r for r in reviews if not r["allowed"]],
            "status": "search_allowed_moves" if allowed else
                      ("no_legal_moves" if not reviews else "no_moves_satisfy_robot_policy"),
            "go_command": f"go movetime {movetime_ms} searchmoves " + " ".join(allowed) if allowed else None,
            "scope": "root robot move only; validate returned bestmove and future robot plies separately"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--legal-moves", type=Path, required=True,
                        help="JSON array from pyffish.legal_moves for the confirmed current position")
    parser.add_argument("--movetime-ms", type=int, default=10000)
    args = parser.parse_args()
    print(json.dumps(search_options(json.loads(args.legal_moves.read_text()), args.movetime_ms),
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
