import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

"""Offline move-filter checks, including UCI rank conversion and search exhaustion."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("move_constraints_tested", Path(__file__).resolve().parents[1]/"src/g1d_chess/chess/move_constraints.py")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class MoveConstraintsTests(unittest.TestCase):
    def test_cross_board_moves_remain_search_candidates(self):
        for source, target in (("g4","a4"), ("e4","b4"), ("b6","i6"), ("d3","h3"),
                               ("g4","d4"), ("b6","f7"), ("e4","c4"), ("d3","g3"),
                               ("a1","b1"), ("h1","i1")):
            self.assertTrue(m.review_move(source, target)["allowed"])

    def test_search_filters_all_legal_roots_and_handles_rank_ten(self):
        result = m.search_options(["g5a5","g5d5","b7i7","b7c9","b8b10","g5d5"])
        self.assertEqual(result["allowed_legal_moves"], ["g5a5","g5d5","b7i7","b7c9","b8b10"])
        self.assertEqual(result["go_command"], "go movetime 10000 searchmoves g5a5 g5d5 b7i7 b7c9 b8b10")
        self.assertEqual(m.review_uci("b8b10")["target"], "b9")

    def test_empty_legal_set_never_generates_unrestricted_search(self):
        result = m.search_options(["g5a5","b7i7"])
        self.assertEqual(result["status"], "search_allowed_moves")
        self.assertEqual(m.search_options([])["status"], "no_legal_moves")
        self.assertIsNone(m.search_options([])["go_command"])
        for move in ("a0a1","a1a11","b7c9\nquit",None):
            with self.assertRaises(ValueError):
                m.review_uci(move)


if __name__ == "__main__":
    unittest.main()
