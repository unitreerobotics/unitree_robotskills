from __future__ import annotations

import sys
import unittest
from pathlib import Path

SKILL_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SKILL_ROOT / "scripts"))

import eef_eval  # noqa: E402

ARM = {"abd_max": 62.8, "elbow_drop_min": 0.051, "tau_yaw_max": 3.81}
PATH = {"ok": True, "fail": None, "max_joint_travel": 0.97, "link_gap_min_per_wp": [0.07, 0.13],
        "self_clearance_min_per_wp": [0.025, 0.032]}


class BriefTest(unittest.TestCase):
    def test_single_arm_line(self):
        line = eef_eval.brief("lettuce", {**ARM, **PATH})
        self.assertEqual(line.split(), "lettuce OK abd_max 62.8 drop_min 0.051 tau 3.81 travel 0.97 "
                                       "selfmin 0.025 gapmin 0.07 n_ok 2".split())

    def test_two_arm_line_reports_the_worse_arm(self):
        better = {"abd_max": 10.0, "elbow_drop_min": 0.19, "tau_yaw_max": 0.2}
        line = eef_eval.brief("push", {"left": better, "right": ARM, **PATH})
        self.assertIn("abd_max 62.8 drop_min 0.051 tau 3.81", line)

    def test_failed_candidate_shows_where_it_failed(self):
        failed = {"abd_max": None, "elbow_drop_min": None, "tau_yaw_max": None, "ok": False,
                  "fail": "wp0: pose out of reach", "max_joint_travel": 0.0, "link_gap_min_per_wp": [],
                  "self_clearance_min_per_wp": []}
        line = eef_eval.brief("bad", failed)
        self.assertIn("NO ", line)
        self.assertIn("n_ok 0 wp0: pose out of reach", line)


if __name__ == "__main__":
    unittest.main()
