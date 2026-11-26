from __future__ import annotations

import datetime as dt
import json
import sys
import tempfile
import unittest
from pathlib import Path

SKILL_ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(SKILL_ROOT / "src"), str(SKILL_ROOT / "scripts")]

import collect_metrics  # noqa: E402


class MetricsTest(unittest.TestCase):
    def test_collects_deduplicated_cost_stages_and_temperature(self):
        start = dt.datetime(2026, 11, 18, 10, 0, tzinfo=dt.timezone.utc)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run_dir = root / "run"
            run_dir.mkdir()
            log = [
                {"t": start.timestamp(), "kind": "exec", "input": {"note": "lettuce"},
                 "result": {"ok": True, "seconds": 2.5, "results": [
                     {"executed": True}, {"gripper": "grasp", "executed": True, "opening": 0.04, "holding": True}
                 ]}},
                {"t": (start + dt.timedelta(minutes=2)).timestamp(), "kind": "exec",
                 "input": {"note": "slice 2"}, "result": {"ok": False, "seconds": 1.0, "results": []}},
                {"t": (start + dt.timedelta(minutes=3)).timestamp(), "kind": "both",
                 "input": {"note": "push plate: forward"}, "result": {"ok": True, "executed": True, "duration_s": 7.4}},
                {"t": (start + dt.timedelta(minutes=4)).timestamp(), "kind": "both",
                 "input": {"note": "push plate: lift"}, "result": {"ok": False, "executed": False}},
                {"t": (start + dt.timedelta(minutes=5)).timestamp(), "kind": "home", "input": ["both"],
                 "result": {"ok": True, "executed": True, "duration_s": 1.6}},
            ]
            (run_dir / "eef_log.jsonl").write_text(
                "\n".join(json.dumps(row) for row in log) + "\n", encoding="utf-8"
            )
            transcript = root / "transcript.jsonl"
            events = [
                {
                    "type": "assistant",
                    "timestamp": (start + dt.timedelta(seconds=30)).isoformat(),
                    "message": {
                        "id": "m1",
                        "model": "test",
                        "usage": {
                            "input_tokens": 1000,
                            "output_tokens": 100,
                            "cache_read_input_tokens": 500,
                            "cache_creation_input_tokens": 200,
                        },
                    },
                },
                {
                    "type": "assistant",
                    "timestamp": (start + dt.timedelta(seconds=31)).isoformat(),
                    "message": {
                        "id": "m1",
                        "model": "test",
                        "usage": {"input_tokens": 1000, "output_tokens": 100},
                    },
                },
                {
                    "type": "user",
                    "timestamp": (start + dt.timedelta(seconds=45)).isoformat(),
                    "message": {"content": [{"type": "text", "text": "temps_max_c 76"}]},
                },
            ]
            transcript.write_text("\n".join(json.dumps(row) for row in events) + "\n", encoding="utf-8")

            metrics = collect_metrics.collect(run_dir, transcript, "test request")

        self.assertEqual(metrics["stages"]["push plate"]["exec_calls"], 2)
        self.assertEqual(metrics["stages"]["push plate"]["wps_executed"], 1)
        self.assertEqual(metrics["stages"]["push plate"]["refused_calls"], 1)
        self.assertEqual(metrics["stages"]["push plate"]["robot_s"], 9.0)
        self.assertEqual(metrics["robot_s"], 12.5)
        self.assertEqual(metrics["model"]["calls"], 1)
        self.assertEqual(metrics["model"]["tokens"]["input"], 1000)
        self.assertEqual(metrics["motor_temp"]["max_c"], 76)
        self.assertEqual(metrics["stages"]["lettuce"]["wps_executed"], 1)
        self.assertEqual(metrics["stages"]["slice 2"]["refused_calls"], 1)


if __name__ == "__main__":
    unittest.main()
