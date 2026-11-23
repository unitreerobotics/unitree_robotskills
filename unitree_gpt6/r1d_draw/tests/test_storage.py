"""Atomic JSON writes and exclusive controller ownership."""

import json
import math
import multiprocessing
import tempfile
import unittest
from pathlib import Path

from unitree_gpt6.r1d_draw.src.storage import ControllerLease, atomic_json, read_json


def _try_lock(path, queue):
    try:
        with ControllerLease([path]):
            queue.put("acquired")
    except RuntimeError as exc:
        queue.put(str(exc))


class AtomicJsonTests(unittest.TestCase):
    def test_round_trip_creates_parent_and_keeps_unicode(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "a" / "b" / "status.json"
            atomic_json(path, {"状态": "完成", "n": 1.5})
            self.assertEqual(read_json(path), {"状态": "完成", "n": 1.5})
            self.assertIn("状态", path.read_text(encoding="utf-8"))
            self.assertEqual([p.name for p in path.parent.iterdir()], ["status.json"])

    def test_failed_write_keeps_the_previous_file_and_leaves_no_temporary(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "status.json"
            atomic_json(path, {"step": 1})
            for bad in ({"x": math.nan}, {"x": {1, 2}}):
                with self.subTest(bad=bad), self.assertRaises((ValueError, TypeError)):
                    atomic_json(path, bad)
            self.assertEqual(json.loads(path.read_text()), {"step": 1})
            self.assertEqual([p.name for p in Path(folder).iterdir()], ["status.json"])


class ControllerLeaseTests(unittest.TestCase):
    def other_process(self, path):
        queue = multiprocessing.Queue()
        process = multiprocessing.Process(target=_try_lock, args=(str(path), queue))
        process.start()
        process.join(10)
        return queue.get(timeout=5)

    def test_second_controller_is_refused_until_release(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "locks" / "controller.lock"
            with ControllerLease([path]):
                self.assertIn("another controller owns", self.other_process(path))
            self.assertEqual(self.other_process(path), "acquired")

    def test_partial_acquisition_releases_what_it_took(self):
        with tempfile.TemporaryDirectory() as folder:
            first, second = Path(folder) / "a.lock", Path(folder) / "b.lock"
            with ControllerLease([second]):
                with self.assertRaises(RuntimeError):
                    ControllerLease([first, second]).__enter__()
                self.assertEqual(self.other_process(first), "acquired")


if __name__ == "__main__":
    unittest.main()
