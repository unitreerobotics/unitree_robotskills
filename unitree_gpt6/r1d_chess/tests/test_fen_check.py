"""Offline: fen_check.py validates structure/counts and draws the overlay."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT/'scripts/fen_check.py'
VIEW = ROOT/'scripts/board_view.py'
FRAME = ROOT/'tests/fixtures/head_low_view_p0.62.jpg'
START = 'rnbakabnr/9/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/9/RNBAKABNR w - - 0 1'


def run(*args):
    return subprocess.run([sys.executable, '-B', str(TOOL), *args], capture_output=True, text=True)


class FenCheckTests(unittest.TestCase):
    def test_start_position_ok(self):
        r = run(START)
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertIn('red 16 pieces, black 16 pieces', r.stdout)

    def test_bad_structure_and_counts(self):
        self.assertNotEqual(run('rnbakabnr/9/1c5c1 w - - 0 1').returncode, 0)          # too few ranks
        r = run('rnbakabnr/9/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/9/RNBAKAKNR w - - 0 1')  # two red kings
        self.assertNotEqual(r.returncode, 0)
        self.assertIn('K', r.stdout)

    @unittest.skipUnless(FRAME.exists(), 'recorded frame missing')
    def test_overlay(self):
        with tempfile.TemporaryDirectory() as out:
            subprocess.run([sys.executable, '-B', str(VIEW), '--image', str(FRAME), '--head-q', '0.624', '0.004',
                            '--output', out], check=True, capture_output=True)
            self.assertEqual(run(START, '--view', out).returncode, 0)
            self.assertGreater((Path(out)/'fen_overlay.jpg').stat().st_size, 10000)


if __name__ == '__main__':
    unittest.main()
