"""Offline: board_points.py measures the grid from two recorded head poses consistently."""
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT/'scripts/board_points.py'
LOW = ROOT/'tests/fixtures/head_low_view_p0.62.jpg'
REF = ROOT/'tests/fixtures/head_calibration_stereo.jpg'
# The recorded frames were taken with the board at the position of this fixture geometry.
GEOMETRY = ROOT/'tests/fixtures/board_geometry_fixture.json'


@unittest.skipUnless(LOW.exists() and REF.exists(), 'recorded frames missing')
class BoardPointsTests(unittest.TestCase):
    def test_two_views_agree(self):
        with tempfile.TemporaryDirectory() as d:
            res = {}
            for name, img, q in (('low', LOW, ('0.624', '0.004')), ('ref', REF, ('0.3439', '0.0043'))):
                subprocess.run([sys.executable, '-B', str(TOOL), '--image', str(img), '--head-q', *q, '--geometry', str(GEOMETRY),
                                '--output', f'{d}/{name}'], check=True, capture_output=True)
                res[name] = json.loads(Path(f'{d}/{name}/board_points.json').read_text())
            self.assertEqual(len(res['low']['points']), 90)
            diff = [math.dist(res['low']['points'][k]['xyz_waist_m'][:2], res['ref']['points'][k]['xyz_waist_m'][:2])
                    for k in res['low']['points']]
            self.assertLess(max(diff), .005)
            self.assertGreater(res['low']['summary']['measured_points'], 50)


if __name__ == '__main__':
    unittest.main()
