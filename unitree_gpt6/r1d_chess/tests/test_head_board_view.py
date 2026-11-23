"""Offline: board_view.py renders all outputs from a recorded head frame."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
FRAME = ROOT/'tests/fixtures/head_low_view_p0.62.jpg'


@unittest.skipUnless(FRAME.exists() and (ROOT/'config/head_camera_calibration.json').exists(), 'recorded frame missing')
class HeadBoardViewTests(unittest.TestCase):
    def test_outputs(self):
        with tempfile.TemporaryDirectory() as out:
            subprocess.run([sys.executable, '-B', str(ROOT/'scripts/board_view.py'), '--image', str(FRAME),
                            '--head-q', '0.624', '0.004', '--output', out], check=True, capture_output=True)
            for name in ('board_topdown.jpg', 'board_tiles.jpg', 'undistorted_left.jpg', 'undistorted_right.jpg'):
                self.assertGreater((Path(out)/name).stat().st_size, 10000, name)
            meta = json.loads((Path(out)/'board_view.json').read_text())
            self.assertEqual(len(meta['topdown_intersection_px']), 90)
            self.assertIn('planning reference only', meta['purpose'])


if __name__ == '__main__':
    unittest.main()
