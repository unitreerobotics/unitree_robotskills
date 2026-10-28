import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

"""Offline synthetic-image checks; contains no real board measurements."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import cv2
import numpy as np

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/mark_board.py'


class BoardAnnotationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.image = self.root / 'synthetic.png'
        canvas = np.full((480, 640, 3), 245, np.uint8)
        for rank in range(10):
            cv2.line(canvas, (80, 400-28*rank), (320, 400-28*rank), (0, 0, 0), 1)
        for file in range(9):
            cv2.line(canvas, (80+30*file, 148), (80+30*file, 400), (0, 0, 0), 1)
        cv2.imwrite(str(self.image), canvas)
        self.controls = self.root / 'controls.json'
        self.call('init', '--image', self.image, '--output', self.controls)
        self.data = json.loads(self.controls.read_text())
        self.data['orientation_confirmed'] = True
        self.data['points'] = [self.point(s, usage) for usage, squares in (
            ('fit', ['a0','i0','a4','i4','e1','e3','a5','i5','a9','i9','e6','e8']),
            ('holdout', ['c2','g3','c6','g7'])) for s in squares]

    def point(self, square, usage):
        return {'square': square, 'pixel_xy': [80+30*(ord(square[0])-97), 400-28*int(square[1])],
                'usage': usage, 'evidence': 'visible_board_line_intersection'}

    def call(self, *args, expected=0):
        result = subprocess.run([sys.executable, str(SCRIPT), *map(str, args)], capture_output=True, text=True)
        self.assertEqual(result.returncode, expected, result.stdout+result.stderr)
        return result

    def build(self, data=None, mapping='global', expected=0):
        self.controls.write_text(json.dumps(data or self.data))
        return self.call('build', '--controls', self.controls, '--output-dir', self.root/'result',
                         '--mapping', mapping, '--max-error-px', .2, expected=expected)

    def test_empty_init_has_no_measurements(self):
        data = json.loads(self.controls.read_text())
        self.assertEqual(data['points'], [])
        self.assertFalse(data['orientation_confirmed'])
        self.assertEqual(data['source_sha256'], hashlib.sha256(self.image.read_bytes()).hexdigest())
        self.assertEqual(data['point_format']['pixel_xy'], [None, None])

    def test_global_90_points_query_and_overlay(self):
        self.build()
        path = self.root/'result/board_points_pixels.json'
        data = json.loads(path.read_text())
        self.assertEqual(len({p['square'] for p in data['points']}), 90)
        self.assertEqual(data['status'], 'pending_visual_review')
        self.assertIsNone(data['robot_base_transform'])
        self.assertEqual(cv2.imread(str(self.root/'result/board_points_annotated.png')).shape, (480,640,3))
        p = json.loads(self.call('query','d7','--reference',path).stdout)['point']
        np.testing.assert_allclose(p['original_image_pixel_xy'], [170,204], atol=1e-5)
        self.assertEqual(p['evidence_status'], 'inferred')

    def test_split_independent_fits(self):
        self.build(mapping='split')
        d = json.loads((self.root/'result/board_points_pixels.json').read_text())
        self.assertEqual(set(d['validation']['metrics']), {'near','far'})
        self.assertTrue(d['validation']['numeric_pass'])

    def test_holdout_error_is_not_hidden(self):
        d=copy.deepcopy(self.data)
        d['points'][-1]['pixel_xy'][0] += 10
        self.build(d, expected=2)
        result=json.loads((self.root/'result/board_points_pixels.json').read_text())
        self.assertEqual(result['status'], 'needs_reannotation')
        self.assertGreater(result['validation']['metrics']['global']['holdout']['max_px'], 9)

    def test_reject_duplicate(self):
        d=copy.deepcopy(self.data);d['points'].append(d['points'][0])
        result=self.build(d, expected=2)
        self.assertIn('重复交点',result.stderr)
        self.assertFalse((self.root/'result').exists())

    def test_reject_insufficient_observations(self):
        d=copy.deepcopy(self.data);d['points']=d['points'][:5]
        self.assertIn('至少6个', self.build(d,expected=2).stderr)

    def test_reject_changed_image(self):
        self.image.write_bytes(self.image.read_bytes()+b'changed')
        self.assertIn('哈希已变化',self.build(expected=2).stderr)

    def test_reject_unconfirmed_direction(self):
        d=copy.deepcopy(self.data);d['orientation_confirmed']=False
        self.assertIn('人工确认方向',self.build(d,expected=2).stderr)

    def test_reject_null_pixel(self):
        d=copy.deepcopy(self.data);d['points'][0]['pixel_xy']=[None,None]
        self.assertIn('有限像素',self.build(d,expected=2).stderr)

    def test_reject_collinear_controls(self):
        d=copy.deepcopy(self.data)
        d['points']=[self.point(f'{f}0','fit') for f in 'abcdef']+[self.point('g1','holdout'),self.point('h2','holdout')]
        self.assertIn('共线',self.build(d,expected=2).stderr)


if __name__ == '__main__':
    unittest.main()
