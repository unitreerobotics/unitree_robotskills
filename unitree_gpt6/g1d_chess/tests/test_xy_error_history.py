import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

"""Offline journal/reference regressions; no robot or kinematics imports."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
from g1d_chess.chess import xy_error_history as history


def report(token, hand='left', source=(8., -4., 6.), target=(2., -1., 3.)):
    records = []
    for moment, delta in (('grasp_success', source), ('release_start', target)):
        records.append({'moment': moment, 'physical_actor': hand,
                        'event_stage': history.STAGES[moment],
                        'actual_fk': {'x': .3 + delta[0]/1000, 'y': .2 + delta[1]/1000,
                                      'z': .1 + delta[2]/1000},
                        'effective_target': {'x': .3, 'y': .2, 'z': .1},
                        'errors': {'vs_effective_target': {'delta_xyz_mm': list(delta)},
                                   'vs_original_plan': {'delta_xyz_mm': [100., 100., 0.]}}})
    return {'token': token, 'outcome': 'placed', 'records': records}


class HistoryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.run = Path(self.tmp.name)
        self.journal = self.run / 'docs' / 'execution_xy_errors.jsonl'
        self.next_round = 1

    def tearDown(self):
        self.tmp.cleanup()

    def payload(self, token, hand='left', context='setup-a', **kwargs):
        number = self.next_round
        self.next_round += 1
        return {'context_id': context, 'game_id': 'game-a', 'run': str(self.run),
                'round_number': number, 'move_plan': '马八进七',
                'main_report': report(token, hand, **kwargs), 'visual_review': {'move_success': True}}

    def test_mean_is_not_sum_and_hands_phases_stay_separate(self):
        history.append_round(self.journal, self.payload('a'))
        history.append_round(self.journal, self.payload('b', source=(4., -2., 2.), target=(4., -3., 5.)))
        row = history.append_round(self.journal, self.payload('c', 'right', source=(-3., 5., -2.)))
        self.assertEqual(row['cumulative']['left']['source'],
                         {'count': 2, 'sum_delta_xyz_mm': [12., -6., 8.],
                          'mean_delta_xyz_mm': [6., -3., 4.]})
        self.assertEqual(row['cumulative']['left']['target']['mean_delta_xyz_mm'], [3., -2., 4.])
        self.assertEqual(row['cumulative']['right']['source']['mean_delta_xyz_mm'], [-3., 5., -2.])
        self.assertEqual(len(self.journal.read_text().splitlines()), 3)
        self.assertEqual(row['samples'][0]['delta_xyz_mm'], [-3., 5., -2.])

    def test_failed_round_retry_preserves_history_and_round_number(self):
        failed = self.payload('failed')
        failed['main_report'].update(outcome='alignment_failed_returned', records=[])
        failed['visual_review']['move_success'] = False
        history.append_round(self.journal, failed)
        retry = self.payload('retry'); retry['round_number'] = 1
        row = history.append_round(self.journal, retry)
        self.assertEqual(len(self.journal.read_text().splitlines()), 2)
        self.assertEqual(row['round_number'], 1)
        self.assertEqual(row['cumulative']['left']['source']['count'], 1)
        duplicate = self.payload('another'); duplicate['round_number'] = 1
        with self.assertRaises(ValueError):
            history.append_round(self.journal, duplicate)

    def test_misplaced_restored_retry_keeps_failed_history(self):
        failed = self.payload('misplaced')
        failed['visual_review'].update(move_success=False, scene_restored_by_user=True)
        history.append_round(self.journal, failed)
        old_content = self.journal.read_bytes()
        retry = self.payload('restored-retry'); retry['round_number'] = 1
        next_journal = self.run / 'retry' / 'docs' / 'execution_xy_errors.jsonl'
        with self.assertRaisesRegex(ValueError, 'game round already journaled'):
            history.append_round(next_journal, retry, self.journal)
        retry['visual_review']['restored_before_execution'] = True
        row = history.append_round(next_journal, retry, self.journal)
        self.assertEqual(self.journal.read_bytes(), old_content)
        self.assertEqual(len(next_journal.read_text().splitlines()), 2)
        self.assertEqual(row['cumulative']['left']['source']['count'], 1)
        self.assertEqual(history.append_round(next_journal, retry, self.journal), row)
        duplicate = self.payload('duplicate'); duplicate['round_number'] = 1
        duplicate['visual_review']['restored_before_execution'] = True
        with self.assertRaisesRegex(ValueError, 'game round already journaled'):
            history.append_round(next_journal, duplicate, self.journal)

    def test_reference_never_changes_xyz_and_uses_selected_hand(self):
        history.append_round(self.journal, self.payload('a'))
        spec = {'source': {'xy_m': [.3, .2], 'z_m': .1},
                'target': {'xy_m': [.4, .1], 'z_m': .12}}
        before = copy.deepcopy(spec)
        ref = history.reference_for_spec(spec, 'left', self.journal, 'setup-a')
        self.assertEqual(spec, before)
        self.assertEqual(ref['usage'], 'reference_only_current_round_image_estimation_no_automatic_coordinate_change')
        self.assertEqual(ref['phases']['source']['mean_delta_xyz_mm'], [8., -4., 6.])
        self.assertEqual(ref['phases']['source']['latest_eligible_event']['effective_target'],
                         {'x': .3, 'y': .2, 'z': .1})
        right = history.reference_for_spec(spec, 'right', self.journal, 'setup-a')
        self.assertEqual(right['phases']['source']['count'], 0)
        self.assertIsNone(right['phases']['source']['latest_eligible_event'])

    def test_contexts_and_games(self):
        history.append_round(self.journal, self.payload('a'))
        other = self.payload('b', context='setup-b', source=(1., 1., 1.))
        history.append_round(self.journal, other)
        new_game = self.payload('c', source=(4., -2., 2.)); new_game['game_id'] = 'game-b'
        row = history.append_round(self.journal, new_game)
        self.assertEqual(row['cumulative']['left']['source']['count'], 2)
        self.assertEqual(history.latest(self.journal, 'setup-b')[1]['left']['source']['count'], 1)

    def test_old_stage_failed_visual_and_extreme_samples_not_learned(self):
        p = self.payload('a', source=(0., 0., 26.))
        p['main_report']['records'][1]['event_stage'] = 'target_high_release_approval'
        row = history.append_round(self.journal, p)
        self.assertEqual(row['cumulative']['left']['source']['count'], 0)
        self.assertEqual(row['cumulative']['left']['target']['count'], 0)
        self.assertIn('delta_xyz_over_25mm_not_learned', row['samples'][0]['excluded_reasons'])
        p = self.payload('b'); p['visual_review']['move_success'] = False
        row = history.append_round(self.journal, p)
        self.assertFalse(any(s['eligible'] for s in row['samples']))

    def test_mean_uses_displacements_not_actual_coordinates(self):
        history.append_round(self.journal, self.payload('a'))
        p = self.payload('b', source=(4., -2., 2.))
        record = p['main_report']['records'][0]
        record['effective_target'] = {'x': .5, 'y': -.1, 'z': .14}
        record['actual_fk'] = {'x': .504, 'y': -.102, 'z': .142}
        history.append_round(self.journal, p)
        ref = history.reference_for_spec({}, 'left', self.journal, 'setup-a')
        self.assertEqual(ref['phases']['source']['mean_delta_xyz_mm'], [6., -3., 4.])
        self.assertEqual(ref['phases']['source']['count'], 2)

    def test_missing_journal_is_no_data_and_does_not_create_file(self):
        ref = history.reference_for_spec({}, 'right', self.journal, 'setup-a')
        self.assertFalse(self.journal.exists())
        for phase in ('source', 'target'):
            self.assertEqual(ref['phases'][phase]['count'], 0)
            self.assertEqual(ref['phases'][phase]['mean_delta_xyz_mm'], [0., 0., 0.])

    def test_idempotence_conflicting_token_and_incomplete_journal(self):
        p = self.payload('a')
        first = history.append_round(self.journal, p)
        self.assertEqual(history.append_round(self.journal, p), first)
        self.assertEqual(len(self.journal.read_text().splitlines()), 1)
        p['round_number'] = 2
        with self.assertRaises(ValueError):
            history.append_round(self.journal, p)
        self.journal.write_text(self.journal.read_text() + '{')
        with self.assertRaises(ValueError):
            history.latest(self.journal, 'setup-a')

    def test_missing_phase_and_auxiliary_exclusion(self):
        p = self.payload('a')
        p['main_report']['records'].pop()
        p['auxiliary_reports'] = [report('victim', 'right')]
        row = history.append_round(self.journal, p)
        self.assertEqual(row['cumulative']['left']['source']['count'], 1)
        self.assertEqual(row['cumulative']['left']['target']['count'], 0)
        self.assertEqual(row['cumulative']['right']['source']['count'], 0)
        self.assertEqual(row['tokens'], ['a', 'victim'])

    def test_round_cannot_be_learned_twice_with_new_tokens_or_context(self):
        history.append_round(self.journal, self.payload('a'))
        p = self.payload('b', context='setup-b')
        p['round_number'] = 1
        with self.assertRaisesRegex(ValueError, 'game round already journaled'):
            history.append_round(self.journal, p)
        self.assertEqual(len(self.journal.read_text().splitlines()), 1)

    def test_finish_named_review_and_retry_recover_without_extra_row(self):
        rp = self.run / 'fk.json'; rp.write_text(json.dumps(report('a')))
        vp = self.run / 'visual.json'; vp.write_text('{"move_success":true}')
        args = (self.journal, self.run, 'game-a', 5, '马八进七', 'setup-a', [rp], vp)
        result = history.finish(*args)
        out = Path(result['review'])
        self.assertRegex(out.name, r'^第5轮_马八进七_\d{8}_\d{6}\.json$')
        merged = json.loads(out.read_text())
        self.assertTrue(merged['review_created_at'].endswith('+08:00'))
        self.assertEqual(Path(result['journal']).parent, out.parent)
        out.unlink()
        self.assertEqual(history.finish(*args)['review'], result['review'])
        self.assertEqual(len(self.journal.read_text().splitlines()), 1)

    def test_next_run_reads_previous_and_carries_history_beside_review(self):
        history.append_round(self.journal, self.payload('a'))
        old_content = self.journal.read_bytes()
        next_run = self.run / 'round2'
        next_run.mkdir()
        spec = {'xy_error_previous_journal': str(self.journal)}
        reference = history.reference_for_spec(spec, 'left', context='setup-a', run=next_run)
        next_journal = next_run / 'docs' / 'execution_xy_errors.jsonl'
        self.assertEqual(reference['journal'], str(next_journal))
        self.assertEqual(reference['phases']['source']['mean_delta_xyz_mm'], [8., -4., 6.])
        self.assertFalse(next_journal.exists())
        rp = next_run / 'fk.json'
        rp.write_text(json.dumps(report('b', source=(4., -2., 2.))))
        vp = next_run / 'visual.json'; vp.write_text('{"move_success":true}')
        pp = next_run / 'plan.json'; pp.write_text(json.dumps({'xy_error_reference': reference}))
        args = (None, next_run, 'game-a', 2, '马八进七', 'setup-a', [rp], vp, pp)
        result = history.finish(*args)
        self.assertEqual(Path(result['review']).parent, next_journal.parent)
        self.assertEqual(result['cumulative']['left']['source']['mean_delta_xyz_mm'], [6., -3., 4.])
        self.assertEqual(len(next_journal.read_text().splitlines()), 2)
        self.assertEqual(history.finish(*args), result)
        self.assertEqual(len(next_journal.read_text().splitlines()), 2)
        self.assertEqual(self.journal.read_bytes(), old_content)

    def test_missing_previous_history_is_not_silently_reset(self):
        with self.assertRaisesRegex(ValueError, 'previous journal'):
            history.reference_for_spec({'xy_error_previous_journal': str(self.run / 'missing.jsonl')},
                                       'left', context='setup-a', run=self.run)

    def test_finish_refuses_journal_outside_review_directory(self):
        with self.assertRaisesRegex(ValueError, 'beside final review'):
            history.finish(self.run / 'calibration' / 'errors.jsonl', self.run,
                           'game-a', 1, '马八进七', 'setup-a', [], self.run / 'unused.json')


if __name__ == '__main__':
    unittest.main()
