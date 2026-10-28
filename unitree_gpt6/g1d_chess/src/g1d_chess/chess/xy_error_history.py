"""Offline per-round delta-XYZ journal, planning references and final review.

No hardware imports. Errors are actual minus effective target, in base-frame mm.
Each round keeps its cumulative journal beside its final review in run/docs.
Pass the previous round journal to carry history across runs and games.
"""
import argparse
from datetime import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import time
from zoneinfo import ZoneInfo

from g1d_chess.paths import ROOT
JOURNAL_NAME = 'execution_xy_errors.jsonl'
CONFIG = ROOT / 'config/xy_error_learning.json'
PHASES = {'grasp_success': 'source', 'release_start': 'target'}
STAGES = {'grasp_success': 'source_contact_success',
          'release_start': 'target_low_release_approval'}
MAX_SAMPLE_MM = 25.0


def read_json(path):
    return json.loads(Path(path).read_text())


def encoded(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, allow_nan=False)


def identifier(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError('context/game identity must be nonempty')
    return value


def empty_stats():
    return {hand: {phase: {'count': 0, 'sum_delta_xyz_mm': [0., 0., 0.],
                          'mean_delta_xyz_mm': [0., 0., 0.]}
                   for phase in ('source', 'target')} for hand in ('left', 'right')}


def load_rows(stream):
    rows = []
    for number, line in enumerate(stream, 1):
        if not line.endswith('\n'):
            raise ValueError(f'journal has incomplete line {number}; do not silently discard it')
        row = json.loads(line)
        if row.get('schema_version') != 1 or row.get('kind') != 'round_delta_xyz_errors':
            raise ValueError(f'unsupported journal row {number}')
        rows.append(row)
    return rows


def cumulative(rows, context):
    stats = empty_stats()
    for row in rows:
        if row['context_id'] != context:
            continue
        for sample in row['samples']:
            if not sample['eligible']:
                continue
            cell = stats[sample['physical_actor']][sample['phase']]
            cell['count'] += 1
            delta = sample['delta_xyz_mm']
            if (not isinstance(delta, list) or len(delta) != 3
                    or not all(type(v) in (int, float) and math.isfinite(v) for v in delta)):
                raise ValueError('eligible historical deltaXYZ must contain three finite values')
            cell['sum_delta_xyz_mm'] = [a + b for a, b in zip(cell['sum_delta_xyz_mm'], delta)]
            cell['mean_delta_xyz_mm'] = [v / cell['count'] for v in cell['sum_delta_xyz_mm']]
    return stats


def latest(journal, context):
    journal = Path(journal)
    if not journal.exists():
        return [], empty_stats()
    with journal.open() as stream:
        fcntl.flock(stream, fcntl.LOCK_SH)
        rows = load_rows(stream)
    return rows, cumulative(rows, context)


def journal_for_run(run):
    return Path(run).resolve() / 'docs' / JOURNAL_NAME


def samples_for(report, visual):
    samples = []
    for record in report['records']:
        moment = record['moment']
        if moment not in PHASES:
            continue
        hand = record['physical_actor']
        if hand not in ('left', 'right'):
            raise ValueError('physical_actor must be left/right, never a logical arm field')
        error = record.get('errors', {}).get('vs_effective_target')
        delta = error.get('delta_xyz_mm') if error else None
        valid = (isinstance(delta, list) and len(delta) == 3
                 and all(type(v) in (int, float) and math.isfinite(v) for v in delta))
        reasons = []
        if report['outcome'] != 'placed' or visual.get('move_success') is not True:
            reasons.append('round_not_visually_successful')
        if record.get('event_stage') != STAGES[moment]:
            reasons.append('not_current_event_stage')
        if not valid:
            reasons.append('missing_or_invalid_effective_error')
        elif math.hypot(*delta) > MAX_SAMPLE_MM:
            reasons.append('delta_xyz_over_25mm_not_learned')
        samples.append({'moment': moment, 'phase': PHASES[moment], 'physical_actor': hand,
                        'delta_xyz_mm': delta if valid else None,
                        'eligible': not reasons, 'excluded_reasons': reasons})
    if len({s['moment'] for s in samples}) != len(samples):
        raise ValueError('duplicate checkpoint in main move report')
    return samples


def append_round(journal, payload, previous_journal=None):
    """Append one physical line with raw round reports AND both hands' cumulative stats.

    A flock serializes writers/readers. Replaying identical tokens is idempotent;
    conflicting metadata is rejected rather than learning a round twice.
    """
    identifier(payload['context_id']); identifier(payload['game_id'])
    if type(payload['round_number']) is not int or payload['round_number'] < 1:
        raise ValueError('round_number must be a positive integer')
    if not re.fullmatch(r'[^\s/\\_]+', payload['move_plan']):
        raise ValueError('move_plan must be compact notation without separators')
    reports = [payload['main_report'], *payload.get('auxiliary_reports', [])]
    tokens = [r['token'] for r in reports]
    if not all(isinstance(t, str) and t for t in tokens) or len(set(tokens)) != len(tokens):
        raise ValueError('missing or duplicate transaction tokens')
    fingerprint = hashlib.sha256(encoded(payload).encode()).hexdigest()
    samples = samples_for(payload['main_report'], payload['visual_review'])
    journal = Path(journal)
    inherited = []
    if previous_journal:
        previous_journal = Path(previous_journal).resolve()
        if previous_journal == journal.resolve():
            raise ValueError('previous journal must differ from current journal')
        if not previous_journal.is_file():
            raise ValueError('previous journal missing; do not silently reset history')
        inherited, _ = latest(previous_journal, payload['context_id'])
    journal.parent.mkdir(parents=True, exist_ok=True)
    with journal.open('a+', encoding='utf-8') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        stream.seek(0)
        rows = load_rows(stream)
        carry_history = not rows
        if carry_history:
            rows = inherited
        elif inherited and rows[:len(inherited)] != inherited:
            raise ValueError('current journal does not extend the specified previous history')
        for row in rows:
            if set(tokens) & set(row['tokens']):
                if row['payload_sha256'] == fingerprint:
                    return row
                raise ValueError('transaction already journaled with different review/context')
            if (row['game_id'], row['round_number']) == (payload['game_id'], payload['round_number']):
                # A failed, explicitly reviewed attempt may be retried without
                # inventing a new chess round. Keep its evidence, learn no sample twice.
                restored_retry = (
                    row['visual_review'].get('scene_restored_by_user') is True
                    and payload['visual_review'].get('restored_before_execution') is True)
                if ((row['main_report'].get('outcome') == 'placed' and not restored_retry)
                        or row['visual_review'].get('move_success') is not False
                        or any(sample['eligible'] for sample in row['samples'])):
                    raise ValueError('game round already journaled with different transaction tokens')
        now = datetime.now(ZoneInfo('Asia/Shanghai')).replace(microsecond=0)
        filename = f"第{payload['round_number']}轮_{payload['move_plan']}_{now:%Y%m%d_%H%M%S}.json"
        while (Path(payload['run']) / 'docs' / filename).exists():
            time.sleep(1)
            now = datetime.now(ZoneInfo('Asia/Shanghai')).replace(microsecond=0)
            filename = f"第{payload['round_number']}轮_{payload['move_plan']}_{now:%Y%m%d_%H%M%S}.json"
        row = {'schema_version': 1, 'kind': 'round_delta_xyz_errors', **payload,
               'review_created_at': now.isoformat(), 'review_filename': filename,
               'tokens': tokens, 'payload_sha256': fingerprint, 'samples': samples,
               'reference': 'actual_fk_minus_effective_target_delta_xyz_mm',
               'auxiliary_learning': 'excluded: offboard/removal transactions retained as evidence only'}
        row['cumulative'] = cumulative([*rows, row], payload['context_id'])
        stream.seek(0, os.SEEK_END)
        if carry_history:
            for previous_row in inherited:
                stream.write(encoded(previous_row) + '\n')
        stream.write(encoded(row) + '\n')
        stream.flush(); os.fsync(stream.fileno())
    return row


def reference_for_spec(spec, hand, journal=None, context=None, run=None, previous_journal=None):
    """Read the selected physical hand's history. Never modify candidate coordinates."""
    if hand not in ('left', 'right'):
        raise ValueError('invalid physical hand')
    context = identifier(context or spec.get('xy_error_context') or read_json(CONFIG)['context_id'])
    journal = Path(journal).resolve() if journal else journal_for_run(run or ROOT)
    previous = previous_journal or spec.get('xy_error_previous_journal')
    previous = Path(previous).resolve() if previous else None
    if previous and (previous == journal or not previous.is_file()):
        raise ValueError('previous journal must exist and differ from current journal')
    source = previous if previous and (not journal.exists() or journal.stat().st_size == 0) else journal
    rows, stats = latest(source, context)
    history = [r for r in rows if r['context_id'] == context]
    audit = {'journal': str(journal), 'context_id': context, 'physical_actor': hand,
             'history_source': str(source),
             'previous_journal': str(previous) if previous else None,
             'usage': 'reference_only_current_round_image_estimation_no_automatic_coordinate_change',
             'history_rows': len(history), 'history_sha256': hashlib.sha256(encoded(history).encode()).hexdigest(),
             'last_round': ({k: history[-1][k] for k in ('game_id', 'round_number', 'review_created_at')}
                            if history else None),
             'reference': 'actual_fk_minus_effective_target_delta_xyz_mm', 'phases': {}}
    for phase in ('source', 'target'):
        recent = None
        for row in reversed(history):
            sample = next((s for s in row['samples'] if s['eligible']
                           and s['physical_actor'] == hand and s['phase'] == phase), None)
            if sample:
                record = next(r for r in row['main_report']['records'] if r['moment'] == sample['moment'])
                recent = {'game_id': row['game_id'], 'round_number': row['round_number'],
                          'review_created_at': row['review_created_at'],
                          'actual_fk': record['actual_fk'], 'effective_target': record['effective_target'],
                          'delta_xyz_mm': sample['delta_xyz_mm']}
                break
        audit['phases'][phase] = {**stats[hand][phase], 'latest_eligible_event': recent}
    return audit


def finish(journal, run, game_id, round_number, move_plan, context, report_paths,
           visual_path, plan_summary=None, previous_journal=None):
    journal = Path(journal).resolve() if journal else journal_for_run(run)
    if journal != journal_for_run(run):
        raise ValueError('journal must be run/docs/execution_xy_errors.jsonl beside final review')
    reports = [read_json(p) for p in report_paths]
    plan = read_json(plan_summary) if plan_summary else {}
    reference = plan.get('xy_error_reference')
    if reference and Path(reference['journal']).resolve() != journal:
        raise ValueError('review journal differs from executed plan')
    planned_previous = reference.get('previous_journal') if reference else None
    if previous_journal and planned_previous and Path(previous_journal).resolve() != Path(planned_previous).resolve():
        raise ValueError('previous journal differs from executed plan')
    previous_journal = previous_journal or planned_previous
    if reference and reference['context_id'] != context:
        raise ValueError('review context differs from executed plan')
    for key, value in (('game_id', game_id), ('round_number', round_number), ('move_plan', move_plan)):
        if plan.get(key) is not None and plan[key] != value:
            raise ValueError(f'review {key} differs from executed plan')
    payload = {'game_id': game_id, 'round_number': round_number, 'move_plan': move_plan,
               'context_id': context, 'run': str(Path(run).resolve()),
               'source_square': plan.get('source_square'), 'target_square': plan.get('target_square'),
               'main_report': reports[0], 'auxiliary_reports': reports[1:],
               'report_paths': [str(Path(p).resolve()) for p in report_paths],
               'visual_review': read_json(visual_path), 'planning_error_reference': reference,
               'previous_journal': str(Path(previous_journal).resolve()) if previous_journal else None}
    row = append_round(journal, payload, previous_journal)
    out = Path(run) / 'docs' / row['review_filename']
    result = {**reports[0], 'game_id': game_id, 'round_number': round_number,
              'move_plan': move_plan, 'review_created_at': row['review_created_at'],
              'source_square': payload['source_square'], 'target_square': payload['target_square'],
              'visual_review': payload['visual_review'], 'auxiliary_reports': reports[1:],
              'planning_error_reference': reference, 'report_paths': payload['report_paths'],
              'xy_error_history': {'path': str(Path(journal).resolve()),
                                   'payload_sha256': row['payload_sha256'], 'context_id': context,
                                   'samples': row['samples'], 'cumulative': row['cumulative']}}
    out.parent.mkdir(parents=True, exist_ok=True)
    content = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    # Same row/time on retry: recover a crash between journal append and review write.
    if out.exists():
        if read_json(out) != result:
            raise ValueError('review filename occupied by different content; never overwrite')
    else:
        with out.open('x', encoding='utf-8') as stream:
            stream.write(content)
    return {'review': str(out.resolve()), 'journal': str(Path(journal).resolve()),
            'cumulative': row['cumulative'], 'robot_calls': 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    p = sub.add_parser('show')
    p.add_argument('--context-id', required=True)
    p.add_argument('--hand', choices=('left', 'right'), required=True)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--previous-journal', type=Path,
                   help='previous completed round docs/execution_xy_errors.jsonl')
    p.add_argument('--output', type=Path, help='optional planning reference snapshot; exclusive create')
    p = sub.add_parser('finish')
    for name in ('run', 'game-id', 'move-plan', 'context-id', 'visual-review'):
        p.add_argument('--' + name, required=True)
    p.add_argument('--round-number', type=int, required=True)
    p.add_argument('--report', action='append', required=True,
                   help='own move FK report first; repeat for capture-removal auxiliary reports')
    p.add_argument('--plan-summary')
    p.add_argument('--previous-journal', type=Path,
                   help='defaults to previous_journal recorded in plan summary')
    args = parser.parse_args()
    if args.action == 'show':
        result = reference_for_spec({}, args.hand, context=args.context_id,
                                    run=args.run, previous_journal=args.previous_journal)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open('x', encoding='utf-8') as stream:
                stream.write(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    else:
        result = finish(None, args.run, args.game_id, args.round_number, args.move_plan,
                        args.context_id, args.report, args.visual_review, args.plan_summary,
                        args.previous_journal)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
