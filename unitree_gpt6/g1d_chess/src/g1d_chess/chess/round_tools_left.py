"""Left actor planner entry; same init/build/tokens/report workflow as right."""
import argparse
import hashlib
import json
from g1d_chess.chess import round_tools as common

_transaction = common.transaction


def left_transaction(state, source, target, run, stage, cfg, **kwargs):
    cfg = {**cfg, 'init_xyz_rpy': common.read(common.ROOT/'config/init_state.json')['left_arm']['xyz_rpy']}
    return _transaction(state, source, target, run, stage, cfg, **kwargs)


def build(run, spec, observation):
    state = common.read(observation)['state']
    if state.get('active_arm') != 'left' or state.get('arm_field_frame') != 'transaction_roles':
        raise ValueError('left planner requires a fresh observation from the left service')
    previous = common.transaction
    common.transaction = left_transaction
    try:
        summary = common.build(run, spec, observation)
    finally:
        common.transaction = previous
    root = common.within_run(run)
    request = common.read(root/'tmp/request.json')
    request['expected_active_arm'] = 'left'
    (root/'tmp/request.json').write_text(json.dumps(request, ensure_ascii=False, indent=2)+'\n')
    summary.update(active_arm='left', actor_plan_field='right_arm', observer_plan_field='left_arm',
                   request_sha256=hashlib.sha256((root/'tmp/request.json').read_bytes()).hexdigest())
    (root/'tmp/plan_summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    return summary


def tokens(run, prepared):
    if common.read(prepared).get('active_arm') != 'left':
        raise ValueError('left planner refuses another arm service token')
    result = common.tokens(run, prepared)
    path = common.within_run(run)/'tmp/execute_request.json'
    request = {**common.read(path), 'expected_active_arm': 'left'}
    path.write_text(json.dumps(request, ensure_ascii=False, indent=2)+'\n')
    return {**result, 'active_arm': 'left'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    p = sub.add_parser('init'); p.add_argument('--run', required=True)
    p = sub.add_parser('build')
    for name in ('run', 'spec', 'observation'): p.add_argument('--'+name, required=True)
    p = sub.add_parser('tokens'); p.add_argument('--run', required=True); p.add_argument('--prepared', required=True)
    p = sub.add_parser('report'); p.add_argument('--result', required=True)
    args = parser.parse_args()
    if args.action == 'init': result = common.initialize(args.run)
    elif args.action == 'build': result = build(args.run, args.spec, args.observation)
    elif args.action == 'tokens': result = tokens(args.run, args.prepared)
    else: result = {**common.report(args.result), 'active_arm': 'left', 'final_right_arm_field_means': 'physical_left_actor'}
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
