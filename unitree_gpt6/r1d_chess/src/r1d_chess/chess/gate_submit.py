#!/usr/bin/env python3
"""Write the evaluation for the current serial gate and submit it at once (`bash r1d gate`).

usage:
  bash r1d gate --prepared <run>/tmp/stage_1_prepared.json --checkpoint <checkpoint.json> \\
      --decision success|adjust|adjust_pitch|retry_high|failure --reason "<两图依据>" [--offset DX DY]

The stage template is chosen from the checkpoint: pre_release uses the 7-field release evaluation,
source high / preclose the 6-field source evaluation. Evidence = the checkpoint's own two images.
--offset (metres, base frame, relative to the checkpoint's actual FK) is required for `adjust`.
The gate decision is the executing assistant's responsibility; see prompts/round_plan.md.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time

from r1d_chess.paths import ROOT


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--prepared', required=True, type=Path)
    p.add_argument('--checkpoint', required=True, type=Path)
    p.add_argument('--decision', required=True,
                   choices=('success', 'adjust', 'adjust_pitch', 'retry_high', 'failure'))
    p.add_argument('--reason', required=True)
    p.add_argument('--offset', type=float, nargs=2, metavar=('DX', 'DY'), default=(0., 0.))
    p.add_argument('--approve-final', action='store_true',
                   help='pre_release adjust on the last correction: the offset comes from an image measurement and '
                        'the service releases right after it, without another gate')
    a = p.parse_args()
    c = json.loads(a.checkpoint.read_text())
    offset = [float(v) for v in a.offset]
    if (a.decision == 'adjust') != (offset != [0., 0.]):
        raise SystemExit('adjust needs a nonzero --offset; other decisions must not give one')
    evidence = [c['observation']['paths'][k] for k in c['camera_order']]
    if c.get('stage') == 'pre_release':
        ev = {'decision': a.decision, 'actual_relative_offset_xy_m': offset, 'reason': a.reason,
              'target_identified': a.decision != 'failure',
              'release_alignment_verified': a.decision == 'success' or (a.decision == 'adjust' and a.approve_final),
              'evidence': evidence, 'obvious_large_offset': a.decision == 'adjust'}
    else:
        ev = {'decision': a.decision, 'actual_relative_offset_xy_m': offset, 'reason': a.reason,
              'source_identified': a.decision not in ('failure',),
              'descent_alignment_verified': a.decision == 'success', 'evidence': evidence}
    out = a.prepared.parent/f"eval_{c.get('stage', 'pregrasp')}_{int(time.time())}.json"
    out.write_text(json.dumps(ev, ensure_ascii=False, indent=1))
    r = subprocess.run(['bash', str(ROOT/'r1d'), 'serial', 'submit', '--prepared', str(a.prepared),
                        '--checkpoint', str(a.checkpoint), '--evaluation', str(out)], capture_output=True, text=True)
    print(json.dumps({'evaluation': str(out), 'returncode': r.returncode,
                      'stdout': r.stdout[-600:], 'stderr': r.stderr[-600:]}, ensure_ascii=False))
    raise SystemExit(r.returncode)


if __name__ == '__main__':
    main()
