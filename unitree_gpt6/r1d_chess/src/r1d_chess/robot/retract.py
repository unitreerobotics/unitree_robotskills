#!/usr/bin/env python3
"""Emergency / on-request retract: replay the last execute's executed joint path in reverse, no new preflight.

Use when an uncaught error stopped the arm, or the operator asks the arm to return. The chess service must
be running (it holds the executed path in memory). The gripper is not changed. A latched fault is cleared
for this retract; motor faults, feedback loss and the 0.20 rad tracking limit still stop it.
"""
import argparse
import json
import sys
from pathlib import Path

from r1d_chess.paths import ROOT
from r1d_chess.robot.robot_client import send_request


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--endpoint', default='ipc:///tmp/astra_r1d_chess.sock')
    p.add_argument('--timeout-s', type=float, default=300.)
    p.add_argument('--save-response')
    a = p.parse_args()
    response, _ = send_request({'cmd': 'retract'}, a.endpoint, 2000, int(a.timeout_s*1000))
    if a.save_response:
        Path(a.save_response).write_text(json.dumps(response, ensure_ascii=False, indent=2))
    summary = {k: response.get(k) for k in ('ok', 'retracted', 'segments', 'error', 'message')}
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    sys.exit(0 if response.get('ok') and response.get('retracted') else 1)


if __name__ == '__main__':
    main()
