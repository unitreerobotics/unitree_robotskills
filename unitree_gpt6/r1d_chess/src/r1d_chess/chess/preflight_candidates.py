#!/usr/bin/env python3
"""Try complete requests in chess rank order; prefer source-side arm, then the other."""
import argparse
import json
from pathlib import Path
import sys
from r1d_chess.robot.r1d_config import ROOT
from r1d_chess.robot.robot_client import send_request


def select(candidates, endpoint):
    attempts=[]
    for candidate in candidates:
        source=candidate['source_square']
        order=['left','right'] if source[0] in 'abcd' else ['right','left']
        if set(candidate['requests']) != {'left','right'}:
            raise ValueError('each candidate requires independently built complete left AND right requests')
        for arm in order:
            path=Path(candidate['requests'][arm]).resolve()
            request=json.loads(path.read_text())
            if request.get('expected_active_arm') != arm:
                raise ValueError('candidate request actor mismatch')
            is_sequence='transactions' in request
            response,_=send_request({'cmd':'set_active_arm','arm':arm},endpoint,1000,5000)
            if not response.get('ok'):
                raise RuntimeError('cannot select actor: '+str(response))
            response,_=send_request({'cmd':'prepare_sequence' if is_sequence else 'prepare_transaction',**request},endpoint,1000,180000)
            attempts.append({'candidate':candidate.get('move'),'arm':arm,'request':str(path),
                             'ok':response.get('ok'),'error':response.get('error')})
            if response.get('ok'):
                prepared=path.parent/'prepared.json'
                with prepared.open('x') as f:json.dump(response,f,ensure_ascii=False,indent=2)
                return {'selected':candidate.get('move'),'arm':arm,'prepared':str(prepared),
                        'request':str(path),'attempts':attempts,'executed':False}
            if response.get('request_sent') and response.get('error')=='response_timeout':
                raise RuntimeError('preflight response timed out; inspect original service before sending more requests')
    return {'selected':None,'attempts':attempts,'executed':False,
            'reason':'all supplied candidates failed complete preflight with both hands; search next legal candidate'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidates',type=Path,required=True)
    p.add_argument('--endpoint',default='ipc:///tmp/astra_r1d_chess.sock')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    result=select(json.loads(a.candidates.read_text()),a.endpoint)
    with a.output.open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
