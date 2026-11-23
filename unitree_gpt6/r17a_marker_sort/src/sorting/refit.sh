#!/bin/bash
# Re-photograph the box from its three views and refit the slot lattice; keep the old fit and fail if the new one is not sane.
set -o pipefail; R=$1; cd "$(dirname "$0")/.."      # src/
cp $R/box.json $R/box_prev.json && python3 -u -c "
import sys, json
from robot import rt
from sorting import task
from robot.rt import *
from sorting.task import set_box
sys.stdout = EventLog('$R/events.jsonl', 'box views (refit)')
box=json.load(open('$R/box.json')); set_box(box); rt.OBSTACLES[0]=(rt.OBSTACLES[0][0]-[0.02,0.02,0], rt.OBSTACLES[0][1]+[0.02,0.02,0.02])
s=state(); assert s['armed'] and not s['fault'] and not s['busy'], s['fault']
if s['gripper']['opening_fraction']<0.95: gripper(1.0)
for i,v in enumerate(json.load(open('$R/box_views.json'))[:3]):
    q=np.array(state()['q']); assert path_clear(q,np.array(v),0.12), 'path to view %d'%i
    move(np.array(v),0.35); snap('$R/boxv%d'%i); print('box top view %d taken'%i)
" 2>&1 | grep -v warn || exit 1
python3 -c "
import json; b=json.load(open('$R/box_prev.json')); P=b['P'][0]; a=b['a']; bb=b['b']; print(P[0],P[1],P[2],a[0],a[1],bb[0],bb[1])" > $R/x0.txt
python3 -m perception.box $R/box.json $(cat $R/x0.txt) $R/boxv0 $R/boxv1 $R/boxv2 || { cp $R/box_prev.json $R/box.json; exit 1; }
python3 -c "
import json, sys, shutil, numpy as np
a=json.load(open('$R/box_prev.json')); b=json.load(open('$R/box.json')); A=np.array(a['P']); B=np.array(b['P']); mv=np.abs(B-A).max()*1000
print('box moved (mm): centre',((B-A).mean(0)*1000).round(1),' rotation %.1f deg'%np.degrees(np.arctan2(b['a'][1],b['a'][0])-np.arctan2(a['a'][1],a['a'][0])))
ok = abs(b['cap_height_m']-0.1505)<0.0025 and mv<20
if not ok: shutil.copy('$R/box_prev.json','$R/box.json'); print('REFIT NOT SANE (cap height %.1f mm, max shift %.0f mm) - kept the previous fit'%(b['cap_height_m']*1000,mv)); sys.exit(1)
"
