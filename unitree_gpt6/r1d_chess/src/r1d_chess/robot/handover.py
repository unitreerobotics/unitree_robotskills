#!/usr/bin/env python3
"""Review or explicitly perform idle head/waist-hold -> chess Runtime ownership transfer."""
import argparse
import hashlib
import json
import subprocess
import sys
from r1d_chess.robot.r1d_config import ROOT, RUNTIME, require_calibration, load_profile, geometry_source
from r1d_chess.robot.robot_client import send_request

# unit -> program its ExecStart must run (anything else is an unknown owner and is refused)
HOLD_PROGRAMS = {'r1d-chess-goinit.service': str(ROOT/'scripts/go_init.py')}
# Program of the other (site-specific) hold units: config/runtime.json "hold_program"; empty = none is accepted.
HOLD_PROGRAM = RUNTIME.get('hold_program') or ''
HOLD_SERVICES = (
    'r1d-chess-goinit.service',
    'r1d-hold-initial.service',
    'r1d-head-cal-reference.service',
    'r1d-chess-reference-after-calib.service',
    'r1d-grippers-closed-reference.service',
    'r1d-chess-head-reference.service',
)


def active_hold_service():
    active = []
    for unit in HOLD_SERVICES:
        status = subprocess.run(['systemctl', '--user', 'is-active', unit],
                                capture_output=True, text=True).stdout.strip()
        if status == 'active':
            command = subprocess.run(['systemctl', '--user', 'show', unit,
                                      '--property=ExecStart', '--value'],
                                     capture_output=True, text=True, check=True).stdout
            expected = HOLD_PROGRAMS.get(unit, HOLD_PROGRAM)
            if not expected or expected not in command:
                raise RuntimeError(f'{unit} is active but is not the expected R1-D hold program')
            active.append(unit)
    if len(active) > 1:
        raise RuntimeError(f'multiple R1-D hold services are active: {active}')
    return active[0] if active else None


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--endpoint',default='ipc:///tmp/astra_r1d_chess.sock')
    p.add_argument('--arm',choices=['left','right'],required=True)
    p.add_argument('--execute',action='store_true',help='Stop the identified idle R1-D hold and explicitly enable chess control')
    a=p.parse_args()
    profile=require_calibration(load_profile(),hardware=True)
    state,_=send_request({'cmd':'get_state'},a.endpoint,1000,5000)
    if not state.get('ok') or state.get('robot')!='R1-D' or state.get('simulated'):
        raise RuntimeError('need a real R1-D chess service')
    if state.get('active_arm')!=a.arm or state.get('control_enabled') or state.get('fault'):
        raise RuntimeError('service must be idle, fault-free and configured for expected physical arm')
    if not state.get('motion_allowed') or not state.get('geometry_ready'):
        raise RuntimeError('service is read-only or complete geometry is missing')
    if state.get('geometry_source') != geometry_source(profile):
        raise RuntimeError('service and local geometry provenance differ')
    digest=hashlib.sha256(json.dumps(profile,sort_keys=True,allow_nan=False).encode()).hexdigest()
    if state.get('profile_sha256')!=digest:
        raise RuntimeError('local calibration differs from running service')
    if any(abs(g['q'])>=.15 for g in state['grippers'].values()):
        raise RuntimeError('handover requires both grippers confirmed empty/closed')
    from r1d_chess.robot.r1d_runtime import R1DRuntimeClient
    with R1DRuntimeClient() as client:
        current=client.state()
        if current.runtime_error or current.thermal_protection_active or current.reset_pose_active:
            raise RuntimeError('Runtime error/thermal/reset state blocks handover')
        from r1d_chess.robot.r1d_transport import InternalDDS
        import numpy as np
        checker=InternalDDS.__new__(InternalDDS);checker.profile=profile
        checker._check_reference(current,np.asarray(state['full_q']),entry=True)
    active=active_hold_service()
    result={'existing_hold_service':active,'will_stop':active,
            'will_enable_actor':a.arm,'executed':False,
            'note':'enable holds current arms/grippers and saved chess head/waist reference; no base/lift selection, no automatic AI restoration'}
    if not a.execute:
        print(json.dumps(result,ensure_ascii=False,indent=2));return
    if current.control_active and active is None:
        raise RuntimeError('unknown Runtime owner; refuse to stop or take over')
    if active is not None:
        subprocess.run(['systemctl','--user','stop',active],check=True)
    # Right after the hold stops its torque ramp-out the joints still settle (the
    # first enable was refused with 'joints moving before handover'); retry while that is the reason.
    import time
    for attempt in range(8):
        if attempt:
            time.sleep(0.25)
        response,_=send_request({'cmd':'enable_control','expected_active_arm':a.arm},a.endpoint,1000,20000)
        if response.get('ok') or 'joints moving' not in str(response.get('error')):
            break
    result.update(executed=True,enable_response=response)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    if not response.get('ok'):
        raise RuntimeError('handover enable failed; previous hold may be stopped. Inspect actual feedback/mode; do not blindly retry or restore AI.')


if __name__=='__main__':main()
