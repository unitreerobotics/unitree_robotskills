#!/usr/bin/env python3
"""Read-only: record the current arms, head, waist and lift as the chess initial pose of THIS robot.
Needs r1d-runtime.service running; sends no commands. Default prints only; --write stores into
config/robot.json (init_joint_rad, head_q_rad, waist_q_rad, lift_height_m) and config/chess_initial_pose.json.
Run it with the robot standing in the pose you want to start every round from."""
import argparse, json, sys, time
from datetime import datetime
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2])); import _common  # noqa
import numpy as np
from r1d_chess.robot.r1d_config import PROFILE
from r1d_chess.robot.r1d_runtime import load_experiment, R1DRuntimeClient

p = argparse.ArgumentParser(description=__doc__); p.add_argument('--write', action='store_true')
p.add_argument('--head-q', type=float, nargs=2, help='override the head reference (e.g. lowest pitch for a centred board)')
a = p.parse_args()
load_experiment()
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
box = {}
ChannelFactoryInitialize(0, _common.RUNTIME['network_interface'])
sub = ChannelSubscriber('rt/lowstate', LowState_); sub.Init(lambda m: box.__setitem__('s', m), 1)
samples = []
end = time.monotonic()+1.
while time.monotonic() < end:
    if 's' in box: samples.append(np.array([x.q for x in box['s'].motor_state]))
    time.sleep(.01)
sub.Close()
if not samples:
    sys.exit('no rt/lowstate')
q = np.median(samples, 0); dq = np.ptp(samples, 0)
with R1DRuntimeClient() as client:
    st = client.state(); base = st.observation.base_group[0]
    lift, moving = base.lift_height, [base.base_linear_vel_x, base.base_linear_vel_y, base.base_angular_vel_z, base.lift_linear_vel_z]
# Normalised rt/state_vel thresholds from docs/r1d_twist_dds_topics.md: linear .01, angular .025.
if np.max(dq[15:31]) > .01 or any(abs(v or 0) > lim for v, lim in zip(moving, (.01, .01, .025, .01))):
    sys.exit('robot is moving; hold still and retry')
pose = {'captured_at': datetime.now().astimezone().isoformat(), 'init_joint_rad': {'left': q[15:22].tolist(), 'right': q[22:29].tolist()},
        'head_q_rad': list(a.head_q) if a.head_q else q[29:31].tolist(), 'measured_head_q_rad': q[29:31].tolist(),
        'waist_q_rad': [float(q[13])], 'lift_height_m': float(lift), 'source': 'rt/lowstate median over 1 s; lift from Runtime base_group'}
print(json.dumps(pose, indent=1))
if a.write:
    prof = json.loads(PROFILE.read_text())
    prof.update({k: pose[k] for k in ('init_joint_rad', 'head_q_rad', 'waist_q_rad', 'lift_height_m')})
    prof['initial_pose_captured_at'] = pose['captured_at']
    prof['initial_pose_evidence'] = 'config/chess_initial_pose.json'
    PROFILE.write_text(json.dumps(prof, indent=2, ensure_ascii=False)+'\n')
    (_common.ROOT/'config/chess_initial_pose.json').write_text(json.dumps(pose, indent=2)+'\n')
    print('written; now rerun write_scene.py so init_xyz_rpy follows the new joints')
