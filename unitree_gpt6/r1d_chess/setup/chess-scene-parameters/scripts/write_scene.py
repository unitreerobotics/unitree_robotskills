#!/usr/bin/env python3
"""Offline: derive and (with --write) store the chess scene parameters.

Writes to config/robot.json: tcp_xyz_rpy (Dex1 flange = dex1_base_joint origin), init_xyz_rpy (FK of
init_joint_rad), scene (BoardGuard envelope), board/tray release heights bound to --pitch; and
config/board_geometry.json (90 intersections) from a scripts/head_board_measure.py result (python scripts/head_board_measure.py).
Default prints the values and the differences only."""
import argparse, json, math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2])); import _common  # noqa
import numpy as np, xml.etree.ElementTree as ET
from r1d_chess.robot.r1d_config import MODEL_PATH, PROFILE, load_profile
from r1d_chess.robot.r1d_kinematics import R1DKinematics
from r1d_chess.robot.motion_harness import waypoint

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--measure', required=True, help='head_board_measure.py output json')
p.add_argument('--pitch', type=float, default=.35, help='grasp/release pitch the release heights are bound to')
p.add_argument('--board-tip-clearance', type=float, default=.013, help='fingertip above the board surface at release')
p.add_argument('--tray-tip-clearance', type=float, default=.068, help='fingertip above the table at tray release; must clear the tray rim')
p.add_argument('--uncertainty', type=float, default=.004)
p.add_argument('--finger-end', type=float, default=.1432, help='flange to fingertip, drawing 143.2 mm')
p.add_argument('--half-width', type=float, default=.0585, help='open outer width / 2, drawing 117 mm')
p.add_argument('--half-thickness', type=float, default=.005, help='finger tip thickness / 2; valid for pitch >= 0.2')
p.add_argument('--board-thickness', type=float, default=.02)
p.add_argument('--board-border', type=float, default=.03, help='board footprint = grid extent + this margin on every side')
p.add_argument('--cell', type=float, nargs=2, default=[.0356, .0337], metavar=('FILE_Y', 'RANK_X'))
p.add_argument('--piece', type=float, nargs=2, default=[.0297, .0099], metavar=('DIAMETER', 'HEIGHT'))
p.add_argument('--trays', type=float, nargs=4, metavar=('LX', 'LY', 'RX', 'RY'), help='default: keep stored trays')
p.add_argument('--write', action='store_true')
a = p.parse_args()
root = ET.parse(str(MODEL_PATH)).getroot(); J = {j.get('name'): j for j in root.findall('joint')}
tcp = {s: [float(x) for x in np.fromstring(J[f'{s}_dex1_base_joint'].find('origin').get('xyz'), sep=' ')]+[0., 0., 0.] for s in ('left', 'right')}
prof = load_profile(); new = json.loads(json.dumps(prof))
new['tcp_xyz_rpy'] = tcp
k = R1DKinematics(new); q = np.r_[prof['init_joint_rad']['left'], prof['init_joint_rad']['right']]
new['init_xyz_rpy'] = {s: [float(waypoint(T)[x]) for x in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')] for s, T in zip(('left', 'right'), k.fk(q))}
m = json.loads(Path(a.measure).read_text())
new['scene'] = {'surface_z': round(m['surface_z_m'], 3), 'uncertainty': a.uncertainty, 'finger_end': a.finger_end,
                'half_width': a.half_width, 'half_thickness': a.half_thickness, 'travel_clearance': .04,
                'minimum_clearance': 0., 'xy_tolerance': .001, 'rotation_tolerance': .008,
                # Two-level support: board top over the footprint, table elsewhere (init poses rest beside the board).
                'table_z': round(round(m['surface_z_m'], 3)-a.board_thickness, 4),
                'board_centre_xy': [round(v, 4) for v in m['grid_centre_xy_m']], 'board_yaw': round(m['grid_yaw_rad'], 4),
                'board_half_extent': [round(4.5*a.cell[1]+a.board_border, 4), round(4*a.cell[0]+a.board_border, 4)]}
lift = a.finger_end*math.sin(a.pitch)
surface = new['scene']['surface_z']; table = surface-a.board_thickness
new['board_release_z_m'] = round(surface+a.board_tip_clearance+lift, 4)
new['tray_release_z_m'] = round(table+a.tray_tip_clearance+lift, 4)
import hashlib
new['kinematic_model_sha256'] = hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest()
new['kinematic_model_urdf'] = (str(MODEL_PATH.relative_to(_common.ROOT)) if MODEL_PATH.is_relative_to(_common.ROOT)
                               else str(MODEL_PATH))
new['release_pitch_rad'] = a.pitch
for key in ('tcp_xyz_rpy', 'scene', 'board_release_z_m', 'tray_release_z_m', 'release_pitch_rad', 'kinematic_model_sha256'):
    print(f'{key}: {json.dumps(new[key])}' + ('' if new.get(key) == prof.get(key) else f'   (was {json.dumps(prof.get(key))})'))
print('init_xyz_rpy max change mm', round(max(np.linalg.norm(np.subtract(new['init_xyz_rpy'][s][:3], prof['init_xyz_rpy'][s][:3]))
                                            for s in ('left', 'right'))*1e3, 2))
cx, cy = m['grid_centre_xy_m']; th = m['grid_yaw_rad']; ca, sa = math.cos(th), math.sin(th)
pts = {f'{f}{r}': [round(cx+(r-4.5)*a.cell[1]*ca-(4-i)*a.cell[0]*sa, 4), round(cy+(r-4.5)*a.cell[1]*sa+(4-i)*a.cell[0]*ca, 4)]
       for i, f in enumerate('abcdefghi') for r in range(10)}
gpath = _common.ROOT/'config/board_geometry.json'
old = json.loads(gpath.read_text()) if gpath.exists() else {}
trays = ({'left': a.trays[:2], 'right': a.trays[2:]} if a.trays else old.get('trays_xy_m'))
if trays is None:
    print('WARNING: no tray positions; pass --trays LX LY RX RY (read them from the top-down image of head_board_measure.py)')
geo = {'schema_version': 1, 'status': 'model_estimate', 'frame': 'waist_yaw_link', 'surface_z_m': round(m['surface_z_m'], 4),
       'centre_xy_m': [round(cx, 4), round(cy, 4)], 'yaw_rad': round(th, 4), 'cell_m': {'file_y': a.cell[0], 'rank_x': a.cell[1]},
       'board_thickness_m': a.board_thickness, 'piece': {'diameter_m': a.piece[0], 'height_m': a.piece[1]},
       'orientation': 'rank 0 nearest the robot; file a on the +y (robot left) side', 'trays_xy_m': trays,
       'intersections_xy_m': pts, 'evidence': str(Path(a.measure).resolve().parent)}
if old.get('intersections_xy_m'):
    d = max(math.dist(pts[s], old['intersections_xy_m'][s]) for s in pts)
    print('board intersections max change mm', round(d*1e3, 2))
if a.write:
    PROFILE.write_text(json.dumps(new, indent=2, ensure_ascii=False)+'\n')
    for key in ('cross_check', 'measured_at', 'valid_for', 'trays_note'):
        if key in old: geo[key] = old[key]
    gpath.write_text(json.dumps(geo, indent=1))
    print('written', PROFILE, gpath)
else:
    print('dry run; add --write to store')
