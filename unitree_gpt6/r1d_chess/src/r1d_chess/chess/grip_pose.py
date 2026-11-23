#!/usr/bin/env python3
"""Spec 抓放位姿换算：棋盘交叉点（夹持点） <-> TCP（Dex1 法兰）。

round_spec 的 source/target/capture 位姿 `xy_m`、`z_m` 是 **TCP**，即 `dex1_base_joint` 原点
（夹爪法兰），不是棋子位置。手指沿 TCP 的 +x 方向伸出，夹持点在离法兰 GRIP_DEPTH_M 处：

    d     = (cos(pitch)·cos(yaw), cos(pitch)·sin(yaw), −sin(pitch))     # 手指方向（与 roll 无关）
    夹持点 = TCP + GRIP_DEPTH_M · d
    TCP   = 夹持点 − GRIP_DEPTH_M · d

默认抓放姿态（抬高手腕，用较大的向下倾角、夹爪前部抓放）：
pitch DEFAULT_PITCH_RAD = 0.8（约 46°），夹持点离法兰 137 mm（指尖内约 6 mm）。手指朝 −y 时 TCP 在
交叉点 +y 方向 95.4 mm、上方 98.3 mm。把 TCP 直接放在交叉点上，棋子会落在 10 cm 以外。

高度规则（与 pitch 无关地固定指尖高度）：
  抓取：指尖在板面（scene surface_z）上方 SOURCE_TIP_ABOVE_SURFACE_M = 3 mm；
  释放：所有走棋棋盘释放 TCP z = BOARD_RELEASE_TCP_Z_M 0.150 m，与 pitch 无关（pitch 0.8 时指尖约
        47 mm，板面上方约 35 mm）。
        托盘：指尖 = tray_release_z_m − finger_end·sin(release_pitch_rad)（profile 标定值）。
  profile 的 board_release_z_m 只对应 release_pitch_rad=0.35；换 pitch 不改 profile，按上式换算。

子命令：
  pose   由格名（或托盘）、手指朝向、可选像素/图像估计的偏移，算出 spec 位姿块
  check  核对 round_spec.json 中 source/target/capture 的夹持点是否落在所写格子的交叉点上
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys

from r1d_chess.paths import ROOT
from r1d_chess.robot.r1d_config import load_profile

BOARD_PATH = ROOT/'config/board_geometry.json'
# 法兰到指尖 143.2 mm（finger_end）。夹爪前部抓放：夹持中心在指尖内约 6 mm，即 137 mm
GRIP_DEPTH_M = .137
DEFAULT_PITCH_RAD = .8
# 抓取高度：指尖在板面上方 3 mm（pitch 0.8 时夹持点在板面上方约 7.4 mm，棋子高 9.9 mm）。
SOURCE_TIP_ABOVE_SURFACE_M = .003
# 棋盘释放高度：所有走棋的棋盘释放 TCP z 固定为 0.150 m，与 pitch 无关
# （pitch 0.8 时指尖约 47 mm，板面上方约 35 mm）。托盘仍用 profile 标定的 tray_release_z_m。
BOARD_RELEASE_TCP_Z_M = .15
# 夹持点与所写格子交叉点的最大距离：小于半格（33.7/2 mm），允许按图像估计的棋子偏位。
SQUARE_TOLERANCE_M = .012
# '+x-y' / '+x+y': fingers point diagonally forward (45 deg); with the side directions the far rank
# (rank 9, x about 0.46 m) is out of reach at any pitch, diagonally it is reachable at pitch <= 0.6.
FINGERS = {'-y': -math.pi/2, '+y': math.pi/2, '+x': 0., '+x-y': -math.pi/4, '+x+y': math.pi/4}


def finger_direction(pitch, yaw):
    return (math.cos(pitch)*math.cos(yaw), math.cos(pitch)*math.sin(yaw), -math.sin(pitch))


def grip_point(tcp_xyz, rpy):
    d = finger_direction(rpy[1], rpy[2])
    return [tcp_xyz[i] + GRIP_DEPTH_M*d[i] for i in range(3)]


def tcp_from_grip(grip_xyz, pitch, yaw):
    d = finger_direction(pitch, yaw)
    return [grip_xyz[i] - GRIP_DEPTH_M*d[i] for i in range(3)]


def release_tcp_z(profile, pitch, kind='board'):
    """TCP z for releasing at `pitch` with the profile's calibrated fingertip release height."""
    key = 'tray_release_z_m' if kind == 'tray' else 'board_release_z_m'
    finger_end = profile['scene']['finger_end']
    if kind != 'tray':
        return BOARD_RELEASE_TCP_Z_M          # every move, any pitch
    tip_z = profile[key] - finger_end*math.sin(profile['release_pitch_rad'])
    return tip_z + finger_end*math.sin(pitch)


def source_tcp_z(profile, pitch):
    """TCP z for grasping: fingertip SOURCE_TIP_ABOVE_SURFACE_M above the board surface."""
    return profile['scene']['surface_z'] + SOURCE_TIP_ABOVE_SURFACE_M + profile['scene']['finger_end']*math.sin(pitch)


def load_board(path=None):
    # R1D_CHESS_BOARD: offline tests only (synthetic board for the synthetic profile).
    return json.loads(Path(path or os.environ.get('R1D_CHESS_BOARD') or BOARD_PATH).read_text())


def intersection(board, square):
    try:
        return board['intersections_xy_m'][square]
    except KeyError:
        raise ValueError(f'board_geometry 没有交叉点 {square}') from None


def nearest(board, xy):
    return min(((name, math.dist(xy, p)) for name, p in board['intersections_xy_m'].items()),
               key=lambda item: item[1])


def check_pose(pose, square, board, *, label, tolerance=SQUARE_TOLERANCE_M):
    """Raise when the grip point is not on `square` (e.g. the TCP itself was put on the intersection)."""
    tcp = list(pose['xy_m']) + [pose['z_m']]
    grip = grip_point(tcp, pose['rpy_rad'])
    target = intersection(board, square)
    error = math.dist(grip[:2], target)
    tcp_error = math.dist(tcp[:2], target)
    result = {'square': square, 'tcp_xyz_m': tcp, 'grip_point_xyz_m': [round(v, 5) for v in grip],
              'intersection_xy_m': target, 'grip_error_m': round(error, 5),
              'nearest_to_grip': nearest(board, grip[:2])[0], 'grip_depth_m': GRIP_DEPTH_M}
    if error > tolerance:
        hint = ('；TCP 本身就在该交叉点上——spec 写的是 TCP（法兰），不是棋子位置，'
                '用 `bash r1d pose` 换算' if tcp_error <= tolerance else '')
        raise ValueError(f'{label} 的夹持点 ({grip[0]:.4f}, {grip[1]:.4f}) 距 {square} 交叉点 '
                         f'{error*1000:.1f} mm，超过 {tolerance*1000:.0f} mm{hint}')
    return result


def check_tray(pose, side, board, *, label, tolerance=.03):
    tcp = list(pose['xy_m']) + [pose['z_m']]
    grip = grip_point(tcp, pose['rpy_rad'])
    centre = board['trays_xy_m'][side]
    error = math.dist(grip[:2], centre)
    if error > tolerance:
        raise ValueError(f'{label} 的夹持点距 {side} 托盘中心 {error*1000:.1f} mm，超过 {tolerance*1000:.0f} mm；'
                         'spec 写的是 TCP，用 `bash r1d pose --role tray` 换算')
    return {'tray': side, 'grip_point_xyz_m': [round(v, 5) for v in grip], 'tray_centre_xy_m': centre,
            'grip_error_m': round(error, 5)}


def check_spec(spec, board):
    """Grip-point checks for a round_spec; returns a summary (raises on the first error)."""
    out = {}
    capture = spec.get('capture')
    out['source'] = check_pose(spec['source'], spec['source_square'], board, label='source')
    out['target'] = check_pose(spec['target'], spec['target_square'], board, label='target')
    if spec.get('move_kind') == 'capture' and capture:
        out['capture.source'] = check_pose(capture['source'], spec['target_square'], board,
                                           label='capture.source')
        tray = capture['offboard_target']
        out['capture.offboard_target'] = check_tray(tray, tray.get('physical_side'), board,
                                                    label='capture.offboard_target')
    return out


def make_pose(args, profile, board):
    pitch = DEFAULT_PITCH_RAD if args.pitch is None else args.pitch
    yaw = FINGERS[args.fingers] if args.yaw is None else args.yaw
    surface = profile['scene']['surface_z']
    dx, dy = args.offset_xy
    if args.role == 'tray':
        cx, cy = board['trays_xy_m'][args.tray]
        where = f'{args.tray} 托盘中心'
    else:
        cx, cy = intersection(board, args.square)
        where = f'{args.square} 交叉点'
    grip_xy = [cx+dx, cy+dy]
    d = finger_direction(pitch, yaw)
    if args.role == 'source':
        if args.grip_z is None:
            tcp_z = source_tcp_z(profile, pitch)
            tcp = [grip_xy[i] - GRIP_DEPTH_M*d[i] for i in range(2)] + [tcp_z]
            grip_z = tcp_z + GRIP_DEPTH_M*d[2]
        else:
            grip_z = args.grip_z
            tcp = tcp_from_grip(grip_xy+[grip_z], pitch, yaw)
        tip_z = tcp[2] - profile['scene']['finger_end']*math.sin(pitch)
        z_note = (f'夹持点 z {grip_z:.4f}（板面 {surface} 上方 {(grip_z-surface)*1000:.1f} mm），'
                  f'指尖 z {tip_z:.4f}（板面上方 {(tip_z-surface)*1000:.1f} mm）')
    else:
        if args.grip_z is not None:
            raise ValueError(f'{args.role} 的 TCP z 由释放指尖高度和 pitch 决定，不能指定 --grip-z')
        tcp_z = release_tcp_z(profile, pitch, 'tray' if args.role == 'tray' else 'board')
        tcp = [grip_xy[i] - GRIP_DEPTH_M*d[i] for i in range(2)] + [tcp_z]
        grip_z = tcp_z + GRIP_DEPTH_M*d[2]
        z_note = f'TCP z {tcp_z:.4f}（释放指尖高度 + finger_end·sin(pitch)），夹持点 z {grip_z:.4f}'
    offset = f'，按图像估计偏移 ({dx*1000:+.1f}, {dy*1000:+.1f}) mm' if (dx or dy) else ''
    return {
        'xy_m': [round(tcp[0], 4), round(tcp[1], 4)], 'z_m': round(tcp[2], 4),
        'rpy_rad': [0, pitch, yaw],
        'evidence': (f'<本轮规划图依据>。TCP 由 `bash r1d pose` 换算：夹持点（离法兰 {GRIP_DEPTH_M*1000:.0f} mm）'
                     f'在{where}{offset} ({grip_xy[0]:.4f}, {grip_xy[1]:.4f})；{z_note}；'
                     f'pitch {pitch}，手指方向 {args.fingers if args.yaw is None else f"yaw {yaw:.4f}"}。'),
        '_grip_point_xyz_m': [round(grip_xy[0], 4), round(grip_xy[1], 4), round(grip_z, 4)],
    }


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)
    q = sub.add_parser('pose', help='由格名/托盘算 spec 位姿块（TCP）')
    q.add_argument('--role', choices=('source', 'target', 'tray'), required=True)
    q.add_argument('--square', help='棋盘格名，如 c5（rank 0 离机器人最近）')
    q.add_argument('--tray', choices=('left', 'right'), help='role=tray 时的同侧托盘')
    q.add_argument('--fingers', choices=tuple(FINGERS), default='-y',
                   help='手指指向（基座系）：-y 朝机器人右侧、+y 朝左侧、+x 朝前')
    q.add_argument('--yaw', type=float, help='直接给 yaw（弧度），覆盖 --fingers')
    q.add_argument('--pitch', type=float, help=f'默认 {DEFAULT_PITCH_RAD}（较大向下倾角，手腕抬高）')
    q.add_argument('--offset-xy', type=float, nargs=2, default=(0., 0.), metavar=('DX', 'DY'),
                   help='棋子相对交叉点的偏移（米，基座系），来自本轮图像估计')
    q.add_argument('--grip-z', type=float, help='仅 source：直接给夹持点高度；默认按指尖在板面上方 3 mm')
    c = sub.add_parser('check', help='核对 round_spec 的夹持点是否落在所写格子上')
    c.add_argument('--spec', required=True, type=Path)
    a = p.parse_args()
    profile, board = load_profile(), load_board()
    if a.cmd == 'pose':
        if (a.role == 'tray') != (a.tray is not None) or (a.role != 'tray') != (a.square is not None):
            raise SystemExit('source/target 需要 --square；tray 需要 --tray')
        result = make_pose(a, profile, board)
    else:
        result = check_spec(json.loads(a.spec.read_text()), board)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
