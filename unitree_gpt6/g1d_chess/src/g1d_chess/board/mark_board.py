#!/usr/bin/env python3
"""Offline board-surface annotation from fresh, manually observed controls."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def grid(square):
    if not isinstance(square, str) or len(square) != 2 or square[0] not in 'abcdefghi' or square[1] not in '0123456789':
        raise ValueError(f'非法交点名称: {square!r}')
    return [ord(square[0]) - ord('a'), int(square[1])]


def image_at(path):
    image = cv2.imread(str(path))
    if image is None:
        raise ValueError(f'无法读取图片: {path}')
    return image


def project(matrix, coordinates):
    homogeneous = np.column_stack((coordinates, np.ones(len(coordinates)))) @ matrix.T
    if np.any(np.abs(homogeneous[:, 2]) < 1e-9):
        raise ValueError('投影退化：交点在无穷远')
    result = homogeneous[:, :2] / homogeneous[:, 2:]
    if not np.isfinite(result).all():
        raise ValueError('投影产生非有限坐标')
    return result


def residual(matrix, rows):
    errors = np.linalg.norm(project(matrix, np.array([grid(p['square']) for p in rows])) - np.array([p['pixel_xy'] for p in rows]), axis=1)
    return {'rms_px': float(np.sqrt(np.mean(errors ** 2))), 'max_px': float(errors.max()),
            'by_square_px': {p['square']: float(e) for p, e in zip(rows, errors)}}


def init(args):
    image = image_at(args.image)
    if args.output.exists():
        raise ValueError('输出已存在，请另选文件名，避免覆盖手工标记')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write(args.output, {
        'schema_version': 1, 'source_image': str(args.image.resolve()),
        'source_sha256': digest(args.image), 'image_size_wh': [image.shape[1], image.shape[0]],
        'orientation_confirmed': False,
        'orientation': 'robot_view_a_to_i_left_to_right_0_to_9_near_to_far',
        'points': [],
        'point_format': {'square': 'a0', 'pixel_xy': [None, None], 'usage': 'fit',
                         'evidence': 'visible_board_line_intersection', 'note': ''}
    })
    return {'controls': str(args.output), 'next': '确认方向，填写原图中实际可见的交点；fit与holdout必须分开。'}


def build(args):
    controls = read(args.controls)
    if controls.get('schema_version') != 1 or controls.get('orientation_confirmed') is not True:
        raise ValueError('需人工确认方向，并设置 orientation_confirmed=true')
    image_path = Path(controls['source_image'])
    if not image_path.is_absolute():
        image_path = args.controls.resolve().parent / image_path
    if digest(image_path) != controls['source_sha256']:
        raise ValueError('图片哈希已变化，请重新标记')
    image = image_at(image_path)
    height, width = image.shape[:2]
    if controls['image_size_wh'] != [width, height]:
        raise ValueError('标记尺寸与原图不一致')
    rows = controls['points']
    seen = set()
    for row in rows:
        square = row['square']
        grid(square)
        if square in seen:
            raise ValueError(f'重复交点: {square}')
        seen.add(square)
        xy = np.asarray(row['pixel_xy'], dtype=float)
        if xy.shape != (2,) or not np.isfinite(xy).all() or not (0 <= xy[0] < width and 0 <= xy[1] < height):
            raise ValueError(f'{square}: 必须填写原图范围内的有限像素坐标')
        if row['usage'] not in ('fit', 'holdout') or row['evidence'] != 'visible_board_line_intersection':
            raise ValueError(f'{square}: 仅可见板面交点能作为fit/holdout，遮挡点不要填写')
    if not np.isfinite(args.max_error_px) or args.max_error_px <= 0:
        raise ValueError('max-error-px必须为正有限值')
    groups = {'global': range(10)} if args.mapping == 'global' else {'near': range(5), 'far': range(5, 10)}
    matrices, metrics, region_points = {}, {}, {}
    for name, ranks in groups.items():
        fit = [p for p in rows if int(p['square'][1]) in ranks and p['usage'] == 'fit']
        holdout = [p for p in rows if int(p['square'][1]) in ranks and p['usage'] == 'holdout']
        if len(fit) < 6 or len(holdout) < 2:
            raise ValueError(f'{name}: 至少6个拟合点、2个独立留出点，并分布在区域周边和内部')
        xy = np.array([grid(p['square']) for p in fit], dtype=float)
        uv = np.array([p['pixel_xy'] for p in fit], dtype=float)
        if np.linalg.matrix_rank(xy - xy.mean(axis=0)) < 2 or np.linalg.matrix_rank(uv - uv.mean(axis=0)) < 2:
            raise ValueError(f'{name}: 控制点共线')
        matrix, _ = cv2.findHomography(xy, uv, method=0)
        if matrix is None or not np.isfinite(matrix).all() or np.linalg.matrix_rank(matrix) < 3:
            raise ValueError(f'{name}: 单应拟合退化')
        matrices[name] = matrix
        metrics[name] = {'fit': residual(matrix, fit), 'holdout': residual(matrix, holdout)}
        squares = [f'{f}{r}' for r in ranks for f in 'abcdefghi']
        region_points[name] = dict(zip(squares, project(matrix, np.array([grid(s) for s in squares])).tolist()))
    observations = {p['square']: p for p in rows}
    points = []
    for name, values in region_points.items():
        for square, xy in values.items():
            if not (0 <= xy[0] < width and 0 <= xy[1] < height):
                raise ValueError(f'{square}: 推算点超出图像，需覆盖完整棋盘的新图或修正标记')
            observation = observations.get(square)
            points.append({'square': square, 'grid_xy': grid(square),
                           'original_image_pixel_xy': xy, 'mapping': name,
                           'evidence_status': observation['usage'] if observation else 'inferred',
                           'observed_pixel_xy': observation['pixel_xy'] if observation else None,
                           'robot_ee_pose': None})
    lookup = {p['square']: p for p in points}
    # Confirm numbering geometry and reject folded/mirrored maps. View convention is explicit.
    for r in range(10):
        for f in range(8):
            a = lookup[f'{chr(97+f)}{r}']['original_image_pixel_xy']
            b = lookup[f'{chr(98+f)}{r}']['original_image_pixel_xy']
            if b[0] <= a[0]:
                raise ValueError('a–i方向不符合约定，或映射发生折叠')
    for f in 'abcdefghi':
        for r in range(9):
            if lookup[f'{f}{r+1}']['original_image_pixel_xy'][1] >= lookup[f'{f}{r}']['original_image_pixel_xy'][1]:
                raise ValueError('0–9应从近到远、在标准头图中从下到上；请核对图像朝向和河两侧拟合')
    passed = all(m[k]['max_px'] <= args.max_error_px for m in metrics.values() for k in ('fit', 'holdout'))
    result = {'schema_version': 1, 'status': 'pending_visual_review' if passed else 'needs_reannotation',
              'source_image': str(image_path.resolve()), 'source_sha256': digest(image_path),
              'controls_sha256': digest(args.controls), 'image_size_wh': [width, height],
              'coordinate_semantics': 'board_surface_pixels_not_piece_top_or_robot_coordinates',
              'mapping': args.mapping, 'mappings': {k: v.tolist() for k, v in matrices.items()},
              'validation': {'max_error_px_threshold': args.max_error_px, 'numeric_pass': passed, 'metrics': metrics},
              'visual_review': 'pending', 'robot_base_transform': None, 'physical_dimensions': None, 'points': points}
    # A fresh output directory keeps previous reviews and image bindings intact.
    args.output_dir.mkdir(parents=True, exist_ok=False)
    write(args.output_dir / 'board_points_pixels.json', result)
    write(args.output_dir / 'controls.json', controls)
    with (args.output_dir / 'board_points.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['square', 'file_index', 'rank', 'u_px', 'v_px', 'evidence_status'])
        for p in points:
            writer.writerow([p['square'], *p['grid_xy'], *p['original_image_pixel_xy'], p['evidence_status']])
    view = image.copy()
    def at(s):
        return tuple(round(v) for v in lookup[s]['original_image_pixel_xy'])
    for r in range(10):
        cv2.line(view, at(f'a{r}'), at(f'i{r}'), (180, 150, 20), 1)
    for f in 'abcdefghi':
        for low, high in ((0, 4), (5, 9)):
            cv2.line(view, at(f'{f}{low}'), at(f'{f}{high}'), (180, 150, 20), 1)
    for p in points:
        xy = at(p['square'])
        color = {'fit': (40, 220, 40), 'holdout': (255, 150, 40), 'inferred': (0, 230, 255)}[p['evidence_status']]
        cv2.circle(view, xy, 3, color, -1)
        if p['observed_pixel_xy'] is not None:
            observed = tuple(round(v) for v in p['observed_pixel_xy'])
            cv2.drawMarker(view, observed, color, cv2.MARKER_CROSS, 9, 1)
        cv2.putText(view, p['square'], (xy[0]+4, xy[1]-4), cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(view, p['square'], (xy[0]+4, xy[1]-4), cv2.FONT_HERSHEY_SIMPLEX, .35, (255, 255, 255), 1, cv2.LINE_AA)
    if not cv2.imwrite(str(args.output_dir / 'board_points_annotated.png'), view):
        raise OSError('标注图写入失败')
    return {'output_dir': str(args.output_dir), 'points': len(points), 'status': result['status'], 'validation': result['validation']}


def query(args):
    grid(args.square)
    data = read(args.reference)
    return {'status': data['status'], 'visual_review': data['visual_review'],
            'source_image': data['source_image'], 'source_sha256': data['source_sha256'],
            'point': next(p for p in data['points'] if p['square'] == args.square),
            'robot_motion_generated': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('init', help='绑定新图并生成空白标记文件')
    p.add_argument('--image', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.set_defaults(run=init)
    p = sub.add_parser('build', help='拟合、留出验证并绘制90交点')
    p.add_argument('--controls', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--mapping', choices=['global', 'split'], default='global')
    p.add_argument('--max-error-px', type=float, required=True, help='根据本次分辨率和读点不确定度明确选择')
    p.set_defaults(run=build)
    p = sub.add_parser('query', help='查询指定结果中的板面像素交点')
    p.add_argument('square')
    p.add_argument('--reference', type=Path, required=True)
    p.set_defaults(run=query)
    args = parser.parse_args()
    try:
        result = args.run(args)
    except (ValueError, KeyError, TypeError, OSError, cv2.error) as exc:
        parser.exit(2, f'错误: {exc}\n')
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    if result.get('status') == 'needs_reannotation':
        raise SystemExit(2)


if __name__ == '__main__':
    main()
