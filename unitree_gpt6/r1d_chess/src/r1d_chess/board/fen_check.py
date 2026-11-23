#!/usr/bin/env python3
"""Check a FEN you wrote against the board image. Offline; no robot.

1. Validates the FEN with pyffish (xiangqi) and prints the board as text, rank 9 on top, files a-i.
2. Counts pieces per side and flags impossible counts (more than 1 king, 2 advisors/elephants/horses/
   rooks/cannons, 5 pawns) and kings outside their palace.
3. With --view (an output directory of board_view.py) draws the FEN letters on board_topdown.jpg at every
   intersection: compare each letter with the piece under it. Writes fen_overlay.jpg there.
Coordinates follow r1d_chess: FEN rank order 9 -> 0 (far -> near), files a -> i (a = robot left).
"""
import argparse
import json
import sys
from pathlib import Path
import cv2
import numpy as np

FILES = 'abcdefghi'
MAX = {'k': 1, 'a': 2, 'b': 2, 'n': 2, 'r': 2, 'c': 2, 'p': 5}


def parse(fen):
    rows = fen.split()[0].split('/')
    if len(rows) != 10:
        raise ValueError(f'FEN needs 10 ranks, got {len(rows)}')
    board = {}
    for i, row in enumerate(rows):
        rank, f = 9-i, 0
        for ch in row:
            if ch.isdigit():
                f += int(ch)
            else:
                if f > 8:
                    raise ValueError(f'rank {rank}: more than 9 files')
                board[f'{FILES[f]}{rank}'] = ch
                f += 1
        if f != 9:
            raise ValueError(f'rank {rank}: {f} files instead of 9')
    return board


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('fen', help='full FEN in quotes, e.g. "rnbakabnr/9/... w - - 0 1"')
    p.add_argument('--view', help='board_view.py output directory (draws fen_overlay.jpg)')
    a = p.parse_args()
    problems = []
    try:
        import pyffish
        code = pyffish.validate_fen(a.fen, 'xiangqi')
        if code != pyffish.FEN_OK:
            problems.append(f'pyffish validate_fen code {code} (not FEN_OK)')
    except ImportError:
        problems.append('pyffish not available; structural checks only')
    try:
        board = parse(a.fen)
    except ValueError as exc:
        print(json.dumps({'ok': False, 'problems': [str(exc)]}, ensure_ascii=False, indent=1))
        sys.exit(1)
    print('    ' + ' '.join(FILES))
    for r in range(9, -1, -1):
        print(f' {r}  ' + ' '.join(board.get(f'{f}{r}', '.') for f in FILES))
    counts = {}
    for sq, ch in board.items():
        counts[ch] = counts.get(ch, 0)+1
    for ch, n in sorted(counts.items()):
        if n > MAX.get(ch.lower(), 0):
            problems.append(f'{n} x {ch} exceeds the maximum {MAX.get(ch.lower(), 0)}')
    for king, ranks in (('K', range(0, 3)), ('k', range(7, 10))):
        where = [sq for sq, ch in board.items() if ch == king]
        if len(where) != 1:
            problems.append(f'expected exactly one {king}, found {len(where)}')
        for sq in where:
            if sq[0] not in 'def' or int(sq[1:]) not in ranks:
                problems.append(f'{king} at {sq} is outside its palace')
    red = sum(n for ch, n in counts.items() if ch.isupper()); black = sum(n for ch, n in counts.items() if ch.islower())
    print(f'red {red} pieces, black {black} pieces; side to move {a.fen.split()[1] if len(a.fen.split()) > 1 else "?"}')
    if a.view:
        view = Path(a.view)
        meta = json.loads((view/'board_view.json').read_text())
        img = cv2.imread(str(view/'board_topdown.jpg'))
        dy = meta['topdown_title_px']+meta['topdown_padding_px']; dx = meta['topdown_padding_px']
        for sq, (col, row) in meta['topdown_intersection_px'].items():
            x, y = col+dx, row+dy
            ch = board.get(sq)
            if ch is None:
                cv2.circle(img, (x, y), 5, (0, 160, 0), -1)
            else:
                colour = (0, 0, 220) if ch.isupper() else (40, 40, 40)
                cv2.rectangle(img, (x+14, y-44), (x+48, y-10), (255, 255, 255), -1)
                cv2.rectangle(img, (x+14, y-44), (x+48, y-10), colour, 2)
                cv2.putText(img, ch, (x+20, y-17), cv2.FONT_HERSHEY_SIMPLEX, 1., colour, 2)
        bar = np.full((40, img.shape[1], 3), 255, np.uint8)
        cv2.putText(bar, 'FEN overlay: boxed letter = your FEN at that intersection, green dot = empty in your FEN',
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 180), 2)
        img = np.vstack((bar, img))
        out = view/'fen_overlay.jpg'
        cv2.imwrite(str(out), img, [cv2.IMWRITE_JPEG_QUALITY, 93])
        print('overlay:', out)
    print(json.dumps({'ok': not problems, 'problems': problems}, ensure_ascii=False, indent=1))
    sys.exit(0 if not problems else 1)


if __name__ == '__main__':
    main()
