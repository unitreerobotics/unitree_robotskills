#!/usr/bin/env python3
"""Confirmed board -> legal moves / UCI search / validated ten-ply forecast."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import queue
import re
import subprocess
import threading
import time
import pyffish
from r1d_chess.chess.move_constraints import review_uci

# Fairy-Stockfish built with largeboards=yes: CHESS_ENGINE, else `fairy-stockfish` on PATH.
ENGINE = Path(os.environ.get('CHESS_ENGINE') or 'fairy-stockfish')


def uci_move(source, target):
    def square(value):
        if not isinstance(value, str) or not re.fullmatch('[a-i][0-9]', value):
            raise ValueError('square must be a0..i9')
        return value[0]+str(int(value[1])+1)
    return square(source)+square(target)


def current_position(board):
    if board.get('confirmed') is not True or not board.get('game_id') or not board.get('evidence'):
        raise ValueError('board requires game_id, confirmed=true and current visual evidence')
    if not isinstance(board['evidence'],list) or not all(isinstance(p,str) and p.strip() for p in board['evidence']):
        raise ValueError('board evidence must be a nonempty list of current image paths')
    fen = board['initial_fen']
    if pyffish.validate_fen(fen, 'xiangqi') != 1:
        raise ValueError('invalid xiangqi FEN')
    history = board.get('moves', [])
    if not isinstance(history, list):
        raise ValueError('moves must contain actual confirmed UCI history')
    for i, move in enumerate(history):
        if move not in pyffish.legal_moves('xiangqi', fen, history[:i]):
            raise ValueError(f'illegal history at ply {i}: {move}')
    current = pyffish.get_fen('xiangqi', fen, history)
    return fen, history, current, pyffish.legal_moves('xiangqi', fen, history)


def pieces(fen):
    result={}
    for rank,row in zip(range(9,-1,-1),fen.split()[0].split('/')):
        column=0
        for char in row:
            if char.isdigit():
                column+=int(char)
            else:
                result[chr(ord('a')+column)+str(rank)]=char
                column+=1
        if column!=9:
            raise ValueError('invalid board row width')
    return result


def validate_move(board, move):
    _, _, current, legal = current_position(board)
    if move not in legal:
        raise ValueError(f'{move} is not legal in the confirmed current position')
    return {'uci': move, 'fen': current, **review_uci(move)}


def forecast_entries(fen, history, pv):
    entries = []
    played = list(history)
    for move in pv[:10]:
        before = pyffish.get_fen('xiangqi', fen, played)
        if move not in pyffish.legal_moves('xiangqi', fen, played):
            raise ValueError(f'illegal engine PV move {move}')
        played.append(move)
        after = pyffish.get_fen('xiangqi', fen, played)
        entries.append({'ply': len(entries), 'move': move, 'before_fen': before,
                        'after_fen': after, **review_uci(move)})
    return entries


def match_forecast(board, forecast):
    fen, history, current, legal = current_position(board)
    if forecast.get('game_id') != board['game_id'] or forecast.get('initial_fen') != fen:
        return {'matched': False, 'reason': 'different game or initial position'}
    baseline = forecast.get('confirmed_history', [])
    offset = len(history)-len(baseline)
    if offset < 0 or history[:len(baseline)] != baseline:
        return {'matched': False, 'reason': 'history changed'}
    entries = forecast.get('plies', [])
    if history[len(baseline):] != [r['move'] for r in entries[:offset]]:
        return {'matched': False, 'reason': 'actual response differs from forecast'}
    if offset >= len(entries):
        return {'matched': False, 'reason': 'forecast exhausted'}
    item = entries[offset]
    if item['before_fen'] != current or item['move'] not in legal:
        return {'matched': False, 'reason': 'position or legal move mismatch'}
    return {'matched': True, 'next': item, 'confirmed_plies_since_search': offset,
            'note': 'candidate only; both physical arms still require complete transaction preflight'}


def search(board, *, engine=ENGINE, movetime_ms=1500, excluded=()):
    fen, history, current, legal = current_position(board)
    allowed = [m for m in legal if m not in excluded]
    if not allowed:
        return {'status': 'no_legal_candidate', 'game_id': board['game_id'], 'plies': []}
    process = subprocess.Popen([str(engine)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, bufsize=1)
    lines = queue.Queue()
    transcript = []

    def read():
        for line in process.stdout:
            lines.put(line.rstrip())
        lines.put(None)

    threading.Thread(target=read, daemon=True).start()

    def send(value):
        process.stdin.write(value+'\n')
        process.stdin.flush()

    def until(prefix, seconds):
        end = time.monotonic()+seconds
        while True:
            remaining = end-time.monotonic()
            if remaining <= 0:
                raise TimeoutError('xiangqi engine timeout')
            try:
                line = lines.get(timeout=remaining)
            except queue.Empty:
                raise TimeoutError('xiangqi engine timeout') from None
            if line is None:
                raise RuntimeError('xiangqi engine exited')
            transcript.append(line)
            if line.startswith(prefix):
                return line

    try:
        send('uci')
        until('uciok', 10)
        if not any('xiangqi' in line and 'UCI_Variant' in line for line in transcript):
            raise RuntimeError('engine lacks xiangqi; build Fairy-Stockfish with largeboards=yes')
        for command in ['setoption name UCI_Variant value xiangqi', 'setoption name Threads value 2',
                        'setoption name Hash value 128', 'setoption name MultiPV value 3', 'isready']:
            send(command)
        until('readyok', 10)
        send('position fen '+fen+(' moves '+' '.join(history) if history else ''))
        send(f'go movetime {movetime_ms} searchmoves '+' '.join(allowed))
        best = until('bestmove', movetime_ms/1000+10).split()[1]
        if best not in allowed:
            raise ValueError('engine bestmove is not an allowed legal move')
        candidates = {}
        for line in transcript:
            if not line.startswith('info ') or ' pv ' not in line or ' score ' not in line:
                continue
            pv = line.split(' pv ', 1)[1].split()
            if not pv or pv[0] not in allowed:
                continue
            score = re.search(r' score (cp|mate) (-?\d+)', line)
            if score is None:
                continue
            depth = re.search(r' depth (\d+)', line)
            candidates[pv[0]] = {'move': pv[0], 'score_type': score[1], 'score': int(score[2]),
                                  'depth': int(depth[1]) if depth else 0,
                                  'plies': forecast_entries(fen, history, pv), 'raw_info': line}
        if candidates:
            deepest = max(row['depth'] for row in candidates.values())
            candidates = {move: row for move, row in candidates.items() if row['depth'] == deepest}
        if best not in candidates:
            candidates[best] = {'move': best, 'score_type': 'unknown', 'score': None,
                                'plies': forecast_entries(fen, history, [best])}
        def ranking(row):
            if row['score_type'] == 'mate' and row['score'] > 0:
                return (2, -row['score'])
            if row['score_type'] == 'cp':
                return (1, row['score'])
            if row['score_type'] == 'mate':
                return (0, -row['score'])
            return (-1, 0)
        ranked = sorted(candidates.values(), key=ranking, reverse=True)
        selected = ranked[0]
        return {'status': 'searched', 'game_id': board['game_id'], 'initial_fen': fen,
                'confirmed_history': history, 'observed_fen': current,
                'created_at': datetime.now().astimezone().isoformat(),
                'bestmove': selected['move'], 'engine_bestmove': best, 'candidates': ranked,
                'plies': selected['plies'], 'raw_engine_log': transcript,
                'scope': 'up to five rounds / ten legal predicted plies; no promise opponent follows PV'}
    finally:
        if process.poll() is None:
            try:
                send('quit')
                process.wait(timeout=2)
            except (BrokenPipeError, subprocess.TimeoutExpired):
                process.kill()
                process.wait()
        process.stdin.close()
        process.stdout.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=['analyze', 'match', 'validate'])
    p.add_argument('--board', type=Path, required=True)
    p.add_argument('--forecast', type=Path)
    p.add_argument('--output', type=Path)
    p.add_argument('--move')
    p.add_argument('--engine', type=Path, default=ENGINE)
    p.add_argument('--movetime-ms', type=int, default=1500)
    p.add_argument('--exclude', nargs='*', default=[])
    a = p.parse_args()
    board = json.loads(a.board.read_text())
    if a.command == 'match':
        result = match_forecast(board, json.loads(a.forecast.read_text()))
    elif a.command == 'validate':
        result = validate_move(board, a.move)
    else:
        if not 1 <= a.movetime_ms <= 60000:
            p.error('movetime must be 1..60000 ms')
        result = search(board, engine=a.engine, movetime_ms=a.movetime_ms, excluded=a.exclude)
    if a.output:
        a.output.parent.mkdir(parents=True, exist_ok=True)
        with a.output.open('x') as f:
            json.dump(result, f, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
