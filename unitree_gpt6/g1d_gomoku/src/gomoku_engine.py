"""Freestyle black move selection from a complete observed 19x19 board.

The caller supplies a perception result, never a requested move. No model,
network, robot or camera is needed to evaluate the recognized position.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

BOARD_SIZE = 19
DIRECTIONS = ((1, 0), (0, 1), (1, 1), (1, -1))
Point = tuple[int, int]


@dataclass(frozen=True)
class Decision:
    row: int
    column: int
    reason: str
    immediate_white_threats: int = 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "row": self.row,
            "column": self.column,
            "color": "black",
            "reason": self.reason,
            "immediate_white_threats": self.immediate_white_threats,
            "selection_source": "autonomous_engine",
            "strategy": "immediate_win_then_block_then_local_threats",
        }


def validate_board(value: Any) -> list[list[int]]:
    if not isinstance(value, list) or len(value) != BOARD_SIZE:
        raise ValueError("Recognized board must have 19 rows")
    for row in value:
        if not isinstance(row, list) or len(row) != BOARD_SIZE:
            raise ValueError("Recognized board must have 19 columns")
        if any(type(cell) is not int or cell not in (0, 1, 2) for cell in row):
            raise ValueError("Board cells must be 0=empty, 1=black or 2=white; resolve uncertainty")
    return [row[:] for row in value]


def inside(row: int, column: int) -> bool:
    return 0 <= row < BOARD_SIZE and 0 <= column < BOARD_SIZE


def run_length(board: list[list[int]], row: int, column: int, dr: int, dc: int, color: int) -> int:
    count = 0
    row, column = row + dr, column + dc
    while inside(row, column) and board[row][column] == color:
        count += 1
        row, column = row + dr, column + dc
    return count


def makes_five(board: list[list[int]], point: Point, color: int) -> bool:
    row, column = point
    return any(
        1
        + run_length(board, row, column, dr, dc, color)
        + run_length(board, row, column, -dr, -dc, color)
        >= 5
        for dr, dc in DIRECTIONS
    )


def winners(board: list[list[int]]) -> set[int]:
    return {
        color
        for row in range(BOARD_SIZE)
        for column in range(BOARD_SIZE)
        if (color := board[row][column]) and makes_five(board, (row, column), color)
    }


def validate_black_turn(board: list[list[int]]) -> None:
    black = sum(cell == 1 for row in board for cell in row)
    white = sum(cell == 2 for row in board for cell in row)
    if black != white:
        raise ValueError(
            "Not black's turn, or recognition is inconsistent: black/white counts differ"
        )
    if winners(board):
        raise ValueError("Game already has a winner; do not place another stone")
    if black + white == BOARD_SIZE**2:
        raise ValueError("Board is full")


def candidates(board: list[list[int]]) -> list[Point]:
    occupied = [(r, c) for r in range(BOARD_SIZE) for c in range(BOARD_SIZE) if board[r][c]]
    if not occupied:
        return [(9, 9)]
    return sorted(
        {
            (r + dr, c + dc)
            for r, c in occupied
            for dr in range(-2, 3)
            for dc in range(-2, 3)
            if inside(r + dr, c + dc) and board[r + dr][c + dc] == 0
        }
    )


def winning_moves(board: list[list[int]], color: int) -> set[Point]:
    return {point for point in candidates(board) if makes_five(board, point, color)}


def potential(board: list[list[int]], point: Point, color: int) -> int:
    """Score open five-cell windows, including broken lines; blocked lines score zero."""
    row, column = point
    total = 0
    for dr, dc in DIRECTIONS:
        for offset in range(5):
            cells = [(row + (i - offset) * dr, column + (i - offset) * dc) for i in range(5)]
            if not all(inside(r, c) for r, c in cells):
                continue
            values = [color if (r, c) == point else board[r][c] for r, c in cells]
            if 3 - color not in values:
                total += 10 ** values.count(color)
    return total


def choose_move(observed_board: Any) -> Decision:
    """Select a legal black move without accepting any target override."""
    board = validate_board(observed_board)
    validate_black_turn(board)
    points = candidates(board)
    wins = winning_moves(board, 1)
    threats = winning_moves(board, 2)
    choices = wins or threats or set(points)

    def rank(point: Point) -> tuple[int, int, int, int]:
        attack = potential(board, point, 1)
        defense = potential(board, point, 2)
        r, c = point
        board[r][c] = 1
        forks = len(winning_moves(board, 1))
        board[r][c] = 0
        return (
            attack * 2 + defense * 3 + min(forks, 2) * 100000,
            -(abs(r - 9) + abs(c - 9)),
            -r,
            -c,
        )

    selected = max(choices, key=rank)
    if wins:
        reason = "本手可连成五颗，优先取胜"
    elif threats:
        reason = (
            "阻挡白方下一手成五"
            if len(threats) == 1
            else "白方有多个成五点，选择防守点；无法保证化解全部威胁"
        )
    elif points == [(9, 9)]:
        reason = "空棋盘由黑方先行，占据中心"
    else:
        reason = "结合连续及跳空棋形，兼顾自身威胁与对方发展"
    return Decision(selected[0] + 1, selected[1] + 1, reason, len(threats))
