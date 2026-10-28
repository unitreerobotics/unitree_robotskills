# Unified Conventions for Board Square Names and Move Notation

English | [中文](board_and_move_conventions.zh.md)

## 1. Board square names (from the robot's viewpoint)

1. The Chinese chess (Xiangqi) board has 9 files × 10 ranks = 90 intersections in total, and pieces sit on the intersections.
2. Each intersection is uniquely named "file letter + rank digit": files a–i, 9 files in total; ranks 0–9, 10 ranks in total.
3. All directions follow the robot's actual viewpoint:
   - Ranks: the side near the robot is rank 0, the side far from the robot is rank 9, and rank numbers increase from near to far (0→9).
   - Files: from the robot's viewpoint, left to right is a → i (a is on the robot's left-hand side, i is on the robot's right-hand side).

## 2. Move notation (must be written as "红方 …" / "黑方 …")

4. Every move must explicitly state the side to move, in the fixed format "红方 …" (Red) / "黑方 …" (Black); keep these Chinese side labels literally. Omitting the side or writing only the piece name is not allowed.
5. Ordinary moves (no capture) are described with intersection coordinates, in the format "<side> <source intersection> → <target intersection>".
   Example: 红方 e6 → e5; 黑方 e3 → e4.
6. A capture must be split into two steps, in the format:
   Step 1: <captured side> <square of the captured piece> → empty (<captured side> <square> 被吃, i.e. captured)
   Step 2: <capturing side> <source square of the capturing piece> → <target square> (target square = original square of the captured piece)
   Example (Red captures Black):
   Step 1: 黑方 e0 → empty (黑方 e0 被吃)
   Step 2: 红方 e3 → e0
7. Traditional notation (红方炮二平五 / 黑方马8进7) may be added as an aid, but the coordinate form must be kept.
