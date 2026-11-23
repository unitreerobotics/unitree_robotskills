#!/bin/bash
# One tracking-benchmark run on the real robot: read-only snapshot -> plan from it -> run once.
# usage: bench.sh <adapter|direct> <off|gravity|gravity_i|full>
# env: FF_CONFIG (feedforward json), PLAN_ARGS, RUN_ARGS, BENCH_DIR (output root), BENCH_PROFILE,
#      R1D_CHESS_PYTHON (interpreter, default: python on PATH)
set -e
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd -- "$HERE/../../.." && pwd)
cd "$ROOT"
P=${R1D_CHESS_PYTHON:-python}
B=$HERE/arm_tracking_benchmark.py
D=${BENCH_DIR:-runs/tracking_benchmark}
mkdir -p "$D"
export R1D_CHESS_PROFILE=${BENCH_PROFILE:-$ROOT/config/robot.json}
timeout 30 $P -B $B snapshot --output $D/start_snapshot.json >/dev/null
$P -B $B plan --start $D/start_snapshot.json --profile "$R1D_CHESS_PROFILE" \
   --output $D/trajectory.json ${PLAN_ARGS} > $D/last_plan.json || { cat $D/last_plan.json; exit 2; }
timeout 150 $P -B $B run --trajectory $D/trajectory.json --path "$1" --ff "$2" \
   --output-dir $D/runs --confirm-motion ${FF_CONFIG:+--ff-config $FF_CONFIG} ${RUN_ARGS}
