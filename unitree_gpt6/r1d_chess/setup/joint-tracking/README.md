# Joint tracking: verification on the Runtime link and torque feedforward fine-tuning

English | [中文](README.zh.md)

## When to use
- After the results of [arm-calibration](../arm-calibration/README.md) are installed, to verify the tracking error on the Runtime link that chess actually uses.
- After replacing the gripper or the load, replacing the URDF, or changing the Runtime PD gains.
- The chess service reports `arm tracking error exceeded R1-D limit`, or the TCP error during execution reaches the centimetre level.
- When arm-calibration is not possible, this procedure alone can also be used to tune from neutral initial values (gravity scale, Coulomb friction, integral), with lower accuracy than the former.

## Background
- The robot runtime (Runtime) sends only position PD to the joints, with the `dq` target always 0; qtau is written unchanged into the `tau` of the motor.
- Without feedforward, the static error is approximately load torque / kp; during motion there is an additional damping lag term, approximately kd·v / kp.
- For the feedforward formula and the meaning of each term, see [docs/r1d_joint_tracking_feedforward.md](../../docs/r1d_joint_tracking_feedforward.md); the parameters are in `config/arm_torque_ff.json`.
- The PD gains kp/kd are read by the scripts from the robot configuration of the built-in Runtime (`third_party/robot_runtime/configs`); do not copy the values by hand.

## Prerequisites
- `r1d-runtime.service` is running, with no other control session. The chess service or the hold service must not run during tuning.
- `config/robot.json` must have at least `init_joint_rad`, `head_q_rad`, `waist_q_rad`, `lift_height_m` (chess-scene-parameters step 1).
- There are no obstacles below or around the arms, and someone is nearby who can take over at any time.

## Safety and authorisation
- Each run of `bench.sh` moves both arms for about 32 seconds: first raising them by about 8 cm, then swinging the 14 joints in both directions, and finally returning to the start point. Every run must be explicitly authorised by the operator.
- The protection of the robot body stays enabled: a tracking error above 0.15 rad, a temperature above 75 °C or stale feedback all abort, and qtau is ramped down to 0 before ending. Do not loosen these thresholds in order to make a test pass.
- While there is no `scene` yet, planning cannot check the table surface clearance. In this case, first confirm manually that the space below the arms is empty, then add `PLAN_ARGS=--no-scene-ok`.

## Two send paths
- `direct` (path name): connects directly to Runtime at 250 Hz and does not need complete geometry. It is the only option while there are no scene parameters yet.
- `adapter`: exactly the same send path as the chess service (50 Hz); it needs complete geometry.

## Steps
All commands are run from the package root, with `S=setup/joint-tracking/scripts`; `python` is the environment with the dependencies installed (`bench.sh` uses `R1D_CHESS_PYTHON`, or `python` on PATH when it is not set).

**A. arm-calibration completed: verification**

1. Run the baseline twice and the full feedforward twice, to estimate the scatter between repeated runs:
   ```bash
   export BENCH_DIR=runs/tracking_benchmark PLAN_ARGS=--no-scene-ok   # remove PLAN_ARGS when scene already exists
   $S/bench.sh direct off
   FF_CONFIG=config/arm_torque_ff.json $S/bench.sh direct full
   FF_CONFIG=config/arm_torque_ff.json $S/bench.sh direct full
   ```
2. Look at the results after each run:
   - `python -B $S/joint_report.py <run>`: the static error and RMS of each joint.
   - `python -B $S/diagnose.py <run>`: joints whose ratio of error to velocity (e/v) is far greater than kd/kp have a gravity residual or friction problem; those whose maximum error appears in the low-speed final segment have a stiction problem.
3. Finish once the acceptance criteria below are met; when individual joints do not meet them, do one or two rounds of friction iterative learning as in step 5 of B.

**B. Tuning from neutral initial values**

1. Start from the template; do not reuse parameters tuned on another robot:
   ```bash
   mkdir -p runs/tracking_benchmark/ff
   cp setup/joint-tracking/templates/arm_torque_ff.initial.json runs/tracking_benchmark/ff/iter0.json
   ```
2. Run the baseline: run the same configuration twice.
   ```bash
   export BENCH_DIR=runs/tracking_benchmark PLAN_ARGS=--no-scene-ok
   $S/bench.sh direct off
   $S/bench.sh direct off
   ```
3. Add gravity only:
   ```bash
   FF_CONFIG=$BENCH_DIR/ff/iter0.json $S/bench.sh direct gravity
   ```
   Then use `python -B $S/model_fit.py <run>` to see whether the gravity model is too small (a run whose error has already converged works best). Write the joints whose least-squares scale is between 0.9 and 1.3 into `gravity.scale`; for those outside this range, check the URDF or the load first, and do not simply force the scale.
4. Identify the Coulomb friction: use the gravity-only runs.
   ```bash
   python -B $S/friction_fit.py <gravity run> [<another gravity run>]
   ```
   Multiply the output values by 0.8 and write them into `friction.coulomb_nm`, and set `error_epsilon_rad=0.01`.
5. Run the full feedforward and iterate:
   ```bash
   FF_CONFIG=$BENCH_DIR/ff/iter1.json $S/bench.sh direct full
   python -B $S/ilc_coulomb.py <run> iterN.json iterN+1.json --gain 0.5     # one friction iterative learning pass
   ```
6. Convergence check: stop iterating when the improvement between two consecutive versions is smaller than the scatter of repeated runs (about ±1 mrad).
7. **Confirm once on the adapter path**: after the scene parameters are completed, run `$S/bench.sh adapter full` once, to confirm that the send path the chess service actually uses gives the same result.
8. Write the final configuration: copy the final version to `config/arm_torque_ff.json`, and state the directory of that final run in `evidence`. You must confirm that the four `enabled` of `gravity`, `friction`, `integral` and `damping` are all true: the `--ff full` of the comparison tool turns them on temporarily, while the chess service reads the file itself.

## Acceptance criteria
| Item | Criterion |
|---|---|
| Static error of each joint in the dwell window | ≤ 0.012 rad |
| Single-joint RMS error | ≤ 0.015 rad |
| Maximum transient error | ≤ 0.05 rad (the latched fault threshold of the chess service is 0.20) |
| TCP RMS error | ≤ 8 mm |
| Temperature | ≤ 60 °C |
| `wire.mean_abs_wire_tau_minus_qtau_nm` | ≈ 0, proving that the torque really reaches the motors |

## Outputs
- `config/arm_torque_ff.json`
- `runs/tracking_benchmark/runs/*`, `ff/iter*.json`

## Common pitfalls
- **The start point drifts over consecutive runs**: every run ends stopped with a residual error. `bench.sh` takes a new snapshot and replans every time; run the same configuration at least twice before comparing.
- **The integral window cannot be smaller than the initial sag**: otherwise the integral never starts. The template uses 0.25 rad.
- **Feedforward being effective does not mean the model is correct**: gravity is computed at the commanded angle. During tuning, a motor estimated torque far smaller than the model usually means stiction is holding the joint, and the scale must not be reduced because of it.
- **The TCP error depends on the TCP definition**: the TCP error in `joint_report.py` uses the current TCP of `robot.json`; after the TCP is changed, results from before and after cannot be compared directly.
- **The profile used by the comparison tool** (`BENCH_PROFILE`) is only for the clearance check of the comparison test and cannot be used for playing.

> Reference (reference robot): without feedforward, the joint RMS error is 49–71 mrad and the TCP error 27–40 mm; with full feedforward, the joint RMS is about 10 mrad and the TCP RMS about 6 mm, and after switching to the dynamics URDF fitted by arm-calibration the TCP RMS drops to about 2.5 mm.
