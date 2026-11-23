# Joint accuracy calibration: dual-arm dynamics and waist and head hold parameters

English | [中文](README.zh.md)

## What is calibrated

| Object | Identified quantities | Result file | How the chess service uses it |
|---|---|---|---|
| The 14 joints of both arms | Link mass and centre of mass, Coulomb friction, load-dependent friction, viscous friction | `r1_d/assets/r1d/r1d_calibrated.urdf` | The gravity term and the friction term of the torque feedforward (`config/arm_torque_ff.json`) |
| Waist yaw, head pitch, head yaw | Dry friction, head gravity hold torque, motor gains | `r1_d/assets/r1d/hold_joints.json` | The waist and head hold feedforward (`config/hold_joints.json`) and the waist and head gains of Runtime |

What is calibrated here is "joint positioning accuracy": making the motors stop accurately at the commanded angle. It does not change the kinematics (FK/IK still use the official `r1d.urdf`), and it does not calibrate the zero positions.

## Tools

`r1_d/` is a standalone direct-connection toolset. It does not go through the robot runtime (Runtime, `third_party/robot_runtime`) or the chess service, and sends LowCmd over DDS itself:

```text
r1_d/
├── config.json        robot IP, network interface, transport (dds direct connection or ssh forwarding)
├── sdk/               connection, state, protocol encoding, control session, dynamics, self-collision check
├── tools/             console, joint test, height test, waist and head calibration, unload
├── calibration/       dual-arm exploration (explore), fitting (fit), held-out evaluation (heldout, compare)
├── assets/r1d/        official r1d.urdf, fitting result r1d_calibrated.urdf, hold_joints.json, meshes
├── tests/             offline tests
└── third_party/       bundled unitree_sdk2_python
```

The `r1d_calibrated.urdf` and `hold_joints.json` in `assets/r1d/` are the results of the reference robot, and are overwritten by the results of this robot after calibration is completed.

## Prerequisites

- **Stop Runtime and the chess service** (`systemctl --user stop r1d-runtime.service`, Ctrl-C for the chess service). If both send commands at the same time, they fight each other.
- Fill in `ip`, `interface`, `transport` in `r1_d/config.json` for this machine (`interface` ships empty and must be filled in). For a direct wired connection use `"transport": "dds"`; over Wi-Fi use `ssh` forwarding, which requires passwordless login to be set up first.
- When the vendor motion control service needs to be released, add `--release-mode` to the command; by default it only queries and does not switch.
- **Move the table and the board away**: exploration makes both arms cover the whole joint range.
- The lift height is not lower than 0.40 m.
- For dependencies, see `r1_d/requirements.txt` (numpy, scipy, cyclonedds); they are compatible with the chess environment, and the same Python can be used.

## Safety and authorisation

- All commands with motion must be explicitly authorised by the operator, and someone is nearby who can cut the power at any time.
- The exploration paths are checked for self-collision during offline planning (arms against torso, head, column and base), but external obstacles are not known.
- A disconnection reconnects automatically and continues from the point of interruption; a tracking error above `--guard` stops the current waypoint.
- After finishing, the arms stay at the last position and hold; they are not unloaded automatically. To unload, use `python tools/unload_arms.py` (look at the read-only preflight first, then add `--execute`); before unloading, support the arms or clear the space they fall into.

## Steps

The following commands are run in `setup/arm-calibration/r1_d/`, and the data goes into `<package root>/runs/arm_calibration/` (written as `$O` below).

**0. Read-only check and offline tests**
```bash
python -B -m unittest discover -s tests -t ..     # offline, without connecting to the robot
python tools/run.py --check                        # read-only: URDF, state topics, motion control mode
```

**1. Dual-arm exploration** (large-range motion of both arms)

Each round first plans collision-free waypoints offline, then executes on the robot and records the angle, velocity and torque of every joint:
```bash
python calibration/explore.py plan $O/plan1.json --count 160 --seed 1 --height <current lift height m> \
    --head-pitch 0 --gravity-urdf assets/r1d/r1d.urdf
python calibration/explore.py run  $O/plan1.json $O/r1 --gravity-urdf assets/r1d/r1d.urdf --hours 1
```
- `--height`, `--waist`, `--head-pitch` must match the actual pose at execution, otherwise execution refuses. With the head level (pitch 0), the usable range of the shoulder joints is largest.
- The first round uses the gravity of the official `r1d.urdf` for feedforward; each later round uses the URDF fitted in the previous round (`--gravity-urdf`).
- Do at least three rounds, changing `--seed` for each round: the first two rounds are used for fitting, and the last round is held out and does not take part in the fitting.

**2. Fitting**
```bash
python calibration/fit.py fit $O/r1 $O/r2 --output $O/fit_a.urdf --record $O/fit_a.json
```
Model: in slow motion `tau = g + (c + mu·|g|)·sign(dq) + b·dq`, where `g` is determined by link mass and centre of mass. Weighting by 1/kp is equivalent to minimising the position error caused by wrong feedforward; parameters that the data cannot observe keep the values of the official URDF.

**3. Held-out validation**
```bash
python calibration/heldout.py assets/r1d/r1d.urdf $O/fit_a.urdf -- $O/r3     # torque residual of each URDF on the held-out data
python calibration/fit.py evaluate $O/fit_a.urdf $O/r3 --record $O/eval_a.json
python calibration/compare.py $O/r3                                           # stop error at each waypoint
```
Stop iterating when the held-out error no longer decreases noticeably; if it is still decreasing, capture another round and put more data into the fitting.

**4. Install and run the joint test**
```bash
cp $O/fit_a.urdf assets/r1d/r1d_calibrated.urdf
python tools/run.py joint                  # the 14 joints go through their full travel one by one, then the two grippers are tested, about 14 minutes
```
- The joint test starts from the base pose (both arms hanging down, elbows at 80°); each joint goes to the midpoint and the end point on both sides of its travel and then returns, and the stop error, the return error, the backlash and the deviation of the other joints are recorded. The default pass line is 0.5°.
- **Do it with the head level**: with the head at the playing view, large shoulder pitch angles hit the head.

**5. Waist and head** (waist and head move, both arms hold)
```bash
python tools/run.py hold arms --pose hang                               # both arms to the hanging pose
python tools/run.py hold sweep --joint waist_yaw  --report $O/waist_sweep.json
python tools/run.py hold sweep --joint head_pitch --report $O/head_pitch_sweep.json
python tools/run.py hold sweep --joint head_yaw   --report $O/head_yaw_sweep.json
python tools/run.py hold fit --records $O/waist_sweep.json $O/head_pitch_sweep.json $O/head_yaw_sweep.json
python tools/run.py hold verify --joint waist_yaw  --period 0.02 --report $O/waist_verify.json
python tools/run.py hold verify --joint head_pitch --period 0.02 --report $O/head_pitch_verify.json
python tools/run.py hold verify --joint head_yaw   --period 0.02 --report $O/head_yaw_verify.json
```
- `sweep`: a single joint moves back and forth at several constant speeds, and the torque is recorded. `fit`: fits the dry friction, the viscous friction and the position-dependent hold torque, and writes them into `assets/r1d/hold_joints.json`. `verify`: goes to random targets and reports the stop error.
- `--period 0.02` makes the feedforward update at the 50 Hz of the chess service, so what is verified is exactly the behaviour during play.
- **First check the actual rotation range of this robot's waist**. The `RANGE` in `tools/hold_calib.py` holds the range of the reference robot (waist about ±0.56 rad, limited by the mechanical stops); when a new robot differs, change it first, or narrow the sweep range with `--centre` and `--amplitude`.
- Do not use the 50 Hz feedforward to add "additional stiffness"; it self-excites into oscillation and shakes both arms with it; put stiffness only in the motor gains. The tool has oscillation protection, and once it triggers, the current verification stops.

**6. Install the results into the chess configuration** (run from the package root)
```bash
python setup/arm-calibration/scripts/install_results.py                 # dry run: prints what will be changed
python setup/arm-calibration/scripts/install_results.py --write --evidence "runs/arm_calibration"
systemctl --user restart r1d-runtime.service                           # restart Runtime so that the waist and head gains take effect
```
It copies the fitted URDF to `assets/models/r1d/r1d_arm_dynamics.urdf`, writes the friction coefficients in the URDF into `config/arm_torque_ff.json` (gravity scale set to 1; integral, damping and clamping unchanged), copies `hold_joints.json`, and updates the waist and head gains in `config/runtime.json`.

**7. Verify on the Runtime link**: run the comparison as in part A of [joint-tracking](../joint-tracking/README.md), to confirm that the tracking error meets the criteria on the link actually used for playing.

## Acceptance criteria

| Item | Criterion |
|---|---|
| Dual-arm joint test | All 14 joints PASS (stop error ≤ 0.5°) |
| Held-out data | The torque residual and the stop error of the new URDF are clearly smaller than those of the official URDF, and no longer decrease noticeably when more data is added |
| Waist and head `verify` | The maximum stop error is within a few milliradians, and the oscillation protection is not triggered |
| After take-over | After the chess service takes over and settles, the waist and head are ≤ 5 mrad from the reference position, and both arms are ≤ 20 mrad from init (`bash r1d start` checks this) |

## Notes

- The calibration results change over time (reducer run-in, temperature); redo the calibration when the stop error becomes clearly larger.
- The calibration is done with no load and at one lift height; the accuracy under load and at other heights is not covered. A piece weighs only 10–20 g and can be ignored.
- `--ignore-link-loss` is a switch for robots on which the communication of individual motors is unstable; it is not needed by default.
- The `ip` and `remote_python` in `r1_d/config.json` are the defaults on the robot side; change them when the robot is configured differently; `provenance/` holds the records of the code origin and is unrelated to running.

> Reference (reference robot):
> - The three rounds of dual-arm exploration total about 400 waypoints and about 1.7 hours of actual capture; on the held-out data the stop error has a mean of 0.078° and a maximum of 0.43°; the maximum error of the joint test is 0.36°.
> - The waist stiction is about 0.6 Nm; the head pitch needs a gravity hold torque of about 1 Nm at the playing view, and sags by about 20 mrad without feedforward. The motor gains are waist 150/6, head pitch 150/4, head yaw 40/0.4.
> - Maximum error over 25 random stop points: waist 7.5 → 3.2 mrad, head pitch 19.4 → 1.5 mrad, head yaw 7.5 → 4.2 mrad. Over multiple starts on the chess service link (including cold starts), the waist is 0.6–0.8 mrad from the reference position and the head 0.3–1.4 mrad.
> - On re-measurement after some time, the dry friction of individual joints had changed noticeably (left wrist pitch from about 0.19 Nm to about 0.77 Nm), and the accuracy was restored after refitting.
