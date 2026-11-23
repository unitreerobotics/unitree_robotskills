# R1-D arm torque feedforward: purpose, control law and limits

English | [中文](r1d_joint_tracking_feedforward.zh.md)

## Why feedforward is needed

The robot runtime (Runtime, `third_party/robot_runtime`) applies only PD control to the joints. When the sustained load torque is not compensated, the arm tracking error is large enough for the tracking protection to abort a move midway. There are three causes:

| Cause | Symptom |
|---|---|
| Gravity is not compensated | The static error multiplied by kp equals the motor's self-reported torque `tau_est` |
| Coulomb friction and stiction | The stop position changes sign with the direction of approach; the largest error appears in the deceleration phase |
| Damping drag | Runtime fixes the `dq` target at 0, so `kd·dq` brakes throughout the motion |

The send path is not a cause: with the same feedforward added, the 50 Hz adapter path and the 250 Hz direct path give the same result.

## Control law

The implementation is in `src/r1d_chess/robot/r1d_torque_ff.py`, and the parameters are in `config/arm_torque_ff.json`. The service sends it every 20 ms together with the joint targets:

```
qtau = scale · g(q_cmd)                                              # gravity, computed at the commanded angle
     + (coulomb + load·|g|) · tanh(dq_cmd/ε_v + (q_cmd − q)/ε_e)      # friction; after stopping it still helps the joint past stiction in the direction of the error
     + kd · dq_cmd                                                   # cancels the damping drag caused by Runtime's dq=0
     + clip(∫ ki·(q_cmd − q) dt, ±limit)                             # integral: accumulates only while the error is inside the window
finally clamped per joint (tau_limit_nm), with a rate of change no greater than tau_rate_nm_s
```

- Gravity is computed with the dynamics URDF specified by `model_urdf` (`assets/models/r1d/r1d_arm_dynamics_reference.urdf` in the package, with link mass and centre of mass fitted on the real robot), including Dex1. It is used only for torque; FK/IK still use the kinematics URDF.
- Each term can be switched on and off separately; with `enabled=false`, qtau=0.
- Feedforward is active both during execution and while holding between rounds. The integral is cleared when the acting arm is switched, when a fault occurs, or when the session ends.
- When the session ends normally or on a fault, qtau is first ramped down to 0 at the rate limit before ending (the motors keep the last command frame after Runtime exits).
- `get_state` returns the state of `arm_torque_ff` and each of its terms, which can be used to check whether it is in effect.
- The transaction engine cannot command torque directly: the `tau` of `command_arm` must be all 0, and feedforward is computed only by the transport layer according to the configuration.

## Tuning on a new robot

`config/arm_torque_ff.json` and the dynamics URDF are the fitting results of one reference robot. Link mass and friction vary between individual robots, so a new robot should be re-identified: first fit the dual-arm dynamics URDF with [setup/arm-calibration](../setup/arm-calibration/README.md), then verify and fine-tune on the Runtime link with [setup/joint-tracking](../setup/joint-tracking/README.md). The comparison test is done as follows:

1. Starting from the measured start point, raise the TCP vertically by a few centimetres to a centre pose.
2. Near the centre pose, make all 14 joints perform small bidirectional swings at the same time, and dwell for 2 s after each motion segment to measure the static error.
3. Compare the joint RMS error and static error in the modes "no feedforward / gravity only / gravity plus integral / all", and adjust term by term.
4. Keep the tracking error, over-temperature and stale feedback protections throughout, and ramp qtau down to 0 before ending.

Without re-tuning, first confirm with `get_state` and one dry-run path acceptance (a move executed without a piece) that the tracking error is within the limits, then do pick and place.

## Tracking limits

| Limit | Value | Description |
|---|---|---|
| Latched tracking error fault during operation `arm_tracking_limit_rad` (checked every 20 ms) | 0.20 rad | Far greater than the transient error with feedforward, and still far below the error in a collision or jam |
| Joint deviation at the execution start point | 0.08 rad | Covers the scatter from holding between rounds, switching arms and stiction stop positions |
| TCP deviation at the execution start point | 15 mm | Between the static error and the maximum transient error |
| Velocity at the execution start point | 0.08 rad/s | Determines whether the arm is stationary |
| Runtime velocity limit, command timeout 0.15 s, over-temperature 75 °C | — | Protection of the robot body |

Do not loosen these thresholds in order to pass the checks.

## Notes

- The wrist pitch limit of the kinematics URDF is only ±0.5236. It is a conservative software limit and restricts the reachable range of IK.
- The friction and gravity parameters are identified within a limited range of poses; in very different poses (such as the arm extended far out), the residual is compensated mainly by the integral.
- The results are for no load. A piece weighs only 10–20 g, and its effect is negligible.
- With the existing PD gains of Runtime, there is a lower bound on the error that feedforward can reach; to reduce it further, the arm kp must be raised or Runtime must send a non-zero dq target.

> Reference (reference robot, no load, maximum over the 14 joints):
>
> | Feedforward | Joint RMS error | Maximum static error | TCP RMS error left/right |
> |---|---|---|---|
> | None | 49–71 mrad | 59–138 mrad | 27–40 / 20–29 mm |
> | Gravity only | 28–29 mrad | 44–46 mrad | about 16 / 16 mm |
> | Gravity + integral | 13 mrad | 14 mrad | 6.8 / 7.7 mm |
> | All (gravity, friction, damping compensation, integral) | about 10 mrad | about 10 mrad | 5.7 / 6.2 mm |
>
> With everything enabled, the maximum transient joint error is 33–38 mrad and the maximum transient TCP error is about 20 mm, occurring in the transitions between motions; the motor temperature does not exceed 55 °C. Without feedforward, one move was aborted in the approach phase because the tracking error exceeded the limit, and the TCP error at the completed waypoints reached 47–56 mm.
