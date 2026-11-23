# Base and lift feedback: the DDS topics used by the chess service

English | [中文](r1d_twist_dds_topics.zh.md)

The chess service does not control the base or the lift. It only reads their feedback, to confirm that the lift height is at the reference value and that the base and the column are stationary. This document describes the meaning of this feedback and the criteria in the service. Models without a base do not publish these topics.

## Topics used

| topic | Type | Rate | Purpose |
|---|---|---|---|
| `rt/state_vel` | `geometry_msgs::msg::dds_::Twist_` | 50 Hz | Measured velocity of the base and the column, to determine whether they are stationary |
| `rt/height_state` | `geometry_msgs::msg::dds_::Point32_` | 50 Hz | Column height |
| `rt/height_alarm` | `unitree_go::msg::dds_::Error_` | 50 Hz | Column alarm, to determine whether the height is trustworthy |

All three come from measured motor feedback. The chess service does not publish `rt/cmd_vel` (the velocity command of the base and the column, normalised to [-1, 1]).

## `rt/state_vel`: velocity feedback

```text
Twist_
├── linear.x      measured forward/backward linear velocity, normalised, relative to the maximum velocity of the current gear
├── linear.z      measured column lift velocity, normalised, positive upwards
├── angular.z     measured steering angular velocity, normalised, left positive, right negative
└── other components  always 0
```

- **The values are normalised to [-1, 1], not m/s or rad/s.**
- There is a dead zone before publishing: 0.005 for `linear.x` and `linear.z`, and 0.02 for `angular.z`. The dead zone of the angular velocity is wider, because differential-drive kinematics amplifies the velocity difference between the left and right wheels, and reading spikes at standstill can reach about 0.012.
- To determine "really moving", do not only check whether the value is non-zero; use thresholds with margin: `|linear| > 0.01`, `|angular.z| > 0.025`.

## `rt/height_state`: column height

```text
Point32_
├── x / y     always 0
└── z         column height, in mm
```

- The origin is 5 mm above the mechanical lower limit, positive upwards; the full travel is 850 mm, and the soft limits are about [0, 840] mm.
- **`z = 0` cannot distinguish "height unknown" from "really at the zero point"**: 0 is also published when the column is not homed or the feedback is invalid, so it must be judged together with `rt/height_alarm`.

## `rt/height_alarm`: column alarm

```text
Error_
├── source    fixed at 200 (base module)
└── state     alarm bit mask
```

| bit | Name | Meaning |
|---|---|---|
| 0 | NotHomed | The column is not homed; the height is not trustworthy |
| 1 | Lower | At or below the lower soft limit |
| 2 | Upper | At or above the upper soft limit |
| 3 | Stall | A sustained stall is detected in the not-homed state |

- Conditions for the height to be trustworthy: `source == 200`, `(state & bit0) == 0`, and `z` is a finite value.
- This alarm covers only the column; it is not an alarm bus for the whole robot. For motor faults, see `motor_state[i].motorstate` of `rt/lowstate`; base emergency stop and free pushing are not published on DDS.

## Use in the chess service

- **Runtime conversion**: Runtime converts these topics into the `base_group` state. The velocity is forwarded unchanged (normalised values); `lift_height = height_state.z / 1000` (in m) is given only when the height is trustworthy (the three conditions above), and is `None` otherwise.
- **Lift height**: the transport layer (`_check_reference` in `src/r1d_chess/robot/r1d_transport.py`) requires `lift_height` not to be `None`, and to differ from `lift_height_m` in `config/robot.json` by no more than `lift_tolerance_m` (5 mm). When the column is not homed, Runtime gives `None` and the service refuses to run, so no separate check of the alarm bits is needed.
- **Stationary check**: when `linear.x` or `linear.z` exceeds 0.01, or `angular.z` exceeds 0.025 (normalised values), the base or the column is considered to be moving, and the service refuses to take over.
- The lift height and the stationary state are checked only at the take-over entry points (`enable_control`, `handover`), and are not checked again during the session; do not move the base or the lift while a game is being played.
- The chess service does not select the base and lift control groups, and the program does not hold the lift in closed loop. When the lift is not at the reference height, first bring the lift to `lift_height_m` with `bash r1d goinit` (see the [operation manual](operation_manual.md)).

## Quick reference

| Concept | Value |
|---|---|
| `rt/state_vel` linear velocity dead zone / angular velocity dead zone | 0.005 / 0.02 |
| Stationary criterion (linear velocity / angular velocity) | 0.01 / 0.025 |
| Column full travel / soft limits | 850 mm / [0, ~840] mm |
| Column maximum lift velocity | 0.08 m/s |
| `rt/height_alarm` source identifier | 200 |
| Feedback rate | 50 Hz |
