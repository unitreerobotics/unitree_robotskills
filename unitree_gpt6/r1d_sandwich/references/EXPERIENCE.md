# R1-D sandwich experience

English | [中文](EXPERIENCE.zh.md)

See also: [Task route](TASK_SANDWICH.md), [Playbook](PLAYBOOK.md), [Results](RESULTS.md).

This file keeps the observations from the reference R1-A7-D runs between 2026-11-17 and 2026-11-25. It is
evidence for choosing and checking a route, not a set of coordinates that can be reused without looking at
the current scene. The route to follow is in [TASK_SANDWICH.md](TASK_SANDWICH.md); when a route described here
differs from it, the task file is the current one.

## Contents

1. [Role and service boundary](#role-and-service-boundary)
2. [Frame, TCP and gripper geometry](#frame-tcp-and-gripper-geometry)
3. [Reference scene](#reference-scene)
4. [Camera use](#camera-use)
5. [Route lessons](#route-lessons)
6. [Observed failure modes](#observed-failure-modes)
7. [Run discipline](#run-discipline)
8. [Run log, 2026-11-24 and 2026-11-25](#run-log-2026-11-24-and-2026-11-25)
9. [Routes used before 2026-11-24](#routes-used-before-2026-11-24)
10. [Where the time goes](#where-the-time-goes)

## Role and service boundary

- The agent reads the images, estimates positions, chooses the next waypoint and judges the result.
- The service owns IK, joint limits, self-collision clearance, table clearance, registered obstacle boxes,
  per-motion travel, execution and the watchdog. A refused waypoint did not move.
- The service checks a joint-limit margin of `0.03` rad, self-collision clearance of at least `20` mm, TCP
  at least `15` mm above the table, arm links at least `50` mm above the table over the table, and a maximum
  TCP travel of `0.6` m per motion.
- The service does not know where the plate, food, loose objects or people are. It cannot confirm grasp force,
  object attachment, plate contact or final placement.
- A watchdog trip latches `stopped`. Do not clear it from the agent side; report it and let the operator
  handle the fault.
- The fixed lift and the arm pose can leave the motors holding load between calls. Do not remain idle in a
  loaded posture while reasoning or evaluating candidates.

## Frame, TCP and gripper geometry

- Positions use the R1-D URDF base frame in metres: x forward, y left, z up.
- TCP is the midpoint between the Dex1 finger pads. At the standard pose, tool x points along the fingers,
  tool y along the opening direction and tool z up.
- `rpy_deg=[r,p,y]` means `R = Rz(y) * Ry(p) * Rx(r)` in the base frame.
- The fingertips extend `2.33` cm beyond TCP and a finger half-height is `2.0` cm. When a finger is `t`
  degrees from vertical, the lowest finger point is below TCP by
  `drop(t) = 0.0233*cos(t) + 0.020*sin(t)` metres.
- Subtract this drop when calculating a grasp height. On the first sandwich attempt, omitting it pushed the
  fingers too deep and caused a gripper stall against the toaster wall.
- Opening ranges from `0` closed to `1` fully open, about 9 cm. Use `action="grasp"` for an object. Do not
  use a sustained opening-0 command to squeeze an object; it can trigger a gripper fault.
- The reported opening after a grasp is evidence, not proof. For bread, `0.13` to `0.20` indicated a held
  slice and `0.02` to `0.06` indicated a miss. A thin leaf can give a small opening while still being held,
  so inspect the wrist image as well.

## Reference scene

These are measured reference values from the successful runs. Re-measure every placement.

| Item | Reference value |
|---|---|
| Table top | `z = 0.595` |
| Fixed lift | `0.522` m |
| Toaster | `26 x 15` cm, top about `z = 0.755`, centre about `(0.46, -0.30)` |
| Bread slice | about `9 x 9 x 2` cm in the first three runs, `12 x 12 x 2` cm from 2026-11-24, exposed to about `z = 0.785` |
| White plate | radius `8.75` cm, surface about `z = 0.61`, centre about `(0.34, -0.06)` |
| Lettuce | green leaf about 3 cm high, small dish about `(0.50, 0.20)` |
| Table edge | reference `table_x_min = 0.20` |
| Waist | keep at `0` for this route |

The right hand takes bread from the toaster and the left hand carries lettuce. In the final checked route,
the waist stays at `0`; an earlier attempt to turn the waist toward the toaster reduced reach and led to
missed grasps.

## Camera use

- The calibrated head camera is useful for the plate, lettuce and the bread when the arm does not occlude it.
  Its table-plane error was about `1.2` cm in the reference setup.
- The head camera did not see table positions with `x < 0.37`. It also failed when a wrist or forearm covered
  the target.
- The wrist cameras are for close alignment. An oblique wrist view overstates positional error. Put the hand
  above the target and look down as much as the fixed lift allows.
- The two wrist calibrations differed by about 3 cm. Measure a held object and its target with the same wrist
  camera.
- The toaster slot is more reliable from the right wrist than from the oblique head view. In the third run,
  toaster localisation and the first slice consumed `19.8` minutes and `31` model calls.
- A high wrist view at `z = 0.867` understated lateral error. An apparent 40-pixel offset was actually 1.5 cm
  and caused two missed grasps. Lower the open gripper and look again before closing.
- When a plate or object is clipped by the image edge, a mask centre can be biased toward the visible part.
  Use a complete target view, or reconstruct the centre from the far rim and the known radius, then check the
  estimate with a ruler mark.

## Route lessons

- Use only a joint-space `home` between stages. Returning with an end-effector pose selected another IK branch
  with about 48 degrees of shoulder abduction and an outward elbow. The right shoulder-yaw motor then carried
  about 4 Nm instead of the standard pose's 0.8 Nm and reached 84 C.
- Start each stage from the checked joint pose. Use `bash r1d eval` to compare posture and reachability before
  sending a new route.
- For flat bread release, entering from the plate's right side with rpy `[90, -10, 20]` kept the elbow below
  the shoulder. The alternate rpy `[-90, 10, 75]` raised the elbow above the shoulder.
- Reaching into the toaster at the fixed lift required about 80 degrees of shoulder abduction. Alignment and
  missed grasps made this posture last 99 seconds in the third run and 40 seconds in the second. Do not turn
  the waist to compensate.
- After the first slice is removed, the remaining slice moves in the image. Read it again before the next grasp
  and before the second slice is inserted.
- Carry lettuce high. A low joint-space carry pushed both dishes toward the robot by about 5 cm in the first
  run. The service does not model the dishes.
- When one hand holds an object over the plate, do not move the other arm.
- Use numeric feedback such as TCP position, opening and refusal code when it answers the question. Every image
  read consumes a model call.

## Observed failure modes

| Observation | Cause | Recovery |
|---|---|---|
| Gripper fault after entering the toaster | Grasp height ignored finger drop or the fingers contacted the wall | Recalculate the height, use `grasp`, open and retreat before retrying |
| Bread missed twice | Head estimate or high wrist view understated lateral error | Lower the open gripper, inspect the right wrist, correct the offset, then close |
| Bread landed on the plate rim | Plate centre was biased by occlusion | Take a complete head view or use the far-rim reconstruction, then verify with `look --mark` |
| Release dragged or scraped | Release height too low and the flat slice remained tilted | Keep the measured clearance above the support surface and slide out before fully opening |
| Right arm blocked by toaster while returning | The route crossed the registered obstacle at low height | Follow the proven exit points and verify the obstacle box after moving the toaster |
| Left elbow rose to shoulder height | The lettuce grasp and close carry need a tilted approach | Use the checked lettuce path and do not linger in the loaded posture |
| `linear: true` refused near reach limit | The straight segment left the reachable set | Return with `linear: false` to the preceding hover point, then approach on a checked route |
| `joint jump` or branch flip | The target orientation was too far from the current IK branch | Change yaw or approach through a checked transition point |
| A later command ran after an earlier refusal | A piped command returned the pipeline tail's status | Use `set -o pipefail` and `&&` for dependent commands |

## Run discipline

- Read `state` before the first motion and record the starting temperatures.
- Check the whole scene before the run: both hands above the table, waist near zero, the authorised mode,
  no latched stop and a reachable hardware emergency stop.
- Preview placement and exit candidates together before carrying the bread over the plate. Seven placement or
  exit waypoints were refused in the third run.
- Save a keyframe before and after a grasp and placement. Do not name a keyframe final until the image actually
  shows the intended result.
- If the service restarts, the log directory changes, or another operator moves an arm while an object is
  held, stop and report the inconsistency.
- Report the completed actions, image conclusion, grasp openings, refusals, measured placement offset,
  temperature and metrics path. State missing evidence rather than inferring success.

## Run log, 2026-11-24 and 2026-11-25

Notes written during the runs, in the order they happened. Later entries correct earlier ones.

### Plate too close

Evidence, run `20261124-204332-real-eef-cc`, plate centre at `(0.284, -0.02)`, rear rim at `x = 0.167`:

- A flat-release TCP nearer the body than about `(0.34, -0.06)` was refused (`17` to `20` mm between the lift column
  and the right shoulder link; joint 12 at its limit for yaw `30`). The first slice landed `3.7` cm beyond the plate
  centre and the top slice `8` cm beyond the lower one, resting on one corner. The run was abandoned.
- The push needs the TCP `4` cm behind the rear rim, here `x = 0.127`. The nearest pose that passed offline was
  `x = 0.178`.

With the plate centre at `(0.341, -0.026)` in the next run the same commands placed the first slice `1.7` cm from
the plate centre and the top slice about `2` cm from the lower one, and the two-hand push passed.

### Bread: 12 cm slices

Measure the slice before choosing the pull height. The fourth run (2026-11-24) used slices of about
`12 x 12` cm, not `9 x 9`. A pull to `z = 0.885` left the bottom edge at slot level, and the carry then
dragged the toaster and knocked it over, for both slices. The agent did not notice: it never looked at the
toaster after the pull.

Use this route after a successful grasp. It was checked offline with `bash r1d eval` for a grasp at
`(0.52, -0.28)`, yaw `-5`:

1. Straight pull to `z = 0.92`. This is the reach limit at lift `0.522`; `0.94` is refused.
2. Roll flat in place: joint-interpolate to the same position with rpy `[90, -10, yaw]`. The slice swings
   up to TCP height, so nothing hangs below the hand. Do not translate or descend while the slice is still
   vertical: at `z = 0.92` no sideways move toward the body is reachable with rpy `[0, 8, yaw]`.
3. Joint-interpolate to `(0.45, -0.15, 0.80)`, rpy `[90, -10, 15]`.
4. Joint-interpolate to the hover above the plate (see "Place flat").

The older route (pull to `0.885`, straight to `(0.46, -0.29, 0.87)` rpy `[0, 8, -12]`, then joint-interpolate
to `(0.40, 0.0, 0.84)` rpy `[0, 5, 15]`) is only valid for `9` cm slices.

With both slots loaded the slices are about `4.9` cm apart. At gripper `0.45` the outer finger lands on the
neighbouring slice; use gripper `0.32` for the approach so that finger fits between the slices.

Second attempt of the fourth run (2026-11-24, run `20261124-151802-real-eef-cc`), with this route:

- Both slices came out without touching the toaster. Grasps at `(0.52, -0.290)` and `(0.52, -0.334)`,
  yaw `-3`, opening `0.188` and `0.174`.
- Near the reach edge the straight descent `0.82 -> 0.79` can be refused as a branch flip. Splitting it at
  `0.805` passed.
- The roll pose is sensitive: `[90, -10, -8]` passed for the left slot and `[90, -10, -15]` for the right;
  `[90, -10, 0]` was refused.
- From the right slot, the arm arrived at `(0.45, -0.15, 0.80)` with the upper arm just under the `20 mm`
  self-collision margin. The service then refused every motion from that pose, `home` included, and the
  operator had to ease the elbow outward by hand. Preview this point from the right-slot roll pose and
  choose a via point with more clearance before using it again.
- With the plate at about `(0.415, -0.02)` the release at `z = 0.66` was refused (wrist `49 mm` above the
  table); `z = 0.68` passed. Release TCP `(0.385, -0.078)` put the first slice centre at about
  `(0.405, -0.02)`: offset `(0.02, 0.058)` for a `12` cm slice.
- The same release TCP for the second slice, on top of the lettuce, left it about `3.8` cm toward `-y` of
  the first. Check the held slice against the stack from the head camera before releasing the second slice.
- Third attempt (run `20261124-154720-real-eef-cc`): no contact with the toaster or the plate and no refusal.
  The right arm did not get stuck: `bash r1d eval` now prints `self_clearance_min_per_wp` (the old
  `link_gap_min_per_wp` is the table gap, not the self-collision clearance), and the route was chosen with
  at least `27 mm` at every waypoint. From the roll pose go straight to the hover above the plate with rpy
  `[90, -10, 10]`; skip the via point `(0.45, -0.15, 0.80)`. Yaw `0` to `10` gave `28` mm, yaw `20` gave
  `24` to `26` mm. A release TCP nearer the body than about `y = -0.05` was refused at `18` mm.
- The bread offset from the release TCP depends on where along the slice the fingers closed. Grasping at
  `x = 0.52` on a slice centred near `x = 0.57` gave a tool-frame offset of about `(0.069, 0.022)` instead
  of `(0.039, 0.048)`. At the pre-release hover, mark the predicted bread centre at `z = 0.70` and correct
  the TCP before releasing. This was done for the first slice (centre within about `2.5` cm of the plate
  centre) and skipped for the second, which landed about `3.9` cm from the first.
- Fourth attempt (run `20261124-160921-real-eef-cc`), same route with fewer, merged commands: no contact,
  no refusal, one grasp each. Checking the held slice at the pre-release hover did not fix the top-slice
  offset (about `4` cm again). The slice moves back toward the robot by `3.6` to `6` cm when the lower finger
  slides out, and that shift is not repeatable. Next thing to try: a lower release height for the top slice,
  or opening fully before the slide-out.
- A lettuce dish at about `(0.51, 0.28)` is out of reach for rpy `[0, 35, -45]`; `[0, 45, -37]` passed, with
  a joint-interpolated descent at the release point.

Fifth and sixth runs (2026-11-24, runs `20261124-172023-real-eef-cc` and `20261124-181646-real-eef-cc`):

- Do not skip the look at the pre-release hover. In the sixth run it was skipped to save time: the first slice
  landed `6` cm beyond the plate centre, on the far rim, and the repair took `253` s. Nudging the placed slice
  back with the side of the fingers recovered only `1.5` cm and dragged the plate `0.7` cm.
- Judge where the fingers sit along the slice only from the lowered view at `z = 0.805`. The high view at
  `z = 0.87` showed the slice about `5` cm nearer than it was, so the hand was moved back and closed on the rear
  end of the slice. The released slice then lies that much farther forward of the TCP.
- When a slice leans in its slot, turn the hand to the slice, not to the slot: a miss at yaw `12` (opening
  `0.023`) became a hold at yaw `6` with the approach opening raised from `0.32` to `0.55`.

Seventh day of runs (2026-11-25, run `20261125-162207-real-eef-cc`), toaster about `4` cm farther than the day
before. Two earlier attempts that day closed on nothing (opening `0.023`, `0.024`) and were abandoned:

- The head camera put the hand between the two slices: at the lowered view both slices were inside the gap.
  Choose the slice from the wrist view, not from the head projection, and never descend while both slices are
  between the fingers.
- The misses came from yaw, not from height or reach. The hand was at yaw `9`, the slice lay at about yaw `0`,
  and the opening was `0.32` (`2.9` cm). A `2` cm slice skewed by `10` degrees over the pad length needs about
  `2.7` cm, so a finger came down on the slice instead of beside it. Measure the skew with
  `look --cams right_wrist --z 0.785 --mark` (marks along the hand axis at the TCP and `2.3` cm either side):
  the slice centre must stay on the marks from the near mark to the far one.
- What held: hand yaw equal to the slice, slice centred between the fingers, opening `0.42` with two slices
  loaded and `0.6` with one, joint-interpolated descents `0.82 -> 0.805 -> 0.79`. Openings `0.181` and `0.166`.
- Pull to `0.92` in its own command and read the opening before rolling and carrying. A miss is then one short
  descent away instead of a trip to the plate and back.
- A straight descent `0.82 -> 0.805` at `x = 0.55` was refused as a branch flip; the joint-interpolated one passed.

### Lettuce

Do not joint-interpolate straight from `home` to the hover above the leaf. In the second attempt of the
fourth run that swing carried the open left fingertips through the centre plate and pushed it about 4 cm;
the lettuce and the second slice were then placed at the plate's old position.

Fix used in run `20261124-154720-real-eef-cc` (no contact):

1. After the first slice is placed, register the centre plate as an obstacle so the planner refuses any
   path through it:

```bash
curl -s --noproxy '*' -X POST http://127.0.0.1:8765/scene -d '{"obstacles":[{"name":"plate","center":[px,py],"size":[0.24,0.24],"top_z":0.66,"yaw":0.0,"margin_xy":0.02}]}'
```

   With `top_z = 0.66` the lettuce release at `z = 0.70` and the second-slice release at `z = 0.725` still
   pass. Register it only after the first slice is down: that release at `z = 0.68` is inside the box.
2. Joint-interpolate from `home` to `(0.509, 0.149, 0.80)`, rpy `[0, 0, 0]`: a rise in place.
3. Straight line at `z = 0.80` to above the leaf, `(lx, ly, 0.80)`, rpy `[0, 0, 0]`.
4. Joint-interpolate down to the hover `(lx, ly, 0.711)` with the grasp rpy. A straight descent from `0.80`
   was refused as a branch flip.

Where the leaf lands, with the release rpy `[0, 5, -75]`, measured in three runs on 2026-11-24. `(bx, by)` is the
centre of the lower slice:

| Release TCP | Result |
|---|---|
| `(bx - 0.01, by + 0.067)` | leaf covers most of the slice |
| `(bx - 0.01, by + 0.039)` | leaf covers the right half and hangs `5` cm over the `-y` edge |
| `(bx - 0.004, by + 0.029)` | leaf covers the right-front half only |
| `(bx - 0.01, by + 0.07)` | leaf covers the right half and hangs over the `-y` and near edges (fourth run) |

The leaf hangs toward `-y` of the fingers, by an amount that depends on where along the leaf the fingers closed.
The same offset `by + 0.07` covered most of the slice in one run and only the right half in another, so no fixed
offset is reliable. The formula under "Lettuce carry" below (`by + 0.039`) is too small in every case. Not yet tried:
looking at the held leaf from the head camera at the carry height and choosing the release point from where the
leaf actually hangs, the way the held bread is checked before release.

Tried once on 2026-11-25: at the hover the leaf stood up from the fingertips and its projected centre was `4` cm
toward `+x` and `4.5` cm toward `-y` of the TCP. The TCP was moved to put that centre on the bread. On release
the standing part fell back toward the robot, and the leaf covered the left and near half of the slice. The
projection of a leaf that is standing up overstates how far forward it will lie; correct `y` from the hover
view, and do not move the hand back in `x`.

### Two-hand plate push

Measured once, in run `20261124-181646-real-eef-cc`: both hands push together, the left with rpy
`[0, 15, -90]` and the right with `[0, 15, 90]`, fingertips facing each other behind the rim.

1. Bring each hand to its hover with single-arm waypoints as in the one-hand push below (left first, then right): TCP
   `x = rx - 0.04`, `z = 0.76`, `0.70`, `0.66`. Starting `y`: left `ry + 0.075`, right `ry - 0.045`.
2. Look once with a zoomed head crop. Shift each hand along `y` until the two fingertips sit about `2` cm either
   side of the plate's rearmost point, with table visible between the fingers and the rim. In the measured run
   the right fingertip appeared `4.6` cm farther toward `-y` than planned and the left `0.7` cm; the corrected
   TCPs were `y = 0.043` and `y = -0.034` for a rear point read at `y = -0.03`.
3. Lower each hand to `z = 0.624`.
4. Preview and run the push with the two-arm command, which moves both TCPs along straight lines at the same
   time:

```bash
bash r1d eef both-check '{"left":{"pos":[0.40,0.043,0.624],"rpy_deg":[0,15,-90]},"right":{"pos":[0.40,-0.034,0.624],"rpy_deg":[0,15,90]}}'
bash r1d eef both '{"note":"push plate","speed":0.1,"left":{"pos":[0.40,0.043,0.624],"rpy_deg":[0,15,-90]},"right":{"pos":[0.40,-0.034,0.624],"rpy_deg":[0,15,90]}}'
```

   From TCP `x = 0.188` an end point of `x = 0.40` passed (self-clearance `32` mm) and moved the plate about
   `20` cm in `7.4` s; `x = 0.41` was refused as a branch flip.
5. Lift both hands with `both` to `z = 0.78`, send each to `(0.45, +-0.15, 0.78)`, rpy `[0, 0, 0]`, then
   `home both`.

Lessons from that run, applied from the next run on:

- Bring both hands to the hover together with `both` and `"linear": false` (one joint-space path for both arms):
  first to `(0.509, +-0.149, 0.80)`, rpy `[0, 0, 0]`, then to the hover at `y = ry + 0.075` (left) and
  `y = ry - 0.075` (right), then `z = 0.70` and `0.66`. Shift both hands along `y` with a straight `both` move at
  `z = 0.66`. Going straight to the final `y` was refused offline (`17` mm between the right finger and the waist).
- Retreat with `both` as well: straight lines to `(0.45, +-0.15, 0.78)`, rpy `[0, 0, 0]`, then `home both`. A
  single-arm `exec` with two segments moves one arm after the other (`14` s). `home both` directly from the lifted
  push pose is refused (`15` mm between the two fingertips).

### One-hand plate push

Measured once, in run `20261124-172023-real-eef-cc`, with a `23` cm plate whose rear rim started at `x = 0.238`.

1. Clear the plate obstacle box (`POST /scene` with `{"obstacles":[]}`); the push ends inside it and is
   refused otherwise. While approaching, a box around the sandwich only (top `0.70`) can stay registered.
2. From `home left`: joint-interpolate to `(0.509, 0.149, 0.80)`, rpy `[0, 0, 0]`, gripper `0.1`.
3. Joint-interpolate to `(rx - 0.04, y, 0.76)`, rpy `[0, 15, -90]`, then to `z = 0.70`, then to `z = 0.66`,
   each as its own joint-interpolated waypoint. Arriving at `z = 0.80` instead selected a raised-elbow branch
   (shoulder abduction about `110` degrees) and the straight descent from it was refused.
4. Look with `left_wrist` and a zoomed head crop. White table must be visible between the front finger and
   the rim. If the finger hides the rim, the finger is above it: move back before descending.
5. Joint-interpolate down to `z = 0.624` and look again with `left_wrist`. The rim wall must be beside the
   front finger, not under it.
6. Straight push in `+x` at speed `0.08` for about `5` cm, then check in the head image that the plate moved
   and the sandwich did not.
7. Straight push to the end point at speed `0.10`. Measured: TCP `x` from `0.183` to `0.39` moved the plate
   about `19` cm, to a centre near `x = 0.53`, between the two `home` hand positions. An end point of
   `x = 0.41` also passed offline.
8. Straight lift to `z = 0.78`, joint-interpolate to `(0.45, 0.15, 0.78)`, rpy `[0, 0, 0]`, then `home left`.

Preview the whole sequence with `bash r1d eval` from the current pose before step 3 and again before step 6.

The first descent used TCP `x = 0.2135` for a rim at `x = 0.238`, a planned gap of `5` mm. The front finger
came down on the rim and dragged the plate `1.7` cm toward the robot. The hover image at `z = 0.66` had
already shown the finger covering the rim. The hand was lifted, the plate re-measured (rear rim at
`x = 0.224`), and the second descent at TCP `x = 0.183` landed behind the rim.

## Routes used before 2026-11-24

These points were measured in the first three runs with `9` cm slices. The plate may no longer be closer than
`x = 0.33`, so the closer-plate variants are outside the supported range. They are kept as measurements.

- Slot yaw was `0` for a straight toaster, `-12` in the second run and `-20` in the third. Third-run grasp
  centres: left slice `(0.486, -0.400)`, right slice `(0.465, -0.442)`, yaw `-20`.
- Approach: joint-interpolate to `(0.47, -0.15, 0.87)` rpy `[0, 0, 0]` with gripper `0.45`, then to
  `(0.40, -0.30, 0.87)` rpy `[0, 5, yaw]`, and read `right_wrist`. A vertical slot in the wrist image is correct.
- Pull for a `9` cm slice: up to `z = 0.885`, straight to `(0.46, -0.29, 0.87)` rpy `[0, 8, -12]`, then
  joint-interpolate to `(0.40, 0.0, 0.84)` rpy `[0, 5, 15]`.

### Place flat

Let the target bread centre be `(bx, by)`. The bread centre is offset from the release TCP, and the
offset depends on where the slice sits between the fingers.

| Plate reference | rpy | Measured release offset `(dx, dy)` |
|---|---|---|
| centre about `(0.30, -0.04)`, farther | `[90, -10, 20]` | `(0.035, 0.042)` |
| same | `[90, -10, 30]` | `(0.022, 0.040)` |
| centre about `(0.235, -0.04)`, closer | `[90, -10, 20]` | `(0.023, 0.043)` |

The estimate `TCP + R * (0.003, 0, -0.037)` understates the measured offset by about 4 cm because the
slice position between the fingers changes. Start from the measured offset and verify with the head
camera.

For a farther plate (`x` about `0.30`):

1. Joint-interpolate to `(0.313, -0.075, 0.716)`, rpy `[90, -10, 20]`.
2. Check the plate centre with `look --z 0.69 --mark`.
3. Descend to release `(0.265, -0.082, 0.66)`, gripper `0.45`.
4. Joint-interpolate to `(0.277, -0.114, 0.672)`, gripper `1.0`. The fingers slide out from the
   bread's right side without dragging it.
5. For the second slice, use rpy `[90, -10, 30]` and `(0.25, -0.08, 0.705)`, then the same
   `0.45` to slide-out `1.0` release.

For a closer plate (`x` about `0.235`), the farther-plate points were refused: reach limit and
self-collision margin `18 mm < 20 mm`. Use:

1. Set `(rx, ry) = (bx - 0.023, by - 0.043)`.
2. Joint-interpolate to `(rx - 0.027, ry - 0.009, 0.716)`, rpy `[90, -10, 20]`.
3. Check the plate with the head camera at `look --z 0.61 --mark`.
4. Straight descent to `(rx - 0.027, ry - 0.009, 0.662)`, then straight lateral motion to
   `(rx, ry, 0.662)`, gripper `0.45`. Going diagonally from the flat pose to release was refused.
5. Joint-interpolate to `(rx + 0.012, ry - 0.032, 0.674)`, gripper `1.0`.
6. For the second slice use the same points with descent `z = 0.695` and slide-out `z = 0.707`.
   All rpy-30 variants were refused; keep rpy `[90, -10, 20]`.

Third-run measurement: release TCP `(0.222, -0.078)` produced bread centre `(0.245, -0.035)`.
The two slices differed by about 1.3 cm.

### Exit

For a farther plate, after the first slice:

1. `(0.31, -0.09, 0.72)`.
2. `(0.33, -0.12, 0.74)`, rpy `[45, -5, 10]`.
3. `(0.45, -0.15, 0.78)`, rpy `[0, 0, 0]`.
4. `home right`.

After the second slice, first run `grip right 0.3`, then use:

1. `(0.32, -0.18, 0.74)`, rpy `[60, -5, 5]`.
2. `(0.40, -0.16, 0.76)`.
3. `(0.45, -0.15, 0.78)`.
4. `home right`.

For a closer plate, both slices use:

1. Straight lift to `(rx + 0.012, ry - 0.04, 0.72)`, same rpy.
2. Joint-interpolate to `(0.33, -0.12, 0.74)`, rpy `[45, -5, 10]`.
3. Joint-interpolate to `(0.45, -0.15, 0.78)`, rpy `[0, 0, 0]`.
4. `home right`.

Do not omit `(0.33, -0.12, 0.74)`. Replacing it with points such as `(0.30, -0.16, 0.76)` was refused.
Calling `home` before reaching `(0.45, -0.15, 0.78)` was refused because the wrist was above the table
in an unsafe path.

### Lettuce carry

1. Straight vertical rise to `z = 0.741`.
2. Straight vertical rise to `z = 0.785`.
3. Carry high. Low joint-space carry pushed both plates by 5 cm in the first run.

For a farther bread position (`x` about `0.30`):

1. Joint-interpolate to `(0.42, 0.14, 0.76)`, rpy `[0, 25, -20]`.
2. Joint-interpolate to `(bx - 0.04*cos(-75 deg), by - 0.04*sin(-75 deg), 0.76)`, rpy
   `[0, 5, -75]`.

For a closer bread position (`x` about `0.245`), yaw `-75` cannot reach. From `(lx, ly, 0.785)`,
joint-interpolate directly to `(bx, by + 0.04, 0.785)`, rpy `[0, 5, -90]`. This was used in the third
run and gave about `58` degrees of shoulder abduction; no better branch was found.


## Where the time goes

Measured on run `20261125-162207-real-eef-cc` (34.5 minutes, 38 model calls, plate push included):

- The robot moved for `178` s in total. One gap of `386` s had neither a robot command nor model output.
- The remaining 25 minutes were model calls, about `40` s each.
- After 13 motions the next call only opened the picture taken by that motion, and the call after it did the
  thinking. Those 13 calls took `440` s.
- The agent had written its own helpers in a temporary directory: a cropped view with pixel ticks, a call to
  `/project`, a one-line summary of `eval`, a script that carried only after a held grasp, and a two-arm
  preview. They are now `look --raw --crop`, `eef project`, `eval --brief`, the `min_opening` waypoint field
  and `eval` with `"side": "both"`.

See [RESULTS.md](RESULTS.md) for the completed runs and their measured limitations.
