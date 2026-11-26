# R1-D sandwich task route

English | [中文](TASK_SANDWICH.zh.md)

See also: [Playbook](PLAYBOOK.md) for commands and rules, [Experience](EXPERIENCE.md) for the measurements
behind the limits below.

## Contents

1. [Task and setup](#task-and-setup)
2. [Reference scene](#reference-scene)
3. [Route in brief](#route-in-brief)
4. [Bread slice notes](#bread-slice-notes)
5. [Lettuce notes](#lettuce-notes)
6. [Plate push notes](#plate-push-notes)
7. [Finish and report](#finish-and-report)

## Task and setup

The right hand takes one bread slice from the toaster and places it flat on the white plate. The left
hand picks lettuce from the small dish and places it on the bread. The right hand takes a second slice
and covers the lettuce.

Every waypoint is an explicit TCP position and rpy. The service solves it with its bundled IK. Do not use a
detector, a symbolic planner or a search over poses on the real robot.

Before starting:

1. Read [PLAYBOOK.md](PLAYBOOK.md), especially the posture, safety and command rules.
2. Obtain explicit user authorisation for the intended motion.
3. Confirm the operator is present, the workspace is clear, and the hardware emergency stop is reachable.
4. Start only the authorised mode. For a real run:

```bash
bash r1d serve --real --fixed-lift 0.522
```

5. Check `state`. Both hands must be above `z = 0.70`, waist near `0`, and the robot not stopped. Record
   the starting motor temperature.
6. Create a run, bring the robot to the start pose and begin recording. `prepare` sets the lift to `0.522` and
   pitches the head down to `0.486` rad; with the head level the camera does not see the near half of the table.
   Check `lift_m` and `head_pitch` in its output:

```bash
bash r1d eef new
bash r1d eef prepare
bash r1d eef rec-start
```

7. Read the head view and both wrist views from the current scene. Do not reuse yesterday's object
   coordinates.

8. Check the toaster. With both slots loaded, take the left slice first and the right one after the lettuce.
   With one slice loaded, ask the operator to put the second slice into the left slot, exposed by about
   `3` cm, before the second-slice stage.

## Reference scene

These are measured reference values from the successful runs. They are not fixed for a new placement.

| Item | Reference value |
|---|---|
| Table top | `z = 0.595` |
| Plate surface | about `z = 0.61` |
| Toaster body | `26 x 15` cm, top about `z = 0.755`, reference centre about `(0.46, -0.30)` |
| Bread slice | about `12 x 12 x 2` cm on 2026-11-24 and 25 (`9 x 9 x 2` cm before), exposed to about `z = 0.785` |
| White plate | radius `8.75` cm, reference centre about `(0.34, -0.06)` |
| Lettuce dish | reference lettuce about `(0.50, 0.20)` |
| Waist | keep at `0` for this route |

The head camera does not see table positions with `x < 0.37`. The wrist calibrations differ by about
3 cm, so measure a held object and its target with the same wrist camera.

### Plate distance limit

The plate centre must be at `x >= 0.33` (rear rim at `x >= 0.22`) for this route. Read the rear rim from the head
image before the first motion. If the plate is closer, ask the operator to move it before starting.

A plate centre at `(0.284, -0.02)` made the release points and the push start unreachable, and that run was
abandoned; at `(0.341, -0.026)` the same commands worked. The numbers are in [EXPERIENCE.md](EXPERIENCE.md).

The toaster box is a hard obstacle. After moving the toaster, redraw the four corners with `look --mark`
and verify the registered box before approaching. TCP `0.790` is the grasp lower bound; do not descend
closer to the toaster.

## Route in brief

This is the waypoint sequence of run `20261125-162207-real-eef-cc` (no refusal, one grasp per item), grouped so
that each step is one turn: one shell command that moves and then looks, and the read of `runs/latest/<cam>.jpg`
in that same turn. Every waypoint below ran on the robot in that run. The grouping is new: that run used 38
model calls for these steps, because step 3 was two commands and every picture was opened in a separate call.
A run with this grouping has not been measured yet.

The coordinates are that day's scene and show the shape of each motion. Read every position from the current
images. The notes after the tables give the limits for each step. The measurements behind them, and the routes
used before this one, are in [EXPERIENCE.md](EXPERIENCE.md).

Abbreviations: `J` joint-interpolated (`"linear": false`), `L` straight line, `g` gripper value.

### Start

| Step | Command | Read in the same turn | Decide |
|---|---|---|---|
| 1 | `new && prepare && rec-start && state && look --cams head_left --raw` | `head_left` | `prepare` reports `lift_m` `0.522` and `head_pitch` about `0.5`. Mode, not stopped, hands above `0.70`, waist `0`, starting temperature. Plate rear rim at `x >= 0.22`. Which slots hold a slice, where the lettuce dish is. |

### Each bread slice, right hand

With both slots loaded take the left slice first, then the right one.

| Step | Command | Read in the same turn | Decide |
|---|---|---|---|
| 2 | `exec`: J `(0.47, -0.15, 0.87)` rpy `[0, 0, 0]` with `g`; J `(0.47, -0.30, 0.87)` rpy `[0, 8, yaw]`; J above the slice `(sx, sy, 0.87)`; J `0.845`; J `0.82`. Then `look --cams right_wrist --raw`. | `right_wrist` | One slice between the fingers, centred, hand yaw equal to the slice. `g` is `0.42` with two slices loaded and `0.6` with one. |
| 2b | Only if step 2 is off: `exec` J up to `0.845`, the corrected `(sx, sy)` and yaw, J `0.82`, J `0.805`, then the same look. | `right_wrist` | As step 2. To measure the skew, use `look --cams right_wrist --z 0.785 --mark` with marks along the hand axis. |
| 3 | `exec`: J `0.805`; J `0.79` with `grasp`; L pull to `0.92`; J roll in place to rpy `[90, -10, -18]` with `"min_opening": 0.1`; J to the pre-release hover `(hx, hy, 0.716)` rpy `[90, -10, 20]`. Then `look --cams head_left --raw --crop` around the plate. | `head_left` | Opening `0.13` to `0.20`. The toaster has not moved. `project` the four corners of the held slice at its height and shift the hover by target minus measured centre. |
| 4 | `exec`: J corrected hover; L down to the release height with `g 0.45`; J slide-out `(+0.012, -0.032, +0.012)` from the release point with `g 1.0`; J up and away from the stack; J `(0.46, -0.12, 0.76)` rpy `[45, -5, 10]`; J `(0.45, -0.15, 0.78)` rpy `[0, 0, 0]`. Then `home right && keyframe ... && look --cams head_left --raw --crop`. | `head_left` | Where the slice landed: `project` its corners at `z = 0.625`. That centre `(bx, by)` is the target for the lettuce and the top slice. |

Values of that run: slices at `(0.55, -0.277)` and `(0.55, -0.32)`, yaw `0`. The first slice needed step 2b
twice (yaw `9` to `1` to `0`, `sy` `-0.281` to `-0.283` to `-0.277`); the second needed none. Hover and release
for the first slice were `(0.36, -0.07, 0.716)` and `(0.37, -0.085, 0.68)`; for the second, on the lettuce,
`(0.37, -0.08, 0.761)` and `(0.36, -0.065, 0.725)`. The point after the slide-out was `(+0.02, -0.03, +0.045)`
from the release point for the first slice and `(+0.021, -0.047, +0.045)` for the second. The held slice was
projected at `z = 0.713` and `0.76`. Openings `0.181` and `0.166`.

The roll pose and the two points after the slide-out are the ones most often refused. Preview step 3 and step
4 together with `bash r1d eval --brief` while the arm is still at `home`.

### Lettuce, left hand

| Step | Command | Read in the same turn | Decide |
|---|---|---|---|
| 5 | `exec`: J `(0.509, 0.149, 0.80)` rpy `[0, 0, 0]` with `g 1.0`; L `(lx, ly, 0.80)`; J `(lx, ly, 0.711)` with the grasp rpy; J `0.675`; J `0.637` with `grasp`; J `0.741`; J `0.785`; J `(0.42, 0.14, 0.76)` rpy `[0, 25, -20]`; J hover `(bx - 0.01, by + 0.07, 0.76)` rpy `[0, 5, -75]`. Then `look --cams head_left --raw --crop` covering the dish and the plate. | `head_left` | The dish is empty, so the leaf is in the hand. Where the leaf hangs: correct `y` from this view and do not move the hand back in `x`. |
| 6 | `exec`: J corrected hover; J down to `0.70` with `g 1.0`; J `0.77`; J `(0.42, 0.14, 0.77)` rpy `[0, 25, -20]`; J `(0.45, 0.15, 0.78)` rpy `[0, 0, 0]`. Then `home left && keyframe ... && look --cams head_left --raw --crop`. | `head_left` | The leaf covers the slice. |

Do not put `min_opening` on the lettuce: a held leaf reads `0.02` to `0.05` (`0.033` in that run). The grasp rpy
was `[0, 45, -37]` for a dish at `(0.48, 0.28)`.

### Push the plate, only when asked

| Step | Command | Read in the same turn | Decide |
|---|---|---|---|
| 7 | `grip left 0.1 && grip right 0.1`, then four `both` commands with `"linear": false`: `(0.509, +-0.149, 0.80)` rpy `[0, 0, 0]`; hover `(rx - 0.04, yl / yr, 0.76)` rpy `[0, 15, -90]` left and `[0, 15, 90]` right; `0.70`; `0.66`. Then `look --cams head_left --raw --crop` behind the plate. | `head_left` | Table is visible between each finger and the rim. |
| 8 | If the fingers are too close to the rim, `both` L back by `1` cm at `0.66`. `both` J down to `0.624`. `both-check` of the push end point. Then the same look. | `head_left` | The rim is beside the fingers, not under them. The check passed. |
| 9 | `both` L push to `x = 0.40` at speed `0.12`; `both` L lift to `0.78`; `both` L retreat to `(0.45, +-0.15, 0.78)` rpy `[0, 0, 0]`; `home both && keyframe final && rec-stop && look --cams head_left --raw`. | `head_left` | The plate moved and the sandwich did not. |

Values of that run: rear rim at `x = 0.244`, hands at `x = 0.204` and then `0.194`, `yl = 0.029`, `yr = -0.048`.
Preview steps 7 to 9 in one `bash r1d eval --brief` request with `"side": "both"` and `"end_at_start": true`
before step 7.

## Bread slice notes

### Finding the slice (step 2)

- Choose the slice from the right wrist view. The head camera sees the toaster at an oblique angle and was off
  by several centimetres; on 2026-11-25 its projection put the hand between the two slices.
- Never descend while both slices are between the fingers. With both slots loaded the slices are about `4.9` cm
  apart, centre to centre.
- Turn the hand to the slice, not to the slot: a slice can lean in its slot. A `2` cm slice skewed by `10`
  degrees over the pad length needs about `2.7` cm between the fingers.
- To measure the skew, use `look --cams right_wrist --z 0.785 --mark` with marks along the hand axis at the TCP
  and `2.3` cm either side. The slice centre must stay on the marks from the near mark to the far one.
- Keep the hand back along the slice: the TCP may be at most `3` cm past the near end of the slice. Only the
  front `6` cm of the fingers (the pads) can straddle a slice. From `3.7` to `5.7` cm behind the TCP the finger
  carriage spans the full `7.8` cm width of the gripper, and it comes down on the slice top. The signs are a hand
  that stops about `1.5` cm above the target (`0.805` for `0.79`) and a grasp opening of `0.02`. On 2026-11-26
  this cost five missed grasps, until the hand was moved back from `x = 0.56` to `0.48` for a slice whose near
  end was at `x = 0.46`. Find the near end from a wrist view taken `2` to `3` cm behind it.
- Approach opening: `0.42` with two slices loaded, `0.6` with one.
- Judge the position only from the lowered view at `z = 0.82` or `0.805`. The view from `z = 0.87` showed the
  slice about `5` cm nearer than it was and understated the sideways error: `40` pixels were `1.5` cm.
- After the first slice is out, read the remaining slice again. Its position in the image changes.
- Do not turn the waist to reach the slice. A `-0.3` rad turn could not reach the depth, caused two misses and
  took a motor to `84 C`.

### Grasp, pull and roll (step 3)

- Descend with joint-interpolated waypoints `0.82 -> 0.805 -> 0.79`. Near the reach edge (`x = 0.55`) a
  straight descent was refused as a branch flip.
- The TCP target `0.790` is the lower bound; the measured TCP stops at about `0.795`.
- Opening `0.13` to `0.20` means held and `0.02` to `0.06` means missed. After a miss, open, look with the wrist
  camera, correct the offset and try again. `"min_opening": 0.1` on the roll waypoint keeps a missed grasp
  above the toaster.
- With the hand back at `x = 0.48` a pull to `0.92` is out of reach; `0.91` passed. On that pull the first slice
  turned the toaster by about `20` degrees on its way out, so read the remaining slice again before the next
  grasp.
- Pull straight up to `z = 0.92`. This is the reach limit at lift `0.522`; `0.94` is refused. A pull to `0.885`
  left the bottom edge of a `12` cm slice at slot level, and the carry then knocked the toaster over.
- Roll flat in place before moving sideways: the slice swings up to TCP height, so nothing hangs below the
  hand. At `z = 0.92` no sideways move toward the body is reachable while the slice is still vertical.
- The roll pose is sensitive. `[90, -10, -18]` passed on 2026-11-25; the day before `[90, -10, -8]` passed for
  the left slot, `[90, -10, -15]` for the right slot, and `[90, -10, 0]` was refused. Preview it.
- Go from the roll pose straight to the pre-release hover. Do not pass through `(0.45, -0.15, 0.80)`: from the
  right slot the upper arm arrived there just under the `20` mm self-collision margin, the service then
  refused every motion, `home` included, and the operator had to move the elbow by hand.
- Check `self_clearance_min_per_wp` in the preview. The route that did not get stuck kept at least `27` mm at
  every waypoint.
- After the pull, check in the head view that the toaster has not moved. If it moved, stop and tell the
  operator.

### Place and exit (steps 3 and 4)

- Measure the held slice at the pre-release hover and correct the TCP before releasing. The offset between the
  release TCP and the slice centre depends on where along the slice the fingers closed, so no fixed offset
  works. Skipping this look once put the slice `6` cm beyond the plate centre and cost `253` s.
- Release height for the first slice: `z = 0.68`. `z = 0.66` was refused with the plate at about
  `(0.415, -0.02)` (wrist `49` mm above the table). The top slice was released at `z = 0.725`.
- A release TCP nearer the body than about `y = -0.05` was refused (`18` mm self-clearance). At the top-slice
  hover height `0.761` the carry must pass through `(0.35, -0.065)` first; going straight to `y = -0.04` was
  refused (`19` mm).
- The top slice does not always slide back. On 2026-11-26 the first slice, released on the plate, moved `4.3` cm
  toward the robot when the finger slid out; the top slice, released on the lettuce, stayed where it was held and
  ended `3.5` cm forward of the lower one.
- When the lower finger slides out, the slice moves back toward the robot by `3.6` to `6` cm, and the amount
  is not repeatable. The top slice ended about `4` cm off on 2026-11-24 and about `2` cm off on 2026-11-25. Not
  yet tried: a lower release height for the top slice, or opening fully before the slide-out.
- Exit through `(0.46, -0.12, 0.76)` rpy `[45, -5, 10]` and `(0.45, -0.15, 0.78)` rpy `[0, 0, 0]`, then
  `home right`. `home` before reaching `(0.45, -0.15, 0.78)` was refused.
- Reference stack heights: plate surface `0.61`, first slice top about `0.63`, lettuce top about `0.66`.
  Confirm the current stack from the camera.

## Lettuce notes

- After the first slice is down, register the centre plate as an obstacle so the planner refuses any path
  through it:

```bash
curl -s --noproxy '*' -X POST http://127.0.0.1:8765/scene -d '{"obstacles":[{"name":"plate","center":[px,py],"size":[0.24,0.24],"top_z":0.66,"yaw":0.0,"margin_xy":0.02}]}'
```

  With `top_z = 0.66` the lettuce release at `z = 0.70` and the top-slice release at `z = 0.725` still pass.
  Do not register it earlier: the first-slice release at `z = 0.68` is inside the box.
- Do not joint-interpolate straight from `home` to the hover above the leaf. That swing carried the open
  fingertips through the centre plate and pushed it about `4` cm. Rise in place, go in a straight line at
  `z = 0.80`, then joint-interpolate down. A straight descent from `0.80` was refused as a branch flip.
- Grasp rpy: `[0, 35, -45]` for a dish near `(0.50, 0.20)`. A dish at about `(0.51, 0.28)` is out of reach for
  it; `[0, 45, -37]` passed. A flat hand at rpy `[0, 15, ...]` failed twice.
- Grasp height: target `z = 0.636` to `0.637`. A target of `0.6413` stopped at `0.644` and closed above the
  leaf; `0.636` stopped at `0.642` and held it.
- Opening `0.02` to `0.05` can mean a leaf is held, and the `holding` flag is not reliable. The wrist image is
  not reliable either: a leaf still lying in the dish shows next to the closed fingertips. After the rise,
  confirm from the head camera that the dish is empty.
- Carry high: the measured route carries at `z = 0.76`. A low carry pushed both plates by `5` cm.
- Where the leaf lands, with release rpy `[0, 5, -75]`; `(bx, by)` is the centre of the lower slice:

| Release TCP | Result |
|---|---|
| `(bx - 0.01, by + 0.067)` | leaf covers most of the slice |
| `(bx - 0.01, by + 0.039)` | leaf covers the right half and hangs `5` cm over the `-y` edge |
| `(bx - 0.004, by + 0.029)` | leaf covers the right-front half only |
| `(bx - 0.01, by + 0.07)` | leaf covers the right half and hangs over the `-y` and near edges |

- The leaf hangs toward `-y` of the fingers by an amount that depends on where the fingers closed, so no fixed
  offset is reliable. Start from `(bx - 0.01, by + 0.07)` and correct `y` from the hover view.
- Do not move the hand back in `x` at the hover. A leaf that stands up from the fingertips projects farther
  forward than it will lie: on 2026-11-25 the hand was moved back and the leaf covered only the left and near
  half of the slice.
- The lowest fingertip is below the TCP; see "Frame, TCP and gripper geometry" in
  [EXPERIENCE.md](EXPERIENCE.md). At pitch `35` the drop is `2.97` cm.

## Plate push notes

Only when the user asks, after the sandwich is finished and both arms are at `home`. The hands push the plate
straight forward (`+x`), the way a person slides a plate across a table with the side of the hand.

### Hand pose

- Left hand: rpy `[0, 15, -90]` with gripper `0.1`. The fingers point toward `-y` across the front of the body and
  the opening axis points along `+x`, so the outer face of the front finger is a vertical surface facing the
  plate. That face pushes the rim.
- Poses with the fingers pointing forward (`[0, 45..89, 0]`, `[0, 30, -30]`, `[0, 30, -60]` and their
  right-hand mirrors) were out of reach for both arms at `x = 0.22` to `0.28`. The rear rim of a plate this
  close to the robot is only reachable with the hand turned sideways.
- The right hand uses the mirrored pose `[0, 15, 90]` in the two-hand push. When only one hand pushes, use the
  left: the mirrored right-hand pose `[0, 20, 90]` is reachable, but its forearm travels toward the toaster.
- The fingertip is the lowest point: `drop(15) = 0.0233*sin(15) + 0.020*cos(15) = 0.0253` m below the TCP.
- The outer face of the front finger is about `0.03` m ahead of the TCP in `x` at gripper `0.1`.

### Where to put the hand

Let the plate's rearmost rim point be `(rx, ry)`, read from the current head image.

- `x`: start with the TCP at `rx - 0.04`, which leaves about `1.2` cm between the finger face and the rim.
- `y`: the rearmost rim point must lie on the finger face, about `1.5` cm back from the fingertip. A flat face
  on a round rim pushes through the plate centre. A fingertip corner touching the rim to one side of that
  point pushes the plate sideways, and the offset grows as it moves.
- Align `y` from the image, not from the TCP numbers. Near `x = 0.2` the fingertip appeared about `3` cm
  farther toward `-y` in the head image than the TCP position predicts. In the measured run TCP `y = 0.016`
  put the fingertip about `1.5` cm past a rear point at `y = -0.017`.
- `z`: push at TCP `z = 0.624`. The fingertip is then about `8` mm above the table (`0.595`) and below the
  rim. `z = 0.620` was refused (`left_wrist_yaw_link 49 mm above the table`).

### Two-hand push (steps 7 to 9)

- Clear the plate obstacle box first (`POST /scene` with `{"obstacles":[]}`). The push ends inside it and is
  refused otherwise.
- Bring both hands down together with `both` and `"linear": false`, in steps `0.76`, `0.70`, `0.66`. Arriving
  at `z = 0.80` selected a raised-elbow branch (shoulder abduction about `110` degrees), and the straight
  descent from it was refused.
- Starting `y`: the two fingertips sit about `2` cm either side of the plate's rearmost point. On 2026-11-25
  the hands went straight to their final `y` (self-clearance `32` mm). The day before that was refused offline
  (`17` mm between the right finger and the waist); the hands hovered at `ry + 0.075` and `ry - 0.075` and then
  shifted with a straight `both` move at `z = 0.66`. Let the preview decide.
- Push end point: from TCP `x = 0.188`, `x = 0.40` passed and moved the plate about `20` cm in `7.4` s;
  `x = 0.41` was refused as a branch flip.
- Retreat with `both`: lift to `z = 0.78`, straight lines to `(0.45, +-0.15, 0.78)` rpy `[0, 0, 0]`, then
  `home both`. `home both` directly from the lifted push pose is refused (`15` mm between the fingertips).
- The plate ends under the forearms' `home` positions. Nothing on it may be taller than the stack, and the
  lettuce dish and the toaster must be outside the swept strip.

### One-hand push

Measured once, in run `20261124-172023-real-eef-cc`, with the left hand and a `23` cm plate whose rear rim
started at `x = 0.238`. From `home left`: joint-interpolate to `(0.509, 0.149, 0.80)` rpy `[0, 0, 0]` with
gripper `0.1`; to `(rx - 0.04, y, 0.76)` rpy `[0, 15, -90]`; to `z = 0.70`; to `z = 0.66`; look; down to
`z = 0.624`; look. Push straight in `+x` at speed `0.08` for about `5` cm and check that the plate moved and the
sandwich did not, then push to the end point at speed `0.10`. TCP `x` from `0.183` to `0.39` moved the plate
about `19` cm. Lift straight to `z = 0.78`, joint-interpolate to `(0.45, 0.15, 0.78)` rpy `[0, 0, 0]`, then
`home left`.

## Finish and report

After the last authorised motion:

1. Inspect the final sandwich from the head camera.
2. Run `bash r1d eef keyframe final`.
3. Run `bash r1d eef rec-stop`.
4. Return both arms with `bash r1d eef home both` only after confirming the motion is safe and
   authorised.
5. Collect metrics:

```bash
bash r1d metrics --run-dir runs/<run> --request "<user request>"
```

Report the actions that actually completed, final image conclusion, grasp openings, any refusal or
manual correction, measured placement offset, maximum temperature and metrics path. If an image or
temperature sample is missing, state that it is missing. Do not infer a successful placement from an
accepted command.
