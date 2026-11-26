# Direct-EEF sandwich playbook

English | [中文](PLAYBOOK.zh.md)

See also: [Task route](TASK_SANDWICH.md), [API](API.md).

This playbook covers the commands, payloads, posture guidance and efficiency rules shared by simulation,
shadow and real-robot runs. The hard rules (authorisation, fixed lift, joint-space `home`, refusals, carrying,
the toaster, the plate) are stated once, in "Hard rules" in [SKILL.md](../SKILL.md), and are not repeated here.

## Contents

1. [Operating constraints](#operating-constraints)
2. [Commands and payloads](#commands-and-payloads)
3. [Offline evaluation and posture](#offline-evaluation-and-posture)
4. [Arm posture rules](#arm-posture-rules)
5. [Safety](#safety)
6. [Efficiency](#efficiency)
7. [After a run](#after-a-run)

## Operating constraints

- Every arm motion is an end-effector waypoint solved by the service's IK. The only exceptions are a joint-space
  `home`, a waist command and a gripper command.
- `exec` sends every waypoint with `auto_lift=False`, so the lift stays at the height the service was started
  with.
- Record the head, left wrist and right wrist views for the whole task.
- Positions are in the R1-D base frame, in metres: x forward, y left, z up. The reference table top is
  `z = 0.595`; the plate surface is about `z = 0.61`.
- Every placement is different. Read object positions from the current images. Poses, relative offsets and
  proven route shapes are reusable only after the new scene is checked.

The route is in [TASK_SANDWICH.md](TASK_SANDWICH.md). Read it before making a sandwich.

## Commands and payloads

Run commands from the directory containing `SKILL.md`:

```bash
bash r1d eef new
bash r1d eef prepare
bash r1d eef rec-start
bash r1d eef state
bash r1d eef look --cams head_left,left_wrist,right_wrist --raw
bash r1d eef look --cams head_left --z 0.61 --crop u0,v0,u1,v1 --scale 4 --mark "x,y,z;x,y,z"
bash r1d eef look --cams head_left --raw --crop u0,v0,u1,v1 --scale 4
bash r1d eef project --cams head_left --z 0.61 "u,v;u,v"
bash r1d eef check '<json>'
bash r1d eef exec '<json>'
bash r1d eef grip right grasp
bash r1d eef waist 0.0
bash r1d eef home right
bash r1d eef keyframe TAG
bash r1d eef rec-stop
```

`new` creates a directory under `runs/`. Later `eef` commands append to that run's
`eef_log.jsonl`, and images go under `looks/` or `keyframes/`.

The ruler view draws a calibrated base-frame grid on the requested plane. Lines are 10 cm apart, dots
are 5 cm apart, red lines have constant x, and blue lines have constant y. `--mark` overlays explicit
base-frame points so an estimate can be checked against the image.

To measure a position instead of estimating it against the grid, read pixels and project them:

1. `look --raw --crop u0,v0,u1,v1 --scale 4` saves the enlarged region with a tick every 10 px, labelled in
   the pixels of the full image.
2. `project --cams head_left --z <height> "u,v;u,v"` prints the base-frame point where each pixel's ray meets
   the horizontal plane at that height. Use the height of the surface the pixel lies on: `0.61` for the plate,
   the height of a held slice for its corners. Without `--z` the head camera uses stereo depth.

Every `look` also saves each view as `runs/latest/<cam>.jpg`. A command that moves the robot (`exec`, `both`,
`grip`, `waist`, `home`) deletes those files first, so a picture at that path was always taken after the last
motion. If the motion was refused and the chained `look` did not run, the file is missing rather than old.

The waypoint payload has this shape:

```json
{
  "note": "slice 1",
  "segments": [
    {
      "side": "right",
      "speed": 0.15,
      "waypoints": [
        {
          "pos": [0.47, -0.15, 0.87],
          "rpy_deg": [0, 0, 0],
          "linear": true,
          "gripper": "grasp"
        }
      ]
    }
  ]
}
```

Write a stage name in `note`: `slice 1`, `lettuce`, `slice 2` or `push plate`. Metrics use these names to
split the run. A `gripper` field may be `grasp` or a number from 0 to 1.

A waypoint may carry `"min_opening": 0.1`. Before moving to that waypoint, `exec` requires the hand's gripper
to be at least that open and otherwise stops there with `NOT HELD` and exit status 1. It uses the opening
reported by the last `grasp` in the same segment, or the measured opening when the segment has no grasp. Put
it on the first waypoint after the pull, so that grasp, pull and carry can share one `exec` and a missed
grasp still stops above the toaster. Use it for bread only: a held lettuce leaf reads `0.02` to `0.05`.

`check` tests each waypoint from the current pose without moving. A refused waypoint returns `ok=false`
and a machine-readable `code`. `exec` stops at the first refused waypoint and exits with status 1.

For bread, an opening of `0.13` to `0.20` after a grasp means held and `0.02` to `0.06` means missed. For
thin lettuce, opening and `holding` are ambiguous; confirm from the head camera that the dish is empty.

## Offline evaluation and posture

Preview candidate sequences with the same planner used by the service:

```bash
bash r1d eval '{"side":"right","waist":0.0,"candidates":{"A":[{"pos":[0.47,-0.15,0.87],"rpy_deg":[0,0,0],"linear":false}],"B":[]}}'
```

An input may include `start_q` with all 14 arm joints. Otherwise the current robot state is used, so the
service must be running and readable. Put 5 to 8 candidates in one request. Evaluate placement and exit
options together before the hand is already holding an object.

`bash r1d eval --brief '<json>'` prints one line per candidate with the numbers that decide between them:
`OK`/`NO`, worst abduction, lowest elbow drop, worst shoulder-yaw torque, largest joint travel, smallest
self-collision clearance (`selfmin`), smallest table gap (`gapmin`) and, for a failure, the waypoint and reason.
`link_gap_min_per_wp` in the full output is the table gap; `self_clearance_min_per_wp` is the self-collision
clearance.

Two-arm sequences, as sent with `eef both`, are previewed with `"side": "both"`; each waypoint then has a
`left` and a `right` pose. Add `"end_at_start": true` to also check the joint-space return to the starting
joints, which is `home` when the arms start there:

```bash
bash r1d eval --brief '{"side":"both","end_at_start":true,"candidates":{"push":[{"left":{"pos":[0.509,0.149,0.80],"rpy_deg":[0,0,0]},"right":{"pos":[0.509,-0.149,0.80],"rpy_deg":[0,0,0]},"linear":false}]}}'
```

Read arm posture with:

```bash
bash r1d posture
```

The posture report uses shoulder abduction, elbow height below the shoulder, and absolute shoulder-yaw
gravity torque. In the packaged helper, "ugly" is defined by exactly these conditions:

- shoulder abduction greater than `45` degrees;
- elbow drop below the shoulder less than `0`;
- absolute shoulder-yaw torque greater than `2.5` Nm.

These thresholds are measurements, not command limits. Use the candidate with lower abduction, a lower
elbow and lower shoulder-yaw load when the path is otherwise safe.

The toaster obstacle box is part of the scene safety model. After the toaster is moved, redraw and verify
its box with `look --mark` and read the current scene:

```bash
curl -s --noproxy '*' -X POST http://127.0.0.1:8765/scene -d '{}'
```

If the box must be updated, send all four base-frame corners or the equivalent centre, size, top height
and yaw. One reference box is `center=[x,y]`, `size=[0.26,0.15]`, `top_z=0.755`, `margin_xy=0.03`.
Those values describe the reference toaster, not a new placement.

## Arm posture rules

- Start every stage from `home`. Evaluate a new route before sending it and choose the candidate with
  less abduction, a lower elbow and less shoulder-yaw torque.
- An end-effector pose used as a "ready position" selected another IK branch in the first real run: `48`
  degrees of shoulder abduction and an outward elbow. The right shoulder-yaw motor then carried about `4` Nm
  instead of the standard pose's `0.8` Nm and reached `84 C`.
- For a flat bread release, enter from the plate's right side with rpy `[90, -10, 20~30]`. This gives
  about `5` to `10` degrees of shoulder abduction and places the elbow `3` to `13` cm below the
  shoulder. Do not use `[-90, 10, 75]`; it raises the elbow above the shoulder.
- Some poses cannot be made natural at the fixed lift. Reaching into the toaster is about `80` degrees
  of abduction. Do not linger above it: the longer hold makes posture worse and heats the joint.
- Picking lettuce and carrying the leaf can raise the left elbow to shoulder height. In the third run,
  the left arm was in a poor posture for `91%` of the measured time.

## Safety

Before any motion, read `state` and confirm:

- mode is the one the user authorised and `stopped` is false;
- both hands are above `z = 0.70`;
- waist is approximately `0`;
- `bash r1d eef prepare` has run in this run: the lift is at `0.522` and the head pitch is about `0.5`;
- record the starting motor temperatures.

Do not leave the arms holding a loaded posture while thinking or evaluating offline.

The service enforces joint limits, self-collision clearance, table clearance, registered obstacle boxes,
single-motion distance and watchdog stopping. It does not know where the plate, food, loose objects or
people are. Approach the plate from above, descend vertically, and inspect the result.

Only one controller may own the robot at a time. If the service restarts, the log directory changes, or
another operator moves an arm, stop and report the inconsistency.

## Efficiency

The target is 30 minutes and 40 model calls per sandwich. The first run under 40 calls was
`20261125-162207-real-eef-cc`: 38 calls, 34.5 minutes and `$12.50`, including the two-hand plate push. Where
that time went:

| Part | Time |
|---|---:|
| Robot moving (all `exec`, `both`, `grip`, `home`) | 178 s |
| One gap with no robot command and no model output | 386 s |
| Everything else: waiting for the model, about 40 s per call | about 25 min |

The arm is not the bottleneck; the number of model calls is. Thirteen of those calls did nothing but open a
picture that the previous command had already taken, and they cost 440 s together.

- **Move and look in one turn.** Chain `exec ... && look ...` in one shell command and, in the same turn, read
  `runs/latest/<cam>.jpg`. The tools run in order, so the picture is the one taken after the motion; after a
  refusal the file is missing. Do not spend a turn only to open an image.
- **One `exec` per decision.** Everything between two looks belongs in one `exec`: hover, lowering and the
  close look; then descent, grasp, pull, roll and carry to the pre-release hover with `min_opening` on the
  roll; then release, slide-out, exit. Chain `home` and `keyframe` after the last one.
- **Keep the looks that decide something.** They are: the lowered open hand over the slice before closing, the
  held slice at the pre-release hover, the leaf at its hover, the fingers behind the plate rim before
  descending for the push, and the result after each placement. Skipping the pre-release look cost `253` s in
  one run.
- **Measure with `project` in the turn that reads the picture.** Read the pixels from a `--raw --crop` view and
  project them; do not estimate against the grid and then spend another look checking the estimate.
- **Preview once per stage** with `eval --brief`, placement and exit together, and write the whole stage's
  waypoints from the result. Do it while the arm is at `home`, not while it holds a loaded pose.
- Reuse proven poses and route shapes; re-search only for a new object position.
- Prefer numeric feedback (TCP position, opening, refusal code) when it answers the question.

Earlier costs that the route now avoids: `19.8` minutes and `31` calls on toaster localisation and the first
slice in the third run (the head view of the toaster was unreliable; use the wrist view above it); about `6`
minutes for two missed grasps on a second slice (check the lowered, open gripper from the wrist camera before
closing); seven refused placement or exit waypoints in one run (preview them together).

## After a run

1. Save a `final` keyframe and run `rec-stop`.
2. Collect metrics into the run directory:

```bash
bash r1d metrics --run-dir runs/<run> --request "<user request>"
```

Metrics use the first to last EEF instruction, with one minute added before and after. If automatic
transcript discovery is ambiguous, pass `--transcript`. The collector reports wall time, model calls,
cost, stage attempts, sent and executed waypoints, grasp openings and motor temperatures.

Write an `annotations.json` for the run when reviewing results. Record each stage's attempts, success,
grasp openings, placement offset, plate contact and notable events. Report only observed facts and name
any missing image or sensor sample explicitly.
