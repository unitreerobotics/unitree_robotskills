# R1-D sandwich results

English | [中文](RESULTS.zh.md)

See also: [Experience](EXPERIENCE.md), [Task route](TASK_SANDWICH.md).

Three real runs completed `3/3` sandwiches. The task was one bread slice from the toaster, lettuce on top, and a
second bread slice covering it. The agent read the cameras and supplied direct end-effector waypoints; the
service solved and checked each waypoint.

| Run | Time | Model calls | Cost | Grasp attempts | Top-slice error | Peak motor temperature |
|---|---:|---:|---:|---|---|---:|
| 1 | 55.7 min | 99 | $32.02 | 1/1/1 | about 6 cm | 76 C |
| 2 | 69.0 min | 96 | $29.66 | 3/3/1 | 2-3 cm | 84 C |
| 3 | 33.8 min | 62 | $8.07 | 1/1/3 | about 1.3 cm | 79 C |

The grasp columns are the recorded counts for the first bread slice, lettuce and second bread slice. The
placement error is the final top-slice offset.

## What changed

- **Run 1:** Completed the stack, but the top slice was visibly offset and the operator stopped further
  correction. An end-effector "ready pose" selected an outward-elbow IK branch. The shoulder-yaw motor reached
  76 C. A low lettuce carry pushed both dishes about 5 cm toward the robot.
- **Run 2:** The stack was nearly aligned, with the top slice about 2-3 cm forward. Joint-space home replaced
  the end-effector ready pose for the bread stage. Turning the waist to reach into the toaster raised the right
  shoulder to 84 C; the fixed lift still required an elbow near shoulder height.
- **Run 3:** The two slices differed by about 1.3 cm. The toaster was located with the right wrist rather than
  the oblique head view. Seven placement or exit waypoints were refused and were replaced with offline-previewed
  candidates. The run was the fastest and cheapest of the three.

## Later run with the plate push

Run `20261125-162207-real-eef-cc` (2026-11-25) made the sandwich and then pushed the plate forward with both
hands. Figures from its `metrics.json`:

| Time | Model calls | Cost | Grasp attempts | Refused commands | Robot motion time | Peak motor temperature |
|---:|---:|---:|---|---:|---:|---:|
| 34.5 min | 38 | $12.50 | 1/1/1 | 0 | 178 s | 72 C |

Grasp openings were `0.181` and `0.166` for the bread and `0.033` for the lettuce. The agent's own report at the
end of the run put the top slice about 2 cm to the right of the lower one, with the lettuce showing at the left
front. It is the first run under 40 model calls; it did not meet the 30-minute target. The playbook's
"Efficiency" section breaks down where the time went.

Run `20261126-153825-real-eef-cc` (2026-11-26) was the first with the self-contained code (bundled executor,
own camera client, no unitree-deploy). It made the sandwich and pushed the plate with both hands in `26.9`
minutes with `41` model calls (`$10.11`); the arm moved for `327` s and the peak motor temperature was `69` C.
Grasp attempts were `2/1/1`: the first bread grasp closed on nothing because the finger carriage rested on the
slice. The top slice ended `3.5` cm from the lower one and turned by about `45` degrees. It is the first run
under `30` minutes. Four more missed bread grasps earlier that day, in run `20261126-121835-real-eef-cc`, are
not in these figures.

## Targets and limitations

- None of the first three runs met both targets of 30 minutes and 40 model calls. The fastest took 33.8
  minutes, 62 calls and `$8.07`. The later run met the call target (38) and not the time target (34.5 minutes).
- Poor arm posture occupied 42%-91% of the measured time. The main causes were reaching into the toaster and
  carrying lettuce with the elbow near shoulder height.
- The second run reached 84 C at the right shoulder. The operator stopped at the temperature boundary rather
  than relying on a service guard alone.
- All results are for the described toaster, white plate and small lettuce dish. They do not establish
  general sandwich making, a new object layout, or another food task.

The first run included an operator-triggered stop, so its wall time includes correction and recovery. Run
boundaries use the first to last robot instruction plus one minute before and after. Costs use the recorded
official token prices.
