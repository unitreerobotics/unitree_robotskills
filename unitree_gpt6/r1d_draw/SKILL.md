---
name: r1d-draw
description: Plan a reference-image painting for the Unitree R1-D (R1-A7-D), measure the paper and paints, record the calibration, compile the whole job with its launch script and run it. Use for painting plans, test strokes, on-site measurement, offline compilation and painting on the robot.
metadata:
  version: "0.1.0"
  language: en
---

# R1-D reference-image painting

English | [中文](SKILL.zh.md)

Installation and commands are in the [README](README.md).

Work through **plan → measure → record → compile and export → run and check**. Design the strokes from the reference image, measure the real layout, and deliver the compiled whole job with its `run.sh`. Local deterministic code runs the job in one go; no model directs single strokes.

Use `unitree_gpt6.r1d_draw.src`, the `r1d-draw` command and the [device configuration example](config/device.example.json) of this repository. Editing and offline compilation never start hardware. The backend is for the R1-D with two 7-axis arms; supply the URDF yourself and make sure it matches the robot.

## Differences from G1-D

- **Connection.** The R1-D's DDS is authenticated, so a workstation cannot read or write its topics. Every read and every command is done by a small program that is run on the robot over wired SSH. The configuration holds the robot's address, interface and MAC; the SSH password is read only from an environment variable (default `R1D_SSH_PASSWORD`) and is never written to a file.
- **Robot interface.** The object returned by `connect(config)` offers `state()`, `photo(camera)`, `take_over()`, `move(route)`, `stop()`, `relax()`, `release()` and `status()`. The real and the simulated robot share these functions.
- **Joints.** 35 motor slots: lift 12, waist yaw 13, left arm 15–21, right arm 22–28, head 29/30, grippers 31/33. Base and lift are never commanded; waist, head and grippers stay where they were at take-over.
- **Frame.** Poses are in `model.base_frame` (default `pelvis_link`), so the lift height does not enter them. The waist yaw is one fixed number per job, `waist_yaw`.
- **Continuing after an interruption.** Progress is recorded per batch; `run-task --resume` continues at the unfinished batch and repaints nothing that was finished.

## Defaults and saved data

The order of precedence is **what the user asks for now → data already accepted for this task → defaults for a new task**. When replanning, keep the accepted paper, material coordinates, contact heights, grip, waist and entry/park routes; record a revision and regenerate the affected outputs only when the user asks or a measurement shows a change.

| Item | Default |
| --- | --- |
| Painting arm | Right; during painting batches the left arm keeps the first 7 angles of `entry_q14[-1]` |
| Test strokes | In the named test area, preferably on the same paper; without one, list the physical test as open |
| Measurement | Reuse a calibration that still holds and check it independently; measure anew with a clean brush touching, or with a reliable tool on site |
| Loading and washing | No built-in rule. `strokes.json` may give `reload_every` (strokes per load) and `wash_station` (station used between colors and at the end); without them each color is loaded once and nothing is washed |
| Finish | Park along `park_q14` and keep holding; run `relax` separately to let the arms go limp |
| Coverage | Fill unintended gaps, merge compatible strokes, allow slight overlap at edges; keep the reference's intended blank areas and layering |

## Workflow

1. **Plan the whole picture first.** Analyse the composition, color areas, layers, connections and intended blank areas. Save the complete color order, paths and coverage, state the paper's u/v directions and keep the proportions. List the missing measurements; never infer robot contact coordinates or real brush width from uncalibrated photos.

2. **Measure the paper and the tool.** Save the joints and brush-tip poses with `observe` and camera frames with `photo`. Collect at least 3 spread-out, non-collinear paper anchors and at least 1 independent check point. Measuring moves use `take_over()` and `move(route)`; check every route offline first. Record the brush-tip tool transform and keep the source and the contact evidence. FK feedback cannot tell whether the brush touches: records default to `contact_verified=false`.

3. **Measure paints and water, and make test strokes.** Save each station's approach, contact/immersion, dwell and withdrawal poses. Load to a usable bristle depth, clear of the ferrule and of the vessel bottom. Record the test area, real brush width, usable stroke length per load, contact height, grip, waist and entry/park routes. Camera frames are not synchronised with each other.

4. **Record and correct the plan.** Write accepted measurements into `measurements.json`; mark confirmed contacts with `contact_verified=true` and a `source`. Adjust `strokes.json` to the brush width and paint capacity. Generate `preview.svg` from the final strokes; the preview shows planned coverage, a photo shows the result on paper.

5. **Compile the whole job and the script.** Run `prepare` with the complete strokes, measurements, device configuration, a new `job_id` and a new output directory. It fits the paper offline, builds the whole job, solves the IK, checks the paths and exports `job.json` and `run.sh`. Record the review of the whole physical path in `clearance_review`; the offline check covers joint limits and an optional workspace box, not collisions.

6. **Run the delivered script last.** With authorisation to move the robot and the scene and posture matching the task, make sure no other program commands the robot and start `run.sh` once. There is one motion controller throughout. Photos are taken separately.

7. **Follow and check.** Keep the job identity, file hashes, progress records, health state and photos. A client timeout is not a cancellation: query `status` of the same session and do not submit again. `job_completed=true` with `parked=true` means the planned motion and the parking are done; judge the paper from photos.

## Measuring commands

```bash
r1d-draw init-task --output ./my-task
# Fill in device.json first.
export R1D_SSH_PASSWORD=...        # only ever in the environment
r1d-draw observe --config ./my-task/device.json --label rest --output ./my-task/rest.json
r1d-draw photo --config ./my-task/device.json --camera head --output ./my-task/head.png
```

Measurements that move the arm use the Python interface:

```python
from unitree_gpt6.r1d_draw.src.robot import connect

robot = connect("./my-task/device.json")
robot.take_over()  # hold the arms where they are
robot.move({"points": [{"q": {"28": 0.25}, "duration_s": 3.0}]})
print(robot.state()["arms"]["right"])
robot.relax()  # arms go limp slowly, grippers keep holding
robot.close()
```

## Delivering and running a job

```bash
r1d-draw prepare --config ./my-task/device.json \
  --strokes ./my-task/strokes.json --measurements ./my-task/measurements.json \
  --job-id painting-attempt-1 --output ./ready/painting-attempt-1
bash ./ready/painting-attempt-1/run.sh
```

Deliver the reference image, the source `strokes.json` and `measurements.json`, the fitted `calibration.json`, `device.json`, the whole `plan.json`, the compiled `job.json`, `preview.svg`, `report.json`, `manifest.json` and `run.sh`. Keep the raw measurements and photos of earlier paintings. The URDF path must exist on the machine that runs the job. After changing a source input export again; never edit compiled outputs.

```bash
r1d-draw status  --config ./ready/painting-attempt-1/device.json
r1d-draw stop    --config ./ready/painting-attempt-1/device.json   # cancel the route, keep holding
r1d-draw relax   --config ./ready/painting-attempt-1/device.json   # arms go limp slowly
r1d-draw release --config ./ready/painting-attempt-1/device.json   # stop commanding; no relax, no parking
bash ./ready/painting-attempt-1/run.sh --resume          # continue at the unfinished batch
bash ./ready/painting-attempt-1/run.sh --resume --lift   # brush still on paper or in a material: lift straight up first
```

After a stop, a fault or lost feedback the controller sends nothing more and the motors keep their last frame: that is not a supervised hold, and nothing retreats or replays by itself. Look at `status` and at the scene before deciding to `--resume`. A `job_id` can be started once; to paint again from the first stroke, `prepare` with a new id and a new directory.

## Conventions

- **Coordinates and brush tip.** `q14` holds `[left 7, right 7]` joint angles in radians; the grippers `grip_q2` and the waist `waist_yaw` are kept separately. Poses have `x,y,z` in meters and `roll,pitch,yaw` as XYZ Euler angles in radians, in `model.base_frame`. The brush tip is the hand frame times the tool transform; the wrist position is not the tip position.
- **Paper and materials.** Flat paper is supported: measured points give its position and tilt, and with them the contact height everywhere. A material station keeps its complete measured pose sequence and a `contact_index`.
- **Motion.** Within a stroke the tip moves at one speed without stopping, except at sharp turns; moves through the air follow a straight line cut into short steps and are also made in one go; touching down, leaving and dipping are separate slow moves. Speeds are set in `motion` of `device.json`; defaults are in the [README](README.md#motion-settings).
- **Tip offset correction.** At the first point above each batch the real joints are read and the measured tip offset is subtracted from the rest of that batch. `--no-correction` turns this off.
- **Checks while running.** The controller checks fresh feedback, other command publishers, motor faults, temperature, tracking error, speed and publishing gaps; when one fails it stops commanding and reports it in `status`.

## What to report

For preparation: link the source plan, measurements, preview, compiled job, report and launch script; state stroke counts, materials and changed parameters; separate offline compilation from execution on the robot. For execution: state the job identity, progress, final state, whether the arms are held, relaxed or released, and the observed painting. List the specific measurements or steps that are still open.
