---
name: g1d-draw
description: Plan reference-image paintings for Unitree G1-D, measure paper and paints, record calibration, compile complete painting jobs and launch scripts, and execute them. Use for painting planning, brush tests, on-site measurement, offline compilation, and physical robot painting.
metadata:
  version: "2.0.0"
  language: en
---

# G1-D reference-image painting

English | [中文](SKILL.zh.md)

Installation and commands are in the [README](README.md).

Follow **plan → measure → record → compile and export → execute and inspect**. Design the composition and strokes from the reference, measure the actual setup, and deliver a complete compiled job with `run.sh`. Execution uses deterministic local code rather than model calls between strokes.

Use the repository package `unitree_gpt6.g1d_draw.src`, its `robot-paint-execute` CLI, and the [configuration template](config/device.example.json). Install from the repository root with `python -m pip install -r unitree_gpt6/g1d_draw/requirements.txt`, add `unitree_gpt6/g1d_draw/scripts` to `PATH`, and add the repository root to `PYTHONPATH`; see [README.md](README.md) for installation, connectivity, cameras, schemas, and default poses. Offline editing/compilation does not start hardware. The backend is for a fixed-base G1-D with two 7-axis arms; the URDF and device configuration must match the actual robot.

## Defaults and saved settings

Apply **the user's current instruction → accepted settings saved with this task → defaults for new tasks**. Replanning preserves accepted paper/material coordinates, contact heights, grip, waist, and work/rest poses unless a requested change or current measurement requires updating them. Record the revision and regenerate affected artifacts.

| Setting | Default |
| --- | --- |
| Painting arm | Right; hold the left arm at the left 7 angles of `entry_q14[-1]` during painting |
| Brush tests | A designated test area, preferably the artwork's paper type; without one, leave physical brush tests pending |
| Measurement | Reuse valid calibration with an independent check; otherwise collect new geometry with suitable available measurement tools or clean-brush contact |
| Work/rest | Editable `default_work_q14()` and `default_rest_q14()` from [src/postures.py](src/postures.py) |
| Grip | `default_grip_q2()=[0.0, 0.0]`, left/right native units; preserve accepted task targets |
| Completion | Finish the reviewed withdrawal/parking route and retain powered hold |
| Coverage | Fill unintended gaps, merge compatible strokes, allow limited boundary overlap, and preserve intentional reference space/layering |

New tasks contain `entry_q14=[rest,work]` and `park_q14=[null,rest]`. The entry's first point is the expected actual starting posture. Read current joints first and update the complete entry route if they differ. Compilation replaces the parking `null` with the final lifted endpoint. Default poses are editable targets; review both arms and tool clearance along the route.

## Workflow

1. **Plan the whole image first.** Identify composition, color regions, foreground/background order, connections, and intentional blank space. Save the complete color order, paths, and intended coverage. Define the paper's u/v directions and preserve aspect ratio. List missing measurements; do not infer robot contact coordinates or brush width from an uncalibrated photograph.

2. **Measure paper and tool geometry.** Save current joints and tool feedback with `observe`. Collect at least 3 distributed, noncollinear paper anchors and at least 1 independent check. Use reviewed `compile-probe/probe` routes or external measurement tools. Record the calibrated brush-tip transform, determine paper position and tilt from the measured points, and save measurement sources and contact evidence. FK from feedback is not a contact sensor; captures start with `contact_verified=false`.

3. **Measure materials and test the brush.** Record each pigment and water station's approach, contact/immersion, dwell, and withdrawal route. For immersion, establish usable bristle depth while keeping the ferrule and container bottom clear. Record the test area, actual width, usable loaded-brush length, contact height, grip, waist, and work/rest routes. Camera names and viewpoint directions depend on the deployment; use available views or operator observation.

4. **Record and refine.** Fill `measurements.json` with accepted physical values, `contact_verified=true`, and `source` where contact is confirmed. Refine `strokes.json` using measured width and paint capacity. Merge same-color compatible paths, remove inefficient short strokes, and correct unintended gaps and overlap. Long strokes must remain within usable loaded length. Use the final stroke data to render `preview.svg`; it shows planned coverage, while photographs show physical results.

5. **Compile the complete painting and scripts.** Run `prepare` with the strokes, measurements, device configuration, a new `job_id`, and a new output directory. It fits calibration, builds the complete job, solves IK offline, checks paths, and exports `job.json` with `run.sh`. Include every entry, transfer, approach, contact range, withdrawal, and parking route. Record complete physical route review in `clearance_review`.

6. **Execute the delivered script.** When physical execution is authorized and layout/current state match the reviewed task, release the previous controller and launch `run.sh` once. Use one motion owner, fixed grip/waist, and a frozen nonpainting arm during painting batches. Runtime makes no model calls, IK solves, or camera captures. Obtain photos separately. Plan a separate job if intermediate inspection or measurement is needed.

7. **Track and inspect.** Save job identity, artifact hashes, attempt record, progress, health, and photographs. Query the same attempt after a timeout rather than resubmitting. `job_completed=true` with `parked=true` indicates completed planned motion and parking; inspect current health for holding state. Report paint quality from actual photographs separately.

## Measurement commands

Run from an installed repository environment:

```bash
robot-paint-execute init-task --output ./my-task
# Fill device.json and the reviewed probe.template.json first.
robot-paint-execute observe --config ./my-task/device.json \
  --label rest --output ./my-task/rest-observation.json
robot-paint-execute compile-probe --config ./my-task/device.json \
  --probe ./my-task/probe.template.json --output ./my-task/probe.json
robot-paint-execute probe --config ./my-task/device.json --probe ./my-task/probe.json
```

`capture_indices` selects captured action points. Mark lifted camera points in `lifted_capture_indices`, a subset of those indices, default `[]`. Lifted captures record target XYZ/orientation/joint errors without arrival gates; unmarked points retain contact checks. Capture records go to state storage under `attempts/<hash>/captures/`; copy accepted values and sources into the measurement input.

Normal probe completion parks and holds. Query status and explicitly release before another controller acquires the robot. Repeated measurement requires a new attempt ID and fresh compilation.

## Task delivery and execution

```bash
robot-paint-execute prepare --config ./my-task/device.json \
  --strokes ./my-task/strokes.json --measurements ./my-task/measurements.json \
  --job-id painting-attempt-1 --output ./ready/painting-attempt-1
bash ./ready/painting-attempt-1/run.sh
```

Deliver the reference, source `strokes.json` and `measurements.json`, fitted `calibration.json`, `device.json`, `build_plan.py`, complete `plan.json`, compiled `job.json`, `preview.svg`, `report.json`, `manifest.json`, and `run.sh`. Retain original measurements and photos of previous paintings. URDF paths must resolve on the execution machine.

The generated launcher locates its adjacent task directory and runs `robot-paint-execute run-task --task ...`. It checks artifact/URDF hashes and executes the compiled job without remeasurement, automatic compilation, ID changes, or retries. Modify source data and re-export instead of editing compiled artifacts.

```bash
robot-paint-execute status --config ./ready/painting-attempt-1/device.json
robot-paint-execute stop --config ./ready/painting-attempt-1/device.json
robot-paint-execute release --config ./ready/painting-attempt-1/device.json
```

The foreground controller retains powered hold after completion or stop. Ctrl-C stops an active run; after completion or stop, Ctrl-C releases. `release` ends publication without a parking movement. On stop, fault, or feedback loss, cancel remaining motion and preserve the terminal stop. A recovery route must start from actual state; do not invent a retreat or automatically replay. Query status to confirm control requests.

Each `job_id` allows one attempt. With an unchanged valid setup and authorization to repaint, reuse the source inputs through `prepare` with a new ID and output directory. Preserve attempt records and shared controller storage. Interrupted jobs do not resume automatically.

## Coordinates and execution conventions

- **Coordinates and brush tip.** `q14` stores joint angles as `[left7,right7]` in radians; grip and waist are stored separately. End-effector (EEF) poses use the configured URDF root frame: `x,y,z` are meters, and `roll,pitch,yaw` are XYZ Euler angles in radians. Calculate brush-tip position through the calibrated tool transform and verify how robot, model, and image directions correspond.
- **Paper and materials.** Support planar paper by determining its position and tilt from measured points, then calculating contact height across the sheet. Curved surfaces or other cleaning materials require adaptation.
- **Strokes.** `start`/`end` define an inclusive contact range. See [README](README.md#data-formats-and-python-api) for formats and fields, and retain its format identifiers for compatibility with existing data.
- **Arrival and runtime checks.** Lifted targets record position, orientation, and joint errors. Lifted work-entry endpoints and ordinary batch withdrawals connect continuously. Final parking records joint error and waits for stable hold. Contact arrival, tracking, feedback, minimum clearance, joint limits, speed, waist/grip targets, and terminal-stop state follow the configured checks; see [README](README.md#arrival-and-runtime-checks) for defaults. Compilation checks the planned trajectory; runtime checks live feedback.
- **Measurements and commands.** Keep each measured pose paired with its published command, avoiding repeated accumulation of the same servo offset into later commands. If acquisition drift is within tracking tolerance, runtime reads fresh feedback and checks the segment connecting the current state to the compiled route.

## Output

For preparation, link the source plan, measurements, preview, compiled task, report, and launch script; state stroke count, materials, and parameter changes. Distinguish offline compilation from physical execution. For execution, give job identity, progress, terminal state, holding/release state, and observed paint result. List specific unfinished measurement or execution work.
