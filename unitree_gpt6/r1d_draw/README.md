# R1-D robot painting

English | [中文](README.zh.md)

The agent workflow is in the [Skill](SKILL.md).

For the Unitree R1-D (R1-A7-D) with two 7-axis arms and grippers. It covers **plan → measure → record → compile and export → run**: the agent designs strokes from a reference image, measures the scene and saves the data, then builds the complete joint trajectories and a `run.sh` offline. No model directs single strokes while the job runs.

## Installation

Needs Linux, Python 3.10+ and a wired network to the robot. From the **repository root**:

```bash
python3.10 -m venv unitree_gpt6/r1d_draw/.venv
source unitree_gpt6/r1d_draw/.venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r unitree_gpt6/r1d_draw/requirements.txt
export PATH="$PWD/unitree_gpt6/r1d_draw/scripts:$PATH"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
r1d-draw --help
```

| Dependency | Use | Licence |
| --- | --- | --- |
| [NumPy](https://github.com/numpy/numpy) | Numerics and trajectories | BSD 3-Clause |
| [SciPy](https://github.com/scipy/scipy) | Numerical IK | BSD 3-Clause |
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio), package `pin` | URDF and forward kinematics | BSD 2-Clause |
| [Paramiko](https://github.com/paramiko/paramiko) | Wired SSH to the robot | LGPL 2.1 |

The workstation needs no Unitree SDK. On the robot the factory Python 3.8, `unitree_dds_wrapper` and Pinocchio are used and nothing is installed: the controller is uploaded to `/tmp/r1d_draw/<session>` at every take-over.

### Why SSH

The R1-D's DDS is authenticated: a workstation discovers the participants but receives no topics. Reading the state, taking photos and commanding are therefore done by programs run on the robot; the workstation starts them over wired SSH and exchanges request and status files. Before connecting, the configured interface MAC is checked so that no other machine is addressed by mistake.

The SSH password is read only from an environment variable; the configuration holds just the variable's name:

```bash
export R1D_SSH_PASSWORD=...
```

### URDF

The repository contains no robot model. Download the official [R1-D URDF](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/r1d_description/r1d.urdf) (the [meshes](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/r1d_description/meshes) next to it are optional; repository licence [BSD-3-Clause](https://github.com/unitreerobotics/unitree_ros/blob/master/LICENSE)) and put its path in `model.urdf` of `device.json`. The URDF is used for IK, brush-tip FK and the gravity compensation on the robot, and must match the real robot.

## Layout

| Path | Content |
| --- | --- |
| [SKILL.md](SKILL.md), `SKILL.zh.md` | The agent's planning, measuring, delivery and execution workflow |
| `src/robot.py` | Robot interface: `connect`, `state`, `photo`, `take_over`, `move`, `stop`, `relax`, `release`, `status` |
| `src/onboard/` | The read-only sampler and the controller that run on the robot |
| `src/calibration.py`, `task.py`, `job.py`, `kinematics.py` | Paper calibration, planning, job validation, IK and compilation |
| `src/timing.py`, `runner.py` | Path timing; running a whole job, recording progress, resuming |
| `src/simulation.py` | A simulated robot and toy kinematics that connect to nothing |
| [config/device.example.json](config/device.example.json) | Example device, camera, model and storage configuration |
| `examples/` | Read-only examples: read the joints, save camera frames |
| `tests/` | Offline tests; they never connect to a robot |

## Check the connection and the images first

Copy the example configuration and fill in the robot's address, interface, MAC and the cameras:

```bash
cp unitree_gpt6/r1d_draw/config/device.example.json ./device.json
python unitree_gpt6/r1d_draw/examples/read_state.py --config ./device.json --output ./state.json
python unitree_gpt6/r1d_draw/examples/capture_images.py --config ./device.json --output ./images-1
```

Both examples only read; they do not take over the arms. Data in `state.json`:

| Data | Order and units |
| --- | --- |
| `q` / `dq` / `tau` / `temperature_c` | 35 motor slots; rad, rad/s, Nm, °C |
| `arms.left` / `arms.right` | 7 axes per arm (slots 15–21, 22–28): shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw |
| `waist_yaw`, `head`, `lift` | Slot 13; 29/30 (pitch, yaw); 12 |
| `grip` | Left, right gripper in device units; slots 31, 33 |
| `commanded`, `command_frames` | Whether a program was sending commands while sampling |

Each camera is defined in `cameras`: `kind` is `rtsp` (e.g. the head camera) or `v4l2` (e.g. wrist cameras on a camera computer), and `host`/`interface`/`mac` say which machine it is attached to. Images are neither cropped nor mirrored, and cameras are not synchronised with each other.

## Controlling the robot from Python

```python
from unitree_gpt6.r1d_draw.src.robot import connect

robot = connect("./device.json")
print(robot.state()["arms"]["right"])  # read-only
png = robot.photo("head")  # read-only, PNG bytes
robot.take_over()  # hold in place: repeats the measured posture and torque,
# then blends to model gravity compensation within 3 s
robot.move({"points": [{"q": {"28": 0.25}, "duration_s": 3.0, "dwell_s": 0.5}]})
robot.stop()  # cancel the route, keep holding
robot.relax()  # arm stiffness and feed-forward fade to zero in 3 s, damping
# stays; the grippers keep holding
robot.release()  # stop commanding; motors keep the last frame, nothing relaxes
robot.close()
```

A route point's `q` maps motor slots to radians and may contain only the waist, the arms and the head. Without `profile` a point starts and ends at rest; `ease_in`, any number of `linear`, then `ease_out` form one continuous move that does not stop at its inner points. The controller refuses routes that leave the joint limits, peak above 0.3 rad/s, or change joint speed by more than 0.01 rad/s between neighbouring points.

Taking over needs: no other program commanding, arms at rest, motors not hot (start below 72 °C, stop at 75 °C). On a fault the controller stops commanding and the motors keep their last frame; `status()` gives the reason.

## Creating a painting task from scratch

### 1. Plan

```bash
r1d-draw init-task --output ./my-task
```

The output directory must be new:

| File | What to fill in |
| --- | --- |
| `device.json` | Robot and camera connections, model, state storage |
| `strokes.json` | Paper size, colors, stroke order and paths on the paper |
| `measurements.json` | Paper, brush, paints, water, grippers, waist, entry and park routes |

### 2. Configure the device and the model

- `device`: robot address, SSH user, interface name and MAC, local source address (optional), name of the environment variable that holds the password.
- `cameras`: host, kind and source of every camera.
- `model`: `urdf`; optional `base_frame` (default `pelvis_link`), `left_frame`/`right_frame` (default `<hand>_wrist_yaw_link`) and `left_tool_transform`/`right_tool_transform` (4×4 transform from the hand frame to the brush tip).
- `motion`: speeds and tolerances, see [Motion settings](#motion-settings).
- `workspace` (optional): `lower`/`upper` bounds of the brush tip, used by the offline check.
- `storage`: directory of the execution records; one robot uses one directory.

### 3. Measure and record

```bash
r1d-draw observe --config ./my-task/device.json --label rest --output ./my-task/rest.json
r1d-draw photo --config ./my-task/device.json --camera left_wrist --output ./my-task/wrist.png
```

`observe` only reads; it saves the joints and the brush-tip pose of both hands. Measurements that move the arm use the Python interface above. Record in `measurements.json`:

- `paper`: size, at least 3 non-collinear anchors and at least 1 independent check point (paper coordinates `uv_m` and robot coordinates `xyz_m`).
- `brush`: painting attitude `rpy_rad`, width, longest single stroke, lift between strokes `clearance_m`.
- `materials`: one station per color (and for water); `action` is the complete pose sequence, `contact_index` names the contact/immersion pose, `dwell_s` is the time spent there.
- `robot`: painting hand, `grip_q2`, `waist_yaw`, entry route `entry_q14`, park route `park_q14` (first item `null`, replaced by the last lifted point when compiling), transfer height `transfer_z_m` and `clearance_review`.

FK feedback does not show that the brush touches. After confirming a real contact set `contact_verified=true` and a `source`.

### 4. Compile and export

```bash
r1d-draw prepare --config ./my-task/device.json \
  --strokes ./my-task/strokes.json --measurements ./my-task/measurements.json \
  --job-id my-painting-attempt-1 --output ./ready/my-painting-attempt-1
```

This fits the paper, maps the strokes, arranges the job, solves all IK and checks the paths, all offline. A large picture (a hundred strokes or more) takes several minutes. The output directory must be new:

| Output | Use |
| --- | --- |
| `preview.svg` | Coverage preview from the final strokes and brush width |
| `plan.json`, `build_plan.py` | The complete end-effector plan and a script that reproduces it |
| `job.json` | All batches and joint trajectories |
| `calibration.json`, `device.json`, `manifest.json`, `report.json` | Calibration, configuration, file hashes, compile report |
| `run.sh` | Starts this job |

After changing a source input run `prepare` again; an edited export fails its hash check.

### 5. Run

Check the preview, the report, the scene, the start posture, grip, waist and the whole path, make sure no other program commands the robot, then:

```bash
bash ./ready/my-painting-attempt-1/run.sh
```

The runner checks that waist and grippers match the job and that the arms are at the first point of `entry_q14`, then takes over, follows the entry route, runs batch after batch, parks and keeps holding. From another terminal:

```bash
r1d-draw status  --config ./ready/my-painting-attempt-1/device.json
r1d-draw stop    --config ./ready/my-painting-attempt-1/device.json
r1d-draw relax   --config ./ready/my-painting-attempt-1/device.json
r1d-draw release --config ./ready/my-painting-attempt-1/device.json
```

Continuing after an interruption:

```bash
bash ./ready/my-painting-attempt-1/run.sh --resume          # continue at the unfinished batch
bash ./ready/my-painting-attempt-1/run.sh --resume --lift   # first lift the brush straight up to the transfer height
```

Finished batches are not painted again; the interrupted batch is done again from its start. With the brush tip below that batch's lift height, resuming is refused unless `--lift` is given.

## Motion settings

Set them in `motion` of the source `device.json` and run `prepare` again:

| Setting | Default | Meaning |
| --- | --- | --- |
| `max_speed_rad_s` | 0.09 | Peak joint speed with the brush in the air |
| `contact_speed_rad_s` | 0.05 | Peak joint speed when touching down, leaving, dipping, and within strokes |
| `posture_speed_rad_s` | 0.05 | Peak joint speed of the entry and park routes |
| `stroke_speed_m_s` | 0.01 | Brush-tip speed along a stroke |
| `travel_speed_m_s` | 0.05 | Brush-tip speed through the air (lowered automatically where a joint would be too fast) |
| `fk_position_tolerance_m` / `fk_orientation_tolerance_rad` | 0.003 / 0.08 | IK/FK agreement required when compiling |
| `start_tolerance_rad` | 0.04 | Allowed difference between the arms and the first entry point at the start |
| `grip_tolerance` / `waist_tolerance_rad` | 0.18 / 0.03 | Allowed difference of grippers and waist from the job at the start |
| `check_step_rad` | 0.02 | Joint spacing of the offline path check |

Built into the controller: feedback older than 0.25 s, another command publisher, a motor fault, 75 °C, tracking error 0.15 rad, joint speed 0.5 rad/s, a command gap of 0.08 s.

## Data formats

| Input | Main fields |
| --- | --- |
| `strokes.json`, `r1d-draw-strokes/v1` | `purpose`, `paper_size_m`, `palette`, ordered `strokes` (`id`, `color`, `points_uv_m`, optional `width_m`); optional `reload_every`, `wash_station`, `stroke_sample_spacing_m` |
| `measurements.json`, `r1d-draw-measurements/v1` | `paper`, `brush`, `materials`, `robot` |
| Compiled job, `r1d-draw-job/v1` | `job_id`, `hand`, `grip_q2`, `waist_yaw`, entry/park routes and `LOAD`/`PAINT`/`WASH` batches; each with `action`, `joint_path_q14` and the straight move to it, `travel_q14` |

A `PAINT` batch holds all strokes of one load; each stroke gives its contact range in `action` with `start` and `end`, both included. `LOAD`/`WASH` name their contact pose with `contact_index`.

## Offline demo and tests

```bash
r1d-draw demo-workflow --output ./offline-demo
python -m unittest discover -s unitree_gpt6/r1d_draw/tests -t .
```

`demo-workflow` records, compiles, exports and runs a synthetic task with the simulated robot and connects to nothing. The kinematics tests need a real model: `export R1D_DRAW_TEST_URDF=/path/to/r1d.urdf`; without it they are skipped.

## Known limits

- The offline check has no collision detection; review the whole path yourself and record it in `clearance_review`.
- There is one transfer height; a lower one close to the body has to be arranged through the station poses for now.
- Gains are the controller's built-in base values and cannot be configured yet.
- Correcting the tip offset at the start of each batch solves IK while the job runs, which pauses a few seconds between batches.

The code is under the repository's [Apache-2.0 licence](../../LICENSE).
