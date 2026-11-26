# R1-D Sandwich

[English](README.md) | [中文](README.zh.md)

Use [SKILL.md](SKILL.md) to let an image-capable agent make one sandwich with a Unitree R1-A7-D and
Dex1 grippers. The agent reads the head and wrist cameras, estimates object positions, writes explicit
end-effector waypoints, and checks each result. A local service performs IK, collision and table checks,
motion execution, watchdog handling and stop-state management.

There is no object detector, no symbolic task planner and no pose search on the robot. The skill covers
the reference toaster, white plate and small lettuce dish, not arbitrary sandwich scenes.

## Directory and skill name

The directory is `r1d_sandwich/` and the skill identifier in `SKILL.md` is `r1d-sandwich`, the convention of
the other skills in this repository. The Agent Skills specification asks for the two to be equal; rename the
directory to `r1d-sandwich` when installing the skill somewhere that checks it.

## Capability

- Reads `head_left`, `left_wrist` and `right_wrist` images from the local service.
- Sends direct TCP position and `rpy_deg` waypoints through `bash r1d eef`.
- Keeps the lift fixed at `0.522` m for the supported workflow.
- Checks joint limits, self-collision, table clearance, registered obstacle boxes and per-motion travel.
- Supports simulation, shadow and real modes, plus no-motion planner preview and run-metric collection.
- Uses joint-space `home` between task stages.
- Converts image pixels to base-frame points with the calibrated cameras (`eef project`). This is geometry
  only: nothing is detected or recognised.
- On request, pushes the finished plate forward with both hands (`eef both`).

## Requirements

- Linux x86_64 with Python 3.10 and the packages in `requirements.txt`.
- Nothing else for the software. IK, collision checking, the executor, robot I/O, the simulated robot and the
  camera client are in `src/`, and the R1-D model (URDF and meshes) is in `assets/r1d/`. No other checkout is
  needed, in simulation or on the robot. The files copied from other projects (r1-ik, r1_d) say so in their
  first line.
- On the robot: a host on the R1-D network, a Unitree R1-A7-D with Dex1 grippers and the reference camera
  setup, explicit user authorisation for every motion, an on-site operator and a reachable hardware emergency
  stop.

The repository ships reference calibration JSON files in `assets/config/`, and the bundled model holds arm
inertials fitted to one robot. They are useful for checking the software path, but they do not replace
calibration for another robot or placement.

## Install

From this directory:

```bash
python3.10 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
```

From the repository root, the equivalent optional dependency set is:

```bash
python3.10 -m pip install -e ".[sandwich-r1d]"
```

`requirements.txt` lists the packages that this skill imports:

| Package | Used for |
|---|---|
| `numpy`, `opencv-python` | Arrays, images, camera model |
| `pin` (Pinocchio, brings `coal`) | Kinematics, gravity terms and collision checks |
| `robokit`, `warp-lang` | The IK solver behind every waypoint; runs on the CPU, or on a CUDA GPU when one is present |
| `pyzmq` | Camera client |
| `cyclonedds`, `unitree_sdk2py` (from GitHub) | Robot state and commands; imported in simulation too |

## Settings and `.env`

No setting is required. To change a default, copy the template and edit it:

```bash
cp .env.example .env
```

The program reads `.env` automatically. Shell environment variables have higher priority. No machine-specific
path is built into the code.

| Variable | Purpose |
|---|---|
| `R1D_SANDWICH_PYTHON` | Optional interpreter for `bash r1d ...`; default `.venv/bin/python`, then `python3` |
| `UNITREE_SDK2_ROOT` | Optional `unitree_sdk2_python` checkout to use instead of the installed package |
| `R1D_URDF` | Optional R1-D URDF for the camera model; default is the bundled one |
| `SERVER_URL` | Local service URL, default `http://127.0.0.1:8765` |
| `RUNS_DIR` | Optional run-output directory, default `<package root>/runs` |

Keep `.env`, run data, recordings, logs and local virtual environments out of version control.

## Environment check

Run the no-motion dependency check:

```bash
bash r1d check-env
```

It checks the bundled IK code and model, the Python imports (including the IK solver packages `robokit` and
`warp`) and every JSON file in `assets/config/`. It does not start a server, connect to a camera or move the
robot.

## Offline and no-motion validation

Run the offline tests from the repository root. They need only the Python standard library:

```bash
python -m unittest discover -s unitree_gpt6/r1d_sandwich/tests -p 'test_*.py'
```

Start a simulated robot at the task's lift height, then preview and check waypoints against it. These
commands do not execute a motion on a robot:

```bash
bash r1d serve --sim --fixed-lift 0.522
bash r1d eval '<candidate-json>'
bash r1d eval --brief '<candidate-json>'
bash r1d eef check '<waypoint-json>'
bash r1d eef both-check '<two-arm-json>'
```

`eval` uses the same planner as the service and reports reachability and posture metrics; `--brief`
prints one line per candidate. With `"side": "both"` it previews two-arm sequences, and `"end_at_start": true`
also checks the joint-space return to the starting joints. `check` and `both-check` check each waypoint from
the current pose. None of these commands is a substitute for on-site validation.

## Start the service

Start only the mode the user has authorised. Run commands from this directory.

```bash
bash r1d serve --sim --fixed-lift 0.522
```

```bash
bash r1d serve --shadow --fixed-lift 0.522
```

```bash
bash r1d serve --real --fixed-lift 0.522
```

| Mode | Reads robot state | Publishes motion |
|---|---|---|
| `--sim` | No | No |
| `--shadow` | Yes | No |
| `--real` | Yes | Yes |

In `--sim` the simulated robot starts at the fixed lift, so routes are previewed at the height the task uses.
Real mode asks the operator to type `s` once at start, and by default asks again before every motion.
Starting a real or shadow service and sending any motion both require explicit authorisation.

On the robot the service drives the arms with its own executor over `unitree_sdk2py` and reads the cameras
from the robot's image server with its own client. It needs no unitree-deploy checkout. The sandwich runs of
2026-11-17 and 18 used this executor; the run of 2026-11-25 used unitree-deploy's control process, which has
since been removed from this skill.

## Run workflow

The agent follows [SKILL.md](SKILL.md): its "One run" section is the procedure and its "Hard rules" section is
the single list of rules for a run. In outline:

1. The user authorises the motion; the workspace is clear and an operator can reach the hardware emergency
   stop.
2. Start the authorised service mode, create a run, bring the robot to the start pose (`prepare`: lift `0.522`,
   head pitched down `0.486` rad) and start recording:

```bash
bash r1d eef new
bash r1d eef prepare
bash r1d eef rec-start
bash r1d eef state
bash r1d eef look --cams head_left,left_wrist,right_wrist --raw
```

3. Read the toaster, plate and lettuce positions from the current images, and preview each stage with
   `bash r1d eval --brief`.
4. Run each step as one command that moves and then looks, and read the picture in the same turn. Every `look`
   also saves each view as `runs/latest/<cam>.jpg`, and every command that moves the robot deletes those files
   first, so a picture at that path was always taken after the last motion:

```bash
bash r1d eef exec '<waypoint-json>' && bash r1d eef look --cams right_wrist --raw
```

5. To measure a position, read pixels from a cropped view with pixel ticks and project them onto a plane:

```bash
bash r1d eef look --cams head_left --raw --crop 180,150,360,290 --scale 4
bash r1d eef project --cams head_left --z 0.61 "261,226;297,224"
```

6. Finish with `home`, `keyframe final`, `rec-stop` and metrics collection:

```bash
bash r1d eef keyframe final
bash r1d eef rec-stop
bash r1d metrics --run-dir runs/<run>
```

A waypoint may carry `"min_opening": 0.1`: `exec` then stops before that waypoint unless the hand's gripper is
at least that open, so the carry can follow a bread grasp in the same command.

The route is in [references/TASK_SANDWICH.md](references/TASK_SANDWICH.md); commands, payloads and efficiency
rules are in [references/PLAYBOOK.md](references/PLAYBOOK.md).

## Safety boundary

The rules the agent must follow are the "Hard rules" in [SKILL.md](SKILL.md). For the operator:

- Every motion needs explicit user authorisation. Real mode asks at the service terminal before each motion
  unless the operator deliberately starts it with `--confirm none`.
- The service checks joint limits, self-collision, table clearance, registered obstacle boxes and per-motion
  travel. It does not detect people, the plate, food or loose objects, and it does not verify grasp force or
  object attachment.
- A software stop is not a hardware emergency stop. Keep the hardware emergency stop within reach.
- The plate centre must be at `x >= 0.33` from the robot base before a run starts.

## Layout

The top level follows the [Agent Skills specification](https://agentskills.io/specification): `SKILL.md` plus
`scripts/`, `references/` and `assets/`.

| Path | Contents |
| --- | --- |
| [SKILL.md](SKILL.md), [SKILL.zh.md](SKILL.zh.md) | Agent workflow: one run, the hard rules, what to read for which task |
| [references/](references/) | Read as needed: task route, playbook, service API, experience, results (each in English and Chinese) |
| `scripts/` | The commands behind `bash r1d`; see the table below |
| `assets/` | `config/`: camera calibration and scene of the reference robot; `r1d/`: R1-D URDFs and meshes; `waypoint_template.json` |
| `r1d` | Entry point: `bash r1d {serve,eef,eval,posture,metrics,check-env,home}` |
| `src/` | The service's code by function; see the table below |
| `tests/` | Offline tests; they need only the Python standard library |
| `requirements.txt`, `.env.example` | Python packages; optional settings |

`bash r1d` sets the interpreter and `PYTHONPATH`. To run a tool in `src/` by hand, run it from `src/` as
`python3 -m <dir>.<tool>`.

| File in `scripts/` | Command | Purpose |
| --- | --- | --- |
| `eef_cli.py` | `bash r1d eef ...` | The agent's commands: `new`, `prepare`, `state`, `look`, `project`, `check`, `exec`, `both`, `both-check`, `grip`, `waist`, `home`, `keyframe`, `rec-start`, `rec-stop` |
| `eef_eval.py` | `bash r1d eval` | Offline preview of waypoint candidates with the service's planner, one or two arms |
| `arm_posture.py` | `bash r1d posture` | Shoulder abduction, elbow height and shoulder-yaw gravity torque of the current pose |
| `collect_metrics.py` | `bash r1d metrics` | Time, model calls, cost, stages and temperatures of one run |
| `check_environment.py` | `bash r1d check-env` | Bundled code, Python imports and configuration files; touches no hardware |
| `serve.sh`, `_env.sh` | `bash r1d serve` | Start the service; choose the interpreter and read `.env` |

| Directory in `src/` | Files | Purpose |
| --- | --- | --- |
| `service/` | `server.py` | The local HTTP service: routes, modes (`--sim`, `--shadow`, `--real`), start-up checks |
| | `client.py` | Client of the service; standard library only |
| | `session.py` | What `bash r1d eef` does: run and check waypoints, state summary, ruler and pixel views, keyframes |
| | `paths.py` | Settings from the environment and `.env`; locations of the bundled code and assets |
| `robot/` | `robot.py` | State, TCP moves, two-arm moves, grippers, home, waist, lift, stop; shadow check and operator confirmation |
| | `grip_exec.py` | Execution layer: the bundled executor plus grippers in the same command frame, and the simulated robot |
| | `simworld.py` | Toy tabletop scene for `--sim`: grasp, carry, release and stacking |
| | `go_home.py` | `bash r1d home`: return to home from an arms-down pose without passing under the table |
| | `ik_executor.py`, `dds_io.py`, `brain.py` | Copied from r1-ik: trajectory, command frames, watchdog and executor; robot I/O over `unitree_sdk2py`; the standard start pose |
| | `model.py`, `protocol.py`, `dynamics.py` | Copied from r1_d: joint groups and limits, `rt/lowcmd` packing, arm gravity and friction feedforward |
| `ik/` | `ik_path.py` | Planner: IK, joint limits, self-collision, table and obstacle checks, straight and joint-space paths, two-arm paths |
| | `kin.py`, `quality.py`, `solvers.py`, `models.py` | Copied from r1-ik: reduced model, posture and collision checks, IK solvers, model variants |
| `perception/` | `image_client.py` | Read-only client of the robot's image server |
| | `cameras.py` | Latest frame of each camera, read in a separate process; simulated cameras |
| | `camera_model.py` | Calibrated intrinsics and base-frame poses of the head pair and the wrist cameras |
| | `projection.py` | Image pixel to base-frame point (plane or head stereo) for `/project` |
| | `record3.py` | Three-view MP4 recorder behind `eef rec-start` / `rec-stop` |

The service contains no object detector and no scripted task stages. Its endpoints are listed in
[references/API.md](references/API.md). Runtime output goes under `runs/`, which Git ignores.

## License

Apache-2.0. See the repository [LICENSE](../../LICENSE).
