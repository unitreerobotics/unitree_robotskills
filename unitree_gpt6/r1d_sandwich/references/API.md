# R1-D sandwich service API

English | [中文](API.zh.md)

See also: [Playbook](PLAYBOOK.md), [Experience](EXPERIENCE.md).

The local HTTP service owns DDS, IK, safety checks, the cameras and execution. The agent talks to it through
`bash r1d eef` or `R1DClient` in `src/service/client.py`. The client uses only the Python standard library and
disables desktop HTTP proxies for the local robot connection.

## Contents

1. [Process model](#process-model)
2. [Starting the service](#starting-the-service)
3. [Frames and orientation](#frames-and-orientation)
4. [Endpoints](#endpoints)
5. [Motion results and refusal codes](#motion-results-and-refusal-codes)
6. [CLI mapping](#cli-mapping)
7. [Safety boundary](#safety-boundary)

## Process model

```text
agent
  -> bash r1d eef / src/service/client.py
  -> 127.0.0.1:8765
  -> src/service/server.py
       -> src/ik/ik_path.py                 IK and checks
       -> src/robot/robot.py                state and motion routing
       -> src/robot/grip_exec.py            execution and gripper frames
       -> src/perception/cameras.py         head and wrist frames
       -> src/perception/image_client.py    reads the robot's image server (ZMQ, read-only)
       -> src/perception/projection.py      image pixel to base-frame point
```

The service is the only process that sends robot commands. Configuration is loaded from
`assets/config/`. Runtime logs, images and recordings go under `runs/`.

## Starting the service

Run from the directory containing `SKILL.md`:

```bash
bash r1d serve --sim --fixed-lift 0.522
bash r1d serve --shadow --fixed-lift 0.522
bash r1d serve --real --fixed-lift 0.522
```

| Mode | Reads the robot | Publishes motion | Purpose |
|---|---|---|---|
| `--sim` | No | No | Offline development with a toy scene |
| `--shadow` | Yes | No | Real feedback, IK and executor checks; `executed=false` |
| `--real` | Yes | Yes | Shadow check followed by real execution and confirmation |

The default address is `http://127.0.0.1:8765`. Override it with `SERVER_URL` in the environment or the package
`.env`. Start only the mode the user authorised. Real and shadow runs require explicit authorisation, and every
motion requires a new explicit authorisation.

`--fixed-lift 0.522` prevents lift changes during the task. Direct EEF commands always pass
`auto_lift=false`. The reference runs used this fixed value. In `--sim` the simulated robot starts at the
fixed lift, so a route can be previewed at the height the task uses; without `--fixed-lift` it starts at
`lift_m` from `assets/config/scene.json` (`0.665`).

`--temp-guard` enables the service's `70 C` start refusal and `80 C` trip. It is off unless requested.

For real mode, the service asks the operator to type `s` once before it connects, and the default
`--confirm all` asks at the service terminal before each motion. It requires an interactive terminal. Any
weaker confirmation mode is a deliberate operator decision, not something the agent may assume.

On the robot the service drives the arms with its own executor over `unitree_sdk2py`: the 250 Hz command
loop, tracking watchdog and soft stop run inside the service process.

## Frames and orientation

- Frame: R1-D URDF base, metres, x forward, y left, z up. TCP is the Dex1 finger-pad midpoint.
- At the standard pose, tool x points along the fingers, tool y is the opening axis and tool z is up.
- Give at most one of `orient`, `rpy_deg` or `quat_wxyz`. If none is supplied, the current orientation is kept.
- `orient="forward"` points the fingers forward.
- `orient="down"` points the fingers down; `yaw_deg` rotates the opening axis.
- `orient="flat"` keeps the fingers forward and the opening axis vertical, suitable for carrying a flat slice.
- `rpy_deg=[r,p,y]` uses `R = Rz(y) * Ry(p) * Rx(r)` in the base frame.
- A direct EEF waypoint normally supplies both `pos` and `rpy_deg`.

## Endpoints

| Method and path | Request body | Main response |
|---|---|---|
| `GET /health` | None | Mode, busy/stopped state, cameras, pixel-projection status, confirmation mode, fixed lift, version and log directory |
| `GET /state` | None | Joints, TCP, gripper opening, head, lift, waist, temperatures and machine state |
| `GET /image/<cam>.jpg` | None | JPEG for `head_left`, `head_right`, `left_wrist` or `right_wrist` |
| `GET /poses` | None | Taught poses and the current scene model |
| `GET /sim/scene` | None | Simulation objects, attachments and completion flags |
| `POST /move` | `{side, pos?, orient?/rpy_deg?/quat_wxyz?, yaw_deg?, speed?, linear?, auto_lift?, check_only?}` | Motion or check result |
| `POST /move_both` | `{left:{pos, rpy_deg}, right:{pos, rpy_deg}, speed?, linear?, check_only?}` | Both TCPs at the same time, checked as one path: straight lines by default, one joint-space path with `linear:false` |
| `POST /gripper` | `{side, opening}` or `{side, action:"open"\|"close"\|"grasp"}` | Opening, holding flag and execution result |
| `POST /home` | `{side:"left"\|"right"\|"both", speed?}` | Joint-space home result |
| `POST /move_joints` | `{side, q:[7], speed?}` | Joint-space replay with safety checks |
| `POST /waist` | `{target}` | Waist motion; arms keep their joint angles |
| `POST /head` | `{pitch?, yaw?}` | Head motion |
| `POST /lift` | `{height_m}` | Lift motion, disabled during a fixed-lift task |
| `POST /stop` | `{}` or `{clear:true}` | Latch or clear the software stop |
| `POST /scene` | `{table_x_min?, table_z?, obstacles?}` | Update the runtime scene model |
| `POST /project` | `{cam, pixel(s), z?, method?, window?}` | Back-project pixels with the calibrated camera model |
| `POST /resync` | None | Shadow mode: pull the twin back to the measured pose |
| `POST /sim/reset` | `{randomize?, seed?}` | Simulation reset |

An `/image` request fails when no fresh frame is available. `/project` is geometry only: it needs the real
cameras' calibration and is unavailable with the simulated cameras. The service has no object detector; the
agent inspects the JPEG directly, measures with `project`, and checks an estimate with `look --mark`.

## Motion results and refusal codes

Every motion result contains at least:

```json
{
  "ok": false,
  "executed": false,
  "reason": "human-readable explanation",
  "code": "unreachable",
  "stopped": false
}
```

`ok=false` does not raise an exception. Check it and stop a dependent sequence. Common codes are
`unreachable`, `too_far`, `collision`, `joint_limit`, `table`, `lift_range`, `invalid_argument`,
`operator_declined`, `watchdog`, `stopped`, `not_ready`, `executor_refused` and `refused`.

Useful successful fields include `final_tcp`, `planned_tcp`, `pos_err_mm`, `rot_err_deg`,
`min_clearance_m`, `min_table_gap_m`, `duration_s` and, for a gripper command, `opening` and `holding`.

A refused `linear:true` waypoint must be handled by returning with `linear:false` to the preceding hover point
and then approaching on a route that has passed. Do not repeat the same refused segment.

## CLI mapping

```bash
bash r1d eef new
bash r1d eef prepare
bash r1d eef state
bash r1d eef look --cams head_left,left_wrist,right_wrist --raw
bash r1d eef look --cams head_left --z 0.61 --mark "0.34,-0.06,0.61"
bash r1d eef look --cams head_left --raw --crop 180,150,360,290 --scale 4
bash r1d eef project --cams head_left --z 0.61 "261,226;297,224"
bash r1d eval '<candidate-json>'
bash r1d eval --brief '<candidate-json>'
bash r1d eef check '<waypoint-json>'
bash r1d eef exec '<waypoint-json>'
bash r1d eef both-check '<two-arm-json>'
bash r1d eef both '<two-arm-json>'
bash r1d eef grip right grasp
bash r1d eef home right
bash r1d eef keyframe final
bash r1d eef rec-start
bash r1d eef rec-stop
bash r1d metrics --run-dir runs/<run>
```

`check` sends only `check_only=true` moves from the current pose. `exec` stops at the first refused waypoint and
returns a nonzero status; it also stops before a waypoint whose `min_opening` the gripper does not meet. `eval`
uses the same planner but requires readable state, so a local service must be running; it never sends a motion
command. `project` calls `POST /project`. `look` saves each view under the run's `looks/` and as
`runs/latest/<cam>.jpg`; `exec`, `both`, `grip`, `waist` and `home` delete the files in `runs/latest/` before
they move the robot.

Use `set -o pipefail` at the beginning of a pipeline and `&&` between dependent stages. Otherwise a refused
`exec ... | tail` can be mistaken for success and the following stage may continue.

## Safety boundary

The planner enforces joint limits with margin, self-collision clearance, registered obstacle boxes, table
clearance and per-motion travel. The execution layer provides the shadow check, watchdog and stop state.

It does not detect the plate, food, loose objects or people. It does not verify grasp force or object
attachment. Approach from above, inspect the images, confirm the gripper opening, and stop when the scene or
result is uncertain. A software stop is not a hardware emergency stop.
