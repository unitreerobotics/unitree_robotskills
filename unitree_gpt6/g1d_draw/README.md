# G1-D robot painting

English | [中文](README.zh.md)

The agent workflow is in the [skill instructions](SKILL.md).

For a fixed-base G1-D with two 7-axis arms and Dex1 grippers. The workflow is **plan → measure → record → compile and export → execute**. An agent designs strokes from a reference and records the physical setup. Offline compilation produces complete joint trajectories and `run.sh`; execution needs no model directing individual strokes.

## Install

Linux (glibc ≥ 2.28), Python 3.10–3.13, and Git are required. Run from the **repository root**:

```bash
python3.10 -m venv unitree_gpt6/g1d_draw/.venv
source unitree_gpt6/g1d_draw/.venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r unitree_gpt6/g1d_draw/requirements.txt
export PATH="$PWD/unitree_gpt6/g1d_draw/scripts:$PATH"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
python -m pip check
robot-paint-execute --help
```

Dependencies are managed in [requirements.txt](requirements.txt), with pinned numerical libraries, Pinocchio, Unitree SDK, OpenCV, and PyZMQ. The SDK is pinned to a specific Git revision. Cameras and painting use the same dependencies. Keep the full checkout available and set `PATH` and `PYTHONPATH` in each shell used to run the CLI or generated scripts.

The module entrypoint is also available:

```bash
python -m unitree_gpt6.g1d_draw.src.main --help
```

| Dependency | Purpose | Open-source license |
| --- | --- | --- |
| [NumPy](https://github.com/numpy/numpy) | Numerical operations and trajectories | BSD 3-Clause |
| [SciPy](https://github.com/scipy/scipy) | Numerical IK | BSD 3-Clause |
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio), package `pin` | URDF, FK, and gravity compensation | BSD 2-Clause |
| [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python) | DDS communication | BSD 3-Clause |
| [CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds) | SDK communication dependency | EPL 2.0 / BSD 3-Clause, according to upstream licensing |
| [OpenCV](https://github.com/opencv/opencv-python) | JPEG decoding and PNG output | OpenCV 4.8+: Apache-2.0; bundled components retain their licenses |
| [PyZMQ](https://github.com/zeromq/pyzmq) | Camera configuration and image subscriptions | BSD 3-Clause; bundled components retain their licenses |

Image processing uses `opencv-python`; install only one OpenCV distribution in the environment. Python 3.11+ or ARM may require building CycloneDDS; follow the [SDK instructions](https://github.com/unitreerobotics/unitree_sdk2_python#installation), set `CYCLONEDDS_HOME`, and retry installation. Supply the robot URDF separately.

### URDF sources and compatibility

Official model downloads:

- [G1-D](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_d_description/g1_d.urdf) / [G1 + Dex1](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_mode_15_with_dex1_1.urdf): repository license [BSD-3-Clause](https://github.com/unitreerobotics/unitree_ros/blob/master/LICENSE).

Set `model.urdf`, `reference_positions` for every non-arm movable joint, both end-effector frames, and tool transforms. The executor currently supports fixed-base models with scalar joints and standard G1 waist joint names. The complete G1-D model needs adaptation for its wheel joints and torso names; it cannot be used directly for execution. Retain the model version and license when downloading, and ensure the model matches the actual device.

The G1-D waist uses slot 12 for yaw and slot 13 for pitch; slot 14 is not enabled. `waist_q3` is `[yaw, 0, pitch]`; velocities and gains follow the same order, with the roll entry unused.

## Layout

| Path | Contents |
| --- | --- |
| [SKILL.md](SKILL.md), `SKILL.zh.md` | Agent workflow for planning, measurement, delivery, and execution |
| `src/` | Calibration, compilation, task export, G1-D/Dex1 backend, and executor |
| [config/device.example.json](config/device.example.json) | Device, model, gains, motion, and storage configuration |
| `examples/` | Read-only connectivity, joint, and camera examples; execution API example |
| [assets/golden_gate_bridge/](assets/golden_gate_bridge/) | Golden Gate Bridge reference image |

## Check connectivity and cameras

```bash
DRAW_DIR=unitree_gpt6/g1d_draw
IFACE=enp4s0
CAMERA_HOST=192.168.123.164
ip -br addr
ip route get "$CAMERA_HOST"

python "$DRAW_DIR/examples/check_connection.py" \
  --config "$DRAW_DIR/config/device.example.json" --interface "$IFACE" \
  --camera-host "$CAMERA_HOST" --timeout 5 --output ./captures/connection.json

python "$DRAW_DIR/examples/read_joints.py" \
  --config "$DRAW_DIR/config/device.example.json" --interface "$IFACE" \
  --samples 3 --interval 0.5 --output ./captures/joints.json
```

These examples subscribe to state or images without acquiring arm control and need no URDF. The connectivity example reads fresh DDS feedback, queries camera configuration, receives images, and records each result in JSON.

| Data | Order and units |
| --- | --- |
| `q14` / `dq14` | Left 7 joints, right 7 joints; rad / rad/s; LowState slots 15–28 |
| Joint order within each arm | shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw |
| `waist_q3` | `[yaw, 0, pitch]`; rad; slots `[12, null, 13]`; roll is fixed at zero, and slot 14 is unused |
| `grip_q2` | Left, right; native device units; internal Dex1 slots 31, 33 |

Cameras use [TeleImager](https://github.com/unitreerobotics/teleimager) or a compatible single-message JPEG/ZMQ service. The configuration service normally listens on port 60000, accepts `GET_DATA`, and returns each stream's `zmq_port`. Use its actual configuration; common head, left-wrist, and right-wrist ports are 55555, 55556, and 55557:

```bash
python "$DRAW_DIR/examples/capture_images.py" \
  --stream "head=tcp://$CAMERA_HOST:55555" \
  --stream "left_wrist=tcp://$CAMERA_HOST:55556" \
  --stream "right_wrist=tcp://$CAMERA_HOST:55557" \
  --warmup-frames 3 --timeout 10 --output ./captures/images-1
```

Output includes original PNG images and `capture.json` with addresses, dimensions, hashes, and receipt times. Images are not cropped or mirrored; cameras and joint snapshots are not exposure-synchronized. The image service is separate from DDS. When the configuration service is unavailable but the image port is known, the connectivity example can use `--stream "head=tcp://$CAMERA_HOST:55555"` directly.

For DDS timeouts, check interface, domain, topic/type, and robot state publication. For camera timeouts, check the service, configuration port, and `enable_zmq`. Use the same dependency installation for image-only use.

## Create a painting task

### 1. Plan

```bash
robot-paint-execute init-task --output ./my-task
```

The output directory must be new:

| File | What to fill |
| --- | --- |
| `device.json` | Interface, model, gains, and state storage |
| `strokes.json` | Paper size, colors, stroke order, and paper-coordinate paths |
| `measurements.json` | Paper, brush, pigments, water, grip, waist, and entry/parking routes |
| `probe.template.json` | Measurement route and capture points |

Follow the [skill](SKILL.md) to analyze the reference, plan the whole composition, and adjust it using measured width and usable loaded-brush length. Use the [Golden Gate Bridge reference](assets/golden_gate_bridge/reference_gold_bridge.jpg) or supply another image.

### 2. Configure the device and model

Fill `device.json`:

- `device`: wired interface, DDS domain, state/command topics, internal or external Dex1.
- `gains`: deployment-specific arm, waist, and gripper PD gains.
- `model`: URDF, end-effector frames, calibrated brush-tip transforms, and reference values for non-arm joints.
- `storage`: attempt records and controller lock. Controllers for one robot share `robot_id`, state storage, and lock.

`init-task` reads the default gains from `config/device.example.json`: each arm's four shoulder and elbow joints use `kp=80`, `kd=3`, and its three wrist joints use `kp=40`, `kd=1.5`. Adjust the generated `device.json` for your device as needed; existing task configurations are not overwritten.

The URDF supplies IK, brush-tip FK, and gravity compensation. Fixed-base models with scalar movable joints are supported. Generate the non-arm reference configuration:

```bash
robot-paint-execute model-info --urdf /path/to/g1.urdf --output model.json
```

1. Open the generated `model.json` and check that the URDF path and both end-effector frame names match your robot model.
2. Replace every `null` in `reference_positions` with the position that joint holds during painting. The list includes every movable joint in the model except the arm joints; use radians for rotational joints.
3. Set `waist_yaw_joint = waist_q3[0]`, `waist_roll_joint = 0`, and `waist_pitch_joint = waist_q3[2]`: use actual yaw/pitch feedback and keep model roll at zero. Nonzero model roll is rejected. Correct mapping alone does not establish that the model's link geometry and payload match the actual device.
4. Update the matching fields under `model` in `device.json`, preserving existing brush-tip transforms and other settings. Do not replace the entire `device.json`.

For end-effector poses, `x/y/z` are measured in the URDF root frame in meters; `roll/pitch/yaw` are XYZ Euler angles in radians. Brush-tip transforms instead describe the tip's position and orientation relative to the end-effector frame.

Internal Dex1 uses LowCmd/LowState slots 31 and 33. For external grippers set `gripper_interface=external` and provide each side's `state` and `command` topic under `external_grippers.left/right`.

### 3. Measure and record

First save read-only joint and tool feedback:

```bash
robot-paint-execute observe --config ./my-task/device.json \
  --label rest --output ./my-task/rest-observation.json
```

Fill the measurement entry, capture, withdrawal, and parking routes. Record at least 3 noncollinear paper anchors, at least 1 independent check point, and each pigment and water contact. Brush tests establish width, usable stroke length, contact height, and provenance. Preserve accepted material, grip, and posture records.

`capture_indices` identifies sampled points. Also list noncontact camera points in `lifted_capture_indices`, default `[]`, which must be a subset. Compile offline, then execute the probe:

```bash
robot-paint-execute compile-probe --config ./my-task/device.json \
  --probe ./my-task/probe.template.json --output ./my-task/probe.json
robot-paint-execute probe --config ./my-task/device.json --probe ./my-task/probe.json
```

Captures are saved under state storage at `attempts/<hash>/captures/`. FK feedback does not establish brush contact; captures default to `contact_verified=false`. After physical contact is confirmed, fill `measurements.json` with the values, `contact_verified=true`, and `source`.

Normal probe completion follows `park_q14` and retains powered hold. Query and release from another terminal before the next job:

```bash
robot-paint-execute status --config ./my-task/device.json
robot-paint-execute release --config ./my-task/device.json
```

For another measurement attempt, use a new `job_id` and compile again.

### 4. Compile and export

```bash
robot-paint-execute prepare --config ./my-task/device.json \
  --strokes ./my-task/strokes.json --measurements ./my-task/measurements.json \
  --job-id my-painting-attempt-1 --output ./ready/my-painting-attempt-1
```

This fits the paper, maps strokes, sequences the job, solves all IK, and checks paths offline. The output directory must be new:

| Artifact | Purpose |
| --- | --- |
| `preview.svg` | Coverage preview from final strokes and widths |
| `plan.json`, `build_plan.py` | Complete end-effector action plan and reproduction script |
| `job.json` | All batches and joint trajectories |
| `device.json`, `manifest.json`, `report.json` | Configuration, artifact integrity, and compilation report |
| `run.sh` | Launch this compiled task |

Source strokes, measurements, and calibration are preserved. The URDF path must resolve on the execution machine. Edit source inputs and run `prepare` again; directly changing exported artifacts fails integrity checks.

### 5. Execute

Review the preview, report, physical layout, entry, grip, waist, and complete clearance. Release the measurement controller, then run:

```bash
bash ./ready/my-painting-attempt-1/run.sh
```

Query, stop, or release from another terminal:

```bash
robot-paint-execute status --config ./ready/my-painting-attempt-1/device.json
robot-paint-execute stop --config ./ready/my-painting-attempt-1/device.json
# End this controller's hold after completion or stop
robot-paint-execute release --config ./ready/my-painting-attempt-1/device.json
```

The foreground executor makes no online model calls, IK solves, or per-stroke captures. After completing the compiled job, it retains powered hold. `job_completed=true` and `parked=true` indicate completed motion and parking. Ctrl-C stops an active run; after completion or stop, Ctrl-C releases. Stop or fault cancels remaining motion and retains the last published targets. `release` ends holding without parking. Recovery parking needs a separately reviewed route from actual state.

## Default work, rest, and grip targets

Defaults live in [src/postures.py](src/postures.py):

```bash
python -c 'from unitree_gpt6.g1d_draw.src.postures import default_work_q14; print(default_work_q14())'
python -c 'from unitree_gpt6.g1d_draw.src.postures import default_rest_q14; print(default_rest_q14())'
```

New tasks contain `entry_q14=[default rest, default work]` and `park_q14=[null, default rest]`. The first entry point represents the expected actual start. Compilation replaces `null` with the final lifted endpoint. Add intermediate clearance waypoints for the setup. Change the entry's last point for another work target or the parking route's last point for another rest target, then compile. During right-arm painting the left arm holds its entry-end angles.

`grip_q2` defaults to `[0.0, 0.0]` in left/right order, using Dex1 native units in `0..5.45`. Defaults are `gripper_kp=[5,5]`, `gripper_kd=[0.05,0.05]`, and feedforward `tau=0`; `tau_est` is feedback. Keep measurement and painting grip targets consistent. Work and parking retain those targets.

The arms may be parked during planning or waiting to reduce prolonged extension. The normal route is **work entry → measurement/test/painting → withdrawal → parking → holding**. A whole painting executes continuously and parks once at the end.

## Arrival and runtime checks

Compilation uses IK/FK to check that planned joints match target poses. Runtime contact checks use FK from measured joints, with separate tolerances. Defaults are listed below; adjust the source `motion` settings and recompile after changes:

| Check | Default behavior |
| --- | --- |
| Compiled IK/FK | Position 3 mm; orientation 0.08 rad |
| Contact arrival | Position 5 mm; orientation 0.08 rad; joint error 0.04 rad |
| Lifted endpoints/camera points | Record XYZ, orientation, and joint target errors without arrival gates |
| Final parking | Record joint error without an arrival gate; wait for 0.2 seconds of stable hold |
| Persistent tracking error | Stop after error exceeds 0.08 rad for 0.2 seconds |
| Stationary operations | Speed at most 0.1; arm/waist rad/s, gripper native units/s |
| Acquisition | Initial entry error at most 0.04 rad; drift within tracking tolerance triggers a newly checked bridge |
| Feedback | Stale after 0.5 seconds; initial connection timeout 10 seconds |

Lifted work-entry endpoints and ordinary batch withdrawals connect directly to the next segment. Camera captures, contact operations, washing, and final parking wait for stability when needed. Minimum clearance, grip/waist targets, speed, joint limits, configured workspace, and terminal-stop checks remain active. Legacy `lifted_xy_tolerance_m` and `lifted_z_tolerance_m` fields are ignored.

## Reuse a plan from the first stroke

When layout, brush, and robot configuration remain valid, reuse the stroke and measurement inputs. Give every attempt a new ID and output directory:

```bash
robot-paint-execute prepare --config ./my-task/device.json \
  --strokes ./my-task/strokes.json --measurements ./my-task/measurements.json \
  --job-id my-painting-attempt-2 --output ./ready/my-painting-attempt-2
bash ./ready/my-painting-attempt-2/run.sh
```

Each `job_id` permits one attempt, including failures that claim it. Query the original request and resolve its cause before creating another. Preserve shared execution records. A compiled, unattempted task can be launched later. Update affected measurements after paper, brush, or material changes. Interrupted jobs do not resume automatically.

## Data formats and Python API

| Input | Main fields |
| --- | --- |
| `strokes.json`, `astra-paint-strokes/v1` | `purpose="painting"`, `paper_size_m`, `palette`, ordered `strokes`; each has `id`, `color`, `points_uv_m`, optional `width_m` |
| `measurements.json`, `astra-paint-measurements/v1` | `paper` anchors/checks; `brush` orientation, width, usable length; `materials` pigment/water actions, contacts, dwell; `robot` grip, waist, entry, parking, transfer height, `clearance_review` |
| Compiled job, `astra-paint-execute/v1` | `job_id`, `hand`, fixed grip/waist, complete entry/parking routes, `LOAD`/`PAINT`/`WASH` batches |

Format identifiers remain compatible with existing data. `q14` contains the left arm’s 7 joints followed by the right arm’s 7 joints, in radians; grip and waist are stored separately. EEF `x/y/z` positions use the configured URDF root frame in meters; `roll/pitch/yaw` are XYZ Euler angles in radians. The tool transform relates the end-effector frame to the brush tip, so wrist position is not brush-tip position.

`width_m` records expected coverage width for previews and paper-boundary checks. Actual line width depends on the brush, contact height, and paint load, so set it from physical brush tests. Increasing this field alone does not change contact height or add fill paths.

Define the paper’s `u/v` directions explicitly. Calibration fits the paper’s position and tilt from measured paper points, then maps each paper path point into robot coordinates. If the paper is tilted, contact height varies with location instead of using one Z value across the sheet. This model supports flat paper; curved surfaces require adaptation.

In a compiled `PAINT` batch, stroke `start` and `end` specify an inclusive contact-index range in `action`. `LOAD` identifies its loading contact with `contact_index`.

See [examples/execute.py](examples/execute.py) for Python integration. The public namespace is `unitree_gpt6.g1d_draw.src`. Individual `calibrate`, `build-plan`, `compile`, and `validate --model-check` commands are also available.

## Offline walkthrough

```bash
robot-paint-execute demo-workflow --output ./offline-demo
```

Synthetic data exercise recording, compilation, export, and simulated execution without connecting to a robot. Inspect `offline-demo/demo_report.json` and `offline-demo/ready/preview.svg`. Use a new output directory each time.

Code uses the repository's [Apache-2.0 license](../../LICENSE). The Golden Gate Bridge reference was copied from `pic/gold_bridge.jpg` in the earlier painting project. Its redistribution permission was not recorded; the software license does not grant rights to this image.
