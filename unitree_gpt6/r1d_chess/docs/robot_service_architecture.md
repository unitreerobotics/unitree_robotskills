# R1-D Chess: Robot Service Setup and Motion Command Path

English | [中文](robot_service_architecture.zh.md)

This document explains how the chess service is established and how motion is sent to the R1-D.

## I. Three-layer service structure

```
bash r1d client ...           (ZMQ REQ, ipc:///tmp/astra_r1d_chess.sock)
        │  JSON commands
        ▼
bash r1d serve  →  src/r1d_chess/robot/serve.py  R1DService (ZMQ REP)
        │  inherits RobotService from src/r1d_chess/robot/internal_service.py
        │  IK / interpolation / preflight / token / checkpoints
        ▼
src/r1d_chess/robot/r1d_transport.py  InternalDDS
        │  subscribes to rt/lowstate, rt/lowcmd (read-only)
        │  a background thread sends six groups of joint targets through the Runtime IPC every 20ms
        ▼
r1d-runtime.service  (bash r1d runtime serve, systemd user service)
        │  RuntimeServer + UnitreeSDK2Driver
        ▼
      DDS LowCmd → R1-D robot
```

### 1. Bottom layer: Runtime (`src/r1d_chess/robot/r1d_runtime.py`)

- `load_experiment()` reads `config/runtime.json`, adds the `src` of the bundled Runtime (`third_party/robot_runtime`) pointed to by `runtime_root` to `sys.path`, then parses the experiment yaml (R1-A7-D mobile_wbc) and rewrites the network interface and camera (TeleImager) configuration.
- `serve()` starts `RuntimeServer` (the subclass `R1DServer` clamps slight limit violations in the feedback back within the limits). It occupies DDS directly and exposes three IPC endpoints: `runtime_state`, `runtime_cmd`, `runtime_request`. What actually runs permanently is the systemd user service `r1d-runtime.service`.
- `R1DRuntimeClient` is the IPC client; importing or connecting by itself does not take over control:
  - `connect()`: subscribes to the Runtime state.
  - `begin()`: sends `start_control` to request a control session. The requested components are `left_arm, right_arm, waist, head, left_ee, right_ee`; the chassis and lift are not included by default.
  - `send_joint_targets()`: every call must carry the qpos of all six groups above, otherwise it raises an error; the caller must keep sending before the cmd timeout.
  - `end()`: sends `stop_control` to end the session.

### 2. Middle layer: InternalDDS (`src/r1d_chess/robot/r1d_transport.py`)

- Motor indices: left arm 15–21, right arm 22–28, waist yaw 13, head pitch/yaw 29/30, left/right gripper 31/33.
- It only subscribes to raw DDS (`rt/lowstate` to read feedback, `rt/lowcmd` to detect whether there are still other publishers) and **does not create a second LowCmd publisher**.
- All IPC sockets are created, used and closed in the background thread `_run()`; the outside submits requests through `_rpc()`, which queues them (timeout 20s).
- **`enable` → `_begin()`** does the following in order:
  1. Checks that the Runtime is idle (no `control_active`/`runtime_error`/`reset_pose_active`) and the joints are stationary (|dq| ≤ 0.1).
  2. `_check_reference()`: no thermal protection; the head is within 0.35 rad of the `head_q_rad` reference position (it ramps there after takeover), and the waist is near `waist_q_rad` (≤ `head_waist_limit_rad`); the lift height differs from `lift_height_m` by no more than ±5mm; the chassis and lift velocities (normalised values of `rt/state_vel`) are linear ≤0.01 and angular ≤0.025, see [r1d_twist_dds_topics.md](r1d_twist_dds_topics.md).
  3. Releases AI mode with `MotionServiceSwitcher` (`release_control_service`).
  4. Waits 0.3s and confirms that no other LowCmd is being sent any more.
  5. Checks the reference pose once more, then calls `client.begin()` to establish the session.
  6. Sets the current joint positions as the initial targets; if the session was not established, restores the original motion service.
- **`_publish_once()`** runs once every 20ms:
  1. Does the health checks: whether lowstate is stale (> `feedback_timeout_s`), whether the Runtime session is still valid, Runtime errors and thermal protection, motor state and temperature (> `max_motor_temperature_c`), and whether the arm tracking error exceeds `arm_tracking_limit_rad` (0.20rad; when exceeded, `tracking_fault_snapshot` is saved).
  2. Moves `arm_sent` towards `arm_target`, with the arms rate-limited to 0.3 rad/s and the grippers to 0.8 rad/s (the gripper target is additionally clamped to within ±0.18 of the current feedback).
  3. Computes the arm feedforward qtau according to `config/arm_torque_ff.json` (gravity + friction + damping compensation + integral, magnitude- and rate-limited); see [r1d_joint_tracking_feedforward.md](r1d_joint_tracking_feedforward.md).
  4. The head and waist ramp at 0.2 rad/s from the position measured at the start of the session to the configured initial reference `head_q_rad/waist_q_rad`, and are held afterwards.
  5. Calls `client.send_joint_targets(targets, qtau)` to send the six groups of targets and the arm qtau.
  - If any check fails, it latches `fault`, first holds the measured position and ramps qtau down to 0 at 30 Nm/s, then calls `client.end()` to end the session. **There is no automatic recovery, and no automatic retract or restoring of AI**.
- The upper layer can only change target values and does not send commands directly:
  - `command_arm(q, tau)`: only changes `arm_target`; `q` must pass the URDF limit check, and `tau` must be all 0. The feedforward is computed only by the transport layer according to the configuration; the transaction engine is not involved.
  - `command_gripper(side, q)`: only changes the gripper target, q ∈ [0, 5.4].
  - `hold()`: sets the arm targets to the current feedback positions.
  - `release()`: only turns off transaction execution (`enabled=False`); the session is maintained as usual and keeps holding the final positions of the head, waist and both arms.
  - `release_waist()`: can be called only after execution is off, and actually ends the whole six-group session.
- `SimulatedDDS` is an offline stand-in that does not import Unitree and does not connect sockets; it is used by `--simulate` and the tests.

### 3. Chess service (`src/r1d_chess/robot/serve.py`)

- Entry: `bash r1d serve [--arm right|left] [--allow-motion] [--simulate] [--endpoint ...] [--log-dir ...]`.
- It uses flock (`/tmp/astra_r1d_chess_<hash>.lock`) to guarantee that there is only one instance, then binds the ZMQ REP endpoint `ipc:///tmp/astra_r1d_chess.sock` (only ipc or 127.0.0.1 is allowed).
- **Read-only by default**: without `--allow-motion`, `enable()` refuses directly.
- `R1DService` inherits the generic `RobotService` and adds several checks around `dispatch()`. For planning commands (`prepare_*`) and motion commands (`enable_control/set_gripper/execute_*`) it requires:
  - calibration validation to pass (`require_calibration`);
  - the request to carry `expected_active_arm`, matching the current acting arm;
  - the profile not to have changed since startup (SHA256 comparison); otherwise it requires restarting the idle service and producing the plan and token again.
- The generic replay commands `execute_plan / execute_cached_plan / restore_cached_transaction` are disabled; only a transaction that has just been preflighted may be executed.
- `set_scene` must match the scene in the profile; `shutdown` is refused while a session still exists.
- `set_active_arm` can switch only while execution is off, and clears the cached plan after switching. When the left arm acts, `LeftActorDDS/LeftActorMotion` in `arm_roles.py` swap the joint vectors and gripper indices (logical right = physical acting arm); **Cartesian coordinates are not mirrored**.
- Extra commands `prepare_init/execute_init`: when both arms are ≤ 0.10m from the init TCP, they hold in place; for an arm beyond the limit, a "vertical rise → high translation/reorientation → vertical descent" path is generated and preflighted.
- SIGINT/SIGTERM: while a session still exists, it only requests cancellation and calls `hold()`, and does not end the session.

## II. How one motion is sent to the robot

1. **Preparation phase**: `client prepare_transaction` (for a capture use `prepare_sequence`) submits TCP waypoints; each waypoint is `x/y/z/roll/pitch/yaw` (`waist_yaw_link` frame) plus `gripper: open/close/hold`.
   - `prepare_cached()` calls `_compile_cached()`, and `MotionHarness.prepare` does IK with the unitree_ros R1-D URDF (`model_urdf` in `config/runtime.json`) to generate joint-space samples `(q, duration)`. The IK is in `Arm.solve` of `src/r1d_chess/robot/r1d_urdf.py`: first analytic-Jacobian damped least squares (`solve_dls`), falling back to scipy `least_squares` only when that does not converge. For grasp retries, the whole success/failure path is preflighted repeatedly for 3 grasp heights × 3 grasp attempts (about 9 passes); this is a safety preflight kept on purpose, so do not remove it to speed things up.
   - At the same time the `BoardGuard` table/tool envelope is checked, and a reverse retract path and gripper undo actions are generated for every segment.
   - Both the success and the failure branch are fully preflighted. Finally a one-time token is returned.
2. **Enabling control**: the service has two layers of state.
   - **Session** (`session_active`): the first `client enable_control` (use `handover --arm X --execute` when there is a hold unit) goes through the `_begin()` above to establish the session, which then holds the head and waist references and both arms until the automatic unload after the move finishes and returns to init, until `retract` completes, or until `release_waist`.
   - **Execution** (`enabled`, `control_enabled` in the state): only means that executing one move is allowed. `release_control` or the end of execute turns it off, and the session is kept. When the session already exists, `enable_control` only turns execution back on.
   - Planning, `round build`, `set_active_arm` and `prepare_init` require execution to be off. So the flow is: take over → `release_control` → planning/preflight → `enable_control` → execute → execution turns off automatically → unload and end the session after returning to init → take over again in the next round.
3. **Execution**: `client execute_transaction --request {token}` enters `_run_cached_steps()` (`src/r1d_chess/robot/internal_service.py:1662`).
   - Inside each sample, smoothstep interpolation is used: `command = prev + (target - prev) * α²(3 - 2α)`, calling `dds.command_arm(command, tau)` about every 10ms; the arm that does not move keeps its original target.
   - Before every send, the feedback FK is compared with the expected FK: a deviation of 9mm is allowed at the low position and 18mm at the high position; beyond that it aborts.
   - During the vertical lift after a failed grasp, the path may be rebuilt at run time: first leave vertically, and connect laterally only after confirming that the transport clearance is reached.
   - Gripper actions are sent through `command_gripper`.
4. **Continuous sending**: the `InternalDDS` background thread reads the latest `arm_target` at 50Hz and, after rate limiting, sends it to the Runtime through `send_joint_targets`; the Runtime then writes DDS LowCmd.
5. **Visual checkpoints**: during execution it pauses at checkpoints and waits for another terminal to submit, with `serial next/submit`, the evaluation made after looking at the images, and then decides whether to continue, correct (`adjust` / `adjust_pitch` / `retry_high`) or fail and retract.
6. **Wrap-up**: after normal completion and return to init, the service turns execution off, ends the six-group session and unloads both arms (`arm_unload` in the result); results stopped midway or with a fault are not unloaded, and the session is kept.

## III. Key file index

| File | Purpose |
|---|---|
| `r1d` | Subcommand launcher, dispatches to the scripts |
| `src/r1d_chess/robot/serve.py` | R1-D chess REP service `R1DService` |
| `src/r1d_chess/robot/r1d_transport.py` | `InternalDDS` (the only owner of the Runtime IPC) / `SimulatedDDS` |
| `src/r1d_chess/robot/r1d_runtime.py` | Runtime experiment loading, `serve()`, `R1DRuntimeClient` |
| `src/r1d_chess/robot/handover.py` | Handover from the hold unit to chess control |
| `src/r1d_chess/robot/r1d_config.py` | Profile loading, calibration validation, head and waist references |
| `src/r1d_chess/robot/internal_service.py` | Generic transaction engine `RobotService` (preflight, token, execution, checkpoints) |
| `src/r1d_chess/robot/motion_harness.py` | Cartesian interpolation and IK (sends no commands) |
| `src/r1d_chess/robot/arm_roles.py` | Role/joint mapping when the left arm acts |
| `src/r1d_chess/robot/robot_client.py` | REQ client CLI |
| `config/runtime.json` | Location of the bundled Runtime, experiment, network interface, camera host |
| `third_party/robot_runtime/` | Bundled minimal robot runtime (server, IPC, SDK2 driver, R1-A7-D configuration) |
| `config/robot.json` | Geometry profile, init, head and waist/lift references, safety thresholds |

## IV. Geometry sources

The scene and release height in `config/robot.json` can be a measured calibration or a model estimate (see [r1d_scene_parameters.md](r1d_scene_parameters.md)); both can pass preflight and the motion-enable gate. Passing the gate only shows that the values are complete; it does not mean grasp and place have been accepted.
