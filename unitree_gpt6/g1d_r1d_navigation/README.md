# g1d_r1d_navigation — AI Agent Navigation (Unitree Navi Agent)

English | [中文](README.zh.md)

Control a Unitree R1-D / G1-D wheeled base with natural-language instructions, letting the robot observe its surroundings and move autonomously.

Target platforms: **R1-D / G1-D wheeled base** (one codebase; the robot model is detected automatically at startup).

Current status: onboard sensors and controllers are connected through an MCP host. The system has been tested indoors on real R1-D and G1-D robots, with a person supervising throughout.

A model that supports image input uses [MCP](https://modelcontextprotocol.io) to call robot tools: view camera frames, move and turn, submit continuous velocity sequences, or return to saved places. It plans from the camera image and continues observing and adjusting while moving, carrying out tasks such as "move next to the table and stop" or "follow the corridor to the end, save it as point B, and return".

```text
"Walk to the water dispenser at the end of the corridor and stop"
        │
        ▼
                  image+text                        stdio/MCP                          DDS/RPC/ZMQ/UDP
┌───────────────┐            ┌────────────────────┐            ┌─────────────────────┐                 ┌────────────────┐
│ Vision LLM    │<──────────>│ MCP host           │<──────────>│ navi_mcp_server.py  │<───────────────>│ G1D / R1D base │
│ (GPT /        │            │ (DeepSeek-Harness  │            │ (Python side of     │                 │ camera / lidar │
│ Qwen-VL ...)  │            │ / Claude Desktop)  │            │ this project)       │                 │ SLAMWARE (G1D) │
└───────────────┘            └────────────────────┘            └─────────────────────┘                 └────────────────┘
                                        ^  polls :3082 every 1 s (read-only)  │
                                        └── navi-observer plugin <────────────┘
```

> **Warning:** This software drives a real robot. For the first run, a person must be on site with the emergency stop within reach, the area must be cleared, and the speed must remain at the default of 0.3 m/s. A software stop (`stop`) is not a substitute for the hardware emergency stop.

---

## Table of Contents

1. [What It Does](#1-what-it-does)
2. [Quick Start (Offline Stub)](#2-quick-start-offline-stub)
3. [Running on the Real Robot](#3-running-on-the-real-robot)
4. [Code Layout](#4-code-layout)
5. [Tool Reference and Semantics](#5-tool-reference-and-semantics)
6. [Three Motion Modes and Mutual Exclusion](#6-three-motion-modes-and-mutual-exclusion)
7. [Safety Layers](#7-safety-layers)
8. [Configuration Reference](#8-configuration-reference)
9. [Persona (System Prompt) and Observer Push](#9-persona-system-prompt-and-observer-push)
10. [Integrating Other MCP Hosts](#10-integrating-other-mcp-hosts)
11. [Porting to Other Robots](#11-porting-to-other-robots)
12. [Frequently Asked Questions](#12-frequently-asked-questions)
13. [No-Motion Preview and Tests](#13-no-motion-preview-and-tests)
14. [Acknowledgements, Attribution, and License](#14-acknowledgements-attribution-and-license)

---

## 1. What It Does

| Capability | Description |
|---|---|
| **Visual navigation** | The model views a camera frame → plans `move` / `turn` / `submit_sequence` → views the returned frame → plans again. Works without a map or localization. |
| **Reasoning while moving** | `submit_sequence` submits a velocity sequence covering the next few seconds and returns a frame captured during motion. The robot continues executing the current sequence while the model plans the next one. The server joins the sequences smoothly, so the robot does not need to stop after every step. |
| **Return to saved places** | With an onboard SLAMWARE map, use `save_place` to store a place and `navigate_to` to return using the onboard planner and obstacle avoidance. R1D uses the base lidar for mapping and localization; `save_place` stores places and `list_places` provides distances and bearings so the model can drive back using motion tools. |
| **Lidar collision avoidance** | The lidar reflex layer limits forward speed on every control tick according to the distance to obstacles ahead, without waiting for the model. See Section 7 for its detection range and blind spots. |
| **Observer push** | If the robot settles or is blocked by the lidar while the model is reasoning, the host plugin inserts a "[观测]" (observation) message into the conversation to report the current state. |
| **Offline stub** | Checks the full flow without a robot: the host connects to MCP, images enter the model's context, and the model calls tools. Useful for tuning prompts or switching models. |

**G1D hardware requirements**: a Unitree G1D wheeled base providing the `agv` RPC service, image_server running on the robot to provide a ZMQ camera stream, and optionally a SLAMWARE lidar with onboard SLAM. The host computer and robot must be on the same internal network. The default robot address is `192.168.123.164`.

**R1D** is detected automatically. It uses `cmd_vel` to control the base, a head left-eye camera providing RTP-H264 video (UDP 5002), and a Livox Mid-360 at the front of the base providing UDP point clouds (internal address `192.168.123.120`). R1D currently has no onboard SLAMWARE and does not support `navigate_to` map navigation, so it relies mainly on visual navigation. This service also supports Mid-360 localization and place marking, letting the model return to places using motion tools (see Section 5).

---

## 2. Quick Start (Offline Stub)

No robot connection is required. The stub replaces camera frames with numbered synthetic images. Tool names, parameters, and return fields match those on the real robot. The `memory_*` topological memory tools store data in a temporary directory, separate from the real robot's memory store.

### 2.1 Prepare the Python Side

```bash
cd unitree_gpt6/g1d_r1d_navigation                          # all commands in this README are run from the skill directory
python3 -m venv .venv && source .venv/bin/activate      # or: conda create -n navi python=3.10
python -m pip install -r requirements.txt
```

### 2.2 Prepare the Host (DeepSeek-Harness)

This project uses DeepSeek-Harness (DSH) as the default MCP host. This TypeScript agent framework manages conversations, calls the model, and provides MCP tools and a Web UI.

```bash
git clone https://github.com/deepseek-ai/deepseek-harness.git deepseek-harness   # inside the skill directory or anywhere else
cd deepseek-harness && pnpm install && pnpm run build && cd ..
export DSH_DIR="$PWD/deepseek-harness"                  # may be omitted when placed at ./deepseek-harness
```

> To use another MCP host, see [Section 10](#10-integrating-other-mcp-hosts). Any MCP host can launch `src/navi_agent/navi_mcp_server.py` directly.

### 2.3 Set the Model Key and Start

The default model is OpenAI's `gpt-6-astra`, configured in `host/navi-gpt.patch.yaml`. New sessions use reasoning level `low`, which can be changed in the UI. The model must be included in the pi-ai model catalog used by DSH; update older DSH versions first. To switch models, change both model-name entries in the patch.

```bash
export NAVI_PYTHON="$PWD/.venv/bin/python"   # lets the host launch the sidecar with the interpreter that has the dependencies installed
export OPENAI_API_KEY=sk-xxxx                # use DASHSCOPE_API_KEY for Bailian, DEEPSEEK_API_KEY for DeepSeek
chmod +x scripts/start.sh
scripts/start.sh --dry-run                 # preview the commands that will be executed
scripts/start.sh                           # = scripts/start.sh gpt stub
```

The terminal prints a URL containing a token (`http://127.0.0.1:3080/...`). Open it in a browser and create a new session. The default model is `gpt-6-astra`. If you change providers, select a model that **supports images**, such as Bailian's `qwen3-vl-plus`, from the drop-down. Then enter:

```text
向前走一段，然后左转 90 度，告诉我你看到了什么
```

(English: "Move forward a little, then turn left 90 degrees, and tell me what you see.")

The model will call `mcp__robot__observe`, `mcp__robot__submit_sequence`, and so on in turn, and the number on the stub frame increments accordingly.

You can also run a single check without opening the UI:

```bash
scripts/start.sh --headless "观察一下当前画面，然后向前走 1 米"
```

(English: "Observe the current view, then move forward 1 meter.")

### 2.4 `start.sh` Arguments

| Argument | Meaning |
|---|---|
| `gpt` / `bailian` / `deepseek` | Select the model provider (default `gpt`): OpenAI GPT (a gateway `baseURL` can be added in `host/navi-gpt.patch.yaml`) / Alibaba Cloud Bailian / official DeepSeek |
| `stub` / `real` | Simulated robot / real robot |
| `r1d` / `g1d` | Select the persona (default `r1d`). The sidecar detects the robot model automatically at startup; this argument must match the actual robot |
| `--dry-run` | Print the commands only |
| `--headless "message"` | Single headless run |

| Environment variable | Meaning |
|---|---|
| `DSH_DIR` | Path to the DSH repository |
| `NAVI_PYTHON` | Python interpreter used to run the sidecar (default `.venv/bin/python` in the skill directory if present, otherwise `python3`) |
| `OPENAI_API_KEY` / `DASHSCOPE_API_KEY` / `DEEPSEEK_API_KEY` | Key for the corresponding provider; if unset, `start.sh` prompts for it in the terminal (not echoed, not written to disk) |
| `DEEPSEEK_BASE_URL` | Overrides the endpoint when DeepSeek is accessed through a relay gateway |
| `NAVI_CAMERA_VIEW=0` | Do not open the browser camera view in real-robot mode |
| `NAVI_OBSERVER_PORT=0` | Disable the observer push plugin |
| `NAVI_RUN_LOG` | Run log path (default `logs/run-<timestamp>.jsonl`; `0` displays the log only on the camera page without writing it to disk) |
| `NAVI_LOG=0` | Do not write the process output log (default `logs/navi-<timestamp>.log`, stdout/stderr of DSH and the sidecar) |

Local paths and settings (`DSH_DIR`, `NAVI_PYTHON`, `PATH`, and the `NAVI_*` variables above) can be saved in `navi.env` in the skill directory. Copy it from `.env.example`; `start.sh` loads and exports its variables at startup. The file is excluded from version control. Do not store model API keys in it.

At startup, `start.sh`:

1. Checks the model API key.
2. Generates patches in `host/generated/` to configure the MCP service. In web mode, it also reads `host/navi-persona-<g1d|r1d>.txt`, creates a `navi` preset containing the persona and compaction chain, and makes it the default preset.
3. Sets `DSH_HOME=<repo>/.dsh-home` to keep session data separate from the global `~/.dsh` directory.
4. Starts DSH with `pnpm dsh --profile web --patch ...`.

---

## 3. Running on the Real Robot

### 3.1 Install the Unitree SDK

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python
cd unitree_sdk2_python && pip install -e .     # also installs cyclonedds
```

R1D's head camera uses `gst-launch-1.0` to decode RTP-H264. Install the GStreamer command-line tools and plugins, for example:

```bash
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-libav
```

### 3.2 Write the Site Configuration

```bash
cp src/navi_agent/config.example.yaml src/navi_agent/config.yaml
```

Confirm at least these two items:

- `net`: the name of the network interface connected to the robot's internal network (`auto` usually works; set it explicitly on machines with multiple interfaces).
- `camera.host`: the IP address of the robot running image_server.

Map navigation is disabled by default. On the first deployment, use visual navigation to check that the basic functions work.

### 3.3 Read-Only Self-Check (Base Does Not Move)

```bash
# Camera: live view + run log + task board in the browser (read-only, sends no motion commands; log and board data come from the running Navi)
python src/navi_agent/tools/camera_view.py --port 3081          # open http://127.0.0.1:3081/
python src/navi_agent/tools/camera_view.py --demo               # verify the HTTP path when no camera is available
```

### 3.4 Start

```bash
scripts/start.sh real          # R1D
scripts/start.sh real g1d      # G1D (uses the G1D persona)
```

For the first run, try these instructions in order: "Look around without moving and describe what you see", "Turn left 30 degrees in place", and "Move forward 0.5 meters". Once observation, turning, and forward motion work correctly, try an actual task.

### 3.5 Enable Map Navigation (Optional)

Prerequisite: a map of the site has been built on the robot with the vendor tools, and localization has been established.

1. In `config.yaml`, set `navi.navigation.enabled: true`, `map_id` (a name of your choice), and `mapping.backend: slamware_ros`.
2. After restarting, ask the model to call `navigation_preflight` and resolve each issue in `errors`. `frame` must match the `header.frame_id` of `rt/robot_pose`. Immediately after startup, subscription data may not be ready, causing `*_stale` / `lidar_not_ready`; check again after a few seconds.
3. Push or drive the robot to the target location and have the model call `save_place("起点")` (place name meaning "start point").
4. Verify that `navigate_to("起点")` returns the robot to it.

The end of `config.example.yaml` lists settings tested on G1D + SLAMWARE that relax some checks or adjust startup behavior, including `odom_required: false`, `receipt_time_freshness: true`, and `pre_align_deg: 60`. Each has an explanation. Check that it suits the current site before enabling it.

### 3.6 On-Site Checklist

- [ ] Emergency stop within reach, area cleared, operator positioned behind and to the side of the robot
- [ ] `robot_connected` and `camera_connected` are true in `get_status`
- [ ] `lidar.ok` is true in `get_status`, and `front_m` matches the actual distance ahead (test by holding a board in front of the robot)
- [ ] Speed left at the default (`motion.move_speed: 0.3`; sequence limit by robot model: G1D 0.5 m/s, R1D 1.0 m/s, automatically reduced by the lidar speed envelope according to forward clearance)
- [ ] Aware that the lidar has a **blind zone directly behind** the robot (measured effective returns on G1D cover roughly ±133°), so reversing has no reflex-layer protection
- [ ] Aware that the lidar **cannot see low objects on the floor** (cables, crossbars, objects lying on the floor); these can be detected only by the model from the camera image
- [ ] Glass doors and glass partitions are usually transparent to the lidar; these likewise rely solely on the camera image

---

## 4. Code Layout

```text
unitree_gpt6/g1d_r1d_navigation/
├── README.md / README.zh.md / .env.example / .gitignore
├── common/                           local preview configuration, contracts, runtime and tests
├── config/default.yaml               no-motion preview config for the repository's unified CLI (unrelated to the real-robot runtime)
├── scripts/
│   ├── start.sh                      one-command launcher (DSH + layered patches)
│   └── preview.sh                    no-motion preview (repository unified CLI)
├── tests/test_preview.py             unittest for the no-motion preview entry point
├── assets/                           publishable assets (currently empty)
├── src/
│   ├── main.py / agent.py / …        no-motion preview entry point for the repository's unified CLI (see Section 13)
│   └── navi_agent/                   ── Python: MCP server (sidecar) (all robot-side logic; flat import layout, not a Python package)
│       ├── navi_mcp_server.py            ★ Entry point. FastMCP stdio server; registers 17 tools (plus 7 memory_* tools registered by topo_memory) and orchestrates the layers
│       ├── robot_tools.py                pure tool logic: parameter guards (guard_move/turn/sequence), result shaping; no MCP dependency
│       ├── driver_factory.py             process-wide HeadlessDriver singleton + basic status/odometry/stop tools
│       ├── config.py                     configuration authority: DEFAULT_CONFIG + deep merge of config.yaml + runtime globals
│       ├── config.example.yaml           site configuration template
│       ├── core/                         ── hardware-independent control logic
│       │   ├── driver.py                 HeadlessDriver: minimal facade over robot + camera (connect/move/turn/stop/status)
│       │   ├── motion.py                 discrete motion executor: acceleration/deceleration ramps + duration + command-integrated odometry
│       │   ├── sequence.py               sequence executor: velocity segment merging (fresh/blend/replace), alignment, watchdog, latency EMA
│       │   ├── heading_hold.py           R1D heading hold: lidar-IMU gyro corrects straight-line yaw drift and stops turns on measured angle
│       │   └── places.py                 named place store: bound to map_id, atomic writes + process lock, corrupt files raise errors instead of falling back
│       ├── services/                     ── hardware/protocol adapters
│       │   ├── robots/                   robot-model layer: read-only G1D / R1D probe on first connection → profile (speed limits/capability flags) + matching base driver
│       │   │   ├── g1d.py                G1D RobotController: 20 Hz agv RPC velocity stream, command watchdog, zero-velocity release
│       │   │   ├── r1d.py                R1D base: normalized cmd_vel velocity stream + feedback freshness/alarm guard (conservative speed limits)
│       │   │   └── probe.py / profile.py / unknown.py   probe subprocess, robot-model parameters, refuse motion when the model is unrecognized
│       │   ├── rtp_camera.py             R1D left-eye RTP-H264 camera (gst decoding)
│       │   ├── topo_memory.py            topological memory: places/landmarks/frontiers/traversed edges, memory_* tools
│       │   ├── g1_agv_client.py          Python implementation of AgvClient (agv service API 1001 Move)
│       │   ├── camera.py                 image_server camera: GET_DATA negotiation + background ZMQ SUB frame reception
│       │   ├── lidar.py                  LidarGuard lidar reflex layer: forward sector/corridor clearance → clamp forward velocity
│       │   ├── livox_mid360.py           R1D Livox Mid-360: UDP point cloud → virtual 2D scan → LidarGuard
│       │   ├── lidar_slam.py             R1D lidar localization: point cloud → 2D scan-to-map matching → pose + map + relocalization
│       │   ├── navigation.py             SLAMWARE localization subscription + goto/cancel + arrival/stall/timeout determination
│       │   ├── slamware_ros_mapping.py   read-only map health: map, localization quality, TF (no write operations)
│       │   ├── slamware_idl.py           CycloneDDS IDL definitions for SLAMWARE topics
│       │   ├── mapping_config.py         typed validation of the navi.navigation.mapping section
│       │   ├── observer.py               embedded read-only HTTP status endpoint (:3082) polled by the host plugin
│       │   └── runlog.py                 run log: tool calls and result summaries (shown on the camera page, logs/run-*.jsonl)
│       ├── tools/
│       │   ├── stub_mcp_server.py        offline stub: same tool surface as the real robot, synthetic frames
│       │   ├── camera_view.py            browser camera view + run log + task board / recorded rooms (read-only, :3081)
│       │   ├── r1d_diag.py               R1D read-only diagnostics: model probe, feedback, topics, camera services, Livox broadcast
│       │   ├── livox_diag.py             Livox Mid-360 diagnostics and mounting extrinsic calibration (read-only by default)
│       │   ├── heading_check.py          R1D heading closed-loop on-site comparison test (moves the base; lidar localization as ground truth)
│       │   └── lidar_slam_replay.py      offline replay of lidar localization (consumes point clouds recorded with NAVI_LIDAR_RECORD; outputs trajectory and map images)
│       ├── tests/ + conftest.py          pytest unit tests (no hardware connection)
│       └── ruff.toml                     inherits the repository Ruff configuration, relaxes formatting rules
└── host/                             ── host side (DeepSeek-Harness configuration and plugins)
    ├── navi-gpt.patch.yaml           provider: OpenAI GPT (default; llm-pi-ai openai route)
    ├── navi-bailian.patch.yaml       provider: Alibaba Cloud Bailian
    ├── navi-deepseek.patch.yaml      provider: official DeepSeek
    ├── navi-persona-r1d.txt          ★ R1D persona (system prompt), written in Chinese (selected by start.sh by robot model, injected according to mode)
    ├── navi-persona-g1d.txt          ★ G1D persona (system prompt), written in Chinese
    ├── navi-observer.patch.yaml      mounts the observer push plugin
    ├── navi-observer/                TypeScript host observer plugin: pushes "[观测]" (observation) messages while the model reasons
    │   ├── index.ts                  plugin entry: polling, pushing, interrupting reasoning when necessary
    │   ├── policy.ts                 pure decision functions: when a push is warranted, when to escalate to an interrupt
    │   ├── policy.test.ts            regression cases for the above (`node --experimental-strip-types --test`)
    │   ├── continuation.ts           re-enables goal auto-continuation after upstream failures disable it
    │   ├── poll.ts                   fetches /state and /frame.jpg from :3082
    │   ├── narration.ts              extracts model narration, user instructions, etc. from DSH session events into the run log (POST /runlog)
    │   └── scope.ts                  determines whether a session belongs to Navi (preset or persona marker)
    └── generated/                    generated by start.sh at runtime (not committed): robot mount + web preset / headless prompt
```

### 4.1 Layer Responsibilities

| Layer | File | Responsibility | Does not |
|---|---|---|---|
| **Persona** | `navi-persona-<g1d\|r1d>.txt` | Explains safety constraints, when to use each tool, and how to plan sequences | Enforce safety limits |
| **Tool layer** | `navi_mcp_server.py` | Parameter validation, mode mutual exclusion, frame registration, latency statistics, result formatting | Interpret images |
| **Executors** | `core/sequence.py` `core/motion.py` | Convert velocity segments into commands sent at the base's control rate (G1D 20 Hz, R1D 50 Hz); merge sequences smoothly and stop on watchdog timeout | Read images |
| **Reflex layer** | `services/lidar.py` | Clamp the forward component according to lidar clearance on every tick | Clamp turning or plan detours |
| **Driver** | `services/robots/` | Robot-model detection + base velocity stream, command watchdog, zero-velocity servo release | Perform localization |
| **Navigation** | `services/navigation.py` | Hand goals to the onboard SLAMWARE, determine arrival/stall | Plan paths itself |

### 4.2 Data Flow of a Single `submit_sequence`

```text
model → submit_sequence(segments, frame_id, observe_after_s=3)
  └ robot_tools.guard_sequence       hard caps on speed/duration/displacement/turn angle; zero-velocity segments stripped
  └ _motion_entry_lock               reject if map navigation is running
  └ SequenceExecutor.submit          merge with the running sequence: same direction → blend, conflict → replace, stationary → fresh
       └ control loop (G1D 20 Hz / R1D 50 Hz): take current segment velocity → LidarGuard.clamp(vx) → RobotController.send_command
  └ wait observe_after_s seconds, capture a frame, register frame_id
  └ return: mode / remaining_s / suggested_horizon_s / latency_est_s / image
model reasons (robot keeps moving) → next submit_sequence …
```

`suggested_horizon_s` uses an exponential moving average of submission intervals to estimate how long each reasoning step takes. Submitted motion segments should cover that time where possible, so the robot does not stop while the model is still reasoning.

The distance traveled without a new observation is still limited by `navi.sequence.max_blind_m`, which defaults to 5 m and is converted to a duration using the plan's average speed. When reasoning is slow (measured at 20–30 s per step), use separate motion segments rather than trying to maintain continuous movement. The `note` in the result tells the model to travel 4–5 m, stop, observe, and then continue.

Motion and observation results include `clearance`, the lidar clearance in four directions at the time the frame was captured, in meters. The model does not need a separate `get_status` call to judge distance.

---

## 5. Tool Reference and Semantics

In the host, tool names take the form `mcp__robot__<name>` (`serverName: robot`).

### Perception

| Tool | Description |
|---|---|
| `observe(eye=None)` | Returns the current camera frame + `frame_id`. `eye`: `left`/`right`/`full`. A stale frame (received locally more than `camera.max_frame_age_s` ago) results in an error instead of returning an old image. |
| `get_status()` | Robot/camera connection and lidar reflex layer status (`lidar.corridor_front_m` travel corridor clearance, `front_m` ±30° sector, `ok`). |
| `get_odometry()` | **Command-integrated estimate**; not measured localization and not valid as evidence of arrival. |
| `get_localization()` | SLAMWARE map localization, measured velocity, freshness (requires the navigation configuration to be enabled). |

### Motion

| Tool | Description |
|---|---|
| `move(direction, distance_m)` | `forward`/`backward` translation, hard cap of 5 m per call, speed `motion.move_speed`. Blocks until complete and returns a settled frame. Returns `status="blocked"` when stopped by the lidar. G1D does not support lateral motion. |
| `turn(angle_deg)` | In-place turn, positive is left and negative is right, `10° ≤ \|angle\| ≤ 360°`. Blocks and returns a settled frame. |
| `submit_sequence(segments, frame_id, observe_after_s, alignment)` | Preferred for continuous motion. `segments=[{"vx","vyaw","duration"},…]`. Linear speed is capped at `capabilities.max_vx` (G1D 0.5, R1D 1.0 m/s). Angular speed is capped at the lower of `capabilities.max_vyaw` (G1D 0.6, R1D 0.4 rad/s) and `navi.sequence.max_vyaw` (default 0.5). Returns immediately. See Section 6 for details. |
| `sequence_status()` | Whether a sequence is running, remaining duration, current commanded velocity, and whether it has been stopped by the watchdog. |
| `stop()` | Clears the sequence / cancels the discrete motion / cancels navigation, and waits until the robot has settled. |

`move` / `turn` / `submit_sequence` share an in-place spin detector. If the robot accumulates 360° of rotation without leaving its position, the result includes a `spin` reminder to choose a direction and drive away, avoiding repeated turns in place while searching. R1D uses lidar localization to detect a position change, with a displacement threshold of ≥ 1 m.

### Task Board

| Tool | Description |
|---|---|
| `update_tasks(tasks)` | Updates the task list on the right side of the camera page (`:3081`). Format: `[{"name","status","note"}]`, where `status` is `todo`/`doing`/`done`/`dropped`. Only updates the display; does not move the base. Each time the model starts or completes a task, it resubmits the full list in parallel with the next motion tool. The page also lists rooms marked with `save_place` during the current run. Earlier places are not displayed but remain available through `list_places`. |

**Run log**: entries scroll in chronological order below the camera image and are written line by line to `logs/run-<timestamp>.jsonl` for later review. The log has two sources:

- The sidecar records tool arguments and result summaries through `services/runlog.py`, including distance traveled, turn angle, forward clearance, error causes, and spin reminders.
- The host plugin uses `host/navi-observer/narration.ts` to extract DSH session events: model output, reasoning summaries, user instructions, "[观测]" (observation) messages, turn stops, and upstream errors. It sends these to the status endpoint's `/runlog` via POST.

**Camera page (:3081)**: the R1D left eye (= the model's image, relayed by the sidecar) and the right eye (UDP 5003, received by the camera page itself) are displayed side by side filling the image area, with the run log below and the task board on the right; for a monocular camera only one image is shown.

### R1D Lidar Localization and Place Marking

R1D has no onboard SLAMWARE. This service implements 2D SLAM using the Mid-360 at the front of the base, in `services/lidar_slam.py`. It projects vertical structures 0.15–2.2 m above the ground (walls, pillars, furniture) into 2D and scan-matches them against a 5 cm occupancy grid built incrementally while driving. Localization does not depend on odometry because R1D's velocity feedback and IMU yaw are unavailable.

| Tool | Behavior on R1D |
|---|---|
| `get_localization()` | Lidar localization pose (`frame=lidar_map`); trustworthy only when `ok=true`. `state` is tracking / degraded / lost / relocalizing; a `hint` is attached in abnormal states |
| `save_place(name)` | Requires the robot to be stationary and localization to be trustworthy; the map is saved before the place |
| `list_places()` | Each place includes `distance_m`, `bearing_deg` (positive = left), and `heading_diff_deg`, which the model uses to drive back manually |
| `navigate_to(name)` | Not yet supported (returns `unsupported_on_r1d` together with manual driving instructions) |

- The map and places are stored in `src/navi_agent/data/lidar_map/<navi.memory.venue>/` (`map.npz`, `places.json`). Map updates are saved every 60 s and again on normal exit. If the process is forcibly terminated, map updates since the last save are lost.
- After restart, the service relocalizes against the old map, using previous trajectory poses and saved places as initial candidates. It cannot localize in unvisited areas; drive back to a previously visited area and remain still for about two seconds.
- Loop-closure detection is not supported. Error accumulates over long runs across large areas, and the map may show ghosting. Long corridors and open halls with few distinct geometric features also reduce matching quality.
- When changing sites or if the map is abnormal, start once with `NAVI_SLAM_NEW_MAP=1`. The old map and places are renamed and kept, then mapping starts from scratch.
- For tuning, start with `NAVI_LIDAR_RECORD=1` to record raw point clouds to `logs/lidar-*.bin.gz` in the skill directory. Replay them with `cd src/navi_agent && python -m tools.lidar_slam_replay <file>` to inspect the trajectory, match quality, per-frame processing time, and map image. Record a loop back to the starting point and use the distance between the final pose and the start to measure accumulated error.

### R1D Heading Closed Loop (Lidar IMU Gyroscope)

R1D's wheels may not point straight ahead at startup. In testing, driving straight for 1 m at 0.5 m/s produced about 10° of heading drift, and the base provides no usable yaw feedback. The base command thread therefore uses the Mid-360's built-in IMU gyroscope (UDP 56401) to correct heading through `core/heading_hold.py`:

- **Straight driving / sequences**: on every tick, "commanded yaw − gyroscope-measured yaw" is accumulated as heading error, and a proportional correction is added to the turning component to bring a deflected heading back.
- **Discrete turns**: when the gyroscope is trustworthy, the turn stops once the **gyroscope-measured turn angle** is reached (the command is ramped down `turn_stop_lead_s` seconds' worth of turn rate early to compensate for post-stop slip). When the gyroscope is unavailable, the turn stops based on command integration, and the angle lost during the start-up ramp is compensated. If the measured angle is not reached in time, the turn stops on timeout and the result includes `turn_stop`.
- **Measured values in results**: `measured_angle_rad` for `turn` (`angle_source=gyro`), `measured_yaw_change_deg` for `move`, and `heading_track` for `submit_sequence` (commanded yaw versus gyroscope measurement since the previous submission). These are also shown in the run log.
- **Fallbacks**: after startup, bias is estimated while the base command is zero. The robot must actually be stationary: pushing it or turning it by remote control will produce an incorrect estimate and require a Navi restart. The first turn checks the gyroscope direction; an inverted direction permanently disables the closed loop. If the IMU stream drops or the error reaches its limit, control falls back to open loop. No heading correction is applied while stopped.

Run `cd src/navi_agent && python -m tools.heading_check [--distance 1.5 --speed 0.5 --repeat 1]` for an on-site comparison. **This test moves the base. Stop Navi first and have a person supervise on site.** In both open-loop and closed-loop control, it runs four in-place ±90° turns and two straight runs followed by 180° turnarounds. Using lidar localization as ground truth, it reports turn-angle error, heading drift, and lateral offset, and suggests a value for `turn_stop_lead_s`. Details are saved to `logs/heading-check-<time>.json`.

### Map Navigation (Available When `navi.navigation.enabled: true`)

| Tool | Description |
|---|---|
| `mapping_capabilities()` | Read-only: capabilities of the current map backend. |
| `mapping_status()` | Read-only: onboard map, localization quality, TF. |
| `navigation_preflight()` | Read-only preflight: configuration, localization, lidar. At cold start, `*_stale` is retried briefly and automatically. |
| `list_places()` | Named places saved under the current `map_id`. |
| `save_place(name, overwrite=True)` | Saves the current fresh localization as a named place; requires the base to be stationary. An existing name is overwritten by default (`replaced=true` in the result); with `overwrite=false` a duplicate name is an error. |
| `navigate_to(name)` | Asynchronously navigates to a saved place; a `running` return only means the goal was submitted. If the target deviates from the robot's heading by more than `pre_align_deg`, the robot first turns in place to align. |
| `navigation_status()` | Arrival is indicated only by `state=arrived` with `active=false`; `stalled`/`timeout`/`fault` are not success. |

### Topological Memory (`services/topo_memory.py`)

Memory is stored separately for each robot model and venue (`navi.memory.venue`). It records visited places, landmarks, unexplored exits, and routes traveled between places for use in later tasks. These records provide clues and cannot replace localization. The offline stub also provides these tools, storing data in a temporary directory separate from the real robot's memory store.

| Tool | Description |
|---|---|
| `memory_context(task, frame_id, max_images)` | Compares the current frame with stored places; returns candidate places with reference images, targets matching `task`, and route hints from the current anchor |
| `memory_localize(place_id, frame_id, via)` | Called after the model confirms by image comparison that it is at a place: moves the anchor and records the edge from the previous anchor |
| `memory_save_place(label, evidence, frame_id, …)` | Saves the current location as a new place, or adds a view to an existing place |
| `memory_remember_landmark(label, evidence, frame_id)` | Records a landmark under the current anchor place |
| `memory_remember_frontier(label, direction, …)` | Records an unexplored exit or direction |
| `memory_mark_unavailable(kind, id, reason)` | Marks a place/landmark/frontier/edge as unavailable |
| `memory_list()` | Overview of all memory for this robot model and venue |

---

## 6. Three Motion Modes and Mutual Exclusion

| Mode | Tool | Characteristics | Use case |
|---|---|---|---|
| Discrete blocking | `move` / `turn` | Returns only on completion, with a settled frame | Alignment under 1 m, confined spaces |
| Continuous sequence | `submit_sequence` | Returns immediately, executes continuously in the background, reasoning while moving | Approach in open areas, obstacle detours, corridors |
| Map navigation | `navigate_to` | Onboard SLAMWARE planning + obstacle avoidance | Returning to saved places |

**Mutual exclusion**: all three are protected by the same `_motion_entry_lock`. While map navigation is in progress, `move`/`turn`/`submit_sequence` are rejected; `move`/`turn` first clear any running sequence. `stop` is available in every mode.

### Sequence Merge Rules

| Returned `mode` | Trigger | Behavior |
|---|---|---|
| `fresh` | No sequence currently running | Start from standstill |
| `blend` | New and old sequences in the same direction | Smooth transition within the `blend_s` window |
| `replace` | Direction conflict (e.g., forward ↔ backward) | Take over immediately |

`alignment=current` (default) starts the new plan when the server receives it. `alignment=continue` uses the time of `frame_id` as the plan's starting point, and the server trims the prefix already executed. The latter is allowed only when the old plan has constant velocity and consistency can be verified; otherwise it is rejected.

**Watchdog**: while a sequence is running, if the model submits no new sequence for more than `max(sequence.watchdog_s, 2×reasoning-latency EMA+1)` seconds, the robot gradually decelerates to a stop.

---

## 7. Safety Layers

The system checks and limits motion through the following layers. Lower layers rely less on model judgment:

| Layer | Mechanism | Default |
|---|---|---|
| 1. Persona | Persona section 「〇、最高约束」 (Section 0, top-priority constraints): never collide; people take priority (when approaching someone head-on, stop at ≥1 m by default, or as close as 0.5 m when the operator explicitly permits it); rely on the image for low objects; never fabricate arrival | — |
| 2. Parameter guards | `robot_tools.guard_*`: single translation ≤5 m, turn 10°–360°, hard caps on sequence speed/duration/displacement/turn angle | 5 m / model max_vx / model max_vyaw (≤0.5 rad/s) / 45 s |
| 3. Lidar reflex layer | `LidarGuard`: forward velocity is set to zero when clearance ahead is below `stop_m`. G1D decelerates linearly between `stop_m` and `slow_m`. R1D uses a speed envelope: allowed speed = max(0.2, (path clearance − stop_m) / 1.5 s), considering only a path as wide as the body, following an arc when moving and turning, and ignoring side walls. **Only the forward component is clamped. Pure in-place turns are unrestricted, but a continuous sequence also brakes its turning component when forward motion is blocked.** G1D uses the SLAMWARE 2D scan. R1D uses the front Mid-360 point cloud: points above 5 cm + 1.5 cm × distance in meters, and no higher than 1.5 m, count as obstacles. At 2 m, the height threshold is about 8 cm. Distance is measured from the front edge of the robot. | 0.45 m / 1.2 m |
| 4. Sequence watchdog | Ramped stop when the model is silent for too long | 8 s (adaptive) |
| 5. Driver watchdog | Sends zero velocity after `cmd_watchdog_s` has elapsed since the last command. After `cmd_watchdog_s + zero_release_s`, stops sending commands and releases the servos so the robot can be pushed by hand. `stop` also counts as a zero-velocity command, so release occurs about 1.5 s after `stop`. | 0.5 s / 1.0 s |
| 6. Navigation gates | Pose/state freshness, map health, lidar readiness, stationary start, stall/timeout determination | See configuration |
| 7. **Hardware emergency stop** | A human operator | **Mandatory** |

**Known blind spots (inform on-site personnel)**

- Lidar blind zone directly behind the robot: reversing has no reflex-layer protection.
- Low objects on the floor (cables, crossbars, a robot lying on the floor) are usually not detected by the lidar.
- Glass doors and glass partitions are usually transparent to the lidar.
- In-place turns are not constrained by the reflex layer; the sides of the body may sweep nearby objects while turning.
- G1D defaults to `navi.lidar.require: false`: forward motion is not blocked if the scan stream drops or the subscription fails. After confirming that the lidar works on site, set this to `true` in `config.yaml`.

**The R1D lidar differs from the G1D lidar** (mounted at the front of the base, 20 cm above the ground, tilted 30° forward and downward):

- It detects nearby objects 5 cm and higher (boxes, table legs, objects lying on the floor), which the G1D single-line lidar cannot see. The height threshold increases with distance, reaching about 8 cm at 2 m.
- **It cannot see overhanging objects close by that are above the upper edge of its field of view**: objects higher than 0.4 m at 0.5 m distance, or higher than 0.6 m at 1 m (tabletops, protruding counters, crossbars). The base being able to pass underneath does not mean the upper body can.
- The rear is occluded by the robot body, so readings exist only for objects above 0.6 m; when there is no rear data, reversing is not blocked.
- Floor-level objects lower than 5 cm (cables, door thresholds) are treated as ground; glass, mirrors, and black light-absorbing surfaces may produce no returns.
- When lidar data is missing or stale, forward velocity is set to zero (`robot.r1d.lidar_require`). Disabling the lidar with `NAVI_NO_LIDAR=1` falls back to "translation allowed only when the camera frame is fresh".
- **Known defect**: if the lidar window is covered by cloth, a bag, or a hand, or an obstacle is within 3 cm of the front edge, there may be no valid returns along the travel path. The reflex layer then treats the path as clear and allows motion instead of setting velocity to zero. Check that the lidar window is unobstructed before running. Do not rely on the reflex layer when a turn leaves the front edge close to an obstacle.
- R1D linear speed limit is 1.0 m/s and angular speed limit is 0.4 rad/s (`robot.r1d.max_vx` / `max_vyaw`). The actual speed is determined by the speed envelope according to forward path clearance: 1 m/s requires about 2 m of clear path ahead; when close to an obstacle the robot approaches at 0.2 m/s and stops at 0.45 m (`robot.r1d.lidar_headway_s` / `lidar_creep_mps`, or `navi.lidar.headway_s` / `creep_mps`). `mps_full` has not yet been verified by measurement; before the first high-speed run, it is recommended to verify the actual speed over a straight segment using `get_localization`.
- R1D camera: the 544×448 left-eye image is a stereo-rectified pinhole image (straight lines stay straight; it is not fisheye), with a horizontal field of view of about 105°, so objects in the center appear small and distant. Each frame provides the model with two images: a "normal field of view" image (about 66°, for judging what is ahead and distances) produced by enlarging the central 50% back to full size, plus the full ultra-wide image (for viewing both sides and searching for targets). `robot.r1d.camera_crop_ratio` adjusts the enlargement ratio.

---

## 8. Configuration Reference

Copy `config.example.yaml` to `src/navi_agent/config.yaml`, which is excluded from version control. Include only the settings you want to change. The service recursively merges them with `DEFAULT_CONFIG` in `src/navi_agent/config.py`, keeping defaults for the rest. Without `config.yaml`, all defaults apply and map navigation stays disabled. Configuration is read only at startup, so restart after making changes.

| Section | Main keys | Description |
|---|---|---|
| `net` | | DDS network interface; `auto` for automatic detection |
| `robot` | `model` `probe_timeout_s` `candidates` `g1d.*` `r1d.*` | Robot-model detection (`auto` runs a single read-only probe on the first connection); `robot.g1d` / `robot.r1d` override model parameters (`max_vx`, `max_vyaw`, `lidar_require`, `camera_crop_ratio`, ...) |
| `motion` | `move_speed` `rate` `ramp_time` `cmd_watchdog_s` `zero_release_s` | Discrete motion and driver parameters |
| `camera` | `host` `request_port` `camera` `eye` `max_frame_age_s` | image_server camera |
| `navi.guard` | `max_move_distance_m` `max_turn_angle_deg` | Hard caps for discrete motion |
| `navi.sequence` | `max_vx` `max_vyaw` `max_accel_mps2` `max_duration_s` `max_distance_m` `max_blind_m` `blend_s` `watchdog_s` `horizon_factor` | Sequence mode |
| `navi.lidar` | `enabled` `stop_m` `slow_m` `headway_s` `creep_mps` `require` `corridor_half_width_m` `yaw_offset_deg` | Lidar reflex layer |
| `navi.observer` | `port` | Observer status endpoint port; 0 disables it |
| `navi.memory` | `dir` `venue` | Storage directory and venue name for topological memory and the R1D lidar map (change `venue` when switching sites) |
| `navi.navigation` | `enabled` `map_id` `frame` `goal_frame` `places_file` `odom_required` `receipt_time_freshness` `max_distance_m` `timeout_s` `stall_s` `pre_align_deg` … | Map navigation |
| `navi.navigation.mapping` | `backend` `trust_slam_state` `require_map_health` `min_quality` | Map health gate |

See the inline comments in `src/navi_agent/config.py` for the meaning of each key.

**Python-side environment variables**

| Variable | Effect |
|---|---|
| `NAVI_NO_CAMERA=1` | Do not connect to the camera (motion-only debugging) |
| `NAVI_NO_LIDAR=1` | Disable the lidar reflex layer (debugging only; do not use on the real robot) |
| `NAVI_CAMERA_HOST` / `NAVI_CAMERA_REQUEST_PORT` | Override the camera address |
| `NAVI_LIDAR_RECORD=1` | R1D: record raw lidar point clouds to `logs/lidar-*.bin.gz` in the skill directory (for offline replay and tuning; about 1.5 GB per 10 minutes) |
| `NAVI_SLAM_NEW_MAP=1` | R1D: rename and keep the old lidar map and places, and build a new map from scratch |

**DDS topics / services used**

| Topic/service | Direction | Purpose |
|---|---|---|
| `agv` RPC (API 1001 Move) | Write | Base velocity |
| `rt/hispeed_state` | Read | Connection detection |
| `rt/slamware_ros_sdk_server_node/scan` | Read | Lidar reflex layer (G1D) |
| `rt/robot_pose` `rt/agv/odom` `rt/slamware_ros_sdk_server_node/state` | Read | Localization, measured velocity, SLAM state |
| `rt/move_base_simple/goal` `rt/slamware_ros_sdk_server_node/cancel_action` | Write | Navigation goal / cancel |
| `rt/slamware_ros_sdk_server_node/map` `.../robot_basic_state` `rt/tf` | Read | Map health (read-only) |

**Additional UDP ports used by R1D** (all on the robot's internal network, not via DDS)

| Port | Direction | Purpose |
|---|---|---|
| 5002 | Receive | Head left-eye RTP-H264 (requires the `stereo_patch_pc1` service to be running; opened automatically at startup) |
| 56301 | Receive | Livox Mid-360 point cloud (about 2084 packets/s) |
| 56100 ← 56101 | Send/receive | Livox commands: at startup and on stream loss, checks the point-cloud and IMU destination addresses and changes them to this host if needed. **This writes to the lidar configuration and persists.** `livox_diag --set-host` can only restore the point-cloud address. Disable automatic changes with `robot.r1d.lidar_auto_host: false`. |
| 56201 | Receive | Livox status broadcast (used only by the diagnostic tool) |
| 56401 | Receive | Livox built-in IMU gyroscope, used for heading control |

---

## 9. Persona (System Prompt) and Observer Push

### 9.1 Persona

`host/navi-persona-r1d.txt` and `navi-persona-g1d.txt` are the system prompts for R1D and G1D. Both are written in Chinese and describe how to observe, move, and handle errors. `start.sh` selects the file using the `r1d`/`g1d` argument, defaulting to `r1d`. Web mode adds the content to the preset; headless mode uses `personaPrefix`.

The general rules are shared and must be updated in both files. The robot-specific differences are:

- The R1D version explains the two images returned by each observation, Mid-360 lidar characteristics, and lidar localization and place marking. It does not provide `navigate_to`.
- The G1D version explains how to use the white arc of the base in the monocular image to judge distance, run SLAMWARE preflight, call `navigate_to`, and handle `stalled`. Its speed table uses the G1D limit of 0.5 m/s.

The prompts have the following sections:

- **〇 最高约束 (Section 0, top-priority constraints)**: how to judge whether the robot can proceed (considering only the travel corridor of base width), people take priority, complete tasks autonomously without asking for help, no cheating, the limits of the lidar layer, and how to treat "[观测]" (observation) messages.
- **一 感知 / 度量导航 (Section 1, perception / metric navigation)**: when `navigate_to` may be used (only for existing saved places), waiting for subscriptions during preflight, retries and alternatives after failures, handling of `stalled`.
- **二 运动 (Section 2, motion)**: task decomposition, tool selection by phase, passing through doors and corridors, the sequence working method (including reference curves from three recorded manual teleoperation sessions), speed and step size, termination conditions.
- **三 拓扑记忆 (Section 3, topological memory)**: when to call `memory_context`, confirming places by image comparison, when to save places / landmarks / unexplored exits.

Most rules address problems encountered in real-robot tests, such as prolonged wall-following, collisions with low objects lying on the floor, and goto failing to respond when the target is behind the robot. Review the corresponding problem before changing a rule.

The observer plugin identifies web sessions by the `navi` preset. The `Unitree Navi Agent` string in the persona's first sentence is a reserved marker for headless sessions. DSH currently does not expose session system prompts to plugins, so headless sessions cannot be identified this way. Observer push and model narration in the run log are therefore unavailable in headless mode. If you change the marker, also update `host/navi-observer/scope.ts`.

**Web mode**: DSH web sessions use the `standard` preset by default, including a coding persona and bash/edit tools. `start.sh` generates a `navi` preset using `navi-persona-<g1d|r1d>.txt`, configures the compaction chain and task goal tools, and sets the registry `default` to `navi`. Robot tools come from the host layer; the `navi` preset does not load coding tools. Existing sessions keep the preset from their creation, so **restart and create a new session** after changing the prompt.

### 9.2 Observer Push

`navi_mcp_server.py` provides read-only status endpoints `/state` and `/frame.jpg` at `127.0.0.1:3082`. The camera page also reads the task board `task` and recorded rooms `places` from `/state`. The host plugin `host/navi-observer/` polls once per second and sends messages during model reasoning as follows:

- After the sequence finishes and the base has settled for more than `NAVI_OBSERVER_IDLE_S` seconds (default 4), it sends "[观测] 已停稳…" (observation: settled…). Each stop produces at most one message. If the model's current frame was already captured after settling, no reminder is sent, avoiding repeated messages that cause the model to respond indefinitely. This applies, for example, when `submit_sequence` uses `observe_after_s` ≥ the motion duration.
- If the lidar reflex layer blocks a running sequence, it sends "[观测] 前方 x.xx m 被拦…" (observation: blocked x.xx m ahead…). If the same obstacle persists for more than 8 s, it interrupts reasoning so the model can reconsider. While a task goal is being auto-continued, it inserts a reminder without interrupting (see Section 9.3). Once the sequence stops, reflex-layer readings stop updating and no further blockage reports are sent.

The plugin only reads status and does not call robot tools. It applies only to Navi sessions and currently recognizes web sessions only (see Section 9.1). Set `NAVI_OBSERVER_PORT=0` to disable it or `NAVI_OBSERVER_IMAGES=0` to omit attached images. `NAVI_OBSERVER_INTERVAL_MS` adjusts the polling interval; `NAVI_OBSERVER_INTERRUPT_IDLE_S` sets how long to wait after settling before interrupting reasoning.

### 9.3 Task Goals and Auto-Continuation

Models sometimes end a reply partway through a task with "本轮未完成" (not completed in this round), for example stopping at a narrow passage and waiting for the user to send "continue". To reduce these interruptions, the `navi` preset includes DSH's task goal tools. The model calls `create_goal` at the start. If it ends a reply before finishing, the host's `goal-round-driver` sends `<goal_round>` to continue the task. Once finished, the model calls `update_goal(complete)`.

- The UI stop button or `/goal pause` pauses the goal and stops auto-continuation. When the user says 「停 / 撞了 / 先别动」 ("stop" / "collided" / "don't move yet"), the persona requires the model to call `stop` and then `update_goal(pause)`. **The UI stop button only stops model reasoning; it does not call the robot's `stop`.** A running sequence continues until its plan finishes or the sequence watchdog triggers (at least 8 s by default). Use the hardware emergency stop when the robot must stop immediately.
- Transient upstream failures (gateway 503 / 403, stream interruption) disable auto-continuation. The observer plugin attempts to re-enable it after 10 s / 30 s / 60 s, up to 3 consecutive attempts. Goals cancelled by the user are not re-enabled.
- `/goal` can be used at any time to view or control the current goal.

---

## 10. Integrating Other MCP Hosts

`navi_mcp_server.py` is a standard stdio MCP server and does not depend on DSH. Any MCP-capable host can mount it directly:

```bash
python src/navi_agent/navi_mcp_server.py            # real robot
python src/navi_agent/tools/stub_mcp_server.py      # offline stub
```

Example for Claude Desktop (`claude_desktop_config.json`):

```json
{
  "mcpServers": {
    "robot": {
      "command": "/path/to/.venv/bin/python",
      "args": ["/path/to/unitree_roboskills/unitree_gpt6/g1d_r1d_navigation/src/navi_agent/tools/stub_mcp_server.py"]
    }
  }
}
```

Notes:

- **stdout is the MCP protocol channel**; all logging goes to stderr. When extending the sidecar, do not `print` to stdout.
- Provide the content of `host/navi-persona-<g1d|r1d>.txt` (by robot model; written in Chinese) to the model as the system prompt (replace the `mcp__robot__` tool-name prefix with the naming actually used by the host).
- A host tool-call timeout of ≥ 60 s is recommended (`move`/`turn` are blocking; `observe_after_s` of `submit_sequence` is at most 20 s).
- The host must pass image blocks returned by tools to the model and limit the historical images included in each request. DSH uses `maxImagesPerRequest: 4` to keep those images from occupying too much context.
- The observer push plugin is DSH-specific. Other hosts work correctly without it; the model simply does not receive proactive notifications while reasoning.

---

## 11. Porting to Other Robots

Replacement points are concentrated in the `services/` layer; the upper tool surface and executors do not need to change:

| Component to replace | Location | Interface contract |
|---|---|---|
| Base driver | `RobotController` in `services/robots/g1d.py` (for R1D see `services/robots/r1d.py`) | `send_command(vx, vy, vyaw)`, `start()`/`stop()`, `get_pose()`, `connected` |
| Camera | `CameraStream` in `services/camera.py` | `start()`, `get_latest_frame() -> bytes(JPEG) \| None`, `connected`/`error` |
| Lidar | Scan subscription in `services/lidar.py` | Provide the latest `ranges` + angle parameters; the `clamp(vx)` logic can be reused |
| Map navigation | `SlamwareBackend` in `services/navigation.py` | Publish goal, cancel, localization callback |

Also adjust the speed limits in `config.py` and `navi.sequence.max_vx/max_vyaw`, as well as the descriptions in the persona related to the base's appearance (such as 「底盘白色弧形」, "the white arc of the base").

---

## 12. Frequently Asked Questions

**Q: In the web UI, the model behaves like a coding assistant and tools such as bash are visible.**

The `navi` preset did not take effect, and the session is still using `standard`. Start through `start.sh` and **create a new session**. Starting with `pnpm dsh` manually or reusing an old session may retain the original preset. Run `pnpm dsh --profile web --patch … --dump-config` to check that `agent-preset-registry.config.default` is `navi` and a `preset-navi` entry exists.

**Q: The model says it cannot see images.**

The selected model does not support images, or the model entry in the provider patch lacks `inputModalities: [text, image]`.

**Q: DashScope returns 400.**

`maxTokens` exceeds the endpoint limit; `navi-bailian.patch.yaml` already sets it to 8192.

**Q: The first `navigation_preflight` reports `pose_stale` / `lidar_not_ready`.**

Data may not be ready immediately after subscriptions are established. The server retries briefly; if it still fails, wait a few seconds and check again. If failures persist across several checks, investigate the connection or configuration.

**Q: `navigate_to` returns `running` but the base does not move / reports `stalled`.**

`stalled` usually means onboard obstacle avoidance has prevented movement, for example because a low object is ahead. Onboard goto may also fail to respond when the target is behind the robot. Enable `pre_align_deg: 60` to turn toward the target first.

**Q: The robot cannot be pushed by hand after stopping.**

Normally, the system stops sending commands and releases the servos once `cmd_watchdog_s + zero_release_s` (default 1.5 s) has elapsed since the last command, including the zero-velocity command sent by `stop`. If the robot still cannot be pushed, check whether another program is continuously sending `Move(0,0,0)`.

**Q: On R1D, `lidar.state` in `get_status` is not `healthy`, and forward motion is always blocked (`no_data`).**

Stop the server, then run `cd src/navi_agent && python -m tools.livox_diag`. It checks the lidar status broadcast, point-cloud destination address, and packet rate, and verifies the mounting extrinsics using a ground-plane fit. Common causes are:

- The point-cloud destination is not this host. The factory configuration may point elsewhere. The server changes it at startup, or you can change it manually with `--set-host <host IP>`.
- Another program is using port 56301. Port reuse may produce no error, but the later-bound program receives the point clouds. Stop the server before running `livox_diag`.
- The lidar was replaced or its mounting changed. Write the measured extrinsics printed by the tool into `robot.r1d` in `config.yaml`.

**Q: R1D has no image (`observe` reports that the camera is not connected).**

After the robot reboots, `stereo_patch_pc1` does not start by default, so port 5002 has no video stream. The Navi server stops `video_hub` and starts `stereo_patch_pc1` at startup, which takes about 10 seconds. If there is still no image, run `cd src/navi_agent && python -m tools.r1d_diag --net <interface>` to check both services (0 = running, 1 = stopped). Also confirm that GStreamer is installed on the host (see Section 3.1).

**Q: DDS cannot connect on a machine with multiple network interfaces.**

In `config.yaml`, set `net` explicitly to the name of the interface connected to the robot's internal network.

---

## 13. No-Motion Preview and Tests

### 13.1 No-Motion Preview (Repository Unified CLI)

`src/main.py` provides the same `preview` entry point as the other repository skills. It matches the input name exactly against `waypoints` in `config/default.yaml` and outputs a symbolic plan, **without connecting to the robot or calling a model**. It is independent of the real-robot runtime (`src/navi_agent/`). Run from the repository root:

```bash
python -m unitree_gpt6.g1d_r1d_navigation.src.main --instruction home
bash unitree_gpt6/g1d_r1d_navigation/scripts/preview.sh
```

### 13.2 Tests

Use the navigation environment with `navigation` and `dev` extras installed.

```bash
# Repository-wide checks (repository root)
for skill in g1d_r1d_navigation/common g1d_gomoku g1d_r1d_navigation; do
  python -m unittest discover -s "unitree_gpt6/$skill/tests" -p 'test_*.py'
done
python -m ruff check . && python -m ruff format --check .

# Switch from the repository root to the skill directory for the following checks
cd unitree_gpt6/g1d_r1d_navigation
# Runtime unit tests: no hardware or SDK required
pip install pytest
(cd src/navi_agent && python -m pytest -q)

# Pure-function unit tests for the host plugin (Node >= 22.6, no extra dependencies)
(cd host/navi-observer && node --experimental-strip-types --test *.test.ts)
```

Real-robot validation is outside the scope of automated testing. Before running on the real robot, confirm each item in [Section 3.6](#36-on-site-checklist) and the repository [safety requirements](../../docs/safety.md).

---

## 14. Acknowledgements, Attribution, and License

- **[DeepSeek-Harness](https://github.com/deepseek-ai/deepseek-harness)** (MIT): the host framework. This project extends it through patches and plugins without modifying its source code.
- **Unitree SDK2 Python** (`unitree_sdk2py`): the foundation for DDS communication and RPC clients. `services/g1_agv_client.py` is a Python reimplementation of the AgvClient interface from `g1/agv` in the C++ `unitree_sdk2`.
- **SLAMWARE ROS SDK**: `services/navigation.py` and `services/slamware_idl.py` define DDS types according to the topics and message formats published by the SLAMWARE ROS SDK (without introducing its runtime dependencies).

This skill is released with the repository under the [Apache-2.0](../../LICENSE) license. Third-party dependencies, including DeepSeek-Harness, unitree_sdk2_python, and the MCP Python SDK, retain their own licenses and are not distributed with this repository.

Do not commit site configuration (`src/navi_agent/config.yaml`, `navi.env`), maps and places, the topological memory store, camera frames, or run logs (`logs/`); these paths are already listed in the skill directory's `.gitignore`.
