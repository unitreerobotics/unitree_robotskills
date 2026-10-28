# -*- coding: utf-8 -*-
"""config.py -- configuration authority: defaults, YAML merge, runtime globals.

Responsibilities (config.py vs. config.yaml):
- config.py   = code authority: DEFAULT_CONFIG is the single set of defaults, plus
                parse/merge logic and runtime globals read by hot-path modules.
- config.yaml = site overrides: only keys that differ from defaults / per-platform
                differences. Without config.yaml the system runs fully on built-in
                defaults (navigation disabled). Template: config.example.yaml.

- DEFAULT_CONFIG   the authoritative defaults.
- load_config()    deep-merges DEFAULT_CONFIG with config.yaml into one effective config.
- apply_config()   writes hot-path motion/network parameters into module globals
                   (consumers must `import config` and read at runtime so defaults
                   are not frozen at import time).
"""

import copy
import sys
from pathlib import Path


def detect_net_iface() -> str:
    """Detect a usable network interface name; fall back to 'eth0' on failure.

    Explicit configuration takes precedence (handled by apply_config); this only detects.
    - Windows: try psutil (optional), else parse the first non-loopback adapter from `ipconfig`.
    - Linux/WSL/macOS: socket.if_nameindex(), skipping loopback and virtual interfaces;
      then fall back to /sys/class/net.
    """
    try:
        if sys.platform == "win32":
            try:
                import psutil  # optional
                for name, addrs in psutil.net_if_addrs().items():
                    if name.lower().startswith(("lo", "loopback")):
                        continue
                    if any(a.family == 2 for a in addrs):  # AF_INET
                        return name
            except ImportError:
                pass
            # No psutil: parse ipconfig (works on Chinese and English Windows)
            import re
            import subprocess
            out = subprocess.run(["ipconfig"], capture_output=True,
                                 text=True, timeout=5).stdout
            for line in out.splitlines():
                m = re.match(r"\s*(?:以太网适配器|Ethernet adapter)\s*([^:]+):", line)
                if m:
                    return m.group(1).strip()
            return "eth0"

        # Linux / WSL / macOS
        import socket
        if hasattr(socket, "if_nameindex"):
            _skip = {"lo", "lo0"}
            for _idx, name in socket.if_nameindex():
                if name in _skip or name.startswith(("veth", "docker", "br-", "virbr")):
                    continue
                return name
        sysdir = Path("/sys/class/net")
        if sysdir.is_dir():
            for child in sorted(sysdir.iterdir()):
                if child.name.startswith(("lo", "veth", "docker", "br-", "virbr")):
                    continue
                return child.name
        return "eth0"
    except Exception:
        return "eth0"


DEFAULT_CONFIG: dict = {
    # DDS network interface (auto = detect at startup).
    "net": "auto",
    # Robot model (services/robots): auto = one read-only probe per startup (no hot swap;
    # changing robots requires restarting the server). Motion is enabled only when exactly
    # one candidate matches; zero or multiple matches refuse all motion. Explicit g1d / r1d
    # skips probing. Candidates sharing (net, domain) share one probe subprocess and are
    # told apart by topic signatures.
    # robot.g1d / robot.r1d may override profile fields (max_vx, max_vyaw, has_lidar,
    # lidar_topic, net, domain_id, ...) and R1D calibration (mps_full, rps_full, turn_sign,
    # feedback_timeout_s).
    "robot": {
        "model": "auto",
        "probe_timeout_s": 2.5,
        "candidates": [
            {"model": "g1d", "net": "auto", "domain_id": 0},
            {"model": "r1d", "net": "auto", "domain_id": 0},
        ],
    },
    "motion": {
        "rate": 20,                # command publish rate (Hz); G1D Move is an RPC, 20 Hz
        "move_distance": 0.5,      # default step distance (m)
        "move_speed": 0.3,         # default linear speed (m/s); conservative for real hardware (G1D vx max 1.5)
        "turn_angle": 0.5,         # default turn rate (rad/s); G1D vyaw max 0.6, keep margin
        "ramp_time": 0.5,          # speed ramp time (s) for smooth acceleration
        "stop_hold_time": 0.5,     # zero-speed hold after stopping (s)
        "cmd_watchdog_s": 0.5,     # publish zero speed if send_command not refreshed within this (s)
        "zero_release_s": 1.0,     # after streaming zero this long, stop sending Move to release
                                   # the chassis servos (otherwise the robot cannot be pushed by hand)
    },
    # Camera (real-robot ZMQ stream)
    "camera": {
        "host": "192.168.123.164",     # robot IP running image_server (G1D internal default)
        "request_port": 60000,         # REQ b"GET_DATA" port for fetching camera config
        "camera": "head_camera",       # head_camera / left_wrist_camera / right_wrist_camera
        "eye": "left",                 # stereo half: left / right / full
        "zmq_port": None,              # fixed subscribe port skips negotiation; None = use GET_DATA
        "binocular": None,             # force stereo on/off; None = use server config
        "request_timeout_ms": 2000,
        "first_frame_timeout_s": 5.0,
        "max_frame_age_s": 1.0,        # observe rejects cached frames older than this (local receive time)
    },
    # Navi Agent tool layer (navi_mcp_server.py)
    "navi": {
        "navigation": {
            "enabled": False,  # off by default for safety; enable in config.yaml after on-site testing
            "engine": "onboard",  # onboard = SLAMWARE autonomous goto (only engine in this version)
            "source_offset_s": 0.0,  # onboard source timestamp + offset = host wall clock
            # Onboard SLAM relaxations. Off by default; enable one at a time once goto is
            # confirmed to drive the chassis.
            "goal_frame": "",  # "" = use pose frame; "map" decouples goal header from rt/robot_pose
            "odom_required": True,  # False = detect arrival by pose stillness when no odom twist
            "receipt_time_freshness": False,  # judge pose freshness by receive time (SLAM clock not synced to host)
            "map_id": "",     # existing map id; places are bound to it and rejected after a map change
            "mapping": {"mode": "existing",   # only existing onboard maps are supported
                        "backend": "disabled",  # slamware_ros = subscribe onboard map/state as a health gate
                        "min_quality": 20,
                        "trust_slam_state": False,  # True = slam_state=connected overrides the numeric quality gate
                        "require_map_health": True},  # False = map/boundary/quality advisory only; SLAMWARE plans
            "frame": "map",   # must equal rt/robot_pose.header.frame_id
            "places_file": "data/places.json",  # relative to src/navi_agent/, not cwd
            "max_age_s": 1.0,
            "state_max_age_s": 3.0,  # slamware state topic is ~1 Hz; sharing pose's 1.0 s goes stale periodically
            "max_distance_m": 15.0,
            "timeout_s": 120.0,
            "position_tolerance_m": 0.15,
            "yaw_tolerance_deg": 10.0,
            "settle_s": 1.0,
            "stall_s": 20.0,
            "pre_align_deg": 0.0,  # turn in place first if goal bearing exceeds this (0 = off);
                                   # onboard goto was observed not to move for goals behind the robot
            "pre_align_turn_speed": 0.35,  # pre-align turn rate rad/s (conservative)
        },
        "guard": {
            "max_move_distance_m": 5.0,     # hard cap per translation (m)
            "max_turn_angle_deg": 360.0,    # hard cap per turn (deg)
        },
        # Sequence mode (submit_sequence): each inference emits a velocity sequence for the
        # near future; the lower layer executes and merges them continuously.
        "sequence": {
            "max_vx": 1.0,             # linear speed cap m/s (min with model max_vx: G1D 0.5, R1D 1.0)
            "max_vyaw": 0.5,           # angular speed cap rad/s (G1D vyaw hardware max 0.6)
            "max_accel_mps2": 0.0,     # linear accel cap; 0 = model max_accel_mps2 (R1D 0.6), else max_vx / ramp_time
            "max_duration_s": 45.0,    # hard cap on total sequence duration s (must exceed inference latency)
            "max_distance_m": 5.0,     # hard cap on total sequence distance m (None = guard.max_move_distance_m)
            "blend_s": 1.0,            # blend window for overlapping same-direction sequences (s)
            "watchdog_s": 8.0,         # upper layer silent this long while a sequence runs -> ramp stop
            "horizon_factor": 1.7,     # suggested_horizon = inference latency EMA x this factor
            "horizon_min_s": 3.0,      # lower bound on suggested horizon
            "max_blind_m": 5.0,        # max blind travel per decision: suggested horizon <= max_blind_m / planned speed.
                                       # With slow inference (20-30 s per step) do not aim for non-stop motion;
                                       # drive one full segment, stop, and look. Forward motion is still bounded
                                       # by the lidar speed envelope; 3 m segments were too choppy in corridors.
            "max_observe_after_s": 20.0,  # cap on optional delayed frame capture after submit (s); must be < MCP toolCallTimeoutMs
        },
        # Lidar reflex layer (services/lidar.py): collision speed clamp that bypasses the LLM;
        # applies to sequence execution and discrete move.
        "lidar": {
            "enabled": True,
            "stop_m": 0.45,            # forward clearance < this -> forward component zeroed
            "slow_m": 1.2,             # linear slowdown between stop_m and slow_m (unused when headway_s > 0)
            "headway_s": 0.0,          # > 0 = speed envelope: allowed = max(creep_mps, (clearance - stop_m) / headway_s);
            "creep_mps": 0.0,          #   both 0 = model default (R1D 1.5 s / 0.2 m/s; G1D none -> linear zone)
            "max_age_s": 1.0,          # scans older than this count as no data
            "require": False,          # True = no lidar data blocks forward motion (recommended on hardware);
                                       # R1D requires it by default (robot.r1d.lidar_require, forward only:
                                       # its lidar is blind to the rear)
            "yaw_offset_deg": 0.0,     # lidar mounting yaw offset
            "corridor_half_width_m": 0.0,  # > 0 enables travel corridor: only returns with body-frame |y| <= this count as obstacles
        },
        # Observer status endpoint (HTTP embedded in navi_mcp_server, polled by the navi-observer host plugin)
        "observer": {
            "port": 3082,
        },
        # Topological memory (services/topo_memory): stored at <dir>/<model>/<venue>/; dir relative to src/navi_agent/
        "memory": {
            "dir": "data/memory",
            "venue": "default",       # venue name; change manually per site so memories never mix
        },
    },
}


# Runtime globals: defaults from DEFAULT_CONFIG, overwritten by apply_config at startup.
# Consumers must `import config` and read at runtime (`from config import X` would freeze them).
NET = DEFAULT_CONFIG["net"]
RATE = DEFAULT_CONFIG["motion"]["rate"]            # Hz
MOVE_DISTANCE = DEFAULT_CONFIG["motion"]["move_distance"]
MOVE_SPEED = DEFAULT_CONFIG["motion"]["move_speed"]
TURN_ANGLE = DEFAULT_CONFIG["motion"]["turn_angle"]
RAMP_TIME = DEFAULT_CONFIG["motion"]["ramp_time"]
STOP_HOLD_TIME = DEFAULT_CONFIG["motion"]["stop_hold_time"]
CMD_WATCHDOG_S = DEFAULT_CONFIG["motion"]["cmd_watchdog_s"]
ZERO_RELEASE_S = DEFAULT_CONFIG["motion"]["zero_release_s"]

CONFIG_FILE = "config.yaml"
_missing_warned = False


def _deep_merge(base: dict, override: dict | None) -> dict:
    """Recursively merge: override wins; nested dicts merge, lists/scalars are replaced whole."""
    if not override:
        return base
    out = dict(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config() -> dict:
    """Load config.yaml merged over built-in defaults and return the effective config.

    Falls back to defaults if config.yaml is missing or unparsable. Returns a fresh dict
    the caller may mutate. Messages go to stderr: stdout is the MCP stdio channel."""
    global _missing_warned
    cfg_path = Path(__file__).resolve().parent / CONFIG_FILE
    if not cfg_path.is_file():
        if not _missing_warned:
            print(f"[警告] 未找到 {CONFIG_FILE}，使用内置默认配置（导航关闭）", file=sys.stderr)
            _missing_warned = True
        return copy.deepcopy(DEFAULT_CONFIG)
    try:
        import yaml
    except ImportError:
        print(f"[警告] 未安装 pyyaml，无法读取 {CONFIG_FILE}：pip install pyyaml", file=sys.stderr)
        return copy.deepcopy(DEFAULT_CONFIG)
    try:
        yaml_cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
    except Exception as e:
        print(f"[警告] 解析 {cfg_path.name} 失败: {e}", file=sys.stderr)
        return copy.deepcopy(DEFAULT_CONFIG)
    return _deep_merge(copy.deepcopy(DEFAULT_CONFIG), yaml_cfg)


# Runtime global name <-> DEFAULT_CONFIG["motion"] key, used by apply_config
_MOTION_GLOBALS = (
    ("RATE", "rate"),
    ("MOVE_DISTANCE", "move_distance"),
    ("MOVE_SPEED", "move_speed"),
    ("TURN_ANGLE", "turn_angle"),
    ("RAMP_TIME", "ramp_time"),
    ("STOP_HOLD_TIME", "stop_hold_time"),
    ("CMD_WATCHDOG_S", "cmd_watchdog_s"),
    ("ZERO_RELEASE_S", "zero_release_s"),
)


def apply_config(cfg: dict):
    """Write the merged config into module runtime globals; unset keys keep defaults."""
    global NET

    net_cfg = cfg.get("net", DEFAULT_CONFIG["net"])
    if not net_cfg or str(net_cfg).strip().lower() in ("auto", "none"):
        NET = detect_net_iface()
        print(f"[信息] 网卡未指定或为 auto，自动检测为 {NET}（可在 config.yaml net 覆盖）",
              file=sys.stderr)
    else:
        NET = str(net_cfg).strip()

    # m always contains every default key thanks to _deep_merge
    m = _deep_merge(DEFAULT_CONFIG["motion"], cfg.get("motion") or {})
    for _name, _key in _MOTION_GLOBALS:
        globals()[_name] = m[_key]


def load_camera_defaults(cfg: dict) -> dict:
    """Return camera settings from the merged config, filled with defaults."""
    return dict(_deep_merge(DEFAULT_CONFIG["camera"], cfg.get("camera") or {}))


def navi_section(cfg: dict, name: str) -> dict:
    """Return section navi.<name> deep-merged with defaults (all keys present)."""
    return _deep_merge(copy.deepcopy(DEFAULT_CONFIG["navi"][name]),
                       ((cfg.get("navi") or {}).get(name)) or {})
