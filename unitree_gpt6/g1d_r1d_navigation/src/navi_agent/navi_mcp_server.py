# -*- coding: utf-8 -*-
"""MCP tool surface of the Unitree Navi Agent (stdio, FastMCP).

Any MCP host (DeepSeek-Harness / Claude Desktop / Cursor / custom agent) launches this script
over stdio to expose a Unitree wheeled base (G1D / R1D, model auto-detected at startup) to an
LLM as a set of guarded tools.

Three mutually exclusive motion modes (_motion_entry_lock ensures one entry point owns the base):
  - Discrete, blocking: move / turn return when done, with a camera frame taken after settling.
  - Continuous sequence: submit_sequence returns immediately (with a fresh frame) after queuing
    a velocity plan; core/sequence.SequenceExecutor runs it and merges it with the previous plan
    aligned on frame time. The caller keeps planning from returned frames, so motion overlaps
    with inference instead of stopping for every decision ("think while walking").
  - Map navigation: save_place on site, then navigate_to; onboard SLAMWARE plans and executes.
    Arrival is confirmed by map pose plus a stationary check. Disabled by default; must be
    enabled explicitly in config.yaml after on-site acceptance.

Safety layers (outer to inner):
  1. Guards (robot_tools.guard_*): invalid args return readable errors; hard caps on
     distance/angle/speed/duration.
  2. Lidar reflex (services/lidar.py): bypasses the LLM and clamps forward velocity every tick
     (zero when too close, slow in the transition band; vyaw is never clamped). Enabled via
     navi.lidar; NAVI_NO_LIDAR=1 disables the subscription (offline use).
     Source per model: G1D subscribes to SLAMWARE LaserScan; R1D receives UDP point clouds from
     the front Livox Mid-360 and converts them to a virtual scan for the same guard
     (services/livox_mid360.py). Models without a usable ranging layer fall back to a liveness
     gate: translation only while the camera frame is fresh.
  3. Executor watchdogs: sequences ramp to stop on silence timeout; the command publisher sends
     zero velocity after 0.5 s without commands, then stops sending Move after 1 s of zeros to
     release the servos (services/robots/).
  4. Map navigation gates: localization/state freshness, map health, lidar readiness,
     stationary check, stall/timeout.

Observer surface: embedded HTTP (default 127.0.0.1:3082; navi.observer.port <= 0 disables) that
the host plugin host/navi-observer polls while the model is thinking, to push observation
messages.

Environment: NAVI_NO_CAMERA / NAVI_CAMERA_HOST / NAVI_CAMERA_REQUEST_PORT (see
driver_factory.py), NAVI_NO_LIDAR=1.
"""
import inspect
import math
import os
import sys
import threading
import time
from collections import OrderedDict
from datetime import datetime, timezone
from functools import wraps
from pathlib import Path

from mcp.server.fastmcp import FastMCP, Image

import config
import driver_factory
import robot_tools
from config import load_config
from core.sequence import SequenceExecutor
from driver_factory import get_driver, tool_odometry, tool_status, tool_stop
from services.lidar import LidarGuard
from services.runlog import RunLog

_SRC = Path(__file__).resolve().parent

mcp = FastMCP("unitree-navi", instructions=(
    "Unitree 轮式底盘导航工具。启动时自动识别机型（G1D/R1D），先 get_status 看 robot.model 与 capabilities；"
    "本机型无 slamware 时地图/导航工具返回 unsupported，改用 observe + move/turn/submit_sequence。"
    "observe 返回当前相机帧（双目默认左眼）。"
    "move/turn 为同步阻塞执行，完成后结果自带一帧停稳后的相机图像（不必再单独 observe）；"
    "submit_sequence 提交未来一段速度序列后立即返回并附新帧，机器人在后台连续执行，"
    "拿新帧继续规划下一段即可。单次距离/角度受硬顶保护；get_odometry 仅为命令积分估计，不是实测。"
    "已有机载地图时：先 navigation_preflight，再 save_place 实地标定、navigate_to 导航；"
    "地图导航与速度模式互斥。查询 navigation_status，只有 state=arrived 且 active=false 才能报告到达；"
    "拓扑记忆 memory_*：按机型/场地分库记地点、地标、前沿和走过的边；memory_context 只给候选与路线提示，"
    "须自己比图后 memory_localize 确认，证据帧只用 observe/运动结果返回的 frame_id。"
    "stop 后也须确认停稳。"))


_config_cache: dict | None = None


def _config() -> dict:
    """Load config.yaml once and cache it: editing the file at runtime does not change safety
    parameters (speed limits, lidar, ...); restart the service to apply changes.

    The reflex layer reads config every tick (20 ms), so caching also avoids re-parsing YAML."""
    global _config_cache
    if _config_cache is None:
        _config_cache = load_config()
    return _config_cache


def _section(name: str) -> dict:
    return config.navi_section(_config(), name)     # deep copy: callers may mutate it safely


def _guard_limits() -> dict:
    g = _section("guard")
    return {"max_move_distance_m": g["max_move_distance_m"],
            "max_turn_angle_deg": g["max_turn_angle_deg"]}


# ---------------------------------------------------------------------------
# Robot profile (services/robots): speed limits = min(config, profile); capability flags
# decide whether lidar/SLAMWARE tools are available.
# ---------------------------------------------------------------------------
def _driver_profile(d=None):
    """Profile of an existing driver. d=None inspects the current singleton and never connects.
    Test doubles have no profile -> None (legacy G1D behaviour)."""
    if d is None:
        d = driver_factory.current_driver()
    return getattr(d, "profile", None) if d is not None else None


def _slamware_error(d=None):
    p = _driver_profile(d)
    if p is not None and not p.has_slamware:
        return f"unsupported_on_{p.model}: 本机型无机载 SLAMWARE 地图/导航，请用 observe + move/turn/submit_sequence"
    return None


def _require_slamware(d):
    """Model gate before initialising SLAMWARE paths (call after get_driver, once the model is
    identified)."""
    error = _slamware_error(d)
    if error:
        raise ValueError(error)
    return d


def _motion_error(d):
    """Model gate for motion tools: refuse if the model is unknown, or if the R1D feedback guard
    has latched a fault (stop clears it)."""
    p = _driver_profile(d)
    if p is not None and not p.can_move:
        return f"robot_model_unknown: {p.reason}；确认只连了一台机器人并重启服务"
    health = getattr(getattr(d, "robot", None), "health", None)
    fault = health().get("fault") if callable(health) else None
    return f"robot_fault: {fault}；先 stop，待反馈恢复后重试" if fault else None


def _profile_limits(cfg: dict, d=None) -> dict:
    """Sequence speed limits = min(navi.sequence, robot profile)."""
    out = dict(cfg)
    p = _driver_profile(d)
    if p is not None:
        out["max_vx"] = min(float(cfg["max_vx"]), float(p.max_vx))
        out["max_vyaw"] = min(float(cfg["max_vyaw"]), float(p.max_vyaw))
    return out


# ---------------------------------------------------------------------------
# Lidar reflex singleton (collision speed clamp outside the LLM; applies to sequences and move)
# ---------------------------------------------------------------------------
_lidar = None
_lidar_feeder = None                   # Livox point-cloud receiver (lidar_kind=livox_mid360 only)
_slam_runner = None                    # R1D lidar SLAM (lidar_slam.SlamRunner), created with feeder
_imu = None                            # R1D lidar built-in IMU (LivoxImu), feeds heading hold
_lidar_lock = threading.Lock()
_lidar_error = None
_lidar_checked = False                 # whether enabled/disabled has been decided
_lidar_next_try = 0.0                  # retry time after a failed subscribe (rate-limited)
_LIDAR_RETRY_S = 5.0
_LIDAR_OFF = {"reason": "disabled", "clear_m": None, "blocked_s": 0.0}
# Cold-start warm-up: right after DDS subscriptions (pose/state/scan) are created and before the
# first sample arrives, a read-only preflight would misreport its own warm-up as staleness/
# readiness failures. Preflight polls within a bounded window until fresh (read-only, no motion).
# The window must exceed the state topic period (~1 Hz measured) or the first state may miss it.
_PREFLIGHT_WARMUP_S = 2.5
_PREFLIGHT_WARMUP_POLL_S = 0.15
_PREFLIGHT_WARMUP_KEYS = ('slam_state_stale', 'pose_stale', 'odom_stale', 'lidar_not_ready')


def _lidar_disabled_by_env() -> bool:
    return bool(os.environ.get("NAVI_NO_LIDAR"))


def _lidar_require(cfg: dict, profile=None) -> bool:
    """Whether forward motion is blocked without lidar data: navi.lidar.require, or the profile's
    own requirement (R1D lidar_require)."""
    return bool(cfg["require"] or getattr(profile, "extra", {}).get("lidar_require"))


def _livox_mount(x: dict):
    """Build the Livox Mount from profile extra calibration (shared extrinsics for the collision
    guard and lidar SLAM)."""
    from services import livox_mid360 as livox
    return livox.Mount(x["lidar_down"], x["lidar_height_m"], x["lidar_yaw_deg"], x["lidar_front_offset_m"])


def _build_livox_guard(cfg: dict, profile, log, batch_sink=None):
    """R1D: Mid-360 point cloud -> virtual scan -> LidarGuard. Returns (guard, feeder); the
    feeder is not started yet.
    batch_sink receives raw packet batches for lidar SLAM (see LivoxFeeder). NAVI_LIDAR_RECORD=1
    records raw packets to <repo>/logs/lidar-<time>.bin.gz (for offline tuning:
    python -m tools.lidar_slam_replay)."""
    from services import livox_mid360 as livox  # lazy import: only Livox models need it
    x = profile.extra
    # Limit forward speed by the travel corridor only (sector_guard=False): in hallways the
    # Mid-360 virtual scan's +/-30 deg sector would hit the side walls.
    guard = LidarGuard(stop_m=cfg["stop_m"], slow_m=cfg["slow_m"], max_age_s=cfg["max_age_s"],
                       require=_lidar_require(cfg, profile), yaw_offset_deg=cfg["yaw_offset_deg"],
                       corridor_half_width_m=(cfg.get("corridor_half_width_m")
                                              or x.get("lidar_corridor_half_width_m", 0.0)),
                       rear_blind=True, log=log, sector_guard=False,
                       headway_s=cfg.get("headway_s") or x.get("lidar_headway_s", 0.0),
                       creep_mps=cfg.get("creep_mps") or x.get("lidar_creep_mps", 0.0))
    mount = _livox_mount(x)
    record = None
    if os.environ.get("NAVI_LIDAR_RECORD"):
        logs = _SRC.parent.parent / "logs"     # <skill root>/logs (_SRC = src/navi_agent)
        logs.mkdir(parents=True, exist_ok=True)
        record = logs / time.strftime("lidar-%Y%m%d-%H%M%S.bin.gz")
    feeder = livox.LivoxFeeder(
        guard.feed, mount, lidar_ip=x["lidar_ip"], host_ip=x["lidar_host_ip"],
        auto_host=x["lidar_auto_host"], log=log, batch_sink=batch_sink, record_path=record,
        scan_kw={"z_min_m": x["lidar_z_min_m"], "z_max_m": x["lidar_z_max_m"],
                 "self_box": tuple(x["lidar_self_box"])})
    return guard, feeder


def _start_heading_hold(profile, mount, log):
    """R1D: start the lidar's built-in IMU and attach it to the base heading-hold loop
    (disable with robot.r1d.heading_hold=false). Failure is only logged: without a gyro the
    loop runs open-loop, and collision protection is unaffected."""
    global _imu
    x = profile.extra
    robot = getattr(driver_factory.current_driver(), "robot", None)
    if not x.get("heading_hold") or not hasattr(robot, "attach_yaw_rate"):
        return
    from services import livox_mid360 as livox
    if _imu is None:
        imu = livox.LivoxImu(mount.rot[2], port=x.get("imu_port", livox.HOST_IMU_PORT), log=log)
        if not imu.start():
            return
        _imu = imu
    robot.attach_yaw_rate(_imu.yaw_rate, log=log, kp=x.get("heading_kp", 1.5),
                          max_corr=x.get("heading_max_corr_rps", 0.25))


def _get_lidar_guard():
    """Lazily create the lidar guard; returns None if enabled=False or NAVI_NO_LIDAR=1.

    Subscribing requires DDS to be initialised first (ChannelFactoryInitialize in
    services/robots/), so failures are NOT permanent: retry every _LIDAR_RETRY_S until the
    driver is connected. Subscribe failures don't raise: with require=False motion is allowed,
    but lidar.error shows up in get_status."""
    global _lidar, _lidar_feeder, _slam_runner, _lidar_checked, _lidar_next_try, _lidar_error
    with _lidar_lock:
        if _lidar is not None:
            return _lidar
        now = time.monotonic()
        if _lidar_checked and now < _lidar_next_try:
            return None
        cfg = _section("lidar")
        if not cfg.get("enabled") or _lidar_disabled_by_env():
            _lidar_checked, _lidar_next_try = True, float("inf")   # explicitly off: no retry
            return None
        profile = _driver_profile()
        if profile is not None and not profile.lidar_ready:
            # No lidar / not validated on this model: don't subscribe; _reflex_limit falls back
            # to the camera-freshness gate.
            _lidar_checked, _lidar_next_try = True, float("inf")
            return None
        if getattr(profile, "lidar_kind", "dds_scan") == "livox_mid360":
            log = getattr(driver_factory.current_driver(), "_log", None)
            runner = None
            if getattr(profile, "localization", None) == "lidar_slam":
                try:
                    runner = _build_slam(profile, log)
                except Exception as exc:  # SLAM failure must not block the guard
                    _slam_error[0] = f"雷达定位启动失败: {type(exc).__name__}: {exc}"
            try:
                g, feeder = _build_livox_guard(cfg, profile, log, batch_sink=runner.submit if runner else None)
                started = feeder.start()
                error = None if started else (feeder.error or "Livox 点云接收启动失败")
            except Exception as exc:  # bad calibration/deps: retry on interval, not every tick
                started, error = False, f"Livox 守卫创建失败: {type(exc).__name__}: {exc}"
            if not started:
                if runner is not None:
                    runner.stop(save=False)
                _lidar_error = error
                _lidar_checked, _lidar_next_try = True, now + _LIDAR_RETRY_S
                return None
            _lidar, _lidar_feeder, _lidar_error, _slam_runner = g, feeder, None, runner
            _start_heading_hold(profile, feeder.mount, log)
            return g
        g = LidarGuard(stop_m=cfg["stop_m"], slow_m=cfg["slow_m"],
                       max_age_s=cfg["max_age_s"], require=_lidar_require(cfg, profile),
                       yaw_offset_deg=cfg["yaw_offset_deg"],
                       corridor_half_width_m=cfg.get("corridor_half_width_m", 0.0),
                       headway_s=cfg.get("headway_s", 0.0), creep_mps=cfg.get("creep_mps", 0.0),
                       **({"topic": profile.lidar_topic} if profile is not None else {}))
        if not g.start():
            _lidar_error = g.error or "雷达订阅失败"
            _lidar_checked, _lidar_next_try = True, now + _LIDAR_RETRY_S
            return None
        _lidar = g
        _lidar_error = None
        return g


# ---------------------------------------------------------------------------
# Lidar localization and places (R1D: 2D SLAM from Livox point clouds, services/lidar_slam.py).
# Map and places live in src/<slam_dir>/<navi.memory.venue>/{map.npz, places.json}; created
# together with the lidar guard.
# ---------------------------------------------------------------------------
_slam_error = [None]
_SLAM_MAX_AGE_S = 1.0


def _uses_lidar_slam(p=None) -> bool:
    p = _driver_profile() if p is None else p
    return p is not None and getattr(p, "localization", None) == "lidar_slam"


def _venue(cfg: dict) -> str:
    """navi.memory.venue: venue name used as a directory name; must be non-empty and contain no
    path separators."""
    venue = str(cfg.get("venue") or "").strip()
    if not venue or venue in (".", "..") or any(c in venue for c in "/\\"):
        raise ValueError(f"navi.memory.venue 无效：{venue!r}（填场地名，不能含路径分隔符）")
    return venue


def _slam_dir(profile) -> Path:
    venue = _venue(_section("memory"))
    root = Path(profile.extra.get("slam_dir") or "data/lidar_map")
    return (root if root.is_absolute() else _SRC / root) / venue


def _archive(path: Path, why: str, log) -> None:
    if path.exists():
        dst = path.with_name(f"{path.stem}.{time.strftime('%Y%m%d-%H%M%S')}.{why}{path.suffix}")
        path.rename(dst)
        if log:
            log(f"旧文件已改名保留：{dst.name}", "slam")


def _build_slam(profile, log):
    """Create lidar SLAM: load an existing map (enters relocalization) or start a new one.
    NAVI_SLAM_NEW_MAP=1 archives the old map and places by renaming, then starts fresh."""
    from services import lidar_slam
    x = profile.extra
    folder = _slam_dir(profile)
    map_path, places_path = folder / "map.npz", folder / "places.json"
    if os.environ.get("NAVI_SLAM_NEW_MAP"):
        _archive(map_path, "replaced", log)
        _archive(places_path, "replaced", log)
    slam = None
    if map_path.exists():
        try:
            slam = lidar_slam.LidarSlam.load(map_path, log=log)
            if log:
                log(f"载入雷达地图 {map_path}（{len(slam.keyposes)} 个轨迹位姿），等待重定位", "slam")
        except Exception as exc:                  # corrupt: archive and start anew, never overwrite
            if log:
                log(f"雷达地图读不出来（{type(exc).__name__}: {exc}），新建地图", "slam")
            _archive(map_path, "broken", log)
            _archive(places_path, "orphaned", log)
    if slam is None:
        slam = lidar_slam.LidarSlam(log=log)
        if places_path.exists():                  # places are meaningless without their map
            _archive(places_path, "orphaned", log)

    def seeds():
        try:
            return [(p["x"], p["y"], p["yaw"]) for p in _slam_place_store(slam).list()]
        except (ValueError, OSError):
            return []

    mount = _livox_mount(x)
    runner = lidar_slam.SlamRunner(slam, mount, map_path=map_path, seeds_fn=seeds, log=log,
                                   struct_kw={"self_box": tuple(x["lidar_self_box"])})
    runner.start()
    _slam_error[0] = None
    return runner


def _slam_place_store(slam=None):
    from core.places import PlaceStore
    from services.lidar_slam import FRAME
    slam = slam or _slam_runner.slam
    return PlaceStore(_slam_dir(_driver_profile()) / "places.json", slam.map_id, FRAME)


def _metric_place_store():
    """Named-place store for the current model: R1D uses the lidar map, others the navigation
    config (SLAMWARE)."""
    if _uses_lidar_slam():
        if _slam_runner is None:
            raise ValueError("雷达定位还没启动（先 get_status 或 get_localization）")
        return _slam_place_store()
    return _place_store()


_SLAM_HINTS = {
    "slam_relocalizing": "载入了上次的地图、还没在上面找到自己：把机器人开到之前走过的地方（例如某个已标定点附近）"
                         "停两秒，会自动重新定位；在没去过的地方定不了位",
    "slam_lost": "雷达匹配连续失败（被抬起/推走、或周围空旷无结构）：停下，开回之前走过的地方会自动重新定位",
    "slam_degraded": "这一帧没匹配上，暂用推算位姿；持续出现就停下看看周围是不是太空旷",
    "slam_initializing": "雷达定位刚启动，稍等一秒再读",
    "slam_stale": "雷达定位没有新数据（点云断流？），先 get_status 看 lidar 状态",
}


def _slam_localization(wait_s: float = 1.0) -> dict:
    """get_localization for R1D. The first call creates the lidar guard and SLAM, and waits for
    the first scan to be processed."""
    _get_lidar_guard()
    r = _slam_runner
    if r is None:
        return {"ok": False, "source": "lidar_slam",
                "error": _slam_error[0] or _lidar_error or "雷达定位未启动（雷达守卫没起来，见 get_status 的 lidar）"}
    deadline = time.monotonic() + wait_s
    while r.slam.scans == 0 and time.monotonic() < deadline:
        time.sleep(0.05)
    snap = r.slam.snapshot(_SLAM_MAX_AGE_S)
    snap["source"] = "lidar_slam"
    hints = [_SLAM_HINTS[e] for e in snap["errors"] if e in _SLAM_HINTS]
    if hints:
        snap["hint"] = hints[0]
    if r.last_relocalize is not None and snap["state"] in ("relocalizing", "tracking"):
        snap["last_relocalize"] = r.last_relocalize
    return snap


def _place_geometry(pose: dict, place: dict) -> dict:
    """Distance and bearing of a place relative to the robot: positive bearing_deg = to the left;
    heading_diff_deg = taught heading - current heading."""
    dx, dy = place["x"] - pose["x"], place["y"] - pose["y"]
    bearing = math.degrees(math.atan2(dy, dx) - pose["yaw"])
    turn = math.degrees(place["yaw"] - pose["yaw"])
    norm = lambda a: round((a + 180.0) % 360.0 - 180.0, 1)        # noqa: E731
    return {"distance_m": round(math.hypot(dx, dy), 2), "bearing_deg": norm(bearing), "heading_diff_deg": norm(turn)}


def _slam_list_places() -> dict:
    loc = _slam_localization()
    if _slam_runner is None:
        return {"ok": False, "error": loc.get("error")}
    store = _slam_place_store()
    places = store.list()
    if loc.get("pose") is not None and loc["state"] in ("tracking", "degraded"):
        places = [{**p, **_place_geometry(loc["pose"], p)} for p in places]
    out = {"ok": True, "map_id": store.map_id, "frame": store.frame, "places": places,
           "localization": {k: loc.get(k) for k in ("ok", "state", "pose", "errors")}}
    if places and not loc.get("ok"):
        out["note"] = "当前没有可信定位，点没有附相对距离/方位：" + loc.get("hint", "")
    return out


def _slam_save_place(name: str, overwrite: bool) -> dict:
    if not _motion_entry_lock.acquire(blocking=False):
        return {"ok": False, "error": "motion_busy"}
    try:
        loc = _slam_localization()
        if not loc.get("ok"):
            return {"ok": False, "error": "雷达定位不可用，不能标定：" + ",".join(loc.get("errors") or [loc.get("error", "")]),
                    "localization": loc}
        d = driver_factory.current_driver()
        moving = (_seq_exec is not None and _seq_exec.status()["active"]) or \
            (d is not None and d.motion is not None and d.motion.is_moving())
        if moving:
            return {"ok": False, "error": "底盘还在动：先停稳（stop 或等序列走完）再标定"}
        if not _slam_runner.save():
            return {"ok": False, "error": _slam_runner.error or "雷达地图保存失败，未标定"}
        place = _slam_place_store().save(name, loc["pose"], overwrite=overwrite, source="lidar_slam")
        return {"ok": True, "place": place, "quality": loc.get("quality")}
    except (ValueError, OSError) as exc:
        return {"ok": False, "error": str(exc)}
    finally:
        _motion_entry_lock.release()


def _blind_translation_block(vx: float):
    """Liveness gate when no ranging layer is available: translation requires a fresh camera
    frame, otherwise it is clamped to zero. Applies to models without lidar, and to models
    declaring camera_gate_without_lidar while the lidar guard is unavailable (R1D lidar off or
    not up yet). Returns a block-reason dict, or None to allow. Uses the existing driver only;
    never connects."""
    if vx == 0.0:
        return None
    d = driver_factory.current_driver()
    p = _driver_profile(d)
    if p is None or not p.can_move:
        return None
    if p.lidar_ready and not getattr(p, "extra", {}).get("camera_gate_without_lidar"):
        return None
    cam = getattr(d, "camera", None)
    snap = cam.snapshot() if cam is not None and hasattr(cam, "snapshot") else None
    if snap is not None and snap.get("fresh"):
        return None
    return {"reason": "camera_stale_no_lidar", "clear_m": None, "blocked_s": 0.0,
            "error": f"{p.model} 当前没有测距兜底，相机帧不新鲜时禁止平移"}


def _reflex_limit(vx: float, vyaw: float) -> tuple[float, float, dict]:
    """Per-tick speed-limit hook for the executors. Uses the lidar guard if present; otherwise
    applies the camera-freshness gate (above), then `require`: if lidar data is required
    (navi.lidar.require / profile lidar_require) forward motion is blocked; only an explicit
    disable lets it through."""
    g = _get_lidar_guard()
    if g is not None:
        return g.limit(vx, vyaw)
    blind = _blind_translation_block(vx)
    if blind is not None:
        return 0.0, vyaw, blind
    p = _driver_profile()
    if p is not None and not p.lidar_ready:
        return vx, vyaw, {"reason": f"no_lidar_on_{p.model}", "clear_m": None, "blocked_s": 0.0}
    cfg = _section("lidar")
    if cfg["enabled"] and _lidar_require(cfg, p) and not _lidar_disabled_by_env():
        # Profile lidar_require only gates forward motion (the rear is a blind spot anyway);
        # an explicit config `require` gates both directions.
        if not (vx < 0 and not cfg["require"]):
            return 0.0, vyaw, {"reason": "no_data", "clear_m": None, "blocked_s": 0.0,
                               "error": _lidar_error or "雷达初始化中"}
    return vx, vyaw, dict(_LIDAR_OFF)


def _attach_reflex(driver) -> None:
    """Attach the reflex limiter to the discrete MotionExecutor (the sequence executor gets it
    at construction)."""
    m = getattr(driver, "motion", None)
    if m is not None and getattr(m, "speed_limiter", None) is None:
        m.speed_limiter = _reflex_limit


def _lidar_status() -> dict:
    """Status read that never creates the guard (observer polling must not decide when the
    motion path subscribes)."""
    if _lidar is None:
        cfg = _section("lidar")
        enabled = bool(cfg["enabled"] and not _lidar_disabled_by_env())
        profile = _driver_profile()
        if enabled and profile is not None and not profile.lidar_ready:
            return {"ok": False, "enabled": False, "state": f"unsupported_on_{profile.model}",
                    "has_lidar": profile.has_lidar, "require": bool(cfg["require"]), "error": None}
        return {"ok": False, "enabled": enabled, "state":
                ("failed" if _lidar_error else "initializing") if enabled else "disabled",
                "error": _lidar_error}
    summary = _lidar.summary()
    out = {"enabled": True, "state": "healthy" if summary["ok"] else "stale", **summary}
    if _lidar_feeder is not None:
        out["source"] = _lidar_feeder.stats()
        out["error"] = out["error"] or out["source"]["error"]
    if _imu is not None:
        out["imu"] = _imu.stats()
    return out


# ---------------------------------------------------------------------------
# Frame registry + inference latency estimate (sequence alignment reference)
# ---------------------------------------------------------------------------
_frames: "OrderedDict[int, tuple[float, float]]" = OrderedDict()   # id -> (sent_at, motion_t)
_frame_jpegs: "OrderedDict[int, bytes]" = OrderedDict()           # id -> JPEG (memory evidence)
_frames_lock = threading.Lock()
_frame_seq = 0
_FRAME_KEEP = 64
_FRAME_JPEG_KEEP = 16
_latency_ema = [0.0]           # inference latency EMA (s); 0 = no samples yet


def _register_frame(jpeg: bytes | None = None) -> tuple[int, float]:
    """Register a frame: record send time (latency estimate) and motion clock (submit trimming
    reference); cache the last few JPEGs as memory evidence."""
    global _frame_seq
    now = time.monotonic()
    motion_t = _seq_exec.motion_clock() if _seq_exec is not None else 0.0
    with _frames_lock:
        _frame_seq += 1
        fid = _frame_seq
        _frames[fid] = (now, motion_t)
        while len(_frames) > _FRAME_KEEP:
            _frames.popitem(last=False)
        if jpeg is not None:
            _frame_jpegs[fid] = jpeg
            while len(_frame_jpegs) > _FRAME_JPEG_KEEP:
                _frame_jpegs.popitem(last=False)
    return fid, now


def _frame_jpeg(frame_id) -> tuple[int, bytes]:
    """Return a cached JPEG by frame_id (None = latest). Only frames registered by this service
    are accepted, never arbitrary images."""
    with _frames_lock:
        if frame_id is None:
            if not _frame_jpegs:
                raise ValueError("还没有可用的帧：先 observe")
            return next(reversed(_frame_jpegs.items()))
        fid = int(frame_id)
        if fid not in _frame_jpegs:
            raise ValueError(f"frame_id {fid} 不在最近 {_FRAME_JPEG_KEEP} 帧缓存里：重新 observe 后用新的 frame_id")
        return fid, _frame_jpegs[fid]


def _lookup_frame(frame_id) -> tuple[float, float] | None:
    with _frames_lock:
        if frame_id is None:
            return next(reversed(_frames.values())) if _frames else None
        return _frames.get(int(frame_id))


def _update_latency(sample_s: float) -> float:
    ema = _latency_ema[0]
    _latency_ema[0] = sample_s if ema <= 0 else 0.7 * ema + 0.3 * sample_s
    return _latency_ema[0]


def _horizon_need(cfg: dict, observe_after_s: float = 4.0, speed: float | None = None) -> tuple[float, float]:
    """Return (motion duration needed to avoid stopping, allowed cap).
    need = observe point + inference latency * 1.3 margin; cap = min(duration hard cap,
    max_blind_m / speed). speed is the plan's mean linear speed (defaults to cfg["max_vx"];
    cfg must already be profile-limited)."""
    need = max(float(cfg["horizon_min_s"]), observe_after_s + _latency_ema[0] * 1.3)
    cap = float(cfg["max_duration_s"])
    v = float(cfg["max_vx"]) if speed is None else float(speed)
    if cfg.get("max_blind_m") and v > 0:
        cap = min(cap, max(float(cfg["horizon_min_s"]), float(cfg["max_blind_m"]) / v))
    return need, cap


def _suggested_horizon(cfg: dict, observe_after_s: float = 4.0, speed: float | None = None) -> float:
    """Suggested total motion duration: the non-stop need, capped (see _horizon_need)."""
    need, cap = _horizon_need(cfg, observe_after_s, speed)
    return round(min(need, cap), 1)


def _clearance():
    """Clearance in metres (R1D: from the front bumper) while the lidar guard is healthy;
    attached to every frame result so the model needn't call get_status.
    ahead_m = nearest obstacle in the body-width travel corridor (null = no return);
    front_m is a +/-30 deg sector that catches side walls in hallways, reference only.
    Returns None if no guard or stale data. Reads the existing guard only; never creates it."""
    g = _lidar
    if g is None:
        return None
    s = g.summary()
    if not s.get("ok"):
        return None
    out = {k: s.get(k) for k in ("front_m", "left_m", "right_m")}
    if "corridor_front_m" in s:
        out = {"ahead_m": s["corridor_front_m"], **out}
    return out


# ---------------------------------------------------------------------------
# Sequence executor singleton (lazy, bound to driver.robot)
# ---------------------------------------------------------------------------
_seq_exec = None
_seq_lock = threading.Lock()


def _get_sequence_executor() -> SequenceExecutor:
    global _seq_exec
    with _seq_lock:
        if _seq_exec is None:
            d = get_driver()
            cfg = _profile_limits(_section("sequence"), d)
            _attach_reflex(d)
            accel = cfg.get("max_accel_mps2") or getattr(_driver_profile(d), "extra", {}).get("max_accel_mps2")
            _seq_exec = SequenceExecutor(
                d.robot, blend_s=cfg["blend_s"], watchdog_s=cfg["watchdog_s"],
                ramp_time=config.RAMP_TIME, max_vx=cfg["max_vx"],
                max_vyaw=cfg["max_vyaw"], max_accel=accel, log=getattr(d, "_log", None),
                speed_limiter=_reflex_limit)
    return _seq_exec


def _stop_sequence_if_any():
    """Discrete move/turn and sequences are exclusive: clear any sequence first."""
    if _seq_exec is not None:
        _seq_exec.stop()


# ---------------------------------------------------------------------------
# Map navigation (optional; physical localization; exclusive with velocity modes).
# Read-only observer polling never initialises navigation.
# ---------------------------------------------------------------------------
_navigation = None
_navigation_lock = threading.Lock()
_motion_entry_lock = threading.Lock()
_map_dispatch_lock = threading.Lock()
_stop_generation = 0
_map_operation_cancel = None  # registered before init so stop never waits on I/O

_ros_mapping = None
_ros_mapping_lock = threading.Lock()


def _navigation_cfg() -> dict:
    return _section("navigation")


def _check_bool(cfg, key):
    if type(cfg.get(key)) is not bool:
        raise ValueError(f"navigation.{key} must be boolean")
    return cfg[key]


def _state_max_age(cfg) -> float:
    state_age = cfg.get("state_max_age_s", cfg["max_age_s"])
    if type(state_age) not in (int, float) or not math.isfinite(state_age) or state_age <= 0:
        raise ValueError("navigation.state_max_age_s must be a finite positive number")
    return float(state_age)


def _uses_ros_mapping(cfg) -> bool:
    return (cfg.get("mapping", {}).get("backend") == "slamware_ros"
            and cfg.get("engine", "onboard") == "onboard")


def _get_ros_mapping():
    """Initialise read-only onboard map/state subscriptions (reuses the driver's DDS factory;
    creates no writers, no motion)."""
    global _ros_mapping
    with _ros_mapping_lock:
        if _ros_mapping is None:
            from services.mapping_config import MappingConfig
            from services.slamware_ros_mapping import SlamwareRosMapping
            cfg = _navigation_cfg()
            mapping = MappingConfig.parse(cfg.get("mapping", {}))
            odom_required = _check_bool(cfg, "odom_required")
            receipt = _check_bool(cfg, "receipt_time_freshness")
            state_age = _state_max_age(cfg)
            # Validate config before touching hardware
            _require_slamware(get_driver())
            service = SlamwareRosMapping(frame=cfg["frame"], max_age_s=cfg["max_age_s"],
                state_max_age_s=state_age,
                min_quality=mapping.min_quality, source_offset_s=cfg.get("source_offset_s", 0.),
                trust_slam_state=mapping.trust_slam_state, odom_required=odom_required,
                require_map_health=mapping.require_map_health, receipt_time_freshness=receipt)
            service.start()
            _ros_mapping = service
        return _ros_mapping


def _ros_navigation_guard(goal=None):
    """Runtime navigation safety gate: map health + localization + lidar ready. Called by
    NavigationManager every tick."""
    mapping = _get_ros_mapping()
    # Taught places (PlaceStore.get) carry no frame key but are in the configured nav frame by
    # construction: stamp the authoritative frame so the goal bounds check has an explicit frame
    # instead of failing closed on every named goal. Goals that already have a frame are kept.
    if isinstance(goal, dict) and "frame" not in goal:
        goal = {**goal, "frame": mapping.localization.frame}
    result = mapping.status(goal)
    errors = list(result["errors"])
    lidar = _lidar_status()
    if not lidar.get("enabled") or not lidar.get("ok"):
        errors.append("lidar_not_ready")
    return dict(ok=not errors, errors=errors, mapping=result, lidar=lidar)


def _navigation_config_diagnostics(cfg=None):
    """Return actionable, non-sensitive config errors before any hardware initialisation."""
    cfg = _navigation_cfg() if cfg is None else cfg
    errors = []
    if type(cfg.get("enabled")) is not bool:
        errors.append("navi.navigation.enabled 必须是 true/false")
    elif not cfg["enabled"]:
        errors.append("navi.navigation.enabled=false；运动未启用，操作员可启用后重启；只读预检仍可执行")
    try:
        from services.mapping_config import MappingConfig
        MappingConfig.parse(cfg.get("mapping", {}))
    except ValueError as exc:
        errors.append(str(exc))
    for key in ("map_id", "frame", "places_file"):
        value = cfg.get(key)
        if not isinstance(value, str) or not value.strip():
            errors.append(f"缺少非空 navi.navigation.{key}（必须填写当前现场真实地图/坐标系，不能猜值）")
    return errors


def _place_store():
    from core.places import PlaceStore
    cfg = _navigation_cfg()
    errors = [e for e in _navigation_config_diagnostics(cfg)
              if any(key in e for key in ("map_id", "frame", "places_file"))]
    if errors:
        raise ValueError("地图配置不完整：" + "；".join(errors))
    path = Path(cfg["places_file"])
    if not path.is_absolute():
        path = _SRC / path
    return PlaceStore(path, cfg["map_id"], cfg["frame"])


def _get_navigation():
    global _navigation
    with _navigation_lock:
        if _navigation is None:
            cfg = _navigation_cfg()
            if not _check_bool(cfg, "enabled"):
                raise ValueError("地图导航未启用；先配置 navi.navigation 并完成现场定位/取消验收")
            if cfg.get("engine", "onboard") != "onboard":
                raise ValueError("navigation.engine 仅支持 onboard")
            from services.navigation import (
                Localization,
                NavigationManager,
                SlamwareBackend,
                resolve_goal_frame,
            )
            store = _place_store()  # validate map identity before touching hardware
            for k in ("max_age_s", "max_distance_m", "timeout_s", "position_tolerance_m",
                      "yaw_tolerance_deg", "settle_s", "stall_s"):
                if not math.isfinite(float(cfg[k])) or float(cfg[k]) <= 0:
                    raise ValueError(f"navigation.{k} 须为有限正数")
                cfg[k] = float(cfg[k])
            offset = cfg.get("source_offset_s", 0.)
            if type(offset) not in (int, float) or not math.isfinite(offset):
                raise ValueError("navigation.source_offset_s must be finite")
            if not isinstance(cfg.get("goal_frame", ""), str):
                raise ValueError("navigation.goal_frame must be a string")
            odom_required = _check_bool(cfg, "odom_required")
            receipt = _check_bool(cfg, "receipt_time_freshness")
            state_age = _state_max_age(cfg)
            d = _require_slamware(get_driver())
            localization = Localization(cfg["frame"], cfg["max_age_s"],
                                        strict_timestamps=True, source_offset_s=offset,
                                        receipt_time_freshness=receipt,
                                        state_max_age_s=state_age)
            backend = SlamwareBackend(d.robot, localization, goal_frame=resolve_goal_frame(cfg))
            options = {k: cfg[k] for k in ("timeout_s", "max_distance_m", "position_tolerance_m",
                                           "yaw_tolerance_deg", "settle_s", "stall_s")}
            if _uses_ros_mapping(cfg):
                _get_ros_mapping()
                _get_lidar_guard()
                options["safety_check"] = _ros_navigation_guard
            _navigation = NavigationManager(localization, backend, store,
                                            odom_required=odom_required, **options)
        return _navigation


def _navigation_status():
    if _navigation is not None:
        return _navigation.status()
    return {"ok": True, "active": False, "state": "not_initialized",
            "enabled": _navigation_cfg()["enabled"]}


def _manual_command(fn):
    """Entry wrapper for manual velocity tools: motion_busy if the motion lock is taken; refuse
    while map navigation owns the base."""
    @wraps(fn)
    def wrapped(*args, **kwargs):
        if not _motion_entry_lock.acquire(blocking=False):
            return {"ok": False, "error": "motion_busy"}
        try:
            if _navigation is not None and _navigation.status()["active"]:
                return {"ok": False, "error": "地图导航占用底盘；先 stop 并确认 navigation_status.active=false"}
            return fn(*args, **kwargs)
        finally:
            _motion_entry_lock.release()
    return wrapped


def _map_operation(fn, *, starts_motion=False):
    """Entry wrapper for map tools: sequences/discrete motion must be stopped; stop can cancel
    the operation during initialisation."""
    global _map_operation_cancel
    if not _motion_entry_lock.acquire(blocking=False):
        return {"ok": False, "error": "motion_busy"}
    try:
        with _map_dispatch_lock:
            generation = _stop_generation
            cancelled = threading.Event()
            _map_operation_cancel = cancelled
        if _seq_exec is not None:
            st = _seq_exec.status()
            if st["active"] or any(abs(v) > 1e-6 for v in st["cmd"].values()):
                return {"ok": False, "error": "序列仍在执行；先 stop 并停稳再标定/导航"}
        d = driver_factory.current_driver()
        if d is not None and d.motion is not None and d.motion.is_moving():
            return {"ok": False, "error": "离散动作仍在执行"}
        nav = _get_navigation()  # possibly slow init, kept outside the stop lock
        with _map_dispatch_lock:
            if generation != _stop_generation:
                return {"ok": False, "error": "初始化期间收到 stop，本次标定/导航已取消"}
        result = fn(nav, cancelled) if starts_motion else fn(nav)
        if cancelled.is_set():
            return {"ok": False, "error": "operation_cancelled_by_stop", "stop_requested": True}
        return result
    except (ValueError, OSError) as exc:
        return {"ok": False, "error": str(exc)}
    finally:
        with _map_dispatch_lock:
            _map_operation_cancel = None
        _motion_entry_lock.release()


# Max wait for a fresh frame for the model (see robot_tools.observe_frame); observer polling
# doesn't wait.
_FRAME_WAIT_S = 1.5


def _with_frame(res: dict):
    """Attach a current frame to the result and register its frame_id (settled frame for
    move/turn, planning reference for submit).

    Capture failure doesn't affect the main result: frame_error is added so the agent can
    observe again.
    """
    out = robot_tools.observe_frame(get_driver(), None, wait_fresh_s=_FRAME_WAIT_S)
    if out.get("ok"):
        fid, _ = _register_frame(out["jpeg"])
        res["frame_id"] = fid
        clearance = _clearance()
        if clearance is not None:
            res["clearance"] = clearance
        res.update({k: out[k] for k in ("source_frame_seq", "received_at", "frame_age_ms", "fresh") if k in out})
        return _frame_content(res, out)
    res["frame_error"] = out.get("error", "抓帧失败")
    return res


def _fov_text(ratio: float) -> tuple[str, str]:
    """Horizontal FOV labels for (normal-FOV crop, full frame); empty if the profile has no
    camera_hfov_deg."""
    p = _driver_profile()
    hfov = (getattr(p, "extra", None) or {}).get("camera_hfov_deg") if p is not None else None
    if not hfov:
        return "", ""
    narrow = math.degrees(2 * math.atan(ratio * math.tan(math.radians(float(hfov)) / 2)))
    return f"（水平约 {narrow:.0f}°）", f"（水平约 {float(hfov):.0f}°）"


def _frame_content(res: dict, out: dict) -> list:
    """Result + image(s). With a centre crop (R1D) the same frame_id yields two images, each
    with a caption: first the zoomed normal-FOV view (path ahead, distances), then the
    ultra-wide full frame (sides, target search)."""
    if "crop_jpeg" not in out:
        return [res, Image(data=out["jpeg"], format="jpeg")]
    res["images"] = ["normal", "wide"]
    res["crop_ratio"] = out["crop_ratio"]
    narrow, wide = _fov_text(out["crop_ratio"])
    return [res,
            f"[frame {res['frame_id']} 图1/2] 正常视场{narrow}：中央 {round(out['crop_ratio'] * 100)}% 放大，"
            "物体大小和远近接近人眼所见——判断正前方行进走廊、障碍与距离以这张为准",
            Image(data=out["crop_jpeg"], format="jpeg"),
            f"[frame {res['frame_id']} 图2/2] 超广角全幅{wide}：画面中心的东西会显得比实际更小、更远，"
            "只用来看两侧、找目标、定方向",
            Image(data=out["jpeg"], format="jpeg")]


_spin = robot_tools.SpinWatch()
_yaw_mark = [None]                 # base yaw totals at the last submit (R1D yaw_totals)


def _heading_since_last_submit(d):
    """Commanded vs gyro-measured yaw since the last submit (deg, positive = left); the
    difference is lag or drift. None without a gyro, on the first submit, or if the gyro had
    gaps in between."""
    totals = getattr(getattr(d, "robot", None), "yaw_totals", None)
    now = totals() if callable(totals) else None
    prev, _yaw_mark[0] = _yaw_mark[0], now
    if now is None or prev is None or now["gaps"] != prev["gaps"]:
        return None
    return {"cmd_deg": round(math.degrees(now["cmd"] - prev["cmd"]), 1),
            "gyro_deg": round(math.degrees(now["gyro"] - prev["gyro"]), 1)}


def _spin_check(res: dict, turn_deg: float, move_m: float) -> None:
    """Record a step; add a spin warning once a full turn accumulates without moving away (the
    model can't keep count itself). Uses lidar SLAM pose while tracking; otherwise SpinWatch
    falls back to commanded translation."""
    pose = None
    if _slam_runner is not None:
        snap = _slam_runner.slam.snapshot(_SLAM_MAX_AGE_S)
        if snap.get("pose") is not None and snap["state"] in ("tracking", "degraded"):
            pose = (snap["pose"]["x"], snap["pose"]["y"])
    hit = _spin.record(turn_deg, move_m, pose)
    if hit:
        res["spin"] = (f"原地打转：自上次挪开位置以来已累计转了 {hit['turned_deg']}°，位置只挪了 {hit['moved_m']}m。"
                       "别再原地左右转着找——从这几帧里挑最像通道/出口的方向（用户给了参照物就先把参照物转到画面中央，"
                       "再看它指定的那一侧），对准后走过去 2~3m 再看；看完一整圈还没找到，就换个位置（走到房间另一头）再看。")


# ---------------------------------------------------------------------------
# Tool logic
# ---------------------------------------------------------------------------
@_manual_command
def _tool_move(direction: str, distance_m: float):
    try:
        g = robot_tools.guard_move(direction, distance_m, _guard_limits()["max_move_distance_m"])
    except ValueError as e:
        return {"ok": False, "error": str(e)}
    d = get_driver()
    error = _motion_error(d)
    if error:
        return {"ok": False, "error": error}
    _stop_sequence_if_any()
    _attach_reflex(d)
    res = d.move(g["direction"], distance_m=g["distance_m"], timeout_s=30.0)
    out = {"ok": res["status"] == "done", "status": res["status"],
           "moved_m": res.get("moved_m", 0.0), "clamped": g["clamped"],
           "distance_source": "command_integral_estimate", "measured_distance_m": None,
           "requested_distance_m": distance_m, "completion_reason": res["status"]}
    if res.get("yaw_change_rad") is not None:            # R1D gyro heading drift (+ = left)
        out["measured_yaw_change_deg"] = round(math.degrees(res["yaw_change_rad"]), 1)
    _spin_check(out, 0.0, out["moved_m"])
    return _with_frame(out)


@_manual_command
def _tool_turn(angle_deg: float):
    try:
        g = robot_tools.guard_turn(angle_deg, _guard_limits()["max_turn_angle_deg"])
    except ValueError as e:
        return {"ok": False, "error": str(e)}
    d = get_driver()
    error = _motion_error(d)
    if error:
        return {"ok": False, "error": error}
    _stop_sequence_if_any()
    _attach_reflex(d)
    res = d.turn(g["angle_deg"], timeout_s=30.0)
    # turned_rad is the command integral (same source as get_odometry). With the R1D gyro,
    # measured_angle_rad is the measured turn (+ = left, incl. post-stop slip), and the turn
    # stops based on it.
    measured = res.get("measured_rad")
    out = {"ok": res["status"] == "done", "status": res["status"],
           "turned_rad": res.get("turned_rad", 0.0), "clamped": g["clamped"],
           "angle_source": "gyro" if measured is not None else "command_integral_estimate",
           "measured_angle_rad": measured}
    if res.get("turn_stop") == "timeout":
        out["turn_stop"] = "timeout: 陀螺实测转角迟迟不到目标（卡住或打滑），按超时停了；看画面确认实际朝向"
    _spin_check(out, math.degrees(abs(measured) if measured is not None else out["turned_rad"]), 0.0)
    return _with_frame(out)


def _tool_observe(eye: str | None = None):
    out = robot_tools.observe_frame(get_driver(), eye, wait_fresh_s=_FRAME_WAIT_S)
    if not out.get("ok"):
        return out
    fid, _ = _register_frame(out["jpeg"])
    res = {"ok": True, "frame_id": fid, "eye": out["eye"],
           **{k: out[k] for k in ("source_frame_seq", "received_at", "frame_age_ms", "fresh") if k in out}}
    clearance = _clearance()
    if clearance is not None:
        res["clearance"] = clearance
    return _frame_content(res, out)


@_manual_command
def _tool_submit_sequence(segments, frame_id=None, observe_after_s=0.0, alignment="current"):
    if alignment not in ("current", "continue"):
        return {"ok": False, "error": "alignment 须为 current 或 continue"}
    if not math.isfinite(float(observe_after_s or 0.0)):
        return {"ok": False, "error": "observe_after_s 须为有限数"}
    d = get_driver()
    error = _motion_error(d)
    if error:
        return {"ok": False, "error": error}
    cfg = _profile_limits(_section("sequence"), d)   # profile-limited so the caps are right
    limits = {**_guard_limits(), "max_vx": cfg["max_vx"], "max_vyaw": cfg["max_vyaw"],
              "max_duration_s": cfg["max_duration_s"]}
    if cfg.get("max_distance_m"):
        limits["max_move_distance_m"] = float(cfg["max_distance_m"])
    try:
        g = robot_tools.guard_sequence(segments, limits)
    except ValueError as e:
        return {"ok": False, "error": str(e)}
    frame = _lookup_frame(frame_id)
    if frame_id is not None and frame is None:
        return {"ok": False, "error": f"frame_id {frame_id} 未登记或已过期，请用最近一次返回的 frame_id"}
    frame_ts, frame_motion_t = frame if frame is not None else (None, None)
    # Cancel any running discrete motion (MotionExecutor is single-flight); the sequence takes over
    if d.motion is not None and d.motion.is_moving():
        d.motion.stop()
    ex = _get_sequence_executor()
    track = _heading_since_last_submit(d)
    res = ex.submit(g["segments"], frame_ts=frame_ts, frame_motion_t=frame_motion_t, alignment=alignment)
    if track is not None:
        res["heading_track"] = track
    if frame_ts is not None:
        ema = _update_latency(time.monotonic() - frame_ts)
        # Widen the watchdog with measured latency: threshold = max(config floor, 2*latency + 1 s),
        # so slow models (~10 s/step) aren't starved every step; fast models keep the floor.
        ex.set_watchdog(max(float(cfg["watchdog_s"]), ema * 2.0 + 1.0))
    res["clamped"] = g["clamped"]
    res["accepted_s"] = g["total_s"]
    res["motion_s"] = g["motion_s"]
    if g["stripped_zero_s"] > 0:
        res["stripped_zero_s"] = g["stripped_zero_s"]
    wait = max(0.0, min(float(observe_after_s or 0.0), float(cfg["max_observe_after_s"])))
    res["latency_est_s"] = round(_latency_ema[0], 2)
    dist = sum(abs(vx) * dur for vx, _, dur in g["segments"])
    speed = dist / g["motion_s"] if g["motion_s"] > 0 and dist > 0 else None    # mean speed
    need, cap = _horizon_need(cfg, wait if wait > 0 else 4.0, speed)
    res["suggested_horizon_s"] = round(min(need, cap), 1)
    if not res.get("ok"):
        return res
    # If lidar already blocks the path at submit time, say so (the model can reroute; the reflex
    # layer has already clamped forward velocity).
    refl = _reflex_limit(float(g["segments"][0][0]), 0.0)[2]
    if refl.get("reason") in ("blocked", "no_data"):
        res["blocked_by_lidar"] = refl.get("clear_m")
    # Optionally wait before capturing, so the post-motion view arrives in the same turn
    if wait > 0:
        time.sleep(wait)
        res["observed_after_s"] = round(wait, 1)
        res["still_moving"] = bool(ex.status()["active"])
    if need > cap and _latency_ema[0] > 0:
        # Inference too slow: non-stop would mean several metres blind. Don't push non-stop, but
        # make each segment long enough (a step can take 20-30 s; 1 m segments mostly idle).
        v = speed or float(cfg["max_vx"])
        res["note"] = (f"推理时延约 {res['latency_est_s']}s，按这段 {v:.2f}m/s 要不停车得一次走 ≥ {need * v:.1f}m"
                       f"（上限 {cfg['max_blind_m']}m），所以每段走完会停下等你——那就每段走够："
                       f"前方行进走廊空（clearance.ahead_m 远、画面里路是通的）时一段给到 {cfg['max_blind_m']}m 左右，"
                       "速度可提到 0.4~0.6m/s（雷达按前方净空自动减速），走完立刻提交下一段，别插 observe/get_status")
    elif wait >= g["motion_s"] - 0.05 and _latency_ema[0] > 0:
        res["note"] = ("机器人已停稳等你思考（observe_after_s ≥ 运动时长）。想不停车："
                       f"observe_after_s 取 3~5s，运动段总时长 ≥ {res['suggested_horizon_s']}s")
    elif g["motion_s"] < wait + _latency_ema[0]:
        res["note"] = (f"运动段 {g['motion_s']}s < 拍帧点 {wait}s + 推理时延 "
                       f"{res['latency_est_s']}s，你提交下一段前机器人会停下；"
                       f"要连续请把运动段放长到 ≥ {res['suggested_horizon_s']}s")
    _spin_check(res, math.degrees(sum(abs(vyaw) * dur for _, vyaw, dur in g["segments"])), dist)
    return _with_frame(res)


def _tool_sequence_status():
    cfg = _profile_limits(_section("sequence"))      # blind-distance cap uses profile limits
    lidar = _lidar_status()
    if _seq_exec is None:
        return {"ok": True, "active": False, "remaining_s": 0.0,
                "latency_est_s": round(_latency_ema[0], 2),
                "suggested_horizon_s": _suggested_horizon(cfg), "lidar": lidar}
    st = _seq_exec.status()
    st.update(ok=True, latency_est_s=round(_latency_ema[0], 2),
              suggested_horizon_s=_suggested_horizon(cfg), lidar=lidar)
    return st


def _wait_first_scan(g, timeout_s: float = 0.8) -> None:
    """Wait briefly for the first scan after the guard is created (Livox needs 0.1-0.2 s per
    scan); otherwise the session's first get_status reports stale / all null."""
    if g is None:
        return
    deadline = time.monotonic() + timeout_s
    while g.n_recv == 0 and time.monotonic() < deadline:
        time.sleep(0.05)


def _tool_get_status():
    d = get_driver()
    _wait_first_scan(_get_lidar_guard())     # explicit query may create the guard
    return {"ok": True, **tool_status(d), "lidar": _lidar_status(),
            "navigation": _navigation_status()}


def _tool_stop_all():
    global _stop_generation
    with _map_dispatch_lock:
        _stop_generation += 1
        if _map_operation_cancel is not None:
            _map_operation_cancel.set()
        nav = _navigation
    # Don't call status() first: the manager lock may be held by I/O during init.
    if nav is not None:
        stopped = nav.stop()
        if stopped.get("active"):
            return stopped  # request and confirmation are separate; never fake "stopped"
    _stop_sequence_if_any()
    return tool_stop(get_driver())


def _pre_align(nav, name, cancelled):
    """Before sending a goal, turn in place if its bearing exceeds navigation.pre_align_deg.

    Field experience: goals ahead succeed; goals behind leave the base motionless until stall
    timeout. The lidar is blind directly behind, so face the goal before handing it to
    SLAMWARE. Closed loop: after each turn wait for a pose update and recompute, at most twice.
    Skips if localization is unavailable or the place is missing; nav.start reports the error."""
    cfg = _navigation_cfg()
    threshold = float(cfg.get("pre_align_deg", 0.0) or 0.0)
    if threshold <= 0:
        return {}
    from core.places import finite_pose
    from services.navigation import pre_align_turn_deg, wrap_angle
    speed = float(cfg.get("pre_align_turn_speed", 0.35))
    try:
        goal = nav.store.get(name)
    except (KeyError, ValueError):
        return {}
    # Same check as nav.start: tolerates odom_stale/invalid_twist when odom_required=false.
    loc_ok = getattr(nav, "_nav_ok", None) or (lambda snap: snap.get("ok"))
    info = {"threshold_deg": threshold, "turned_deg": 0.0}
    for _ in range(2):
        s = nav.localization.snapshot()
        if not loc_ok(s):
            info["skipped"] = "localization_unavailable:" + ",".join(map(str, s.get("errors") or []))
            break
        err = pre_align_turn_deg(finite_pose(s["pose"]), goal, threshold)
        info.setdefault("initial_error_deg", round(err, 1))
        if not err:
            break
        if cancelled.is_set():
            return {**info, "error": "operation_cancelled_by_stop"}
        d = get_driver()
        _attach_reflex(d)
        res = d.turn(err, turn_speed=speed, timeout_s=30.0)
        if res.get("status") != "done":
            return {**info, "error": f"pre_align_turn_{res.get('status')}"}
        info["turned_deg"] = round(info["turned_deg"] + err, 1)
        time.sleep(1.0)  # settle and wait for rt/robot_pose to refresh before recomputing
    s = nav.localization.snapshot()
    if loc_ok(s):
        p = finite_pose(s["pose"])
        info["final_error_deg"] = round(abs(math.degrees(wrap_angle(
            math.atan2(goal["y"] - p["y"], goal["x"] - p["x"]) - p["yaw"]))), 1)
    return info


def _ros_preflight(cfg, config_errors):
    """Read-only ROS diagnostics (works even with navigation disabled); never builds a
    NavigationManager or goal writer.

    Serialised with motion/map operations, but stop doesn't take this lock and can interrupt."""
    active = _navigation.status()["active"] if _navigation is not None else False
    errors = list(config_errors)
    result = dict(ok=False, read_only=True, motion_started=False, active=active,
                  backend="slamware_ros", engine=cfg.get("engine", "onboard"),
                  config=dict(ready=not config_errors, errors=list(config_errors),
                              enabled=cfg.get("enabled"), map_id=cfg.get("map_id"),
                              frame=cfg.get("frame")))
    if not _motion_entry_lock.acquire(blocking=False):
        return dict(result, errors=errors + ["motion_busy"], error="motion_busy")
    try:
        # Cold-start warm-up: poll new DDS subscriptions within a bounded window until fresh.
        # When already warm the loop breaks after one pass.
        deadline = time.monotonic() + _PREFLIGHT_WARMUP_S
        while True:
            mapping, telemetry_error, warmup = None, None, []
            try:
                mapping = _get_ros_mapping().status()  # read-only; no goal writer
                warmup.extend(mapping["errors"])
            except Exception as exc:
                unsupported = _slamware_error()
                # Model without SLAMWARE detected on first driver creation: model gate, not a
                # telemetry fault.
                if unsupported:
                    return dict(result, errors=[unsupported], error=unsupported)
                telemetry_error = "ros_telemetry_unavailable: " + str(exc)
            _get_lidar_guard()
            lidar = _lidar_status()
            if not lidar.get("enabled") or not lidar.get("ok"):
                warmup.append("lidar_not_ready")
            # Only freshness/readiness errors are worth waiting for; config/slam_not_connected/
            # frame_mismatch etc. won't clear by waiting.
            if time.monotonic() >= deadline or (
                    telemetry_error is None
                    and not any(e in _PREFLIGHT_WARMUP_KEYS for e in warmup)):
                break
            time.sleep(_PREFLIGHT_WARMUP_POLL_S)
        if mapping is not None:
            result["mapping"] = mapping
            result["localization"] = mapping["localization"]
            errors.extend(mapping["errors"])
            from services.navigation import NavigationManager
            if mapping["localization"]["ok"] and not NavigationManager._stationary(mapping["localization"]):
                errors.append("robot_not_stationary")
        if telemetry_error is not None:
            errors.append(telemetry_error)
        result["lidar"] = lidar
        if not lidar.get("enabled") or not lidar.get("ok"):
            errors.append("lidar_not_ready")
        if active:
            errors.append("navigation_active：先停止并确认控制权释放")
        if _seq_exec is not None and _seq_exec.status()["active"]:
            errors.append("sequence_active")
        result["errors"] = list(dict.fromkeys(errors))
        result["ok"] = not result["errors"]
        return result
    finally:
        _motion_entry_lock.release()


# ---------------------------------------------------------------------------
# Observer surface (polled by the host plugin navi-observer, see services/observer.py)
# ---------------------------------------------------------------------------
def _frame_state():
    """Latest frame given to the model: its age and how many seconds of non-zero sequence
    commands ran since. moved_since_s ~ 0 means the model's view is current, so the observer
    plugin stops pushing "settled". None if no frame has been sent."""
    with _frames_lock:
        if not _frames:
            return None
        fid, (sent_at, motion_t) = next(reversed(_frames.items()))
    moved = (_seq_exec.motion_clock() - motion_t) if _seq_exec is not None else 0.0
    return {"id": fid, "age_s": round(time.monotonic() - sent_at, 2), "moved_since_s": round(max(0.0, moved), 2)}


_task_board = {"tasks": [], "updated_at": None}    # set by update_tasks, shown on camera page
_RUN_STARTED = datetime.now(timezone.utc)          # camera page lists only places taught this run
_task_lock = threading.Lock()


def _taught_this_run(place: dict) -> bool:
    try:
        return datetime.fromisoformat(place.get("taught_at", "")) >= _RUN_STARTED
    except (TypeError, ValueError):
        return False


def _panel_places():
    """Camera page "recorded rooms": places saved (or re-taught) by save_place in this run only.
    Read-only: returns None if lidar SLAM / map config isn't up, never initialising anything."""
    try:
        if _uses_lidar_slam():
            store = _slam_place_store() if _slam_runner is not None else None
        else:
            store = _place_store() if _driver_profile() is not None else None
        if store is None:
            return None
        return [{"name": p["name"], "taught_at": p["taught_at"]} for p in store.list()
                if _taught_this_run(p)]
    except (ValueError, OSError):
        return None


def _observer_state() -> dict:
    """Status JSON: sequence progress, latency, lidar, navigation, odometry, latest frame, task
    board and recorded rooms. Deliberately never creates the driver."""
    d = driver_factory.current_driver()
    with _task_lock:
        task = {"tasks": list(_task_board["tasks"]), "updated_at": _task_board["updated_at"]}
    out = {"ts": round(time.time(), 3),
           "latency_est_s": round(_latency_ema[0], 2),
           "lidar": _lidar_status(), "navigation": _navigation_status(),
           "task": task, "places": _panel_places()}
    out["seq"] = _seq_exec.status() if _seq_exec is not None else {"active": False, "remaining_s": 0.0}
    frame = _frame_state()
    if frame is not None:
        out["frame"] = frame
    profile = _driver_profile(d)
    if profile is not None:
        out["robot"] = {"model": profile.model, "reason": profile.reason,
                        "capabilities": profile.capabilities()}
    if d is not None:
        try:
            s = d.status()
            out["driver"] = {"connected": s.get("robot_connected"),
                             "position": s.get("position"), "yaw": s.get("yaw"),
                             "moving": bool(d.motion is not None and d.motion.is_moving())}
        except Exception as e:
            out["driver"] = {"error": f"{type(e).__name__}: {e}"}
    return out


def _observer_frame():
    """Latest camera frame as JPEG, or None if the camera is unavailable."""
    d = driver_factory.current_driver()
    if d is None:
        return None
    out = robot_tools.observe_frame(d, None, with_crop=False)
    return out.get("jpeg") if out.get("ok") else None


_observer = None


def _runlog_since(query: dict) -> dict:
    """Observer /runlog?after=N: run-log entries with seq >= N (incremental camera-page fetch)."""
    try:
        after = max(0, int(query.get("after", 0)))
    except (TypeError, ValueError):
        after = 0
    return _runlog.since(after)


def _start_observer():
    """Start the observer surface (disabled if navi.observer.port <= 0). __main__ only."""
    global _observer
    port = int(_section("observer").get("port", 0) or 0)
    if port <= 0:
        return None
    from services.observer import ObserverServer
    _observer = ObserverServer(_observer_state, _observer_frame, port=port,
                               routes={"/runlog": _runlog_since},
                               post_routes={"/runlog": lambda body: _runlog.note(body)})
    _observer.start()
    return _observer


# ---------------------------------------------------------------------------
# Tool registration. Every call goes through _runlog (shown under the camera page view; written
# to disk if NAVI_RUN_LOG is a path).
# ---------------------------------------------------------------------------
_runlog = RunLog(os.environ.get("NAVI_RUN_LOG") if os.environ.get("NAVI_RUN_LOG", "0") != "0" else None)


def _tool(fn):
    """Register an MCP tool, logging call args and result summary to the run log (signature and
    docstring pass through to FastMCP unchanged via wraps)."""
    sig = inspect.signature(fn)

    @wraps(fn)
    def wrapped(*args, **kwargs):
        params = sig.bind_partial(*args, **kwargs).arguments
        return _runlog.wrap(fn.__name__, dict(params), lambda: fn(*args, **kwargs))
    return mcp.tool()(wrapped)


@_tool
def observe(eye: str | None = None):
    """获取一帧当前相机图像，附 frame_id（提交序列时用它做时间对齐基准）。
    eye: left/right/full（双目相机有效，默认 left）。"""
    return _tool_observe(eye)


@_tool
def move(direction: str, distance_m: float):
    """朝 direction（forward/backward）平移 distance_m 米。单次硬顶 5m。
    速度固定 0.3 m/s（对位用）；开阔地长距离请用 submit_sequence（可到 capabilities.max_vx）。
    同步阻塞；执行完成后结果自带一帧停稳后的相机图像——移动后不必再调 observe。
    若有序列在执行会先被清空。G1D 轮式底盘不支持横移，转向请用 turn。
    返回 status="blocked"：前方/后方雷达净空不足被反射层拦住而中止（此时 ok=false），
    别再重复同一条指令——先转向绕开。measured_yaw_change_deg（R1D 陀螺）：这段直行里车头实际偏了多少（正=左）。"""
    return _tool_move(direction, distance_m)


@_tool
def turn(angle_deg: float):
    """原地转向 angle_deg 度（正=左转，负=右转）。|角度| 须 ≥ 10°（更小无意义，会被拒绝），
    单次硬顶 ±360°；同步阻塞。避障绕行一次至少 30°。
    执行完成后结果自带一帧停稳后的相机图像——转向后不必再调 observe。
    turned_rad 为指令积分估算（get_odometry 同源）；R1D 陀螺可用时按陀螺实测转角到位才停，
    measured_angle_rad 给实测转角（正=左），angle_source=gyro。"""
    return _tool_turn(angle_deg)


@_tool
def submit_sequence(segments: list[dict], frame_id: int | None = None,
                    observe_after_s: float = 0.0, alignment: str = "current"):
    """连续行走的首选工具：提交未来一段速度序列，后台连续执行、与新序列自动平滑合并，
    立即返回并附一帧运动中画面——用它边走边想，别退回 move/turn 一步一停。

    典型用法（接近目标 / 开阔巡航）：一次给多段，直段走完接弧段绕行，中途不停：
      submit_sequence(segments=[
        {"vx": 0.4, "vyaw": 0.0,  "duration": 4},   # 开阔直行，vx 可到 capabilities.max_vx
        {"vx": 0.3, "vyaw": 0.35, "duration": 3},   # 左弧绕过前方障碍：vx、vyaw 同时非零＝画弧
        {"vx": 0.4, "vyaw": 0.0,  "duration": 3},
      ], observe_after_s=3)
    绕障用弧段（vx+vyaw 同时非零）而不是原地 turn，轨迹才连贯平滑。

    边走边想：observe_after_s 取小（接近阶段 3~5s，别设成等于总时长那样走一段停一段）；
    返回的是运动中画面，机器人还在走，你想完在它停下前提交下一段，服务端按 current 从
    收到时刻平滑接上（mode=blend）。只有对位/前方不确定时才等停稳再看。

    segments: [{"vx": m/s 正前负后, "vyaw": rad/s 正左负右, "duration": 秒}, ...]；不支持侧移。
      线速度按 capabilities.max_vx 封顶（R1D 1.0 m/s）、角速度 ≤ capabilities.max_vyaw，总时长/位移/转角
      有硬顶，超出被守门员钳制。前进时雷达反射层按前方行进路径的净空自动减速、贴近才停，不必自己预先降速。
      被拦期间转向也会一起刹停；被拦持续约 2s 剩余计划会被自动放弃（状态里 aborted=true），
      届时先 observe 看清挡路物、换方向后再提交，别原样重发。
      只写真正要走的段：走完自动停，别用零速段凑时长（会被剥掉），停车用 stop。
    alignment=current（默认）：新计划从收到时刻开始。
    alignment=continue：仅当帧后旧计划恒速且新计划被裁前缀与其一致才允许裁头；无法验证或
      已过期会拒绝，须重新观察按 current 规划。
    frame_id: 参考帧登记 ID，用于延迟统计，缺省取最近帧；source_frame_seq 相同＝复用同一缓存帧，
      frame_age_ms 是本机接收后年龄。
    observe_after_s: 提交后延迟抓帧，不保证此时停稳；过期相机帧返回错误但不自动取消序列，
      须 stop 恢复观测后再规划。
    返回 mode/remaining_s/alignment/dropped_s 与延迟、图像元数据；motion_s/driven_s 是命令时间
      统计不是实测运动。heading_track（R1D 陀螺）：上次提交以来指令偏航 cmd_deg 与陀螺实测 gyro_deg，
      两者差得多说明车头被带偏了，按差值修方向。与地图导航互斥；已标定点任务优先 navigate_to，不要把坐标转成盲走序列。
    """
    return _tool_submit_sequence(segments, frame_id, observe_after_s, alignment)


@_tool
def sequence_status() -> dict:
    """查询序列执行状态：active / remaining_s / 当前下发速度 / 距上次提交时长 / 是否被看门狗停过。"""
    return _tool_sequence_status()


@_tool
def stop() -> dict:
    """请求停止：清空速度序列/取消离散动作，或取消地图导航并验证停稳。
    切换模式、观测失效和风险出现时均可用。地图模式需继续查询 navigation_status，
    active=false 才表示控制权已释放；fault/active=true 禁止继续运动。软件停止不替代硬件急停。"""
    return _tool_stop_all()


@_tool
def get_odometry() -> dict:
    """读取手动速度命令积分估计（不是地图定位/实测里程计），地图导航期间不会随之更新。"""
    return {**tool_odometry(get_driver()), "source": "command_integral_estimate", "measured": False}


@_tool
def get_status() -> dict:
    """机器人/相机/连接当前状态，含雷达反射层（lidar.corridor_front_m = 正前方车身同宽走廊里的净空米数，
    null = 这条路上没有回波；front_m 是 ±30° 扇区、会扫到两侧墙，仅参考；ok=数据新鲜；
    前方净空不足时前进分量会被自动减速/归零，无需你干预）。"""
    return _tool_get_status()


@_tool
def get_localization() -> dict:
    """读取真实 SLAMWARE 地图定位、实测速度、新鲜度（不是 get_odometry 开环积分）。
    首次调用仅创建订阅，未收到数据时 ok=false；确认 frame 与配置一致。"""
    try:
        d = get_driver()
        if _uses_lidar_slam(_driver_profile(d)):
            return _slam_localization()
        error = _slamware_error(d)
        if error:
            return {"ok": False, "error": error}
        return _get_navigation().localization.snapshot()
    except (ValueError, OSError) as exc:
        return {"ok": False, "error": str(exc)}


@_tool
def list_places() -> dict:
    """列出当前地图下实地标定的命名点。R1D（雷达定位）另给每个点相对你的 distance_m / bearing_deg（正=左）。"""
    try:
        if _uses_lidar_slam():
            return _slam_list_places()
        store = _place_store()
    except (ValueError, OSError) as exc:
        return {"ok": False, "error": str(exc)}
    return {"ok": True, "map_id": store.map_id, "frame": store.frame, "places": store.list()}


@_tool
def update_tasks(tasks: list[dict]) -> dict:
    """把任务清单同步到现场相机页（只做展示：不动底盘、不影响任何运动和判断）。
    每次发整份清单：[{"name": "A2 找到厨房并标记", "status": "doing", "note": ""}, ...]
      name 用用户的编号和原话；status：todo 待做 / doing 正在做 / done 已完成 / dropped 放弃或确认做不成；
      note 写一句结果（「已标记」「门未关」「未找到」），没有就省略。
    拆好任务清单时调一次，之后每开始或做完一项再调一次；和下一步的运动/观察工具放在同一次回复里并行调用，
    不要为它单独占一轮。空列表清空看板。"""
    try:
        items = robot_tools.normalize_tasks(tasks)
    except ValueError as exc:
        return {"ok": False, "error": str(exc)}
    with _task_lock:
        _task_board.update(tasks=items, updated_at=round(time.time(), 3))
    return {"ok": True, "doing": [t["name"] for t in items if t["status"] == "doing"],
            "done": sum(t["status"] == "done" for t in items), "total": len(items)}


@_tool
def save_place(name: str, overwrite: bool = True) -> dict:
    """用户要求标定时，将当前新鲜 SLAM 定位保存为命名点。须底盘静止。不接受模型猜测的坐标。
    标房间（卧室/厨房/客厅…）必须人已经在房间里面：车身已越过门口或区域分界，房间的标志陈设就在你左右身边
    1.5m 内；只是从门口或隔壁区域看见它（例如在客厅看到远处的厨房台面）不算进去，继续往里走，进去后再标。
    重名默认覆盖刷新——每次任务的标记本来就该是当下实测的新点（用户的标记指令就是授权，不必等确认）；
    结果里 replaced=true 表示覆盖了旧点。只有明确想保留旧点、先确认有没有重名时才传 overwrite=false。"""
    if _uses_lidar_slam(_driver_profile(get_driver())):
        return _slam_save_place(name, overwrite)
    return _map_operation(lambda nav: {"ok": True, "place": nav.save_place(name, overwrite)})


@_tool
def navigate_to(name: str) -> dict:
    """异步导航到已标定点；由机载 SLAMWARE 规划避障，不是 VLM 盲走坐标。
    仅接受 list_places 中当前地图的点。返回 running 只代表已提交，随后查询 navigation_status。
    期间禁止 move/turn/submit_sequence。stop 可取消；只有 state=arrived 才能宣称到达。"""
    if _uses_lidar_slam(_driver_profile(get_driver())):
        return {"ok": False, "error": "unsupported_on_r1d: 本机型还没有自动导航。用 list_places 给的 distance_m / "
                                      "bearing_deg 手动走过去：先转向 bearing_deg，再走 distance_m，每走一两段再 list_places "
                                      "复核；distance_m ≤ 0.5 且 heading_diff_deg 不大才算到"}
    def run(nav, cancelled):
        aligned = _pre_align(nav, name, cancelled)
        if aligned.get("error"):
            return dict(ok=False, error=aligned["error"], pre_align=aligned)
        result = nav.start(name, cancel=cancelled)
        if aligned and isinstance(result, dict):
            result = {**result, "pre_align": aligned}
        return result
    return _map_operation(run, starts_motion=True)


@_tool
def navigation_status() -> dict:
    """地图导航状态及距目标距离/朝向误差。active=true 也可能是停车待确认故障，不能切速度控制。
    arrived 才是定位+实测静止确认的到达；running/timeout/stalled/fault 均不是成功。"""
    return _navigation_status()


@_tool
def mapping_capabilities() -> dict:
    """只读返回当前地图后端能力；不初始化设备。"""
    if _uses_lidar_slam():
        return dict(ok=True, read_only=True, motion_started=False, backend="lidar_slam",
                    navigation_enabled=False, existing_map_navigation=False, map_health_gate=False,
                    online_map_building=True, map_save_load=True, persistence="local_file",
                    note="R1D：本服务用底盘 Livox 雷达边走边建二维地图并定位（get_localization），地图与标定点"
                         "自动存盘、重启后自动重定位；save_place / list_places 可用，自动导航（navigate_to）暂不支持。")
    cfg = _navigation_cfg()
    backend = cfg.get("mapping", {}).get("backend", "disabled")
    ros = backend == "slamware_ros"
    return dict(ok=True, read_only=True, motion_started=False, backend=backend,
                engine=cfg.get("engine"), navigation_enabled=cfg.get("enabled"),
                existing_map_navigation=ros, map_health_gate=ros,
                online_map_building=False, map_save_load=False,
                persistence="onboard_managed" if ros else "unconfigured",
                note=("地图由机器人机载 SLAMWARE 创建与保存；本工具面只读地图健康并提交目标点，"
                      "不支持在线清图/导图。") if ros else
                     "未配置 slamware_ros：可用手动/序列模式；地图导航需现场配置 navi.navigation.mapping。")


@_tool
def mapping_status() -> dict:
    """只读回读当前机载地图、定位质量、TF 和建图开关（仅 mapping.backend=slamware_ros）。
    不清图/加载/发布运动，不代表导航验收通过。"""
    if _navigation_cfg().get("mapping", {}).get("backend") != "slamware_ros":
        return dict(ok=False, read_only=True, motion_started=False,
                    error="mapping.backend 不是 slamware_ros")
    try:
        return _get_ros_mapping().status()
    except (ValueError, OSError, RuntimeError, ImportError) as exc:
        return dict(ok=False, read_only=True, motion_started=False, error=str(exc))


@_tool
def navigation_preflight() -> dict:
    """显式初始化导航订阅并检查配置/定位/雷达；不申请底盘、不下发运动。
    返回结构化配置缺失项，避免把空 map_id/frame 混成泛化的导航失败。
    冷启动第一次可能因订阅热身报 stale，隔 1~2s 重试一次即可。"""
    # Only inspect an existing driver's profile; don't connect before config validation. If no
    # driver exists yet, _require_slamware gates it later.
    unsupported = _slamware_error()
    if unsupported:
        return {"ok": False, "motion_started": False, "errors": [unsupported], "error": unsupported}
    cfg = _navigation_cfg()
    config_errors = _navigation_config_diagnostics(cfg)
    if _uses_ros_mapping(cfg):
        return _ros_preflight(cfg, config_errors)
    if config_errors:
        active = _navigation.status()["active"] if _navigation is not None else False
        return {"ok": False, "motion_started": False, "active": active,
                "engine": cfg.get("engine", "onboard"), "errors": config_errors,
                "config": {"ready": False, "errors": config_errors, "frame": cfg.get("frame")},
                "lidar": _lidar_status(),
                "error": "导航预检未执行：先补齐上述配置并重新启动服务"}

    def run(nav):
        # Explicit preflight (unlike status polling) may create subscriptions; it never claims
        # the base or sends velocities/goals.
        _get_lidar_guard()
        s = nav.localization.snapshot()
        stationary = nav._stationary(s)
        result = {"ok": stationary, "engine": "onboard", "localization": s,
                  "errors": [] if stationary else list(s.get("errors", [])) or ["robot_not_stationary"],
                  "motion_started": False, "active": nav.status()["active"],
                  "config": {"ready": True, "map_id": cfg["map_id"], "frame": cfg["frame"]},
                  "lidar": _lidar_status()}
        errors = list(result["errors"])
        if result["lidar"].get("enabled") and not result["lidar"].get("ok"):
            errors.append("lidar_not_ready：等待有效雷达数据后重新预检；不得用运动触发初始化")
        if result["active"]:
            errors.append("navigation_active：先停止并确认控制权释放")
        result["ok"] = not errors
        result["errors"] = errors
        return result
    return _map_operation(run)


# ---------------------------------------------------------------------------
# Topological memory (services/topo_memory): one store per <model>/<venue>; evidence frames come
# only from this service's frame cache.
# ---------------------------------------------------------------------------
_memory = None
_memory_key = None
_memory_lock = threading.Lock()


def _memory_model() -> str:
    """Model used to select the memory store; only inspects an existing driver's profile and
    never connects hardware just for memory."""
    p = _driver_profile()
    if p is not None:
        if p.model not in ("g1d", "r1d"):
            raise ValueError(f"robot_model_unknown: {p.reason}；机型未识别时不读写记忆（记忆按机型分库）")
        return p.model
    if driver_factory.current_driver() is not None:
        return "g1d"          # driver without profile (test double): legacy G1D behaviour
    model = str((_config().get("robot") or {}).get("model") or "auto").strip().lower()
    if model in ("g1d", "r1d"):
        return model
    raise ValueError("机型尚未识别：先 observe 一次（会连接机器人并识别机型）再用记忆")


def _get_memory():
    global _memory, _memory_key
    from services.topo_memory import TopoMemory
    model, cfg = _memory_model(), _section("memory")
    venue = _venue(cfg)
    root = Path(cfg["dir"])
    if not root.is_absolute():
        root = _SRC / root
    key = (model, venue, str(root))
    with _memory_lock:
        if _memory_key != key:
            if _memory is not None:
                _memory.close()
                _memory, _memory_key = None, None
            _memory = TopoMemory(root / model / venue, model, venue)
            _memory_key = key
        return _memory


def _memory_navigable() -> bool:
    p = _driver_profile()
    return p is None or bool(p.has_slamware)


from services.topo_memory import register_memory_tools  # noqa: E402

register_memory_tools(mcp, _get_memory, _frame_jpeg, _runlog.wrap,
                      navigable=_memory_navigable,
                      check_metric_place=lambda name: _metric_place_store().get(name))


if __name__ == "__main__":
    _start_observer()          # observer surface for navi-observer (the only channel besides stdio)
    try:
        mcp.run()
    finally:
        # Best effort on graceful exit only; SIGKILL/power loss never reaches here.
        if _navigation is not None and _navigation.status()["active"]:
            _navigation.stop()
            if _navigation._thread is not None:
                _navigation._thread.join(timeout=4.)
        # Stop sequence/discrete motion; the base sends zero velocity before the publisher closes
        # (both stdio disconnect and Ctrl-C end up here).
        if _seq_exec is not None:
            try:
                _seq_exec.close()
            except Exception as exc:
                print(f"[error] 序列执行器关闭异常: {type(exc).__name__}: {exc}", file=sys.stderr)
        _d = driver_factory.current_driver()
        if _d is not None:
            try:
                _d.close()
            except Exception as exc:
                print(f"[error] driver 关闭异常: {type(exc).__name__}: {exc}", file=sys.stderr)
        if _slam_runner is not None:
            _slam_runner.stop(save=True)          # persist lidar map (also autosaved every 60 s)
