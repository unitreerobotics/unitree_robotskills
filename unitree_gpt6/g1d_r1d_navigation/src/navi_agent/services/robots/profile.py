# -*- coding: utf-8 -*-
"""profile.py -- robot capability description (RobotProfile) with built-in g1d / r1d defaults.

Upper layers (driver / navi_mcp_server) use only the profile to decide speed limits, lidar
subscription and SLAMWARE tool availability; all chassis differences live in
services/robots/<model>.py. config.yaml robot.<model> can override individual fields.

has_lidar is tri-state: True = range data verified; False = confirmed absent; None = unverified,
degraded as no lidar.
lidar_kind: dds_scan = LaserScan topic on DDS (G1D SLAMWARE); livox_mid360 = Livox UDP point
cloud (front-mounted Mid-360 on R1D, converted to a virtual scan by services/livox_mid360.py).
"""
from __future__ import annotations

from dataclasses import dataclass, field, fields, replace

# G1D hardware limits (same as services/g1_agv_client.py, which imports the SDK at top level,
# so it cannot be imported here)
AGV_MAX_VX = 1.5
AGV_MAX_VYAW = 0.6


@dataclass(frozen=True)
class RobotProfile:
    model: str                     # g1d / r1d / unknown
    max_vx: float                  # hard linear speed cap m/s (min with navi.sequence / motion config)
    max_vyaw: float                # hard angular speed cap rad/s
    rate_hz: float                 # velocity command publish rate
    has_lidar: bool | None         # see module docstring
    lidar_topic: str               # LaserScan topic subscribed when has_lidar=True
    has_slamware: bool             # on-board SLAMWARE (navigate_to / mapping_* / navigation_*)
    has_lift: bool                 # lift column (never exposed in phase 1)
    camera_kind: str               # zmq (G1D) / r1d_rtp (R1D, services/rtp_camera.py)
    net: str = "auto"              # DDS NIC; auto = config.NET
    domain_id: int = 0
    camera_crop_ratio: float = 1.0  # <1: also give the model a center crop (side ratio) per frame; 1.0 = full frame only
    can_move: bool = True          # False = unrecognised/ambiguous; all motion tools refuse
    reason: str = ""               # detection result (refusal reason when unknown)
    lidar_kind: str = "dds_scan"   # dds_scan / livox_mid360, see module docstring
    extra: dict = field(default_factory=dict, compare=False)   # model-specific params (r1d calibration etc.)

    @property
    def lidar_ready(self) -> bool:
        if self.has_lidar is not True:
            return False
        return bool(self.lidar_topic) if self.lidar_kind == "dds_scan" else True

    @property
    def localization(self) -> str | None:
        """Localization source: slamware (on-board) / lidar_slam (built here from Livox point
        clouds, services/lidar_slam.py) / None."""
        if self.has_slamware:
            return "slamware"
        if self.lidar_ready and self.lidar_kind == "livox_mid360" and self.extra.get("slam"):
            return "lidar_slam"
        return None

    def capabilities(self) -> dict:
        return {"move": self.can_move, "lidar": self.has_lidar, "slamware": self.has_slamware,
                "localization": self.localization,
                "lift": self.has_lift, "camera": self.camera_kind,
                "max_vx": self.max_vx, "max_vyaw": self.max_vyaw}

    def summary(self) -> dict:
        return {"model": self.model, "net": self.net, "domain_id": self.domain_id,
                "reason": self.reason, "capabilities": self.capabilities()}


# G1D linear speed defaults to 0.5 (the former navi.sequence.max_vx default, unchanged behaviour);
# robot.g1d.max_vx may raise it up to the 1.5 hardware limit
G1D = RobotProfile(
    model="g1d", max_vx=0.5, max_vyaw=AGV_MAX_VYAW, rate_hz=20.0,
    has_lidar=True, lidar_topic="rt/slamware_ros_sdk_server_node/scan",
    has_slamware=True, has_lift=False, camera_kind="zmq")

# R1D: normalised full scale mps_full=1.0 / rps_full=1.2 (medium gear);
# linear cap 1.0 m/s (field-confirmed: 0.2 is a safe cruise speed, open corridors may use 1 m/s;
# actual speed is set by the lidar speed envelope from path clearance, see lidar_headway_s);
# turning stays at 0.4 rad/s (turns are not protected by the reflex layer). mps_full is not
# verified by measurement, so actual linear speed may differ from the command.
# turn_sign=-1: on R1D negative turn = left, Navi positive vyaw = left -> inverted. Both still
# to be verified on the robot.
# Lidar: front-mounted Livox Mid-360 (internal 192.168.123.120, UDP, not on DDS). lidar_* are
# measured mounting extrinsics; re-measure with tools/livox_diag.py (prints new values) after
# replacing or remounting the lidar.
R1D = RobotProfile(
    model="r1d", max_vx=1.0, max_vyaw=0.4, rate_hz=50.0,
    has_lidar=True, lidar_topic="", lidar_kind="livox_mid360",
    has_slamware=False, has_lift=False, camera_kind="r1d_rtp",
    # The 544x448 left image is a stereo-rectified pinhole image (straight lines stay straight,
    # not raw fisheye), but its FOV is ~105 deg, so central objects look small and far. A center
    # 50% crop upscaled to full size gives a "normal FOV" view (~66 deg, close to a phone main camera).
    camera_crop_ratio=0.5,
    extra={"mps_full": 1.0, "rps_full": 1.2, "turn_sign": -1.0,
           "feedback_timeout_s": 0.30,
           # Left-eye horizontal FOV. Rectified intrinsics are unpublished; estimated (~+/-10 deg)
           # from a plumb-line check (no measurable distortion) and the 146 deg fisheye focal
           # length scaled to 544 px width (~208 px). Only used to describe the FOV to the model.
           "camera_hfov_deg": 105.0,
           "max_accel_mps2": 0.6,                    # linear accel cap for sequences (heavy upper body; avoid harsh starts)
           # Heading hold (core/heading_hold.py): corrects the turn component with the gyro of the
           # lidar's built-in IMU (UDP 56401). Wheels don't start aligned forward: measured ~10 deg
           # heading drift over 1 m straight at 0.5 m/s. false = pure open loop.
           "heading_hold": True, "heading_kp": 1.5, "heading_max_corr_rps": 0.25, "imu_port": 56401,
           "lidar_ip": "192.168.123.120",
           "lidar_host_ip": "",                      # local address for point clouds; empty = address of the NIC facing the lidar
           "lidar_auto_host": True,                  # on start/stream loss, point the lidar's destination at this host (standard Livox driver practice)
           "lidar_down": (0.505, -0.008, -0.863),    # unit vector toward the ground in lidar frame (pitch down 30.4 deg, roll -0.55 deg)
           "lidar_height_m": 0.205,                  # lidar height above ground
           "lidar_yaw_deg": 0.0,                     # horizontal yaw of lidar x-axis relative to robot forward
           "lidar_front_offset_m": 0.27,             # front edge distance ahead of the lidar (sloped fairing measured at 0.25-0.28)
           "lidar_self_box": (-0.90, 0.03, 0.40),    # body box (x_min, x_max, y_half), origin at the front edge
           "lidar_z_min_m": 0.05, "lidar_z_max_m": 1.5,   # height band above ground counted as obstacle
           "lidar_corridor_half_width_m": 0.35,      # travel corridor half-width (used when navi.lidar.corridor_half_width_m is 0)
           # Lidar speed envelope (used when navi.lidar.headway_s / creep_mps are 0): allowed speed =
           # max(creep, (path clearance - stop_m) / headway). With 1.5 s, 1 m/s needs ~2 m clear,
           # 0.5 m/s ~1.2 m. Near obstacles it creeps at 0.2 m/s until stop_m (field: 0.2 is safe,
           # slower is unnecessary).
           "lidar_headway_s": 1.5, "lidar_creep_mps": 0.2,
           "lidar_require": True,                    # block forward motion without lidar data (rear is a blind spot; reversing not gated)
           "camera_gate_without_lidar": True,        # with lidar disabled, fall back to "translate only while camera frames are fresh"
           # Lidar localization and waypoints (services/lidar_slam.py): map and points stored in
           # src/<slam_dir>/<navi.memory.venue>/
           "slam": True,
           "slam_dir": "data/lidar_map",
           "camera_host": "0.0.0.0", "camera_port": 5002,    # left-eye RTP-H264 pushed to local UDP 5002
           # Right eye from the same service on UDP 5003 (544x448 rectified; 5001 is a separate
           # 720x1280 portrait wide-angle). Shown side by side on the camera page only; the model sees the left eye only.
           "camera_right_port": 5003,
           # When stopping a turn on gyro-measured angle, cut the command this many seconds of
           # turn rate early (compensates post-stop coasting); initial value, to be measured with tools/heading_check.py
           "turn_stop_lead_s": 0.1,
           # If 5002 has no video, stop video_hub and start stereo_patch_pc1 (services/r1d_camera_services.py)
           "camera_auto_switch": True, "camera_probe_s": 3.0,
           # ServiceSwitch measured ~10 s server-side; timeout 20 s (at 5 s the client always reports 3104)
           "camera_service_timeout_s": 20.0, "camera_startup_s": 15.0})

BUILTIN = {"g1d": G1D, "r1d": R1D}


def unknown(reason: str) -> RobotProfile:
    """Fail-closed profile for failed detection: no driving, no lidar subscription, no SLAMWARE tools."""
    return RobotProfile(model="unknown", max_vx=0.0, max_vyaw=0.0, rate_hz=20.0,
                        has_lidar=False, lidar_topic="", has_slamware=False, has_lift=False,
                        camera_kind="none", can_move=False, reason=reason)


_OVERRIDABLE = {f.name for f in fields(RobotProfile)} - {"model", "can_move", "reason", "extra"}


def build_profile(model: str, overrides: dict | None = None, **kw) -> RobotProfile:
    """Built-in profile + config.yaml robot.<model> overrides. Keys of base.extra go into extra
    (model-specific calibration); other unknown keys are rejected."""
    base = BUILTIN.get(model)
    if base is None:
        raise ValueError(f"未知机型 {model!r}（可选 {sorted(BUILTIN)}）")
    top, extra = {}, dict(base.extra)
    for k, v in (overrides or {}).items():
        if k in _OVERRIDABLE:
            top[k] = v
        elif k in base.extra:
            extra[k] = v
        else:
            raise ValueError(f"robot.{model}.{k} 不是可配置项")
    for k in ("max_vx", "max_vyaw", "rate_hz"):
        if k in top and not float(top[k]) > 0:
            raise ValueError(f"robot.{model}.{k} 须为正数")
    if "camera_crop_ratio" in top and not 0 < float(top["camera_crop_ratio"]) <= 1:
        raise ValueError(f"robot.{model}.camera_crop_ratio 须在 (0, 1] 范围内")
    # config can never raise G1D hardware limits
    if model == "g1d":
        top["max_vx"] = min(float(top.get("max_vx", base.max_vx)), AGV_MAX_VX)
        top["max_vyaw"] = min(float(top.get("max_vyaw", base.max_vyaw)), AGV_MAX_VYAW)
    return replace(base, **top, **kw, extra=extra)
