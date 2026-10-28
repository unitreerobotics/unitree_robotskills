# -*- coding: utf-8 -*-
"""driver.py -- HeadlessDriver: minimal facade over robot control and camera stream.

- connect() / close(): robot command publisher thread + camera stream
- move() / turn() / stop(): low-level synchronous motion (blocks until result)
- status(): connection and command-integrated odometry snapshot

Logs go to stderr (stdout is the MCP stdio channel).
"""

import math
import sys
import time

import config
from config import apply_config, load_config
from core.motion import MotionExecutor
from services import camera as camera_mod
from services.camera import CameraStream, request_camera_config
from services.robots import build_robot
from services.rtp_camera import RtpCameraStream

# Direction name -> normalized (vx, vy), vx positive forward.
# Wheeled chassis cannot strafe (vy always 0); only forward/backward, use turn for heading.
DIRECTIONS = {
    "forward":  (1.0, 0.0),
    "backward": (-1.0, 0.0),
}


class HeadlessDriver:
    """Combines robot controller, discrete motion executor and camera behind a blocking API."""

    def __init__(self, cfg=None, camera_cfg=None, robot=None, motion=None, log=None):
        """
        Args:
            cfg: merged config dict; defaults to load_config() (apply_config keeps config globals in sync)
            camera_cfg: camera config {host, request_port, eye, ...}; None = no camera
            robot: injected robot controller (tests); defaults to services.robots.build_robot (per robot.model)
            motion: injected MotionExecutor; defaults to MotionExecutor(self.robot)
            log: log callback (msg, kind); defaults to stderr
        """
        self.raw_cfg = cfg if cfg is not None else load_config()
        apply_config(self.raw_cfg)               # idempotent: keep config runtime globals consistent with raw_cfg
        self.camera_cfg = camera_cfg
        self._log_fn = log if log is not None else self._default_log
        if robot is None:
            self.profile, robot = build_robot(self.raw_cfg)
        else:
            self.profile = getattr(robot, "profile", None)
        self.robot = robot
        self.motion = motion if motion is not None else MotionExecutor(self.robot, log=self._log)
        lead = (getattr(self.profile, "extra", None) or {}).get("turn_stop_lead_s")
        if lead is not None and motion is None:
            self.motion.turn_stop_lead_s = float(lead)     # gyro turn-stop lead time (per-model calibration, measured with tools/heading_check.py)
        self.camera = None

    def _default_log(self, msg, kind="info"):
        print(f"[{kind}] {msg}", file=sys.stderr)

    def _log(self, msg, kind="info"):
        self._log_fn(msg, kind)

    def connect(self, wait_robot_data=False) -> bool:
        """Connect to the robot (start the command publisher thread). Returns success."""
        try:
            if getattr(self.robot, "start_publisher", None):
                self.robot.start_publisher()
            if wait_robot_data and getattr(self.robot, "wait_for_data", None):
                self.robot.wait_for_data()
            self._log("机器人已连接", "info")
            return True
        except Exception as e:
            self._log(f"机器人连接失败: {type(e).__name__}: {e}", "error")
            return False

    def connect_camera(self, camera_cfg=None, timeout_s=10.0) -> bool:
        """Connect the camera stream (blocks until first frame). Returns success."""
        if camera_cfg is not None:
            self.camera_cfg = camera_cfg
        if not self.camera_cfg:
            self._log("未配置相机，跳过", "camera")
            return False
        cfg = self.camera_cfg
        if getattr(self.profile, "camera_kind", None) == "r1d_rtp":
            return self._connect_rtp_camera(cfg, timeout_s)
        host = str(cfg.get("host", "192.168.123.164"))
        req_port = int(cfg.get("request_port", 60000))
        if camera_mod.zmq is None:
            self._log("相机协议不可用（缺少 pyzmq）", "camera")
            return False
        conf = request_camera_config(host, req_port)
        if conf is None:
            self._log(f"请求相机配置失败（{host}:{req_port}）", "camera")
            return False
        # Server returns config nested by camera name {head_camera: {zmq_port, binocular, ...}, ...};
        # also accept top-level camera_port/port from servers with a flat layout.
        cam_name = cfg.get("camera", "head_camera")
        cam_entry = (conf or {}).get(cam_name) or {}
        port = int(cfg.get("zmq_port") or cam_entry.get("zmq_port")
                   or conf.get("camera_port") or conf.get("port") or 0)
        if not port:
            self._log("相机配置缺少端口", "camera")
            return False
        eye = cfg.get("eye", "left")
        binocular = cfg.get("binocular")
        if binocular is None:
            binocular = bool(cam_entry.get("binocular", conf.get("binocular", True)))
        cam = CameraStream(host, port, bool(binocular),
                           first_frame_timeout_s=float(cfg.get("first_frame_timeout_s", 5.0)),
                           max_frame_age_s=float(cfg.get("max_frame_age_s", 1.0)))
        self.camera = cam
        cam.start()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if cam.connected:
                self._log(f"相机已连接（{host}:{port}，眼={eye}）", "camera")
                return True
            if cam.error:
                self._log(f"相机连接失败: {cam.error}", "camera")
                self.camera = None
                return False
            time.sleep(0.1)
        self._log(f"相机连接超时（{timeout_s:.0f}s）", "camera")
        self.camera = None
        return False

    def _wait_rtp_camera(self, cfg, timeout_s):
        """Start an RTP receiver and wait for the first frame; returns (cam, None) or stops it and returns (None, reason)."""
        extra = self.profile.extra
        cam = RtpCameraStream(str(extra.get("camera_host", "0.0.0.0")), int(extra.get("camera_port", 5002)),
                              first_frame_timeout_s=float(timeout_s),
                              max_frame_age_s=float(cfg.get("max_frame_age_s", 1.0)))
        cam.start()
        deadline = time.monotonic() + timeout_s + 1.0
        while time.monotonic() < deadline:
            if cam.connected:
                return cam, None
            if cam.error:
                break
            time.sleep(0.1)
        cam.stop()
        return None, cam.error or "超时"

    def _connect_rtp_camera(self, cfg, timeout_s) -> bool:
        """R1D: receive left-eye RTP-H264 on local UDP 5002.

        First listen passively for camera_probe_s: if frames arrive (services already switched),
        leave robot services alone. Otherwise, with camera_auto_switch=True, stop video_hub,
        start stereo_patch_pc1, and wait for the first frame again.
        """
        extra = self.profile.extra
        auto = bool(extra.get("camera_auto_switch", True))
        probe_s = float(extra.get("camera_probe_s", 3.0)) if auto else float(timeout_s)
        cam, err = self._wait_rtp_camera(cfg, probe_s)
        if cam is None and auto:
            self._log(f"R1D 相机 {probe_s:.0f}s 内无画面（{err}），自动切换相机服务", "camera")
            from services.r1d_camera_services import switch_camera_services
            res = switch_camera_services(float(extra.get("camera_service_timeout_s", 20.0)), log=self._log)
            if res["error"]:
                self._log(f"R1D 相机服务切换：{res['error']}（继续等画面，以收到帧为准）", "camera")
            cam, err = self._wait_rtp_camera(cfg, float(extra.get("camera_startup_s", 15.0)))
        if cam is not None:
            self.camera = cam
            self._log(f"R1D 相机已连接（UDP {cam.host}:{cam.port}）", "camera")
            return True
        self._log(f"R1D 相机连接失败: {err}", "camera")
        self.camera = None
        return False

    def close(self):
        """Stop motion and shut down the camera and publisher thread."""
        self.stop()
        if self.camera is not None:
            try:
                self.camera.stop()
            except Exception as e:
                self._log(f"相机停止异常: {type(e).__name__}: {e}", "error")
            self.camera = None
        if getattr(self.robot, "stop_publisher", None):
            try:
                self.robot.stop_publisher()
            except Exception as e:
                self._log(f"发布线程停止异常: {type(e).__name__}: {e}", "error")

    def move(self, direction, distance_m=None, speed=None, timeout_s=30.0) -> dict:
        """Drive straight distance_m meters (blocking).
        direction in forward/backward (no strafing; use turn). Returns {"status", "moved_m"}."""
        vec = DIRECTIONS.get(direction)
        if vec is None:
            raise ValueError(f"未知方向: {direction}（可选 {list(DIRECTIONS)}）")
        if self.motion is None:
            return {"status": "no_motion", "moved_m": 0.0}
        vx, vy = vec
        speed = self._profile_cap(config.MOVE_SPEED if speed is None else speed, "max_vx")
        return self.motion.move_linear_sync(vx, vy, speed=speed,
                                            distance=distance_m, timeout_s=timeout_s)

    def _profile_cap(self, value, key):
        """Cap by model profile so duration is computed from the clamped speed and odometry is not skewed by chassis-side clamping."""
        cap = getattr(getattr(self, "profile", None), key, None)
        return min(float(value), float(cap)) if cap is not None else value

    def turn(self, angle_deg, turn_speed=None, timeout_s=30.0) -> dict:
        """Turn in place angle_deg degrees (+left), blocking. Returns {"status", "turned_rad"}."""
        if self.motion is None:
            return {"status": "no_motion", "turned_rad": 0.0}
        turn_speed = self._profile_cap(config.TURN_ANGLE if turn_speed is None else turn_speed,
                                       "max_vyaw")
        return self.motion.turn_angle_sync(math.radians(angle_deg),
                                           turn_speed=turn_speed, timeout_s=timeout_s)

    def stop(self):
        """Stop the current discrete motion (idempotent)."""
        if self.motion is not None:
            try:
                self.motion.stop()
            except Exception as e:
                self._log(f"运动停止异常: {type(e).__name__}: {e}", "error")

    def status(self) -> dict:
        """Status snapshot for get_status. position/yaw are command-integrated estimates, not measured localization."""
        odom = getattr(self.robot, "odom", None) or {}
        pos = odom.get("position")
        yaw = odom.get("yaw")
        out = {
            "robot_connected": bool(getattr(self.robot, "connected", False)),
            "camera_connected": bool(self.camera is not None and self.camera.connected),
            "position": list(pos) if isinstance(pos, (list, tuple)) else None,
            "yaw": float(yaw) if yaw is not None else None,
        }
        profile = getattr(self, "profile", None)
        if profile is not None:
            out["robot"] = profile.summary()
            health = getattr(self.robot, "health", None)
            if callable(health):
                out["robot"]["health"] = health()
        return out
