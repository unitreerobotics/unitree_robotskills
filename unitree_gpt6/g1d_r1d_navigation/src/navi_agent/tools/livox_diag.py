# -*- coding: utf-8 -*-
"""livox_diag.py -- diagnostics and mounting-extrinsics calibration for the R1D base's Livox Mid-360.

**Read-only by default**: listens to status broadcasts, queries config, captures point clouds
and fits the floor without changing any lidar setting. Only an explicit --set-host writes to
the lidar (changes the point-cloud destination IP; persisted on the lidar).

Checks:
  1. Status broadcast (UDP 56201): model, serial number, firmware, work state;
  2. Command query: point-cloud / IMU destination address -- point clouds go to this single
     address only; if it is not this host, nothing is received;
  3. Point cloud (UDP 56301): packet rate, packet loss, valid point count;
  4. Floor fit -> mounting extrinsics (lidar_down / lidar_height_m), compared with the current profile;
  5. A virtual scan with the current extrinsics, printing clearance per direction (should match reality).

Usage (a running Navi server holds port 56301 and items 3-5 will fail; stop the server first):
  cd src/navi_agent && python -m tools.livox_diag [--lidar-ip 192.168.123.120] [--seconds 1.5]
  python -m tools.livox_diag --set-host 192.168.123.164     # send point clouds to this host
  python -m tools.livox_diag --set-host 192.168.123.12      # restore the original value
During calibration the robot must stand on flat open ground (floor 0.5-3 m ahead of the nose).
If the printed lidar_down / lidar_height_m differ from the profile, write them to the robot.r1d
section of config.yaml. For lidar_front_offset_m (how far the nose's front edge is ahead of the
lidar), read the printed distance the nose deflector extends ahead of the lidar and use a
slightly larger value.
"""
from __future__ import annotations

import argparse
import os
import socket
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import load_config  # noqa: E402
from services import livox_mid360 as lv  # noqa: E402
from services.lidar import scan_summary  # noqa: E402
from services.robots.profile import build_profile  # noqa: E402

PUSH_KEYS = {0x8000: "sn", 0x8001: "product_info", 0x8002: "version_app", 0x8006: "cur_work_state"}
WORK_STATES = {1: "SAMPLING（正在出点云）", 2: "IDLE（待机）", 4: "ERROR", 5: "SELFCHECK",
               6: "MOTOR_STARTUP", 7: "MOTOR_STOP", 8: "UPGRADE", 9: "READY"}


def listen_push(timeout_s: float = 3.0) -> dict | None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("0.0.0.0", lv.HOST_PUSH_PORT))
        s.settimeout(timeout_s)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            raw, addr = s.recvfrom(4096)
            try:
                f = lv.parse_frame(raw)
            except ValueError:
                continue
            if f["cmd_id"] != lv.CMD_PUSH:
                continue
            n = int.from_bytes(f["data"][:2], "little")
            kv = lv.decode_key_values(f["data"], 4, n)
            out = {"from": addr[0]}
            for key, name in PUSH_KEYS.items():
                if key in kv:
                    v = kv[key]
                    out[name] = (v[0] if name == "cur_work_state" else
                                 ".".join(str(b) for b in v) if name == "version_app" else
                                 v.split(b"\x00")[0].decode("ascii", "replace"))
            for key, name in ((lv.KEY_POINT_HOST, "pointcloud_host"), (lv.KEY_IMU_HOST, "imu_host")):
                if key in kv:
                    out[name] = lv.decode_host_cfg(kv[key])
            return out
    except (TimeoutError, OSError):
        return None
    finally:
        s.close()
    return None


def capture(seconds: float) -> tuple[list, str | None]:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    try:
        s.bind(("0.0.0.0", lv.HOST_POINT_PORT))
    except OSError as exc:
        s.close()
        return [], f"端口 {lv.HOST_POINT_PORT} 绑定失败（Navi server 在跑？）: {exc}"
    s.settimeout(0.5)
    packets, t0 = [], time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            try:
                packets.append(s.recv(2048))
            except TimeoutError:
                pass
    finally:
        s.close()
    return packets, None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Livox Mid-360 diagnostics and extrinsics calibration (read-only by default)")
    ap.add_argument("--lidar-ip", default="192.168.123.120")
    ap.add_argument("--seconds", type=float, default=1.5, help="point-cloud capture duration (s)")
    ap.add_argument("--set-host", metavar="IP", help="change the point-cloud destination to this IP (writes to the lidar, persistent)")
    args = ap.parse_args(argv)

    print("== 状态广播")
    push = listen_push()
    if push is None:
        print(f"   3s 内没有收到 Livox 状态广播（UDP {lv.HOST_PUSH_PORT}）：雷达没上电 / 不在这个网段")
    else:
        state = push.get("cur_work_state")
        print(f"   来自 {push['from']}  {push.get('product_info', '?')}")
        print(f"   序列号 {push.get('sn', '?')}  固件 {push.get('version_app', '?')}  "
              f"工作状态 {state} {WORK_STATES.get(state, '')}")

    host = lv.local_ip_towards(args.lidar_ip)
    print(f"== 目的地址（本机朝向雷达的地址是 {host}）")
    cmd = lv.LivoxCommander(args.lidar_ip, host)
    try:
        kv = cmd.get([lv.KEY_POINT_HOST, lv.KEY_IMU_HOST])
        cur = lv.decode_host_cfg(kv[lv.KEY_POINT_HOST])
        print(f"   点云 → {cur['ip']}:{cur['host_port']}   IMU → {lv.decode_host_cfg(kv[lv.KEY_IMU_HOST])['ip']}")
        if args.set_host:
            if cur["ip"] == args.set_host:
                print(f"   点云目的地址已经是 {args.set_host}，不用改")
            else:
                cmd.set({lv.KEY_POINT_HOST: lv.with_host_ip(kv[lv.KEY_POINT_HOST], args.set_host)})
                now = lv.decode_host_cfg(cmd.get([lv.KEY_POINT_HOST])[lv.KEY_POINT_HOST])
                print(f"   已写入：点云 {cur['ip']} → {now['ip']}（改回用 --set-host {cur['ip']}）")
                cur = now
        if cur["ip"] != host:
            print(f"   ⚠ 点云没有发到本机：本机收不到。Navi server 启动时会自动改（lidar_auto_host），"
                  f"或手动 --set-host {host}")
    except Exception as exc:
        print(f"   命令通道失败：{type(exc).__name__}: {exc}")
        return 2

    print(f"== 点云（采 {args.seconds:.1f}s）")
    packets, err = capture(args.seconds)
    if err:
        print("   " + err)
        return 2
    xyz, lost = lv.parse_point_packets(packets)
    print(f"   {len(packets)} 包（{len(packets) / args.seconds:.0f}/s，正常约 2084/s），丢 {lost} 包，有效点 {len(xyz)}")
    if len(xyz) < 1000:
        print("   点太少：雷达没在采样，或点云没发到本机")
        return 2

    x = build_profile("r1d", ((load_config().get("robot") or {}).get("r1d")) or None).extra   # includes config.yaml overrides
    print("== 地面拟合 → 安装外参")
    fit = lv.fit_floor(xyz)
    if fit is None:
        print("   没找到离雷达 1m 以内的大平面：车头前要有一片平整空地")
    else:
        down, height, inliers = fit
        cfg_down = np.asarray(x["lidar_down"], dtype=float) / np.linalg.norm(x["lidar_down"])
        tilt_err = float(np.degrees(np.arccos(np.clip(float(down @ cfg_down), -1, 1))))
        print(f"   实测  lidar_down: [{down[0]:.3f}, {down[1]:.3f}, {down[2]:.3f}]   lidar_height_m: {height:.3f}"
              f"   （地面内点 {inliers}，俯角 {np.degrees(np.arcsin(down[0])):.1f}°）")
        print(f"   当前  lidar_down: {list(x['lidar_down'])}   lidar_height_m: {x['lidar_height_m']}")
        ok = tilt_err < 1.0 and abs(height - x["lidar_height_m"]) < 0.02
        print(f"   偏差  朝向 {tilt_err:.2f}°，高度 {abs(height - x['lidar_height_m']) * 100:.1f}cm"
              f"  → {'一致' if ok else '⚠ 不一致：把实测值写进 config.yaml 的 robot.r1d'}")

    print("== 净空（用当前外参；原点在车头前缘）")
    mount = lv.Mount(x["lidar_down"], x["lidar_height_m"], x["lidar_yaw_deg"], x["lidar_front_offset_m"])
    pts = mount.to_robot(xyz)
    floor = pts[(np.abs(pts[:, 2]) < 0.05) & (np.hypot(pts[:, 0], pts[:, 1]) > 0.3)]
    if len(floor):
        print(f"   地面点高度：均值 {floor[:, 2].mean() * 100:+.1f}cm，标准差 {floor[:, 2].std() * 100:.1f}cm（应在 ±1 / 2cm 内）")
    nose = pts[(pts[:, 0] > -x["lidar_front_offset_m"]) & (pts[:, 0] < 0.15) & (np.abs(pts[:, 1]) < 0.4)
               & (pts[:, 2] > 0.04) & (pts[:, 2] < 0.19)]
    if len(nose) >= 20:                                   # 99th percentile: ignore stray outliers
        tip = float(np.percentile(nose[:, 0], 99)) + x["lidar_front_offset_m"]
        print(f"   车头导流面（离地 >4cm 的自遮挡点）伸到雷达前 {tip:.2f}m"
              f"（lidar_front_offset_m 现为 {x['lidar_front_offset_m']}，应不小于它）")
    a, inc, ranges = lv.virtual_scan(pts, z_min_m=x["lidar_z_min_m"], z_max_m=x["lidar_z_max_m"],
                                     self_box=tuple(x["lidar_self_box"]))
    s = scan_summary({"angle_min": a, "angle_inc": inc, "ranges": ranges}, 0.0, x["lidar_corridor_half_width_m"])
    fmt = lambda v: "无数据" if v is None else f"{v:.2f}m"   # noqa: E731
    print(f"   正前扇区 {fmt(s['front_m'])}   行进走廊 {fmt(s['corridor_front_m'])}   "
          f"左 {fmt(s['left_m'])}   右 {fmt(s['right_m'])}   后 {fmt(s['rear_m'])}（后方只看得到高处）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
