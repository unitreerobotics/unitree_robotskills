# -*- coding: utf-8 -*-
"""lidar_slam_replay.py -- offline replay of lidar localization (read-only, no robot connection).

Input: raw point clouds recorded by the Navi server with NAVI_LIDAR_RECORD=1
(<repo>/logs/lidar-*.bin.gz). Packets are batched every 0.1 s by their recorded receive time
and fed to the same SlamRunner used online. Output:
  - per-second pose, state, match quality and per-frame processing time;
  - a map PNG (white=free, black=wall, blue=trajectory, red=end; up=+y);
  - loop-closure check: if the recording starts somewhere, drives a loop and returns to the
    start, the final pose's distance from the origin is the accumulated drift.

Usage:
  cd src/navi_agent && python -m tools.lidar_slam_replay ../../logs/lidar-<YYYYmmdd-HHMMSS>.bin.gz [--out map.png]
        [--set min_quality=0.5 ...] [--speed 0]   # speed 0 = as fast as possible; 1 = recorded pace (real-time check)
"""
from __future__ import annotations

import argparse
import gzip
import math
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2  # noqa: E402

from services import lidar_slam as ls  # noqa: E402
from services import livox_mid360 as lv  # noqa: E402
from services.robots.profile import build_profile  # noqa: E402


def read_records(path):
    """Yield (receive_time, packet) pairs. Record format: see LivoxFeeder._write_record."""
    with gzip.open(path, "rb") as f:
        while True:
            head = f.read(10)
            if len(head) < 10:
                return
            t, n = struct.unpack("<dH", head)
            data = f.read(n)
            if len(data) < n:
                return
            yield t, data


def batches(records, period_s=0.1):
    batch, t_end = [], None
    for t, pkt in records:
        if t_end is None:
            t_end = t + period_s
        if t >= t_end:
            yield t_end, batch
            batch, t_end = [], t_end + period_s * max(1, int((t - t_end) / period_s) + 1)
        batch.append(pkt)
    if batch:
        yield t_end, batch


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Offline replay of lidar localization")
    ap.add_argument("record")
    ap.add_argument("--out", default="lidar_slam_replay.png")
    ap.add_argument("--set", nargs="*", default=[], help="LidarSlam parameter overrides, e.g. min_quality=0.5")
    ap.add_argument("--speed", type=float, default=0.0)
    args = ap.parse_args(argv)

    x = build_profile("r1d").extra
    mount = lv.Mount(x["lidar_down"], x["lidar_height_m"], x["lidar_yaw_deg"], x["lidar_front_offset_m"])
    kw = {k: float(v) for k, v in (it.split("=", 1) for it in args.set)}
    slam = ls.LidarSlam(**kw)
    runner = ls.SlamRunner(slam, mount, struct_kw={"self_box": tuple(x["lidar_self_box"])})
    t0 = wall0 = None
    states, ms, last_print = {}, [], -1
    for t_end, batch in batches(read_records(args.record)):
        if t0 is None:
            t0, wall0 = t_end, time.monotonic()
        if args.speed > 0:
            time.sleep(max(0.0, (t_end - t0) / args.speed - (time.monotonic() - wall0)))
        st = runner.step(batch)
        states[st] = states.get(st, 0) + 1
        if runner.proc_ms is not None:
            ms.append(runner.proc_ms)
        sec = int(t_end - t0)
        if sec != last_print:
            last_print = sec
            p = slam.pose
            print("%4ds  %-12s q=%-5s  pose (%.2f, %.2f, %.1f°)  %.0fms" % (
                sec, st, None if slam.quality is None else round(slam.quality, 2), p[0], p[1], math.degrees(p[2]),
                runner.proc_ms or 0))
    if not ms:
        print("录制里没有可用的点云包")
        return 2
    ms.sort()
    p = slam.pose
    print("帧状态 %s  单帧耗时 中位 %.0fms / p95 %.0fms / 最大 %.0fms" % (
        states, ms[len(ms) // 2], ms[int(len(ms) * 0.95)], ms[-1]))
    print("末位姿 (%.3f, %.3f, %.1f°)  离起点 %.3fm（绕一圈回到原地时，这就是累积误差）" % (
        p[0], p[1], math.degrees(p[2]), math.hypot(p[0], p[1])))
    cv2.imwrite(args.out, slam.render(scale=2))
    print("地图 →", args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
