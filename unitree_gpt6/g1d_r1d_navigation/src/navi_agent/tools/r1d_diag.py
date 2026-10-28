# -*- coding: utf-8 -*-
"""r1d_diag.py -- READ-ONLY on-robot diagnostics for R1D.

**Read-only: subscribe/query only. Never creates a Publisher, never calls ServiceSwitch /
SetFsmId, never sends velocity commands.** Actions such as enabling the camera service,
checking turn sign or calibrating speed are done manually by the operator on site.

Checks:
  1. Model probe: whether the services.robots.probe subprocess matches R1D signatures on (net, domain);
  2. Feedback: rate and latest value of rt/state_vel / rt/height_state / rt/height_alarm (alarm bits decoded);
  3. Topic discovery: DDS built-in discovery lists all published topics + types and flags likely
     ranging topics (scan/range/tof/ultra/cloud/lidar);
  4. Camera services: RobotStateClient.ServiceList (query) for video_hub / stereo_patch_pc1 status;
  5. Camera stream: whether RTP packets arrive on the local UDP port (default 5002). Reported as
     unavailable if the port is taken by a running Navi server.
  6. Livox lidar: listens for status broadcasts on UDP 56201. The R1D base's Mid-360 is not on DDS,
     so checking topic names in item 3 alone would wrongly conclude there is no ranging sensor.
     Use tools/livox_diag.py for details and calibration.

Usage (on the R1D subnet, with an interpreter that has unitree_sdk2py; do not run the Navi server concurrently):
  cd src/navi_agent && python -m tools.r1d_diag --net <iface> [--domain 0]
  Optional: --listen S (subscribe/discovery duration, default 3)  --udp-port 5002  --json
Exit code: 0 = all three R1D feedback topics received; 2 = not received (wrong iface/domain or robot not powered).
"""
from __future__ import annotations

import argparse
import json
import os
import re
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from services.robots import probe  # noqa: E402

RANGE_RE = re.compile(r"scan|range|tof|ultra|sonar|cloud|lidar|laser|depth|obstacle", re.I)
ALARM_BITS = ((1, "NotHomed"), (2, "Lower"), (4, "Upper"), (8, "STALL"))
CAMERA_SERVICES = ("video_hub", "stereo_patch_pc1")


def _feedback(listen_s: float) -> dict:
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_, Twist_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import Error_

    lock = threading.Lock()
    out = {k: {"count": 0, "last": None} for k in ("state_vel", "height_state", "height_alarm")}

    def rec(key, value):
        with lock:
            out[key]["count"] += 1
            out[key]["last"] = value

    subs = []
    for topic, typ, key, conv in (
            ("rt/state_vel", Twist_, "state_vel",
             lambda m: {"linear_x": m.linear.x, "linear_z": m.linear.z, "angular_z": m.angular.z}),
            ("rt/height_state", Point32_, "height_state", lambda m: {"z": m.z}),
            ("rt/height_alarm", Error_, "height_alarm",
             lambda m: {"source": int(m.source), "state": int(m.state),
                        "bits": [n for b, n in ALARM_BITS if int(m.state) & b]})):
        s = ChannelSubscriber(topic, typ)
        s.Init(lambda m, key=key, conv=conv: rec(key, conv(m)), 1)
        subs.append(s)
    time.sleep(listen_s)
    with lock:
        for v in out.values():
            v["hz"] = round(v["count"] / listen_s, 1)
        return json.loads(json.dumps(out))


def _topics(listen_s: float) -> dict:
    try:
        from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
        from unitree_sdk2py.core.channel import ChannelFactory
        participant = ChannelFactory._ChannelFactory__participant
        reader = BuiltinDataReader(participant, BuiltinTopicDcpsPublication)
        time.sleep(listen_s)
        seen = {}
        for s in reader.take(N=10000):
            name = getattr(s, "topic_name", None)
            if name:
                seen[name] = getattr(s, "type_name", "")
    except Exception as exc:
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
    topics = [{"topic": t, "type": seen[t]} for t in sorted(seen)]
    return {"ok": True, "count": len(topics), "topics": topics,
            "range_candidates": [x for x in topics if RANGE_RE.search(x["topic"] + " " + x["type"])]}


def _services(timeout_s: float = 3.0) -> dict:
    try:
        from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
        client = RobotStateClient()
        client.SetTimeout(timeout_s)
        client.Init()
        code, services = client.ServiceList()
    except Exception as exc:
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
    if code != 0:
        return {"ok": False, "error": f"ServiceList code={code}"}
    items = [{"name": s.name, "status": s.status, "protect": s.protect} for s in services or []]
    cam = {n: next((i for i in items if i["name"] == n), None) for n in CAMERA_SERVICES}
    return {"ok": True, "services": items, "camera_services": cam}


def _udp(port: int, listen_s: float) -> dict:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", port))
    except OSError as exc:
        sock.close()
        return {"ok": False, "error": f"UDP {port} 被占用（Navi server 或其它接收器在跑？）: {exc}"}
    sock.settimeout(0.2)
    n, nbytes, senders = 0, 0, set()
    end = time.monotonic() + listen_s
    try:
        while time.monotonic() < end:
            try:
                data, addr = sock.recvfrom(65536)
            except TimeoutError:
                continue
            n += 1
            nbytes += len(data)
            senders.add(addr[0])
    finally:
        sock.close()
    return {"ok": n > 0, "port": port, "packets": n, "bytes": nbytes, "senders": sorted(senders)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="R1D read-only diagnostics (subscribe/query only, sends no commands)")
    ap.add_argument("--net", required=True)
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--listen", type=float, default=3.0)
    ap.add_argument("--udp-port", type=int, default=5002)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)

    report = {"net": a.net, "domain_id": a.domain}
    obs = probe.run_probe(a.net, a.domain, a.listen)
    report["probe"] = {"observation": obs, "r1d": probe.matches("r1d", obs), "g1d": probe.matches("g1d", obs)}

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    ChannelFactoryInitialize(a.domain, a.net)
    report["feedback"] = _feedback(a.listen)
    report["topics"] = _topics(a.listen)
    report["services"] = _services()
    report["camera_udp"] = _udp(a.udp_port, a.listen)
    from tools.livox_diag import listen_push
    report["livox"] = listen_push(min(a.listen, 3.0))
    ok = all(v["count"] > 0 for v in report["feedback"].values())

    if a.json:
        print(json.dumps(report, ensure_ascii=False, indent=1))
    else:
        p = report["probe"]
        print(f"== 探测 {a.net}/domain {a.domain}: r1d={p['r1d']} g1d={p['g1d']}  {obs}")
        print("== 反馈")
        for k, v in report["feedback"].items():
            print(f"   {k:13s} {v['hz']:5.1f} Hz  last={v['last']}")
        t = report["topics"]
        if t["ok"]:
            print(f"== 话题 {t['count']} 个；疑似测距：")
            for x in t["range_candidates"] or [{"topic": "（无）", "type": ""}]:
                print(f"   {x['topic']}  {x['type']}")
            print("   全部话题见 --json")
        else:
            print(f"== 话题发现失败：{t['error']}")
        s = report["services"]
        print("== 相机服务: " + (json.dumps(s["camera_services"], ensure_ascii=False) if s["ok"] else s["error"]))
        u = report["camera_udp"]
        print(f"== 相机 UDP {a.udp_port}: " + (f"{u['packets']} 包 来自 {u['senders']}" if u.get("port") else u["error"]))
        lx = report["livox"]
        print("== Livox 雷达（UDP 广播，不在 DDS 上）: " + (
            f"{lx.get('product_info', '?')} @ {lx['from']}，工作状态 {lx.get('cur_work_state')}，"
            f"点云发往 {(lx.get('pointcloud_host') or {}).get('ip', '?')}（详查: python -m tools.livox_diag）"
            if lx else "未收到状态广播"))
        print("结论: " + ("R1D 反馈三路齐全" if ok else "未收到完整 R1D 反馈，检查网卡/domain/上电"))
    sys.stdout.flush()
    os._exit(0 if ok else 2)


if __name__ == "__main__":
    main()
