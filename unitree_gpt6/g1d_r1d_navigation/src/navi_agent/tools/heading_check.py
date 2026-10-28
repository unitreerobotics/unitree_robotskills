# -*- coding: utf-8 -*-
"""heading_check.py -- on-site A/B test of R1D heading closed loop: run the same robot "open loop"
and "closed loop", using lidar localization as ground truth.

What is measured:
  - In-place turns: ±90° twice each plus a 180° U-turn after each straight leg; compares requested
    angle / command integral / gyro measurement / lidar-localization measurement;
  - Straight: submit_sequence drives --distance metres at --speed (the nose may be pulled off at
    start-up; ~10° has been observed); compares heading change and lateral offset from lidar
    localization, and heading change from the gyro.
  Open loop = no heading correction + turns stop on command integral;
  closed loop = heading correction + turns stop on gyro-measured angle.

How to run (**MOVES THE BASE**: operator on site, e-stop at hand; stop Navi first, otherwise the
base/lidar/IMU ports are occupied):
  cd src/navi_agent && python -m tools.heading_check                       # default: 1.5 m straight at 0.5 m/s, 1 round per mode
  python -m tools.heading_check --distance 2 --speed 0.5 --repeat 2
Area: no obstacles within distance + 0.8 m straight ahead of the nose, and 0.6 m clearance all
around for in-place turns (after each straight leg the robot turns 180° and drives back).
Straight driving is backstopped by the lidar reflex layer; in-place turns are not, so check the
surroundings yourself before turning. Lidar localization must keep tracking (start up somewhere
already mapped, or build a new map).

Output: per-item table + summary, saved to logs/heading-check-<time>.json; a run log is also
written to logs/heading-check-<time>.run.jsonl.
The summary suggests a gyro turn-stop lead (profile robot.r1d.turn_stop_lead_s).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import statistics
import sys
import time

_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, _SRC)
_STAMP = time.strftime("%Y%m%d-%H%M%S")
_LOGS = os.path.join(_SRC, "..", "..", "logs")


class Abort(RuntimeError):
    pass


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def _first(res):
    """Extract the dict from a tool result: a dict, or [dict, image, ...] when a camera is present."""
    return next((x for x in res if isinstance(x, dict)), {}) if isinstance(res, (list, tuple)) else res


def _plain(o):
    """JSON fallback: convert numpy scalars (float64 / bool_) to plain Python values; json rejects numpy.bool_ etc."""
    return o.item() if hasattr(o, "item") else str(o)


def _navi_running(port: int) -> bool:
    if port <= 0:
        return False
    with socket.socket() as s:
        s.settimeout(0.3)
        return s.connect_ex(("127.0.0.1", port)) == 0


class Check:
    def __init__(self, srv, args):
        self.srv, self.a = srv, args
        import driver_factory
        self.driver = driver_factory.current_driver()
        self.robot = getattr(self.driver, "robot", None)
        self.rows: list[dict] = []

    def pose(self):
        """Wait `settle` seconds, then read lidar localization (x, y, yaw in degrees); abort if unreliable."""
        time.sleep(self.a.settle)
        loc = self.srv.get_localization()
        if not loc.get("ok"):
            raise Abort(f"雷达定位不可用：{loc.get('errors') or loc.get('error')} {loc.get('hint', '')}")
        p = loc["pose"]
        return float(p["x"]), float(p["y"]), math.degrees(float(p["yaw"]))    # pose values are numpy scalars; convert for JSON

    def gyro_total(self):
        t = self.robot.yaw_totals() if hasattr(self.robot, "yaw_totals") else None
        return None if t is None else (math.degrees(t["gyro"]), t["gaps"])

    def configure(self, closed: bool):
        self.robot.heading.enabled = closed
        self.driver.motion.gyro_turns = closed

    def turn(self, mode: str, angle: float):
        p0 = self.pose()
        r = _first(self.srv.turn(angle))
        if not r.get("ok"):
            raise Abort(f"转向 {angle}° 失败：{r.get('error') or r.get('status')}")
        p1 = self.pose()
        sign = 1.0 if angle > 0 else -1.0
        meas = r.get("measured_angle_rad")
        row = {"mode": mode, "kind": "turn", "req_deg": angle,
               "cmd_deg": round(sign * math.degrees(r.get("turned_rad", 0.0)), 1),
               "gyro_deg": None if meas is None else round(math.degrees(meas), 1),
               "slam_deg": round(_wrap_deg(p1[2] - p0[2]), 1),
               "shift_m": round(math.hypot(p1[0] - p0[0], p1[1] - p0[1]), 3),
               "turn_stop": r.get("turn_stop")}
        self._print(row)
        self.rows.append(row)

    def straight(self, mode: str):
        dist, speed = self.a.distance, self.a.speed
        p0, g0 = self.pose(), self.gyro_total()
        r = _first(self.srv.submit_sequence([{"vx": speed, "vyaw": 0.0, "duration": dist / speed}]))
        if not r.get("ok"):
            raise Abort(f"直行提交失败：{r.get('error')}")
        deadline = time.monotonic() + dist / speed + 15.0
        while self.srv.sequence_status().get("active"):
            if time.monotonic() > deadline:
                raise Abort("直行超时没走完")
            time.sleep(0.2)
        p1, g1 = self.pose(), self.gyro_total()
        h = math.radians(p0[2])
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        forward = dx * math.cos(h) + dy * math.sin(h)
        lateral = -dx * math.sin(h) + dy * math.cos(h)          # positive = drifted left of the start heading
        gyro = None if g0 is None or g1 is None or g1[1] != g0[1] else round(g1[0] - g0[0], 1)
        row = {"mode": mode, "kind": "straight", "speed": speed, "req_m": dist,
               "forward_m": round(forward, 3), "lateral_m": round(lateral, 3),
               "slam_deg": round(_wrap_deg(p1[2] - p0[2]), 1), "gyro_deg": gyro,
               "blocked": forward < 0.8 * dist}
        self._print(row)
        self.rows.append(row)

    @staticmethod
    def _print(row):
        if row["kind"] == "turn":
            print(f"  [{row['mode']}] 转 {row['req_deg']:+.0f}°：指令 {row['cmd_deg']:+.1f}°  陀螺 "
                  f"{'—' if row['gyro_deg'] is None else format(row['gyro_deg'], '+.1f') + '°'}  雷达 {row['slam_deg']:+.1f}°"
                  f"  原地挪动 {row['shift_m']:.2f}m{'  ⚠ 超时停' if row['turn_stop'] else ''}", flush=True)
        else:
            print(f"  [{row['mode']}] 直行 {row['req_m']}m@{row['speed']}m/s：前进 {row['forward_m']:.2f}m  "
                  f"横偏 {row['lateral_m']:+.3f}m  航向 雷达 {row['slam_deg']:+.1f}° / 陀螺 "
                  f"{'—' if row['gyro_deg'] is None else format(row['gyro_deg'], '+.1f') + '°'}"
                  f"{'  ⚠ 没走够（被雷达拦？）' if row['blocked'] else ''}", flush=True)

    def warm_up(self):
        """Gyro bias is estimated at rest and its sign verified on the first turn; if not ready, turn ±30° in place once."""
        h = getattr(self.robot, "heading", None)
        if h is None:
            raise Abort("底盘没挂上陀螺（雷达 IMU 没起来，或 robot.r1d.heading_hold=false），没法测")
        deadline = time.monotonic() + 10.0
        while h.state in ("no_imu", "calibrating") and time.monotonic() < deadline:
            time.sleep(0.2)
        if h.state == "verifying":
            print("陀螺方向核对：原地 +30° / -30°", flush=True)
            for a in (30, -30):
                _first(self.srv.turn(a))
        if h.state not in ("active", "off"):
            raise Abort(f"陀螺没就绪：{h.state} {h.status()}")
        print(f"陀螺就绪：零偏 {h.bias:+.4f} rad/s", flush=True)

    def run(self):
        for _ in range(self.a.repeat):
            for mode, closed in (("开环", False), ("闭环", True)):
                self.configure(closed)
                print(f"== {mode}：航向修正 {'开' if closed else '关'}，转向按{'陀螺实测' if closed else '指令积分'}停", flush=True)
                for angle in (90, -90, 90, -90):
                    self.turn(mode, angle)
                for _leg in range(2):
                    self.straight(mode)
                    self.turn(mode, 180)
        self.configure(True)

    def summary(self) -> dict:
        out = {}
        for mode in ("开环", "闭环"):
            turns = [r for r in self.rows if r["mode"] == mode and r["kind"] == "turn"]
            legs = [r for r in self.rows if r["mode"] == mode and r["kind"] == "straight" and not r["blocked"]]
            s = {}
            if turns:
                # Relative to the requested direction: positive = overshoot, negative = undershoot
                # (otherwise signed errors would cancel out over alternating ±90° turns).
                err = [math.copysign(1.0, r["req_deg"]) * _wrap_deg(r["slam_deg"] - r["req_deg"]) for r in turns]
                s["turn_err_deg_mean"] = round(statistics.mean(err), 1)
                s["turn_err_deg_maxabs"] = round(max(abs(e) for e in err), 1)
                gyro = [math.copysign(1.0, r["req_deg"]) * _wrap_deg(r["gyro_deg"] - r["slam_deg"])
                        for r in turns if r["gyro_deg"] is not None]
                if gyro:
                    s["gyro_minus_slam_deg_mean"] = round(statistics.mean(gyro), 1)
            if legs:
                s["straight_yaw_deg_mean"] = round(statistics.mean(r["slam_deg"] for r in legs), 1)
                s["straight_yaw_deg_maxabs"] = round(max(abs(r["slam_deg"]) for r in legs), 1)
                s["lateral_m_per_m_maxabs"] = round(max(abs(r["lateral_m"]) / max(r["forward_m"], 0.1) for r in legs), 3)
            out[mode] = s
        closed = [r for r in self.rows if r["mode"] == "闭环" and r["kind"] == "turn" and r["gyro_deg"] is not None]
        if closed:
            # Overshoot after gyro stop (measured - requested) -> extra lead = overshoot angle / turn rate
            over = statistics.mean(abs(r["gyro_deg"]) - abs(r["req_deg"]) for r in closed)
            lead = self.driver.motion.turn_stop_lead_s
            speed = float(self.driver.profile.max_vyaw) if self.driver.profile else 0.4
            out["closed_turn_overshoot_deg_mean"] = round(over, 1)
            out["turn_stop_lead_s_now"] = lead
            out["turn_stop_lead_s_suggest"] = round(max(0.0, lead + math.radians(over) / speed), 3)
        return out


def main():
    ap = argparse.ArgumentParser(description="R1D heading closed-loop on-site A/B test (moves the base)")
    ap.add_argument("--distance", type=float, default=1.5, help="distance per straight leg in m (0.5-3)")
    ap.add_argument("--speed", type=float, default=0.5, help="straight speed in m/s (0.2-0.6)")
    ap.add_argument("--repeat", type=int, default=1, help="rounds per mode, open / closed loop (1-3)")
    ap.add_argument("--settle", type=float, default=1.5, help="settle time in s before each lidar-localization reading")
    ap.add_argument("--yes", action="store_true", help="skip the pre-run confirmation (an operator must still be on site)")
    a = ap.parse_args()
    if not (0.5 <= a.distance <= 3.0 and 0.2 <= a.speed <= 0.6 and 1 <= a.repeat <= 3):
        raise SystemExit("参数超范围：--distance 0.5~3，--speed 0.2~0.6，--repeat 1~3")

    os.makedirs(_LOGS, exist_ok=True)
    os.environ["NAVI_NO_CAMERA"] = "1"                         # base, lidar and IMU only; leave the camera service untouched
    os.environ.setdefault("NAVI_RUN_LOG", os.path.join(_LOGS, f"heading-check-{_STAMP}.run.jsonl"))
    import config
    port = int(config.navi_section(config.load_config(), "observer").get("port", 0) or 0)
    if _navi_running(port):
        raise SystemExit(f"Navi 还在跑（127.0.0.1:{port} 有人在听）：先停掉 Navi，底盘和雷达不能两边同时用")

    print(f"计划：开环、闭环各 {a.repeat} 轮；每轮原地 ±90° 四次，再「直行 {a.distance}m@{a.speed}m/s → 掉头 180°」两次。\n"
          f"场地：车头正前方 {a.distance + 0.8:.1f}m 直线内无障碍，四周 0.6m 内能原地转身；人站侧后方，急停在手边。\n"
          "原地转向没有雷达兜底。Ctrl+C 随时停车。", flush=True)
    if not a.yes and input("确认场地已清空、开始测试？输入 yes：").strip() != "yes":
        raise SystemExit("已取消，未动底盘")

    import navi_mcp_server as srv
    check = None
    try:
        st = srv.get_status()
        if not st.get("robot_connected"):
            raise Abort(f"底盘没连上：{st}")
        if (st.get("robot") or {}).get("model") != "r1d":
            raise Abort(f"这个测试只针对 R1D，当前机型 {(st.get('robot') or {}).get('model')}")
        deadline = time.monotonic() + 20.0
        while not srv.get_localization().get("ok"):
            if time.monotonic() > deadline:
                raise Abort("雷达定位 20s 内没就绪（重定位没成功？开到走过的地方再试，或 NAVI_SLAM_NEW_MAP=1 新建地图）")
            time.sleep(0.5)
        check = Check(srv, a)
        check.warm_up()
        check.run()
    except KeyboardInterrupt:
        print("\n中断：停车", flush=True)
    except Abort as exc:
        print(f"\n中止：{exc}", flush=True)
    finally:
        srv.stop()
        if check is not None:
            check.configure(True)
    if check is None or not check.rows:
        return
    summary = check.summary()
    print("\n== 汇总\n" + json.dumps(summary, ensure_ascii=False, indent=2, default=_plain), flush=True)
    path = os.path.join(_LOGS, f"heading-check-{_STAMP}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"args": vars(a), "rows": check.rows, "summary": summary}, f, ensure_ascii=False, indent=2,
                  default=_plain)
    print(f"明细：{path}")


if __name__ == "__main__":
    main()
