# -*- coding: utf-8 -*-
"""probe.py -- read-only model detection: one short-lived subprocess per (NIC, domain);
subscribe only, never publish.

Why a subprocess: ChannelFactoryInitialize(domain, iface) can be called only once per process,
and the main process must keep it for the real controller; the probe exits after subscribing.
The subprocess has stdin=DEVNULL and stdout piped (MCP stdio stdout is JSON-RPC and must never
be polluted by probe output); the result is the last JSON line.

Signatures (each model only checks its own topics; no Publisher / RPC client is created):
  g1d: receives rt/hispeed_state (Point32, column height)
  r1d: receives rt/height_alarm with source == 200, and rt/state_vel (Twist)
Adopted only if exactly one candidate matches; zero or several matches -> fail closed (no driving).

Standalone (diagnostics, read-only): python -m services.robots.probe --net enP2p1s0 --domain 0
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

T_G1D = "rt/hispeed_state"
T_R1D_ALARM = "rt/height_alarm"
T_R1D_VEL = "rt/state_vel"
R1D_ALARM_SOURCE = 200
SRC_ROOT = Path(__file__).resolve().parents[2]


def matches(model: str, obs: dict) -> bool:
    """Whether the probe observation matches the model's signature. obs is the subprocess JSON."""
    if not obs.get("ok"):
        return False
    if model == "g1d":
        return obs.get("hispeed_state", 0) > 0
    if model == "r1d":
        return obs.get("alarm_source_ok", 0) > 0 and obs.get("state_vel", 0) > 0
    return False


def listen(net: str, domain: int, timeout_s: float) -> dict:
    """Runs in the subprocess: init DDS, subscribe to signature topics for timeout_s, return per-topic message counts."""
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_, Twist_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import Error_

    counts = {"hispeed_state": 0, "height_alarm": 0, "alarm_source_ok": 0, "state_vel": 0}
    lock = threading.Lock()

    def bump(*keys):
        with lock:
            for k in keys:
                counts[k] += 1

    def on_alarm(m):
        bump("height_alarm", *(("alarm_source_ok",) if int(m.source) == R1D_ALARM_SOURCE else ()))

    ChannelFactoryInitialize(int(domain), net)
    subs = []
    for topic, msg_type, cb in ((T_G1D, Point32_, lambda m: bump("hispeed_state")),
                                (T_R1D_ALARM, Error_, on_alarm),
                                (T_R1D_VEL, Twist_, lambda m: bump("state_vel"))):
        s = ChannelSubscriber(topic, msg_type)
        s.Init(cb, 1)
        subs.append(s)
    time.sleep(timeout_s)
    with lock:
        return {"ok": True, "net": net, "domain_id": int(domain), **counts}


def run_probe(net: str, domain: int, timeout_s: float) -> dict:
    """Main-process side: spawn a subprocess running listen and parse its last JSON line.
    Any failure returns ok=False (never raises)."""
    cmd = [sys.executable, "-m", "services.robots.probe", "--net", str(net),
           "--domain", str(int(domain)), "--timeout", str(float(timeout_s))]
    try:
        p = subprocess.run(cmd, cwd=str(SRC_ROOT), stdin=subprocess.DEVNULL,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           timeout=float(timeout_s) + 15.0, env=dict(os.environ))
    except subprocess.TimeoutExpired:
        return {"ok": False, "net": net, "domain_id": domain, "error": "probe_timeout"}
    except OSError as exc:
        return {"ok": False, "net": net, "domain_id": domain, "error": f"probe_spawn_failed: {exc}"}
    lines = [ln for ln in p.stdout.decode("utf-8", "replace").splitlines() if ln.strip()]
    try:
        return json.loads(lines[-1])
    except (IndexError, ValueError):
        tail = p.stderr.decode("utf-8", "replace").strip().splitlines()[-1:] or [""]
        return {"ok": False, "net": net, "domain_id": domain,
                "error": f"probe_failed(rc={p.returncode}): {tail[0][:200]}"}


def detect(candidates: list[dict], timeout_s: float, runner=run_probe) -> tuple[dict | None, str, list]:
    """Probe candidates per (net, domain). Returns (matched candidate | None, reason, observations).

    candidates items: {model, net, domain_id}, with net already resolved to a real NIC name."""
    groups: dict[tuple, list[dict]] = {}
    for c in candidates:
        groups.setdefault((c["net"], int(c.get("domain_id", 0))), []).append(c)
    hits, observations = [], []
    for (net, domain), members in groups.items():
        obs = runner(net, domain, timeout_s)
        observations.append(obs)
        hits.extend(c for c in members if matches(c["model"], obs))
    if len(hits) == 1:
        c = hits[0]
        return c, f"auto_detected: {c['model']} @ {c['net']}/domain {c.get('domain_id', 0)}", observations
    if not hits:
        errs = [o.get("error") for o in observations if o.get("error")]
        return None, "no_robot_detected" + (f" ({'; '.join(errs)})" if errs else ""), observations
    desc = ", ".join(f"{c['model']}@{c['net']}/{c.get('domain_id', 0)}" for c in hits)
    return None, f"ambiguous_robot_detected: {desc}", observations


def _main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="只读机型探测（只订阅，不发布）")
    ap.add_argument("--net", required=True)
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--timeout", type=float, default=2.5)
    a = ap.parse_args(argv)
    try:
        out = listen(a.net, a.domain, a.timeout)
    except Exception as exc:
        out = {"ok": False, "net": a.net, "domain_id": a.domain,
               "error": f"{type(exc).__name__}: {exc}"}
    sys.stdout.write(json.dumps(out) + "\n")
    sys.stdout.flush()
    os._exit(0)          # don't wait for DDS background threads to wind down


if __name__ == "__main__":
    _main()
