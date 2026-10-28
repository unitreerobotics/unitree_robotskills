# -*- coding: utf-8 -*-
"""services.robots -- multi-model chassis adapters (G1D / R1D) and startup model detection.

  profile.py  RobotProfile (capabilities / speed limits / NIC / domain) + built-in g1d / r1d
  g1d.py      G1D agv RPC controller
  r1d.py      R1D rt/cmd_vel controller
  unknown.py  placeholder for unrecognised/ambiguous detection (refuses all motion)
  probe.py    read-only model detection subprocess

build_robot(cfg) is the only entry point: selects the model from config robot.model
(auto | g1d | r1d). With auto, detection runs once per startup (no hot switching; changing
robots = restarting the server); drives only if exactly one candidate matches, else fails closed.
"""
from __future__ import annotations

import sys

import config
from services.robots.profile import RobotProfile, build_profile
from services.robots.profile import (
    unknown as _unknown,  # alias: importing submodule services.robots.unknown overwrites the package attribute `unknown`
)

__all__ = ["RobotProfile", "build_profile", "build_robot", "resolve_profile"]


def _log(msg):
    print(f"[robot] {msg}", file=sys.stderr)


def _net(value) -> str:
    """net from candidates/overrides: auto/empty -> config.NET (NIC already resolved by apply_config)."""
    if not value or str(value).strip().lower() in ("auto", "none"):
        return config.NET
    return str(value).strip()


def _robot_cfg(cfg: dict) -> dict:
    base = dict(config.DEFAULT_CONFIG["robot"])
    base.update((cfg or {}).get("robot") or {})
    return base


def resolve_profile(cfg: dict, runner=None) -> RobotProfile:
    """Resolve the effective profile (no controller, no in-process DDS init; auto only spawns
    read-only probe subprocesses)."""
    rc = _robot_cfg(cfg)
    model = str(rc.get("model") or "auto").strip().lower()
    if model in ("g1d", "r1d"):
        overrides = dict(rc.get(model) or {})
        net = _net(overrides.pop("net", None))
        domain = int(overrides.pop("domain_id", 0))
        return build_profile(model, overrides, net=net, domain_id=domain,
                             reason=f"configured: robot.model={model}")
    if model != "auto":
        return _unknown(f"robot.model={model!r} 无效（auto | g1d | r1d）")

    from services.robots import probe
    candidates = []
    for c in rc.get("candidates") or []:
        m = str((c or {}).get("model", "")).strip().lower()
        if m not in ("g1d", "r1d"):
            return _unknown(f"robot.candidates 含未知机型 {m!r}")
        candidates.append({"model": m, "net": _net(c.get("net")), "domain_id": int(c.get("domain_id", 0))})
    if not candidates:
        return _unknown("robot.model=auto 但 robot.candidates 为空")
    hit, reason, observations = probe.detect(candidates, float(rc.get("probe_timeout_s", 2.5)),
                                             **({"runner": runner} if runner else {}))
    _log(f"机型探测: {reason}; 观测={observations}")
    if hit is None:
        return _unknown(reason)
    overrides = dict(rc.get(hit["model"]) or {})
    overrides.pop("net", None)
    overrides.pop("domain_id", None)
    return build_profile(hit["model"], overrides, net=hit["net"], domain_id=hit["domain_id"],
                         reason=reason)


def build_robot(cfg: dict, runner=None):
    """Return (profile, robot). On detection failure returns UnknownRobot (no DDS init, all motion refused)."""
    try:
        profile = resolve_profile(cfg, runner=runner)
    except (ValueError, TypeError) as exc:
        profile = _unknown(f"robot 配置无效: {exc}")
    if profile.model == "g1d":
        from services.robots.g1d import RobotController
        robot = RobotController(net=profile.net, domain_id=profile.domain_id)
    elif profile.model == "r1d":
        from services.robots.r1d import R1DRobot
        robot = R1DRobot(profile, net=profile.net, domain_id=profile.domain_id)
    else:
        from services.robots.unknown import UnknownRobot
        robot = UnknownRobot(profile.reason)
        _log(f"未识别机型，拒绝一切运动：{profile.reason}")
    robot.profile = profile
    _log(f"机型 {profile.model}（{profile.reason}）net={profile.net} domain={profile.domain_id}")
    return profile, robot
