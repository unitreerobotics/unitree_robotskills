"""Check G1-D joint feedback and camera images without publishing robot commands.

Examples:
  python examples/check_connection.py --config device.json --interface enp4s0
  python examples/check_connection.py --camera-host 192.168.123.164
  python examples/check_connection.py --stream head=tcp://192.168.123.164:55555
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
from datetime import datetime, timezone
from pathlib import Path

if __package__:
    from .capture_images import capture, parse_streams
else:
    from capture_images import capture, parse_streams


CAMERA_STREAMS = ("head_camera", "left_wrist_camera", "right_wrist_camera")


def camera_endpoints(config, host):
    """Resolve enabled TeleImager streams from the server's live configuration."""
    if not isinstance(config, dict):
        raise ValueError("camera configuration must be a JSON object")
    endpoints = {}
    for name in CAMERA_STREAMS:
        stream = config.get(name, {})
        if not isinstance(stream, dict):
            raise ValueError(f"camera configuration {name} must be an object")
        enabled = stream.get("enable_zmq", False)
        if type(enabled) is not bool:
            raise ValueError(f"{name}.enable_zmq must be a boolean")
        if not enabled:
            continue
        port = stream.get("zmq_port")
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError(f"{name}.zmq_port must be an integer from 1 to 65535")
        endpoints[name] = f"tcp://{host}:{port}"
    if not endpoints:
        raise ValueError("camera configuration has no enabled JPEG/ZMQ streams")
    return parse_streams([f"{name}={endpoint}" for name, endpoint in endpoints.items()])


def request_camera_config(host, port=60000, timeout=5.0):
    """Request live configuration; never silently substitute a cached file."""
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be finite and positive")
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError("request port must be an integer from 1 to 65535")
    endpoint = parse_streams([f"config=tcp://{host}:{port}"])["config"]
    import zmq

    context = zmq.Context()
    requester = context.socket(zmq.REQ)
    try:
        requester.setsockopt(zmq.LINGER, 0)
        requester.setsockopt(zmq.SNDTIMEO, max(1, math.ceil(timeout * 1000)))
        requester.connect(endpoint)
        requester.send(b"GET_DATA")
        if not requester.poll(max(1, math.ceil(timeout * 1000)), zmq.POLLIN):
            raise TimeoutError(f"no camera configuration received from {endpoint}")
        config = requester.recv_json()
        camera_endpoints(config, host)
        return config
    finally:
        requester.close(linger=0)
        context.term()


def check_dds(path, interface=None, timeout=5.0):
    """Read one fresh feedback sample; neither acquire nor command is called."""
    from unitree_gpt6.g1d_draw.src.config import load_config
    from unitree_gpt6.g1d_draw.src.g1 import WAIST_Q3_SLOTS, G1DDSBackend

    config = load_config(path)
    if interface:
        config["device"]["interface"] = interface
    device = config["device"]
    socket.if_nametoindex(device["interface"])
    config["motion"]["connect_timeout_s"] = timeout
    backend = G1DDSBackend(config)
    try:
        backend.connect()
        sample = backend.read()
        health = backend.health()
        if health["publishing"] or not health["telemetry_fresh"] or health["fault"]:
            raise RuntimeError(f"DDS feedback check failed: {health}")
        return {
            "ok": True,
            "interface": device["interface"],
            "domain_id": device["domain_id"],
            "topic": device["lowstate_topic"],
            "host_read_utc": datetime.now(timezone.utc).isoformat(),
            "telemetry_age_s": health["telemetry_age_s"],
            "publishing": False,
            "arm_q14_rad": sample["q14"].tolist(),
            "waist_q3_rad": sample["waist_q3"].tolist(),
            "waist_order": ["yaw", "roll", "pitch"],
            "waist_lowstate_slots": list(WAIST_Q3_SLOTS),
            "waist_note": "G1-D roll is fixed at zero; only yaw/pitch are measured.",
            "grip_q2_native": sample["grip_q2"].tolist(),
        }
    finally:
        backend.close()


def run_checks(
    *, config=None, interface=None, camera_host=None, request_port=60000, streams=None, timeout=5.0
):
    """Keep partial results when another requested component is unavailable."""
    checks = {}
    if config is not None:
        try:
            checks["dds"] = check_dds(config, interface, timeout)
        except Exception as exc:
            checks["dds"] = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
    discovered = {}
    if camera_host is not None:
        try:
            camera_config = request_camera_config(camera_host, request_port, timeout)
            discovered = camera_endpoints(camera_config, camera_host)
            checks["camera_config"] = {
                "ok": True,
                "host": camera_host,
                "request_port": request_port,
                "streams": discovered.copy(),
                "configuration": camera_config,
            }
        except Exception as exc:
            checks["camera_config"] = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
    # Explicit endpoints can replace a same-name discovered endpoint.
    for name, endpoint in {**discovered, **(streams or {})}.items():
        try:
            frame = capture({name: endpoint}, timeout=timeout, warmup_frames=0)[name]
            checks[f"image:{name}"] = {
                "ok": True,
                "endpoint": endpoint,
                "shape_hw_channels": list(frame["bgr"].shape),
                "host_received_utc": frame["host_received_utc"],
                "host_received_monotonic_s": frame["host_received_monotonic_s"],
            }
        except Exception as exc:
            checks[f"image:{name}"] = {
                "ok": False,
                "endpoint": endpoint,
                "error": f"{type(exc).__name__}: {exc}",
            }
    return {
        "format": "robot-paint-connection-check/v1",
        "ok": bool(checks) and all(result["ok"] for result in checks.values()),
        "checked_utc": datetime.now(timezone.utc).isoformat(),
        "timestamp_note": "Host read/receipt times, not camera exposure times.",
        "checks": checks,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, help="device JSON for read-only DDS check")
    parser.add_argument("--interface", help="override config's wired DDS interface")
    parser.add_argument("--camera-host", help="TeleImager server host")
    parser.add_argument("--camera-config-port", dest="request_port", type=int, default=60000)
    parser.add_argument(
        "--stream",
        action="append",
        default=[],
        metavar="NAME=tcp://HOST:PORT",
        help="direct JPEG stream; replaces a same-name discovered endpoint",
    )
    parser.add_argument("--timeout", type=float, default=5.0, help="seconds per component check")
    parser.add_argument("--output", type=Path, help="optional new JSON report file")
    args = parser.parse_args(argv)
    if not (args.config or args.camera_host or args.stream):
        parser.error("provide --config, --camera-host or --stream")
    if args.interface and not args.config:
        parser.error("--interface requires --config")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    try:
        streams = parse_streams(args.stream) if args.stream else {}
        output = args.output.expanduser() if args.output else None
        if output is not None and output.exists():
            raise FileExistsError(f"report already exists: {output}")
        report = run_checks(
            config=args.config,
            interface=args.interface,
            camera_host=args.camera_host,
            request_port=args.request_port,
            streams=streams,
            timeout=args.timeout,
        )
        text = json.dumps(report, indent=2, allow_nan=False) + "\n"
        if output is not None:
            output.parent.mkdir(parents=True, exist_ok=True)
            with output.open("x", encoding="utf-8") as handle:
                handle.write(text)
        print(text, end="")
        return 0 if report["ok"] else 1
    except (OSError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
