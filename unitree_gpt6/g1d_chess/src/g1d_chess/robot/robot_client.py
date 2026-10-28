#!/usr/bin/env python3
"""CLI client for the G1 robot control service.

Sends one JSON command to the service over a ZMQ REQ socket and prints the
response. Used to test the service, and as the canonical way for the LLM / a
script to talk to the robot through the service pipe.

Examples:
    python robot_client.py ping
    python robot_client.py get_arm_pose
    python robot_client.py get_arm_q
    python robot_client.py get_full_q
    python robot_client.py capture_cameras --cameras cam_left_high cam_right_wrist
    python robot_client.py execute_plan --plan arm_plan.json --move-duration 1.0
    python robot_client.py shutdown
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time
import uuid

import zmq


def send_request(req: dict, endpoint: str, connect_timeout: int, response_timeout: int):
    """Send exactly once; a missing reply never implies that execution stopped."""
    started = time.monotonic()
    timing = {"connect_ms": 0.0, "send_ms": 0.0, "wait_ms": 0.0}
    ctx = None
    sock = None
    sent = False
    phase = "connect"
    phase_started = started
    try:
        ctx = zmq.Context()
        sock = ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.LINGER, 0)
        # Without IMMEDIATE, send_json succeeds even when no service exists.
        sock.setsockopt(zmq.IMMEDIATE, 1)
        sock.setsockopt(zmq.SNDTIMEO, connect_timeout)
        sock.setsockopt(zmq.RCVTIMEO, response_timeout)
        sock.connect(endpoint)
        timing["connect_ms"] = (time.monotonic() - phase_started) * 1000
        phase = "send"
        phase_started = time.monotonic()
        sock.send_json(req)
        sent = True
        timing["send_ms"] = (time.monotonic() - phase_started) * 1000
        phase = "response"
        phase_started = time.monotonic()
        response = sock.recv_json()
        if not isinstance(response, dict):
            raise ValueError("Service response must be a JSON object")
        timing["wait_ms"] = (time.monotonic() - phase_started) * 1000
    except (zmq.ZMQError, ValueError, TypeError) as exc:
        timing_key = {"connect": "connect_ms", "send": "send_ms", "response": "wait_ms"}[phase]
        timing[timing_key] = (time.monotonic() - phase_started) * 1000
        if isinstance(exc, zmq.Again) and not sent:
            error = "connect_timeout"
            message = (f"No connected service accepted the request within {connect_timeout} ms; "
                       "the request was not sent. Check the service and endpoint.")
        elif isinstance(exc, zmq.Again):
            error = "response_timeout"
            message = (f"The request was sent, but no response arrived within {response_timeout} ms. "
                       "The service may still be executing or waiting. This timeout does not cancel "
                       "execution; do not blindly resend a motion command.")
        else:
            error = "response_error" if sent else "connection_error"
            message = str(exc)
            if sent:
                message += (" The request was sent; execution status is unknown. This error does not "
                            "cancel execution; do not blindly resend a motion command.")
            else:
                message += " The request was not sent."
        response = {"ok": False, "error": error, "message": message,
                    "endpoint": endpoint, "cmd": req.get("cmd"),
                    "request_id": req.get("request_id"), "phase": phase,
                    "request_sent": sent}
    finally:
        try:
            if sock is not None:
                sock.close(linger=0)
        finally:
            if ctx is not None:
                ctx.term()
        timing["total_ms"] = (time.monotonic() - started) * 1000
    return response, timing


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default=os.environ.get("CHESS_ENDPOINT") or "ipc:///tmp/g1d_chess.sock", help="Service REP endpoint")
    parser.add_argument("cmd", help="Command to send")
    parser.add_argument("--cameras", nargs="+", default=None, help="Camera keys for capture_cameras")
    parser.add_argument("--output-dir", default="images", help="Output dir for capture_cameras")
    parser.add_argument("--plan", default=None, help="Plan JSON file for execute_plan")
    parser.add_argument("--side", choices=["left", "right"])
    parser.add_argument("--q", type=float, help="Signed Dex1 motor angle in rad; default opening sign +1: [0, 5.4], 0 closes")
    parser.add_argument("--brief", action="store_true", help="Print compact result; service retains full observations")
    parser.add_argument("--request", help="JSON request file; supports scene configuration")
    parser.add_argument("--save-response", help="Save the complete response JSON to this path")
    parser.add_argument("--move-duration", type=float, default=1.0, help="Per-waypoint wait for execute_plan")
    parser.add_argument("--timeout", type=int, default=60000, help="Reply timeout in ms, starting after send")
    parser.add_argument("--connect-timeout", type=int, default=1000,
                        help="Timeout in ms for sending to a connected service")
    parser.add_argument("--timing", action="store_true",
                        help="Print connect/send/wait/total timing to stderr; send includes connection readiness")
    args = parser.parse_args()
    if args.connect_timeout < 0 or args.timeout < 0:
        parser.error("--connect-timeout and --timeout must be non-negative")

    req = {"cmd": args.cmd}
    if args.request:
        with open(args.request) as f:
            req.update(json.load(f))
    if args.side is not None:
        req["side"] = args.side
    if args.q is not None:
        req["q"] = args.q
    if args.cameras is not None:
        req["cameras"] = args.cameras
    if req.get("cmd") == "capture_cameras":
        req["output_dir"] = args.output_dir
    if args.plan is not None:
        with open(args.plan) as f:
            req["plan"] = json.load(f)
        req["move_duration"] = args.move_duration

    req.setdefault("request_id", uuid.uuid4().hex)
    resp, timing = send_request(req, args.endpoint, args.connect_timeout, args.timeout)
    if args.timing:
        print(json.dumps({"event": "client_timing", "endpoint": args.endpoint,
                          "cmd": req.get("cmd"), "request_id": req["request_id"],
                          **{key: round(value, 3) for key, value in timing.items()}}), file=sys.stderr)
    if (not resp.get("ok") and "request_sent" in resp
            and resp.get("phase") in ("connect", "send", "response")):
        print(f"error: {resp['error']}: {resp['message']} "
              f"(endpoint={args.endpoint}, cmd={req.get('cmd')}, "
              f"request_id={req['request_id']}, phase={resp['phase']})", file=sys.stderr)

    if args.save_response:
        path = Path(args.save_response)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(resp, indent=2, ensure_ascii=False) + "\n")
    display = resp
    if args.brief and resp.get("ok"):
        display = {k:v for k,v in resp.items() if k not in ("state", "full_q", "arm_q", "arm_dq", "controlled_motors")}
        if "completed" in resp:
            display["completed"] = [{"step":s["step"], "errors":s["errors"], "grippers":s["grippers"],
                                     "paths":s["observation"]["paths"],
                                     "right_arm":s["observation"]["state"]["right_arm"]} for s in resp["completed"]]
    print(json.dumps(display, indent=2, ensure_ascii=False))
    return 0 if resp.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
