#!/usr/bin/env python3
"""Read G1-D arm, waist and Dex1 feedback without acquiring control or a URDF."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from unitree_gpt6.g1d_draw.src.config import load_config
from unitree_gpt6.g1d_draw.src.g1 import ARM_SLOTS, GRIP_SLOTS, WAIST_Q3_SLOTS, G1DDSBackend
from unitree_gpt6.g1d_draw.src.kinematics import ARM_JOINT_NAMES


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def capture(config, *, samples=1, interval=0.5):
    """Return polled snapshots; sample count is not the DDS callback rate.

    ``connect_timeout_s`` and ``state_timeout_s`` in config control feedback
    timeouts. ``model`` is unused: joint feedback does not need FK or IK.
    """
    if type(samples) is not int or samples < 1:
        raise ValueError("samples must be a positive integer")
    if not math.isfinite(interval) or interval <= 0:
        raise ValueError("interval must be finite and positive")
    device = config["device"]
    internal = device["gripper_interface"] == "internal"
    report = {
        "format": "robot-paint-joint-observation/v1",
        "device": {
            key: device[key]
            for key in ("interface", "domain_id", "lowstate_topic", "gripper_interface")
        },
        "arm_order": list(ARM_JOINT_NAMES),
        "arm_lowstate_slots": list(ARM_SLOTS),
        "waist_order": ["waist_yaw", "waist_roll", "waist_pitch"],
        "waist_lowstate_slots": list(WAIST_Q3_SLOTS),
        "waist_note": (
            "G1-D has yaw/pitch only; roll is a fixed zero, not a sensor reading. "
            "Slot 14 is unused."
        ),
        "grip_order": ["left", "right"],
        "grip_lowstate_slots": list(GRIP_SLOTS) if internal else None,
        "units": {
            "q14": "rad",
            "dq14": "rad/s",
            "q14_deg": "deg",
            "waist_q3": "rad",
            "waist_dq3": "rad/s",
            "waist_q3_deg": "deg",
            "grip_q2": "native motor units",
            "grip_dq2": "native motor units/s",
        },
        "timing": (
            "UTC times bracket host read(), not robot acquisition. Callback ages are "
            "measured at a subsequent health check and may refer to newer callbacks. "
            "Samples are polled latest states; external gripper streams are not synchronized."
        ),
        "requested_interval_s": interval,
        "samples": [],
    }
    if not internal:
        report["device"]["external_grippers"] = device["external_grippers"]
    backend = G1DDSBackend(config)
    try:
        backend.connect()  # Subscriptions only: no acquire(), command(), or publisher.
        started = time.monotonic()
        for index in range(samples):
            if index:
                time.sleep(interval)
            before = utc_now()
            state = backend.read()
            after = utc_now()
            health = backend.health()
            if health["publishing"] or health["fault"] or not health["telemetry_fresh"]:
                raise RuntimeError(f"unexpected read-only backend health: {health}")
            record = {
                "sample_index": index,
                "host_read_started_at_utc": before,
                "host_read_completed_at_utc": after,
                "host_elapsed_s": time.monotonic() - started,
                "callback_age_s_at_health_check": health["telemetry_age_s"],
                **{
                    key: state[key].tolist()
                    for key in ("q14", "dq14", "waist_q3", "waist_dq3", "grip_q2", "grip_dq2")
                },
                "q14_deg": np.rad2deg(state["q14"]).tolist(),
                "waist_q3_deg": np.rad2deg(state["waist_q3"]).tolist(),
            }
            report["samples"].append(record)
    finally:
        backend.close()
    errors = backend.health()["close_errors"]
    if errors:
        raise RuntimeError(f"DDS channel cleanup failed: {errors}")
    return report


def print_report(report):
    for sample in report["samples"]:
        print(f"Sample {sample['sample_index']} — host {sample['host_read_completed_at_utc']}")
        print(f"{'Joint':30} {'slot':>4} {'rad':>11} {'deg':>11} {'rad/s':>11}")
        for index, name in enumerate(report["arm_order"]):
            print(
                f"{name:30} {report['arm_lowstate_slots'][index]:4d} "
                f"{sample['q14'][index]:11.6f} {sample['q14_deg'][index]:11.3f} "
                f"{sample['dq14'][index]:11.6f}"
            )
        for index, name in enumerate(report["waist_order"]):
            slot = report["waist_lowstate_slots"][index]
            slot_label = "--" if slot is None else str(slot)
            print(
                f"{name:30} {slot_label:>4} "
                f"{sample['waist_q3'][index]:11.6f} {sample['waist_q3_deg'][index]:11.3f} "
                f"{sample['waist_dq3'][index]:11.6f}"
            )
        print(f"Dex1 left/right (native units): {sample['grip_q2']}")
        print(f"Callback ages (s): {sample['callback_age_s_at_health_check']}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="Device JSON; model/URDF is unused")
    parser.add_argument("--interface", help="Override device.interface, e.g. enp4s0")
    parser.add_argument(
        "--samples", type=int, default=1, help="Number of polled snapshots (default: 1)"
    )
    parser.add_argument(
        "--interval", type=float, default=0.5, help="Seconds between reads (default: 0.5)"
    )
    parser.add_argument(
        "--output", required=True, type=Path, help="New JSON file; never overwrite a capture"
    )
    args = parser.parse_args(argv)
    output_created = False
    try:
        config = load_config(args.config)
        if args.interface is not None:
            if not args.interface.strip():
                raise ValueError("interface must name the actual wired DDS interface")
            config["device"]["interface"] = args.interface
        args.output.parent.mkdir(parents=True, exist_ok=True)
        # Reserve the destination before connecting; failure leaves no partial capture.
        with args.output.open("x", encoding="utf-8") as stream:
            output_created = True
            report = capture(config, samples=args.samples, interval=args.interval)
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
        output_created = False
        print_report(report)
        print(f"Saved {len(report['samples'])} snapshots: {args.output}")
        return 0
    except KeyboardInterrupt:
        print("Read interrupted; DDS subscriptions closed.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        if output_created:
            args.output.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
