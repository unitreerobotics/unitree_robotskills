"""Save images from TeleImager-compatible single-message JPEG/ZMQ streams.

Example: python examples/capture_images.py --stream head=tcp://HOST:55555
         --stream left_wrist=tcp://HOST:55556 --output captures/view-1

No DDS connection or robot control is involved. A camera server must already
publish JPEG bytes; this is not an RTSP, WebRTC or multipart-stream client.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import urlsplit

import numpy as np


def parse_streams(values):
    """Map user labels to explicit endpoints; labels are also file names."""
    streams = {}
    for value in values:
        name, separator, endpoint = value.partition("=")
        if not separator or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", name):
            raise ValueError("each --stream must be NAME=tcp://HOST:PORT")
        url = urlsplit(endpoint)
        if (
            url.scheme != "tcp"
            or not url.hostname
            or not url.port
            or url.path
            or url.query
            or url.fragment
            or url.username
            or url.password
        ):
            raise ValueError(f"invalid TCP stream endpoint: {endpoint}")
        if name in streams or endpoint in streams.values():
            raise ValueError("stream names and endpoints must be unique")
        streams[name] = endpoint
    if not streams:
        raise ValueError("at least one stream is required")
    return streams


def capture(streams, *, timeout=10.0, warmup_frames=3):
    """Return decoded BGR frames and host receipt times, with bounded waiting.

    Streams run concurrently. Images are not exposure-synchronized, and a host
    receipt time cannot establish when the sensor exposed the image.
    """
    streams = parse_streams([f"{name}={endpoint}" for name, endpoint in streams.items()])
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be finite and positive")
    if type(warmup_frames) is not int or warmup_frames < 0:
        raise ValueError("warmup_frames must be a nonnegative integer")
    # Optional imports stay inside capture so --help needs only the base package.
    import cv2
    import zmq

    frames, sockets = {}, {}
    counts = {name: 0 for name in streams}
    invalid = {name: 0 for name in streams}
    context = zmq.Context()
    poller = zmq.Poller()
    try:
        for name, endpoint in streams.items():
            socket = context.socket(zmq.SUB)
            sockets[socket] = name
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.RCVHWM, 1)
            socket.setsockopt(zmq.CONFLATE, 1)  # This protocol uses one message/frame.
            socket.setsockopt(zmq.SUBSCRIBE, b"")
            socket.connect(endpoint)
            poller.register(socket, zmq.POLLIN)
        deadline = time.monotonic() + timeout
        while len(frames) < len(streams):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                missing = sorted(set(streams) - set(frames))
                raise TimeoutError(
                    f"no usable image before timeout for {missing}; "
                    f"received counts={counts}, invalid JPEG counts={invalid}. "
                    "Check the camera server, address, port and JPEG/ZMQ protocol."
                )
            for socket, event in poller.poll(max(1, min(100, math.ceil(remaining * 1000)))):
                if not event & zmq.POLLIN:
                    continue
                payload = socket.recv()
                received_utc = datetime.now(timezone.utc).isoformat()
                received_monotonic = time.monotonic()
                name = sockets[socket]
                counts[name] += 1
                # Reject other protocols instead of treating arbitrary bytes as images.
                frame = None
                if payload.startswith(b"\xff\xd8"):
                    try:
                        frame = cv2.imdecode(
                            np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR
                        )
                    except cv2.error:
                        pass
                if frame is None:
                    invalid[name] += 1
                    continue
                if counts[name] - invalid[name] <= warmup_frames:
                    continue
                frames[name] = {
                    "bgr": frame,
                    "endpoint": streams[name],
                    "host_received_utc": received_utc,
                    "host_received_monotonic_s": received_monotonic,
                    "received_messages": counts[name],
                    "invalid_messages": invalid[name],
                }
                poller.unregister(socket)
        return frames
    except zmq.ZMQError as exc:
        raise RuntimeError(f"camera transport failed: {exc}") from exc
    finally:
        for socket in sockets:
            socket.close(linger=0)
        context.term()


def save_capture(frames, output):
    """Write lossless PNGs and provenance in a new directory."""
    import cv2

    output = Path(output).expanduser()
    # Encode first so a bad image cannot leave a misleading completed capture.
    encoded, records = {}, {}
    for name, frame in frames.items():
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", name):
            raise ValueError("invalid image name")
        ok, png = cv2.imencode(".png", frame["bgr"])
        if not ok:
            raise RuntimeError(f"failed to encode {name}")
        encoded[name] = png.tobytes()
        records[name] = {key: value for key, value in frame.items() if key != "bgr"}
        records[name].update(
            file=f"{name}.png",
            shape_hw_channels=list(frame["bgr"].shape),
            sha256=hashlib.sha256(encoded[name]).hexdigest(),
        )
    if not frames:
        raise ValueError("no images to save")
    report = {
        "format": "robot-paint-image-capture/v1",
        "transport": "zmq-single-jpeg",
        "timestamp_note": (
            "Host receipt times, not sensor exposure times; streams are not synchronized."
        ),
        "images": records,
    }
    output.mkdir(parents=True, exist_ok=False)
    for name, data in encoded.items():
        (output / f"{name}.png").write_bytes(data)
    # The manifest is written last, only after every image was saved.
    (output / "capture.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stream", action="append", required=True, metavar="NAME=tcp://HOST:PORT")
    parser.add_argument("--output", type=Path, required=True, help="new output directory")
    parser.add_argument(
        "--timeout", type=float, default=10.0, help="total connection/frame timeout in seconds"
    )
    parser.add_argument(
        "--warmup-frames", type=int, default=3, help="valid frames to skip per stream"
    )
    args = parser.parse_args(argv)
    try:
        if args.output.expanduser().exists():
            raise FileExistsError(f"output already exists: {args.output}; choose a new directory")
        streams = parse_streams(args.stream)
        frames = capture(streams, timeout=args.timeout, warmup_frames=args.warmup_frames)
        save_capture(frames, args.output)
    except ImportError as exc:
        print(
            f"{exc}. From the repository root, run: "
            "python -m pip install -r unitree_gpt6/g1d_draw/requirements.txt",
            file=sys.stderr,
        )
        return 1
    except (ValueError, OSError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    print(f"Saved {len(frames)} images and capture.json to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
