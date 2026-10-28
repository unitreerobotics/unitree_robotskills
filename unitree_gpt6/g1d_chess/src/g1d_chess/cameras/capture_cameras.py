#!/usr/bin/env python3
"""Capture one observation from the Unitree G1 internal cameras.

Saves selected camera images (default: cam_left_high only) using LeRobot-style
paths, keyed by a timestamp::

    images/observation.images.cam_left_high/{timestamp}.jpg
    images/observation.images.cam_left_wrist/{timestamp}.jpg
    images/observation.images.cam_right_high/{timestamp}.jpg
    images/observation.images.cam_right_wrist/{timestamp}.jpg

and prints the unique timestamp identifier (e.g. ``20260918_201850_123456``)
as the only line on stdout.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from datetime import datetime

import cv2

from g1d_chess.cameras.imageclient import ImageClientCamera

logger = logging.getLogger("capture_cameras")

# hardware camera key -> LeRobot-style observation directory name
CAMERA_KEYS = [
    "cam_left_high",
    "cam_right_high",
    "cam_left_wrist",
    "cam_right_wrist",
]

OBS_KEY = {
    "cam_left_high": "observation.images.cam_left_high",
    "cam_right_high": "observation.images.cam_right_high",
    "cam_left_wrist": "observation.images.cam_left_wrist",
    "cam_right_wrist": "observation.images.cam_right_wrist",
}


def timestamp_id(root: str, keys) -> str:
    """Return a unique, sortable timestamp identifier (microsecond precision).

    ``YYYYMMDD_HHMMSS_ffffff``; if that ``.jpg`` file already exists under any
    of ``keys`` (same-microsecond collision), a numeric suffix is appended.
    """
    base = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    ts = base
    i = 0
    while any(os.path.exists(os.path.join(root, OBS_KEY[key], f"{ts}.jpg")) for key in keys):
        i += 1
        ts = f"{base}_{i}"
    return ts


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.environ.get("CHESS_CAMERA_HOST"), help="Image server IP (G1 board)")
    parser.add_argument("--output-dir", default="images", help="Root output directory (default: images)")
    parser.add_argument(
        "--cameras",
        nargs="+",
        default=None,
        help="Which cameras to read (default: cam_left_high only). Space- or comma-separated keys from: "
             "cam_left_high, cam_right_high, cam_left_wrist, cam_right_wrist.",
    )
    args = parser.parse_args(argv)
    if not args.host:
        parser.error("pass --host or set CHESS_CAMERA_HOST")

    logging.basicConfig(
        stream=sys.stderr, level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s"
    )

    output_dir = os.path.abspath(args.output_dir)

    # Determine which cameras to read.
    if args.cameras is None:
        selected_keys = ["cam_left_high"]
    else:
        selected_keys = []
        for item in args.cameras:
            selected_keys.extend(s.strip() for s in item.split(",") if s.strip())
        invalid = [k for k in selected_keys if k not in CAMERA_KEYS]
        if invalid:
            logger.error("invalid camera key(s): %s (valid: %s)", invalid, CAMERA_KEYS)
            return 1
    logger.info("cameras to read: %s", selected_keys)

    camera = ImageClientCamera(host_ip=args.host)
    camera.connect()
    try:
        # PUB/SUB needs a few frames to warm up before a latest frame is available.
        colors = {}
        deadline = time.time() + 5.0
        while time.time() < deadline:
            colors = camera.async_read(selected_keys)
            if all(colors.get(k) is not None for k in selected_keys):
                break
            time.sleep(0.05)
    finally:
        camera.disconnect()

    if not colors:
        logger.error("No camera frames received from %s", args.host)
        return 1

    # Timestamp is taken at capture time (after frames are received).
    episode_id = timestamp_id(output_dir, selected_keys)

    saved = 0
    for key in selected_keys:
        img = colors.get(key)
        if img is None:
            logger.warning("Camera '%s' not available, skipping.", key)
            continue
        cam_dir = os.path.join(output_dir, OBS_KEY[key])
        os.makedirs(cam_dir, exist_ok=True)
        path = os.path.join(cam_dir, f"{episode_id}.jpg")
        cv2.imwrite(path, img)
        logger.info("saved %s", os.path.relpath(path, output_dir))
        saved += 1

    logger.info("captured %s (%d/%d cameras)", episode_id, saved, len(selected_keys))
    print(episode_id, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
