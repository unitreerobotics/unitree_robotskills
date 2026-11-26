"""Record the three robot views (head left eye, left wrist, right wrist) to MP4, read-only.

Runs in its own process with its own teleimager subscriber, so it never competes with the skill server.
    usage (from src/): python3 -m perception.record3 [--out DIR] [--fps 15] [--seconds N]
    normally started and stopped by `bash r1d eef rec-start` / `rec-stop`
Stops on Ctrl+C / SIGTERM / --seconds. Writes head.mp4, left_wrist.mp4, right_wrist.mp4 and a 2x2-ish mosaic
all.mp4 (head at 2x on the left, wrists stacked on the right) with a time stamp. Frames are converted from the client's BGR correctly.
"""
import argparse
import os
import signal
import time
from pathlib import Path

import cv2
import numpy as np

from perception.image_client import ImageClient
from service.paths import RUNS_DIR

VIEWS = {"head": "head_left", "left_wrist": "left_wrist", "right_wrist": "right_wrist"}
SIZE = {"head": (544, 448), "left_wrist": (640, 480), "right_wrist": (640, 480)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(RUNS_DIR / "recordings" / time.strftime("%Y%m%d-%H%M%S")))
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = until stopped")
    ap.add_argument("--host", default="192.168.123.164")
    a = ap.parse_args()
    for k in ("http_proxy", "https_proxy", "HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "all_proxy"):
        os.environ.pop(k, None)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    cam = ImageClient(a.host)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writers = {k: cv2.VideoWriter(str(out / f"{k}.mp4"), fourcc, a.fps, SIZE[k]) for k in VIEWS}
    mw, mh = 1088 + 640, 960                                 # head at 2x on the left, the two wrists stacked on the right
    writers["all"] = cv2.VideoWriter(str(out / "all.mp4"), fourcc, a.fps, (mw, mh))
    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("flag", True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("flag", True))
    last = {k: np.zeros((SIZE[k][1], SIZE[k][0], 3), np.uint8) for k in VIEWS}
    t0 = time.time()
    n = 0
    next_t = t0
    print(f"recording to {out} at {a.fps} fps (Ctrl+C to stop)", flush=True)
    while not stop["flag"] and (a.seconds <= 0 or time.time() - t0 < a.seconds):
        try:
            frames = cam.frames()
        except Exception:
            frames = {}
        for k, src in VIEWS.items():
            f = frames.get(src)
            if f is not None:
                if (f.shape[1], f.shape[0]) != SIZE[k]:
                    f = cv2.resize(f, SIZE[k])
                last[k] = f                                  # BGR, as VideoWriter expects
        stamp = f"{time.strftime('%H:%M:%S')}  t={time.time() - t0:6.1f}s"
        for k in VIEWS:
            img = last[k].copy()
            cv2.putText(img, f"{k}  {stamp}", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(img, f"{k}  {stamp}", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            writers[k].write(img)
        mos = np.zeros((mh, mw, 3), np.uint8)
        mos[32:32 + 896, :1088] = cv2.resize(last["head"], (1088, 896), interpolation=cv2.INTER_LINEAR)
        mos[:480, 1088:] = last["left_wrist"]
        mos[480:, 1088:] = last["right_wrist"]
        cv2.putText(mos, stamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        writers["all"].write(mos)
        n += 1
        next_t += 1.0 / a.fps
        time.sleep(max(0.0, next_t - time.time()))
    for w in writers.values():
        w.release()
    print(f"saved {n} frames ({time.time() - t0:.0f} s) to {out}", flush=True)
    os._exit(0)


if __name__ == "__main__":
    main()
