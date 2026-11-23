"""Static waypoint error per (URDF, friction share): python compare.py RUN_DIR [first_index]."""
import json
import sys

import numpy as np

rows = [json.loads(a) for a in open(sys.argv[1] + "/waypoints.jsonl")]
first = int(sys.argv[2]) if len(sys.argv) > 2 else 0
by = {}
previous = None
for row in rows:
    if previous is not None and row["index"] == previous["index"] + 1 and row["index"] >= first:
        error = np.degrees(np.array(row["q"]) - np.array(row["commanded"]))
        step = np.array(row["commanded"]) - np.array(previous["commanded"])
        along = error * np.sign(step)
        along[np.abs(step) < 0.02] = np.nan
        by.setdefault((row.get("model", ""), row.get("friction_share", 1.0)), []).append((error, along))
    previous = row
np.set_printoptions(precision=3, suppress=True, linewidth=200)
for share, items in sorted(by.items()):
    error = np.array([a for a, _ in items])
    along = np.array([b for _, b in items])
    a = np.abs(error)
    print(f"share {share}: n={len(items)} mean {a.mean():.3f} p95 {np.percentile(a, 95):.3f} max {a.max():.3f}  per-wp max: mean {a.max(1).mean():.3f} p95 {np.percentile(a.max(1), 95):.3f}")
    print("  mean|e| ", a.mean(0))
    print("  max|e|  ", a.max(0))
    print("  along   ", np.nanmean(along, 0))
    print("  std     ", np.nanstd(along, 0))
