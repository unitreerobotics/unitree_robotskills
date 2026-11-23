"""Save one PNG frame from every configured camera. Read-only: no command is sent.

python capture_images.py --config device.json --output ./captures/view-1
"""

import argparse
import json
import time
from pathlib import Path

from unitree_gpt6.r1d_draw.src.config import load_config
from unitree_gpt6.r1d_draw.src.robot import connect, png_size


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True, help="new directory for the images")
    args = parser.parse_args(argv)
    config = load_config(args.config)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    robot, report = connect(config), {}
    try:
        for name in config["cameras"]:
            try:
                png = robot.photo(name)
                (output / f"{name}.png").write_bytes(png)
                report[name] = {
                    "file": f"{name}.png",
                    "size": list(png_size(png)),
                    "time_unix": time.time(),
                }
            except (RuntimeError, OSError, TimeoutError) as exc:
                report[name] = {"error": str(exc)}
    finally:
        robot.close()
    (output / "capture.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 1 if any("error" in item for item in report.values()) else 0


if __name__ == "__main__":
    raise SystemExit(main())
