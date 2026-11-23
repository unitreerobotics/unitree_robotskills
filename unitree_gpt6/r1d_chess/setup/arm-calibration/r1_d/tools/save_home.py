"""Save the robot's current pose and camera images as the home pose; read-only.

  save_home.py [--port 8080]

Reads joints, grippers and lift height from the running web panel (run.py web) and one
image from every camera; sends nothing to the robot. Writes assets/poses/home_<time>/ (pose.json
and the images) and copies pose.json to assets/poses/home.json: the start pose of every task, where every home goes.
"""

import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
import sys
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from r1_d.config import HOME_POSE, POSE_DIR
from r1_d.sdk.camera_relay import STREAMS
from r1_d.sdk.model import GROUPS, load_home_pose

# Arms, grippers and the lift must be at rest when the pose is taken (rad/s).
STILL = 0.05
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def get(url):
    with OPENER.open(url, timeout=5) as response:
        return response.status, response.read()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, default=8080)
    args = p.parse_args()
    base = f"http://127.0.0.1:{args.port}/api/"
    snap = json.loads(get(base + "state")[1])
    if snap.get("phase") == "offline" or "motors" not in snap:
        raise SystemExit("面板未连接机器人，无法读取姿态")
    age = (snap.get("feedback_age") or {}).get("rt/lowstate")
    if age is None or age > 0.5:
        raise SystemExit(f"关节反馈过旧（{age} s）")
    if snap.get("height_mm") is None or not snap.get("lift_ok"):
        raise SystemExit("升降高度不可用或有报警")
    motors = snap["motors"]
    if any(m["lost"] for name in GROUPS for m in motors[name]):
        raise SystemExit("有电机掉线，读数不可信")
    moving = max(abs(m["dq"]) for name in GROUPS for m in motors[name])
    if moving > STILL:
        raise SystemExit(f"双臂或夹爪在运动（{moving:.3f} rad/s）")
    pose = {
        "time": datetime.now().isoformat(timespec="seconds"),
        "q": {name: [m["q"] for m in motors[name]] for name in GROUPS},
        "height": snap["height_mm"] / 1000.0,
        "hold": [m["q"] for m in motors["hold"]],
        "units": "q in rad (grippers in motor units 0..5.4), height in m",
    }
    folder = POSE_DIR / f"home_{datetime.now():%Y%m%d_%H%M%S}"
    folder.mkdir(parents=True)
    (folder / "pose.json").write_text(json.dumps(pose, indent=2, ensure_ascii=False) + "\n")
    load_home_pose(folder / "pose.json")  # same checks the control session applies
    saved = []
    for name in STREAMS:
        try:
            status, data = get(base + f"camera/{name}")
        except OSError as exc:
            print(f"图像 {name} 读取失败：{exc}")
            continue
        if status == 200 and data:
            (folder / f"{name}.jpg").write_bytes(data)
            saved.append(name)
        else:
            print(f"图像 {name} 暂无画面")
    shutil.copyfile(folder / "pose.json", HOME_POSE)
    print(f"已保存：{folder}（图像 {len(saved)}/{len(STREAMS)}）")
    print(f"归位姿态：{HOME_POSE}")
    print(json.dumps(pose, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
