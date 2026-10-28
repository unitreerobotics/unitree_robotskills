"""Named, taught points in ONE explicitly identified SLAM map. No example poses.

No hardware dependencies. Writes use an advisory process lock and atomic replace;
corrupt files / wrong map identity are errors, never an empty-map fallback.
"""
from __future__ import annotations

import json
import math
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path


def finite_pose(pose: dict) -> dict:
    values = {key: float(pose[key]) for key in ("x", "y", "yaw")}
    if not all(math.isfinite(v) for v in values.values()):
        raise ValueError("定位坐标必须为有限数")
    return values


class PlaceStore:
    def __init__(self, path, map_id: str, frame: str):
        if not map_id.strip() or not frame.strip():
            raise ValueError("请配置非空 navigation.map_id 和 frame（换图必须换 map_id/点文件）")
        self.path, self.map_id, self.frame = Path(path), map_id, frame

    def _read(self):
        if not self.path.exists():
            return {"version": 1, "map_id": self.map_id, "frame": self.frame, "places": {}}
        data = json.loads(self.path.read_text(encoding="utf-8"))
        if (data.get("version"), data.get("map_id"), data.get("frame")) != (1, self.map_id, self.frame):
            raise ValueError("点文件版本/地图/坐标系不匹配；禁止跨地图复用标定点")
        if not isinstance(data.get("places"), dict):
            raise ValueError("点文件格式无效")
        for name, entry in data["places"].items():
            self._name(name)
            finite_pose(entry)
        return data

    @staticmethod
    def _name(name):
        if not isinstance(name, str) or not name.strip() or len(name) > 80 or any(ord(c) < 32 for c in name):
            raise ValueError("点名称须为 1~80 个非控制字符")
        return name.strip()

    def list(self):
        return [{"name": name, **entry} for name, entry in self._read()["places"].items()]

    def get(self, name):
        name = self._name(name)
        entry = self._read()["places"].get(name)
        if entry is None:
            raise ValueError(f"未标定地点：{name}；先在实地调用 save_place")
        return {"name": name, **entry}

    def save(self, name, pose, *, overwrite=False, source="slamware_localization"):
        import fcntl  # Real navigation persistence is Linux-only; read/stub paths stay portable.
        name = self._name(name)
        values = finite_pose(pose)
        if pose.get("frame") != self.frame:
            raise ValueError("定位坐标系与配置不匹配，拒绝标定")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.with_suffix(self.path.suffix + ".lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            data = self._read()
            if name in data["places"] and not overwrite:
                raise ValueError("地点已存在；确认重新标定后显式使用 overwrite=true")
            replaced = name in data["places"]
            entry = {**values, "taught_at": datetime.now(timezone.utc).isoformat(), "source": source}
            data["places"][name] = entry
            self._write(data)
        return {"name": name, **entry, "map_id": self.map_id, "frame": self.frame,
                "replaced": replaced}

    def _write(self, data):
        """Caller holds the process lock; no partial JSON is visible to readers."""
        fd, temp = tempfile.mkstemp(prefix=".places-", dir=self.path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as out:
                json.dump(data, out, ensure_ascii=False, indent=2, allow_nan=False)
                out.write("\n")
                out.flush()
                os.fsync(out.fileno())
            os.replace(temp, self.path)
        finally:
            if os.path.exists(temp):
                os.unlink(temp)

