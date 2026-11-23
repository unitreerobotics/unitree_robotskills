"""Atomic JSON files and single-controller ownership on Linux."""

from __future__ import annotations

import fcntl
import json
import os
import tempfile
from pathlib import Path


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def atomic_json(path, value):
    """Write JSON so readers see either the old file or the complete new one."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class ControllerLease:
    """Exclusive lock files held for as long as one process controls the robot."""

    def __init__(self, paths):
        self.paths = sorted({str(Path(p).resolve()) for p in paths})
        self.streams = []

    def __enter__(self):
        try:
            for path in self.paths:
                Path(path).parent.mkdir(parents=True, exist_ok=True)
                stream = open(path, "a+")
                self.streams.append(stream)
                try:
                    fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as exc:
                    raise RuntimeError(f"another controller owns {path}") from exc
        except BaseException:
            self.__exit__(None, None, None)
            raise
        return self

    def __exit__(self, *_):
        for stream in reversed(self.streams):
            stream.close()
        self.streams.clear()
