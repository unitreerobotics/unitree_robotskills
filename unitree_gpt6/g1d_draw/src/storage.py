"""Atomic progress, persistent attempt claims and Linux controller ownership."""

from __future__ import annotations

import fcntl
import hashlib
import json
import os
import tempfile
from pathlib import Path


def atomic_json(path, value):
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


def claim_attempt(directory, job_id, digest):
    folder = Path(directory) / "claims"
    folder.mkdir(parents=True, exist_ok=True)
    key = hashlib.sha256(job_id.encode()).hexdigest()
    path = folder / f"{key}.json"
    try:
        descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError as exc:
        raise RuntimeError(
            "this job_id already has an attempt; query status, do not replay"
        ) from exc
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        json.dump(
            {"job_id": job_id, "sha256": digest, "state": "claimed_before_acquisition"}, stream
        )
        stream.flush()
        os.fsync(stream.fileno())
    return path
