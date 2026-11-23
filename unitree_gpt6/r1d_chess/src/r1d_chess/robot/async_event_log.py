"""Best-effort event logging; disk latency never gates robot execution.

Checkpoint, decision and receipt files remain the authoritative synchronous
handoff. Only the additional audit stream is handled by this worker.
"""
import copy
from datetime import datetime
import json
from pathlib import Path
import queue
import threading


class AsyncEventLog:
    def __init__(self, path, capacity=2048):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.pending = queue.Queue(maxsize=capacity)
        self.stopping = threading.Event()
        self.dropped = 0
        self.write_errors = 0
        self.last_error = None
        self.thread = threading.Thread(target=self._run, name="robot-event-log", daemon=True)
        self.thread.start()

    def append(self, event):
        if self.stopping.is_set():
            self.dropped += 1
            return
        # Preserve the event as it was when emitted, before callers mutate it.
        event = copy.deepcopy({"time": datetime.now().isoformat(), **event})
        try:
            self.pending.put_nowait(event)
        except queue.Full:
            self.dropped += 1

    def _write(self, event):
        line = json.dumps(event, ensure_ascii=False) + "\n"
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(line)

    def _run(self):
        while not self.stopping.is_set() or not self.pending.empty():
            try:
                event = self.pending.get(timeout=.05)
            except queue.Empty:
                continue
            try:
                self._write(event)
            except Exception as exc:
                self.write_errors += 1
                self.last_error = str(exc)
            finally:
                self.pending.task_done()

    def status(self):
        return {"queued": self.pending.qsize(), "dropped": self.dropped,
                "write_errors": self.write_errors, "last_error": self.last_error}

    def close(self, timeout=2.):
        """Bounded flush during service teardown, after motor release."""
        self.stopping.set()
        self.thread.join(timeout=max(0., timeout))
        return not self.thread.is_alive()
