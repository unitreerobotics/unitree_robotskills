#!/usr/bin/env python3
"""Selective TeleImager ZMQ capture, JPEG snapshots, and a browser viewer."""

import argparse
import errno
import json
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import cv2
import numpy as np
import zmq

from .common import load_config, positive

VIEWS = {
    "cam_left_high": "head",
    "cam_right_high": "head",
    "cam_left_wrist": "left_wrist",
    "cam_right_wrist": "right_wrist",
}


class CameraCapture:
    def __init__(self, config):
        self.config = config
        if not isinstance(config.get("host"), str) or not config["host"].strip():
            raise ValueError("Set camera.host to the address of your camera publisher")
        enabled = config.get("enabled")
        if not isinstance(enabled, (list, tuple)) or any(
            not isinstance(name, str) for name in enabled
        ):
            raise ValueError("camera.enabled must contain supported camera names")
        self.enabled = tuple(enabled)
        if not self.enabled or len(set(self.enabled)) != len(self.enabled):
            raise ValueError("camera.enabled must be nonempty and contain no duplicates")
        if set(self.enabled) - VIEWS.keys():
            raise ValueError(f"Unknown camera names: {set(self.enabled) - VIEWS.keys()}")
        self.stale_after = positive(config.get("stale_after", 2), "stale_after")
        self.quality = int(config.get("jpeg_quality", 90))
        if not 1 <= self.quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")
        self.sources = {VIEWS[name] for name in self.enabled}
        ports = config.get("ports", {})
        for source in self.sources:
            if (
                not isinstance(ports, dict)
                or type(ports.get(source)) is not int
                or not 1 <= ports[source] <= 65535
            ):
                raise ValueError(f"camera.ports.{source} must be an integer TCP port")
        self.endpoints = {
            source: f"tcp://{config['host']}:{ports[source]}" for source in self.sources
        }
        self._frames = {}
        self._errors = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads = []
        self._context = None

    def start(self):
        if self._context is not None:
            raise RuntimeError("CameraCapture already started")
        self._stop.clear()
        self._context = zmq.Context()
        for source in sorted(self.sources):
            worker = threading.Thread(target=self._receive, args=(source,), daemon=True)
            self._threads.append(worker)
            worker.start()
        return self

    def _receive(self, source):
        # Each socket is created, used and closed by its owning thread.
        sock = self._context.socket(zmq.SUB)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        try:
            sock.connect(self.endpoints[source])
            while not self._stop.is_set():
                if not sock.poll(100):
                    continue
                raw = sock.recv()
                try:
                    decoded = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
                    if decoded is None:
                        raise ValueError("Invalid JPEG frame")
                    self._store(source, decoded)
                except (ValueError, cv2.error) as exc:
                    with self._lock:
                        self._errors[source] = str(exc)
        except Exception as exc:
            logging.exception("Camera source %s stopped", source)
            with self._lock:
                self._errors[source] = str(exc)
        finally:
            sock.close(0)

    def _store(self, source, bgr):
        if source == "head" and bgr.shape[1] % 2:
            raise ValueError("Binocular head image width must be even")
        now, timestamp = time.monotonic(), time.time()
        frames = {}
        for name in self.enabled:
            if VIEWS[name] != source:
                continue
            view = bgr
            if source == "head":
                mid = bgr.shape[1] // 2
                view = bgr[:, :mid] if name == "cam_left_high" else bgr[:, mid:]
            ok, jpeg = cv2.imencode(".jpg", view, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if not ok:
                raise ValueError(f"Cannot encode {name}")
            frames[name] = (jpeg.tobytes(), now, timestamp, view.shape[:2])
        with self._lock:
            self._frames.update(frames)
            self._errors.pop(source, None)

    def jpeg(self, name):
        if name not in self.enabled:
            raise KeyError(name)
        with self._lock:
            frame = self._frames.get(name)
        if frame is None or time.monotonic() - frame[1] > self.stale_after:
            return None
        return frame[0]

    def status(self):
        with self._lock:
            now = time.monotonic()
            return {
                name: {
                    "available": name in self._frames
                    and now - self._frames[name][1] <= self.stale_after,
                    "age_seconds": now - self._frames[name][1] if name in self._frames else None,
                    "received_at": self._frames[name][2] if name in self._frames else None,
                    "shape": self._frames[name][3] if name in self._frames else None,
                    "error": self._errors.get(VIEWS[name]),
                }
                for name in self.enabled
            }

    def close(self):
        self._stop.set()
        for worker in self._threads:
            worker.join()
        self._threads.clear()
        if self._context is not None:
            self._context.term()
            self._context = None

    def snapshot(self, directory, timeout=5):
        deadline = time.monotonic() + positive(timeout, "timeout")
        while True:
            frames = {name: self.jpeg(name) for name in self.enabled}
            if all(frame is not None for frame in frames.values()):
                break
            if time.monotonic() >= deadline:
                missing = [name for name, frame in frames.items() if frame is None]
                raise TimeoutError(f"No fresh image from: {missing}")
            self._stop.wait(0.02)
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        result = {}
        for name, jpeg in frames.items():
            path = directory / f"{name}.jpg"
            path.write_bytes(jpeg)
            result[name] = str(path.resolve())
        return result


def make_server(capture, host, port, fps=15):
    interval = 1 / positive(fps, "web_fps")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, status, content_type, payload):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            try:
                self.route()
            except (BrokenPipeError, ConnectionResetError):
                pass

        def route(self):
            path = urlsplit(self.path).path
            if path == "/":
                cards = "".join(
                    f'<article><h2>{n}</h2><img src="/stream/{n}"><p id="{n}">等待图像…</p>'
                    f'<a href="/snapshot/{n}.jpg" target="_blank">查看原尺寸图像</a></article>'
                    for n in capture.enabled
                )
                page = (
                    """<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>机器人相机</title><style>
body{font:16px system-ui;background:#101827;color:#e5edf6;margin:24px}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:20px}
article{background:#1e293b;padding:16px;border-radius:12px}img{width:100%;min-height:160px;object-fit:contain}
h2{font-size:18px}a{color:#7dd3fc}.stale{opacity:.2}
</style><h1>机器人相机</h1><main>"""
                    + cards
                    + """</main><script>
async function refresh(){try{const r=await fetch('/api/cameras');const data=await r.json();
for(const [name,s] of Object.entries(data)){const p=document.getElementById(name);
p.textContent=s.available?'在线 · '+s.shape[1]+' × '+s.shape[0]:'无信号 / 图像过期';
p.parentNode.querySelector('img').classList.toggle('stale',!s.available);}}
catch(e){document.querySelectorAll('p').forEach(p=>p.textContent='服务已断开');}}
setInterval(refresh,1000);refresh();</script></html>"""
                )
                self.reply(200, "text/html; charset=utf-8", page.encode())
            elif path == "/api/cameras":
                self.reply(200, "application/json", json.dumps(capture.status()).encode())
            elif path.startswith("/snapshot/"):
                name = path[len("/snapshot/") :].removesuffix(".jpg")
                if name not in capture.enabled:
                    self.reply(404, "text/plain", b"Camera not enabled")
                    return
                frame = capture.jpeg(name)
                self.reply(
                    200 if frame else 503,
                    "image/jpeg" if frame else "text/plain",
                    frame or b"No fresh image",
                )
            elif path.startswith("/stream/"):
                name = path[len("/stream/") :]
                if name not in capture.enabled:
                    self.reply(404, "text/plain", b"Camera not enabled")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                previous = None
                while not capture._stop.is_set():
                    frame = capture.jpeg(name)
                    if frame is not None and frame is not previous:
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                            + str(len(frame)).encode()
                            + b"\r\n\r\n"
                            + frame
                            + b"\r\n"
                        )
                        self.wfile.flush()
                        previous = frame
                    capture._stop.wait(interval)
            else:
                self.reply(404, "text/plain", b"Not found")

    return ThreadingHTTPServer((host, port), Handler)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--config")
    parser.add_argument("--cameras", nargs="+", choices=tuple(VIEWS))
    parser.add_argument("--host", help="Camera server IP")
    parser.add_argument("--web-host")
    parser.add_argument("--web-port", type=int)
    parser.add_argument("--snapshot", metavar="DIR", help="Save enabled views and exit")
    parser.add_argument("--timeout", type=float, default=5)
    args = parser.parse_args(argv)
    cfg = load_config(args.config)["camera"]
    for key, value in {
        "enabled": args.cameras,
        "host": args.host,
        "web_host": args.web_host,
        "web_port": args.web_port,
    }.items():
        if value is not None:
            cfg[key] = value
    capture = CameraCapture(cfg)
    server = None
    try:
        if not args.snapshot:
            try:
                server = make_server(
                    capture, cfg["web_host"], cfg["web_port"], cfg.get("web_fps", 15)
                )
            except OSError as exc:
                if exc.errno != errno.EADDRINUSE:
                    raise
                parser.exit(
                    1,
                    f"网页地址 {cfg['web_host']}:{cfg['web_port']} 已被占用。\n"
                    "若已有相机服务，请直接打开其网页；需要更换相机配置时，"
                    "先在原终端按 Ctrl+C 停止服务。\n"
                    "也可添加 --web-port 8081 使用另一端口，或 --web-port 0 自动分配空闲端口。\n",
                )
        capture.start()
        if args.snapshot:
            print(
                json.dumps(
                    capture.snapshot(args.snapshot, args.timeout), ensure_ascii=False, indent=2
                )
            )
        else:
            print(f"Camera viewer: http://{cfg['web_host']}:{server.server_port}", flush=True)
            print(f"Enabled: {', '.join(capture.enabled)}", flush=True)
            server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        capture.close()
        if server:
            server.server_close()


if __name__ == "__main__":
    main()
