# -*- coding: utf-8 -*-
"""observer.py -- observation status surface (embedded HTTP, polled by the DSH host plugin navi-observer).

The MCP sidecar is a stdio subprocess, so the host plugin (TypeScript) cannot reach its memory.
This module exposes "what the robot is doing and what is ahead" as a read-only HTTP surface
that the plugin polls while the model is thinking and injects as notices/images:

  GET /state      status JSON (sequence progress + latency estimate + lidar clearance + odometry)
  GET /frame.jpg  current camera frame (JPEG, latest frame of the stream; never contends
                  with tool calls for locks)
  GET /           pretty-printed status (debugging)
  Extra read-only JSON routes can be injected (routes={path: fn(query dict)}), e.g. /runlog?after=N;
  and POST routes (post_routes={path: fn(JSON body)}) that only accept display content
  supplied by the host plugin (e.g. what the model said, for the run log). No request that
  controls the robot is ever accepted.

Bound to 127.0.0.1 only: the surface is for local host processes. state_fn/frame_fn are
injected by the caller; this module imports no other internal modules and runs standalone.
"""
from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    # Injection points, patched onto a subclass by ObserverServer (avoids rebuilding per request)
    state_fn = staticmethod(lambda: {})
    frame_fn = staticmethod(lambda: None)
    routes: dict = {}
    post_routes: dict = {}
    log = staticmethod(lambda *a, **k: None)

    def log_message(self, fmt, *args):      # silence the default stderr access log
        pass

    def _send(self, code, body: bytes, ctype: str):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):                       # name required by BaseHTTPRequestHandler
        path, _, query = self.path.partition("?")
        try:
            if path in self.routes:
                params = {k: v[0] for k, v in parse_qs(query).items()}
                body = json.dumps(self.routes[path](params), ensure_ascii=False).encode("utf-8")
                self._send(200, body, "application/json; charset=utf-8")
            elif path in ("/state", "/state.json"):
                body = json.dumps(self.state_fn(), ensure_ascii=False).encode("utf-8")
                self._send(200, body, "application/json; charset=utf-8")
            elif path in ("/frame.jpg", "/frame"):
                jpg = self.frame_fn()
                if not jpg:
                    self._send(503, b'{"ok":false,"error":"no frame"}',
                               "application/json; charset=utf-8")
                else:
                    self._send(200, jpg, "image/jpeg")
            elif path == "/":
                self._send(200, json.dumps(self.state_fn(), ensure_ascii=False,
                                           indent=2).encode("utf-8"),
                           "application/json; charset=utf-8")
            else:
                self._send(404, b'{"ok":false,"error":"not found"}',
                           "application/json; charset=utf-8")
        except Exception as e:              # never propagate exceptions to the host
            self.log(f"observer 请求 {path} 失败: {type(e).__name__}: {e}", "error")
            try:
                self._send(500, b'{"ok":false,"error":"internal"}',
                           "application/json; charset=utf-8")
            except Exception:
                pass


    def do_POST(self):                      # noqa: N802
        path = self.path.split("?", 1)[0]
        try:
            n = int(self.headers.get("Content-Length") or 0)
            if path not in self.post_routes or not 0 < n <= _MAX_POST:
                self.close_connection = True    # body left unread, so the connection cannot be reused
                self._send(404 if path not in self.post_routes else 413,
                           b'{"ok":false,"error":"not accepted"}', "application/json; charset=utf-8")
                return
            body = json.loads(self.rfile.read(n).decode("utf-8"))
            out = json.dumps(self.post_routes[path](body), ensure_ascii=False).encode("utf-8")
            self._send(200, out, "application/json; charset=utf-8")
        except Exception as e:              # as in do_GET: never propagate to the host
            self.log(f"observer POST {path} 失败: {type(e).__name__}: {e}", "error")
            try:
                self._send(400, b'{"ok":false,"error":"bad request"}', "application/json; charset=utf-8")
            except Exception:
                pass


_MAX_POST = 64 * 1024


class ObserverServer:
    """Read-only status surface. start() never raises (sets `error`) and returns whether it is running."""

    def __init__(self, state_fn, frame_fn, *, host="127.0.0.1", port=3082, log=None, routes=None,
                 post_routes=None):
        self.state_fn = state_fn
        self.frame_fn = frame_fn
        self.routes = dict(routes or {})
        self.post_routes = dict(post_routes or {})
        self.host = host
        self.port = int(port)
        self._log = log or (lambda *a, **k: None)
        self._httpd = None
        self._thread = None
        self.error: str | None = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> bool:
        if self.running:
            return True
        handler = type("_BoundHandler", (_Handler,), {
            "state_fn": staticmethod(self.state_fn),
            "frame_fn": staticmethod(self.frame_fn),
            "routes": dict(self.routes),          # functions stored in a dict are not bound to self
            "post_routes": dict(self.post_routes),
            "log": staticmethod(self._log)})
        try:
            self._httpd = ThreadingHTTPServer((self.host, self.port), handler)
        except Exception as e:
            self.error = f"观测面启动失败: {type(e).__name__}: {e}"
            self._log(self.error, "error")
            return False
        self._httpd.daemon_threads = True
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        self._log(f"观测面已监听 http://{self.host}:{self.port}/state", "observer")
        return True

    def stop(self):
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
        self._httpd = None
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None
