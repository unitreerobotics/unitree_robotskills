# -*- coding: utf-8 -*-
"""camera_view.py -- live robot camera view (MJPEG over HTTP, viewable directly in a browser).

Zero DSH intrusion: a separate process reuses the ZMQ subscription from services/camera
(PUB/SUB supports multiple subscribers, so the MCP server's feed is unaffected) and pushes
the latest frame to the browser as multipart/x-mixed-replace. start.sh launches it
automatically in real-robot mode; open it in a window next to the DSH page.

    python tools/camera_view.py                 # default 127.0.0.1:3081, eye = config camera.eye
    python tools/camera_view.py --port 3081 --eye full --fps 15
    python tools/camera_view.py --demo          # no camera: verify the HTTP chain with synthetic frames

Routes: /            page (left/right eyes side by side + run log below + task board on the right;
                     a monocular camera shows a single tile)
        /stream?eye= MJPEG stream (left / right / full; on R1D, right is the UDP 5003 right eye,
                     on G1D the binocular frame is split in half)
        /snapshot.jpg single frame
        /status.json  connection state, received-frame count
        /panel.json   task board (current / done / todo) + recorded rooms: proxied from the sidecar
                      observer's /state (task list written by update_tasks, places saved by save_place)
        /runlog.json?after=N  run log: proxied from the sidecar observer's /runlog (tool calls and
                      results, model utterances, observation messages, ...)
Environment variables follow driver_factory.py: NAVI_CAMERA_HOST / NAVI_CAMERA_REQUEST_PORT.

Frame source (--source; default auto = read-only model probe at startup, using the same
services.robots logic as the MCP sidecar):
  zmq       G1D: subscribe to image_server directly
  observer  R1D: pull frames from the sidecar observer at http://127.0.0.1:<navi.observer.port>/frame.jpg.
            The R1D left-eye RTP on UDP 5002 allows only one receiver (the sidecar). The right-eye RTP
            on UDP 5003 (profile camera_right_port) has no other receiver, so this page receives it
            itself (gst decoding) and shows it next to the left eye; the model sees only the left eye.
            The sidecar camera is lazy-loaded with the driver: frames appear only after the model's
            first robot-tool call.
"""
import argparse
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import config  # noqa: E402
from services import camera as camera_mod  # noqa: E402

_state = {"cam": None, "binocular": True, "eye": "left", "demo": False, "err": None,
          "source": "zmq", "model": None, "observer_port": 0, "cam_right": None}

class _DemoStream:
    """Synthetic frame source (incrementing counter + time) for verifying the HTTP/MJPEG chain."""
    connected = True
    binocular = False
    error = None

    def __init__(self):
        self.n_recv = 0

    def get_latest_frame(self):
        self.n_recv += 1
        f = np.full((480, 640, 3), 255, np.uint8)
        cv2.putText(f, f"DEMO {self.n_recv:05d}", (60, 240), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (0, 0, 0), 8)
        cv2.putText(f, time.strftime("%H:%M:%S"), (60, 400), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        ok, buf = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes() if ok else None


def _connect(cfg: dict, timeout_s: float = 10.0):
    host = str(cfg.get("host"))
    conf = camera_mod.request_camera_config(host, int(cfg.get("request_port", 60000)),
                                            int(cfg.get("request_timeout_ms", 2000)))
    if conf is None:
        raise RuntimeError(f"请求相机配置失败（{host}:{cfg.get('request_port')}）")
    entry = (conf or {}).get(cfg.get("camera", "head_camera")) or {}
    port = int(cfg.get("zmq_port") or entry.get("zmq_port") or conf.get("camera_port") or conf.get("port") or 0)
    if not port:
        raise RuntimeError("相机配置缺少端口")
    binocular = cfg.get("binocular")
    if binocular is None:
        binocular = bool(entry.get("binocular", conf.get("binocular", True)))
    cam = camera_mod.CameraStream(host, port, bool(binocular),
                                  first_frame_timeout_s=float(cfg.get("first_frame_timeout_s", 5.0)))
    cam.start()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if cam.connected:
            return cam, bool(binocular)
        if cam.error:
            raise RuntimeError(cam.error)
        time.sleep(0.1)
    raise RuntimeError(f"相机连接超时（{timeout_s:.0f}s）")


class _ObserverStream:
    """Poll /frame.jpg from the sidecar observer (read-only HTTP, holds no camera port). Same interface as CameraStream."""

    binocular = False

    def __init__(self, port: int, fps: float = 10.0, stale_s: float = 2.0):
        self.url = f"http://127.0.0.1:{int(port)}/frame.jpg"
        self.period = 1.0 / max(1.0, fps)
        self.stale_s = stale_s
        self.n_recv = 0
        self.error = None
        self._jpg = None
        self._t = 0.0
        threading.Thread(target=self._run, daemon=True).start()

    @property
    def connected(self) -> bool:
        return self._jpg is not None and time.monotonic() - self._t < self.stale_s

    def get_latest_frame(self):
        return self._jpg if self.connected else None

    def _run(self):
        while True:
            t0 = time.monotonic()
            try:
                with urllib.request.urlopen(self.url, timeout=2.0) as r:
                    jpg = r.read()
                if jpg != self._jpg:
                    self._jpg, self._t = jpg, time.monotonic()
                    self.n_recv += 1
                self.error = None
            except urllib.error.HTTPError as e:
                self.error = ("等待 sidecar 相机（模型第一次调用机器人工具后才出画面）" if e.code == 503
                              else f"观测面 HTTP {e.code}")
            except Exception:
                self.error = f"连不上 sidecar 观测面 {self.url}（Navi 未启动或 navi.observer.port=0）"
            time.sleep(max(0.0, self.period - (time.monotonic() - t0)))


def _frame_jpeg(eye: str):
    """Return the latest frame as JPEG. With a separate right-eye stream (R1D), "right" uses it directly;
    binocular frames are split by eye and re-encoded; monocular or "full" passes the original JPEG through."""
    if eye == "right" and _state["cam_right"] is not None:
        return _state["cam_right"].get_latest_frame()
    cam = _state["cam"]
    if cam is None:
        return None
    raw = cam.get_latest_frame()
    if raw is None:
        return None
    if not _state["binocular"] or eye == "full":
        return raw
    key = (id(raw), eye)
    if _render_cache[0] == key and _render_cache[1] is raw:
        return _render_cache[2]            # avoid re-decoding the same source frame; the stream dedups by object identity
    frame = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        return None
    if _state["binocular"] and eye != "full":
        half = frame.shape[1] // 2
        frame = frame[:, :half] if eye == "left" else frame[:, half:]
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    out = buf.tobytes() if ok else None
    _render_cache[:] = [key, raw, out]
    return out


_render_cache = [None, None, None]         # [(id(raw), eye), raw, jpeg]


def _pick_source(requested: str):
    """auto -> read-only model probe: R1D (camera_kind=r1d_rtp) uses observer, others zmq. Returns (source, profile|None)."""
    if requested != "auto":
        return requested, None
    try:
        from services.robots import resolve_profile
        cfg = config.load_config()
        config.apply_config(cfg)
        profile = resolve_profile(cfg)
    except Exception as e:
        print(f"camera view: 机型探测失败，按 zmq：{type(e).__name__}: {e}", flush=True)
        return "zmq", None
    return ("observer" if profile.camera_kind == "r1d_rtp" else "zmq"), profile


def _panel():
    """Task board + recorded rooms: proxy task / places from the sidecar observer's /state (written by the
    model via update_tasks / save_place; places only include points taught in this run)."""
    port = _state["observer_port"]
    if port <= 0:
        return {"ok": False, "error": "navi.observer.port=0，看板不可用"}
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/state", timeout=1.0) as r:
            st = json.loads(r.read())
    except Exception:
        return {"ok": False, "error": "连不上 Navi（sidecar 还没启动）"}
    return {"ok": True, "task": st.get("task") or {"tasks": []}, "places": st.get("places")}


def _runlog(after: int):
    """Run log: proxy the sidecar observer's /runlog?after=N (tool call/result summaries with seq >= N)."""
    port = _state["observer_port"]
    if port <= 0:
        return {"ok": False, "error": "navi.observer.port=0，运行日志不可用"}
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/runlog?after={int(after)}", timeout=1.0) as r:
            return {"ok": True, **json.loads(r.read())}
    except Exception:
        return {"ok": False, "error": "连不上 Navi（sidecar 还没启动）"}


_PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>Navi 相机</title>
<style>body{margin:0;background:#111;color:#ddd;font:18px system-ui;display:flex;flex-direction:column;height:100vh}
#bar{padding:4px 12px;display:flex;gap:12px;align-items:center;background:#222;font-size:14px}
#st{margin-left:auto;color:#8a8}
#main{flex:1;min-height:0;display:flex}
#left{flex:1;min-width:0;display:flex;flex-direction:column}
#cam{flex:1;min-height:0;display:grid;grid-template-columns:repeat(%N,minmax(0,1fr));grid-template-rows:minmax(0,1fr);gap:2px;background:#000}
.tile{position:relative;min-width:0;min-height:0}
.tile img{position:absolute;inset:0;width:100%;height:100%;object-fit:fill}
.tile span{position:absolute;left:8px;top:6px;padding:1px 8px;border-radius:4px;background:#000a;color:#ddd;font-size:14px}
#logbox{height:36%;flex:none;display:flex;flex-direction:column;background:#0c0c0c;border-top:1px solid #333}
#logbox h3{margin:0;padding:5px 12px;font-size:15px;color:#9cf;font-weight:600;background:#151515}
#log{flex:1;overflow:auto;padding:4px 12px 8px;font:15px/1.5 ui-monospace,Menlo,Consolas,monospace}
#log div{white-space:pre-wrap;word-break:break-all}
#log .tm{color:#666;margin-right:10px}#log .call{color:#9cf}#log .result{color:#8b8}#log .bad{color:#f88}
#log .say{color:#eee}#log .think{color:#888;font-style:italic}#log .user{color:#fd6;font-weight:700}
#log .observe{color:#fb6}#log .goal{color:#c9f}#log .stop,#log .error{color:#f66}
#side{width:420px;flex:none;overflow:auto;padding:6px 18px 16px;background:#181818;border-left:1px solid #333;box-sizing:border-box}
#side h3{margin:16px 0 6px;font-size:16px;color:#9cf;font-weight:600}
#doing{font-size:26px;font-weight:700;color:#fd6;line-height:1.35}
ul{list-style:none;margin:0;padding:0}li{padding:3px 0;line-height:1.4}
li.done{color:#7d7}li.dropped{color:#f99}li.todo{color:#aaa}
.note{color:#bbb;font-size:16px;font-weight:400;margin-left:6px}
.chip{display:inline-block;margin:0 8px 8px 0;padding:4px 12px;border:1px solid #456;border-radius:16px;background:#123}
.chip small{color:#8ab;font-size:13px;margin-left:4px}
.dim{color:#777;font-size:16px;font-weight:400}
@media (max-width:900px){#main{flex-direction:column}#side{width:auto;max-height:40vh;border-left:none;border-top:1px solid #333}}
</style></head><body>
<div id="bar"><b>Navi 相机</b><span id="st">…</span></div>
<div id="main">
 <div id="left">
  <div id="cam">%T</div>
  <div id="logbox"><h3>运行日志 <span class="dim" id="logst"></span></h3><div id="log"></div></div>
 </div>
 <div id="side">
  <h3>当前任务</h3><div id="doing">…</div>
  <h3>已完成 <span id="ndone"></span></h3><ul id="done"></ul>
  <h3>待做</h3><ul id="todo"></ul>
  <h3>已记录房间</h3><div id="places"></div>
 </div>
</div>
<script>
const $=id=>document.getElementById(id);
const esc=s=>String(s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));
const note=t=>t.note?'<span class="note">— '+esc(t.note)+'</span>':'';
const hm=iso=>{const d=new Date(iso);return isNaN(d)?'':d.toLocaleTimeString('zh-CN',{hour:'2-digit',minute:'2-digit'});};
const items=(ts,mark)=>ts.map(t=>'<li class="'+t.status+'">'+mark(t)+esc(t.name)+note(t)+'</li>').join('')||'<li class="dim">—</li>';
async function drawStatus(){try{const s=await (await fetch('/status.json')).json();
$('st').textContent=(s.connected?'● 已连接':'○ 未连接')+'  帧 '+s.n_recv
 +(s.source==='observer'?'  左目=模型画面':'')+(s.error?'  '+s.error:'')
 +(s.right?'   右目 '+(s.right.connected?'●':'○')+' 帧 '+s.right.n_recv+(s.right.error?'  '+s.right.error:''):'');}catch(e){}}
setInterval(drawStatus,1000);drawStatus();
async function drawPanel(){
 try{
  const p=await (await fetch('/panel.json')).json();
  const ts=(p.task&&p.task.tasks)||[];
  const doing=ts.filter(t=>t.status==='doing'),todo=ts.filter(t=>t.status==='todo');
  const closed=ts.filter(t=>t.status==='done'||t.status==='dropped');
  $('doing').innerHTML=doing.map(t=>'<div>'+esc(t.name)+note(t)+'</div>').join('')
   ||'<span class="dim">'+(!p.ok?esc(p.error):ts.length?'（没有进行中的任务）':'等待模型下发任务清单')+'</span>';
  $('ndone').textContent=ts.length?closed.filter(t=>t.status==='done').length+' / '+ts.length:'';
  $('done').innerHTML=items(closed,t=>t.status==='done'?'✓ ':'✗ ');
  $('todo').innerHTML=items(todo,t=>'· ');
  const pl=p.places;
  $('places').innerHTML=pl==null?'<span class="dim">'+(p.ok?'雷达定位还没启动':'—')+'</span>'
   :pl.map(q=>'<span class="chip">'+esc(q.name)+'<small>'+hm(q.taught_at)+'</small></span>').join('')||'<span class="dim">暂无</span>';
 }catch(e){}
}
setInterval(drawPanel,1000);drawPanel();
let logNext=0,logBusy=false;
const TAGS={call:'▶ ',result:'  ↳ ',say:'【模型】',think:'【思考】',user:'【用户】',observe:'【观测】',goal:'【目标】',stop:'【停止】',error:'【出错】'};
const hms=t=>new Date(t*1000).toLocaleTimeString('zh-CN',{hour12:false});
async function drawLog(){
 if(logBusy)return;logBusy=true;
 try{
  const r=await (await fetch('/runlog.json?after='+logNext)).json(),box=$('log');
  $('logst').textContent=r.ok?(r.error?'  '+r.error:''):'  '+r.error;
  if(!r.ok)return;
  if(r.n<logNext){box.innerHTML='';logNext=0;return;}          // Navi restarted: fetch from the beginning
  const atEnd=box.scrollTop+box.clientHeight>=box.scrollHeight-30;
  for(const e of r.entries){
   const d=document.createElement('div'),tag=TAGS[e.kind]??'['+e.kind+'] ';
   const cls=e.kind==='result'?(e.ok===false?'bad':'result'):e.kind;
   d.innerHTML='<span class="tm">'+hms(e.t)+'</span><span class="'+cls+'">'+tag+esc(e.text)+'</span>';
   box.appendChild(d);
  }
  while(box.childElementCount>400)box.removeChild(box.firstChild);
  logNext=r.n;
  if(atEnd&&r.entries.length)box.scrollTop=box.scrollHeight;
 }catch(e){}finally{logBusy=false;}
}
setInterval(drawLog,1000);drawLog();
</script>
</body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):  # silence access log
        pass

    def _eye(self, q):
        e = (q.get("eye") or [_state["eye"]])[0]
        return e if e in ("left", "right", "full") else _state["eye"]

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path == "/":
            stereo = _state["cam_right"] is not None or _state["binocular"]
            tiles = (['<div class="tile"><img src="/stream?eye=left" alt="left"><span>左目</span></div>',
                      '<div class="tile"><img src="/stream?eye=right" alt="right"><span>右目</span></div>'] if stereo
                     else [f'<div class="tile"><img src="/stream?eye={self._eye(q)}" alt="camera"></div>'])
            body = _PAGE.replace("%T", "".join(tiles)).replace("%N", str(len(tiles))).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif u.path == "/status.json":
            cam, right = _state["cam"], _state["cam_right"]
            body = json.dumps({"right": None if right is None else
                               {"connected": bool(right.connected), "n_recv": right.n_recv, "error": right.error},
                               "connected": bool(cam and cam.connected),
                               "n_recv": getattr(cam, "n_recv", 0),
                               "binocular": _state["binocular"], "demo": _state["demo"],
                               "source": _state["source"], "model": _state["model"],
                               "error": _state["err"] or getattr(cam, "error", None)}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif u.path == "/panel.json":
            body = json.dumps(_panel(), ensure_ascii=False).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        elif u.path == "/runlog.json":
            try:
                after = max(0, int((q.get("after") or ["0"])[0]))
            except ValueError:
                after = 0
            body = json.dumps(_runlog(after), ensure_ascii=False).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        elif u.path == "/snapshot.jpg":
            jpg = _frame_jpeg(self._eye(q))
            if jpg is None:
                self.send_error(503, "no frame")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(jpg)
        elif u.path == "/stream":
            self._stream(self._eye(q))
        else:
            self.send_error(404)

    def _stream(self, eye):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        period = 1.0 / max(1.0, float(self.server.fps))
        last = None
        try:
            while True:
                t0 = time.monotonic()
                jpg = _frame_jpeg(eye)
                if jpg is not None and jpg is not last:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    self.wfile.flush()
                    last = jpg
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    ap = argparse.ArgumentParser(description="Navi live camera view (MJPEG)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=3081)
    ap.add_argument("--eye", default=None, help="left/right/full; defaults to config camera.eye")
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--demo", action="store_true", help="no camera: verify the chain with synthetic frames")
    ap.add_argument("--source", choices=["auto", "zmq", "observer"], default="auto",
                    help="frame source: auto=by model (R1D->observer, G1D->zmq) | zmq=image_server | observer=sidecar observer")
    ap.add_argument("--observer-port", type=int, default=None, help="sidecar observer port (default navi.observer.port)")
    ap.add_argument("--right-port", type=int, default=None,
                    help="local UDP port for R1D right-eye RTP (default profile camera_right_port=5003; 0 = hide right eye)")
    a = ap.parse_args()

    cfg = config.load_camera_defaults(config.load_config())
    if os.environ.get("NAVI_CAMERA_HOST"):
        cfg["host"] = os.environ["NAVI_CAMERA_HOST"]
    if os.environ.get("NAVI_CAMERA_REQUEST_PORT"):
        cfg["request_port"] = int(os.environ["NAVI_CAMERA_REQUEST_PORT"])
    _state["eye"] = a.eye or cfg.get("eye", "left")

    profile = None
    if a.demo:
        _state.update(cam=_DemoStream(), binocular=False, demo=True)
    else:
        source, profile = _pick_source(a.source)
        _state.update(source=source, model=getattr(profile, "model", None))
    # Observer port: R1D pulls frames from it; all models read the task board / recorded rooms from it.
    port = a.observer_port or int(config.navi_section(config.load_config(), "observer").get("port", 0) or 0)
    _state["observer_port"] = port
    if not a.demo and _state["source"] == "observer":
        if port <= 0:
            raise SystemExit("camera view: navi.observer.port=0，observer 来源不可用（改用 --source zmq）")
        _state.update(cam=_ObserverStream(port, fps=min(a.fps, 10.0)), binocular=False)
        extra = getattr(profile, "extra", None) or {}
        right_port = a.right_port if a.right_port is not None else int(extra.get("camera_right_port") or 0)
        if right_port > 0:
            # The right eye has no other receiver, so receive it here (stream only; never toggles robot
            # services). The sidecar enables the service on the model's first tool call, so wait for the
            # first frame indefinitely instead of giving up after the default 15 s.
            from services.rtp_camera import RtpCameraStream
            right = RtpCameraStream(str(extra.get("camera_host", "0.0.0.0")), right_port,
                                    first_frame_timeout_s=float("inf"))
            right.start()
            _state["cam_right"] = right
    elif not a.demo:
        def _bg():
            # retry in the background on disconnect; the page status shows the error instead of exiting
            while True:
                try:
                    cam, bino = _connect(cfg)
                    _state.update(cam=cam, binocular=bino, err=None)
                    while cam.connected and not cam.error:
                        time.sleep(1.0)
                    _state["err"] = cam.error or "相机流中断"
                except Exception as e:
                    _state["err"] = str(e)
                time.sleep(3.0)
        threading.Thread(target=_bg, daemon=True).start()

    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    srv.daemon_threads = True
    srv.fps = a.fps
    where = ("demo" if a.demo else f"sidecar 观测面 {_state['cam'].url}" if _state["source"] == "observer"
             else cfg["host"])
    print(f"camera view: http://{a.host}:{a.port}/  （机型={_state['model'] or '-'}，眼={_state['eye']}，{where}）",
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
