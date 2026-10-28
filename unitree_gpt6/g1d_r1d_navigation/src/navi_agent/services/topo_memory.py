# -*- coding: utf-8 -*-
"""topo_memory.py -- visual topological memory (SQLite): places with multi-view evidence images,
landmarks, frontiers and traversed directed edges.

Pipeline: 16x16 grayscale signature similarity recalls candidates -> the model confirms by
comparing images; BFS provides route hints; items can be marked unavailable. Notes:
  - Own schema (user_version starts at 1); no full runs/observations archive, only the
    evidence images that are referenced;
  - Edge evidence is a model-written route summary (via): a hint only, never replayed;
  - Directories are split by <robot model>/<venue> (camera height/viewpoint differ per
    model, so thumbnails are not comparable); meta is re-checked to catch manual copy mistakes;
  - The anchor (current place) lives in process memory only: after restart the robot must
    localize again, so a stale anchor never creates a wrong edge.

The tool layer (register_memory_tools) accepts only frame_id, never model-supplied image
paths; evidence frames come from the frame cache registered by observe, which prevents
"look at one frame, save another". Relation to places.json metric points: a place may link a
metric_place; navigate_to is allowed only if linked and the model has SLAMWARE, otherwise pilot.
"""
from __future__ import annotations

import json
import os
import sqlite3
import threading
import time
import uuid
from collections import deque
from pathlib import Path

import cv2
import numpy as np

SCHEMA_VERSION = 1
SIG_SIZE = 16
DIRECTIONS = ("forward", "left", "right", "back")
KINDS = ("place", "landmark", "frontier", "edge")
NEW_PLACE_HINT = 0.6          # best score below this hints "likely a new place" (coarse signature, hint only)

_SCHEMA = """
CREATE TABLE IF NOT EXISTS meta(key TEXT PRIMARY KEY, value TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS places(
    id TEXT PRIMARY KEY, label TEXT NOT NULL, aliases_json TEXT NOT NULL DEFAULT '[]',
    status TEXT NOT NULL DEFAULT 'active', evidence TEXT NOT NULL DEFAULT '',
    metric_place TEXT, updated_at REAL NOT NULL);
CREATE TABLE IF NOT EXISTS views(
    id TEXT PRIMARY KEY, place_id TEXT NOT NULL REFERENCES places(id),
    signature BLOB NOT NULL, created_at REAL NOT NULL);
CREATE TABLE IF NOT EXISTS landmarks(
    id TEXT PRIMARY KEY, place_id TEXT NOT NULL REFERENCES places(id), label TEXT NOT NULL,
    status TEXT NOT NULL DEFAULT 'active', evidence TEXT NOT NULL DEFAULT '',
    view_id TEXT REFERENCES views(id), updated_at REAL NOT NULL, UNIQUE(place_id, label));
CREATE TABLE IF NOT EXISTS frontiers(
    id TEXT PRIMARY KEY, place_id TEXT NOT NULL REFERENCES places(id), label TEXT NOT NULL,
    direction TEXT NOT NULL, status TEXT NOT NULL DEFAULT 'open', evidence TEXT NOT NULL DEFAULT '',
    view_id TEXT REFERENCES views(id), destination_id TEXT REFERENCES places(id),
    updated_at REAL NOT NULL, UNIQUE(place_id, label));
CREATE TABLE IF NOT EXISTS edges(
    id TEXT PRIMARY KEY, from_place TEXT NOT NULL REFERENCES places(id),
    to_place TEXT NOT NULL REFERENCES places(id), status TEXT NOT NULL DEFAULT 'traversed',
    evidence TEXT NOT NULL DEFAULT '', traversals INTEGER NOT NULL DEFAULT 1,
    updated_at REAL NOT NULL, UNIQUE(from_place, to_place));
"""


def signature(jpeg: bytes) -> bytes:
    """16x16 grayscale signature normalised by mean/std (insensitive to global brightness)."""
    img = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError("证据帧解码失败")
    small = cv2.resize(img, (SIG_SIZE, SIG_SIZE), interpolation=cv2.INTER_AREA).astype(np.float32)
    norm = (small - small.mean()) * 32 / max(float(small.std()), 1.0) + 128
    return np.clip(norm, 0, 255).astype(np.uint8).tobytes()


def similarity(a: bytes, b: bytes) -> float:
    """Signature similarity in [0, 1]: 1 - mean absolute difference / 128."""
    x = np.frombuffer(a, dtype=np.uint8).astype(np.float32)
    y = np.frombuffer(b, dtype=np.uint8).astype(np.float32)
    return max(0.0, 1.0 - float(np.abs(x - y).mean()) / 128.0)


def _norm(text) -> str:
    return "".join(str(text or "").split()).lower()


def _text(value, name: str, *, required=False, limit=200) -> str:
    if value is None:
        value = ""
    if not isinstance(value, str):
        raise ValueError(f"{name} 须为字符串")
    value = value.strip()
    if required and not value:
        raise ValueError(f"{name} 不能为空")
    if len(value) > limit:
        raise ValueError(f"{name} 超过 {limit} 字")
    return value


def _new_id(prefix: str) -> str:
    return f"{prefix}_{uuid.uuid4().hex[:12]}"


class TopoMemory:
    """Memory store for one <robot model>/<venue>. Thread-safe (one lock serialises all access)."""

    def __init__(self, root, robot_model: str, venue: str):
        self.root = Path(root)
        self.robot_model = robot_model
        self.venue = venue
        self.images = self.root / "images"
        self.images.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self.anchor: str | None = None          # current place, in-process only
        self._db = sqlite3.connect(str(self.root / "memory.sqlite3"), check_same_thread=False)
        self._db.row_factory = sqlite3.Row
        self._db.execute("PRAGMA foreign_keys=ON")
        with self._db:
            version = self._db.execute("PRAGMA user_version").fetchone()[0]
            if version not in (0, SCHEMA_VERSION):
                raise ValueError(f"记忆库版本 {version} 不受支持（期望 {SCHEMA_VERSION}）：{self.root}")
            self._db.executescript(_SCHEMA)
            self._db.execute(f"PRAGMA user_version={SCHEMA_VERSION}")
            for key, value in (("robot_model", robot_model), ("venue", venue)):
                row = self._db.execute("SELECT value FROM meta WHERE key=?", (key,)).fetchone()
                if row is None:
                    self._db.execute("INSERT INTO meta(key, value) VALUES(?, ?)", (key, value))
                elif row["value"] != value:
                    raise ValueError(f"记忆库 {key}={row['value']!r} 与当前 {value!r} 不符，"
                                     "不同机型/场地的记忆不能混用")

    def close(self):
        with self._lock:
            self._db.close()

    def _row(self, table: str, row_id: str):
        row = self._db.execute(f"SELECT * FROM {table} WHERE id=?", (row_id,)).fetchone()
        if row is None:
            raise ValueError(f"未知 {table[:-1]} id：{row_id}")
        return row

    def _active_place(self, place_id: str):
        row = self._row("places", place_id)
        if row["status"] != "active":
            raise ValueError(f"地点 {place_id}（{row['label']}）已标记失效：{row['evidence']}")
        return row

    def _add_view(self, place_id: str, jpeg: bytes) -> str:
        sig = signature(jpeg)
        view_id = _new_id("view")
        path = self.images / f"{view_id}.jpg"
        tmp = path.with_suffix(".tmp")
        tmp.write_bytes(jpeg)
        os.replace(tmp, path)
        self._db.execute("INSERT INTO views(id, place_id, signature, created_at) VALUES(?,?,?,?)",
                         (view_id, place_id, sig, time.time()))
        return view_id

    def _connect(self, to_place: str, via: str, frontier_id: str | None) -> dict | None:
        """Record a traversed edge anchor -> to_place; mark frontier_id explored if given."""
        now = time.time()
        edge = None
        if frontier_id:
            f = self._row("frontiers", frontier_id)
            self._db.execute("UPDATE frontiers SET status='explored', destination_id=?, updated_at=? "
                             "WHERE id=?", (to_place, now, f["id"]))
        if self.anchor and self.anchor != to_place:
            self._db.execute(
                "INSERT INTO edges(id, from_place, to_place, evidence, updated_at) VALUES(?,?,?,?,?) "
                "ON CONFLICT(from_place, to_place) DO UPDATE SET status='traversed', "
                "traversals=traversals+1, evidence=CASE WHEN excluded.evidence='' THEN evidence "
                "ELSE excluded.evidence END, updated_at=excluded.updated_at",
                (_new_id("edge"), self.anchor, to_place, via, now))
            row = self._db.execute("SELECT * FROM edges WHERE from_place=? AND to_place=?",
                                   (self.anchor, to_place)).fetchone()
            edge = {"id": row["id"], "from_place": row["from_place"], "to_place": row["to_place"],
                    "traversals": row["traversals"]}
        self.anchor = to_place
        return edge

    def _place_brief(self, row) -> dict:
        return {"place_id": row["id"], "label": row["label"],
                "aliases": json.loads(row["aliases_json"]), "metric_place": row["metric_place"]}

    def save_place(self, label, jpeg: bytes, *, aliases=None, evidence="", place_id=None,
                   metric_place=None, via="", frontier_id=None) -> dict:
        """Create a place, or add a view to an existing one (place_id); then move the anchor here and connect an edge."""
        label = _text(label, "label", required=place_id is None)
        evidence = _text(evidence, "evidence", limit=500)
        via = _text(via, "via", limit=500)
        aliases = [_text(a, "alias", required=True) for a in (aliases or [])]
        metric_place = _text(metric_place, "metric_place") or None
        with self._lock, self._db:
            now = time.time()
            if place_id:
                row = self._active_place(place_id)
                merged = list(dict.fromkeys(json.loads(row["aliases_json"]) + aliases))
                self._db.execute(
                    "UPDATE places SET aliases_json=?, metric_place=COALESCE(?, metric_place), "
                    "evidence=CASE WHEN ?='' THEN evidence ELSE ? END, updated_at=? WHERE id=?",
                    (json.dumps(merged, ensure_ascii=False), metric_place, evidence, evidence, now, place_id))
                created = False
            else:
                clash = [r["id"] for r in self._db.execute(
                    "SELECT id, label, aliases_json FROM places WHERE status='active'")
                    if _norm(label) in {_norm(r["label"]), *map(_norm, json.loads(r["aliases_json"]))}]
                if clash:
                    raise ValueError(f"已有同名地点 {clash[0]}（{label}）：给它加视角请传 place_id")
                place_id = _new_id("place")
                self._db.execute(
                    "INSERT INTO places(id, label, aliases_json, evidence, metric_place, updated_at) "
                    "VALUES(?,?,?,?,?,?)",
                    (place_id, label, json.dumps(aliases, ensure_ascii=False), evidence, metric_place, now))
                created = True
            view_id = self._add_view(place_id, jpeg)
            edge = self._connect(place_id, via, frontier_id)
            row = self._row("places", place_id)
        return {"created": created, **self._place_brief(row), "view_id": view_id, "edge": edge,
                "anchor": self.anchor}

    def localize(self, place_id, jpeg: bytes | None = None, *, via="", frontier_id=None) -> dict:
        """Model confirmed "I am at place_id": move the anchor and connect an edge; add a view if jpeg is given."""
        via = _text(via, "via", limit=500)
        with self._lock, self._db:
            row = self._active_place(place_id)
            view_id = self._add_view(place_id, jpeg) if jpeg is not None else None
            edge = self._connect(place_id, via, frontier_id)
        return {**self._place_brief(row), "view_id": view_id, "edge": edge, "anchor": self.anchor}

    def _require_anchor(self) -> str:
        if not self.anchor:
            raise ValueError("当前地点未知：先 memory_localize 确认或 memory_save_place 新建当前地点")
        return self._active_place(self.anchor)["id"]

    def remember_landmark(self, label, jpeg: bytes, *, evidence="") -> dict:
        label = _text(label, "label", required=True)
        evidence = _text(evidence, "evidence", limit=500)
        with self._lock, self._db:
            place_id = self._require_anchor()
            view_id = self._add_view(place_id, jpeg)
            self._db.execute(
                "INSERT INTO landmarks(id, place_id, label, evidence, view_id, updated_at) VALUES(?,?,?,?,?,?) "
                "ON CONFLICT(place_id, label) DO UPDATE SET status='active', evidence=excluded.evidence, "
                "view_id=excluded.view_id, updated_at=excluded.updated_at",
                (_new_id("lm"), place_id, label, evidence, view_id, time.time()))
            row = self._db.execute("SELECT id FROM landmarks WHERE place_id=? AND label=?",
                                   (place_id, label)).fetchone()
        return {"landmark_id": row["id"], "place_id": place_id, "label": label, "view_id": view_id}

    def remember_frontier(self, label, direction, jpeg: bytes, *, evidence="") -> dict:
        label = _text(label, "label", required=True)
        evidence = _text(evidence, "evidence", limit=500)
        if direction not in DIRECTIONS:
            raise ValueError(f"direction 须为 {DIRECTIONS} 之一")
        with self._lock, self._db:
            place_id = self._require_anchor()
            view_id = self._add_view(place_id, jpeg)
            self._db.execute(
                "INSERT INTO frontiers(id, place_id, label, direction, evidence, view_id, updated_at) "
                "VALUES(?,?,?,?,?,?,?) ON CONFLICT(place_id, label) DO UPDATE SET "
                "direction=excluded.direction, evidence=excluded.evidence, view_id=excluded.view_id, "
                "updated_at=excluded.updated_at",
                (_new_id("fr"), place_id, label, direction, evidence, view_id, time.time()))
            row = self._db.execute("SELECT id, status FROM frontiers WHERE place_id=? AND label=?",
                                   (place_id, label)).fetchone()
        return {"frontier_id": row["id"], "place_id": place_id, "label": label,
                "direction": direction, "status": row["status"], "view_id": view_id}

    def mark_unavailable(self, kind, row_id, reason) -> dict:
        """Mark unavailable (moved away, door closed, path blocked). Unavailable places are
        excluded from recall and routes; the anchor is cleared if it pointed there."""
        reason = _text(reason, "reason", required=True, limit=500)
        if kind not in KINDS:
            raise ValueError(f"kind 须为 {KINDS} 之一")
        table = {"place": "places", "landmark": "landmarks", "frontier": "frontiers", "edge": "edges"}[kind]
        with self._lock, self._db:
            self._row(table, row_id)
            self._db.execute(f"UPDATE {table} SET status='unavailable', evidence=?, updated_at=? WHERE id=?",
                             (reason, time.time(), row_id))
            if kind == "place" and self.anchor == row_id:
                self.anchor = None
        return {"kind": kind, "id": row_id, "status": "unavailable", "anchor": self.anchor}

    def candidates(self, jpeg: bytes, k: int = 3) -> list[dict]:
        """Compare the frame against all views of active places; best view per place, sorted by score descending."""
        sig = signature(jpeg)
        best: dict[str, dict] = {}
        with self._lock:
            rows = self._db.execute(
                "SELECT v.id AS view_id, v.signature, p.* FROM views v JOIN places p ON p.id=v.place_id "
                "WHERE p.status='active'").fetchall()
        for r in rows:
            score = similarity(sig, r["signature"])
            if r["id"] not in best or score > best[r["id"]]["score"]:
                best[r["id"]] = {**self._place_brief(r), "score": round(score, 3), "view_id": r["view_id"]}
        return sorted(best.values(), key=lambda c: -c["score"])[:max(0, int(k))]

    def targets(self, task) -> list[dict]:
        """Substring match of the task text against active place/landmark labels and aliases
        (whitespace stripped, case-insensitive)."""
        text = _norm(task)
        if not text:
            return []
        def hit(names):
            return any(n and (n in text or text in n) for n in map(_norm, names))
        out = []
        with self._lock:
            for r in self._db.execute("SELECT * FROM places WHERE status='active'"):
                if hit([r["label"], *json.loads(r["aliases_json"])]):
                    out.append({"kind": "place", **self._place_brief(r)})
            for r in self._db.execute(
                    "SELECT l.id, l.label, l.evidence, l.place_id, p.label AS place_label FROM landmarks l "
                    "JOIN places p ON p.id=l.place_id WHERE l.status='active' AND p.status='active'"):
                if hit([r["label"]]):
                    out.append({"kind": "landmark", "landmark_id": r["id"], "label": r["label"],
                                "place_id": r["place_id"], "place_label": r["place_label"],
                                "evidence": r["evidence"]})
        return out

    def route(self, origin: str | None, goals) -> list[dict] | None:
        """BFS over traversed active edges: edge list origin -> any goal; [] if origin is a goal, None if unreachable."""
        goals = set(goals)
        if not origin or not goals:
            return None
        queue, seen = deque([(origin, [])]), {origin}
        with self._lock:
            while queue:
                place_id, path = queue.popleft()
                if place_id in goals:
                    return path
                for r in self._db.execute(
                        "SELECT e.*, p.label AS to_label FROM edges e JOIN places p ON p.id=e.to_place "
                        "WHERE e.from_place=? AND e.status='traversed' AND p.status='active'", (place_id,)):
                    if r["to_place"] not in seen:
                        seen.add(r["to_place"])
                        queue.append((r["to_place"], path + [{
                            "edge_id": r["id"], "from_place": r["from_place"], "to_place": r["to_place"],
                            "to_label": r["to_label"], "via": r["evidence"]}]))
        return None

    def open_frontiers(self, place_id: str | None) -> list[dict]:
        if not place_id:
            return []
        with self._lock:
            return [{"frontier_id": r["id"], "label": r["label"], "direction": r["direction"],
                     "evidence": r["evidence"]}
                    for r in self._db.execute(
                        "SELECT * FROM frontiers WHERE place_id=? AND status='open'", (place_id,))]

    def view_jpeg(self, view_id: str) -> bytes | None:
        path = self.images / f"{view_id}.jpg"
        return path.read_bytes() if path.is_file() else None

    def context(self, task, jpeg: bytes | None, k: int = 3) -> dict:
        cands = self.candidates(jpeg, k) if jpeg is not None else []
        targets = self.targets(task)
        routes = []
        for t in targets:
            path = self.route(self.anchor, [t["place_id"]])
            routes.append({"target": t["label"], "place_id": t["place_id"], "reachable": path is not None,
                           "edges": path or []})
        with self._lock:
            anchor = self._row("places", self.anchor) if self.anchor else None
        return {"anchor": self._place_brief(anchor) if anchor is not None else None,
                "candidates": cands, "targets": targets, "routes": routes,
                "open_frontiers": self.open_frontiers(self.anchor),
                "likely_new_place": bool(jpeg is not None and (not cands or cands[0]["score"] < NEW_PLACE_HINT))}

    def summary(self) -> dict:
        with self._lock:
            places = []
            for r in self._db.execute("SELECT * FROM places ORDER BY updated_at"):
                views = self._db.execute("SELECT COUNT(*) FROM views WHERE place_id=?", (r["id"],)).fetchone()[0]
                places.append({**self._place_brief(r), "status": r["status"], "evidence": r["evidence"],
                               "views": views})
            landmarks = [dict(r) for r in self._db.execute(
                "SELECT id, place_id, label, status, evidence FROM landmarks ORDER BY updated_at")]
            frontiers = [dict(r) for r in self._db.execute(
                "SELECT id, place_id, label, direction, status, destination_id FROM frontiers ORDER BY updated_at")]
            edges = [dict(r) for r in self._db.execute(
                "SELECT id, from_place, to_place, status, evidence AS via, traversals FROM edges ORDER BY updated_at")]
        return {"robot_model": self.robot_model, "venue": self.venue, "anchor": self.anchor,
                "places": places, "landmarks": landmarks, "frontiers": frontiers, "edges": edges}


# MCP tools (shared by the real-robot server and the offline stub)
def register_memory_tools(mcp, get_memory, get_frame, wrapper=None, *,
                          navigable=lambda: True, check_metric_place=lambda name: None):
    """get_memory() -> TopoMemory; get_frame(frame_id|None) -> (frame_id, jpeg); both raise ValueError.
    navigable() -> whether this model has SLAMWARE (can navigate_to the linked metric_place);
    check_metric_place(name) validates a metric point exists before linking (raises ValueError)."""
    from mcp.server.fastmcp import Image

    def call(name, params, fn):
        def run():
            try:
                result = fn(get_memory())
            except (ValueError, OSError, sqlite3.Error) as exc:
                return {"ok": False, "error": str(exc)}
            if isinstance(result, dict):
                return {"ok": True, **result}
            return result
        return wrapper(name, params, run) if wrapper else run()

    def with_navigable(entry: dict) -> dict:
        if "metric_place" in entry:
            entry["navigable"] = bool(entry["metric_place"]) and navigable()
        return entry

    @mcp.tool()
    def memory_context(task: str = "", frame_id: int | None = None, max_images: int = 2):
        """拓扑记忆召回：当前帧（frame_id，省略=最近一帧）与已存地点比缩略图，给 top-k 候选（score 很粗，
        只作提示，务必自己比对附带的参考图再 memory_localize）；task 与地点/地标名匹配出目标，并按走过的边
        给出从当前锚点出发的路线提示（via 是当时的路线摘要，只作参考，不回放）。navigable=true 的目标才可
        navigate_to(metric_place)，否则 pilot。不连底盘、不运动。"""
        def run(m):
            fid, jpeg = get_frame(frame_id)
            out = m.context(task, jpeg)
            for key in ("candidates", "targets"):
                out[key] = [with_navigable(c) for c in out[key]]
            if out["anchor"]:
                with_navigable(out["anchor"])
            out["frame_id"] = fid
            images = []
            for c in out["candidates"][:max(0, min(int(max_images), 3))]:
                data = m.view_jpeg(c["view_id"])
                if data:
                    c["image_index"] = len(images) + 1
                    images.append(Image(data=data, format="jpeg"))
            return [{"ok": True, **out}, *images] if images else out
        return call("memory_context", {"task": task, "frame_id": frame_id, "max_images": max_images}, run)

    @mcp.tool()
    def memory_localize(place_id: str, frame_id: int | None = None, via: str = "",
                        frontier_id: str | None = None, add_view: bool = False) -> dict:
        """比图确认"我现在在 place_id"后调用：把当前锚点移到这里；若之前有锚点，自动记一条
        上个锚点→这里的走过的边（via 写一句路线摘要，如"出门右转沿走廊直行约5m"）。
        从某个前沿出发到达的，传 frontier_id 把它标为已探索。add_view=true 时把 frame_id 那帧
        作为该地点的新视角（换了角度认不出时用）。没把握就别 localize——宁可当新地点存。"""
        def run(m):
            jpeg = get_frame(frame_id)[1] if add_view else None
            return with_navigable(m.localize(place_id, jpeg, via=via, frontier_id=frontier_id))
        return call("memory_localize", {"place_id": place_id, "frame_id": frame_id, "via": via,
                                        "frontier_id": frontier_id, "add_view": add_view}, run)

    @mcp.tool()
    def memory_save_place(label: str = "", evidence: str = "", frame_id: int | None = None,
                          aliases: list[str] | None = None, place_id: str | None = None,
                          metric_place: str | None = None, via: str = "",
                          frontier_id: str | None = None) -> dict:
        """把 frame_id 那帧存为地点证据：不传 place_id = 新建地点（label 必填，同名会拒绝），
        传 place_id = 给已有地点加视角/别名。随后锚点移到这里并从上个锚点连边（via/frontier_id
        同 memory_localize）。metric_place 只填 list_places 里已存在的标定点名。evidence 写看到的
        直接依据（如"蓝色玻璃门、右侧饮水机"）。"""
        def run(m):
            if metric_place:
                check_metric_place(metric_place)
            _, jpeg = get_frame(frame_id)
            return with_navigable(m.save_place(label, jpeg, aliases=aliases, evidence=evidence,
                                               place_id=place_id, metric_place=metric_place,
                                               via=via, frontier_id=frontier_id))
        return call("memory_save_place", {"label": label, "evidence": evidence, "frame_id": frame_id,
                                          "aliases": aliases, "place_id": place_id,
                                          "metric_place": metric_place, "via": via,
                                          "frontier_id": frontier_id}, run)

    @mcp.tool()
    def memory_remember_landmark(label: str, evidence: str = "", frame_id: int | None = None) -> dict:
        """在当前锚点地点下记一个地标（饮水机、打印机、玻璃门……），frame_id 那帧为证据。
        同一地点同名地标会更新。需要先有锚点（memory_localize / memory_save_place）。"""
        def run(m):
            return m.remember_landmark(label, get_frame(frame_id)[1], evidence=evidence)
        return call("memory_remember_landmark", {"label": label, "evidence": evidence,
                                                 "frame_id": frame_id}, run)

    @mcp.tool()
    def memory_remember_frontier(label: str, direction: str, evidence: str = "",
                                 frame_id: int | None = None) -> dict:
        """在当前锚点地点下记一个还没探索的出口/方向（direction=forward/left/right/back，
        相对 frame_id 那帧的朝向）。以后从这里出发探索到新地点时，save_place/localize 传
        frontier_id 把它标为已探索。"""
        def run(m):
            return m.remember_frontier(label, direction, get_frame(frame_id)[1], evidence=evidence)
        return call("memory_remember_frontier", {"label": label, "direction": direction,
                                                 "evidence": evidence, "frame_id": frame_id}, run)

    @mcp.tool()
    def memory_mark_unavailable(kind: str, id: str, reason: str) -> dict:
        """标记失效：kind=place/landmark/frontier/edge。东西搬走了、门关了、那条路走不通时用；
        失效项不再召回、不进路线提示。reason 写直接依据。"""
        def run(m):
            return m.mark_unavailable(kind, id, reason)
        return call("memory_mark_unavailable", {"kind": kind, "id": id, "reason": reason}, run)

    @mcp.tool()
    def memory_list() -> dict:
        """拓扑记忆概览：本机型/本场地的全部地点（含视角数、失效状态）、地标、前沿、边和当前锚点。
        不连底盘、不运动。"""
        def run(m):
            out = m.summary()
            out["places"] = [with_navigable(p) for p in out["places"]]
            return out
        return call("memory_list", {}, run)
