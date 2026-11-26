"""Collect one direct-EEF sandwich run's time, calls, token cost, stages, and temperature summary."""
from __future__ import annotations

import argparse
import datetime as dt
import json
import re
import sys
from pathlib import Path

PRICE = {"input": 8.0, "output": 40.0, "cache_read": 0.20, "cache_write": 10.0}
# (text looked for in the lower-cased `note` of a command, stage it starts)
STAGE_KEYS = (
    ("slice 1", "slice 1"),
    ("slice1", "slice 1"),
    ("lettuce", "lettuce"),
    ("slice 2", "slice 2"),
    ("slice2", "slice 2"),
    ("push", "push plate"),
)
MOTION_KINDS = ("exec", "both", "grip", "waist", "home", "prepare")


def _ts(value: str) -> float:
    return dt.datetime.fromisoformat(value.replace("Z", "+00:00")).timestamp()


def _jsonl(path: Path) -> list[dict]:
    rows = []
    with path.open(encoding="utf-8") as f:
        for raw in f:
            try:
                row = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if isinstance(row, dict):
                rows.append(row)
    return rows


def _stage_of(note: str, current: str) -> str:
    lower = note.lower()
    for key, stage in STAGE_KEYS:
        if key in lower:
            return stage
    return current


def _usage_rows(transcript: Path, start: float, end: float) -> list[dict]:
    rows = []
    seen_ids = set()
    for index, event in enumerate(_jsonl(transcript)):
        message = event.get("message")
        if event.get("type") != "assistant" or not isinstance(message, dict):
            continue
        usage = message.get("usage")
        if not isinstance(usage, dict):
            continue
        timestamp = event.get("timestamp")
        if not timestamp:
            continue
        try:
            when = _ts(timestamp)
        except ValueError:
            continue
        if not start <= when < end:
            continue
        message_id = message.get("id")
        if message_id:
            if message_id in seen_ids:
                continue
            seen_ids.add(message_id)
        rows.append(
            {
                "key": message_id or f"line-{index}",
                "t": when,
                "model": message.get("model"),
                "input": int(usage.get("input_tokens", 0) or 0),
                "output": int(usage.get("output_tokens", 0) or 0),
                "cache_read": int(usage.get("cache_read_input_tokens", 0) or 0),
                "cache_write": int(usage.get("cache_creation_input_tokens", 0) or 0),
            }
        )
    return rows


def _cost(rows: list[dict]) -> dict:
    tokens = {key: sum(row[key] for row in rows) for key in ("input", "output", "cache_read", "cache_write")}
    usd = {key: tokens[key] * PRICE[key] / 1_000_000 for key in tokens}
    usd["total"] = sum(usd.values())
    return {
        "calls": len(rows),
        "tokens": tokens,
        "usd": {key: round(value, 2) for key, value in usd.items()},
    }


def _transcript_text(event: dict) -> str:
    content = (event.get("message") or {}).get("content")
    if isinstance(content, str):
        return content
    try:
        return json.dumps(content, ensure_ascii=False)
    except TypeError:
        return str(content)


def _temperatures(transcript: Path, start: float, end: float) -> dict:
    samples = []
    pattern = re.compile(r"(?:temps_max_c\\?\"?: ?|temps_max_c )(\d+)")
    for event in _jsonl(transcript):
        if event.get("type") != "user" or not event.get("timestamp"):
            continue
        try:
            when = _ts(event["timestamp"])
        except ValueError:
            continue
        if not start <= when < end:
            continue
        for match in pattern.finditer(_transcript_text(event)):
            value = int(match.group(1))
            if 50 <= value <= 95:
                samples.append({"t": round(when, 2), "max_c": value})
    values = [sample["max_c"] for sample in samples]
    return {
        "start_c": values[0] if values else None,
        "max_c": max(values) if values else None,
        "end_c": values[-1] if values else None,
        "samples": samples,
    }


def _find_transcript(run_name: str, start: float, end: float) -> Path | None:
    roots = [
        Path.home() / ".claude" / "projects",
        Path.home() / ".codex" / "sessions",
    ]
    best = None
    for root in roots:
        if not root.exists():
            continue
        for path in root.rglob("*.jsonl"):
            try:
                text = path.read_text(encoding="utf-8", errors="ignore")
            except OSError:
                continue
            if run_name not in text:
                continue
            count = len(_usage_rows(path, start, end))
            if count and (best is None or count > best[0]):
                best = (count, path)
    return best[1] if best else None


def _parse_until(value: str, day: dt.date, tz: dt.tzinfo) -> float:
    parsed = dt.time.fromisoformat(value)
    return dt.datetime.combine(day, parsed, tzinfo=tz).timestamp()


def collect(run_dir: Path, transcript: Path, request: str, until: str | None = None) -> dict:
    log_path = run_dir / "eef_log.jsonl"
    if not log_path.is_file():
        raise ValueError(f"missing {log_path}")
    log = _jsonl(log_path)
    if not log:
        raise ValueError(f"empty {log_path}")
    first = float(log[0]["t"])
    tz = dt.datetime.fromtimestamp(first).astimezone().tzinfo
    day = dt.datetime.fromtimestamp(first, tz).date()
    until_ts = _parse_until(until, day, tz) if until else None
    if until_ts is not None:
        log = [event for event in log if float(event.get("t", 0.0)) <= until_ts]
    start = float(log[0]["t"]) - 60.0
    end = float(log[-1]["t"]) + 60.0

    current = "setup"
    stages: dict[str, dict] = {}
    for event in log:
        kind = event.get("kind")
        if kind not in MOTION_KINDS + ("keyframe", "scene"):
            continue
        payload = event.get("input")
        if isinstance(payload, dict):
            note = str(payload.get("note") or "")
        elif isinstance(payload, list):
            note = " ".join(str(value) for value in payload)
        else:
            note = str(event.get("note") or "")
        current = _stage_of(note, current)
        stage = stages.setdefault(
            current,
            {
                "exec_calls": 0,
                "wps_sent": 0,
                "wps_executed": 0,
                "refused_calls": 0,
                "robot_s": 0.0,
                "grasps": [],
            },
        )
        result = event.get("result") or {}
        if kind == "exec":
            stage["exec_calls"] += 1
            stage["wps_sent"] += sum(len(segment.get("waypoints", [])) for segment in payload.get("segments", []))
            moves = [item for item in result.get("results", []) if "gripper" not in item]
            stage["wps_executed"] += sum(1 for item in moves if item.get("executed"))
            stage["refused_calls"] += 0 if result.get("ok") else 1
            stage["robot_s"] += float(result.get("seconds", 0.0) or 0.0)
            for item in result.get("results", []):
                if item.get("gripper") == "grasp":
                    stage["grasps"].append(
                        {"opening": item.get("opening"), "holding": item.get("holding")}
                    )
        elif kind == "both":                    # one two-arm waypoint per command
            stage["exec_calls"] += 1
            stage["wps_sent"] += 1
            stage["wps_executed"] += 1 if result.get("executed") else 0
            stage["refused_calls"] += 0 if result.get("ok") else 1
            stage["robot_s"] += float(result.get("duration_s", 0.0) or 0.0)
        elif kind in MOTION_KINDS:              # grip, waist, home
            stage["robot_s"] += float(result.get("duration_s", 0.0) or 0.0)

    calls = _usage_rows(transcript, start, end)
    call_times = [row["t"] for row in calls]
    for name, stage in stages.items():
        stage["robot_s"] = round(stage["robot_s"], 1)
    if calls:
        output_start = dt.datetime.fromtimestamp(call_times[0], tz).strftime("%H:%M:%S")
        output_end = dt.datetime.fromtimestamp(call_times[-1], tz).strftime("%H:%M:%S")
        wall_min = round((call_times[-1] - call_times[0]) / 60.0, 1)
    else:
        output_start = output_end = None
        wall_min = None
    return {
        "request": request,
        "run_dir": str(run_dir),
        "transcript": str(transcript),
        "window": {
            "start": dt.datetime.fromtimestamp(start, tz).isoformat(timespec="seconds"),
            "end": dt.datetime.fromtimestamp(end, tz).isoformat(timespec="seconds"),
        },
        "start": output_start,
        "end": output_end,
        "wall_min": wall_min,
        "robot_s": round(sum(stage["robot_s"] for stage in stages.values()), 1),
        "model": _cost(calls),
        "stages": stages,
        "motor_temp": _temperatures(transcript, start, end),
        "price_per_mtok": PRICE,
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run-dir", required=True, type=Path)
    ap.add_argument("--transcript", type=Path)
    ap.add_argument("--request", default="")
    ap.add_argument("--until", help="HH:MM:SS; ignore EEF log entries after this local time")
    ap.add_argument("--output", type=Path, help="default: <run-dir>/metrics.json; use - for stdout")
    args = ap.parse_args(argv)
    run_dir = args.run_dir.resolve()
    log_path = run_dir / "eef_log.jsonl"
    try:
        initial = _jsonl(log_path)
        if not initial:
            raise ValueError(f"empty {log_path}")
        # search the run's own time window: a later session that merely mentions the run must not win
        start, end = float(initial[0]["t"]) - 60.0, float(initial[-1]["t"]) + 60.0
        transcript = args.transcript or _find_transcript(run_dir.name, start, end)
        if transcript is None:
            raise ValueError("transcript not found; pass --transcript")
        metrics = collect(run_dir, transcript.resolve(), args.request, args.until)
    except (OSError, KeyError, TypeError, ValueError) as exc:
        print(f"collect_metrics: {exc}", file=sys.stderr)
        return 1
    if args.output is not None and str(args.output) == "-":
        json.dump(metrics, sys.stdout, ensure_ascii=False, indent=2)
        print()
        return 0
    output = args.output or run_dir / "metrics.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(metrics, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
