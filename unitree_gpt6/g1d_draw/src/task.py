"""Offline stroke planning, paper previews and immutable execution snapshots.

The SVG is a plan, never a photograph. Elevated Cartesian waypoints do not
establish whole-arm collision clearance: the compiled joint interpolation
must be checked with the deployment model and reviewed separately.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import re
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from .config import load_config, normalize_config
from .job import FORMAT, POSE_KEYS, fingerprint, validate_job

STROKES_FORMAT = "astra-paint-strokes/v1"
TASK_FORMAT = "astra-paint-task/v1"
TASK_ARTIFACTS = frozenset(
    {
        "source_device.json",
        "device.json",
        "strokes.json",
        "measurements.json",
        "calibration.json",
        "plan.json",
        "job.json",
        "report.json",
        "preview.svg",
        "build_plan.py",
        "run.sh",
    }
)


def _paper_metric(calibration):
    """An orthonormal 2-D chart of the measured plane and edge distances."""
    matrix = calibration["paper"]["matrix_uv1_to_xyz"]
    a, b = [row[0] for row in matrix], [row[1] for row in matrix]
    norm_a, norm_b = math.hypot(*a), math.hypot(*b)
    cross = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
    area = math.hypot(*cross)
    return (
        norm_a,
        sum(x * y for x, y in zip(a, b, strict=False)) / norm_a,
        area / norm_a,
        area / norm_b,
    )


def _number(value, label, *, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(f"{label} must be finite" + (" and positive" if positive else ""))
    return float(value)


def _checked_inputs(strokes, calibration):
    from .calibration import map_paper, validate_calibration

    # The original dictionaries remain untouched, including their evidence.
    calibration = validate_calibration(calibration)
    if not isinstance(strokes, dict) or strokes.get("format") != STROKES_FORMAT:
        raise ValueError(f"expected format={STROKES_FORMAT}")
    try:
        fingerprint(strokes)  # Reject non-JSON/nonfinite metadata as well as geometry.
    except (TypeError, ValueError) as exc:
        raise ValueError("strokes must contain finite JSON data") from exc
    purpose = strokes.get("purpose")
    if purpose not in {"painting", "simulation"} or purpose != calibration.get("purpose"):
        raise ValueError("strokes and calibration must have the same explicit purpose")
    size = strokes.get("paper_size_m")
    if (
        not isinstance(size, list)
        or len(size) != 2
        or any(
            not math.isclose(_number(a, "paper_size_m", positive=True), b, rel_tol=0, abs_tol=1e-9)
            for a, b in zip(size, calibration["paper"]["size_m"], strict=False)
        )
    ):
        raise ValueError("strokes paper_size_m must match the calibrated paper")
    palette = strokes.get("palette")
    if (
        not isinstance(palette, dict)
        or not palette
        or any(
            not isinstance(name, str)
            or not name
            or name == "water"
            or not isinstance(color, str)
            or re.fullmatch(
                r"#(?:[0-9a-fA-F]{3}|[0-9a-fA-F]{4}|[0-9a-fA-F]{6}|[0-9a-fA-F]{8})", color
            )
            is None
            for name, color in palette.items()
        )
    ):
        raise ValueError("palette needs pigment names and hexadecimal SVG colors")
    paths = strokes.get("strokes")
    if not isinstance(paths, list) or not paths:
        raise ValueError("strokes must be a nonempty list")
    checked, identifiers = [], set()
    brush_width = calibration["brush"]["width_m"]
    _, _, edge_v_metric, edge_u_metric = _paper_metric(calibration)
    for path in paths:
        if not isinstance(path, dict):
            raise ValueError("each stroke must be an object")
        identifier, color = path.get("id"), path.get("color")
        if not isinstance(identifier, str) or not identifier or identifier in identifiers:
            raise ValueError("stroke IDs must be nonempty and unique")
        identifiers.add(identifier)
        if (
            not isinstance(color, str)
            or color not in palette
            or color not in calibration["materials"]
        ):
            raise ValueError(f"stroke {identifier} needs a palette color and calibrated material")
        width = max(
            brush_width, _number(path.get("width_m", brush_width), "stroke.width_m", positive=True)
        )
        points = path.get("points_uv_m")
        if not isinstance(points, list) or not points:
            raise ValueError("points_uv_m needs at least one paper point")
        uv, xyz = [], []
        for point in points:
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise ValueError("each points_uv_m point needs two finite meters")
            u, v = (_number(item, "paper coordinate") for item in point)
            radius = width / 2
            if (
                min(u, size[0] - u) * edge_u_metric < radius
                or min(v, size[1] - v) * edge_v_metric < radius
            ):
                raise ValueError(f"stroke {identifier} brush footprint crosses the paper boundary")
            uv.append([u, v])
            xyz.append(map_paper(calibration, [u, v]).tolist())
        checked.append({"id": identifier, "color": color, "width_m": width, "uv": uv, "xyz": xyz})
    return calibration, checked


def _split_path(points, maximum):
    """Split by measured 3-D arc length, preserving corners and order."""
    total = sum(math.dist(a, b) for a, b in zip(points, points[1:], strict=False))
    if total / maximum > 100000:
        raise ValueError("max_stroke_length_m would create more than 100000 actual strokes")
    epsilon = maximum * 1e-12
    current, length, result = [list(points[0])], 0.0, []
    for endpoint in points[1:]:
        endpoint = list(endpoint)
        start = current[-1]
        distance = math.dist(start, endpoint)
        if distance == 0:
            # Repeated source points can carry an intentional stationary dot.
            current.append(endpoint)
            continue
        while distance > epsilon:
            remaining = maximum - length
            if remaining <= epsilon:
                result.append(current)
                current, length = [list(start)], 0.0
                continue
            if distance <= remaining + epsilon:
                current.append(endpoint)
                length += distance
                break
            ratio = remaining / distance
            cut = [a + (b - a) * ratio for a, b in zip(start, endpoint, strict=False)]
            current.append(cut)
            result.append(current)
            current, length, start = [cut], 0.0, cut
            distance = math.dist(start, endpoint)
        if current[-1] != endpoint:
            current.append(endpoint)
    if len(current) > 1 or not result:
        result.append(current)
    return result


def _pose(xyz, rpy):
    return dict(zip(POSE_KEYS, [*xyz, *rpy], strict=False))


def _sample_contact(points, spacing):
    """Retain every original corner while adding bounded Cartesian IK targets."""
    sampled = [list(points[0])]
    for start, end in zip(points, points[1:], strict=False):
        count = max(1, math.ceil(math.dist(start, end) / spacing))
        if len(sampled) + count > 4091:
            raise ValueError(
                "contact sampling exceeds 4091 points; "
                "adjust stroke_sample_spacing_m or max_stroke_length_m"
            )
        sampled.extend(
            [
                [a + (b - a) * i / count for a, b in zip(start, end, strict=False)]
                for i in range(1, count + 1)
            ]
        )
    return sampled


def _at_height(pose, height):
    result = copy.deepcopy(pose)
    result["z"] = height
    return result


def build_plan(strokes: dict, calibration: dict, job_id: str) -> dict:
    """Map measured paper strokes to uncompiled EEF batches, without hardware.

    Every split segment is an actual brush stroke for the reload cadence.
    Materials retain their complete measured action and attitude. Only new
    transfer waypoints are added; no material contact coordinates are moved.
    """
    from .calibration import map_paper

    original_calibration = calibration
    calibration, paths = _checked_inputs(strokes, calibration)
    if not isinstance(job_id, str) or not job_id.strip():
        raise ValueError("job_id must be a nonempty string")
    cadence = strokes.get("reload_every", 3)
    if type(cadence) is not int or not 1 <= cadence <= 64:
        raise ValueError("reload_every must be an integer from 1 to 64")
    robot, brush, materials = (calibration[key] for key in ("robot", "brush", "materials"))
    spacing = _number(
        strokes.get("stroke_sample_spacing_m", min(brush["width_m"] / 2, 0.003)),
        "stroke_sample_spacing_m",
        positive=True,
    )
    transfer = _number(robot.get("transfer_z_m"), "robot.transfer_z_m")
    width, height = calibration["paper"]["size_m"]
    paper_ceiling = max(
        map_paper(calibration, uv)[2]
        for uv in ([0.0, 0.0], [width, 0.0], [0.0, height], [width, height])
    )
    material_ceiling = max(
        point["z"] for material in materials.values() for point in material["action"]
    )
    if transfer <= max(paper_ceiling + brush["clearance_m"], material_ceiling):
        raise ValueError(
            "robot.transfer_z_m must exceed the lifted paper and every material waypoint"
        )
    park = copy.deepcopy(robot["park_q14"])
    if len(park) < 2 or park[0] is not None:
        raise ValueError("robot.park_q14 needs a null compiled start followed by a reviewed route")
    batches, loaded, used, actual_count = [], None, 0, 0

    def add_material(color, phase):
        material = materials[color]
        action = copy.deepcopy(material["action"])
        result = {
            "id": f"batch-{len(batches) + 1:05d}",
            "phase": phase,
            "color": color,
            "action": [_at_height(action[0], transfer), *action, _at_height(action[-1], transfer)],
            "contact_index": material["contact_index"] + 1,
            "clearance_m": material["clearance_m"],
            "dwell_s": material["dwell_s"],
            "contact_settle_s": material.get("contact_settle_s", 0.0),
            "end_clear": True,
            "strokes": [],
            "source_material": color,
        }
        if phase == "WASH":
            result["wash_step"] = "water"
        batches.append(result)

    for source_index, path in enumerate(paths, 1):
        parts = _split_path(path["xyz"], brush["max_stroke_length_m"])
        for part_index, points in enumerate(parts, 1):
            color = path["color"]
            if loaded is not None and loaded != color:
                add_material("water", "WASH")
                loaded, used = None, 0
            if loaded != color or used >= cadence:
                add_material(color, "LOAD")
                loaded, used = color, 0
            contact = [_pose(xyz, brush["rpy_rad"]) for xyz in _sample_contact(points, spacing)]
            lift_z = max(point["z"] for point in contact) + brush["clearance_m"]
            batches.append(
                {
                    "id": f"batch-{len(batches) + 1:05d}",
                    "phase": "PAINT",
                    "color": color,
                    "action": [
                        _at_height(contact[0], transfer),
                        _at_height(contact[0], lift_z),
                        *contact,
                        _at_height(contact[-1], lift_z),
                        _at_height(contact[-1], transfer),
                    ],
                    "strokes": [
                        {
                            "id": f"stroke-{source_index:05d}-part-{part_index:04d}",
                            "start": 2,
                            "end": len(contact) + 1,
                            "source_stroke_id": path["id"],
                            "source_part": part_index,
                            "source_parts": len(parts),
                            "width_m": path["width_m"],
                        }
                    ],
                    "dwell_s": 0.0,
                    "contact_settle_s": 0.0,
                    "clearance_m": brush["clearance_m"],
                    "end_clear": True,
                }
            )
            used, actual_count = used + 1, actual_count + 1
    add_material("water", "WASH")
    plan = {
        "format": FORMAT,
        "job_id": job_id,
        "purpose": calibration["purpose"],
        "hand": robot["hand"],
        "reload_every": cadence,
        "initial_brush": "clean",
        "grip_q2": copy.deepcopy(robot["grip_q2"]),
        "waist_q3": copy.deepcopy(robot["waist_q3"]),
        "entry_q14": copy.deepcopy(robot["entry_q14"]),
        "park_q14": park,
        "clearance_review": robot["clearance_review"],
        "batches": batches,
        "provenance": {
            "strokes_sha256": fingerprint(strokes),
            "calibration_sha256": fingerprint(original_calibration),
        },
        "planning": {
            "source_strokes": len(paths),
            "actual_strokes": actual_count,
            "transfer_z_m": transfer,
            "stroke_sample_spacing_m": spacing,
            "preview_is_photograph": False,
            "collision_detection": False,
            "physical_execution": False,
            "clearance_note": (
                "Cartesian targets alone do not establish collision clearance or physical "
                "paper contact; review the actual model FK along joint interpolation."
            ),
        },
    }
    fingerprint(plan)
    return plan


def render_preview(strokes, calibration) -> str:
    """Return a paper-coordinate SVG of the planned brush footprint."""
    calibration, paths = _checked_inputs(strokes, calibration)
    paper_width, paper_height = calibration["paper"]["size_m"]
    norm_a, b_parallel, b_perp, _ = _paper_metric(calibration)
    corners = [
        (u * norm_a + v * b_parallel, v * b_perp)
        for u, v in [
            (0.0, 0.0),
            (paper_width, 0.0),
            (paper_width, paper_height),
            (0.0, paper_height),
        ]
    ]
    minimum_x = min(point[0] for point in corners)
    width = (max(point[0] for point in corners) - minimum_x) * 1000
    height = paper_height * b_perp * 1000

    def drawing_point(u, v):
        return ((u * norm_a + v * b_parallel - minimum_x) * 1000, v * b_perp * 1000)

    banner = max(12.0, height * 0.15)
    ET.register_namespace("", "http://www.w3.org/2000/svg")

    def element(parent, name, **attributes):
        return ET.SubElement(
            parent,
            f"{{http://www.w3.org/2000/svg}}{name}",
            {key.replace("_", "-"): str(value) for key, value in attributes.items()},
        )

    svg = ET.Element(
        "{http://www.w3.org/2000/svg}svg",
        {
            "viewBox": f"0 0 {width:g} {height + banner:g}",
            "role": "img",
            "width": f"{width:g}mm",
            "height": f"{height + banner:g}mm",
        },
    )
    element(svg, "title").text = "Planned brush coverage — not a photograph"
    element(svg, "desc").text = (
        f"planned not photograph; {calibration['purpose']}; "
        "calibrated paper plane in millimeters; no physical contact or collision verification"
    )
    element(
        svg,
        "polygon",
        points=" ".join(f"{(x - minimum_x) * 1000:.9g},{y * 1000:.9g}" for x, y in corners),
        fill="white",
        stroke="#777",
        stroke_width="0.2",
    )
    for path in paths:
        group = element(svg, "g", data_stroke_id=path["id"])
        element(group, "title").text = path["id"]
        color, pen = strokes["palette"][path["color"]], path["width_m"] * 1000
        if len(path["uv"]) == 1:
            x, y = drawing_point(*path["uv"][0])
            element(group, "circle", cx=f"{x:.9g}", cy=f"{y:.9g}", r=f"{pen / 2:.9g}", fill=color)
        else:
            element(
                group,
                "polyline",
                points=" ".join(
                    f"{x:.9g},{y:.9g}" for x, y in (drawing_point(u, v) for u, v in path["uv"])
                ),
                fill="none",
                stroke=color,
                stroke_width=f"{pen:.9g}",
                stroke_linecap="round",
                stroke_linejoin="round",
            )
    element(
        svg,
        "text",
        x="0.5",
        y=f"{height + banner * 0.45:g}",
        font_size=f"{min(3.0, width / 18):g}",
        fill="#222",
    ).text = "PLANNED — NOT A PHOTOGRAPH"
    element(
        svg,
        "text",
        x="0.5",
        y=f"{height + banner * 0.8:g}",
        font_size=f"{min(2.5, width / 21):g}",
        fill="#333",
    ).text = (
        f"{calibration['purpose']} · {len(paths)} source strokes · "
        f"UV {paper_width * 1000:g} × {paper_height * 1000:g} mm"
    )
    return ET.tostring(svg, encoding="unicode", xml_declaration=True)


def _json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode("utf-8")


def _sha256(value, label):
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{64}", value) is None:
        raise ValueError(f"{label} requires a lowercase SHA-256 digest")
    return value


def verify_task(directory) -> dict:
    """Verify a fixed snapshot before any backend or robot model is created.

    These hashes catch accidental changes; the manifest is not a digital
    signature. Model files remain external absolute paths and must retain
    their recorded content. Additional unlisted files, such as an offline
    ``plan.rebuilt.json``, do not change the executable snapshot.
    """
    root = Path(directory).expanduser().resolve(strict=True)
    if not root.is_dir():
        raise ValueError("task must be a directory")
    manifest_path = root / "manifest.json"
    if manifest_path.is_symlink():
        raise ValueError("task manifest must be a regular local file")
    manifest = json.loads(manifest_path.read_bytes())
    if not isinstance(manifest, dict) or manifest.get("format") != TASK_FORMAT:
        raise ValueError(f"expected task format={TASK_FORMAT}")
    fingerprint(manifest)
    hashes = manifest.get("artifacts_sha256")
    if not isinstance(hashes, dict) or not TASK_ARTIFACTS <= hashes.keys():
        raise ValueError("task manifest is missing required artifact hashes")
    checked = {}
    for name, checksum in hashes.items():
        if (
            not isinstance(name, str)
            or not name
            or name in {".", "..", "manifest.json"}
            or Path(name).name != name
            or "/" in name
            or "\\" in name
        ):
            raise ValueError("task artifact names must be local filenames")
        checksum = _sha256(checksum, f"artifact {name}")
        artifact = root / name
        if artifact.is_symlink() or not artifact.is_file():
            raise ValueError(f"task artifact is missing or is not a regular local file: {name}")
        content = artifact.read_bytes()
        if hashlib.sha256(content).hexdigest() != checksum:
            raise ValueError(f"task artifact SHA-256 mismatch: {name}")
        checked[name] = content
    source_hashes = manifest.get("source_sha256")
    if not isinstance(source_hashes, dict):
        raise ValueError("task manifest requires source hashes")
    for source, name in (
        ("config", "source_device.json"),
        ("strokes", "strokes.json"),
        ("measurements", "measurements.json"),
    ):
        if hashlib.sha256(checked[name]).hexdigest() != _sha256(source_hashes.get(source), source):
            raise ValueError(f"task source SHA-256 mismatch: {source}")

    # Parse the bytes that were hashed, avoiding a second read of mutable inputs.
    config = normalize_config(json.loads(checked["device.json"]), root)
    job = validate_job(json.loads(checked["job.json"]))
    purpose = manifest.get("purpose")
    expected_command = "simulate" if purpose == "simulation" else "run"
    if (
        purpose not in {"painting", "simulation"}
        or job.get("purpose") != purpose
        or manifest.get("command") != expected_command
        or manifest.get("job_id") != job["job_id"]
        or _sha256(manifest.get("job_sha256"), "job") != fingerprint(job)
    ):
        raise ValueError(
            "task manifest job ID, purpose, command or SHA-256 does not match the fixed job"
        )
    plan, report = (json.loads(checked[name]) for name in ("plan.json", "report.json"))
    if (
        not isinstance(plan, dict)
        or plan.get("job_id") != job["job_id"]
        or plan.get("purpose") != purpose
        or not isinstance(report, dict)
        or report.get("job_id") != job["job_id"]
        or report.get("sha256") != fingerprint(job)
    ):
        raise ValueError("task plan/report do not identify the fixed job")
    model = config.get("model")
    if purpose == "painting" and not model:
        raise ValueError("physical task requires a recorded URDF model")
    assets = manifest.get("model_assets", {})
    if model:
        urdf = Path(model["urdf"])
        record = assets.get("urdf") if isinstance(assets, dict) else None
        if (
            not isinstance(record, dict)
            or record.get("path") != str(urdf)
            or not urdf.is_absolute()
            or not urdf.is_file()
        ):
            raise ValueError("task URDF model is missing or differs from its recorded path")
        if hashlib.sha256(urdf.read_bytes()).hexdigest() != _sha256(record.get("sha256"), "URDF"):
            raise ValueError("task URDF SHA-256 mismatch")
    elif assets:
        raise ValueError("task records model assets without a configured model")
    return {"config": config, "job": job, "manifest": manifest, "root": root}


def export_task(
    output_dir, config_path, strokes_path, measurements_path, calibration, plan, job, report
) -> dict:
    """Publish a complete fixed-job snapshot, or leave no executable task.

    Inputs are copied, deployment paths are normalized relative to the source
    configuration, and all source/artifact bytes are hashed in manifest.json.
    ``build_plan.py`` writes ``plan.rebuilt.json``; ``run.sh`` always uses the
    original compiled ``job.json`` and never rebuilds or changes its ID.
    """
    from .calibration import calibrate_measurements

    output = Path(output_dir).expanduser().absolute()
    if output.exists() or output.is_symlink():
        raise FileExistsError(f"task output already exists: {output}")
    sources = {
        name: Path(path).expanduser().resolve()
        for name, path in (
            ("config", config_path),
            ("strokes", strokes_path),
            ("measurements", measurements_path),
        )
    }
    raw = {name: path.read_bytes() for name, path in sources.items()}
    strokes = json.loads(raw["strokes"])
    measurements = json.loads(raw["measurements"])
    if not isinstance(measurements, dict) or measurements.get("purpose") != plan.get("purpose"):
        raise ValueError("measurements and plan must have the same explicit purpose")
    if calibration.get("provenance", {}).get("measurements_sha256") != fingerprint(measurements):
        raise ValueError("calibration provenance does not match the source measurements")
    if fingerprint(calibrate_measurements(measurements)) != fingerprint(calibration):
        raise ValueError("calibration content differs from the source measurements")
    rebuilt = build_plan(strokes, calibration, plan.get("job_id"))
    if fingerprint(rebuilt) != fingerprint(plan):
        raise ValueError("plan differs from its source strokes/calibration")
    job = validate_job(job)
    for key, value in plan.items():
        if key not in {"batches", "park_q14"} and job.get(key) != value:
            raise ValueError(f"compiled job differs from plan: {key}")
    if len(job["batches"]) != len(plan["batches"]) or job["park_q14"][1:] != plan["park_q14"][1:]:
        raise ValueError("compiled job changes the planned batches or reviewed park route")
    for planned, compiled in zip(plan["batches"], job["batches"], strict=False):
        if any(compiled.get(key) != value for key, value in planned.items()):
            raise ValueError("compiled job changes planned batch geometry")
    if (
        not isinstance(report, dict)
        or report.get("job_id") != job["job_id"]
        or report.get("sha256") != fingerprint(job)
    ):
        raise ValueError("report must identify the exact compiled job and its sha256")
    config = load_config(sources["config"])
    command = "simulate" if job["purpose"] == "simulation" else "run"
    run_script = (
        "#!/bin/sh\nset -eu\n"
        'TASK_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)\n'
        'exec robot-paint-execute run-task --task "$TASK_DIR"\n'
    )
    build_script = (
        "#!/usr/bin/env python3\n"
        '"""Rebuild the offline plan; the fixed executable job is unchanged."""\n'
        "import argparse\nimport json\nfrom pathlib import Path\n"
        "from unitree_gpt6.g1d_draw.src.task import build_plan\n\n"
        "base = Path(__file__).resolve().parent\n"
        "parser = argparse.ArgumentParser(description=__doc__)\n"
        "parser.add_argument('--output', type=Path, default=base / 'plan.rebuilt.json')\n"
        "args = parser.parse_args()\n"
        "strokes = json.loads((base / 'strokes.json').read_text(encoding='utf-8'))\n"
        "calibration = json.loads((base / 'calibration.json').read_text(encoding='utf-8'))\n"
        f"plan = build_plan(strokes, calibration, {job['job_id']!r})\n"
        "args.output.write_text(json.dumps(plan, ensure_ascii=False, indent=2, allow_nan=False)"
        " + '\\n', encoding='utf-8')\n"
    )
    artifacts = {
        "source_device.json": raw["config"],
        "device.json": _json_bytes(config),
        "strokes.json": raw["strokes"],
        "measurements.json": raw["measurements"],
        "calibration.json": _json_bytes(calibration),
        "plan.json": _json_bytes(plan),
        "job.json": _json_bytes(job),
        "report.json": _json_bytes(report),
        "preview.svg": render_preview(strokes, calibration).encode("utf-8"),
        "build_plan.py": build_script.encode("utf-8"),
        "run.sh": run_script.encode("utf-8"),
    }
    manifest = {
        "format": TASK_FORMAT,
        "job_id": job["job_id"],
        "purpose": job["purpose"],
        "command": command,
        "job_sha256": fingerprint(job),
        "source_paths": {name: str(path) for name, path in sources.items()},
        "source_sha256": {name: hashlib.sha256(data).hexdigest() for name, data in raw.items()},
        "artifacts_sha256": {
            name: hashlib.sha256(data).hexdigest() for name, data in artifacts.items()
        },
        "physical_execution": False,
        "preview_is_photograph": False,
        "collision_detection": False,
    }
    if config.get("model"):
        urdf = Path(config["model"]["urdf"])
        manifest["model_assets"] = {
            "urdf": {"path": str(urdf), "sha256": hashlib.sha256(urdf.read_bytes()).hexdigest()}
        }
    elif job["purpose"] == "painting":
        raise ValueError("physical task requires a recorded URDF model")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=f".{output.name}.building-", dir=output.parent))
    try:
        for name, data in artifacts.items():
            (temporary / name).write_bytes(data)
        (temporary / "manifest.json").write_bytes(_json_bytes(manifest))
        verify_task(temporary)
        # Nothing executable exists at the requested path until the snapshot
        # is complete. A failed write is removed together with its scripts.
        (temporary / "run.sh").chmod(0o700)
        (temporary / "build_plan.py").chmod(0o700)
        if output.exists() or output.is_symlink():
            raise FileExistsError(f"task output already exists: {output}")
        os.rename(temporary, output)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
    return {"output": str(output), **manifest}
