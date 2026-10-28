"""Task setup and the measured-plan-to-execution workflow, with lazy adapters.

The agent supplies the artistic stroke plan. Real measurements require physical
confirmation; the offline example is a synthetic transport, never G1 geometry.
"""

from __future__ import annotations

import copy
import json
from pathlib import Path

from .config import load_config
from .postures import default_grip_q2, default_rest_q14, default_work_q14
from .storage import atomic_json


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def prepare_task(*, config_path, strokes_path, measurements_path, job_id, output, simulation=False):
    """Calibrate, assemble, compile, inspect and export. No robot connection."""
    from .calibration import calibrate_measurements
    from .job import validate_job
    from .kinematics import PinocchioKinematics, compile_job
    from .runner import preflight
    from .task import build_plan, export_task

    config = load_config(config_path)
    measurements = read_json(measurements_path)
    strokes = read_json(strokes_path)
    expected = "simulation" if simulation else "painting"
    if measurements.get("purpose") != expected or strokes.get("purpose") != expected:
        raise ValueError(
            f"prepare requires purpose={expected} in both measurement and stroke files"
        )
    calibration = calibrate_measurements(measurements)
    plan = build_plan(strokes, calibration, job_id)
    if simulation:
        from .simulation import SimulatedKinematics

        model = SimulatedKinematics()
    else:
        model = PinocchioKinematics(config)
    job = validate_job(compile_job(plan, model))
    report = preflight(job, config, model)
    report.update(
        model_checked=True, synthetic_model=simulation, calibration=calibration["paper"]["report"]
    )
    return export_task(
        output, config_path, strokes_path, measurements_path, calibration, plan, job, report
    )


def _configuration():
    """Build a new task configuration with the packaged example's gain defaults."""
    return {
        "device": {
            "robot_id": "g1d",
            "interface": "eth0",
            "domain_id": 0,
            "lowstate_topic": "rt/lowstate",
            "lowcmd_topic": "rt/lowcmd",
            "gripper_interface": "internal",
        },
        "gains": read_json(Path(__file__).resolve().parents[1] / "config/device.example.json")[
            "gains"
        ],
        "model": {
            "urdf": "models/g1.urdf",
            "left_frame": "left_wrist_yaw_joint",
            "right_frame": "right_wrist_yaw_joint",
            "reference_positions": {},
        },
        "storage": {"directory": "../runtime", "additional_lock_files": []},
    }


def task_inputs(*, simulation=False):
    """Return unmeasured forms with editable work/rest targets, or synthetic data."""
    purpose = "simulation" if simulation else "painting"
    config = _configuration()
    strokes = {
        "format": "astra-paint-strokes/v1",
        "purpose": purpose,
        "paper_size_m": [None, None],
        "reload_every": 3,
        "palette": {},
        "strokes": [],
    }
    blank_sample = {
        "uv_m": [None, None],
        "xyz_m": [None, None, None],
        "contact_verified": False,
        "source": "",
    }
    blank_station = {
        "action": [],
        "contact_index": None,
        "clearance_m": None,
        "dwell_s": None,
        "contact_verified": False,
        "source": "",
    }
    measurements = {
        "format": "astra-paint-measurements/v1",
        "purpose": purpose,
        "paper": {
            "size_m": [None, None],
            "anchors": [copy.deepcopy(blank_sample) for _ in range(3)],
            "checks": [copy.deepcopy(blank_sample)],
            "tolerance_m": None,
        },
        "brush": {
            "rpy_rad": [None, None, None],
            "width_m": None,
            "max_stroke_length_m": None,
            "clearance_m": None,
            "contact_verified": False,
            "source": "",
        },
        "materials": {"blue": copy.deepcopy(blank_station), "water": copy.deepcopy(blank_station)},
        "robot": {
            "hand": "right",
            "grip_q2": default_grip_q2(),
            "waist_q3": [None, 0.0, None],
            "entry_q14": [default_rest_q14(), default_work_q14()],
            "park_q14": [None, default_rest_q14()],
            "transfer_z_m": None,
            "clearance_review": "",
        },
    }
    probe = {
        "format": "astra-paint-probe/v1",
        "job_id": "measurement-attempt-1",
        "purpose": "measurement",
        "hand": "right",
        "grip_q2": default_grip_q2(),
        "waist_q3": [None, 0.0, None],
        "entry_q14": [default_rest_q14(), default_work_q14()],
        "action": [],
        "capture_indices": [],
        "lifted_capture_indices": [],
        "park_q14": [None, default_rest_q14()],
        "end_clear": True,
        "clearance_review": "",
    }
    if not simulation:
        return config, strokes, measurements, probe

    config["device"].update(robot_id="synthetic-workflow", interface="offline")
    config.pop("model")
    strokes.update(paper_size_m=[0.06, 0.04], palette={"blue": "#397eb9", "green": "#76ac48"})
    strokes["strokes"] = [
        {"id": "blue-1", "color": "blue", "points_uv_m": [[0.008, 0.009], [0.052, 0.009]]},
        {"id": "blue-2", "color": "blue", "points_uv_m": [[0.052, 0.015], [0.008, 0.015]]},
        {"id": "blue-3", "color": "blue", "points_uv_m": [[0.008, 0.021], [0.052, 0.021]]},
        {"id": "blue-4", "color": "blue", "points_uv_m": [[0.052, 0.027], [0.008, 0.027]]},
        {"id": "green-1", "color": "green", "points_uv_m": [[0.008, 0.033], [0.052, 0.033]]},
    ]

    def sample(u, v):
        return {
            "uv_m": [u, v],
            "xyz_m": [u, v, 0.02 + 0.02 * u - 0.01 * v],
            "contact_verified": False,
            "source": "synthetic fixture, not a physical observation",
        }

    def pose(x, y, z):
        return {"x": x, "y": y, "z": z, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}

    def station(x, y):
        return {
            "action": [pose(x, y, 0.11), pose(x, y, 0.04), pose(x, y, 0.11)],
            "contact_index": 1,
            "clearance_m": 0.05,
            "dwell_s": 0.1,
            "contact_verified": False,
            "source": "synthetic fixture",
        }

    measurements["paper"] = {
        "size_m": [0.06, 0.04],
        "anchors": [sample(0.0, 0.0), sample(0.06, 0.0), sample(0.0, 0.04)],
        "checks": [sample(0.04, 0.03)],
        "tolerance_m": 0.0001,
    }
    measurements["brush"].update(
        rpy_rad=[0.0, 0.0, 0.0],
        width_m=0.006,
        max_stroke_length_m=0.05,
        clearance_m=0.025,
        source="synthetic brush model",
    )
    measurements["materials"] = {
        "blue": station(0.08, 0.01),
        "green": station(0.08, 0.04),
        "water": station(0.08, 0.08),
    }
    measurements["materials"]["water"]["dwell_s"] = 3.0
    rest = [0.0] * 14
    rest[9] = 0.15
    measurements["robot"].update(
        grip_q2=[0.0, 0.0],
        waist_q3=[0.0, 0.0, 0.0],
        entry_q14=[rest],
        park_q14=[None, rest],
        transfer_z_m=0.15,
        clearance_review="Synthetic offline transport only; no collision or contact claim.",
    )
    observed = measurements["paper"]["anchors"] + measurements["paper"]["checks"]
    probe.update(
        purpose="simulation",
        grip_q2=[0.0, 0.0],
        waist_q3=[0.0, 0.0, 0.0],
        entry_q14=[rest],
        park_q14=[None, rest],
        action=[pose(*point["xyz_m"]) for point in observed] + [pose(0.0, 0.0, 0.15)],
        capture_indices=list(range(len(observed))),
        clearance_review=measurements["robot"]["clearance_review"],
    )
    return config, strokes, measurements, probe


def initialize_task(output, *, simulation=False):
    directory = Path(output).expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=False)
    values = task_inputs(simulation=simulation)
    names = ("device.json", "strokes.json", "measurements.json", "probe.template.json")
    for name, value in zip(names, values, strict=False):
        atomic_json(directory / name, value)
    return {
        "output": str(directory),
        "purpose": "simulation" if simulation else "painting",
        "ready_to_execute": False,
        "files": list(names),
    }


def run_demo_workflow(output):
    """Exercise the same measurement/compiler/executor APIs with synthetic data."""
    from .calibration import observe_device
    from .probe import ProbeExecutor, compile_probe
    from .runner import Executor
    from .simulation import SimulatedBackend, SimulatedKinematics

    output = Path(output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    inputs = output / "inputs"
    initialize_task(inputs, simulation=True)
    config_path = inputs / "device.json"
    config = load_config(config_path)
    model = SimulatedKinematics()
    probe = compile_probe(read_json(inputs / "probe.template.json"), model)
    atomic_json(output / "probe.json", probe)
    observer = SimulatedBackend(probe)
    observation = observe_device(config, observer, model, "synthetic rest")
    atomic_json(output / "observation.json", observation)
    measurement_runner = ProbeExecutor(config, SimulatedBackend(probe), model)
    try:
        measured = measurement_runner.run(probe)
        if measured["state"] != "job_completed":
            raise RuntimeError(f"synthetic measurement failed: {measured}")
        records = read_json(inputs / "measurements.json")
        points = records["paper"]["anchors"] + records["paper"]["checks"]
        for index, point in enumerate(points):
            capture_path = measurement_runner.directory / "captures" / f"{index:05d}.json"
            capture = read_json(capture_path)
            point["xyz_m"] = [row[3] for row in capture["right_fk"][:3]]
            point["source"] = str(capture_path)
        atomic_json(inputs / "measurements.json", records)
    finally:
        measurement_runner.close()
    ready = output / "ready"
    exported = prepare_task(
        config_path=config_path,
        strokes_path=inputs / "strokes.json",
        measurements_path=inputs / "measurements.json",
        job_id="synthetic-painting-attempt-1",
        output=ready,
        simulation=True,
    )
    job = read_json(ready / "job.json")
    executor = Executor(load_config(ready / "device.json"), SimulatedBackend(job), model)
    try:
        result = executor.run(job)
    finally:
        executor.close()
    summary = {
        "physical_execution": False,
        "purpose": "simulation",
        "output": str(output),
        "measurement_state": measured["state"],
        "painting_state": result["state"],
        "completed_batches": result["completed_batches"],
        "completed_strokes": result["completed_strokes"],
        "parked": result["parked"],
        "export": exported,
        "note": "Synthetic workflow test; not hardware or paint-quality validation.",
    }
    atomic_json(output / "demo_report.json", summary)
    return summary
