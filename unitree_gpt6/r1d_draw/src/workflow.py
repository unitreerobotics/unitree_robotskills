"""Task templates and the measured-plan-to-job workflow. No hardware is touched here."""

from __future__ import annotations

import copy
from pathlib import Path

from .config import load_config
from .storage import atomic_json, read_json

DEMO_DEVICE = {"host": "127.0.0.1", "mac": "00:00:00:00:00:00"}


def prepare_task(*, config_path, strokes_path, measurements_path, job_id, output, simulation=False):
    """Calibrate, plan, compile, check and export a complete task directory."""
    from .calibration import calibrate_measurements
    from .job import validate_job
    from .kinematics import PinocchioKinematics, compile_job
    from .runner import preflight
    from .task import build_plan, export_task

    config = load_config(config_path)
    measurements, strokes = read_json(measurements_path), read_json(strokes_path)
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
        model = PinocchioKinematics(config, plan["waist_yaw"])
    job = validate_job(compile_job(plan, model))
    report = preflight(job, config, model)
    report.update(synthetic_model=simulation, calibration=calibration["paper"]["report"])
    return export_task(
        output, config_path, strokes_path, measurements_path, calibration, plan, job, report
    )


def task_inputs(*, simulation=False):
    """Blank forms for a new task, or a small synthetic task for the offline demo."""
    purpose = "simulation" if simulation else "painting"
    config = read_json(Path(__file__).resolve().parents[1] / "config/device.example.json")
    strokes = {
        "format": "r1d-draw-strokes/v1",
        "purpose": purpose,
        "paper_size_m": [None, None],
        "palette": {},
        "strokes": [],
    }
    sample = {
        "uv_m": [None, None],
        "xyz_m": [None, None, None],
        "contact_verified": False,
        "source": "",
    }
    station = {
        "action": [],
        "contact_index": None,
        "clearance_m": None,
        "dwell_s": None,
        "contact_verified": False,
        "source": "",
    }
    measurements = {
        "format": "r1d-draw-measurements/v1",
        "purpose": purpose,
        "paper": {
            "size_m": [None, None],
            "anchors": [copy.deepcopy(sample) for _ in range(3)],
            "checks": [copy.deepcopy(sample)],
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
        "materials": {"blue": copy.deepcopy(station)},
        "robot": {
            "hand": "right",
            "grip_q2": [None, None],
            "waist_yaw": None,
            "entry_q14": [],
            "park_q14": [None],
            "transfer_z_m": None,
            "clearance_review": "",
        },
    }
    if not simulation:
        return config, strokes, measurements

    config = {
        "device": {**DEMO_DEVICE, "robot_id": "offline-demo"},
        "storage": {"directory": "../runtime"},
    }
    strokes.update(
        paper_size_m=[0.06, 0.04],
        palette={"blue": "#397eb9", "green": "#76ac48"},
        reload_every=2,
        wash_station="water",
    )
    strokes["strokes"] = [
        {"id": "blue-1", "color": "blue", "points_uv_m": [[0.008, 0.009], [0.052, 0.009]]},
        {"id": "blue-2", "color": "blue", "points_uv_m": [[0.052, 0.015], [0.008, 0.015]]},
        {"id": "blue-3", "color": "blue", "points_uv_m": [[0.008, 0.021], [0.052, 0.021]]},
        {"id": "green-1", "color": "green", "points_uv_m": [[0.008, 0.033], [0.052, 0.033]]},
    ]

    def anchor(u, v):
        return {
            "uv_m": [u, v],
            "xyz_m": [u, v, 0.02 + 0.02 * u - 0.01 * v],
            "contact_verified": False,
            "source": "synthetic",
        }

    def at(x, y, z):
        return {"x": x, "y": y, "z": z, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}

    def place(x, y, dwell):
        return {
            "action": [at(x, y, 0.11), at(x, y, 0.04), at(x, y, 0.11)],
            "contact_index": 1,
            "clearance_m": 0.05,
            "dwell_s": dwell,
            "contact_verified": False,
            "source": "synthetic",
        }

    measurements["paper"] = {
        "size_m": [0.06, 0.04],
        "tolerance_m": 0.0001,
        "anchors": [anchor(0.0, 0.0), anchor(0.06, 0.0), anchor(0.0, 0.04)],
        "checks": [anchor(0.04, 0.03)],
    }
    measurements["brush"].update(
        rpy_rad=[0.0, 0.0, 0.0],
        width_m=0.006,
        max_stroke_length_m=0.05,
        clearance_m=0.025,
        source="synthetic",
    )
    measurements["materials"] = {
        "blue": place(0.08, 0.01, 0.1),
        "green": place(0.08, 0.04, 0.1),
        "water": place(0.08, 0.08, 0.5),
    }
    rest = [0.0] * 14
    rest[9] = 0.15
    measurements["robot"].update(
        grip_q2=[0.0, 0.0],
        waist_yaw=0.0,
        entry_q14=[rest],
        park_q14=[None, rest],
        transfer_z_m=0.15,
        clearance_review="Synthetic offline data; no clearance claim.",
    )
    return config, strokes, measurements


def initialize_task(output, *, simulation=False):
    directory = Path(output).expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=False)
    names = ("device.json", "strokes.json", "measurements.json")
    for name, value in zip(names, task_inputs(simulation=simulation), strict=True):
        atomic_json(directory / name, value)
    return {
        "output": str(directory),
        "files": list(names),
        "ready_to_execute": False,
        "purpose": "simulation" if simulation else "painting",
    }


def run_demo_workflow(output):
    """Plan, compile and execute a synthetic task with the simulated robot."""
    from .runner import Executor
    from .simulation import SimulatedKinematics, SimulatedRobot

    output = Path(output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    inputs = output / "inputs"
    initialize_task(inputs, simulation=True)
    exported = prepare_task(
        config_path=inputs / "device.json",
        strokes_path=inputs / "strokes.json",
        measurements_path=inputs / "measurements.json",
        job_id="offline-demo-1",
        output=output / "ready",
        simulation=True,
    )
    job = read_json(output / "ready" / "job.json")
    robot = SimulatedRobot(job["entry_q14"][0], waist_yaw=job["waist_yaw"], grip_q2=job["grip_q2"])
    result = Executor(
        load_config(output / "ready" / "device.json"), robot, SimulatedKinematics()
    ).run(job)
    summary = {
        "physical_execution": False,
        "output": str(output),
        "state": result["state"],
        "completed_batches": result["completed_batches"],
        "completed_strokes": result["completed_strokes"],
        "parked": result["parked"],
        "routes_sent": len(robot.routes),
        "error": result.get("error"),
        "export": exported,
    }
    atomic_json(output / "demo_report.json", summary)
    return summary
