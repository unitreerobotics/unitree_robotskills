"""Check the R1-D sandwich skill's code, model and Python dependencies without touching the robot."""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR.parent / "src"))

from service import paths  # noqa: E402


def _row(label: str, ok: bool, detail: str) -> dict:
    return {"check": label, "ok": ok, "detail": detail}


def _import_check(python: str, modules: str, roots: list[Path]) -> dict:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        [*[str(p) for p in roots if p], *[p for p in env.get("PYTHONPATH", "").split(os.pathsep) if p]]
    )
    try:
        proc = subprocess.run(
            [python, "-c", f"import {modules}"],
            capture_output=True,
            text=True,
            env=env,
            timeout=60,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return _row(f"import {modules}", False, str(exc))
    lines = (proc.stderr or proc.stdout).strip().splitlines()
    detail = "ok" if proc.returncode == 0 else (lines[-1] if lines else f"exit {proc.returncode}")
    return _row(f"import {modules}", proc.returncode == 0, detail)


def collect_checks(
    *,
    src_dir: Path,
    model_dir: Path,
    urdf: Path,
    config_dir: Path,
    sdk2_root: Path | None,
    python: str = sys.executable,
) -> list[dict]:
    needed = [src_dir / "ik" / "kin.py", src_dir / "robot" / "ik_executor.py", src_dir / "robot" / "model.py",
              model_dir / "r1d.urdf", model_dir / "r1d_calibrated.urdf", model_dir / "a7d_cal_ik.urdf",
              model_dir / "meshes"]
    missing = [str(p) for p in needed if not p.exists()]
    rows = [
        _row("IK code and R1-D model", not missing, f"missing {missing}" if missing else f"{src_dir}, {model_dir}"),
        _row("R1-D URDF", urdf.is_file(), str(urdf)),
        _import_check(python, "numpy, cv2, zmq, pinocchio, coal", []),
        _import_check(python, "robokit, warp", []),
        _import_check(python, "unitree_sdk2py", [sdk2_root]),
        _import_check(python, "ik.kin, ik.quality, ik.solvers, robot.ik_executor, robot.brain", [src_dir, sdk2_root]),
    ]
    configs = sorted(config_dir.glob("*.json")) if config_dir.is_dir() else []
    rows.append(_row("assets/config/*.json", bool(configs),
                     f"{len(configs)} file(s)" if configs else f"missing JSON files in {config_dir}"))
    for cfg in configs:
        try:
            json.loads(cfg.read_text(encoding="utf-8"))
            ok, detail = True, "valid JSON"
        except (OSError, json.JSONDecodeError) as exc:
            ok, detail = False, str(exc)
        rows.append(_row(f"config {cfg.name}", ok, detail))
    return rows


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--json", action="store_true", help="print machine-readable JSON")
    args = ap.parse_args(argv)
    rows = collect_checks(
        src_dir=paths.PACKAGE_ROOT / "src",
        model_dir=paths.MODEL_DIR,
        urdf=paths.R1D_URDF,
        config_dir=paths.CONFIG_DIR,
        sdk2_root=paths.UNITREE_SDK2_ROOT,
    )
    ok = all(row["ok"] for row in rows)
    if args.json:
        print(json.dumps({"ok": ok, "checks": rows}, indent=2))
    else:
        width = max(len(row["check"]) for row in rows)
        for row in rows:
            print(f"{'OK' if row['ok'] else 'MISSING':7}  {row['check']:<{width}}  {row['detail']}")
        print("\nEnvironment ready." if ok else "\nEnvironment is incomplete.")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
