"""Settings and locations for the R1-D sandwich skill.

Everything the skill needs is inside this directory: IK, the executor and robot I/O are in ``src/``, the R1-D
model is in ``assets/r1d``, and the camera client is ``src/perception/image_client.py``. Settings come from the
environment, then ``<package root>/.env``; none is required.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
ASSETS_DIR = PACKAGE_ROOT / "assets"
CONFIG_DIR = ASSETS_DIR / "config"

_DEFAULTS = {
    "SERVER_URL": "http://127.0.0.1:8765",
    "RUNS_DIR": str(PACKAGE_ROOT / "runs"),
}


def _read_dotenv(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path.is_file():
        return values
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[7:].lstrip()
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        key, value = key.strip(), value.strip()
        if not key:
            continue
        if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
            value = value[1:-1]
        values[key] = value
    return values


_DOTENV = _read_dotenv(PACKAGE_ROOT / ".env")


def getenv(name: str) -> str | None:
    """Return a setting using environment > .env > built-in default; None when it is not set anywhere."""
    for source in (os.environ, _DOTENV, _DEFAULTS):
        if source.get(name):
            return source[name]
    return None


def _path(name: str, resolve: bool = True) -> Path | None:
    value = getenv(name)
    if value is None:
        return None
    path = Path(value).expanduser()
    return path.resolve() if resolve else path


MODEL_DIR = ASSETS_DIR / "r1d"                           # R1-D URDFs and meshes
R1D_URDF = _path("R1D_URDF") or MODEL_DIR / "r1d.urdf"
UNITREE_SDK2_ROOT = _path("UNITREE_SDK2_ROOT")           # optional: a checkout instead of the installed package
SERVER_URL = getenv("SERVER_URL").rstrip("/")
RUNS_DIR = _path("RUNS_DIR")


def ensure_external_paths() -> None:
    """Put the optional unitree_sdk2_python checkout on sys.path when UNITREE_SDK2_ROOT is set."""
    if UNITREE_SDK2_ROOT is not None and str(UNITREE_SDK2_ROOT) not in sys.path:
        sys.path.insert(0, str(UNITREE_SDK2_ROOT))
