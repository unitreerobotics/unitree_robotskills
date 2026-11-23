"""R1-D control SDK. Importing never connects to hardware."""

from pathlib import Path
import sys

# Bundled, unmodified Unitree DDS types/CRC; no global installation required.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "third_party/unitree_sdk2_python"))

from .client import R1D
from .session import ControlSession

__all__ = ["R1D", "ControlSession"]
