"""Locate resources relative to this checkout; no machine-specific paths."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
