"""Resolve installed assets without a checkout path or device initialization."""

import json
import xml.etree.ElementTree as ET
from pathlib import Path

URDF_NAME = "g1_body29_hand14.urdf"


def asset_directory() -> Path:
    return Path(__file__).resolve().parent / "assets"


def validate_assets() -> dict:
    """Check that the installed URDF and every referenced mesh are present."""
    root = asset_directory().resolve()
    urdf = root / URDF_NAME
    if not urdf.is_file():
        raise ValueError("Bundled G1 URDF missing; reinstall the complete package")
    try:
        document = ET.parse(urdf)
    except ET.ParseError as exc:
        raise ValueError("Bundled G1 URDF is not valid XML") from exc
    meshes = {mesh.attrib.get("filename", "") for mesh in document.iter("mesh")}
    if not meshes or "" in meshes:
        raise ValueError("Bundled G1 URDF has no valid mesh references")
    for name in meshes:
        path = (root / name).resolve()
        if not path.is_relative_to(root) or not path.is_file() or path.stat().st_size == 0:
            raise ValueError(f"Bundled G1 mesh missing or outside assets: {name}")
    manifest = json.loads((root.parent / "manifest.json").read_text(encoding="utf-8"))
    if set(manifest["files"]) != {URDF_NAME, *meshes}:
        raise ValueError("Bundled G1 asset inventory does not match the URDF")
    return {"available": True, "urdf": URDF_NAME, "mesh_count": len(meshes)}
