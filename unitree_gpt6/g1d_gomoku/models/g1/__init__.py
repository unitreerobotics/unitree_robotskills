"""Bundled G1 model; importing resource helpers requires no hardware libraries."""

from .resources import asset_directory, validate_assets

__all__ = ["asset_directory", "validate_assets"]
