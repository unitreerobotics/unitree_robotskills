# -*- coding: utf-8 -*-
"""pytest path shim: lets tests/ import top-level modules in the flat src/ layout."""
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))


@pytest.fixture(autouse=True)
def _no_lidar_dds(monkeypatch):
    """Never subscribe to lidar DDS in unit tests (no ChannelFactory in the test process; it would hang).

    Reflex-layer logic is covered by tests/test_lidar.py constructing LidarGuard directly;
    on hardware, navi.lidar.enabled decides."""
    monkeypatch.setenv("NAVI_NO_LIDAR", "1")
