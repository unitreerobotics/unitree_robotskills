# src/tests/test_frame_content.py
# -*- coding: utf-8 -*-
"""Tool image content: single image (G1D) vs. full frame + center crop pair (R1D)."""
from mcp.server.fastmcp import Image

import navi_mcp_server as server


def test_single_image_without_crop():
    res = {"ok": True, "frame_id": 3}
    out = server._frame_content(res, {"jpeg": b"full"})
    assert out[0] is res and "images" not in res
    assert len(out) == 2 and isinstance(out[1], Image) and out[1].data == b"full"


def test_normal_view_first_then_wide(monkeypatch):
    """R1D: zoomed normal-FOV center crop first (for judging what is ahead and distance), then the ultra-wide full frame; angles labeled from the model's HFOV."""
    from types import SimpleNamespace
    monkeypatch.setattr(server, "_driver_profile", lambda d=None: SimpleNamespace(extra={"camera_hfov_deg": 105.0}))
    res = {"ok": True, "frame_id": 7}
    out = server._frame_content(res, {"jpeg": b"full", "crop_jpeg": b"crop", "crop_ratio": 0.5})
    assert res["images"] == ["normal", "wide"] and res["crop_ratio"] == 0.5
    assert [type(x) for x in out] == [dict, str, Image, str, Image]
    assert out[2].data == b"crop" and out[4].data == b"full"
    assert "frame 7" in out[1] and "正常视场" in out[1] and "50%" in out[1] and "约 66°" in out[1]
    assert "frame 7" in out[3] and "超广角全幅" in out[3] and "约 105°" in out[3]


def test_fov_text_empty_without_profile_fov(monkeypatch):
    monkeypatch.setattr(server, "_driver_profile", lambda d=None: None)
    assert server._fov_text(0.5) == ("", "")
