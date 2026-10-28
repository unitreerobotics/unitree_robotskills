"""Look up a target waypoint; this is not a collision-free route planner."""

from typing import Any

from unitree_gpt6.g1d_r1d_navigation.common.config import finite_number, section
from unitree_gpt6.g1d_r1d_navigation.common.contracts import Action


def build_plan(target: str, config: dict[str, Any]) -> list[Action]:
    navigation = section(config, "navigation")
    if navigation.get("coordinate_frame") != "map_unregistered":
        raise ValueError("Only the unregistered demo map frame is supported")
    waypoints = section(config, "waypoints")
    point = waypoints.get(target)
    if not isinstance(point, dict):
        raise ValueError("Waypoint must be a mapping")
    pose = {key: finite_number(point.get(key), key) for key in ("x_m", "y_m", "yaw_rad")}
    return [
        Action(
            "navigation_goal_preview",
            {"target": target, "pose": pose, "frame": navigation["coordinate_frame"]},
        )
    ]
