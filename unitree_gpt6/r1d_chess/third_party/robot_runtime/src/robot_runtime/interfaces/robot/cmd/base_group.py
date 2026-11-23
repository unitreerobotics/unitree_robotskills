import math


class BaseGroupCmd:
    """Body, base, lift speed, and waist joint targets.

    Only supported targets are populated. Body z velocity and lift height
    are feedback-only. Lift speed retains native protocol units.
    """

    def __init__(
        self,
        *,
        body_linear_vel_x: float | None = None,
        body_linear_vel_y: float | None = None,
        body_angular_vel_z: float | None = None,
        body_height: float | None = None,
        base_linear_vel_x: float | None = None,
        base_linear_vel_y: float | None = None,
        base_angular_vel_z: float | None = None,
        lift_linear_vel_z: float | None = None,
        waist_roll_qpos: float | None = None,
        waist_pitch_qpos: float | None = None,
        waist_yaw_qpos: float | None = None,
    ) -> None:
        self.body_linear_vel_x = body_linear_vel_x
        self.body_linear_vel_y = body_linear_vel_y
        self.body_angular_vel_z = body_angular_vel_z
        self.body_height = body_height
        self.base_linear_vel_x = base_linear_vel_x
        self.base_linear_vel_y = base_linear_vel_y
        self.base_angular_vel_z = base_angular_vel_z
        self.lift_linear_vel_z = lift_linear_vel_z
        self.waist_roll_qpos = waist_roll_qpos
        self.waist_pitch_qpos = waist_pitch_qpos
        self.waist_yaw_qpos = waist_yaw_qpos

        for name, value in vars(self).items():
            if value is not None and not math.isfinite(value):
                raise ValueError(f"BaseGroupCmd.{name} must be finite.")
