class BaseGroupState:
    """Body or physical base/lift feedback; unavailable values stay None.

    Body height is controller feedback, not world position z. Lift height
    uses the device's reference; lift speed retains native protocol units.
    Waist feedback is exposed through JointGroupState only.
    """

    def __init__(
        self,
        *,
        body_linear_vel_x: float | None = None,
        body_linear_vel_y: float | None = None,
        body_linear_vel_z: float | None = None,
        body_angular_vel_z: float | None = None,
        body_height: float | None = None,
        base_linear_vel_x: float | None = None,
        base_linear_vel_y: float | None = None,
        base_angular_vel_z: float | None = None,
        lift_height: float | None = None,
        lift_linear_vel_z: float | None = None,
    ) -> None:
        self.body_linear_vel_x = body_linear_vel_x
        self.body_linear_vel_y = body_linear_vel_y
        self.body_linear_vel_z = body_linear_vel_z
        self.body_angular_vel_z = body_angular_vel_z
        self.body_height = body_height
        self.base_linear_vel_x = base_linear_vel_x
        self.base_linear_vel_y = base_linear_vel_y
        self.base_angular_vel_z = base_angular_vel_z
        self.lift_height = lift_height
        self.lift_linear_vel_z = lift_linear_vel_z
