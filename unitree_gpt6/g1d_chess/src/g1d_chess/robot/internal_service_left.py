"""Left-hand entry point for the same serial chess workflow.

Select this service BEFORE planning. The existing right-hand service is left
unchanged. Protocol plan slots represent roles in this entry: right=left actor,
left=right observer. Physical base XYZ and camera stream names are unchanged.
"""
from g1d_chess.robot import internal_service as base
from g1d_chess.robot.arm_roles import LeftActorDDS, LeftActorMotion, physical_motor


class LeftRobotService(base.RobotService):
    active_arm = "left"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.dds = LeftActorDDS(self.dds)
        self.motion = LeftActorMotion(self.motion)

    def state(self):
        state = super().state()
        state.update(active_arm="left", arm_field_frame="transaction_roles",
                     arm_role_mapping={"left": "physical_right_observer",
                                       "right": "physical_left_actor"})
        for gripper in state["grippers"].values():
            gripper["motor_index"] = physical_motor(gripper["motor_index"])
        state["motor_diagnostics"] = {
            str(physical_motor(int(k))): v for k, v in state["motor_diagnostics"].items()}
        return state

    def log(self, event):
        super().log({"active_arm": "left", "arm_field_frame": "transaction_roles", **event})

    def dispatch(self, req):
        cmd = req.get("cmd")
        if cmd == "set_active_arm":
            raise ValueError("arm is fixed by entry point; select another service before planning")
        motion_commands = {"enable_control", "set_gripper", "prepare_plan", "execute_plan",
                           "prepare_cached_plan", "execute_cached_plan", "prepare_transaction",
                           "restore_cached_transaction", "execute_transaction", "prepare_sequence", "execute_sequence"}
        if cmd in motion_commands and req.get("expected_active_arm") != "left":
            raise ValueError("left service requires expected_active_arm=left; plan slots represent roles")
        result = super().dispatch(req)
        return {**result, "active_arm": "left", "arm_field_frame": "transaction_roles"}


if __name__ == "__main__":
    # Reuse the unchanged server loop, local endpoint guard, signal handling,
    # shutdown and single REP connection; only its constructor is selected.
    base.RobotService = LeftRobotService
    base.main()
