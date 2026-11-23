"""Read-only: print arm, gripper, waist, head joint state and lift height once."""
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from r1_d.config import DEFAULTS
from r1_d.sdk import R1D
from r1_d.sdk.model import GROUPS, HOLD

robot = R1D(DEFAULTS["ip"], DEFAULTS["interface"], transport=DEFAULTS["transport"], multicast=True)
robot.inspect(1.0)
state = robot.arm_state()
out = {name: [round(state.motor_state[i].q, 4) for i in ids] for name, ids in GROUPS.items()}
for label, i in zip(("waist", "head_pitch", "head_yaw"), HOLD):
    m = state.motor_state[i]
    out[label] = {"q": round(m.q, 4), "tau": round(m.tau_est, 3), "temp": list(m.temperature)}
out["temperature_max"] = max(state.motor_state[i].temperature[0] for ids in GROUPS.values() for i in ids)
out["arm_tau"] = {n: [round(state.motor_state[i].tau_est, 2) for i in GROUPS[n]] for n in ("left_arm", "right_arm")}
out["height"] = robot.height()
print(json.dumps(out))
robot.close()
