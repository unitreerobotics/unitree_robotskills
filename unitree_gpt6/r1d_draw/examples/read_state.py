"""Read the R1-D's joints and temperatures. Read-only: no command is sent.

python read_state.py --config device.json [--output state.json]
"""

import argparse
import json

from unitree_gpt6.r1d_draw.src.robot import connect


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    robot = connect(args.config)
    try:
        state = robot.state()
    finally:
        robot.close()
    if args.output:
        with open(args.output, "x", encoding="utf-8") as stream:
            json.dump(state, stream, indent=2)
    for hand in ("left", "right"):
        print(f"{hand} arm (rad): {[round(v, 4) for v in state['arms'][hand]]}")
    print(
        f"waist yaw {state['waist_yaw']:.4f}  head {[round(v, 4) for v in state['head']]}  "
        f"grippers {[round(v, 3) for v in state['grip']]}"
    )
    print(
        f"hottest arm motor {max(state['temperature_c'][15:29]):.0f} C; "
        f"another program is commanding the joints: {state['commanded']}; "
        f"motor faults: {state['motor_faults']}"
    )


if __name__ == "__main__":
    raise SystemExit(main())
