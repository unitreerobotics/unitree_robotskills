"""
G1D AGV (wheeled chassis) client for unitree_sdk2py.

Python re-implementation of the AgvClient interface from unitree_sdk2 (C++,
unitree_sdk2/include/unitree/robot/g1/agv/), built on top of the
unitree_sdk2py RPC layer.

Service : "agv"
API     : 1001 ROBOT_API_ID_AGV_MOVE
          1002 ROBOT_API_ID_AGV_HEIGHT_ADJUST
Limits  : vx   in [-1.5, 1.5] m/s   (forward +, backward -)
          vy   not supported by the AGV, ignored (pass 0.0)
          vyaw in [-0.6, 0.6] rad/s (ccw +, cw -)
          vz   in [-1.0, 1.0] normalized column velocity
               (vz * 76.5 mm/s, +1.0 = max rise, -1.0 = max descent)
"""

import json

from unitree_sdk2py.rpc.client import Client

# service name / api version (must match the on-board agv server)
AGV_SERVICE_NAME = "agv"
AGV_API_VERSION = "1.0.0.1"

# api id
ROBOT_API_ID_AGV_MOVE = 1001
ROBOT_API_ID_AGV_HEIGHT_ADJUST = 1002

# mechanical limits, used by Move()/HeightAdjust() to reject bad commands
AGV_MAX_VX = 1.5      # m/s
AGV_MAX_VYAW = 0.6    # rad/s
AGV_MAX_VZ = 1.0      # normalized


class AgvClient(Client):
    """Linear/angular velocity + height-column control for the G1D chassis."""

    def __init__(self):
        super().__init__(AGV_SERVICE_NAME, False)

    def Init(self):
        self._SetApiVerson(AGV_API_VERSION)
        self._RegistApi(ROBOT_API_ID_AGV_MOVE, 0)
        self._RegistApi(ROBOT_API_ID_AGV_HEIGHT_ADJUST, 0)

    def Move(self, vx: float, vy: float, vyaw: float):
        """
        Velocity control. Non-blocking, streams a single command; call it
        periodically (e.g. every 50 ms) for continuous motion.

        :param vx:   linear velocity x, m/s, [-1.5, 1.5], + = forward
        :param vy:   lateral velocity y, unsupported, pass 0.0
        :param vyaw: angular velocity around z, rad/s, [-0.6, 0.6], + = ccw
        :return: (code, data), code 0 on success
        """
        vx = float(vx)
        vy = float(vy)
        vyaw = float(vyaw)

        if abs(vx) > AGV_MAX_VX:
            raise ValueError(f"vx {vx} out of range [-{AGV_MAX_VX}, {AGV_MAX_VX}] m/s")
        if abs(vyaw) > AGV_MAX_VYAW:
            raise ValueError(f"vyaw {vyaw} out of range [-{AGV_MAX_VYAW}, {AGV_MAX_VYAW}] rad/s")

        parameter = json.dumps({"vx": vx, "vy": vy, "vyaw": vyaw})
        return self._Call(ROBOT_API_ID_AGV_MOVE, parameter)

    def HeightAdjust(self, vz: float):
        """
        Height column velocity control. vz is normalized [-1.0, 1.0] and maps
        linearly to column velocity: column_velocity(mm/s) = vz * 76.5.
        Send 0.0 to stop the column.

        :param vz: height velocity command, [-1.0, 1.0]
        :return: (code, data), code 0 on success
        """
        vz = float(vz)

        if abs(vz) > AGV_MAX_VZ:
            raise ValueError(f"vz {vz} out of range [-{AGV_MAX_VZ}, {AGV_MAX_VZ}]")

        parameter = json.dumps({"data": vz})
        return self._Call(ROBOT_API_ID_AGV_HEIGHT_ADJUST, parameter)
