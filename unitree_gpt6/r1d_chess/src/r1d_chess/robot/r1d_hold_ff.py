"""Waist/head feedforward for the chess service and goinit (50 Hz Runtime path).

Parameters: config/hold_joints.json, fitted on the robot by setup/arm-calibration/r1_d/tools/hold_calib.py.
The motor gains in that file are applied by the Runtime (r1d_runtime
load_experiment); this module adds bias (head gravity), friction while the reference
moves, and a small integral at rest. HoldCompensator is a copy of setup/arm-calibration/r1_d/sdk/hold.py.
"""
import json
import math
from pathlib import Path

import numpy as np

from r1d_chess.paths import ROOT
CONFIG = ROOT/'config/hold_joints.json'
ORDER = ('head_pitch', 'head_yaw', 'waist_yaw')      # order of q[HEAD+WAIST]
RAMP_NM_S = 3.                                        # torque slew when starting / ending


class HoldCompensator:
    """One joint. torque() is called once per control cycle."""

    def __init__(self, params):
        self.coulomb = float(params.get('coulomb', 0.0))
        self.viscous = float(params.get('viscous', 0.0))
        self.bias = tuple(float(v) for v in params.get('bias', (0.0, 0.0, 0.0)))
        self.ki = float(params.get('ki', 0.0))
        self.integral_limit = float(params.get('integral_limit', 0.0))
        self.deadzone = float(params.get('deadzone', 0.001))
        self.limit = float(params.get('limit', 0.0))
        self.fade = float(params.get('fade_seconds', 0.5))
        self.creep = float(params.get('creep', 0.004))
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.direction = 0.0

    def static(self, q):
        c, s, k = self.bias
        return c + s*math.sin(q) + k*math.cos(q)

    def torque(self, q, target, target_velocity, dt, dq=0.0):
        if abs(target_velocity) > 1e-4:
            self.direction = math.copysign(1.0, target_velocity)
            self.integral = 0.0
        else:
            self.direction = math.copysign(max(0.0, abs(self.direction)-dt/self.fade), self.direction)
            error = target-q
            if error*self.integral < 0.0:
                self.integral = 0.0
            if abs(self.direction) == 0.0 and abs(error) > self.deadzone and abs(dq) < self.creep:
                self.integral += self.ki*error*dt
                self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        tau = self.static(target)+self.coulomb*self.direction+self.viscous*target_velocity+self.integral
        return max(-self.limit, min(self.limit, tau))


def load_config(path=CONFIG):
    return json.loads(Path(path).read_text(encoding='utf-8')) if Path(path).is_file() else {}


def runtime_gains(config=None):
    """{'waist': {'kp': [..], 'kd': [..]}, 'head': {...}} for the Runtime joint groups, or {}."""
    config = load_config() if config is None else config
    if not all(name in config and 'kp' in config[name] for name in ORDER):
        return {}
    return {'waist': {'kp': [config['waist_yaw']['kp']], 'kd': [config['waist_yaw']['kd']]},
            'head': {'kp': [config['head_pitch']['kp'], config['head_yaw']['kp']],
                     'kd': [config['head_pitch']['kd'], config['head_yaw']['kd']]}}


class HoldFeedforward:
    """Torques (Nm) for [head_pitch, head_yaw, waist_yaw]; zeros when not configured."""

    def __init__(self, config=None):
        config = load_config() if config is None else config
        self.enabled = all(name in config for name in ORDER)
        self.joints = [HoldCompensator(config[name]) for name in ORDER] if self.enabled else []
        self.output = np.zeros(3)
        self.previous = None
        self.zero = False

    def reset(self):
        for joint in self.joints:
            joint.reset()
        self.previous = None
        self.zero = False

    def request_zero(self, zero):
        self.zero = bool(zero)

    def compute(self, q, dq, target, dt):
        """q, dq, target: head pitch, head yaw, waist yaw. Slew limited from the last output."""
        target = np.asarray(target, float)
        wanted = np.zeros(3)
        if self.enabled and not self.zero:
            velocity = np.zeros(3) if self.previous is None else (target-self.previous)/max(dt, 1e-3)
            wanted = np.array([joint.torque(float(q[i]), float(target[i]), float(velocity[i]), dt, float(dq[i]))
                               for i, joint in enumerate(self.joints)])
        self.previous = target.copy()
        self.output = self.output+np.clip(wanted-self.output, -RAMP_NM_S*dt, RAMP_NM_S*dt)
        return self.output.copy()

    def status(self):
        return {'enabled': self.enabled, 'tau_nm': self.output.tolist(),
                'integral_nm': [joint.integral for joint in self.joints]}
