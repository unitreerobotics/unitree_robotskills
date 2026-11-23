"""Bounded arm torque feedforward for the R1-D Runtime PD joint controller.

qtau = scale * g(q_cmd or q_fb) + (coulomb + load * |g|) * tanh(dq_cmd / eps) + integral(q_cmd - q_fb)

All terms are optional and configured in config/arm_torque_ff.json. The total
is clipped per joint and rate limited, so enabling, disabling or a sudden
parameter change ramps instead of stepping. Offline only; the caller sends it.
"""
import copy
import json
import os
from pathlib import Path
import numpy as np

from r1d_chess.paths import ROOT
CONFIG = ROOT / 'config/arm_torque_ff.json'
MODES = ('off', 'gravity', 'gravity_i', 'full')


def _vector(value, n, label, *, nonnegative=True):
    array = np.asarray(value, dtype=float)
    if array.shape != (n,) or not np.isfinite(array).all() or (nonnegative and np.any(array < 0)):
        raise ValueError(f'arm_torque_ff {label}: requires {n} finite{" non-negative" if nonnegative else ""} values')
    return array


def _sides(section, key, label, *, nonnegative=True):
    return np.concatenate([_vector(section[key][side], 7, f'{label}.{side}', nonnegative=nonnegative)
                           for side in ('left', 'right')])


def load_config(path=None):
    path = Path(path or os.environ.get('R1D_ARM_TORQUE_FF', CONFIG)).resolve()
    value = json.loads(path.read_text())
    validate(value)
    return value


def validate(c):
    if c.get('schema_version') != 1 or type(c.get('enabled')) is not bool:
        raise ValueError('arm_torque_ff requires schema_version=1 and boolean enabled')
    _sides(c['gravity'], 'scale', 'gravity.scale')
    for side in ('left', 'right'):
        p = c['gravity']['payload'][side]
        if not 0 <= float(p['mass_kg']) <= 2.:
            raise ValueError('payload mass must be within [0, 2] kg')
        _vector(p['com_xyz_m'], 3, f'payload.{side}.com', nonnegative=False)
    _sides(c['friction'], 'coulomb_nm', 'friction.coulomb_nm')
    if 'load_coefficient' in c['friction']:
        # Dry friction that grows with the gravity torque the joint carries (gearbox loss).
        if np.any(_sides(c['friction'], 'load_coefficient', 'friction.load_coefficient') > 1.):
            raise ValueError('friction.load_coefficient must be within [0, 1]')
    if c['gravity'].get('at', 'command') not in ('command', 'feedback'):
        raise ValueError('gravity.at must be command or feedback')
    if not float(c['friction']['velocity_epsilon_rad_s']) > 0:
        raise ValueError('friction velocity epsilon must be positive')
    if float(c['friction'].get('error_epsilon_rad', 0.)) < 0:
        raise ValueError('friction error epsilon must be >= 0 (0 disables)')
    _vector(c['integral']['ki_nm_per_rad_s'], 7, 'integral.ki')
    _vector(c['integral']['limit_nm'], 7, 'integral.limit')
    if not float(c['integral']['error_window_rad']) > 0:
        raise ValueError('integral error window must be positive')
    limit = _vector(c['tau_limit_nm'], 7, 'tau_limit_nm')
    if np.any(limit > 20.):
        raise ValueError('tau_limit_nm above 20 Nm requires a reviewed motor rating')
    if not 0 < float(c['tau_rate_nm_s']) <= 50.:
        raise ValueError('tau_rate_nm_s must be in (0, 50]')
    for key in ('gravity', 'friction', 'integral'):
        if type(c[key].get('enabled')) is not bool:
            raise ValueError(f'{key}.enabled must be boolean')
    if 'damping' in c:
        # Runtime sends dq=0, so motor damping kd*dq brakes every motion; this cancels it.
        if type(c['damping'].get('enabled')) is not bool:
            raise ValueError('damping.enabled must be boolean')
        _vector(c['damping']['kd_nm_s_per_rad'], 7, 'damping.kd')
    return c


def with_mode(config, mode):
    """Benchmark variant of a validated config; never written back to disk."""
    if mode not in MODES:
        raise ValueError(f'mode must be one of {MODES}')
    c = copy.deepcopy(config)
    c['enabled'] = mode != 'off'
    c['gravity']['enabled'] = mode != 'off'
    c['integral']['enabled'] = mode in ('gravity_i', 'full')
    c['friction']['enabled'] = mode == 'full'
    if 'damping' in c:
        c['damping']['enabled'] = mode == 'full'
    return validate(c)


class TorqueFeedforward:
    def __init__(self, config, gravity=None):
        self.config = validate(copy.deepcopy(config))
        if gravity is None and self.config['enabled'] and self.config['gravity']['enabled']:
            from r1d_chess.robot.r1d_gravity import ArmGravity
            model = self.config.get('model_urdf')        # relative: against the package root
            gravity = ArmGravity(ROOT / model if model else None)
        self.gravity = gravity
        c = self.config
        self.scale = _sides(c['gravity'], 'scale', 'scale')
        self.payload = c['gravity']['payload']
        self.coulomb = _sides(c['friction'], 'coulomb_nm', 'coulomb')
        self.load = (_sides(c['friction'], 'load_coefficient', 'load') if 'load_coefficient' in c['friction']
                     else np.zeros(14))
        self.eps = float(c['friction']['velocity_epsilon_rad_s'])
        # Optional: keep breaking stiction toward the target after the command stops.
        self.error_eps = float(c['friction'].get('error_epsilon_rad', 0.))
        self.ki = np.tile(_vector(c['integral']['ki_nm_per_rad_s'], 7, 'ki'), 2)
        self.i_limit = np.tile(_vector(c['integral']['limit_nm'], 7, 'i_limit'), 2)
        self.window = float(c['integral']['error_window_rad'])
        self.limit = np.tile(_vector(c['tau_limit_nm'], 7, 'tau_limit'), 2)
        self.rate = float(c['tau_rate_nm_s'])
        damping = c.get('damping', {'enabled': False, 'kd_nm_s_per_rad': [0.]*7})
        self.damping_enabled = damping['enabled']
        self.kd = np.tile(_vector(damping['kd_nm_s_per_rad'], 7, 'kd'), 2)
        self.integral = np.zeros(14)
        self.output = np.zeros(14)
        self.last_terms = {'gravity': np.zeros(14), 'friction': np.zeros(14), 'integral': np.zeros(14),
                           'damping': np.zeros(14)}
        self.zero_requested = False

    @property
    def enabled(self):
        return self.config['enabled']

    def reset(self):
        """Clear integral state; the rate limiter still ramps the output."""
        self.integral[:] = 0.

    def request_zero(self, value=True):
        self.zero_requested = bool(value)
        if value:
            self.reset()

    def target(self, q_fb, q_cmd, dq_cmd, dt, *, integrate=True):
        c = self.config
        zeros = np.zeros(14)
        terms = {'gravity': zeros.copy(), 'friction': zeros.copy(), 'integral': zeros.copy(),
                 'damping': zeros.copy()}
        if not c['enabled'] or self.zero_requested:
            self.last_terms = terms
            return zeros
        if c['gravity']['enabled']:
            # 'command': gravity evaluated at the commanded joint angles.
            q_model = q_cmd if c['gravity'].get('at', 'command') == 'command' else q_fb
            terms['gravity'] = self.scale * self.gravity.torque(q_model, self.payload)
        if c['friction']['enabled']:
            drive = np.asarray(dq_cmd, float) / self.eps
            if self.error_eps > 0:
                drive = drive + (np.asarray(q_cmd, float) - np.asarray(q_fb, float)) / self.error_eps
            terms['friction'] = (self.coulomb + self.load * np.abs(terms['gravity'])) * np.tanh(drive)
        if self.damping_enabled:
            terms['damping'] = self.kd * np.asarray(dq_cmd, float)
        if c['integral']['enabled']:
            error = np.asarray(q_cmd, float) - np.asarray(q_fb, float)
            # Conditional integration: large transients are not wound up.
            if integrate:
                active = np.abs(error) <= self.window
                self.integral = np.clip(self.integral + active * self.ki * error * dt,
                                        -self.i_limit, self.i_limit)
            terms['integral'] = self.integral.copy()
        self.last_terms = terms
        return terms['gravity'] + terms['friction'] + terms['integral'] + terms['damping']

    def compute(self, q_fb, q_cmd, dq_cmd, dt, *, integrate=True):
        wanted = np.clip(self.target(q_fb, q_cmd, dq_cmd, dt, integrate=integrate), -self.limit, self.limit)
        step = self.rate * dt
        self.output = np.clip(self.output + np.clip(wanted - self.output, -step, step), -self.limit, self.limit)
        return self.output.copy()

    def status(self):
        return {'enabled': self.enabled, 'zero_requested': self.zero_requested,
                'tau_sent_nm': self.output.tolist(), 'integral_nm': self.integral.tolist(),
                'terms_nm': {k: v.tolist() for k, v in self.last_terms.items()}}
