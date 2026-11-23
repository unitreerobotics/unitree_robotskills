"""Offline checks for arm gravity model, bounded feedforward and qtau encoding."""
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from r1d_chess.robot.r1d_gravity import ArmGravity
from r1d_chess.robot.r1d_torque_ff import TorqueFeedforward, with_mode
from r1d_chess.robot.r1d_torque_ff import load_config as _load_config


def load_config():
    """Fixed test parameters, independent of the tuned config/arm_torque_ff.json."""
    c = _load_config()
    c['gravity']['scale'] = {'left': [1.] * 7, 'right': [1.] * 7}
    c['tau_rate_nm_s'] = 15.
    c['tau_limit_nm'] = [12., 12., 8., 8., 3., 3., 3.]
    c['integral'].update(ki_nm_per_rad_s=[40., 40., 30., 30., 15., 15., 15.],
                         limit_nm=[6., 6., 4., 4., 1.5, 1.5, 1.5], error_window_rad=.25)
    c.pop('damping', None)
    c['friction'].pop('error_epsilon_rad', None)
    return c


class GravityModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gravity = ArmGravity()

    def test_torque_is_potential_gradient(self):
        rng = np.random.default_rng(1)
        for _ in range(10):
            q = rng.uniform(-.8, .8, 14)
            tau = self.gravity.torque(q, {'left': {'mass_kg': .4, 'com_xyz_m': [.06, 0, -.01]}})
            for side, offset in (('left', 0), ('right', 7)):
                mass = .4 if side == 'left' else 0.
                for j in range(7):
                    step = np.zeros(7)
                    step[j] = 1e-6
                    q7 = q[offset:offset+7]
                    numeric = (self.gravity.potential(side, q7+step, mass, (.06, 0, -.01)) -
                               self.gravity.potential(side, q7-step, mass, (.06, 0, -.01))) / 2e-6
                    self.assertAlmostEqual(tau[offset+j], numeric, places=5)

    def test_mirrored_arms(self):
        mirror = np.array([1, -1, -1, 1, -1, 1, -1])
        q = np.array([-.3, .4, .2, .5, -.3, .2, .1])
        tau = self.gravity.torque(np.r_[q, q*mirror])
        np.testing.assert_allclose(tau[7:], tau[:7]*mirror, atol=.05)

    def test_measured_left_shoulder_roll_hold(self):
        # Recorded read-only hold: kp*error = 3.46 Nm on motor 16.
        q = [.158, .813, .232, -.174, -.807, .56, -.358, -.109, -.631, -.437, .481, .559, -.047, .487]
        from r1d_chess.robot.r1d_config import MODEL_PATH
        model = ArmGravity(MODEL_PATH)
        self.assertAlmostEqual(model.torque(q)[1], 4.04, delta=.05)
        self.assertAlmostEqual(model.arms['left'].tool_mass, .5708, delta=1e-4)  # Dex1


class FeedforwardTests(unittest.TestCase):
    def test_off_mode_is_exactly_zero(self):
        ff = TorqueFeedforward(with_mode(load_config(), 'off'))
        q = np.full(14, .2)
        for _ in range(10):
            self.assertFalse(np.any(ff.compute(q, q+.1, np.ones(14), .02)))

    def test_rate_limit_clip_and_integral_window(self):
        config = with_mode(load_config(), 'gravity_i')
        gravity = type('G', (), {'torque': lambda self, q, payload=None: np.full(14, 50.)})()
        ff = TorqueFeedforward(config, gravity)
        q = np.zeros(14)
        out = ff.compute(q, q, np.zeros(14), .02)
        np.testing.assert_allclose(out, config['tau_rate_nm_s'] * .02)
        for _ in range(500):
            out = ff.compute(q, q, np.zeros(14), .02)
        np.testing.assert_allclose(out, np.tile(config['tau_limit_nm'], 2))
        ff.integral[:] = 0
        ff.compute(q, q + .5, np.zeros(14), .02)  # outside error window: no windup
        self.assertFalse(np.any(ff.integral))
        for _ in range(2000):
            ff.compute(q, q + .1, np.zeros(14), .02)
        np.testing.assert_allclose(ff.integral, np.tile(config['integral']['limit_nm'], 2))

    def test_request_zero_ramps_down(self):
        gravity = type('G', (), {'torque': lambda self, q, payload=None: np.full(14, 2.)})()
        ff = TorqueFeedforward(with_mode(load_config(), 'gravity'), gravity)
        for _ in range(100):
            ff.compute(np.zeros(14), np.zeros(14), np.zeros(14), .02)
        ff.request_zero()
        out = ff.compute(np.zeros(14), np.zeros(14), np.zeros(14), .02)
        np.testing.assert_allclose(out, 1.7)

    def test_simulated_sag_converges_with_integral(self):
        # Plant: static PD spring kp*(q_cmd-q) + tau = load; load is 3x the model.
        kp = np.tile([50., 50, 40, 40, 30, 30, 30], 2)
        load = np.tile([6., 6, 4, 4, 1.5, 1.5, 1.5], 2)
        gravity = type('G', (), {'torque': lambda self, q, payload=None: load / 3})()
        q_cmd = np.zeros(14)
        errors = {}
        for mode in ('off', 'gravity', 'gravity_i'):
            ff = TorqueFeedforward(with_mode(load_config(), mode), gravity)
            q = np.zeros(14)
            for _ in range(1500):
                tau = ff.compute(q, q_cmd, np.zeros(14), .02)
                q = q_cmd - (load - tau) / kp
            errors[mode] = np.max(np.abs(q_cmd - q))
        self.assertGreater(errors['off'], errors['gravity'])
        self.assertLess(errors['gravity_i'], 1e-3)


def _offline_runtime_config():
    """config/runtime.json with a placeholder DDS interface: these tests never open a connection."""
    import json, tempfile
    from r1d_chess.paths import ROOT as _ROOT
    config = json.loads((_ROOT/'config/runtime.json').read_text())
    config['network_interface'] = config.get('network_interface') or 'lo'
    handle = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False)
    json.dump(config, handle)
    handle.close()
    return handle.name


class QtauSerializationTests(unittest.TestCase):
    def test_qtau_roundtrip_and_validation(self):
        import msgspec
        from r1d_chess.robot.r1d_runtime import R1DRuntimeClient, R1D_JOINT_GROUPS, load_experiment
        load_experiment(_offline_runtime_config())
        from robot_runtime.runtime.ipc.cmd import RuntimeCmd
        client = R1DRuntimeClient.__new__(R1DRuntimeClient)
        client.include_base = False
        actions = []
        client.send_action = actions.append
        sizes = dict(left_arm=7, right_arm=7, waist=1, head=2, left_ee=1, right_ee=1)
        targets = {k: tuple(np.zeros(sizes[k])) for k in R1D_JOINT_GROUPS}
        qtau = {'left_arm': tuple(np.arange(7, dtype=np.float64)), 'right_arm': tuple(-np.arange(7.))}
        client.send_joint_targets(targets, qtau)
        decoded = msgspec.msgpack.decode(msgspec.msgpack.encode(RuntimeCmd(
            cmd_session_id='x', cmd_sequence=1, timestamp=0., action=actions[0])), type=RuntimeCmd)
        groups = {g.schema_key: g for g in decoded.action.joint_groups}
        self.assertEqual(groups['left_arm'].qtau, [float(v) for v in range(7)])
        self.assertIsNone(groups['head'].qtau)
        # Head and waist take feedforward too (hold_joints.json); the grippers do not.
        client.send_joint_targets(targets, {'head': (0.5, 0.), 'waist': (-0.2,)})
        self.assertEqual({g.schema_key: g.qtau for g in actions[-1].joint_groups}['head'], [0.5, 0.])
        with self.assertRaises(ValueError):
            client.send_joint_targets(targets, {'left_ee': (0.,)})
        with self.assertRaises(ValueError):
            client.send_joint_targets(targets, {'left_arm': (float('nan'),)*7})


if __name__ == '__main__':
    unittest.main()
