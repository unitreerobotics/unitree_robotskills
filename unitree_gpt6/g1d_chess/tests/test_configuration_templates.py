import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

"""Configuration isolation checks; no hardware, network, or service startup."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
from g1d_chess.robot import camera_policy as camera


class ConfigurationTemplatesTests(unittest.TestCase):
    def test_unconfigured_camera_mapping_rejected(self):
        data = json.loads((ROOT/'config/chess_camera_policy.json').read_text())
        self.assertEqual(data['wrist_stream_by_physical_arm'], {'left': None, 'right': None})
        with self.assertRaisesRegex(ValueError, 'configure config/chess_camera_policy.json'):
            camera.validate_policy(data)

    def test_both_real_mapping_arrangements_supported(self):
        for left, right in [('cam_left_wrist','cam_right_wrist'), ('cam_right_wrist','cam_left_wrist')]:
            data = {'schema_version':1, 'planning_camera':'cam_left_high', 'head_camera':'cam_left_high',
                    'wrist_stream_by_physical_arm':{'left':left,'right':right},
                    'mapping_evidence':'synthetic test'}
            camera.validate_policy(data)
            with patch.object(camera, '_policy', return_value=data):
                for hand, stream in [('left',left),('right',right)]:
                    selection = camera.camera_selection(hand)
                    self.assertEqual(selection['execution_cameras'], ['cam_left_high', stream])
                    self.assertEqual(camera.execution_cameras(hand,selection), ['cam_left_high', stream])

    def test_unconfigured_context_has_no_history_identity(self):
        data=json.loads((ROOT/'config/xy_error_learning.json').read_text())
        self.assertIsNone(data['context_id'])
        spec=importlib.util.spec_from_file_location('site_history',ROOT/'src/g1d_chess/chess/xy_error_history.py')
        history=importlib.util.module_from_spec(spec);spec.loader.exec_module(history)
        with self.assertRaisesRegex(ValueError,'identity must be nonempty'):
            history.reference_for_spec({},'left',run=ROOT/'runs/site-unconfigured')

    def test_template_fields_have_instructions_and_no_local_values(self):
        for name in ('environment.example.json','package_environment.json'):
            data=json.loads((ROOT/'config'/name).read_text())
            for field in ('package_root','python_executable','endpoint','interface','camera_host','gripper_open_sign'):
                if field == 'gripper_open_sign':
                    self.assertEqual(data[field], 1)
                else:
                    self.assertIsNone(data[field])
                self.assertIn('description',data['_field_help'][field])
                self.assertIn('example',data['_field_help'][field])
        for path in (ROOT/'config').glob('*.json'):
            text=path.read_text()
            for marker in ('/home/', '/Users/'):
                self.assertNotIn(marker,text,str(path))
        scene=json.loads((ROOT/'config/scene.example.json').read_text())
        self.assertTrue(all(value is None for value in scene['scene'].values()))

    def test_init_and_staging_require_site_measurements(self):
        data = json.loads((ROOT/'config/init_state.json').read_text())
        for hand in ('left', 'right'):
            self.assertIsNone(data[f'{hand}_arm']['q'])
            self.assertIsNone(data[f'{hand}_arm']['xyz_rpy'])
        for path in (ROOT/'assets/templates').glob('*.json'):
            data = json.loads(path.read_text())
            if 'staging_xy_m' in data:
                self.assertIsNone(data['staging_xy_m'])

    def test_service_requires_configuration_before_creating_connections(self):
        from g1d_chess.robot import internal_service as service
        with patch.dict(os.environ, {}, clear=True), \
             patch.object(sys, 'argv', ['internal_service.py']), \
             patch.object(service.zmq, 'Context') as context, \
             patch.object(service, 'RobotService') as robot:
            with self.assertRaises(SystemExit) as result:
                service.main()
            self.assertEqual(result.exception.code, 2)
            context.assert_not_called()
            robot.assert_not_called()

    def test_service_default_and_override_opening_direction_without_hardware(self):
        from g1d_chess.robot import internal_service as service
        cases = [({}, [], 1), ({'CHESS_GRIPPER_SIGN': ''}, [], 1),
                 ({'CHESS_GRIPPER_SIGN': '-1'}, [], -1),
                 ({'CHESS_GRIPPER_SIGN': '-1'}, ['--gripper-open-sign', '1'], 1)]
        for environment, arguments, expected in cases:
            with self.subTest(environment=environment, arguments=arguments), \
                 patch.dict(os.environ, environment, clear=True), \
                 patch.object(sys, 'argv', ['internal_service.py', '--interface', 'offline',
                                           '--camera-host', 'offline', *arguments]), \
                 patch.object(service.zmq, 'Context'), \
                 patch.object(service, 'RobotService', side_effect=RuntimeError('offline stop')) as robot:
                with self.assertRaisesRegex(RuntimeError, 'offline stop'):
                    service.main()
                self.assertEqual(robot.call_args.args[3], expected)
