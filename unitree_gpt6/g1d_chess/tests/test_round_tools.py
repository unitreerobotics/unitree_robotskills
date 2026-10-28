import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

"""Offline regression checks; fake images/state, no DDS/ZMQ imports or requests."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import sys
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
from g1d_chess.chess import round_tools as m


class RoundToolsTests(unittest.TestCase):
    def setUp(self):
        from g1d_chess.robot import camera_policy
        fixture = {'schema_version': 1, 'head_camera': 'cam_left_high',
                   'planning_camera': 'cam_left_high',
                   'wrist_stream_by_physical_arm': {'left': 'cam_right_wrist', 'right': 'cam_left_wrist'},
                   'mapping_evidence': 'synthetic offline fixture, not site configuration'}
        camera_patch = patch.object(camera_policy, '_policy', return_value=fixture)
        camera_patch.start()
        self.addCleanup(camera_patch.stop)
        original_read = m.read
        synthetic_init = {'left_arm': {'xyz_rpy': [.12, .24, .05, 0., 0., 0.]},
                          'right_arm': {'xyz_rpy': [.18, -.24, .05, 0., 0., 0.]}}
        def fixture_read(path):
            if Path(path).resolve() == ROOT/'config/init_state.json':
                return copy.deepcopy(synthetic_init)
            return original_read(path)
        init_patch = patch.object(m, 'read', side_effect=fixture_read)
        init_patch.start()
        self.addCleanup(init_patch.stop)
        (ROOT/'runs').mkdir(exist_ok=True)
        self.tmp=tempfile.TemporaryDirectory(dir=ROOT/'runs',prefix='offline_test_')
        self.run=Path(self.tmp.name)/'round'
        m.initialize(self.run)
        self.s=m.read(self.run/'round_spec.json')
        self.s.update(staging_xy_m=[.22,-.21],source_square='i0',target_square='h0',move_legality_review='offline fixture',
                      xy_error_context='offline-test-only',
                      board_registration_review='offline fixture',pose_context_review='offline fixture',
                      scene_and_neighbors_review='offline fixture')
        self.s['source'].update(xy_m=[.1773,-.2321],evidence='offline fixture')
        self.s['target'].update(xy_m=[.1756,-.2022],evidence='offline fixture')
        self.p={k:v for k,v in zip(m.POSE_KEYS,[.18,-.23,.04,0.,0.,.15])}
        self.left={**self.p,'x':.11,'y':.27,'yaw':-.84}
        self.obs={'ok':True,'timestamp':'OFFLINE_TEST_ONLY','state':{
            'control_enabled':False,'fault':None,'right_arm':self.p,'left_arm':self.left,
            'waist_q':[-.16,0.,0.],'grippers':{'right':{'q':.016}}},'paths':{}}
        for c in ('cam_left_high',):
            f=self.run/'images'/(c+'.fixture');f.write_text('offline non-camera fixture')
            self.obs['paths'][c]=str(f)
        self.op=self.run/'tmp/observation.json';m.save(self.op,self.obs)

    def tearDown(self):self.tmp.cleanup()

    def build(self):
        sp=self.run/'spec_filled.json';m.save(sp,self.s)
        return m.build(self.run,sp,self.op),m.read(self.run/'tmp/request.json')

    def prepared(self, tx=None, token='fake_offline_token'):
        if tx is None:tx=m.read(self.run/'tmp/request.json')['transaction']
        directory=Path(tx['checkpoint_dir'])
        p={'ok':True,'token':token,'decision_channel':{
            'mode':'serial','require_worker':False,'worker_id':'offline_worker'},
           'confirm_after_descent':True,'preclose_high_recheck_supported':True,'gripper_open_q':2.3}
        p['pre_release_adjustment']=copy.deepcopy(tx['pre_release_adjustment'])
        p['camera_selection']=m.read(self.run/'tmp/plan_summary.json')['camera_selection']
        for key,suffix in (('checkpoint_path','_checkpoint.json'),
                           ('decision_path','_decision.json'),
                           ('receipt_path','_decision_receipt.json'),
                           ('checkpoint_discovery_path','_checkpoint_latest.json')):
            p[key]=str(directory/(token+suffix))
        return p

    def fill_capture(self):
        self.s['move_kind']='capture';self.s['capture']={'side':'黑方',
            'source':copy.deepcopy(self.s['target']),
            'offboard_target':{'kind':'round_tray','physical_side':'right',
                               'xy_m':[.20,-.33],'z_m':.159,'rpy_rad':[0.,.7854,1.4082],
                               'evidence':'offline fixture'},'offboard_surface_review':'offline fixture'}

    def test_normal_paths_and_continuous_tail(self):
        summary,req=self.build();t=req['transaction']
        self.assertEqual(summary['moves'],['红方 i0 → h0'])
        self.assertEqual([p['gripper'] for p in t['success_plan']['right_arm'][:3]],['hold','close','hold'])
        self.assertNotIn('auto_grasp_gate',t)
        self.assertEqual(t['pregrasp_adjustment']['success_source_steps'],[0,1,2])
        self.assertEqual(t['gripper_open_q'],2.3)
        self.assertEqual(t['pre_release_adjustment']['max_corrections'],3)
        self.assertEqual(t['pre_release_adjustment']['on_correction_limit'],'release_without_another_photo')
        self.assertEqual(summary['post_grasp_visual_gates'],1)
        for name in ('success_plan','failure_plan'):
            self.assertEqual(t[name]['right_arm'][-1]['gripper'],'close')
            self.assertEqual(t[name]['left_arm'][0]['y'],.27)
        self.assertEqual(t['grasp_retry']['unknown_failure_plan']['right_arm'][0]['gripper'],'open')
        # Releasing and closing each occupy an unchanged pose, not a combined move.
        plans=[t['grasp_plan'],t['success_plan'],t['failure_plan'],
               t['grasp_retry']['retry_plan'],t['grasp_retry']['unknown_failure_plan']]
        for plan in plans:
            for prev,cur in zip(plan['right_arm'],plan['right_arm'][1:]):
                if cur['gripper']!='hold':
                    self.assertEqual([prev[k] for k in m.POSE_KEYS],[cur[k] for k in m.POSE_KEYS])

    def test_xy_history_requires_estimation_review_without_changing_coordinates(self):
        reference = {'physical_actor': 'right', 'context_id': 'offline',
                     'phases': {'source': {'count': 1, 'mean_delta_xyz_mm': [8., -4., 6.]},
                                'target': {'count': 1, 'mean_delta_xyz_mm': [2., -1., 3.]}}}
        sp = self.run/'filled.json';m.save(sp,self.s)
        with patch.object(m, 'reference_for_spec', return_value=copy.deepcopy(reference)) as read_ref:
            with self.assertRaisesRegex(ValueError, 'xy_error_reference_review'):
                m.build(self.run,sp,self.op)
            self.s['xy_error_reference_review'] = '本轮图像支持原候选；仅参考历史误差，不直接叠加'
            sp.write_text(json.dumps(self.s))
            summary = m.build(self.run,sp,self.op)
            read_ref.assert_called_with(self.s, 'right', run=self.run)
        self.assertEqual([summary['source'][k] for k in ('x','y')], self.s['source']['xy_m'])
        self.assertEqual([summary['target'][k] for k in ('x','y')], self.s['target']['xy_m'])
        self.assertEqual(summary['source']['z'], self.s['source']['z_m'])
        self.assertEqual(summary['target']['z'], self.s['target']['z_m'])
        self.assertEqual(summary['xy_error_reference']['phases']['source']['mean_delta_xyz_mm'], [8., -4., 6.])

    def test_direct_transfer_and_release_rise(self):
        _,req=self.build()
        points=req['transaction']['success_plan']['right_arm']
        self.assertEqual([points[3][k] for k in ('x','y')],self.s['target']['xy_m'])
        release=next(i for i,p in enumerate(points) if p['gripper']=='open')
        up=points[release+1]
        self.assertGreater(up['z'],points[release]['z'])
        for key in ('x','y','roll','pitch','yaw'):
            self.assertEqual(up[key],points[release][key])

    def test_single_grasp_attempt_keeps_failure_return(self):
        self.s['max_grasp_attempts']=1
        _,req=self.build()
        retry=req['transaction']['grasp_retry']
        self.assertEqual(retry['max_attempts'],1)
        self.assertEqual(retry['failure_plan']['right_arm'][-1]['gripper'],'close')

    def test_old_service_without_release_gate_cannot_generate_execute(self):
        self.build()
        prepared=self.prepared()
        prepared.pop('pre_release_adjustment')
        path=self.run/'tmp/old_service_prepared.json'
        m.save(path,prepared)
        with self.assertRaisesRegex(ValueError,'下降后开爪前'):
            m.tokens(self.run,path)
        self.assertFalse((self.run/'tmp/execute_request.json').exists())

    def test_old_high_gate_service_cannot_generate_execute(self):
        self.build()
        prepared=self.prepared()
        prepared['pre_release_adjustment'].pop('review_pose')
        path=self.run/'tmp/high_gate_prepared.json'
        m.save(path,prepared)
        with self.assertRaisesRegex(ValueError,'下降后开爪前'):
            m.tokens(self.run,path)
        self.assertFalse((self.run/'tmp/execute_request.json').exists())

    def test_invalid_grasp_attempt_limit_rejected(self):
        for value in (0,4,True,1.5):
            self.s['max_grasp_attempts']=value
            sp=self.run/('invalid_'+str(value)+'.json');m.save(sp,self.s)
            with self.assertRaisesRegex(ValueError,'max_grasp_attempts'):
                m.build(self.run,sp,self.op)

    def test_old_large_offset_only_policy_cannot_generate_execute(self):
        self.build()
        prepared = self.prepared()
        prepared['pre_release_adjustment']['adjustment_policy'] = 'only_obvious_large_offset'
        path = self.run/'tmp/old_release_policy.json'
        m.save(path, prepared)
        with self.assertRaisesRegex(ValueError, '腕部落点交点判定'):
            m.tokens(self.run, path)
        self.assertFalse((self.run/'tmp/execute_request.json').exists())

    def test_capture_two_stages_no_init_handoff(self):
        self.fill_capture()
        summary,req=self.build();first,second=req['transactions']
        self.assertEqual(summary['moves'],['黑方 h0 → empty（黑方 h0 被吃）','红方 i0 → h0'])
        self.assertEqual(first['pre_release_adjustment']['alignment_criterion'],
                         'actor_wrist_predicted_landing_inside_same_side_tray')
        self.assertEqual(summary['capture_target']['physical_side'], 'right')
        opened = next(p for p in first['success_plan']['right_arm'] if p['gripper']=='open')
        self.assertEqual([opened[k] for k in ('x','y','z')], [.20,-.33,.159])
        self.assertEqual(second['pre_release_adjustment']['alignment_criterion'],
                         'actor_wrist_predicted_landing_near_target_intersection')
        self.assertEqual(first['success_plan']['right_arm'][-1]['z'],.205)
        self.assertEqual(first['success_plan']['right_arm'][-1]['gripper'],'hold')
        self.assertGreater(second['grasp_plan']['right_arm'][0]['z'],.205)
        self.assertTrue(all(p['gripper']=='hold' for p in second['grasp_plan']['right_arm']))
        self.assertEqual(second['grasp_plan']['right_arm'][-1]['z'],self.s['gate_z_m'])
        self.assertEqual(second['success_plan']['right_arm'][-1]['gripper'],'close')
        self.assertTrue(all('grasp_retry' in t and 'auto_grasp_gate' not in t for t in req['transactions']))

    def test_capture_rejects_wrong_side_or_non_tray_target(self):
        self.fill_capture()
        original = copy.deepcopy(self.s['capture']['offboard_target'])
        spec_path = self.run/'invalid_capture.json'
        for field, value in (('physical_side','left'), ('physical_side',None), ('kind','table')):
            with self.subTest(field=field, value=value):
                self.s['capture']['offboard_target'] = {**original, field:value}
                spec_path.write_text(json.dumps(self.s))
                with self.assertRaisesRegex(ValueError, '圆盘'):
                    m.build(self.run,spec_path,self.op)
                self.assertFalse((self.run/'tmp/request.json').exists())

    def test_capture_rejects_other_release_heights_on_both_hands(self):
        self.fill_capture()
        spec_path = self.run/'invalid_height.json'
        for arm in ('left', 'right'):
            self.obs['state']['active_arm'] = arm
            self.op.write_text(json.dumps(self.obs))
            self.s['capture']['offboard_target']['physical_side'] = arm
            for height in (.139, .16):
                with self.subTest(arm=arm, height=height):
                    self.s['capture']['offboard_target']['z_m'] = height
                    spec_path.write_text(json.dumps(self.s))
                    with self.assertRaisesRegex(ValueError, 'Z=0.159m'):
                        m.build(self.run,spec_path,self.op)
                    self.assertFalse((self.run/'tmp/request.json').exists())

    def test_left_capture_uses_physical_left_tray_without_mirroring_pose(self):

        from g1d_chess.chess import round_tools_left
        self.fill_capture()
        self.obs['state'].update(active_arm='left',arm_field_frame='transaction_roles')
        self.op.write_text(json.dumps(self.obs))
        self.s['capture']['offboard_target'].update(physical_side='left',xy_m=[.21,.34])
        spec_path = self.run/'left_capture.json';m.save(spec_path,self.s)
        summary = round_tools_left.build(self.run,spec_path,self.op)
        first,second = m.read(self.run/'tmp/request.json')['transactions']
        self.assertEqual(summary['capture_target']['physical_side'], 'left')
        self.assertIn('左手同侧圆盘', first['pre_release_adjustment']['target_description'])
        opened = next(p for p in first['success_plan']['right_arm'] if p['gripper']=='open')
        self.assertEqual([opened[k] for k in ('x','y','z')], [.21,.34,.159])
        self.assertTrue(all(p['gripper']=='hold' for p in second['grasp_plan']['right_arm']))

    def test_opposite_edge_moves_remain_plannable(self):
        spec_path = self.run/'edge_move.json'
        for source, target in (('g4','a4'), ('b6','i6')):
            with self.subTest(source=source,target=target):
                self.s.update(source_square=source,target_square=target)
                spec_path.write_text(json.dumps(self.s))
                summary=m.build(self.run,spec_path,self.op)
                self.assertTrue(summary['move_constraint']['allowed'])
                self.assertTrue((self.run/'tmp/request.json').is_file())
                (self.run/'tmp/request.json').unlink()
                (self.run/'tmp/plan_summary.json').unlink()

    def test_holding_state_rejected(self):
        self.obs['state']['grippers']['right']['q']=1.8
        self.op.write_text(json.dumps(self.obs))
        with self.assertRaisesRegex(ValueError,'空闭合'):self.build()

    def test_high_start_does_not_descend_in_entrance(self):
        self.obs['state']['right_arm']['z']=.30
        self.op.write_text(json.dumps(self.obs))
        _,req=self.build()
        first=req['transaction']['grasp_plan']['right_arm'][0]
        self.assertGreaterEqual(first['z'],.30)
        for key in ('x','y','roll','pitch','yaw'):
            self.assertEqual(first[key],self.obs['state']['right_arm'][key])

    def test_nonfinite_gripper_rejected(self):
        self.obs['state']['grippers']['right']['q']=float('nan')
        self.op.write_text(json.dumps(self.obs))
        with self.assertRaisesRegex(ValueError,'非有限'):self.build()

    def test_missing_target_evidence_rejected(self):
        self.s['target']['evidence']=None
        with self.assertRaisesRegex(ValueError,'evidence'):self.build()

    def test_nonfinite_pose_rejected(self):
        self.s['source']['xy_m']=[float('nan'),0.]
        fixture=self.run/'invalid_spec.json'
        fixture.write_text(json.dumps(self.s))
        with self.assertRaisesRegex(ValueError,'非有限'):
            m.build(self.run,fixture,self.op)

    def test_invalid_observation_rejected(self):
        self.obs['paths']['cam_left_high']='/nonexistent.fixture'
        self.op.write_text(json.dumps(self.obs))
        with self.assertRaisesRegex(ValueError,'cam_left_high'):self.build()

    def test_planning_rejects_extra_wrist_image(self):
        self.obs['paths']['cam_left_wrist']=self.obs['paths']['cam_left_high']
        self.op.write_text(json.dumps(self.obs))
        with self.assertRaisesRegex(ValueError,'只允许 cam_left_high'):self.build()

    def test_left_planner_accepts_head_only_and_binds_physical_left_wrist(self):

        from g1d_chess.chess import round_tools_left
        self.obs['state'].update(active_arm='left',arm_field_frame='transaction_roles')
        self.op.write_text(json.dumps(self.obs))
        spec_path=self.run/'filled_left_spec.json'
        m.save(spec_path,self.s)
        summary=round_tools_left.build(self.run,spec_path,self.op)
        self.assertEqual(summary['xy_error_reference']['physical_actor'], 'left')
        self.assertEqual(summary['camera_selection']['planning_cameras'],['cam_left_high'])
        self.assertEqual(summary['camera_selection']['execution_cameras'],['cam_left_high','cam_right_wrist'])
        self.assertEqual(m.read(self.run/'tmp/request.json')['expected_active_arm'],'left')

    def test_prepared_wrong_physical_wrist_rejected(self):
        self.build()
        prepared=self.prepared()
        prepared['camera_selection']=m.camera_selection('left')
        path=self.run/'tmp/wrong_camera_prepared.json'
        m.save(path,prepared)
        with self.assertRaisesRegex(ValueError,'预检相机策略不匹配'):
            m.tokens(self.run,path)
        self.assertFalse((self.run/'tmp/execute_request.json').exists())

    def test_token_scope_and_no_overwrite(self):
        self.build()
        p=self.prepared()
        pp=self.run/'tmp/prepared_fixture.json';m.save(pp,p)
        out=m.tokens(self.run,pp)
        self.assertTrue(Path(out['execute_request']).is_file())
        with self.assertRaisesRegex(ValueError,'不覆盖'):m.tokens(self.run,pp)

    def test_wrong_run_prepared_rejected(self):
        self.build()
        p=self.prepared();p['checkpoint_path']=str(ROOT/'runs/other/test.json')
        pp=self.run/'tmp/wrong.json';m.save(pp,p)
        with self.assertRaisesRegex(ValueError,'不属于此run'):m.tokens(self.run,pp)

    def test_changed_request_rejected(self):
        self.build();p=self.prepared()
        request=self.run/'tmp/request.json'
        request.write_text(request.read_text()+'\n')
        pp=self.run/'tmp/prepared_fixture.json';m.save(pp,p)
        with self.assertRaisesRegex(ValueError,'内容已改变'):m.tokens(self.run,pp)

    def test_serial_identity_required(self):
        self.build();p=self.prepared();del p['decision_channel']['worker_id']
        pp=self.run/'tmp/prepared_fixture.json';m.save(pp,p)
        with self.assertRaisesRegex(ValueError,'worker身份'):m.tokens(self.run,pp)

    def test_capture_token_stages_ordered(self):
        self.fill_capture();_,req=self.build()
        parts=[self.prepared(tx,f'fake_stage_{i}') for i,tx in enumerate(req['transactions'])]
        for part in parts:del part['ok']
        p={'ok':True,'sequence_token':'fake_sequence','transactions':parts}
        pp=self.run/'tmp/prepared_fixture.json';m.save(pp,p)
        out=m.tokens(self.run,pp)
        self.assertEqual(len(out['prepared_stages']),2)
        self.assertEqual(m.read(out['execute_request']),{'sequence_token':'fake_sequence'})

    def test_swapped_capture_stages_rejected(self):
        self.fill_capture();_,req=self.build()
        parts=[self.prepared(tx,f'fake_stage_{i}') for i,tx in enumerate(req['transactions'])]
        pp=self.run/'tmp/prepared_fixture.json'
        m.save(pp,{'ok':True,'sequence_token':'fake_sequence','transactions':parts[::-1]})
        with self.assertRaisesRegex(ValueError,'阶段/顺序不一致'):m.tokens(self.run,pp)

    def test_failed_preflight_rejected(self):
        self.build();pp=self.run/'tmp/failed.json';m.save(pp,{'ok':False,'error':'IK failure'})
        with self.assertRaisesRegex(ValueError,'预检未成功'):m.tokens(self.run,pp)

    def test_placed_not_visual_success(self):
        pp=self.run/'tmp/result.json';m.save(pp,{'outcome':'placed','control_enabled':False})
        self.assertIsNone(m.report(pp)['move_success'])


if __name__=='__main__':unittest.main()
