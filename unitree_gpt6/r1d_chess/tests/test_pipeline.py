"""R1-D integration checks using real URDF/IK and explicitly simulated transport."""
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import numpy as np
import pyffish

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from r1d_chess.robot.r1d_config import load_profile, require_calibration, geometry_source
from r1d_chess.robot.r1d_kinematics import R1DKinematics
from r1d_chess.robot.motion_harness import waypoint
from r1d_chess.robot.serve import R1DService, profile_digest
from r1d_chess.chess.xiangqi import validate_move, uci_move, forecast_entries, match_forecast
from r1d_chess.chess import round_tools
from r1d_chess.robot.checkpoint_runner import decision_from_evaluation, atomic_json
from r1d_chess.robot.serial_checkpoint import failure_from_checkpoint
from r1d_chess.robot.r1d_transport import InternalDDS
from r1d_chess.robot.r1d_torque_ff import TorqueFeedforward, with_mode, load_config as load_torque_ff
from types import SimpleNamespace


def synthetic_profile():
    p = load_profile(ROOT/'config/robot.json')
    p['tcp_xyz_rpy']={'left':[0.]*6, 'right':[0.]*6}
    poses = R1DKinematics(p).fk(np.zeros(14))
    p.update(calibration_verified=True, calibration_evidence='OFFLINE SYNTHETIC TEST ONLY',
             simulation_only=True, camera_mapping_verified=True, lift_height_m=.44,
             head_q_rad=[0.,0.], waist_q_rad=[0.],
             init_xyz_rpy={s:[waypoint(T)[k] for k in round_tools.POSE_KEYS]
                           for s,T in zip(['left','right'],poses)},
             board_release_z_m=float(poses[1][2,3]+.005),
             tray_release_z_m=float(poses[1][2,3]+.005),
             scene={'surface_z':-.4,'uncertainty':.001,'finger_end':.10,
                    'half_width':.025,'half_thickness':.01,'travel_clearance':.04,
                    'minimum_clearance':.003,'xy_tolerance':.001,'rotation_tolerance':.008})
    return p


class PipelineTests(unittest.TestCase):
    def setUp(self):
        (ROOT/'runs').mkdir(exist_ok=True)      # created on demand; the package ships without runs/
        self.directory = tempfile.TemporaryDirectory(dir=ROOT/'runs', prefix='offline_test_')
        self.run = Path(self.directory.name)/'round'
        self.profile = synthetic_profile()
        self.profile_file = Path(self.directory.name)/'profile.json'
        self.profile_file.write_text(json.dumps(self.profile))
        self.environment = patch.dict(os.environ, {'R1D_CHESS_PROFILE':str(self.profile_file)})
        self.environment.start()
        # Synthetic world: the absolute 0.15 m board release (real robot) does not apply; use the
        # synthetic profile's own release heights. The real rule is covered by test_grip_pose.
        from r1d_chess.chess import grip_pose
        synthetic=lambda prof,pitch,kind='board': prof['tray_release_z_m' if kind=='tray' else 'board_release_z_m']
        self.release_patches=[patch.object(grip_pose,'release_tcp_z',synthetic),
                              patch.object(round_tools,'release_tcp_z',synthetic)]
        for item in self.release_patches:
            item.start()
        self.service = R1DService('unused', 'unused', Path(self.directory.name)/'logs', simulate=True)

    def tearDown(self):
        for item in self.release_patches:
            item.stop()
        self.service.dds.close()
        self.service.event_log.close()
        self.environment.stop()
        self.directory.cleanup()

    def build(self, arm='right', capture=False):
        self.service.set_actor(arm)
        round_tools.initialize(self.run)
        observation = {'ok':True, **self.service.capture(output_dir=self.run/'images/initial')}
        op = self.run/'tmp/observation.json'
        op.write_text(json.dumps(observation))
        state = observation['state']
        position = state['right_arm']
        board = {'game_id':'offline-only', 'confirmed':True,
                 'evidence':[observation['paths']['cam_left_high']],
                 'initial_fen':pyffish.start_fen('xiangqi'),'moves':[]}
        if capture:
            board['initial_fen'] = '4k4/9/9/9/4p4/9/9/4R4/9/4K4 w - - 0 1'
        bp = self.run/'tmp/board.json'
        bp.write_text(json.dumps(board))
        spec = round_tools.read(self.run/'round_spec.json')
        spec.update(game_id='offline-only', round_number=1, move_plan='兵七进一',
                    board_state_file=str(bp), source_square='g3',target_square='g4',
                    xy_error_context='r1d_'+profile_digest(self.profile)[:16], scene_and_neighbors_review='synthetic',
                    board_registration_review='synthetic', pose_context_review='synthetic',
                    move_legality_review='pyffish synthetic fixture',
                    gate_z_m=position['z']+.01,travel_z_m=position['z']+.02,
                    staging_xy_m=[position['x'],position['y']],
                    ignore_source_tool_envelope=False)
        rpy=[position[k] for k in ('roll','pitch','yaw')]
        spec['source'].update(xy_m=[position['x']-.003,position['y']],
                              z_m=position['z']+.005,rpy_rad=rpy,evidence='synthetic source')
        from r1d_chess.chess import grip_pose
        spec['target'].update(xy_m=[position['x']-.005,position['y']],
                              z_m=grip_pose.release_tcp_z(self.profile,rpy[1],'board'),rpy_rad=rpy,
                              evidence='synthetic target')
        if capture:
            spec.update(source_square='e2',target_square='e5',move_kind='capture',move_plan='车五进三')
            spec['capture']={'side':'黑方','source':copy.deepcopy(spec['source']),
                             'offboard_surface_review':'synthetic only',
                             'offboard_target':{**copy.deepcopy(spec['target']),'kind':'round_tray',
                                                'physical_side':arm,
                                                'z_m':grip_pose.release_tcp_z(self.profile,rpy[1],'tray')}}
        sp=self.run/'round_spec.json'
        sp.write_text(json.dumps(spec))
        # Synthetic board: the named squares sit under the synthetic grip points
        # (spec poses are TCP; round build checks the 131 mm grip point).
        from r1d_chess.chess import grip_pose
        grips={sq:grip_pose.grip_point(spec[k]['xy_m']+[spec[k]['z_m']],spec[k]['rpy_rad'])[:2]
               for k,sq in (('source',spec['source_square']),('target',spec['target_square']))}
        trays={}
        if capture:
            drop=spec['capture']['offboard_target']
            trays[arm]=grip_pose.grip_point(drop['xy_m']+[drop['z_m']],drop['rpy_rad'])[:2]
        bg=Path(self.directory.name)/'board_geometry.json'
        bg.write_text(json.dumps({'intersections_xy_m':grips,'trays_xy_m':trays}))
        with patch.dict(os.environ,{'R1D_CHESS_BOARD':str(bg)}):
            summary=round_tools.build(self.run,sp,op)
        return summary,round_tools.read(self.run/'tmp/request.json')

    def test_uncalibrated_and_simulation_profiles_cannot_enable_hardware(self):
        for key in ('scene','board_release_z_m','tray_release_z_m'):
            incomplete=load_profile(ROOT/'config/robot.json')
            incomplete[key]=None
            with self.assertRaises(ValueError):
                require_calibration(incomplete)
        with self.assertRaises(ValueError):
            require_calibration(self.profile,hardware=True)

    def test_documented_model_estimates_can_pass_geometry_gate(self):
        estimate=copy.deepcopy(self.profile)
        estimate.update(simulation_only=False, calibration_verified=False,
                        calibration_evidence=None, camera_mapping_verified=False,
                        geometry_source='model_estimate',
                        estimate_evidence='Synthetic geometry and camera mapping for gate test')
        self.assertIs(require_calibration(estimate,hardware=True),estimate)
        self.assertEqual(geometry_source(estimate),'model_estimate')
        estimate['tcp_xyz_rpy']['left']=None
        with self.assertRaisesRegex(ValueError,'wrist-to-TCP'):
            require_calibration(estimate,hardware=True)
        estimate['tcp_xyz_rpy']['left']=[0.]*6
        estimate['estimate_evidence']=''
        with self.assertRaisesRegex(ValueError,'estimate_evidence'):
            require_calibration(estimate,hardware=True)

    def test_urdf_both_arms_reachable_and_unreachable(self):
        model=R1DKinematics(self.profile)
        q=np.array([.3,.2,.2,.2,.1,.1,.1,.3,-.2,-.2,.2,-.1,.1,-.1])
        targets=model.fk(q)
        solution=model.solve(targets,q+.002)
        self.assertLess(max(np.linalg.norm(a[:3,3]-b[:3,3]) for a,b in zip(targets,model.fk(solution))),1e-4)
        targets[0][0,3]+=5
        with self.assertRaises(ValueError):model.solve(targets,q)

    def test_left_role_uses_physical_camera_and_joint_permutation(self):
        self.service.set_actor('left')
        physical=self.service.physical_motion.robot
        q=np.array([.3,.2,.2,.2,.1,.1,.1,.3,-.2,-.2,.2,-.1,.1,-.1])
        actual=self.service.motion.fk(np.r_[q[7:],q[:7]])
        np.testing.assert_allclose(actual[1],physical.fk(q)[0])
        state=self.service.state()
        self.assertEqual(state['camera_selection']['execution_cameras'],['cam_left_high','cam_left_wrist'])
        self.assertEqual(state['grippers']['right']['motor_index'],31)

    def test_init_threshold_includes_exact_ten_cm_without_movement(self):
        self.service.profile['init_xyz_rpy']['right'][0]+=.10
        result=self.service.prepare_init(str(self.run/'tmp/init'))
        self.assertFalse(result['init_required'])
        self.assertAlmostEqual(result['distances_m']['right'],.10)
        self.assertFalse(self.service.physical_dds.enabled)

    def test_complete_normal_preflight_tokens_and_replay_rejection(self):
        summary,request=self.build()
        prepared={'ok':True,**self.service.dispatch({'cmd':'prepare_transaction',**request})}
        pp=self.run/'tmp/prepared.json';pp.write_text(json.dumps(prepared))
        generated=round_tools.tokens(self.run,pp)
        execute=round_tools.read(generated['execute_request'])
        self.assertEqual(execute['expected_active_arm'],'right')
        self.assertTrue(prepared['preclose_high_recheck_supported'])
        self.assertTrue(prepared['pre_release_adjustment'])
        with self.assertRaises(ValueError):round_tools.tokens(self.run,pp)

    def test_capture_both_hands_full_sequence_preflight(self):
        for arm in ('left','right'):
            with self.subTest(arm=arm):
                self.run=Path(self.directory.name)/('capture_'+arm)
                summary,request=self.build(arm,capture=True)
                prepared=self.service.dispatch({'cmd':'prepare_sequence',**request})
                self.assertEqual(len(prepared['transactions']),2)
                self.assertTrue(prepared['sequence_token'])

    def run_with_decisions(self, prepared, choose):
        actor=self.service.active_arm
        self.service.dispatch({'cmd':'enable_control','expected_active_arm':actor})
        done=threading.Event();seen=[];errors=[]
        def reviewer():
            while not done.is_set():
                try:
                    for path in (self.run/'tmp/checkpoints').rglob('*_checkpoint.json'):
                        c=json.loads(path.read_text())
                        if Path(c['decision_path']).exists():continue
                        # Serial gates (source high, preclose, pre_release) never time out.
                        if c.get('wait_indefinitely') is not True or c.get('deadline_unix_s') is not None:
                            raise AssertionError(f"gate {c.get('stage')} has a deadline")
                        action,offset=choose(c,seen)
                        if action=='failure':
                            decision=failure_from_checkpoint(c)
                        else:
                            evaluation={'decision':action,'actual_relative_offset_xy_m':offset,
                                        'reason':'SIMULATION ONLY',
                                        'evidence':[c['observation']['paths'][k] for k in c['camera_order']]}
                            if c.get('stage')=='pre_release':
                                evaluation.update(target_identified=True,release_alignment_verified=action=='success',
                                                  obvious_large_offset=action=='adjust')
                            else:
                                evaluation.update(source_identified=True,descent_alignment_verified=action=='success')
                            decision=decision_from_evaluation(c,evaluation)
                        atomic_json(c['decision_path'],decision,exclusive=True)
                        seen.append((c.get('stage','high'),action))
                except Exception as exc:
                    errors.append(str(exc));self.service.cancelled.set();return
                time.sleep(.01)
        worker=threading.Thread(target=reviewer,daemon=True);worker.start()
        try:
            execution=({'cmd':'execute_sequence','sequence_token':prepared['sequence_token']}
                       if 'sequence_token' in prepared else
                       {'cmd':'execute_transaction','token':prepared['token']})
            result=self.service.dispatch({**execution,'expected_active_arm':actor})
        finally:
            done.set();worker.join(2)
        self.assertFalse(errors,errors)
        return result,seen

    def test_left_capture_executes_two_stages_with_one_enable(self):
        _,request=self.build('left',capture=True)
        prepared=self.service.dispatch({'cmd':'prepare_sequence',**request})
        result,seen=self.run_with_decisions(prepared,lambda c,s:('success',[0,0]))
        self.assertEqual(result['outcome'],'placed',result)
        self.assertTrue(result['arm_unload']['unloaded'], result.get('arm_unload'))
        self.assertFalse(self.service.physical_dds.session_active)
        self.assertEqual(len(result['transactions']),2)
        self.assertEqual([p['outcome'] for p in result['transactions']],['placed','placed'])
        self.assertEqual([s for s,a in seen].count('pre_release'),2)
        self.assertTrue(all(p['simulated'] for p in result['transactions']))
        self.assertEqual(self.service.active_arm,'left')

    def test_low_gate_pitch_reduction_then_close(self):
        """piece beyond the fingertips -> reduce pitch (TCP xyz/yaw/roll kept), re-photo, close."""
        _,request=self.build()
        prepared=self.service.dispatch({'cmd':'prepare_transaction',**request})
        lows=[]
        def choose(c,seen):
            if c.get('stage')=='preclose':
                lows.append(c)
                if len(lows)==1:
                    self.assertEqual(c['low_pitch_adjustments_remaining'],2)
                    self.assertEqual(c['low_z_adjustments_remaining'],0)
                    # adjust_z is disabled by default now
                    with self.assertRaisesRegex(ValueError,'adjust_z is disabled'):
                        decision_from_evaluation(c,{'decision':'adjust_z','actual_relative_offset_xy_m':[0,0],
                            'reason':'x','source_identified':True,'descent_alignment_verified':False,'evidence':['x']})
                    return 'adjust_pitch',[0,0]
            return 'success',[0,0]
        result,seen=self.run_with_decisions(prepared,choose)
        self.assertEqual(result['outcome'],'placed',result.get('error'))
        # Release first, then move back: the opening step records the post-open dwell.
        dwell=[json.loads(l) for l in self.service.log_path.read_text().splitlines() if 'release_post_open_dwell' in l]
        self.assertEqual(len(dwell),1)
        self.assertEqual(dwell[0]['dwell_s'],.5)
        # The release opens to the per-move release opening chosen by round build.
        wanted=request['transaction']['pre_release_adjustment']['release_open_q']
        self.assertAlmostEqual(dwell[0]['gripper']['target_q'],wanted,places=6)

        self.assertIn(('preclose','adjust_pitch'),seen)
        self.assertEqual(len(lows),2)
        self.assertEqual(lows[1]['low_pitch_adjustments_remaining'],1)
        events=[json.loads(l) for l in self.service.log_path.read_text().splitlines() if 'low_grasp_pitch_step' in l]
        self.assertEqual(len(events),1)
        e=events[0]
        self.assertAlmostEqual(e['target']['pitch'],e['pitch_before']-.1,places=9)
        a,b=lows[0]['actual_right_arm'],lows[1]['actual_right_arm']
        for k in ('x','y','z','yaw'):
            self.assertAlmostEqual(a[k],b[k],places=3,msg=k)      # simulated transport tracks exactly
        self.assertAlmostEqual(b['pitch'],a['pitch']-.1,places=3)

    def test_release_open_rule(self):
        import math
        occ={'d8':'a','c5':'N'}
        # fingers along -y -> jaws open along ranks: d8 occupied -> normal opening
        q,n,b=round_tools.release_open_q('d7',-math.pi/2,occ,'c5',2.3)
        self.assertEqual((q,sorted(n),b),(2.3,['d6','d8'],['d8']))
        q,_,_=round_tools.release_open_q('d7',-math.pi/2,{'c7':'n'},'c5',2.3)
        self.assertEqual(q,round_tools.RELEASE_OPEN_WIDE_Q)          # c7 is not on the opening axis
        q,_,b=round_tools.release_open_q('c6',-math.pi/2,{'c5':'N'},'c5',2.3)
        self.assertEqual((q,b),(round_tools.RELEASE_OPEN_WIDE_Q,[]))  # the moving piece's own square is free
        q,_,_=round_tools.release_open_q('d7',0.,{'d8':'a'},'c5',2.3)
        self.assertEqual(q,round_tools.RELEASE_OPEN_WIDE_Q)          # fingers along x open along files

    def test_retry_high_and_target_low_xy_correction(self):
        _,request=self.build()
        prepared=self.service.dispatch({'cmd':'prepare_transaction',**request})
        def choose(c,seen):
            if c.get('stage')=='preclose' and ('preclose','retry_high') not in seen:
                return 'retry_high',[0,0]
            if c.get('stage')=='pre_release' and ('pre_release','adjust') not in seen:
                return 'adjust',[.0005,0]
            return 'success',[0,0]
        result,seen=self.run_with_decisions(prepared,choose)
        self.assertEqual(result['outcome'],'placed',result.get('error'))
        self.assertIn(('preclose','retry_high'),seen)
        self.assertIn(('pre_release','adjust'),seen)

    def test_low_failure_returns_without_regrasp_or_release(self):
        _,request=self.build()
        prepared=self.service.dispatch({'cmd':'prepare_transaction',**request})
        result,seen=self.run_with_decisions(prepared,lambda c,s:('failure' if c.get('stage')=='preclose' else 'success',[0,0]))
        self.assertIn('returned',result['outcome'],result.get('error'))
        self.assertNotIn('pre_release',[stage for stage,_ in seen])
        self.assertEqual(len([s for s in seen if s[0]=='preclose']),1)

    def test_actual_serial_gate_execution_and_snapshot_fk(self):
        _,request=self.build()
        prepared=self.service.dispatch({'cmd':'prepare_transaction',**request})
        self.service.dispatch({'cmd':'enable_control','expected_active_arm':'right'})
        done=threading.Event();errors=[];seen=[]
        def reviewer():
            while not done.is_set():
                for path in (self.run/'tmp/checkpoints').rglob('*_checkpoint.json'):
                    c=json.loads(path.read_text())
                    if Path(c['decision_path']).exists():continue
                    evidence=[c['observation']['paths'][k] for k in c['camera_order']]
                    evaluation={'decision':'success','actual_relative_offset_xy_m':[0,0],
                                'reason':'SIMULATION ONLY; no visual inference','evidence':evidence}
                    if c.get('stage')=='pre_release':
                        evaluation.update(target_identified=True,release_alignment_verified=True,obvious_large_offset=False)
                    else:
                        evaluation.update(source_identified=True,descent_alignment_verified=True)
                    try:
                        decision=decision_from_evaluation(c,evaluation)
                        atomic_json(c['decision_path'],decision,exclusive=True)
                        seen.append(c.get('stage','high'))
                    except Exception as exc:
                        errors.append(str(exc));done.set();return
                time.sleep(.01)
        thread=threading.Thread(target=reviewer,daemon=True);thread.start()
        try:
            result=self.service.dispatch({'cmd':'execute_transaction','token':prepared['token'],'expected_active_arm':'right'})
        finally:
            done.set();thread.join(2)
        self.assertFalse(errors,errors)
        self.assertEqual(result['outcome'],'placed',result.get('error'))
        self.assertIn('preclose',seen)
        self.assertIn('pre_release',seen)
        self.assertEqual({r['moment'] for r in result['pose_audit']['records']},{'grasp_success','release_start'})
        from r1d_chess.chess.review_pose_audit import review
        report=review(result,self.service.physical_motion)
        report.update(robot='R1-D',simulated=True)
        self.assertEqual(report['missing_events'],[])
        from r1d_chess.chess.xy_error_history import finish
        rp=self.run/'docs/pose_review.json';rp.write_text(json.dumps(report))
        vp=self.run/'tmp/visual_review.json';vp.write_text(json.dumps({'move_success':True,'note':'SIMULATED'}))
        finished=finish(None,self.run,'offline-only',1,'兵七进一','r1d_'+profile_digest(self.profile)[:16],
                        [rp],vp,self.run/'tmp/plan_summary.json')
        self.assertTrue(Path(finished['review']).is_file())
        repeated=finish(None,self.run,'offline-only',1,'兵七进一','r1d_'+profile_digest(self.profile)[:16],
                        [rp],vp,self.run/'tmp/plan_summary.json')
        self.assertEqual(finished['review'],repeated['review'])
        self.assertEqual(len(Path(finished['journal']).read_text().splitlines()),1)
        with self.assertRaises(ValueError):
            self.service.dispatch({'cmd':'execute_transaction','token':prepared['token'],'expected_active_arm':'right'})


class ChessTests(unittest.TestCase):
    def test_final_release_correction_requires_approval_in_client_and_service(self):
        from r1d_chess.robot.release_alignment import validate_release_decision
        checkpoint = dict(token='test', worker_id='test', checkpoint_id='fresh',
                          stage='pre_release', correction_index=2,
                          grasp_attempt_index=0, corrections_remaining=1)
        evaluation = dict(decision='adjust', actual_relative_offset_xy_m=[.001, 0],
                          reason='SIMULATION: predicted corrected landing still uncertain',
                          target_identified=True, release_alignment_verified=False,
                          obvious_large_offset=True, evidence=['current-head', 'current-wrist'])
        with self.assertRaisesRegex(ValueError, 'Final correction'):
            decision_from_evaluation(checkpoint, evaluation)
        # A client bypass must not defeat the service-side check.
        raw = {**checkpoint, **evaluation, 'offset_xy_m':[.001, 0]}
        with self.assertRaisesRegex(ValueError, 'final release correction'):
            validate_release_decision(checkpoint, raw)
        evaluation.update(release_alignment_verified=True,
                          reason='SIMULATION: corrected landing explicitly approved from current images')
        decision = decision_from_evaluation(checkpoint, evaluation)
        self.assertEqual(validate_release_decision(checkpoint, decision), 'adjust')

    def test_board_survey_rejects_degenerate_points_and_fits_all_90(self):
        from r1d_chess.board.board_registration import fit
        data={'frame':'waist_yaw_link','evidence':'synthetic','spacing_m':[.0356,.0337],
              'points':[{'square':s,'xyz_m':xyz} for s,xyz in
                        [('a0',[.2,-.15,-.1]),('i0',[.2,.1348,-.1]),
                         ('a9',[.5033,-.15,-.1]),('i9',[.5033,.1348,-.1])]]}
        result=fit(data)
        self.assertEqual(len(result['points']),90)
        self.assertLess(result['fit_max_m'],1e-8)
        data['points']=[{'square':s,'xyz_m':[.2,0.,-.1]} for s in ['a0','b0','c0','d0']]
        with self.assertRaises(ValueError):fit(data)

    def test_uci_rank10_and_invalid_moves(self):
        self.assertEqual(uci_move('b6','c8'),'b7c9')
        self.assertEqual(uci_move('a9','a8'),'a10a9')
        board={'confirmed':True,'game_id':'test','evidence':['synthetic'],'initial_fen':pyffish.start_fen('xiangqi'),'moves':[]}
        validate_move(board,'g4g5')
        with self.assertRaises(ValueError):validate_move(board,'g4g10')
        entries=forecast_entries(board['initial_fen'],[],['g4g5','a7a6'])
        forecast={'game_id':'test','initial_fen':board['initial_fen'],'confirmed_history':[],'plies':entries}
        self.assertTrue(match_forecast(board,forecast)['matched'])
        changed={**board,'moves':['c4c5']}
        self.assertFalse(match_forecast(changed,forecast)['matched'])


class TransportTests(unittest.TestCase):
    def test_single_runtime_frame_keeps_head_waist_and_bounded_arm_commands(self):
        adapter=InternalDDS.__new__(InternalDDS)
        adapter.profile=synthetic_profile()
        adapter.lock=threading.RLock()
        adapter.enabled=True
        adapter.session_started_at=time.monotonic()
        adapter.arm_sent=np.zeros(14)
        adapter.arm_target=np.ones(14)*.01
        adapter.gripper_sent=np.zeros(2)
        adapter.gripper_target=np.ones(2)*2.3
        adapter.published=0
        adapter.head_waist_sent=None
        adapter.torque_ff=TorqueFeedforward(with_mode(load_torque_ff(),'off'))
        from r1d_chess.robot.r1d_hold_ff import HoldFeedforward
        adapter.hold_ff=HoldFeedforward({})
        torques=[]
        motors=[SimpleNamespace(motorstate=0,temperature=[30,30]) for _ in range(35)]
        q=np.zeros(35)
        adapter.state=lambda:(q,np.zeros(35),SimpleNamespace(motor_state=motors),0.)
        base=SimpleNamespace(lift_height=.44,base_linear_vel_x=0.,base_linear_vel_y=0.,
                             base_angular_vel_z=0.,lift_linear_vel_z=0.)
        state=SimpleNamespace(control_active=True,runtime_error=None,thermal_protection_active=False,
                              observation=SimpleNamespace(base_group=[base]))
        sent=[]
        client=SimpleNamespace(state_receiver=SimpleNamespace(latest_state=lambda **kw:state),
                               send_joint_targets=lambda targets,qtau=None:(sent.append(targets),torques.append(qtau)))
        adapter._publish_once(client,.02)
        # Without fitted waist/head parameters no torque is added to those groups.
        self.assertEqual(torques[0]['head'],(0.,0.))
        self.assertEqual(torques[0]['waist'],(0.,))
        self.assertEqual(set(sent[0]),{'left_arm','right_arm','left_ee','right_ee','head','waist'})
        self.assertEqual(sent[0]['head'],(0.,0.))
        self.assertEqual(sent[0]['waist'],(0.,))
        self.assertAlmostEqual(sent[0]['left_arm'][0],.006)
        # A nonzero saved reference must be held, never silently replaced by zero.
        adapter.profile.update(head_q_rad=[.3258,-.0036],waist_q_rad=[.0299])
        # The saved reference is approached at a bounded rate, never stepped to or replaced by zero;
        # feedback is checked against the ramped command, not the final reference.
        adapter._publish_once(client,.02)
        self.assertAlmostEqual(sent[-1]['head'][0],.004)
        self.assertAlmostEqual(sent[-1]['head'][1],-.0036)
        self.assertAlmostEqual(sent[-1]['waist'][0],.004)
        for _ in range(100):
            q[[29,30,13]]=adapter.head_waist_sent
            adapter._publish_once(client,.02)
        q[[29,30,13]]=[.3258,-.0036,.0299]
        self.assertEqual(sent[-1]['head'],(.3258,-.0036))
        self.assertEqual(sent[-1]['waist'],(.0299,))
        del sent[2:]
        # Head/waist offsets, lift motion and base motion are entry conditions only.
        for slot in (29,30,13):
            q[slot]+=.11
            with self.assertRaisesRegex(RuntimeError,'saved chess initial reference'):
                adapter._check_reference(state,q)
            adapter._publish_once(client,.02)
            q[slot]-=.11
        base.base_angular_vel_z=.02           # below the 0.025 motion threshold: stationary at entry
        adapter._check_reference(state,q)
        base.base_angular_vel_z=0.
        base.lift_linear_vel_z=.1
        with self.assertRaises(RuntimeError):adapter._check_reference(state,q)
        adapter._publish_once(client,.02)     # a running session keeps going
        base.lift_linear_vel_z=0.
        state.runtime_error='fault'
        with self.assertRaisesRegex(RuntimeError,'Runtime error'):adapter._publish_once(client,.02)
        state.runtime_error=None

    def test_entry_allows_head_ramp_but_not_waist_offset(self):
        adapter=InternalDDS.__new__(InternalDDS)
        adapter.profile=synthetic_profile()
        adapter.profile.update(head_q_rad=[.62,-.0036],waist_q_rad=[.03])
        base=SimpleNamespace(lift_height=.44,base_linear_vel_x=0.,base_linear_vel_y=0.,
                             base_angular_vel_z=0.,lift_linear_vel_z=0.)
        state=SimpleNamespace(runtime_error=None,thermal_protection_active=False,
                              observation=SimpleNamespace(base_group=[base]))
        q=np.zeros(35);q[[29,30,13]]=[.34,0.,.03]
        adapter._check_reference(state,q,entry=True)
        with self.assertRaisesRegex(RuntimeError,'initial reference'):
            adapter._check_reference(state,q)
        q[29]=.20
        with self.assertRaisesRegex(RuntimeError,'initial reference'):
            adapter._check_reference(state,q,entry=True)
        q[29]=.34;q[13]=.15
        with self.assertRaisesRegex(RuntimeError,'initial reference'):
            adapter._check_reference(state,q,entry=True)

    def test_invalid_reference_rejected(self):
        from r1d_chess.robot.r1d_config import head_waist_reference
        for head,waist in (([float('nan'),0.],[0.]),([0.],[0.]),([.7,0.],[0.]),([0.,0.],[3.])):
            with self.assertRaises(ValueError):
                head_waist_reference({'head_q_rad':head,'waist_q_rad':waist})
        head,waist=head_waist_reference({})
        np.testing.assert_array_equal(head,[0.,0.])
        np.testing.assert_array_equal(waist,[0.])

    def test_stale_lowstate_rejected_without_command_publication(self):
        adapter=InternalDDS.__new__(InternalDDS)
        adapter.lock=threading.RLock();adapter.profile=synthetic_profile()
        adapter.msg=object();adapter.received_at=time.monotonic()-1
        with self.assertRaises(RuntimeError):adapter.state()


if __name__=='__main__':
    unittest.main()
