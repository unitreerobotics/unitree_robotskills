#!/usr/bin/env python3
"""Offline round setup, complete branch planning, token-file extraction and reporting.

This script never opens a robot or camera connection. Runtime preflight and
execution use the bundled robot_client.py explicitly as documented in README.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys

from r1d_chess.paths import ROOT
from r1d_chess.chess.xy_error_history import reference_for_spec
from r1d_chess.chess.move_constraints import review_move
from r1d_chess.robot.r1d_config import load_profile, require_calibration, geometry_source
from r1d_chess.chess.xiangqi import validate_move, uci_move, pieces
from r1d_chess.chess.grip_pose import check_spec as check_grip_points, load_board, release_tcp_z
from r1d_chess.robot.camera_policy import planning_cameras, camera_selection
POSE_KEYS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


def read(path):
    return json.loads(Path(path).read_text())


def save(path, value):
    path = Path(path)
    with path.open('x') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')


def vector(value, count, name):
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f'{name}: 必须提供 {count} 个数字')
    if any(type(v) not in (float, int) or not math.isfinite(v) for v in value):
        raise ValueError(f'{name}: 不接受空值或非有限数值')
    return list(map(float, value))


def scalar(value, name):
    return vector([value], 1, name)[0]


def square(value):
    if not isinstance(value, str) or len(value) != 2 or value[0] not in 'abcdefghi' or value[1] not in '0123456789':
        raise ValueError('交点必须为机器人视角 a0–i9')
    return value


def evidence(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'必须填写 {name}，记录本轮依据')
    return value


def pose(value, name):
    xy = vector(value['xy_m'], 2, name+'.xy_m')
    z = scalar(value['z_m'], name+'.z_m')
    rpy = vector(value['rpy_rad'], 3, name+'.rpy_rad')
    evidence(value.get('evidence'), name+'.evidence')
    return dict(zip(POSE_KEYS, xy + [z] + rpy), gripper='hold')


def wp(value, **changes):
    return {**value, **changes}


def within_run(run):
    run = Path(run).resolve()
    # Runtime accepts checkpoint/image artifacts only in these observation roots.
    for dirname in ('runs',):
        try:
            run.relative_to(ROOT / dirname)
            return run
        except ValueError:
            pass
    raise ValueError('run 必须位于本操作包 runs/ 内')


def initialize(run):
    run = within_run(run)
    run.mkdir(parents=True, exist_ok=False)
    for name in ('tmp', 'images', 'docs'):
        (run/name).mkdir()
    save(run/'round_spec.json', read(ROOT/'assets/templates/round_spec.json'))
    save(run/'resume.json', {
        'status': 'new_not_observed', 'run': str(run),
        'client_exec_session_id': None, 'active_prepared_file': None,
        'last_checkpoint_path': None, 'execution_request_submitted': False,
        'reminder': '收到发送后超时/用户打断不等于取消；先读结果或原会话，禁止盲目重发。'})
    return {'run': str(run), 'spec': str(run/'round_spec.json'), 'robot_requests': 0}


def transaction(state, source, target, run, stage, cfg, *, end_at_staging=False, continuation=False):
    travel, gate = cfg['travel'], cfg['gate']
    start = {k: state['right_arm'][k] for k in POSE_KEYS}
    start['gripper'] = 'hold'
    left = {k: state['left_arm'][k] for k in POSE_KEYS}
    left['gripper'] = 'hold'
    actor = cfg['physical_actor']
    init_values = cfg['init_xyz_rpy'][actor]
    init = dict(zip(POSE_KEYS, vector(init_values, 6, f'{actor} init')), gripper='hold')
    # the last pose of every return must be the init qpose (init_joint_rad). Travel to
    # above init, rotating while travelling into init roll/yaw with pitch RETURN_PITCH_RAD (reachable from
    # the carry branch; an in-place rotation at staging is branch-sensitive), then a joint_q step:
    # motion_harness changes configuration at travel height into the top of the vertical IK chain seeded
    # from init_joint_rad and descends along it, ending exactly on init_joint_rad.
    # Right actor: the init yaw (-0.50) is not reachable on the travel from staging; yaw 0 is (offline
    # search). The left actor keeps the init yaw (tested on the real robot).
    ret = {'roll': init['roll'], 'pitch': RETURN_PITCH_RAD, 'yaw': init['yaw'] if actor == 'left' else 0.}
    init_q = wp(init, joint_q=[float(v) for v in vector(cfg['init_joint_rad'][actor], 7, f'{actor} init joints')])
    def init_end():
        # The joint_q step swaps configuration at travel height with the TCP held fixed (pose-constrained
        # IK, motion_harness), then descends along the init joint chain.
        return [wp(init, z=travel, **ret), init_q, wp(init, gripper='close')]
    staging = wp(source, x=cfg['staging'][0], y=cfg['staging'][1], z=travel)
    target_staging = wp(staging, **{k:target[k] for k in ('roll','pitch','yaw')})
    source_high = wp(source, z=travel)
    gate_pose = wp(source, z=gate)
    target_high = wp(target, z=travel)
    # Extra high entrance accommodates the predecessor's predicted IK residual.
    # prepare_sequence refreshes this first waypoint to the actual predicted handoff.
    entrance_z = max(start['z'], travel) + (.010 if continuation else 0.)
    prefix = [wp(start,z=entrance_z),
              wp(start,x=staging['x'],y=staging['y'],z=entrance_z),
              wp(staging,z=entrance_z),wp(source_high,z=entrance_z)]
    if not continuation:
        prefix.append(wp(source_high,z=entrance_z,gripper='open'))
    prefix.append(gate_pose)
    return_fixed = [staging] + init_end()
    failure = [source_high,wp(source_high,gripper='close')] + return_fixed
    unknown = [wp(source,gripper='open'),gate_pose,source_high] + return_fixed
    success = [source,wp(source,gripper='close'),source_high]
    rotated = any(abs(source[k]-target[k])>1e-9 for k in ('roll','pitch','yaw'))
    if rotated:
        # Carry at travel height with the source orientation, then rotate in place above the target
        # (the left tray is reachable only at yaw -0.8, and that rotation is reachable
        # only above the tray, not above the board or at staging).
        success.append(wp(target_high,**{k:source[k] for k in ('roll','pitch','yaw')}))
    success += [target_high,target,wp(target,gripper='open'),target_high]
    if end_at_staging and rotated:
        # Capture first stage: rotate back in place above the target (the held high connector) and
        # hand over from there; the target orientation is not reachable at staging.
        success.append(wp(target_high,**{k:source[k] for k in ('roll','pitch','yaw')}))
    else:
        success.append(target_staging)
    if not end_at_staging:
        success += [wp(target_staging,gripper='close')] + init_end()
    def plan(points):
        return {'left_arm':[copy.deepcopy(left)],'right_arm':copy.deepcopy(points),
                'speed_scale':cfg['speed'],'skip_preflight':False}
    return {
        'grasp_plan':plan(prefix),'success_plan':plan(success),'failure_plan':plan(failure),
        'checkpoint_dir':str(run/'tmp/checkpoints'/stage), 'image_dir':str(run/'images'/stage),
        # Serial main-agent channel: every gate (source high, preclose, post-adjustment,
        # pre-release) waits without a deadline while the arm holds; no decision_timeout_s.
        'decision_channel':{'mode':'serial','require_worker':False},
        'gripper_open_q':cfg['open_q'],
        'pre_release_adjustment':{'review_pose':'after_descent_before_open',
                                  'adjustment_policy':'obvious_off_target_intersection',
                                  'alignment_criterion':('actor_wrist_predicted_landing_inside_same_side_tray'
                                                         if stage=='remove_opponent' else
                                                         'actor_wrist_predicted_landing_near_target_intersection'),
                                  'correction_motion':'direct_low_xy','max_corrections':3,
                                  'on_correction_limit':'release_only_if_final_correction_preapproved',
                                  'target_description':stage},
        'pregrasp_adjustment':{'max_corrections':3,
                              'require_post_adjustment_check':True,
                              'success_source_steps':[0,1,2]},
        'grasp_retry':{'max_attempts':cfg['max_attempts'],'after_close_step':1,'empty_below_q':.15,
                       'success_q':1.98,'success_tolerance_q':.03,'max_abs_dq':.1,
                       'success_hold_s':1.5,'contact_timeout_s':2.,
                       # Low gate: pitch reduction replaces Z steps; no extra
                       # precompiled heights, so the preflight covers only the planned source Z.
                       'confirm_after_descent':True,'max_low_z_adjustments':cfg['max_low_z_adjustments'],
                       'max_low_pitch_adjustments':cfg['max_low_pitch_adjustments'],'low_pitch_step_rad':.1,
                       'alignment_failure_policy':'recheck_high_then_grasp',
                       'max_high_rechecks':3,
                       'grasp_z_options_m':[source['z']],
                       'ignore_grasp_tool_envelope':cfg['ignore_source_envelope'],
                       'retry_plan':plan([gate_pose,wp(gate_pose,gripper='open')]),
                       'failure_plan':plan(failure),'failure_source_steps':[0,1],
                       'unknown_failure_plan':plan(unknown),'unknown_failure_source_steps':[0,1,2]}}


RETURN_PITCH_RAD = .4        # return orientation pitch (init roll/yaw) above init, before the joint swap
RELEASE_OPEN_WIDE_Q = 3.0     # release opening when both neighbours along the opening axis are empty


def release_open_q(target_square, yaw, occupied, source_square, normal_q):
    """Open wider at release unless a neighbour along the jaw-opening axis is occupied.

    The jaws open along the TCP y axis: fingers along +-y (|sin yaw| large) open along the ranks
    (neighbours rank+-1), fingers along x open along the files."""
    f, r = target_square[0], int(target_square[1])
    if abs(math.sin(yaw)) > .7:
        neighbours = [f+str(r+d) for d in (-1, 1) if 0 <= r+d <= 9]
    else:
        neighbours = [chr(ord(f)+d)+str(r) for d in (-1, 1) if 'a' <= chr(ord(f)+d) <= 'i']
    blocked = [n for n in neighbours if n in occupied and n != source_square]
    return (normal_q if blocked else max(normal_q, RELEASE_OPEN_WIDE_Q)), neighbours, blocked


def build(run, spec_path, observation_path):
    run = within_run(run)
    if any((run/'tmp'/name).exists() for name in ('request.json','plan_summary.json','prepared.json','execute_request.json','result.json')):
        raise ValueError('本run已有计划/令牌/结果；新版本请创建新run，避免覆盖或重放')
    spec, obs = read(spec_path), read(observation_path)
    profile = require_calibration(load_profile())
    if not obs.get('ok') or not isinstance(obs.get('paths'), dict):
        raise ValueError('必须提供本轮capture_cameras完整响应（含paths/state）')
    state = obs['state']
    if (state.get('robot') != 'R1-D' or state.get('geometry_ready') is not True or
            state.get('geometry_source') != geometry_source(profile)):
        raise ValueError('必须提供几何配置完整、来源一致的 R1-D 服务本轮观测')
    import hashlib
    digest = hashlib.sha256(json.dumps(profile, sort_keys=True, allow_nan=False).encode()).hexdigest()
    if state.get('profile_sha256') != digest:
        raise ValueError('当前profile与观测不一致；刷新观测并重新规划')
    if not str(spec.get('xy_error_context','')).startswith('r1d_'+digest[:16]):
        raise ValueError('xy_error_context必须以r1d_<profile_sha256前16位>开头，不能混用G1误差日志')
    if state.get('control_enabled') is not False or state.get('fault') is not None:
        raise ValueError('从控制关闭且无故障的本轮状态构建完整计划')
    if abs(scalar(state['grippers']['right']['q'], 'initial_right_gripper_q')) >= .15:
        raise ValueError('初始右爪不是明确空闭合状态；不能套用此空爪起步模板')
    for side in ('left_arm','right_arm'):
        vector([state[side][k] for k in POSE_KEYS],6,side)
    order={'left_arm':'right','right_arm':'left'} if state.get('active_arm')=='left' else {'left_arm':'left','right_arm':'right'}
    for key,physical in order.items():
        distance=math.sqrt(sum((state[key][axis]-profile['init_xyz_rpy'][physical][i])**2
                               for i,axis in enumerate(('x','y','z'))))
        if distance>.10+1e-9:
            raise ValueError(f'{physical}臂距自身init {distance:.4f}m，须先prepare_init/execute_init后刷新观测')
    if set(obs['paths']) != set(planning_cameras()):
        raise ValueError('走棋规划观测只允许 cam_left_high 一张图；执行门禁由服务选择头部+执行手腕部')
    for camera in planning_cameras():
        if not Path(obs['paths'].get(camera, '')).is_file():
            raise ValueError(f'缺少实际可读的本轮 {camera} 原图')
    side=spec['side']
    if side not in ('红方','黑方'):
        raise ValueError('必须指定红方或黑方')
    src, dst=square(spec['source_square']),square(spec['target_square'])
    if src==dst: raise ValueError('source和target不能相同')
    board = read(spec['board_state_file'])
    legality = validate_move(board, uci_move(src, dst))
    if board['game_id'] != spec.get('game_id') or (side == '红方') != (legality['fen'].split()[1] == 'w'):
        raise ValueError('棋局归属/行棋方与spec不一致')
    if type(spec.get('round_number')) is not int or spec['round_number']<1 or not spec.get('move_plan'):
        raise ValueError('规划前必须确定本局我方轮次与中文棋谱着法')
    occupied = pieces(legality['fen'])
    if (dst in occupied) != (spec.get('move_kind')=='capture'):
        raise ValueError('目标是否有对方棋子与普通走子/吃子事务类型不一致')
    if obs['paths']['cam_left_high'] not in board['evidence']:
        raise ValueError('棋局确认必须引用本轮规划头图')
    move_constraint = review_move(src, dst)
    if not move_constraint['allowed']:
        raise ValueError('走法不符合当前机械策略')
    for key in ('scene_and_neighbors_review','board_registration_review','pose_context_review','move_legality_review'):
        evidence(spec.get(key),key)
    source, target=pose(spec['source'],'source'),pose(spec['target'],'target')
    # spec 位姿是 TCP（法兰）；夹持点（离法兰 131 mm）必须落在所写格子的交叉点上。
    grip_points = check_grip_points(spec, load_board())
    # Release Z follows the calibrated fingertip height at this pose's pitch (grip_pose.release_tcp_z);
    # spec values are rounded to 0.1 mm by `bash r1d pose`.
    if not math.isclose(target['z'], release_tcp_z(profile, target['pitch'], 'board'), abs_tol=6e-5, rel_tol=0):
        raise ValueError('棋盘释放Z必须等于标定指尖释放高度在该pitch下的TCP高度（用 bash r1d pose 生成）')
    # Historical deltaXYZ means assist current-image estimation; never offset poses.
    xy_reference = reference_for_spec(spec, state.get('active_arm', 'right'), run=run)
    if any(cell['count'] for cell in xy_reference['phases'].values()):
        evidence(spec.get('xy_error_reference_review'), 'xy_error_reference_review')
    xy_reference['estimation_review'] = spec.get('xy_error_reference_review')
    cfg={'open_q':scalar(spec['gripper_open_q'],'open_q'),
         'gate':scalar(spec['gate_z_m'],'gate_z'),'travel':scalar(spec['travel_z_m'],'travel_z'),
         'staging':vector(spec['staging_xy_m'],2,'staging'),
         'speed':scalar(spec['speed_scale'],'speed_scale'),
         'ignore_source_envelope':spec['ignore_source_tool_envelope'],
         'max_attempts':spec.get('max_grasp_attempts',3),
         'max_low_z_adjustments':spec.get('max_low_z_adjustments',0),
         'max_low_pitch_adjustments':spec.get('max_low_pitch_adjustments',2),
         'physical_actor':state.get('active_arm', 'right'),
         'init_xyz_rpy':profile['init_xyz_rpy'],'init_joint_rad':profile['init_joint_rad']}
    if type(cfg['max_attempts']) is not int or not 1 <= cfg['max_attempts'] <= 3:
        raise ValueError('max_grasp_attempts必须为1到3的整数')
    if type(cfg['max_low_z_adjustments']) is not int or not 0 <= cfg['max_low_z_adjustments'] <= 2:
        raise ValueError('max_low_z_adjustments必须为0到2的整数')
    if type(cfg['max_low_pitch_adjustments']) is not int or not 0 <= cfg['max_low_pitch_adjustments'] <= 3:
        raise ValueError('max_low_pitch_adjustments必须为0到3的整数')
    if not 0 < cfg['open_q'] <= 5.4 or not 0 < cfg['speed'] <= 1:
        raise ValueError('open_q或speed_scale超出服务范围')
    if type(cfg['ignore_source_envelope']) is not bool:
        raise ValueError('source包络例外必须显式布尔值')
    if not source['z'] < cfg['gate'] <= cfg['travel'] or target['z'] >= cfg['travel']:
        raise ValueError('必须满足source低位<gate<=travel、target低位<travel')
    note=[f'{side} {src} → {dst}']
    capture_target = None
    if spec['move_kind']=='normal':
        if spec.get('capture') is not None: raise ValueError('普通走子capture必须为null')
        request={'transaction':transaction(state,source,target,run,'move',cfg)}
        kind='transaction'
    elif spec['move_kind']=='capture':
        cap=spec['capture'];other='黑方' if side=='红方' else '红方'
        if cap['side']!=other: raise ValueError('被吃方必须是对方')
        actor = state.get('active_arm', 'right')
        tray = cap['offboard_target']
        if tray.get('kind') != 'round_tray':
            raise ValueError('capture.offboard_target.kind 必须为 round_tray（棋盘侧边圆盘）')
        if actor not in ('left', 'right') or tray.get('physical_side') != actor:
            raise ValueError('被吃子必须放入物理执行手同侧圆盘：左手left、右手right')
        evidence(cap.get('offboard_surface_review'),'offboard_surface_review')
        victim=pose(cap['source'],'capture.source');drop=pose(cap['offboard_target'],'capture.offboard_target')
        if not math.isclose(drop['z'], release_tcp_z(profile, drop['pitch'], 'tray'), rel_tol=0., abs_tol=6e-5):
            raise ValueError('圆盘释放高度必须匹配本台R1-D标定profile')
        capture_target = {'kind': 'round_tray', 'physical_side': actor,
                          'pose': drop, 'evidence': tray['evidence'],
                          'surface_review': cap['offboard_surface_review']}
        if not victim['z'] < cfg['gate'] or drop['z'] >= cfg['travel']:
            raise ValueError('被吃子抓放低位必须低于对应高位')
        first=transaction(state,victim,drop,run,'remove_opponent',cfg,end_at_staging=True)
        handoff=copy.deepcopy(state);handoff['right_arm']=copy.deepcopy(first['success_plan']['right_arm'][-1])
        second=transaction(handoff,source,target,run,'move_own',cfg,continuation=True)
        request={'transactions':[first,second]};kind='sequence'
        note.insert(0,f'{other} {dst} → empty（{other} {dst} 被吃）')
    else: raise ValueError('move_kind只能为normal或capture')
    request['expected_active_arm'] = cfg['physical_actor']
    release_open, release_neighbours, release_blocked = release_open_q(
        dst, target['yaw'], occupied, src, cfg['open_q'])
    placement_transactions = request.get('transactions', [request.get('transaction')])
    for tx in placement_transactions:
        offboard = Path(tx['checkpoint_dir']).name == 'remove_opponent'
        tx['pre_release_adjustment']['release_open_q'] = (max(cfg['open_q'], RELEASE_OPEN_WIDE_Q) if offboard
                                                          else release_open)
        tx['pre_release_adjustment'].update(
            target_description=(f"被吃子放入物理{'左' if capture_target['physical_side']=='left' else '右'}手同侧圆盘内可用区域"
                                if offboard else f'{side} {src} → {dst}'),
            target_evidence=(spec['capture']['offboard_target']['evidence'] if offboard else spec['target']['evidence']))
    summary={'status':'built_not_preflighted_not_executed','kind':kind,'moves':note,
             'robot':'R1-D','profile_sha256':digest,'legality':legality,
             'game_id':spec.get('game_id'),'round_number':spec.get('round_number'),
             'move_plan':spec.get('move_plan'),
             'move_constraint':move_constraint,'grip_points':grip_points,
             'release_open':{'q':release_open,'axis_neighbours':release_neighbours,'occupied':release_blocked},
             'camera_selection':camera_selection(state.get('active_arm','right')),
             'spec':str(Path(spec_path).resolve()),'observation':str(Path(observation_path).resolve()),
             'observation_timestamp':obs.get('timestamp'),'initial_waist_q':state.get('waist_q'),
             'request_file':str(run/'tmp/request.json'),'prepare_command':'prepare_'+kind,
             'execute_command':'execute_'+kind,'robot_requests':0,'execution_submitted':False,
             'post_grasp_visual_gates':1,'release_max_xy_corrections':3,
             'release_limit_action':'release_only_if_final_correction_preapproved',
             'grasp_open_q_equals_release_q':cfg['open_q'],
             'source':source,'target':target,'collision_checked':False,
             'capture_target':capture_target,
             'source_square':src,'target_square':dst,'xy_error_reference':xy_reference,
             'limitation':'仅生成候选路径；需当前服务完整预检，不证明落点/邻子无碰撞。'}
    save(run/'tmp/request.json',request)
    summary['request_sha256']=hashlib.sha256((run/'tmp/request.json').read_bytes()).hexdigest()
    summary['request_binding_scope']='本地请求未变检查；服务未返回请求摘要，不证明token与请求内容完全绑定'
    save(run/'tmp/plan_summary.json',summary)
    return summary


def tokens(run, prepared_path):
    run=within_run(run);p=read(prepared_path)
    if p.get('ok') is not True: raise ValueError('预检未成功，不生成执行请求')
    summary=read(run/'tmp/plan_summary.json')
    digest=hashlib.sha256((run/'tmp/request.json').read_bytes()).hexdigest()
    if digest != summary.get('request_sha256'):
        raise ValueError('本轮request内容已改变或缺少构建摘要；创建新run后重新构建和预检')
    if summary['kind'] not in ('transaction', 'sequence'):
        raise ValueError('未知计划类型，不生成执行请求')
    sequence=summary['kind']=='sequence'
    request=read(run/'tmp/request.json')
    requested=request['transactions'] if sequence else [request['transaction']]
    key='sequence_token' if sequence else 'token'
    value=p.get(key)
    if not isinstance(value,str) or not value.strip():raise ValueError('缺少预检token')
    parts=p['transactions'] if sequence else [p]
    count=2 if sequence else 1
    if not isinstance(parts,list) or len(parts)!=count or len(requested)!=count:
        raise ValueError('预检事务数不一致')
    outputs=[run/'tmp/execute_request.json']+[run/'tmp'/f'stage_{i+1}_prepared.json' for i in range(count)]
    if any(path.exists() for path in outputs):raise ValueError('执行/检查点入口已存在；不覆盖或重复发令')
    if (run/'tmp/result.json').exists():raise ValueError('本run已有执行结果，禁止再次生成执行入口')
    seen_tokens=set()
    for item,tx in zip(parts,requested):
        token=item.get('token')
        if (not isinstance(token,str) or not token.strip() or
                any(c in token for c in ('/', '\\')) or token in seen_tokens):
            raise ValueError('预检子事务token无效或重复')
        seen_tokens.add(token)
        c=item['decision_channel']
        if c['mode']!='serial' or c['require_worker'] is not False:raise ValueError('必须为serial通道')
        if not isinstance(c.get('worker_id'),str) or not c['worker_id'].strip():
            raise ValueError('预检缺少serial检查点worker身份')
        if item.get('confirm_after_descent') is not True:raise ValueError('本模板预检缺失抓前低位确认')
        if (tx.get('grasp_retry',{}).get('alignment_failure_policy')=='recheck_high_then_grasp' and
                item.get('preclose_high_recheck_supported') is not True):
            raise ValueError('运行服务未加载低位返回高位复拍能力；空闲时加载更新并重新预检')
        if item.get('camera_selection') != summary['camera_selection']:
            raise ValueError('预检相机策略不匹配：规划只用left_high，执行只用头部和物理执行手腕部')
        checkpoint_dir = Path(item['checkpoint_path']).resolve().parent
        within_run(checkpoint_dir)
        try: checkpoint_dir.relative_to(run)
        except ValueError: raise ValueError('预检检查点不属于此run，禁止串用旧token')
        if checkpoint_dir != Path(tx['checkpoint_dir']).resolve():
            raise ValueError('预检检查点与本轮请求阶段/顺序不一致')
        release=item.get('pre_release_adjustment',{})
        expected=tx.get('pre_release_adjustment',{})
        if (not release or any(release.get(k)!=expected.get(k) for k in
                ('review_pose','adjustment_policy','alignment_criterion','correction_motion','max_corrections','on_correction_limit'))):
            raise ValueError('预检缺失或不匹配下降后开爪前复拍/执行手腕部落点交点判定能力；重新加载新版服务并预检')
        for field,suffix in (('checkpoint_path','_checkpoint.json'),
                             ('decision_path','_decision.json'),
                             ('receipt_path','_decision_receipt.json'),
                             ('checkpoint_discovery_path','_checkpoint_latest.json')):
            raw=item.get(field)
            if not isinstance(raw,str) or Path(raw).resolve()!=checkpoint_dir/(token+suffix):
                raise ValueError(f'预检{field}未绑定本阶段token，无法交给serial_checkpoint')
        if (checkpoint_dir/(token+'_result.json')).exists():
            raise ValueError('预检token已有执行结果，禁止生成重放请求')
        if (item.get('gripper_open_q') != summary['grasp_open_q_equals_release_q'] or
                item.get('gripper_open_q') != tx['gripper_open_q']):
            raise ValueError('预检开口与本轮计划不一致')
    # Publish the execution entry last, only after all serial descriptors exist.
    for path,item in zip(outputs[1:],parts):save(path,item)
    save(outputs[0],{key:value,'expected_active_arm':summary['camera_selection']['physical_actor']})
    return {'status':'prepared_execution_not_submitted','execute_request':str(outputs[0]),
            'prepared_stages':[str(p) for p in outputs[1:]],
            'request_sha256':digest,
            'request_binding_scope':'仅检查本地请求未改变、stage目录及响应身份；服务未返回请求摘要，未查询当前硬件/缓存',
            'warning':'token仅存在于当前服务进程，一次性；重启/已消费后不可复用'}


def report(result_path):
    r=read(result_path);parts=r.get('transactions',[r]);result=[]
    for part in parts:
        state=part.get('state',{});attempts=[]
        for a in part.get('grasp_attempts',[]):
            samples=a.get('samples',[]);q=[s['q'] for s in samples]
            attempts.append({'status':a.get('status'),'held_s':a.get('held_s'),
                             'q_range':[min(q),max(q)] if q else None})
        result.append({'outcome':part.get('outcome'),'error':part.get('error'),
                       'control_enabled':part.get('control_enabled'),'fault':state.get('fault',part.get('dds_fault')),
                       'grasp_attempts':attempts,'high_xy_corrections':len(part.get('adjustments',[])),
                       'release_xy_corrections':len(part.get('release_alignment',{}).get('corrections',[])),
                       'release_approval_reason':part.get('release_alignment',{}).get('approval_reason'),
                       'effective_release_target':part.get('release_alignment',{}).get('effective_target'),
                       'low_alignment_checks':part.get('low_alignment_checks',[]),
                       'final_timestamp':state.get('timestamp'),'final_right_arm':state.get('right_arm'),
                       'final_grippers':state.get('grippers'),'visual_verification':'requires_final_images'})
    return {'outcome':r.get('outcome'),'stages':result,
            'move_success':None,'reason':'动作结果不等于准确落格，完成后须图像验收source/target及邻子。'}


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='action',required=True)
    a=sub.add_parser('init');a.add_argument('--run',required=True)
    a=sub.add_parser('build');a.add_argument('--run',required=True);a.add_argument('--spec',required=True);a.add_argument('--observation',required=True)
    a=sub.add_parser('tokens');a.add_argument('--run',required=True);a.add_argument('--prepared',required=True)
    a=sub.add_parser('report');a.add_argument('--result',required=True)
    a=p.parse_args()
    if a.action=='init':out=initialize(a.run)
    elif a.action=='build':out=build(a.run,a.spec,a.observation)
    elif a.action=='tokens':out=tokens(a.run,a.prepared)
    else:out=report(a.result)
    print(json.dumps(out,ensure_ascii=False,indent=2,allow_nan=False))


if __name__=='__main__':
    try:main()
    except (ValueError,KeyError,TypeError,OSError) as e:
        print(json.dumps({'ok':False,'error':str(e)},ensure_ascii=False),file=sys.stderr)
        sys.exit(1)
