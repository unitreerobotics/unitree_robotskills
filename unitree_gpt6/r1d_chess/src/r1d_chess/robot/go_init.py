#!/usr/bin/env python3
"""回到下棋初始位姿（升降、腰、头、双臂、夹爪），在启动象棋服务之前单独运行。

**不做任何预检**：不做 IK、碰撞、桌面净空或路径检查，轨迹是关节空间直线插值；只检查目标在
关节限位内。必须由人确认机器人周围、手臂运动范围内没有人和障碍物后再执行。

目标全部来自 config/robot.json（与 config/chess_initial_pose.json 同源）：
  lift_height_m、waist_q_rad、head_q_rad、init_joint_rad.left/right，夹爪闭合（0）。
到位后与象棋服务接管入口一致：升降差 ≤ lift_tolerance_m，头腰在参考位附近，夹爪 <0.15，静止。

用法：
  bash r1d goinit                 只读：打印当前值、目标、差值和运动时长，不动机器人
  bash r1d goinit --execute       确认安全后执行：作为 systemd 用户单元 r1d-chess-goinit 后台运行，
                                  到位后持续保持，直到被 handover 接管或 --stop
  bash r1d goinit --status        查看单元状态和最近日志
  bash r1d goinit --stop          停止保持（结束 Runtime 会话；手臂失去保持，先扶住）
之后：bash r1d serve --allow-motion --arm 所选手（另一终端/后台），再
  bash r1d handover --arm 所选手 --execute   停止本单元并由象棋服务接管。

运动顺序：先接管并原地保持 → 升降到目标高度（手臂保持当前姿态）→ 腰、头、双臂、夹爪一起五次多项式
插值到目标 → 持续保持。先调升降，是为了避免手臂已在桌面上方的 init 位置时再降低机身把手压向桌面。
"""
import argparse
import json
import os
import math
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from r1d_chess.paths import ROOT
from r1d_chess.robot.r1d_config import load_profile

UNIT = 'r1d-chess-goinit'
GROUPS = ('left_arm', 'right_arm', 'waist', 'head', 'left_ee', 'right_ee')
JOINT_SPEED_RAD_S = .15          # average joint speed; quintic peak is 1.9x this
MIN_DURATION_S = 6.
LIFT_MAX_SPEED_M_S = .02         # physical; column cap is 0.08 m/s at normalised 1.0
COLUMN_MAX_M_S = .08
LIFT_GAIN_PER_S = 1.5            # v = gain * error, clipped
LIFT_DONE_M = .0015
LIFT_MIN_SPEED_M_S = .008          # below ~5 % command the column stalls
LIFT_ACCEPT_STALLED_M = .003       # accept a stalled column this close (chess tolerance is 5 mm)
ARM_ABORT_RAD = .30              # tracking error that freezes the motion (hold where it is)
OTHER_ABORT_RAD = .20
STILL_QVEL = .1


def report(event, **values):
    print(json.dumps({'time': round(time.time(), 3), 'event': event, **values}, ensure_ascii=False), flush=True)


def targets_from_profile(profile):
    init = profile['init_joint_rad']
    return {
        'left_arm': np.asarray(init['left'], float), 'right_arm': np.asarray(init['right'], float),
        'waist': np.asarray(profile['waist_q_rad'], float), 'head': np.asarray(profile['head_q_rad'], float),
        'left_ee': np.zeros(1), 'right_ee': np.zeros(1),
    }


def quintic(u):
    u = min(1., max(0., u))
    return 10*u**3 - 15*u**4 + 6*u**5


def lift_of(state):
    base = state.observation.base_group or []
    if len(base) != 1 or base[0].lift_height is None or not math.isfinite(base[0].lift_height):
        raise RuntimeError('没有升降高度反馈（立柱未回零或底盘状态缺失）')
    return float(base[0].lift_height)


def plan(client, profile):
    state = client.state()
    current = {k: np.asarray(v, float) for k, v in client.current_joint_targets().items()}
    target = targets_from_profile(profile)
    for key in GROUPS:
        limits = np.asarray(client.robot_spec.joint_group_spec(key).qpos_limits[key], float)
        if target[key].shape != (len(limits),):
            raise ValueError(f'{key} 目标维数不对')
        if np.any(target[key] < limits[:, 0]) or np.any(target[key] > limits[:, 1]):
            raise ValueError(f'{key} 目标超出 Runtime 关节限位')
    delta = {k: float(np.max(np.abs(target[k]-current[k]))) for k in GROUPS}
    duration = max(MIN_DURATION_S, max(delta[k] for k in GROUPS if not k.endswith('_ee'))/JOINT_SPEED_RAD_S)
    lift_now = lift_of(state)
    lift_goal = float(profile['lift_height_m'])
    return {'state': state, 'current': current, 'target': target, 'delta': delta, 'duration': duration,
            'lift_now': lift_now, 'lift_goal': lift_goal,
            'lift_time': abs(lift_goal-lift_now)/LIFT_MAX_SPEED_M_S}


def summary(p):
    return {
        'no_preflight': '不做 IK/碰撞/桌面净空/路径预检，关节空间直线插值；只检查关节限位。确认安全后再 --execute',
        'runtime_control_active': bool(p['state'].control_active),
        'lift_m': {'now': round(p['lift_now'], 4), 'target': round(p['lift_goal'], 4),
                   'delta_mm': round((p['lift_goal']-p['lift_now'])*1000, 1), 'est_s': round(p['lift_time'], 1)},
        'joints': {k: {'now': np.round(p['current'][k], 3).tolist(), 'target': np.round(p['target'][k], 3).tolist(),
                       'max_delta_rad': round(p['delta'][k], 3)} for k in GROUPS},
        'joint_motion_s': round(p['duration'], 1),
        'order': ['原地保持 1 s', '升降到目标（手臂保持）', '腰/头/双臂/夹爪一起插值', '持续保持直到 handover 或 --stop'],
    }


def run(profile):
    """Foreground worker (normally inside the systemd unit)."""
    from r1d_chess.robot.r1d_runtime import R1DRuntimeClient, load_experiment
    from r1d_chess.robot.r1d_torque_ff import TorqueFeedforward, load_config as load_torque_ff
    from r1d_chess.robot.r1d_hold_ff import HoldFeedforward
    experiment = load_experiment()           # puts the robot runtime source on sys.path
    from robot_runtime.schema.frame import ActionData
    from robot_runtime.schema.joint_group import JointGroupData
    from robot_runtime.schema.base_group import BaseGroupData
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from robot_runtime.drivers.unitree_sdk2.robot.motion.switcher import MotionServiceSwitcher

    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    client = R1DRuntimeClient()
    switcher = MotionServiceSwitcher(restore_motion_mode='ai')
    ff = TorqueFeedforward(load_torque_ff())
    hold_ff = HoldFeedforward()
    subs, lowcmd_at, feedback = [], [0.], [None]
    entered = False

    def on_state(message):
        previous = feedback[0]
        if previous is None or message.tick != previous[1]:
            feedback[0] = (time.monotonic(), message.tick)

    def fresh():
        if feedback[0] is None or time.monotonic()-feedback[0][0] > .15:
            raise RuntimeError('rt/lowstate 0.15 s 内没有更新')

    def send(targets, lift_velocity, qtau, hold=(0., 0., 0.)):
        client.send_action(ActionData(
            joint_groups=[JointGroupData(schema_key=k, qpos=[float(v) for v in targets[k]],
                                         qtau=([float(v) for v in qtau[:7]] if k == 'left_arm' else
                                               [float(v) for v in qtau[7:]] if k == 'right_arm' else
                                               [float(v) for v in hold[:2]] if k == 'head' else
                                               [float(v) for v in hold[2:]] if k == 'waist' else None))
                          for k in GROUPS],
            base_group=[BaseGroupData(schema_key='base_lift', base_linear_vel_x=0., base_linear_vel_y=0.,
                                      base_angular_vel_z=0., lift_linear_vel_z=float(lift_velocity))]))

    try:
        client.connect()
        p = plan(client, profile)
        state = p['state']
        if state.control_active or state.reset_pose_active or state.runtime_error:
            raise RuntimeError('Runtime 正被占用或有错误（象棋服务/其他保持程序在控制？）')
        for group in state.observation.joint_groups:
            if any(abs(v) > STILL_QVEL for v in group.qvel or ()):
                raise RuntimeError(f'{group.schema_key} 在运动，拒绝接管')
        report('plan', **summary(p))
        ChannelFactoryInitialize(0, experiment.robot_driver_configs['unitree_sdk2']['network_interface'])
        sub = ChannelSubscriber('rt/lowcmd', LowCmd_); sub.Init(lambda _: lowcmd_at.__setitem__(0, time.monotonic()), 1)
        fb = ChannelSubscriber('rt/lowstate', LowState_); fb.Init(on_state, 1)
        subs += [sub, fb]
        switcher.initialize()
        mode, payload = switcher.service_state()
        time.sleep(.3)
        fresh()
        report('motion_mode', mode=payload)
        if mode not in ('ai', ''):
            raise RuntimeError(f'意外的运动模式 {mode!r}')
        if mode == 'ai':
            switcher.release_control_service(timeout=6.)
        time.sleep(.15)
        quiet = time.monotonic()
        time.sleep(.15)
        if lowcmd_at[0] >= quiet:
            raise RuntimeError('释放 AI 后仍有其他 LowCmd 发布者')
        fresh()
        p = plan(client, profile)            # fresh snapshot right before taking control
        start_q, target, duration = p['current'], p['target'], p['duration']
        client.begin(include_base=True)
        entered = True
        switcher.restore_required = False
        report('control_started', joint_motion_s=round(duration, 1))
        phase, phase_t0, frozen = 'settle', time.monotonic(), None
        command = {k: v.copy() for k, v in start_q.items()}
        arm_prev = np.r_[command['left_arm'], command['right_arm']]
        last, last_report, lift_done_since = time.monotonic(), 0., None
        lift_history = []
        while True:
            now = time.monotonic(); dt = max(1e-3, now-last); last = now
            fresh()
            state = client.state_receiver.latest_state(max_age=.15)
            if state.runtime_error or state.thermal_protection_active:
                raise RuntimeError(f'Runtime 故障：{state.runtime_error} thermal={state.thermal_protection_active}')
            if now-phase_t0 > .5 and not state.control_active:
                raise RuntimeError('Runtime 会话意外结束')
            measured = {k: np.asarray(state.observation.joint_group_qpos(k), float) for k in GROUPS}
            lift = lift_of(state)
            lift_v = 0.
            if frozen is None:
                if phase == 'settle' and now-phase_t0 >= 1.:
                    phase, phase_t0 = 'lift', now
                    report('phase', phase='lift', lift_now=round(lift, 4), lift_target=p['lift_goal'])
                if phase == 'lift':
                    error = p['lift_goal']-lift
                    lift_history.append((now, lift))
                    while lift_history and now-lift_history[0][0] > 1.5:
                        lift_history.pop(0)
                    stalled = (now-phase_t0 > 3. and lift_history and now-lift_history[0][0] > 1.4 and
                               max(h for _, h in lift_history)-min(h for _, h in lift_history) < .0003)
                    if abs(error) > LIFT_DONE_M:
                        lift_done_since = None
                        # The column stalls on tiny speed commands (stopped 2.8 mm short at
                        # ~5 % speed), so keep at least LIFT_MIN_SPEED_M_S until inside the done band.
                        v = float(np.clip(LIFT_GAIN_PER_S*abs(error), LIFT_MIN_SPEED_M_S, LIFT_MAX_SPEED_M_S))
                        lift_v = math.copysign(v, error)/COLUMN_MAX_M_S
                    else:
                        lift_done_since = lift_done_since or now
                    accept = None
                    if lift_done_since and now-lift_done_since > .6:
                        accept = 'reached'
                    elif stalled and abs(error) <= LIFT_ACCEPT_STALLED_M:
                        accept = 'stalled_within_3mm'
                    elif now-phase_t0 > p['lift_time']+30:
                        if abs(error) <= profile['lift_tolerance_m']:
                            accept = 'timeout_within_chess_tolerance'
                        else:
                            frozen = {k: measured[k].copy() for k in GROUPS}
                            command = {k: v.copy() for k, v in frozen.items()}
                            report('frozen', reason=f'升降未到位：{lift:.4f} m，目标 {p["lift_goal"]:.4f} m',
                                   note='原地保持，不再继续；检查立柱后 --stop')
                    if accept:
                        phase, phase_t0 = 'joints', now
                        report('phase', phase='joints', lift=round(lift, 4), lift_accept=accept,
                               lift_error_mm=round(-error*1000, 1), duration_s=round(duration, 1))
                if phase in ('joints', 'hold'):
                    fraction = quintic((now-phase_t0)/duration)
                    command = {k: start_q[k]+(target[k]-start_q[k])*fraction for k in GROUPS}
                    if phase == 'joints' and fraction >= 1.:
                        phase = 'hold'
                        report('phase', phase='hold')
                errors = {k: float(np.max(np.abs(measured[k]-command[k]))) for k in GROUPS if not k.endswith('_ee')}
                worst = max(errors, key=errors.get)
                if errors[worst] > (ARM_ABORT_RAD if worst.endswith('arm') else OTHER_ABORT_RAD):
                    frozen = {k: measured[k].copy() for k in GROUPS}
                    command = {k: v.copy() for k, v in frozen.items()}
                    report('frozen', reason=f'{worst} 跟随误差 {errors[worst]:.3f} rad', note='原地保持，不再继续；检查现场后 --stop')
            arm_cmd = np.r_[command['left_arm'], command['right_arm']]
            qtau = ff.compute(np.r_[measured['left_arm'], measured['right_arm']], arm_cmd, (arm_cmd-arm_prev)/dt, dt)
            arm_prev = arm_cmd
            velocity = {g.schema_key: np.asarray(g.qvel if g.qvel is not None else np.zeros(len(g.qpos)), float)
                        for g in state.observation.joint_groups if g.schema_key in ('head', 'waist')}
            hold = hold_ff.compute(np.r_[measured['head'], measured['waist']], np.r_[velocity['head'], velocity['waist']],
                                   np.r_[command['head'], command['waist']], dt)
            send(command, lift_v, qtau, hold)
            if now-last_report >= 2.:
                last_report = now
                report('status', phase=('frozen' if frozen else phase), lift=round(lift, 4),
                       lift_error_mm=round((lift-p['lift_goal'])*1000, 1),
                       error_to_target_rad={k: round(float(np.max(np.abs(measured[k]-target[k]))), 3) for k in GROUPS},
                       ready_for_chess=bool(frozen is None and phase == 'hold'
                                            and abs(lift-p['lift_goal']) <= profile['lift_tolerance_m']
                                            and max(abs(measured['left_ee'][0]), abs(measured['right_ee'][0])) < .15))
            time.sleep(max(0., .02-(time.monotonic()-now)))
    except KeyboardInterrupt:
        report('stop_requested')
    except Exception as exc:
        report('failed', error=str(exc), entered=entered)
        raise
    finally:
        try:
            if entered:
                # Ramp feedforward out while holding the last command, then end the session.
                ff.request_zero(True)
                hold_ff.request_zero(True)
                end = time.monotonic()+1.5
                while time.monotonic() < end and (np.any(ff.output != 0) or np.any(hold_ff.output != 0)):
                    arm_cmd = np.r_[command['left_arm'], command['right_arm']]
                    qtau = ff.compute(arm_cmd, arm_cmd, np.zeros(14), .02)
                    target = np.r_[command['head'], command['waist']]
                    hold = hold_ff.compute(target, np.zeros(3), target, .02)
                    try:
                        send(command, 0., qtau, hold)
                    except Exception:
                        break
                    time.sleep(.02)
            client.close()
        finally:
            if not entered:
                switcher.restore_service()
            for s in subs:
                s.Close()
            switcher.shutdown()
            report('stopped', entered=entered)


def systemctl(*args, check=True):
    return subprocess.run(['systemctl', '--user', *args], capture_output=True, text=True, check=check)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = p.add_mutually_exclusive_group()
    g.add_argument('--execute', action='store_true', help='人已确认安全：作为 systemd 用户单元后台执行并保持')
    g.add_argument('--status', action='store_true')
    g.add_argument('--stop', action='store_true', help='停止保持（手臂失去保持）')
    g.add_argument('--run', action='store_true', help=argparse.SUPPRESS)   # worker entry inside the unit
    a = p.parse_args()
    profile = load_profile()
    if a.run:
        return run(profile)
    if a.status:
        print(systemctl('is-active', UNIT+'.service', check=False).stdout.strip())
        print(subprocess.run(['journalctl', '--user', '-u', UNIT, '-n', '15', '--no-pager', '-o', 'cat'],
                             capture_output=True, text=True).stdout)
        return
    if a.stop:
        systemctl('stop', UNIT+'.service', check=False)
        print(systemctl('is-active', UNIT+'.service', check=False).stdout.strip())
        return
    if systemctl('is-active', UNIT+'.service', check=False).stdout.strip() == 'active':
        raise SystemExit(f'{UNIT} 已在运行；用 --status 查看或 --stop 停止')
    from r1d_chess.robot.r1d_runtime import R1DRuntimeClient
    with R1DRuntimeClient() as client:
        info = plan(client, profile)
    print(json.dumps(summary(info), ensure_ascii=False, indent=2))
    if not a.execute:
        print('\n只读预览，没有运动。确认机器人周围安全后执行：bash r1d goinit --execute')
        return
    if info['state'].control_active:
        raise SystemExit('Runtime 正被占用（象棋服务已接管或其他保持程序在运行），拒绝执行')
    python = sys.executable
    subprocess.run(['systemd-run', '--user', '--unit', UNIT, '--collect', '--property=KillSignal=SIGINT',
                    '--property=TimeoutStopSec=10', f'--working-directory={ROOT}',
                    python, '-B', str(ROOT/'scripts/go_init.py'), '--run'], check=True)
    print(f'已启动 {UNIT}。查看进度：bash r1d goinit --status 或 journalctl --user -u {UNIT} -f')


if __name__ == '__main__':
    main()
