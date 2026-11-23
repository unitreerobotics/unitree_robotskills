#!/usr/bin/env python3
"""一条命令：启动象棋服务，并把机器人调到下棋参考位的允许误差内。

  bash r1d start --arm right             只读预览：各服务状态、当前离参考位多远，没有运动
  bash r1d start --arm right --execute   人已确认周围安全后执行

--execute 依次做（原来要手工串的 goinit → handover → 逐项核对）：
  1. Runtime（r1d-runtime.service）没在运行就启动；
  2. 象棋服务没在运行就作为 systemd 用户单元 r1d-chess-serve 启动（--allow-motion）；
  3. 象棋服务尚未接管时：goinit 把升降、双臂、头、腰带到参考位，然后 handover 交给象棋服务；
  4. 等腰、头稳定后核对：腰/头离参考位 ≤ 5 mrad、双臂离 init ≤ 20 mrad、升降在 profile 容差内。
不合格时以 [ERROR] 退出（码 3），不会默默继续。腰/头靠 config/hold_joints.json 的电机增益和
前馈到位。
"""
import argparse, json, subprocess, sys, time
from datetime import datetime
from pathlib import Path

import numpy as np

from r1d_chess.paths import ROOT
from r1d_chess.robot.r1d_config import load_profile, head_waist_reference
from r1d_chess.robot.robot_client import send_request

ENDPOINT = 'ipc:///tmp/astra_r1d_chess.sock'
RUNTIME_UNIT, SERVE_UNIT, GOINIT_UNIT = 'r1d-runtime.service', 'r1d-chess-serve', 'r1d-chess-goinit'
HEAD, WAIST = [29, 30], [13]
LEFT, RIGHT = list(range(15, 22)), list(range(22, 29))
WAIST_HEAD_TOLERANCE, ARM_TOLERANCE = 0.005, 0.020


def log(message):
    print(f'[Start] {datetime.now():%H:%M:%S} {message}', flush=True)


def fail(message, hint):
    print(f'[ERROR] [Start] {message}', file=sys.stderr)
    print(f'[ERROR] 需要 agent/人处理：{hint}', file=sys.stderr)
    raise SystemExit(3)


def active(unit):
    return subprocess.run(['systemctl', '--user', 'is-active', unit], capture_output=True, text=True).stdout.strip() == 'active'


def service_state():
    try:
        state, _ = send_request({'cmd': 'get_state'}, ENDPOINT, 1000, 5000)
        return state if state.get('ok') else None
    except Exception:
        return None


def deviations(profile, q):
    q = np.asarray(q, float)
    head, waist = head_waist_reference(profile)
    init = profile['init_joint_rad']
    return {'waist': float(abs(q[WAIST]-waist).max()), 'head': float(abs(q[HEAD]-head).max()),
            'left_arm': float(abs(q[LEFT]-np.asarray(init['left'])).max()),
            'right_arm': float(abs(q[RIGHT]-np.asarray(init['right'])).max())}


def within(d):
    return (d['waist'] <= WAIST_HEAD_TOLERANCE and d['head'] <= WAIST_HEAD_TOLERANCE
            and d['left_arm'] <= ARM_TOLERANCE and d['right_arm'] <= ARM_TOLERANCE)


def text(d):
    return ' '.join(f'{k} {1000*v:.1f} mrad' for k, v in d.items())


def goinit_ready(timeout, since):
    """Follow the goinit unit until it reports ready_for_chess, froze or failed."""
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        lines = subprocess.run(['journalctl', '--user', '-u', GOINIT_UNIT, '-n', '6', '--no-pager', '-o', 'cat'],
                               capture_output=True, text=True).stdout.strip().splitlines()
        for line in reversed(lines):
            try:
                event = json.loads(line)
            except ValueError:
                continue
            if event.get('time', 0.) < since:      # left over from an earlier goinit run
                break
            if event.get('event') in ('frozen', 'failed', 'stopped'):
                return False, event
            if event.get('event') == 'status':
                if event.get('ready_for_chess'):
                    return True, event
                break
        if time.time()-since > 5. and not active(GOINIT_UNIT+'.service'):
            return False, {'event': 'goinit unit is not running'}
        time.sleep(1.0)
    return False, {'event': 'timeout'}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--arm', choices=['left', 'right'], default='right')
    p.add_argument('--execute', action='store_true', help='人已确认机器人周围安全')
    p.add_argument('--settle', type=float, default=25.0, help='接管后最多等腰/头稳定的秒数')
    a = p.parse_args()
    profile = load_profile()
    python = sys.executable
    runtime, state = active(RUNTIME_UNIT), service_state()
    log(f'Runtime {"运行中" if runtime else "未运行"}；象棋服务 {"运行中" if state else "未运行"}'
        + (f'（active_arm={state.get("active_arm")} 已接管={state.get("control_enabled")} 故障={state.get("fault")}）' if state else ''))
    if state and state.get('physical_full_q'):
        log('当前离参考位：'+text(deviations(profile, state['physical_full_q'])))
    if not a.execute:
        log('只读预览，没有运动。确认机器人周围安全后：bash r1d start --arm %s --execute' % a.arm)
        return
    if not runtime:
        subprocess.run(['systemctl', '--user', 'start', RUNTIME_UNIT], check=True)
        time.sleep(6.0)
        if not active(RUNTIME_UNIT):
            fail('Runtime 启动失败', f'journalctl --user -u {RUNTIME_UNIT} -n 30')
        log('Runtime 已启动')
    if state is None:
        subprocess.run(['systemd-run', '--user', '--unit', SERVE_UNIT, '--collect', '--property=KillSignal=SIGINT',
                        '--property=TimeoutStopSec=15', f'--working-directory={ROOT}',
                        python, '-B', str(ROOT/'scripts/serve.py'), '--allow-motion', '--arm', a.arm], check=True)
        for _ in range(30):
            time.sleep(1.0)
            state = service_state()
            if state:
                break
        if state is None:
            fail('象棋服务没有应答', f'journalctl --user -u {SERVE_UNIT} -n 30')
        log(f'象棋服务已启动（单元 {SERVE_UNIT}）')
    if state.get('fault'):
        fail(f'象棋服务有锁存故障：{state["fault"]}', '没有运动时重启象棋服务清除，再重跑 bash r1d start')
    if state.get('active_arm') != a.arm:
        if state.get('control_enabled'):
            fail(f'象棋服务已接管且 active_arm={state.get("active_arm")}', '先结束当前会话或用 set_active_arm')
        send_request({'cmd': 'set_active_arm', 'arm': a.arm}, ENDPOINT, 1000, 5000)
    if not state.get('control_enabled'):
        if state.get('head_waist_session_active'):
            # release_control keeps the service's whole-body session; goinit needs the Runtime free.
            send_request({'cmd': 'release_waist'}, ENDPOINT, 1000, 10000)
            time.sleep(1.0)
            log('已结束象棋服务遗留的保持会话')
        since = time.time()-1.
        if not active(GOINIT_UNIT+'.service'):
            r = subprocess.run([python, '-B', str(ROOT/'scripts/go_init.py'), '--execute'], capture_output=True, text=True)
            if r.returncode:
                fail('goinit 启动失败：'+(r.stderr or r.stdout).strip()[-300:], 'bash r1d goinit --status')
        log('goinit 运行中：升降、双臂、头、腰回参考位')
        ok, event = goinit_ready(180.0, since)
        if not ok:
            fail(f'goinit 没有到位：{json.dumps(event, ensure_ascii=False)[:300]}', 'bash r1d goinit --status；检查现场后 bash r1d goinit --stop')
        log(f'goinit 到位：{event.get("error_to_target_rad")}')
        r = subprocess.run([python, '-B', str(ROOT/'scripts/handover.py'), '--arm', a.arm, '--execute'], capture_output=True, text=True)
        if r.returncode:
            fail('handover 失败：'+(r.stderr or r.stdout).strip()[-300:], '核对实际姿态和运动模式，不要盲目重试')
        log('已交给象棋服务')
    state = service_state()
    d = deviations(profile, state['physical_full_q'])
    if max(d['left_arm'], d['right_arm']) > ARM_TOLERANCE:
        log('交接后双臂下垂：'+text(d)+'；回到 init')
        response, _ = send_request({'cmd': 'settle_init'}, ENDPOINT, 1000, 20000)
        if not response.get('ok'):
            fail(f'settle_init 失败：{response.get("error")}', '双臂离 init 太远时重跑 bash r1d start')
    deadline, d, stable = time.monotonic()+a.settle, None, 0
    while time.monotonic() < deadline:
        time.sleep(1.0)
        state = service_state()
        if state is None or state.get('fault'):
            fail(f'接管后象棋服务异常：{state and state.get("fault")}', '查看象棋服务日志')
        d = deviations(profile, state['physical_full_q'])
        stable = stable+1 if within(d) else 0
        if stable >= 3:
            break
    log('离参考位：'+text(d))
    if stable < 3:
        fail('机器人没有进入参考位允许误差（腰/头 5 mrad，双臂 20 mrad）',
             '腰/头超差：检查 config/hold_joints.json 与 Runtime joint_gains 是否生效（Runtime 需重启才加载）；双臂超差：重跑 goinit')
    log('就绪：象棋服务已接管，机器人在参考位')


if __name__ == '__main__':
    main()
