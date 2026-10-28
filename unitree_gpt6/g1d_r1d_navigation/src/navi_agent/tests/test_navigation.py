"""Pure navigation tests: synthetic DDS messages only; never construct a real backend."""
import json
import math
from types import SimpleNamespace as NS

import pytest

from core.places import PlaceStore
from services.navigation import Localization, NavigationManager, resolve_goal_frame


class Clock:
    t = 10.
    def __call__(self):
        return self.t


def feed(loc, *, x=0., y=0., yaw=0., speed=0., wz=0., frame='map', state='connected'):
    loc.on_pose(NS(header=NS(frame_id=frame), pose=NS(position=NS(x=x, y=y),
        orientation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2)))))
    loc.on_odom(NS(twist=NS(twist=NS(linear=NS(x=speed, y=0.), angular=NS(z=wz)))))
    loc.on_state(NS(data=state))


def feed_no_odom(loc, *, x=0., y=0., yaw=0., frame='map', state='connected'):
    """Fresh map pose + SLAM state, but NO measured odom twist (onboard pose-only)."""
    loc.on_pose(NS(header=NS(frame_id=frame), pose=NS(position=NS(x=x, y=y),
        orientation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2)))))
    loc.on_state(NS(data=state))


class Backend:
    def __init__(self):
        self.calls = []
        self.fail = None
    def _call(self, name):
        self.calls.append(name)
        if self.fail == name:
            raise RuntimeError(name + ' failed')
    def acquire(self): self._call('acquire')
    def release(self): self._call('release')
    def goal(self, pose): self._call('goal')
    def halt(self): self._call('halt')


@pytest.fixture
def rig(tmp_path):
    clock = Clock()
    loc = Localization('map', now=clock)
    feed(loc)
    store = PlaceStore(tmp_path/'points.json', 'lab-v1', 'map')
    store.save('B', dict(x=1., y=0., yaw=0., frame='map'))
    backend = Backend()
    def sleep(dt):
        clock.t += dt
        feed(loc)
    nav = NavigationManager(loc, backend, store, now=clock, start_thread=False, sleep=sleep)
    return nav, loc, backend, clock


def test_place_persistence_and_overwrite(tmp_path):
    store = PlaceStore(tmp_path/'points.json', 'lab-v1', 'map')
    pose = dict(x=1., y=2., yaw=.3, frame='map')
    store.save(' A ', pose)
    assert PlaceStore(store.path, 'lab-v1', 'map').get('A')['x'] == 1.
    with pytest.raises(ValueError): store.save('A', pose)
    store.save('A', {**pose, 'x': 3.}, overwrite=True)
    assert store.list()[0]['x'] == 3.
    with pytest.raises(ValueError): PlaceStore(store.path, 'lab-v2', 'map').list()
    with pytest.raises(ValueError): PlaceStore(store.path, 'lab-v1', 'odom').list()
    with pytest.raises(ValueError): store.get('unknown')
    with pytest.raises(ValueError): store.save('B', {**pose, 'frame': 'odom'})
    with pytest.raises(ValueError): store.save('B', {**pose, 'x': math.nan})
    with pytest.raises(ValueError): store.save('\n', pose)
    store.path.write_text('{broken')
    with pytest.raises(ValueError): store.list()


def test_localization_missing_stale_wrong_frame_and_invalid(rig):
    _, loc, _, clock = rig
    assert loc.snapshot()['ok']
    clock.t += 2
    assert set(loc.snapshot()['errors']) == {'pose_stale', 'odom_stale', 'slam_state_stale'}
    feed(loc, frame='odom', speed=math.inf, state='disconnected')
    assert set(loc.snapshot()['errors']) == {'frame_mismatch', 'invalid_twist', 'slam_not_connected'}
    feed(loc, x=math.nan)
    assert 'invalid_pose' in loc.snapshot()['errors']
    json.dumps(loc.snapshot(), allow_nan=False)
    assert not Localization('map').snapshot()['ok']


@pytest.mark.parametrize('topic', ['pose', 'odom', 'state'])
def test_each_source_must_remain_fresh(rig, topic):
    _, loc, _, clock = rig
    clock.t += 2
    old = getattr(loc, '_'+topic)
    feed(loc)
    setattr(loc, '_'+topic, old)
    assert not loc.snapshot()['ok']


def test_teaching_requires_fresh_still_and_idle(rig):
    nav, loc, _, clock = rig
    nav.save_place('A')
    feed(loc, speed=.1)
    with pytest.raises(ValueError): nav.save_place('C')
    feed(loc)
    clock.t += 2
    with pytest.raises(ValueError): nav.save_place('C')
    feed(loc)
    assert nav.start('B')['active']
    with pytest.raises(ValueError): nav.save_place('C')


def test_start_rejects_unmarked_far_moving_and_busy(rig):
    nav, loc, b, _ = rig
    with pytest.raises(ValueError): nav.start('missing')
    feed(loc, x=30.)
    assert not nav.start('B')['ok']
    feed(loc, speed=.1)
    assert not nav.start('B')['ok']
    assert b.calls == []
    feed(loc)
    assert nav.start('B')['state'] == 'running'
    assert b.calls == ['acquire', 'goal']
    assert not nav.start('B')['ok']


def test_arrival_requires_yaw_stationary_and_dwell(rig):
    nav, loc, b, clock = rig
    nav.start('B')
    feed(loc, x=1., yaw=.5)
    assert nav.step() == 'running'
    feed(loc, x=1., speed=.1)
    assert nav.step() == 'running'
    feed(loc, x=1.)
    assert nav.step() == 'running'
    clock.t += 1.1
    feed(loc, x=1.)
    assert nav.step() == 'arrived'
    # Still owns the base until cancellation + NEW measured stationary samples.
    assert nav.status()['active']
    def sleep(dt):
        clock.t += dt
        feed(loc, x=1.)
    nav.sleep = sleep
    nav._finish('arrived')
    assert nav.status()['state'] == 'arrived' and not nav.status()['active']
    assert b.calls[-2:] == ['halt', 'release']


@pytest.mark.parametrize('verdict', ['timeout', 'stalled', 'localization_lost', 'cancelled'])
def test_progress_failure_paths(rig, verdict):
    nav, loc, _, clock = rig
    nav.start('B')
    nav.step()
    if verdict == 'timeout':
        clock.t += 121
        feed(loc, x=1., yaw=1.)  # At position, wrong yaw still times out.
    elif verdict == 'stalled':
        clock.t += 21
        feed(loc)
    elif verdict == 'localization_lost':
        clock.t += 2
    else:
        nav._cancel.set()
    assert nav.step() == verdict


@pytest.mark.parametrize('failure', ['goal', 'halt', 'old_feedback'])
def test_fault_keeps_ownership_until_confirmed_stop(rig, failure):
    nav, loc, b, clock = rig
    if failure == 'goal': b.fail = 'goal'
    nav.start('B')
    if failure == 'halt': b.fail = 'halt'
    if failure == 'old_feedback':
        nav.sleep = lambda dt: setattr(clock, 't', clock.t+dt)
    if failure != 'goal': nav._finish('cancelled')
    assert nav.status()['state'] == 'fault' and nav.status()['active']
    assert 'release' not in b.calls
    b.fail = None
    def sleep(dt):
        clock.t += dt
        feed(loc)
    nav.sleep = sleep
    result = nav.stop()
    assert result['stop_requested']
    nav._thread.join(timeout=1)
    assert not nav.status()['active'] and nav.status()['state'] == 'cancelled'


def test_cached_post_cancel_sample_alone_cannot_confirm_dwell(rig):
    nav, loc, b, clock = rig
    nav.start('B')
    def sleep(dt):
        clock.t += dt
        if len(b.calls) == 3 and clock.t < 10.1:
            feed(loc)  # only first new sample, then source freezes
    nav.sleep = sleep
    nav._finish('cancelled')
    assert nav.status()['active'] and 'release' not in b.calls


def test_onboard_detour_is_progress_but_stationary_noise_is_not(rig):
    nav, loc, _, clock = rig
    nav.stall_s = .5
    nav.start('B')
    for i in range(20):
        clock.t += .1
        feed(loc, x=-.02*i, yaw=math.pi, speed=.2)
        assert nav.step() == 'running'
    # Noise around one actual position must not replenish the movement budget.
    verdict = None
    for i in range(10):
        clock.t += .1
        feed(loc, x=-.38+(.005 if i % 2 else -.005))
        verdict = nav.step()
        if verdict != 'running':
            break
    assert verdict == 'stalled'


@pytest.mark.parametrize('bad_source', ['duplicate', 'old', 'future', 'missing'])
def test_strict_onboard_stop_never_releases_on_replayed_feedback(rig, bad_source):
    nav, _, backend, clock = rig
    loc = Localization('map', now=clock, strict_timestamps=True, wall_now=clock)
    nav.localization = loc
    source = clock()
    def telemetry(stamp):
        header = NS(frame_id='map', stamp=NS(sec=int(stamp), nanosec=round((stamp-int(stamp))*1e9)))
        loc.on_pose(NS(header=header, pose=NS(position=NS(x=0., y=0.),
                     orientation=NS(x=0., y=0., z=0., w=1.))))
        loc.on_odom(NS(header=header, twist=NS(twist=NS(linear=NS(x=0., y=0.), angular=NS(z=0.)))))
        loc.on_state(NS(data='connected'))
    telemetry(source)
    assert nav.start('B')['active']
    def sleep(dt):
        clock.t += dt
        stamp = {'duplicate': source, 'old': clock()-10., 'future': clock()+1., 'missing': 0.}[bad_source]
        telemetry(stamp)
    nav.sleep = sleep
    nav._finish('cancelled')
    assert nav.status()['state'] == 'fault' and nav.status()['active']
    assert 'release' not in backend.calls
    # Recovery requires genuinely new measurement timestamps.
    nav.sleep = lambda dt: (setattr(clock, 't', clock.t+dt), telemetry(clock()))
    nav._finish('cancelled')
    assert not nav.status()['active']


@pytest.mark.parametrize('fail_acquire', [False, True])
def test_stop_during_handoff_returns_promptly_and_eventually_confirms(rig, monkeypatch, fail_acquire):
    import threading
    nav, _, backend, _ = rig
    entered, resume = threading.Event(), threading.Event()
    original = backend.acquire
    def acquire():
        entered.set()
        assert resume.wait(3.)
        if fail_acquire:
            raise RuntimeError('handoff failure')
        original()
    monkeypatch.setattr(backend, 'acquire', acquire)
    worker = threading.Thread(target=lambda: nav.start('B'))
    worker.start()
    try:
        assert entered.wait(2.)
        # No sleeps/timing assumptions: the blocked acquisition is still blocked
        # when stop returns, and its event is already visible to startup.
        result = nav.stop()
        assert result['stop_requested'] and nav._cancel.is_set()
        assert not resume.is_set() and 'goal' not in backend.calls
        with nav._stop_waiters_lock:
            waiters = list(nav._stop_waiters.values())
    finally:
        resume.set()
        worker.join(3.)
    for thread in waiters:
        thread.join(3.)
        assert not thread.is_alive()
    nav._thread.join(3.)
    assert 'goal' not in backend.calls
    assert nav.status()['state'] == 'cancelled' and not nav.status()['active']


def test_cancelled_dispatch_cannot_be_reset_by_start(rig):
    import threading
    nav, _, backend, _ = rig
    cancelled = threading.Event()
    cancelled.set()
    assert nav.start('B', cancel=cancelled)['state'] == 'cancelled'
    assert not backend.calls


def test_deferred_stop_does_not_cancel_a_new_session(rig):
    import threading
    nav, _, backend, _ = rig
    with nav._lock:
        stopper = threading.Thread(target=nav.stop)
        stopper.start(); stopper.join(2.)
        assert not stopper.is_alive()
        with nav._stop_waiters_lock:
            waiters = list(nav._stop_waiters.values())
        # Start a new epoch before the old waiter gets the lock.
        nav.start('B')
    for waiter in waiters:
        waiter.join(2.)
    assert not nav._cancel.is_set()
    assert backend.calls == ['acquire', 'goal']


def test_navigation_safety_check_blocks_save_start_and_running(rig):
    nav, _, backend, _ = rig
    nav.safety_check = lambda goal=None: {'ok': False, 'errors': ['pose_outside_map_bounds']}
    with pytest.raises(ValueError, match='navigation_safety_blocked'):
        nav.save_place('unsafe')
    out = nav.start('B')
    assert not out['ok'] and out['error'] == 'navigation_safety_blocked'
    assert backend.calls == []


# ── onboard-SLAM fallback path: optional odom + decoupled goal frame (off by default, per instance/config) ──
# Pose-only onboard goto needs only a fresh map pose + slam connected;
# without odom, arrival and stop confirmation fall back to position hold.

def test_odom_optional_pose_only_start_arrival_and_finish(tmp_path):
    clock = Clock()
    loc = Localization('map', now=clock)
    feed_no_odom(loc, x=0.)                                  # fresh pose + slam, no measured twist
    store = PlaceStore(tmp_path/'points.json', 'lab-v1', 'map')
    store.save('B', dict(x=1., y=0., yaw=0., frame='map'))
    backend = Backend()
    def sleep(dt):
        clock.t += dt
        feed_no_odom(loc, x=1.)
    nav = NavigationManager(loc, backend, store, now=clock, start_thread=False,
                            sleep=sleep, odom_required=False, position_hold_s=1.)
    # Pose-only start: allowed without odom when the pose is fresh and slam is connected.
    assert nav.start('B')['state'] == 'running'
    assert backend.calls == ['acquire', 'goal']
    feed_no_odom(loc, x=1.)
    assert nav.step() == 'running'                           # in position but hold time not yet met
    clock.t += 1.1
    feed_no_odom(loc, x=1.)
    assert nav.step() == 'arrived'                           # held >= position_hold_s
    nav._finish('arrived')
    assert nav.status()['state'] == 'arrived' and not nav.status()['active']
    assert backend.calls[-2:] == ['halt', 'release']         # released only after pose deltas confirm standstill


def test_default_manager_still_requires_odom_for_start(tmp_path):
    # Regression guard: with the default odom_required=True, missing odom still blocks start (the relaxation must not leak into the inherited smooth path).
    clock = Clock()
    loc = Localization('map', now=clock)
    feed_no_odom(loc, x=0.)                                  # fresh pose + slam, but no twist
    store = PlaceStore(tmp_path/'points.json', 'lab-v1', 'map')
    store.save('B', dict(x=1., y=0., yaw=0., frame='map'))
    backend = Backend()
    nav = NavigationManager(loc, backend, store, now=clock, start_thread=False)
    out = nav.start('B')
    assert not out['ok'] and '停稳' in out['error']
    assert backend.calls == []


@pytest.mark.parametrize('kwargs', [
    {'odom_required': 'yes'}, {'odom_required': 1}, {'position_hold_s': 0.}, {'position_hold_s': -1.},
])
def test_constructor_rejects_bad_odom_flag_and_hold(tmp_path, kwargs):
    loc = Localization('map')
    store = PlaceStore(tmp_path/'points.json', 'lab-v1', 'map')
    with pytest.raises(ValueError):
        NavigationManager(loc, Backend(), store, start_thread=False, **kwargs)


def test_resolve_goal_frame_decouples_from_pose_frame():
    # Missing/empty goal_frame falls back to the pose frame (unchanged behavior).
    assert resolve_goal_frame({'frame': 'robot_pose'}) == 'robot_pose'
    assert resolve_goal_frame({'frame': 'robot_pose', 'goal_frame': ''}) == 'robot_pose'
    # An explicit goal_frame wins; onboard goto goals are usually in 'map'.
    assert resolve_goal_frame({'frame': 'robot_pose', 'goal_frame': 'map'}) == 'map'


def test_pose_only_finish_never_releases_while_still_creeping(tmp_path):
    # Safety guard: pose-only standstill confirmation must use a speed gate (0.03 m/s), not a per-frame displacement gate.
    # Creeping at ~0.15 m/s moves <2cm per 0.05s loop, so the old `dist<0.02` check would misjudge
    # standstill and release a still-moving servo chassis (releasing a moving chassis is dangerous).
    # Continuous creep -> standstill never confirmed -> must keep the control lock and fault, never release.
    clock = Clock()
    loc = Localization('map', now=clock)
    feed_no_odom(loc, x=0.)
    store = PlaceStore(tmp_path/'points.json', 'lab-v1', 'map')
    store.save('B', dict(x=1., y=0., yaw=0., frame='map'))
    backend = Backend()
    creep = {'x': 0.}
    def sleep(dt):
        clock.t += dt
        creep['x'] += 0.15*dt            # 0.15 m/s: 0.0075m per 0.05s (the old <0.02 check would call this standstill)
        feed_no_odom(loc, x=creep['x'])
    nav = NavigationManager(loc, backend, store, now=clock, start_thread=False,
                            sleep=sleep, odom_required=False, position_hold_s=1.)
    assert nav.start('B')['state'] == 'running'
    nav._finish('arrived')
    assert nav.status()['state'] == 'fault' and nav.status()['active']
    assert 'release' not in backend.calls


@pytest.mark.parametrize('degrade', ['pose_stale', 'slam_disconnected', 'frame_mismatch'])
def test_pose_only_start_rejects_bad_localization_even_without_odom(tmp_path, degrade):
    # odom_required=False tolerates only a missing twist (odom_stale/invalid_twist); a stale pose,
    # slam disconnect or frame mismatch still blocks start and never acquires.
    clock = Clock()
    loc = Localization('map', now=clock)
    store = PlaceStore(tmp_path/'points.json', 'lab-v1', 'map')
    store.save('B', dict(x=1., y=0., yaw=0., frame='map'))
    backend = Backend()
    nav = NavigationManager(loc, backend, store, now=clock, start_thread=False, odom_required=False)
    if degrade == 'pose_stale':
        feed_no_odom(loc, x=0.); clock.t += 2          # pose stale
    elif degrade == 'slam_disconnected':
        feed_no_odom(loc, x=0., state='disconnected')  # slam disconnected
    else:
        feed_no_odom(loc, x=0., frame='odom')          # pose frame does not match config
    assert not nav._nav_ok(loc.snapshot())
    out = nav.start('B')
    assert not out['ok']
    assert backend.calls == []


# ── Receipt-time freshness (off by default, onboard only): freshness judged by receive time, immune to the
# SLAM clock lagging ~181s / drifting; source_stamp is still used against replay, so post-stop checks stay intact. ──

def _stamped(loc, wall, *, source_lag=0., x=0., stamp=None):
    """Feed pose+odom+state with real header stamps. source = wall()-source_lag (or an explicit stamp)."""
    src = wall()-source_lag if stamp is None else stamp
    hdr = NS(frame_id='map', stamp=NS(sec=int(src), nanosec=round((src-int(src))*1e9)))
    loc.on_pose(NS(header=hdr, pose=NS(position=NS(x=x, y=0.),
                orientation=NS(x=0., y=0., z=0., w=1.))))
    loc.on_odom(NS(header=hdr, twist=NS(twist=NS(linear=NS(x=0., y=0.), angular=NS(z=0.)))))
    loc.on_state(NS(data='connected'))


def test_receipt_time_freshness_ignores_slam_clock_offset():
    # SLAM clock 181s behind: receipt-time mode sees fresh data; source-clock mode marks every frame stale.
    clock, wall = Clock(), lambda: 1000.
    recv = Localization('map', now=clock, strict_timestamps=True,
                        receipt_time_freshness=True, wall_now=wall)
    _stamped(recv, wall, source_lag=181.)                 # source stamp 181s behind
    snap = recv.snapshot()
    assert snap['ok'] and 'pose_stale' not in snap['errors']
    assert snap['pose_age_s'] is not None and snap['pose_age_s'] <= recv.max_age_s
    # Control: source-clock mode (default) goes stale on the 181s lag.
    strict = Localization('map', now=clock, strict_timestamps=True, wall_now=wall)
    _stamped(strict, wall, source_lag=181.)
    assert 'pose_stale' in strict.snapshot()['errors']


@pytest.mark.parametrize('bad,expect_stale', [
    ('duplicate', True), ('old', True), ('missing', True), ('future', False)])
def test_receipt_time_freshness_still_rejects_replayed_feedback(bad, expect_stale):
    # Receipt-time mode still dedupes by source_stamp: duplicate/older/missing stamps are rejected (pose_stale);
    # only a strictly newer stamp (future = genuinely new measurement) is accepted. This is the only intended difference from source-clock mode.
    clock, wall = Clock(), lambda: 1000.
    loc = Localization('map', now=clock, strict_timestamps=True,
                       receipt_time_freshness=True, wall_now=wall)
    good = wall()-181.
    _stamped(loc, wall, stamp=good)                       # first frame
    assert 'pose_stale' not in loc.snapshot()['errors']
    replay = {'duplicate': good, 'old': good-10., 'future': good+5., 'missing': 0.}[bad]
    _stamped(loc, wall, stamp=replay)                     # second frame
    assert ('pose_stale' in loc.snapshot()['errors']) is expect_stale


def test_state_max_age_tolerates_1hz_slam_state_topic():
    # Measured: the slamware state topic runs at ~1Hz (~1.005s period), pose at 10Hz. With a shared 1.0s limit,
    # tens of ms per second read slam_state_stale and abort navigation. A separate limit fixes the 1Hz false positives.
    clock = Clock()
    shared = Localization('map', now=clock)
    split = Localization('map', now=clock, state_max_age_s=3.)
    for loc in (shared, split):
        feed(loc)
    clock.t += 1.02                                         # next state message not yet arrived
    for loc in (shared, split):
        loc.on_pose(NS(header=NS(frame_id='map'), pose=NS(position=NS(x=0., y=0.),
            orientation=NS(x=0., y=0., z=0., w=1.))))
        loc.on_odom(NS(twist=NS(twist=NS(linear=NS(x=0., y=0.), angular=NS(z=0.)))))
    assert 'slam_state_stale' in shared.snapshot()['errors']
    assert split.snapshot()['ok']


def test_state_max_age_still_catches_dead_state_stream_and_keeps_pose_limit():
    clock = Clock()
    loc = Localization('map', now=clock, state_max_age_s=3.)
    feed(loc)
    clock.t += 1.5                                          # pose still uses the 1.0s limit
    assert set(loc.snapshot()['errors']) >= {'pose_stale', 'odom_stale'}
    assert 'slam_state_stale' not in loc.snapshot()['errors']
    clock.t += 2.                                           # state stream dead for 3.5s: still rejected
    assert 'slam_state_stale' in loc.snapshot()['errors']


@pytest.mark.parametrize('bad', [0., -1., float('nan'), float('inf')])
def test_state_max_age_rejects_invalid(bad):
    with pytest.raises(ValueError):
        Localization('map', state_max_age_s=bad)
