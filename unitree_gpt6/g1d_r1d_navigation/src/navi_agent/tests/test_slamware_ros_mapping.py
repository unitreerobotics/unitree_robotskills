import math
from types import SimpleNamespace as NS

from services.slamware_ros_mapping import SlamwareRosMapping


def q(yaw=0):
    return NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2))


def stamp(sec=100):
    return NS(sec=sec, nanosec=0)


def header(frame='slamware_map'):
    return NS(frame_id=frame, stamp=stamp())


def msgs():
    origin = NS(position=NS(x=0., y=0., z=0.), orientation=q())
    info = NS(width=100, height=100, resolution=.1, origin=origin)
    grid = NS(header=header(), info=info, data=[0] * 10000)
    pose = NS(header=header('robot_pose'), pose=NS(position=NS(x=1., y=1., z=0.), orientation=q()))
    odom = NS(header=header('odom'), child_frame_id='base_link',
              twist=NS(twist=NS(linear=NS(x=0., y=0.), angular=NS(z=0.))))
    basic = NS(is_map_building_enabled=False, is_localization_enabled=True,
               localization_quality=80)
    return grid, pose, odom, basic


def test_ros_status_is_read_only_and_safe():
    clock = [100.]
    m = SlamwareRosMapping(frame='robot_pose', now=lambda: clock[0], wall_now=lambda: 100.)
    grid, pose, odom, basic = msgs()
    m._on_map(grid); m._on_basic(basic); m._on_tf(NS(transforms=[NS(header=header('slamware_map'), child_frame_id='robot_pose', transform=NS(translation=NS(x=0., y=0., z=0.), rotation=q()))])); m.localization.on_pose(pose)
    m.localization.on_odom(odom); m.localization.on_state(NS(data='connected'))
    out = m.status()
    assert out['ok'] and out['safety']['pose_in_map_bounds']
    assert out['quality'] == 80 and out['read_only'] and not out['lifecycle_writes']
    assert out['navigation_supported']
    assert 'device_verified' not in out and 'writes_supported' not in out


def test_ros_status_blocks_outside_map_and_low_quality():
    clock = [100.]
    m = SlamwareRosMapping(frame='robot_pose', now=lambda: clock[0], wall_now=lambda: 100.)
    grid, pose, odom, basic = msgs()
    pose.pose.position.x = 20.
    basic.localization_quality = 5
    m._on_map(grid); m._on_basic(basic); m._on_tf(NS(transforms=[NS(header=header('slamware_map'), child_frame_id='robot_pose', transform=NS(translation=NS(x=0., y=0., z=0.), rotation=q()))])); m.localization.on_pose(pose)
    m.localization.on_odom(odom); m.localization.on_state(NS(data='connected'))
    out = m.status()
    assert not out['ok']
    assert 'pose_outside_map_bounds' in out['errors']
    assert 'localization_quality_low_or_unknown' in out['errors']


def test_trust_slam_state_drops_numeric_quality_gate():
    # Onboard goto: trust slam_state=connected instead of the numeric localization-quality gate (quality=5 observed
    # in the field still passes), but still enforce slam_not_connected + lidar + map bounds; quality stays in the result for observation.
    clock = [100.]
    m = SlamwareRosMapping(frame='robot_pose', now=lambda: clock[0], wall_now=lambda: 100.,
                           trust_slam_state=True)
    grid, pose, odom, basic = msgs()
    basic.localization_quality = 5
    m._on_map(grid); m._on_basic(basic); m._on_tf(NS(transforms=[NS(header=header('slamware_map'), child_frame_id='robot_pose', transform=NS(translation=NS(x=0., y=0., z=0.), rotation=q()))])); m.localization.on_pose(pose)
    m.localization.on_odom(odom); m.localization.on_state(NS(data='connected'))
    out = m.status()
    assert out['ok'] and 'localization_quality_low_or_unknown' not in out['errors']
    assert out['quality'] == 5
    # Disconnected slam still blocks: we trust "connected", not blindly pass.
    m.localization.on_state(NS(data='disconnected'))
    blocked = m.status()
    assert not blocked['ok'] and 'slam_not_connected' in blocked['errors']


def test_odom_optional_does_not_block_on_missing_twist():
    # Pose-only onboard: without a measured twist the strict default blocks; odom_required=False allows it.
    grid, pose, _, basic = msgs()
    def build(**kw):
        m = SlamwareRosMapping(frame='robot_pose', now=lambda: 100., wall_now=lambda: 100., **kw)
        m._on_map(grid); m._on_basic(basic)
        m._on_tf(NS(transforms=[NS(header=header('slamware_map'), child_frame_id='robot_pose',
                  transform=NS(translation=NS(x=0., y=0., z=0.), rotation=q()))]))
        m.localization.on_pose(pose)                     # no odom: pose-only stream
        m.localization.on_state(NS(data='connected'))
        return m.status()
    strict = build()
    assert not strict['ok'] and 'odom_stale' in strict['errors']
    relaxed = build(odom_required=False)
    assert relaxed['ok'] and 'odom_stale' not in relaxed['errors']


def test_goal_without_frame_key_is_rejected_but_stamped_frame_passes():
    # PlaceStore.get() returns goal dicts without a 'frame' key, so in_bounds fails closed and status adds
    # goal_outside_map_bounds_or_frame_unverified (which is why the guard layer must stamp the frame).
    # An in-bounds goal that carries the pose frame passes; that is the path taken once the guard stamps it.
    clock = [100.]
    m = SlamwareRosMapping(frame='robot_pose', now=lambda: clock[0], wall_now=lambda: 100.)
    grid, pose, odom, basic = msgs()
    m._on_map(grid); m._on_basic(basic)
    m._on_tf(NS(transforms=[NS(header=header('slamware_map'), child_frame_id='robot_pose',
              transform=NS(translation=NS(x=0., y=0., z=0.), rotation=q()))]))
    m.localization.on_pose(pose); m.localization.on_odom(odom)
    m.localization.on_state(NS(data='connected'))
    frameless = m.status(dict(x=1., y=1., yaw=0.))            # mimics store.get: no frame
    assert 'goal_outside_map_bounds_or_frame_unverified' in frameless['errors']
    stamped = m.status(dict(x=1., y=1., yaw=0., frame='robot_pose'))
    assert 'goal_outside_map_bounds_or_frame_unverified' not in stamped['errors']


def test_require_map_health_false_drops_map_errors():
    # Onboard goto itself ignores map/in-map/quality/localization-enabled/map-building. require_map_health=False
    # demotes these map-derived gates to advisory: missing map, out-of-bounds pose, low quality or odd localization
    # flags no longer block, leaving planning to onboard SLAMWARE; only slam connected + fresh pose + lidar + distance cap
    # are enforced. Quality/map remain visible in the result.
    clock = [100.]
    m = SlamwareRosMapping(frame='robot_pose', now=lambda: clock[0], wall_now=lambda: 100.,
                           require_map_health=False)
    grid, pose, odom, basic = msgs()
    pose.pose.position.x = 20.                     # out of bounds
    basic.localization_quality = 5                 # low quality
    basic.is_localization_enabled = False          # localization disabled
    basic.is_map_building_enabled = True           # map building
    m._on_map(grid); m._on_basic(basic)
    m._on_tf(NS(transforms=[NS(header=header('slamware_map'), child_frame_id='robot_pose',
              transform=NS(translation=NS(x=0., y=0., z=0.), rotation=q()))]))
    m.localization.on_pose(pose); m.localization.on_odom(odom)
    m.localization.on_state(NS(data='connected'))
    out = m.status()
    assert out['ok']
    for e in ('map_missing_or_invalid', 'pose_outside_map_bounds', 'pose_map_bounds_unverified',
              'localization_quality_low_or_unknown', 'localization_disabled_or_unknown',
              'existing_map_is_updating_recalibrate_after_mapping'):
        assert e not in out['errors']
    # Advisory fields are still returned (non-blocking).
    assert out['quality'] == 5 and out['map'] is not None
    # Out-of-bounds goals are no longer blocked by the map gate (onboard goto has no bounds gate).
    assert m.status(dict(x=99., y=99., yaw=0., frame='robot_pose'))['ok']
    # The strict default instance still blocks the same input (the relaxation does not leak into the default path).
    strict = SlamwareRosMapping(frame='robot_pose', now=lambda: clock[0], wall_now=lambda: 100.)
    strict._on_map(grid); strict._on_basic(basic)
    strict._on_tf(NS(transforms=[NS(header=header('slamware_map'), child_frame_id='robot_pose',
                  transform=NS(translation=NS(x=0., y=0., z=0.), rotation=q()))]))
    strict.localization.on_pose(pose); strict.localization.on_odom(odom)
    strict.localization.on_state(NS(data='connected'))
    assert not strict.status()['ok']


def test_require_map_health_false_still_blocks_slam_and_pose():
    # require_map_health=False only demotes map-derived gates; slam disconnect, stale pose and frame mismatch still block.
    clock = [100.]
    def build():
        m = SlamwareRosMapping(frame='robot_pose', now=lambda: clock[0], wall_now=lambda: 100.,
                               require_map_health=False, odom_required=False)
        grid, pose, odom, basic = msgs()
        return m, grid, pose, odom, basic
    # slam disconnected
    m, grid, pose, odom, basic = build()
    m._on_map(grid); m._on_basic(basic); m.localization.on_pose(pose)
    m.localization.on_state(NS(data='disconnected'))
    out = m.status()
    assert not out['ok'] and 'slam_not_connected' in out['errors']
    # stale pose
    clock[0] = 100.
    m, grid, pose, odom, basic = build()
    m._on_map(grid); m._on_basic(basic); m.localization.on_pose(pose)
    m.localization.on_state(NS(data='connected'))
    clock[0] += 2.                                 # now past max_age_s
    out = m.status()
    assert not out['ok'] and 'pose_stale' in out['errors']
    # frame mismatch
    clock[0] = 100.
    m, grid, pose, odom, basic = build()
    pose.header = header('odom')                   # pose frame != configured frame
    m._on_map(grid); m._on_basic(basic); m.localization.on_pose(pose)
    m.localization.on_state(NS(data='connected'))
    out = m.status()
    assert not out['ok'] and 'frame_mismatch' in out['errors']


def test_relaxed_status_labels_quality_and_map_fields_advisory_only():
    """Seen on the real robot: errors=[] but safety.quality_ok=false made the model refuse to navigate; when relaxed it must be clearly marked advisory."""
    def build(**kw):
        m = SlamwareRosMapping(frame='robot_pose', now=lambda: 100., wall_now=lambda: 100., **kw)
        grid, pose, odom, basic = msgs()
        basic.localization_quality = 0
        m._on_map(grid); m._on_basic(basic); m._on_tf(NS(transforms=[NS(header=header('slamware_map'), child_frame_id='robot_pose', transform=NS(translation=NS(x=0., y=0., z=0.), rotation=q()))])); m.localization.on_pose(pose)
        m.localization.on_odom(odom); m.localization.on_state(NS(data='connected'))
        return m.status()
    relaxed = build(trust_slam_state=True)
    assert relaxed['ok'] and relaxed['advisory_only'] == ['localization_quality_low_or_unknown']
    assert relaxed['gate_note'] and relaxed['safety']['quality_ok'] is False
    strict = build()
    assert not strict['ok'] and 'advisory_only' not in strict and 'gate_note' not in strict
