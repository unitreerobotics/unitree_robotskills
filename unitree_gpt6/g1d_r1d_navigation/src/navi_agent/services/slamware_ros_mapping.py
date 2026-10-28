"""Native, read-only SLAMWARE ROS2/DDS map health for existing-map navigation.

No REST, DataWriter, controller, service calls, guessed identity or map reset.
Direct planar TF only; unsupported/missing transforms fail closed. Receipt and
source freshness are separate; headerless basic/state topics use receipt age.
"""
from __future__ import annotations

import math
import threading
import time

from services.navigation import Localization, SlamwareBackend, source_stamp


class SlamwareRosMapping:
    def __init__(self, *, frame='robot_pose', max_age_s=1., min_quality=20,
                 source_offset_s=0., now=time.monotonic, wall_now=time.time,
                 trust_slam_state=False, odom_required=True, require_map_health=True,
                 state_max_age_s=None,
                 receipt_time_freshness=False):
        self.now, self.wall_now = now, wall_now
        self.source_offset_s = source_offset_s
        self.min_quality = min_quality
        self.trust_slam_state = trust_slam_state
        self.odom_required = odom_required
        self.require_map_health = require_map_health
        self.localization = Localization(frame, max_age_s, now, strict_timestamps=True,
                                         source_offset_s=source_offset_s, wall_now=wall_now,
                                         receipt_time_freshness=receipt_time_freshness,
                                         state_max_age_s=state_max_age_s)
        self._lock = threading.RLock()
        self._start_lock = threading.Lock()
        self._samples, self._sources, self._transforms = {}, {}, {}
        self._subs = []
        self.map_max_age_s = 3.  # map/basic state are slower than pose/odom

    @staticmethod
    def _planar(q):
        values = (q.x, q.y, q.z, q.w)
        if (not all(math.isfinite(v) for v in values) or
                abs(sum(v*v for v in values)-1.) > .01 or abs(q.x) > .001 or abs(q.y) > .001):
            raise ValueError('non_planar_or_invalid_rotation')
        return math.atan2(2*q.w*q.z, 1-2*q.z*q.z)

    def _put(self, key, value, msg=None):
        with self._lock:
            stamp, received = source_stamp(msg), self.now()
            if msg is not None:
                delay = None if stamp is None else self.wall_now()-(stamp+self.source_offset_s)
                previous = self._sources.get(key)
                if (delay is None or not 0 <= delay <= self.map_max_age_s or
                        (previous is not None and stamp <= previous)):
                    return  # duplicate/replay does not refresh the sample
                self._sources[key] = stamp
                received -= delay
            self._samples[key] = (value, received)

    def _on_map(self, msg):
        try:
            m, p = msg.info, msg.info.origin.position
            yaw = self._planar(m.origin.orientation)
            if (not all(math.isfinite(v) for v in (m.resolution, p.x, p.y, p.z)) or
                    m.resolution <= 0 or m.width <= 0 or m.height <= 0 or
                    len(msg.data) != m.width*m.height or not msg.header.frame_id):
                raise ValueError('invalid_map')
            value = dict(frame=msg.header.frame_id, width=m.width, height=m.height,
                         resolution=m.resolution, origin=dict(x=p.x, y=p.y, yaw=yaw))
        except (ValueError, AttributeError, TypeError):
            value = None
        self._put('map', value, msg)

    def _on_basic(self, msg):
        self._put('basic', dict(mapping_enabled=msg.is_map_building_enabled,
                               localization_enabled=msg.is_localization_enabled,
                               quality=msg.localization_quality))

    def _on_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id != self.localization.frame:
                continue
            try:
                p = t.transform.translation
                yaw = self._planar(t.transform.rotation)
                if not all(math.isfinite(v) for v in (p.x, p.y, p.z)) or abs(p.z) > .01:
                    raise ValueError('invalid_tf')
                value = dict(x=p.x, y=p.y, yaw=yaw)
            except (ValueError, AttributeError, TypeError):
                value = None
            self._put(('tf', t.header.frame_id, t.child_frame_id), value, t)

    def start(self):
        """Shared DDS factory must already exist. Creates readers only, atomically."""
        with self._start_lock:
            if self._subs:
                return self
            from unitree_sdk2py.core.channel import ChannelSubscriber

            from services.slamware_idl import OccupancyGrid_, RobotBasicState_, TFMessage_
            subs = []
            try:
                subs.extend(SlamwareBackend.subscribe(self.localization))
                for topic, typ, cb in (
                    ('rt/slamware_ros_sdk_server_node/map', OccupancyGrid_, self._on_map),
                    ('rt/slamware_ros_sdk_server_node/robot_basic_state', RobotBasicState_, self._on_basic),
                    ('rt/tf', TFMessage_, self._on_tf),
                ):
                    sub = ChannelSubscriber(topic, typ)
                    subs.append(sub)
                    sub.Init(cb, 1)
                self._subs = subs
            except Exception:
                for sub in subs:
                    sub.Close()
                raise
        return self

    def close(self):
        with self._start_lock:
            for sub in self._subs:
                sub.Close()
            self._subs = []
            with self._lock:
                self._samples.clear()
                self._sources.clear()

    def status(self, goal=None):
        loc = self.localization.snapshot()
        errors = list(loc['errors'])
        # Pose-only onboard mode: a missing measured twist is informational,
        # not a blocker. slam_state connected + fresh pose + lidar + bounds still gate.
        if not self.odom_required:
            errors = [e for e in errors if e not in ('odom_stale', 'invalid_twist')]
        # Map/bounds/quality/localization-flag errors accumulate separately. Onboard goto
        # itself does not depend on any of them; require_map_health
        # =False drops them to advisory (still computed & returned below for visibility) and
        # trusts the onboard SLAMWARE's own localization+planning. slam_not_connected/pose_stale/
        # invalid_pose/frame_mismatch stay hard (already copied into errors above).
        map_errors = []
        ages = {}
        with self._lock:
            samples = dict(self._samples)
        def sample(key, label):
            value, ts = samples.get(key, (None, None))
            age = None if ts is None else self.now()-ts
            ages[label] = None if age is None else round(age, 3)
            if value is None:
                map_errors.append(label+'_missing_or_invalid')
                return None
            if not 0 <= age <= self.map_max_age_s:
                map_errors.append(label+'_stale')
            return value
        grid, basic = sample('map', 'map'), sample('basic', 'basic_state')
        quality_ok = bool(basic and type(basic['quality']) is int and self.min_quality <= basic['quality'] <= 100)
        # trust_slam_state (onboard goto) drops the numeric quality gate; it still
        # relies on slam_not_connected + lidar + map-bounds. Quality stays visible below.
        if not quality_ok and not self.trust_slam_state:
            map_errors.append('localization_quality_low_or_unknown')
        if not basic or basic['localization_enabled'] is not True:
            map_errors.append('localization_disabled_or_unknown')
        if basic and basic['mapping_enabled'] is not False:
            map_errors.append('existing_map_is_updating_recalibrate_after_mapping')
        transform = dict(x=0., y=0., yaw=0.)
        if grid and grid['frame'] != self.localization.frame:
            transform = sample(('tf', grid['frame'], self.localization.frame), 'map_to_pose_tf')
        def in_bounds(pose):
            if not grid or transform is None or not pose:
                return None
            if pose.get('frame') != self.localization.frame:
                return None
            if any(type(pose.get(k)) not in (int, float) or not math.isfinite(pose[k]) for k in ('x', 'y')):
                return None
            c, s = math.cos(transform['yaw']), math.sin(transform['yaw'])
            x = transform['x'] + c*pose['x'] - s*pose['y'] - grid['origin']['x']
            y = transform['y'] + s*pose['x'] + c*pose['y'] - grid['origin']['y']
            c, s = math.cos(grid['origin']['yaw']), math.sin(grid['origin']['yaw'])
            u, v = c*x+s*y, -s*x+c*y
            return 0 <= u < grid['width']*grid['resolution'] and 0 <= v < grid['height']*grid['resolution']
        pose_ok = in_bounds(loc['pose'])
        if pose_ok is not True:
            map_errors.append('pose_outside_map_bounds' if pose_ok is False else 'pose_map_bounds_unverified')
        if goal is not None and in_bounds(goal) is not True:
            map_errors.append('goal_outside_map_bounds_or_frame_unverified')
        if self.require_map_health:
            errors.extend(map_errors)
        # Preflight once passed with errors=[], yet the model refused navigate_to because
        # safety.quality_ok=false (onboard quality reads 0 while stationary in mapping mode).
        # When relaxed, label unenforced items as advisory so they don't look like hard gates.
        advisory = ([] if self.require_map_health else list(map_errors)) + (
            ['localization_quality_low_or_unknown'] if not quality_ok and self.trust_slam_state else [])
        extra = dict(advisory_only=list(dict.fromkeys(advisory)),
                     gate_note='advisory_only 与 safety 里的质量/图内项仅供观察，不是导航门；'
                               '能否导航以 ok/errors 与 navigation_preflight 为准') if advisory else {}
        return dict(ok=not errors, read_only=True, motion_started=False, backend='slamware_ros',
                    state=loc['slam_state'], map=grid, pose=loc['pose'], localization=loc,
                    **(basic or dict(quality=None, mapping_enabled=None, localization_enabled=None)),
                    age_s=ages, errors=errors,
                    safety=dict(quality_ok=quality_ok, pose_in_map_bounds=pose_ok), **extra,
                    backend_mode='native_ros_dds', lifecycle_writes=False,
                    navigation_supported=True, device_identity='ros_participant_unverified',
                    map_identity='operator_managed_namespace')
