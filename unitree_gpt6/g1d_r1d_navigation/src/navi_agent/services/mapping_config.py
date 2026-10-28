# -*- coding: utf-8 -*-
"""Typed validation for the navi.navigation.mapping section.

Only the read-only "existing onboard SLAMWARE map" path is supported: map creation and
saving belong to the robot's own SLAM; this project only reads map health / localization
quality and submits goals to the onboard planner.
"""
from __future__ import annotations

from dataclasses import dataclass


class MappingError(ValueError):
    pass


@dataclass(frozen=True)
class MappingConfig:
    mode: str = 'existing'
    backend: str = 'disabled'      # disabled = no map-health gate; slamware_ros = subscribe to onboard map/state topics
    min_quality: int = 20          # localization quality gate (0..100)
    trust_slam_state: bool = False  # True = slam_state=connected overrides the numeric quality gate
    require_map_health: bool = True  # False = map/bounds/quality are advisory; onboard planner decides

    @classmethod
    def parse(cls, raw):
        if not isinstance(raw, dict):
            raise MappingError('navigation.mapping must be an object')
        try:
            cfg = cls(**raw)
        except TypeError as exc:
            raise MappingError('unknown navigation.mapping keys') from exc
        if cfg.mode != 'existing' or cfg.backend not in ('disabled', 'slamware_ros'):
            raise MappingError('mapping.mode must be existing; mapping.backend must be disabled or slamware_ros')
        if type(cfg.min_quality) is not int or not 1 <= cfg.min_quality <= 100:
            raise MappingError('mapping.min_quality must be 1..100')
        for key in ('trust_slam_state', 'require_map_health'):
            if type(getattr(cfg, key)) is not bool:
                raise MappingError(f'mapping.{key} must be boolean')
        return cfg
