"""Scheduled world change for recording tests; never feeds geometry to Safety."""
from dataclasses import replace
import math

import numpy as np

from navigation.geometry import obstacle_distance


class PopupExperiment:
    def __init__(self, mission, set_enabled, at_seconds=3.):
        if len(mission.obstacles) != 1 or not math.isfinite(at_seconds) or at_seconds < 0:
            raise ValueError('Popup requires one obstacle and a nonnegative navigation-relative time')
        self.mission = mission
        self.empty_environment = replace(mission, obstacles=()).environment()
        self.active_environment = mission.environment()
        self.set_enabled = set_enabled
        self.at_seconds = at_seconds
        self.event = None
        self.first_detection = None

    def prepare(self, mission_run):
        self.set_enabled(False)
        mission_run.env = self.empty_environment

    def update(self, state, mission_run):
        start = mission_run.nav_start_s
        if self.event is not None or start is None or state['time_s']-start < self.at_seconds-1e-9:
            return
        xy = np.asarray(state['position_m'][:2])
        gap = float(obstacle_distance(self.active_environment, xy)[0])-.07
        # Never create a collider overlapping the vehicle. Record/test the
        # clearance and speed rather than assuming every schedule is feasible.
        if gap <= 0:
            raise RuntimeError('Popup would overlap the vehicle; refuse this schedule')
        self.set_enabled(True)
        mission_run.env = self.active_environment  # Collision metrics only.
        self.event = dict(time_s=float(state['time_s']), navigation_time_s=float(state['time_s']-start),
                          position_m=list(map(float,state['position_m'])),
                          speed_m_s=float(np.linalg.norm(state['linear_velocity_m_s'][:2])),
                          clearance_at_appearance_m=gap)
        print(f'POPUP_APPEARED {self.event}', flush=True)

    def observe(self, readings):
        suffix = '/'+self.mission.obstacles[0]['name']
        if self.event is None:
            if any(suffix in row.get('hit_prim','') for row in readings):
                raise RuntimeError('Inactive popup was visible to Ranger; collider disable did not take effect')
            return
        if self.first_detection is not None:
            return
        for row in readings:
            if row['sample_time_s'] >= self.event['time_s'] and suffix in row.get('hit_prim',''):
                self.first_detection = {k:row[k] for k in
                    ('sample_time_s','delivery_time_s','direction','hit_prim','range_mm')}
                print(f'POPUP_DETECTED {self.first_detection}', flush=True)
                break

    def report(self):
        return dict(scheduled_navigation_time_s=self.at_seconds, appearance=self.event,
                    first_detection=self.first_detection)
