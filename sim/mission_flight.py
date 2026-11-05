"""WedgeBug mission flight for run_sim.py --mission: take off, navigate, hold.

An alternative to flight_demo.py's fixed hover/move demo, with the same
target(state) interface. The navigator is the C core in
apps/wedgebug/src/wedgebug.c (through cffirmware) unless backend="python".
It runs at 50 Hz on what the flight controller sees (ground truth in
firmware_pid/ekf_shadow, the EKF estimate in ekf_closed_loop) and reads the
held 10 Hz Multi-ranger values from ranger.py (state["ranger"], filled by
run_sim.py) every tick, as an app calling rangeGet() would. Its output is a
hover setpoint for the firmware PID: body-frame vx/vy, yaw rate, and the
takeoff height.
"""

import math
import json
from dataclasses import asdict
from pathlib import Path

import numpy as np

from controllers.firmware_pid import rotation_and_rpy
from navigation.firmware_nav import make_navigator
from navigation.geometry import obstacle_distance, validate_waypoints
from navigation.wedgebug import MODES, PHASES

NAV_PERIOD_S = 0.02          # navigator tick, as p.dt in the MATLAB/2-D simulator
TAKEOFF_POSITION_TOL_M = 0.05
TAKEOFF_SPEED_TOL_M_S = 0.05
TAKEOFF_HOLD_S = 1.0         # settled this long at the start before navigating
TAKEOFF_TIMEOUT_S = 20.0
NAV_RANGER_DIRECTIONS = ("front", "left", "back", "right")  # navigator order; "up" is unused


def deck_ranges(fields):
    """[front, left, back, right] in metres from ranger.py's held telemetry; inf = no valid return."""
    return np.array([fields[f"range.{name}"] / 1000 if fields[f"range_{name}_valid"] else math.inf
                     for name in NAV_RANGER_DIRECTIONS])


class WedgeBugMission:
    def __init__(self, mission, p, target_z_m, backend="firmware", use_estimate=False, safety_config=None):
        self.mission = mission
        self.p = p
        self.target_z_m = target_z_m
        self.backend = backend
        self.use_estimate = use_estimate
        self.safety_config = safety_config
        self.safety_faults = {}  # smoke-test injection, navigation-relative [start,end) windows
        self.env = mission.environment()
        validate_waypoints(mission.start, mission.targets, self.env, p.nav.d_wall)
        self.time_limit_s = TAKEOFF_TIMEOUT_S + p.t_max
        self.reset()

    def reset(self):
        m = self.mission
        self.nav = make_navigator(self.backend, m.start, m.targets, self.p)
        self.safety = self.safety_transport = None
        if self.safety_config is not None:
            from safety import SafetySupervisor, EstimatedRangeTransport, RangeFrame
            self.nav.prepare_safety()
            self.safety = SafetySupervisor(self.safety_config)
            self.safety_transport = EstimatedRangeTransport()
            self.safety_frame = RangeFrame.unavailable()
        self.phase = "takeoff"
        self.done = False
        self.estimate = None
        self.stable_since = None
        self.nav_start_s = None
        self.next_nav_s = 0.0
        self.ranges = np.full(4, math.inf)
        self.u_body, self.yaw_rate = np.zeros(2), 0.0
        self.nav_x, self.nav_psi = np.full(4, math.nan), math.nan
        self.hold_xy, self.hold_yaw = None, 0.0
        self.min_clearance = math.inf
        self.t_finish = math.nan

    def observe_estimate(self, estimate):
        """Latest EKF output (command['estimate']), used by the next tick in ekf_closed_loop."""
        self.estimate = estimate

    def _nav_state(self, state):
        """[x, y, vx, vy], yaw and z as the flight controller sees them."""
        source = self.estimate if self.use_estimate and self.estimate is not None else state
        position = np.asarray(source["position_m"], dtype=float)
        velocity = np.asarray(source["linear_velocity_m_s"], dtype=float)
        _, rpy = rotation_and_rpy(source["orientation_wxyz"])
        return np.array([position[0], position[1], velocity[0], velocity[1]]), rpy[2], position[2], velocity[2]

    def _fault(self, name, now):
        return any(start <= now < end for start, end in self.safety_faults.get(name, ()))

    def _safety_tick(self, observation, state, now, mission_hold=False):
        nav_time = now-self.nav_start_s
        if not self._fault("ranger_dropout", nav_time):
            self.safety_frame = self.safety_transport.frame(state["ranger"])
        return self.safety.step(observation, self.safety_frame, now, self.nav, nav_time,
            primary_available=not self._fault("primary_timeout", nav_time),
            navigator_available=not self._fault("navigator_timeout", nav_time),
            mission_hold=mission_hold, hold_target=self.hold_xy if mission_hold else None)

    def target(self, state):
        now = float(state["time_s"])
        x, psi, z, vz = self._nav_state(state)
        observation = None
        if self.safety is not None:
            # In closed loop this is the same ESTIMATE the PID sees, not PhysX truth.
            source = self.estimate if self.use_estimate and self.estimate is not None else state
            observation = self.safety_transport.observe(now, source)
        start = self.mission.start

        if self.phase == "takeoff":
            error = math.hypot(math.hypot(x[0] - start[0], x[1] - start[1]), z - self.target_z_m)
            speed = math.hypot(np.linalg.norm(x[2:4]), vz)
            if error < TAKEOFF_POSITION_TOL_M and speed < TAKEOFF_SPEED_TOL_M_S:
                self.stable_since = now if self.stable_since is None else self.stable_since
                if now - self.stable_since >= TAKEOFF_HOLD_S:
                    self.phase = "navigate"
                    self.nav_start_s = self.next_nav_s = now
            else:
                self.stable_since = None
                if now > TAKEOFF_TIMEOUT_S:
                    raise RuntimeError(f"Did not settle at the mission start within {TAKEOFF_TIMEOUT_S:g} s")
            if self.phase == "takeoff":
                return {"position_m": (start[0], start[1], self.target_z_m), "yaw_rad": self.p.psi0}

        if self.phase == "navigate":
            if now >= self.next_nav_s - 1e-9:
                t_nav = now - self.nav_start_s
                # The deck measures from the TRUE pose, whatever the estimate says.
                if self.safety is None:
                    self.ranges = deck_ranges(state["ranger"])
                    self.u_body, self.yaw_rate = self.nav.step(x, psi, self.ranges, t_nav)
                else:
                    self.u_body, self.yaw_rate = self._safety_tick(observation, state, now)
                    self.ranges = self.safety.processor.ranges.copy()
                self.nav_x, self.nav_psi = x, psi
                self.next_nav_s += NAV_PERIOD_S
                true_xy = np.asarray(state["position_m"][:2], dtype=float)
                self.min_clearance = min(self.min_clearance,
                                         float(obstacle_distance(self.env, true_xy)[0]) - self.p.radius)
                if (self.nav.mode == "DONE" and np.linalg.norm(x[2:4]) < self.p.goal_vtol
                        and (self.safety is None or self.safety.accepted)):
                    self.phase, self.done = "hold", True
                    self.hold_xy, self.hold_yaw = x[0:2].copy(), psi
                    self.t_finish = t_nav
            if self.phase == "navigate":
                goal = self.nav.target()
                return {"position_m": (goal[0], goal[1], self.target_z_m),
                        "velocity_body_m_s": self.u_body, "yaw_rate_rad_s": self.yaw_rate}

        if self.safety is not None:
            if now >= self.next_nav_s-1e-9:
                self.u_body, self.yaw_rate = self._safety_tick(observation, state, now, mission_hold=True)
                self.next_nav_s += NAV_PERIOD_S
            return {"position_m": (self.hold_xy[0], self.hold_xy[1], self.target_z_m),
                    "velocity_body_m_s": self.u_body, "yaw_rate_rad_s": self.yaw_rate}
        return {"position_m": (self.hold_xy[0], self.hold_xy[1], self.target_z_m), "yaw_rad": self.hold_yaw}

    def log_fields(self):
        """Extra CSV columns, identical keys every row."""
        navigating = self.phase != "takeoff"
        fields = {
            "nav_mode": MODES.index(self.nav.mode) + 1 if navigating else 0,
            "nav_phase": PHASES.index(self.nav.phase) + 1 if navigating and self.nav.phase else 0,
            "nav_wp": self.nav.wp,
            "nav_x_m": self.nav_x[0], "nav_y_m": self.nav_x[1], "nav_yaw_rad": self.nav_psi,
            "range_front_m": self.ranges[0], "range_left_m": self.ranges[1],
            "range_back_m": self.ranges[2], "range_right_m": self.ranges[3],
            "cmd_forward_m_s": self.u_body[0], "cmd_left_m_s": self.u_body[1],
            "cmd_yaw_rate_rad_s": self.yaw_rate,
        }
        if self.safety is not None:
            fields.update(self.safety.log_fields())
            fields["safety_active"] = int(navigating)
            fields["safety_frame_seq"] = self.safety_frame.seq
        return fields

    def summary(self):
        names = [f"waypoint {i}" for i in range(1, len(self.mission.targets))] + ["goal"]
        reached = sum(e["type"] == "reached" for e in self.nav.events)
        result = {
            "mission": self.mission.name, "navigator": self.backend, "phase": self.phase,
            "arrived": self.done and len(self.mission.targets) - 1 not in self.nav.unreachable,
            "t_finish_s": self.t_finish, "targets_reached": reached, "n_targets": len(self.mission.targets),
            "unreachable": [names[i] for i in self.nav.unreachable],
            "sweeps": len(self.nav.sweeps), "avoid_episodes": sum(e["type"] == "hit" for e in self.nav.events),
            "min_clearance_m": self.min_clearance, "collided": self.min_clearance < 0,
        }
        if self.safety is not None:
            brakes = [e for e in self.safety.events if e["type"] == "brake"]
            result.update(safety_enabled=True, safety_brakes=len(brakes),
                          backup_brakes=sum(e["source"] == "backup" for e in brakes),
                          safety_last_error=self.safety.last_error)
        return result

    def write_safety_report(self, path):
        if self.safety is None:
            return
        report = {"summary": self.summary(), "config": asdict(self.safety_config),
                  "events": self.safety.events,
                  "braking_bounds_calibrated": False,
                  "scope": "horizontal navigation/hold only; PID altitude and attitude remain active"}
        def clean(value):
            if isinstance(value, dict):
                return {k: clean(v) for k, v in value.items()}
            if isinstance(value, (tuple, list)):
                return [clean(v) for v in value]
            if isinstance(value, float) and not math.isfinite(value):
                return None
            return value
        Path(path).write_text(json.dumps(clean(report), indent=2, allow_nan=False)+"\n")
