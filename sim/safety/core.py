"""Range supervision independent of Kit, PhysX, cffirmware, and scene geometry.

Ports the MATLAB Primary/Backup/arbiter/fallback architecture. Its braking
envelope is configurable for PID/PhysX, NOT MATLAB's first-order dynamics.
Defaults are assumptions awaiting the supplied live braking validation.
"""

from collections import deque
from dataclasses import dataclass, field, replace
import math
import numpy as np

from navigation.geometry import rot2d, sat_norm

AXES_BODY = np.array([[1., 0., 0.], [0., 1., 0.], [-1., 0., 0.], [0., -1., 0.]])
HIT, NO_RETURN, INVALID, UNAVAILABLE = 1, 2, 3, 0


@dataclass(frozen=True)
class SafetyConfig:
    dt: float = .02
    radius: float = .07
    range_max: float = 4.
    half_fov: float = math.radians(13.5)
    offsets: tuple = (.01625, .0163, .01625, .0163)
    noise_std: float = 0.
    margin: float = .04
    noise_sigma: float = 2.
    backup_margin: float = .015
    backup_noise_sigma: float = 1.
    hard_gap: float = .01
    max_age: float = .35
    heartbeat_timeout: float = .08
    approach_speed: float = .02
    stop_speed: float = .03
    clear_frames: int = 3
    filter_window: int = 3
    filter_rotation_tol: float = math.radians(3)
    recover_speed: float = .04
    recover_time: float = 2.
    cruise_speed: float = .5
    max_speed: float = 1.
    hold_gain: float = 1.
    hold_speed: float = .1
    # Assumed bounds; identify them with the stock PID at the flight height.
    brake_accel: float = .5
    brake_response: float = .2
    backup_brake_accel: float = .5
    backup_brake_response: float = .25
    ranger_period: float = .1
    ranger_latency: float = .1
    max_tilt: float = math.radians(20)

    def __post_init__(self):
        positive = (self.dt, self.radius, self.range_max, self.max_age,
                    self.heartbeat_timeout, self.stop_speed, self.recover_time,
                    self.max_speed, self.brake_accel, self.backup_brake_accel)
        if not all(math.isfinite(v) and v > 0 for v in positive):
            raise ValueError("Safety time, geometry and braking bounds must be finite and positive")
        for name in ("noise_std", "margin", "noise_sigma", "backup_margin", "backup_noise_sigma",
                     "hard_gap", "approach_speed", "recover_speed", "cruise_speed", "hold_gain",
                     "hold_speed", "brake_response", "backup_brake_response", "ranger_period", "ranger_latency"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if not 0 <= self.half_fov < math.pi/2 or not 0 < self.max_tilt < math.pi/2:
            raise ValueError("Invalid cone or tilt limit")
        if self.clear_frames < 1 or self.filter_window < 1:
            raise ValueError("Frame counts must be positive")

    def braking_distance(self, speed):
        """Response travel followed by bounded deceleration; uncalibrated default."""
        return speed*self.brake_response + speed*speed/(2*self.brake_accel)


@dataclass
class Observation:
    position: np.ndarray
    velocity: np.ndarray
    rotation: np.ndarray
    yaw: float

    def finite(self):
        return (np.shape(self.position) == (3,) and np.shape(self.velocity) == (3,)
                and np.shape(self.rotation) == (3, 3)
                and np.isfinite(np.r_[self.position, self.velocity, self.rotation.ravel(), self.yaw]).all()
                and np.allclose(self.rotation.T@self.rotation, np.eye(3), atol=1e-5)
                and abs(np.linalg.det(self.rotation)-1) < 1e-5)


@dataclass
class RangeFrame:
    seq: int
    sample_time: float
    delivery_time: float
    ranges: np.ndarray
    status: np.ndarray
    position: np.ndarray
    rotation: np.ndarray

    @property
    def yaw(self):
        return math.atan2(self.rotation[1, 0], self.rotation[0, 0])

    @classmethod
    def unavailable(cls):
        return cls(0, -math.inf, -math.inf, np.full(4, np.inf), np.zeros(4, int),
                   np.zeros(3), np.eye(3))


@dataclass
class Monitor:
    trip: bool = False
    reason: str = ""
    margin: float = math.inf
    required: np.ndarray = field(default_factory=lambda: np.zeros(4))
    available: np.ndarray = field(default_factory=lambda: np.full(4, np.inf))


def rotation_difference(a, b):
    return math.acos(float(np.clip((np.trace(a.T@b)-1)/2, -1, 1)))


def closure(velocity, frame, config):
    """Worst projection in each sampled 3-D cone; XY safety uses horizontal v."""
    v = np.r_[np.asarray(velocity)[:2], 0.]
    speed = float(np.linalg.norm(v))
    if speed < 1e-12:
        return np.zeros(4)
    directions = AXES_BODY@frame.rotation.T
    angles = np.arccos(np.clip(directions@v/speed, -1, 1))
    return np.maximum(speed*np.cos(np.maximum(angles-config.half_fov, 0)), 0)


def raw_problem(frame, now, config):
    if frame.seq == 0:
        return "waiting_for_ranges"
    if (not isinstance(frame.seq, (int, np.integer)) or frame.seq < 1
            or not np.isfinite([frame.sample_time, frame.delivery_time]).all()
            or frame.sample_time < 0 or frame.sample_time > frame.delivery_time+1e-8
            or frame.delivery_time > now+1e-8
            or np.shape(frame.ranges) != (4,) or np.shape(frame.status) != (4,)
            or np.shape(frame.position) != (3,) or np.shape(frame.rotation) != (3, 3)):
        return "invalid_raw_frame"
    if (not np.isin(frame.status, [HIT, NO_RETURN]).all()
            or np.isnan(frame.ranges).any() or (frame.ranges < 0).any()
            or not np.array_equal(np.isfinite(frame.ranges), frame.status == HIT)
            or (frame.ranges[frame.status == HIT] > config.range_max).any()
            or not np.isfinite(frame.position).all() or not np.isfinite(frame.rotation).all()
            or not np.allclose(frame.rotation.T@frame.rotation, np.eye(3), atol=1e-5)
            or abs(np.linalg.det(frame.rotation)-1) > 1e-5):
        return "invalid_raw_frame"
    return "ranger_timeout" if now-frame.sample_time > config.max_age+1e-8 else ""


class RangeProcessor:
    def __init__(self, config):
        self.config = config
        self.frames = deque(maxlen=config.filter_window)
        self.last = None
        self.ranges = np.full(4, np.inf)
        self.closing_rate = np.zeros(4)  # diagnostic only

    def update(self, frame, now):
        self.closing_rate = np.zeros(4)
        if raw_problem(frame, now, self.config) or (self.last and frame.seq <= self.last.seq):
            return self.ranges.copy()
        if self.last and rotation_difference(frame.rotation, self.last.rotation) > self.config.filter_rotation_tol:
            self.frames.clear()
        self.frames.append(frame)
        values = [np.maximum(f.ranges-np.linalg.norm(frame.position-f.position), 0) for f in self.frames]
        filtered = np.median(values, axis=0)
        if self.last and rotation_difference(frame.rotation, self.last.rotation) <= self.config.filter_rotation_tol:
            dt = frame.sample_time-self.last.sample_time
            valid = np.isfinite(filtered) & np.isfinite(self.ranges)
            if dt > 0:
                self.closing_rate[valid] = (self.ranges[valid]-filtered[valid])/dt
        self.last, self.ranges = frame, filtered
        return filtered.copy()


def primary_monitor(frame, ranges, observation, candidate_body, now, config):
    if not observation.finite() or not np.isfinite(candidate_body).all():
        return Monitor(True, "invalid_state_or_command")
    problem = raw_problem(frame, now, config)
    if problem:
        return Monitor(True, problem)
    if np.shape(ranges) != (4,) or np.isnan(ranges).any() or (ranges < 0).any():
        return Monitor(True, "invalid_processed_ranges")
    command = rot2d(observation.yaw)@candidate_body
    proposed = np.r_[command, 0.]
    speed = float(max(np.linalg.norm(observation.velocity[:2]), np.linalg.norm(command)))
    if np.linalg.norm(command) > config.approach_speed:
        angles = np.arccos(np.clip((AXES_BODY@frame.rotation.T)@proposed/np.linalg.norm(proposed), -1, 1))
        if np.min(angles) > max(config.half_fov, math.radians(2))+1e-8:
            return Monitor(True, "unobserved_direction")
    close = np.maximum(closure(observation.velocity, frame, config), closure(proposed, frame, config))
    travel = closure(observation.position-frame.position, frame, config)
    available = np.minimum(ranges, config.range_max)+np.asarray(config.offsets)*math.cos(config.half_fov)-config.radius
    reaction = np.maximum(close*max(now-frame.sample_time, 0), travel)+close*config.dt
    required = (config.margin+config.noise_sigma*config.noise_std+reaction
                +config.braking_distance(speed)*close/max(speed, 1e-12))
    look = close > config.approach_speed
    margin = float(np.min((available-required)[look])) if np.any(look) else math.inf
    return Monitor(margin <= 0, "stopping_distance" if margin <= 0 else "", margin, required, available)


def backup_monitor(frame, observation, now, primary_time, navigator_time, navigator_required,
                   last_seq, last_sample_time, config):
    """Independent RAW path; never calls primary_monitor or RangeProcessor."""
    if not observation.finite():
        return Monitor(True, "invalid_estimate")
    problem = raw_problem(frame, now, config)
    if problem:
        return Monitor(True, problem)
    if (frame.seq < last_seq or (frame.seq == last_seq and frame.sample_time != last_sample_time)
            or (frame.seq > last_seq and frame.sample_time <= last_sample_time)):
        return Monitor(True, "nonmonotonic_frame")
    if now-primary_time > config.heartbeat_timeout+1e-8:
        return Monitor(True, "primary_timeout")
    if navigator_required and now-navigator_time > config.heartbeat_timeout+1e-8:
        return Monitor(True, "navigator_timeout")
    tilt = math.acos(float(np.clip(observation.rotation[2, 2], -1, 1)))
    if tilt > config.max_tilt:
        return Monitor(True, "tilt_limit")
    gap = np.minimum(frame.ranges, config.range_max)+np.asarray(config.offsets)*math.cos(config.half_fov)-config.radius
    if np.any(gap[frame.status == HIT] < config.hard_gap):
        return Monitor(True, "hard_distance", float(np.min(gap)-config.hard_gap))
    # Compute the raw cone envelope independently, with its own response bound.
    v = np.r_[observation.velocity[:2], 0.]
    speed = float(np.linalg.norm(v))
    if speed > config.max_speed+1e-8:
        return Monitor(True, "overspeed")
    if speed < 1e-12:
        return Monitor()
    axes = AXES_BODY@frame.rotation.T
    angle = np.arccos(np.clip(axes@v/speed, -1, 1))
    close = np.maximum(speed*np.cos(np.maximum(angle-config.half_fov, 0)), 0)
    delta = np.r_[observation.position[:2]-frame.position[:2], 0.]
    length = float(np.linalg.norm(delta))
    travel = np.zeros(4)
    if length > 1e-12:
        a = np.arccos(np.clip(axes@delta/length, -1, 1))
        travel = np.maximum(length*np.cos(np.maximum(a-config.half_fov, 0)), 0)
    required = (config.backup_margin+config.backup_noise_sigma*config.noise_std
                +np.maximum(close*(config.ranger_period+config.ranger_latency), travel)
                +close*(config.dt+config.backup_brake_response+speed/(2*config.backup_brake_accel)))
    look = close > config.approach_speed
    margin = float(np.min((gap-required)[look])) if np.any(look) else math.inf
    return Monitor(margin <= 0, "backup_distance" if margin <= 0 else "", margin, required, gap)


class SafetySupervisor:
    """One horizontal command owner; navigation checkpoints prevent false progress."""

    def __init__(self, config=None):
        self.config = config or SafetyConfig()
        self.processor = RangeProcessor(self.config)
        self.state, self.reason, self.source = "HOLD", "waiting_for_ranges", "startup"
        self.hold_position = None
        self.primary_time = self.navigator_time = None
        self.last_seq, self.last_sample = 0, -math.inf
        self.clear_seq, self.clear_count = 0, 0
        self.need_replan, self.recover_since = False, 0.
        self.last_nominal = np.zeros(2)
        self.last_error = ""
        self.events = []
        self.primary = self.backup = Monitor(True, "waiting_for_ranges")
        self.accepted = False
        self.nominal = self.candidate = self.selected = np.zeros(2)
        self.selected_yaw = 0.
        self.age = math.inf
        self.hold_limited = False

    def _event(self, now, kind, observation):
        self.events.append({"t": now, "type": kind, "source": self.source,
                            "reason": self.reason, "position": observation.position[:2].tolist()})

    def step(self, observation, frame, now, navigator, nav_time=None, *,
             primary_available=True, navigator_available=True, processed_bias=0., mission_hold=False,
             hold_target=None):
        c = self.config
        if not math.isfinite(now) or now < 0:
            raise ValueError("Safety time must be finite and nonnegative")
        if self.primary_time is None:
            self.primary_time = self.navigator_time = now
        if self.hold_position is None and observation.finite():
            self.hold_position = observation.position[:2].copy()
        self.age = now-frame.sample_time
        ranges = self.processor.update(frame, now)+processed_bias
        running = self.state in ("NORMAL", "RECOVER")
        snapshot = None
        nominal, yaw = np.zeros(2), 0.
        if running:
            if navigator_available:
                try:
                    if not mission_hold:
                        snapshot = navigator.checkpoint()
                        x = np.r_[observation.position[:2], observation.velocity[:2]]
                        clock = now if nav_time is None else nav_time
                        # Navigator events/scans use its mission-relative clock;
                        # monitor ages and raw transport remain absolute.
                        nav_frame = replace(frame, sample_time=frame.sample_time-(now-clock),
                                            delivery_time=frame.delivery_time-(now-clock))
                        nominal, yaw = navigator.step(x, observation.yaw, ranges, clock, frame=nav_frame)
                    elif hold_target is not None:
                        desired = sat_norm(c.hold_gain*(np.asarray(hold_target)-observation.position[:2]), c.hold_speed)
                        nominal = rot2d(observation.yaw).T@desired
                    if np.shape(nominal) != (2,) or not np.isfinite(np.r_[nominal, yaw]).all():
                        raise ValueError("Navigator returned an invalid command")
                    self.navigator_time = now
                    self.last_nominal = np.array(nominal, copy=True)
                except Exception as error:
                    self.last_error = f"navigator: {error}"
                    self.navigator_time = -math.inf
                    nominal, yaw = np.zeros(2), 0.
            else:
                nominal = self.last_nominal.copy()
        candidate = sat_norm(np.asarray(nominal, dtype=float), c.max_speed)
        if self.state == "RECOVER":
            f = np.clip((now-self.recover_since)/c.recover_time, 0, 1)
            candidate = sat_norm(candidate, c.recover_speed+f*(c.cruise_speed-c.recover_speed))
        if primary_available:
            try:
                self.primary = primary_monitor(frame, ranges, observation, candidate, now, c)
                self.primary_time = now
            except Exception as error:
                self.last_error = f"primary: {error}"
                self.primary_time = -math.inf
        self.backup = backup_monitor(frame, observation, now, self.primary_time, self.navigator_time,
                                     (running and not mission_hold) or not navigator_available,
                                     self.last_seq, self.last_sample, c)
        if not raw_problem(frame, now, c) and frame.seq > self.last_seq:
            self.last_seq, self.last_sample = frame.seq, frame.sample_time
        trip = self.primary.trip or self.backup.trip
        speed = float(np.linalg.norm(observation.velocity[:2]))
        if trip:
            self.clear_count = 0
            monitor, source = (self.backup, "backup") if self.backup.trip else (self.primary, "primary")
            self.reason, self.source = monitor.reason, source
            if self.state in ("NORMAL", "RECOVER"):
                self.state = "BRAKE"
                self.need_replan |= speed > c.stop_speed or np.linalg.norm(nominal) > c.approach_speed
                self._event(now, "brake", observation)
        if self.state == "BRAKE" and observation.finite() and speed <= c.stop_speed:
            self.state = "HOLD"
            self.hold_position = observation.position[:2].copy()
            self._event(now, "stopped", observation)
        self.accepted = False
        resume = False
        if self.state == "HOLD":
            if not trip and speed <= c.stop_speed and frame.seq > self.clear_seq:
                self.clear_count += 1
            self.clear_seq = max(self.clear_seq, frame.seq)
            if not trip and self.clear_count >= c.clear_frames:
                self.state, self.recover_since = "RECOVER", now
                resume, self.need_replan = self.need_replan, False
                self._event(now, "resume", observation)
                self.reason = self.source = ""
        elif self.state != "BRAKE":
            if self.state == "RECOVER" and now-self.recover_since >= c.recover_time:
                self.state = "NORMAL"
            self.accepted = not trip
        if not self.accepted and snapshot is not None:
            navigator.restore(snapshot)
        if resume and not mission_hold:
            navigator.resume(np.r_[observation.position[:2], observation.velocity[:2]], observation.yaw,
                             now if nav_time is None else nav_time)
        if self.accepted:
            selected, selected_yaw = candidate, yaw
        else:
            selected, selected_yaw = np.zeros(2), 0.
            if self.state == "HOLD" and not trip and observation.finite() and self.hold_position is not None:
                target = sat_norm(c.hold_gain*(self.hold_position-observation.position[:2]), c.hold_speed)
                selected = rot2d(observation.yaw).T@target
                # Holding feedback is also a motion command. It must not bypass
                # the raw monitor or send a correction through a blind sector.
                if np.linalg.norm(target) > 1e-12:
                    v3 = np.r_[target, 0.]
                    angles = np.arccos(np.clip((AXES_BODY@frame.rotation.T)@v3/np.linalg.norm(v3), -1, 1))
                    hold_observation = replace(observation, velocity=v3)
                    check = backup_monitor(frame, hold_observation, now, self.primary_time, self.navigator_time,
                                           False, self.last_seq, self.last_sample, c)
                    if check.trip or np.min(angles) > max(c.half_fov, math.radians(2))+1e-8:
                        selected = np.zeros(2)
        self.hold_limited = (not self.accepted and self.state == "HOLD"
                             and np.linalg.norm(selected) < 1e-12 and observation.finite()
                             and self.hold_position is not None
                             and np.linalg.norm(self.hold_position-observation.position[:2]) > 1e-6)
        self.nominal, self.candidate = np.asarray(nominal), candidate
        self.selected, self.selected_yaw = selected, float(selected_yaw)
        return selected.copy(), float(selected_yaw)

    def log_fields(self):
        return {"safety_state": self.state, "safety_reason": self.reason, "safety_source": self.source,
                "safety_accepted": int(self.accepted), "safety_ranger_age_s": self.age,
                "safety_primary_trip": int(self.primary.trip), "safety_backup_trip": int(self.backup.trip),
                "safety_primary_margin_m": self.primary.margin, "safety_backup_margin_m": self.backup.margin,
                "safety_nominal_speed_m_s": float(np.linalg.norm(self.nominal)),
                "safety_candidate_speed_m_s": float(np.linalg.norm(self.candidate)),
                "safety_selected_speed_m_s": float(np.linalg.norm(self.selected)),
                "safety_hold_limited": int(self.hold_limited),
                "safety_last_error": self.last_error}
