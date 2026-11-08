"""Associate RAW ranging with historical estimator poses, never PhysX truth."""

from collections import deque
import math
import numpy as np

from controllers.firmware_pid import rotation_and_rpy
from .core import Observation, RangeFrame, HIT, NO_RETURN, INVALID

DIRECTIONS = ("front", "left", "back", "right")


class EstimatedRangeTransport:
    def __init__(self, history_s=2.):
        self.history_s = history_s
        self.poses = deque()
        self.cached = RangeFrame.unavailable()
        self.signature = None

    def observe(self, now, source):
        rotation, rpy = rotation_and_rpy(source["orientation_wxyz"])
        obs = Observation(np.asarray(source["position_m"], dtype=float).copy(),
                          np.asarray(source["linear_velocity_m_s"], dtype=float).copy(), rotation, float(rpy[2]))
        self.poses.append((float(now), obs.position.copy(), obs.rotation.copy()))
        while len(self.poses) > 1 and self.poses[1][0] < now-self.history_s:
            self.poses.popleft()
        return obs

    def frame(self, fields):
        try:
            return self._decode(fields)
        except (ValueError, TypeError, KeyError, OverflowError):
            return RangeFrame(-1, math.nan, math.nan, np.full(4,np.nan), np.full(4,INVALID),
                              np.full(3,np.nan), np.eye(3))

    def _decode(self, fields):
        seq = fields.get("ranger_frame_seq", 0)
        if seq == 0:
            return RangeFrame.unavailable()
        sample = float(fields.get("ranger_sample_time_s", math.nan))
        delivery = float(fields.get("ranger_delivery_time_s", math.nan))
        signature = (seq, sample, delivery)+tuple(fields.get(key) for d in DIRECTIONS
            for key in (f"range.{d}",f"range_{d}_valid",f"range_{d}_status"))
        if seq == self.cached.seq:
            if signature == self.signature:
                return self.cached
            # A held packet cannot change content without a sequence change.
            raise ValueError("Ranger packet changed without a new sequence")
        status = np.array([fields.get(f"range_{d}_status", INVALID) for d in DIRECTIONS])
        ranges = np.full(4, np.nan)
        for i, d in enumerate(DIRECTIONS):
            mm = fields.get(f"range.{d}", math.nan)
            valid = fields.get(f"range_{d}_valid", 0)
            if status[i] == HIT and valid == 1:
                ranges[i] = mm/1000
            elif status[i] == NO_RETURN and valid == 0 and mm == 32767:
                ranges[i] = math.inf
            else:
                status[i] = INVALID
        pose = next((h for h in reversed(self.poses) if h[0] <= sample+1e-8), None)
        if pose is None or sample-pose[0] > .002+1e-8:
            status[:] = INVALID
            position, rotation = np.full(3, np.nan), np.eye(3)
        else:
            position, rotation = pose[1].copy(), pose[2].copy()
        self.cached = RangeFrame(seq, sample, delivery, ranges, status, position, rotation)
        self.signature = signature
        return self.cached
