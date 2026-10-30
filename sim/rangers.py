"""Multi-ranger deck: four horizontal VL53L1x ToF sensors, by PhysX ray casts.

Same sensor model as navigation.geometry.read_rangers (MATLAB read_rangers.m),
but cast against the actual USD scene in 3-D: each sensor casts
p.ranger.n_rays rays across its p.ranger.fov cone in the body XY plane and
reports the nearest hit; inf = no return within p.ranger.range_max. Rays
rotate with the full attitude, so tilting while accelerating tilts them too.
The drone's own colliders are ignored. Requires a running SimulationApp.
"""

import math

import numpy as np

from controllers.firmware_pid import rotation_and_rpy
from navigation.geometry import apply_ranger_noise, ranger_spread

# Deck mount relative to the body origin [m]. The deck sits on top of the
# frame; this offset is a placeholder until measured.
MOUNT_OFFSET_BODY_M = (0.0, 0.0, 0.0)


class PhysXMultiRanger:
    def __init__(self, p, ignore_prefix, seed=0, mount_offset_body_m=MOUNT_OFFSET_BODY_M):
        import carb
        from omni.physx import get_physx_scene_query_interface

        self.carb = carb
        self.query = get_physx_scene_query_interface()
        self.p = p
        self.ignore_prefix = ignore_prefix.rstrip("/") + "/"
        self.offset = np.asarray(mount_offset_body_m, dtype=float)
        spread = ranger_spread(p)
        # [sensor, ray, xyz] body-frame unit vectors: front, left, back, right.
        self.rays_body = np.array([[[math.cos(j * math.pi / 2 + a), math.sin(j * math.pi / 2 + a), 0.0]
                                    for a in spread] for j in range(4)])
        # Cast past range_max so noise can pull a slightly-too-far return into range, as in MATLAB.
        self.cast_distance = 2.0 * p.ranger.range_max
        self.reset(seed)

    def reset(self, seed=0):
        self.rng = np.random.default_rng(seed)

    def _cast(self, origin, direction):
        best = math.inf
        prefix = self.ignore_prefix

        def report(hit):
            nonlocal best
            if not hit.collision.startswith(prefix) and hit.distance < best:
                best = hit.distance
            return True  # keep traversing: the drone's own colliders may come first

        self.query.raycast_all(self.carb.Float3(*map(float, origin)), self.carb.Float3(*map(float, direction)),
                               self.cast_distance, report)
        return best

    def read(self, position_m, orientation_wxyz):
        """[front, left, back, right] in metres for the given world pose."""
        rotation, _ = rotation_and_rpy(orientation_wxyz)
        origin = np.asarray(position_m, dtype=float) + rotation @ self.offset
        r = np.full(4, math.inf)
        for j in range(4):
            for ray in self.rays_body[j]:
                r[j] = min(r[j], self._cast(origin, rotation @ ray))
        return apply_ranger_noise(r, self.p, self.rng)
