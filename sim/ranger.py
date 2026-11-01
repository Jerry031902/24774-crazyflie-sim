"""Multi-ranger measurement simulation, independent of the EKF and controller.

Firmware: src/deck/drivers/src/multiranger.c:99-105,134-143 (invalid/rate),
src/modules/src/range.c:38-43,66-86 (uint16 millimeters and direction names).
Hardware: https://www.bitcraze.io/products/multi-ranger-deck/ (five axes, 4 m,
datasheet drawing for sensor positions) and the ST VL53L1X datasheet (27 deg
field of view). Each sensor reports the nearest hit in its cone of rays.
matlab-trajectory-sim/src/sensors/read_rangers.m is the same model in 2-D.
"""

import csv
from dataclasses import dataclass
import heapq
import json
import math
from pathlib import Path

import numpy as np

from controllers.firmware_pid import rotation_and_rpy, vector3

# Firmware scheduling is nominal; hardware waits for data-ready sequentially.
RANGER_PERIOD_MS = 100
RANGER_MAX_DISTANCE_M = 4.0  # Nominal hardware range, not a guarantee on materials.
RANGER_INVALID_MM = 32767   # Driver's invalid status sentinel, never a 32.767 m hit.
RANGER_NOISE_STD_M = 0.003   # "Within a few millimeters" (Bitcraze); NOT calibrated to this drone.
RANGER_BIAS_M = 0.0
# Every 100 ms the driver reads each sensor's result and restarts it, so a
# value reaches range.c one period after its measurement began. The driver
# keeps the ST defaults (long mode, 41 ms timing budget), which finish within
# the period. Geometry is taken at the start of the measurement.
RANGER_DELAY_MS = RANGER_PERIOD_MS
RANGER_SEED = 7
# VL53L1X full field of view with its default 16x16 SPAD ROI, which the driver
# never changes. RANGER_CONE_RAYS spans the cone's width, as p.ranger.n_rays.
RANGER_FOV_RAD = math.radians(27)
RANGER_CONE_RAYS = 9

# Body frame: X forward, Y left, Z up. Axis names follow range.c.
RANGER_DIRECTIONS_BODY = {
    "front": (1., 0., 0.), "back": (-1., 0., 0.),
    "left": (0., 1., 0.), "right": (0., -1., 0.), "up": (0., 0., 1.),
}
# Sensor faces from the Multi-ranger datasheet drawing: the four side sensors
# stand on edge daughter boards 32.5 mm apart front-back and 32.6 mm left-right;
# the up sensor is at the deck centre. Heights above the body origin are
# unmeasured and left at 0. Meters relative to body. Self is excluded.
RANGER_ORIGINS_BODY_M = {
    "front": (0.01625, 0., 0.), "back": (-0.01625, 0., 0.),
    "left": (0., 0.0163, 0.), "right": (0., -0.0163, 0.), "up": (0., 0., 0.),
}


def cone_directions(axis, fov_rad=RANGER_FOV_RAD, cone_rays=RANGER_CONE_RAYS):
    """Body-frame unit rays filling a circular cone about one sensor's axis.

    An n x n grid of angles (across, then up/down), clipped to the cone. For a
    horizontal sensor the middle row is the 2-D fan of read_rangers.m, so level
    flight past vertical walls reads as in MATLAB. One ray when n=1 or fov=0.
    """
    axis = np.asarray(axis, dtype=float)
    if cone_rays <= 1 or fov_rad <= 0:
        return axis[None, :]
    across = np.cross((0., 0., 1.), axis)  # +90 deg yaw: front -> left
    if np.linalg.norm(across) < 1e-9:
        across = np.array((1., 0., 0.))   # up sensor: spread over body X and Y
    vertical = np.cross(axis, across)
    half = fov_rad / 2
    angles = np.linspace(-half, half, cone_rays)
    rays = [math.cos(b) * (math.cos(a) * axis + math.sin(a) * across) + math.sin(b) * vertical
            for b in angles for a in angles if math.cos(a) * math.cos(b) >= math.cos(half) - 1e-12]
    return np.array(rays)


@dataclass(frozen=True)
class RayHit:
    distance_m: float
    collision_path: str


class PhysXRaycaster:
    """Closest non-self collider using the API bundled with Isaac Sim 5.1.

    raycast_all hits are not guaranteed sorted. Continue traversal after self
    hits and choose the nearest remaining hit. Queries read live physics state.
    omni.physx is the current PhysX extension, not deprecated omni.isaac.*.
    """

    def __init__(self, excluded_roots=("/World/Crazyflie",)):
        # Call only AFTER SimulationApp and physics initialization.
        from omni.physx import get_physx_scene_query_interface
        from pxr import Gf

        self.query = get_physx_scene_query_interface()
        self.vec = Gf.Vec3f
        self.excluded_roots = tuple(root.rstrip("/") for root in excluded_roots)

    def __call__(self, origin, direction, max_distance):
        closest = None

        def report(hit):
            nonlocal closest
            collision, body = str(hit.collision), str(hit.rigid_body)
            if any(path == root or path.startswith(root + "/")
                   for root in self.excluded_roots for path in (collision, body)):
                return True
            distance = float(hit.distance)
            if 0 <= distance <= max_distance and (closest is None or distance < closest.distance_m):
                closest = RayHit(distance, collision)
            return True

        self.query.raycast_all(self.vec(*map(float, origin)),
                               self.vec(*map(float, direction)), float(max_distance), report, True)
        return closest


class MultiRanger:
    """Sample five body-fixed ray cones at 10 Hz and deliver them a period later.

    Pass a callable raycaster so timing/noise/frame conversion can be tested
    without Kit. RNG is separate from SensorSuite; enabling this deck must not
    change the IMU/Flow/ToF noise sequence or any controller input.
    """

    def __init__(self, raycast, noise=False, seed=RANGER_SEED,
                 std_m=RANGER_NOISE_STD_M, bias_m=RANGER_BIAS_M,
                 delay_ms=RANGER_DELAY_MS, max_distance_m=RANGER_MAX_DISTANCE_M,
                 origins_body_m=None, fov_rad=RANGER_FOV_RAD, cone_rays=RANGER_CONE_RAYS):
        if not callable(raycast):
            raise ValueError("raycast must be callable")
        if not math.isfinite(std_m) or std_m < 0 or not math.isfinite(bias_m):
            raise ValueError("Ranger std must be finite/nonnegative and bias finite")
        if not math.isfinite(max_distance_m) or not 0 < max_distance_m < RANGER_INVALID_MM/1000:
            raise ValueError("Ranger maximum distance must be positive and below the invalid sentinel")
        if not math.isfinite(delay_ms) or delay_ms < 0 or delay_ms != int(delay_ms):
            raise ValueError("Ranger delay must be nonnegative integer milliseconds")
        if not isinstance(seed, (int, np.integer)) or seed < 0:
            raise ValueError("Ranger seed must be a nonnegative integer")
        if not math.isfinite(fov_rad) or not 0 <= fov_rad < math.pi:
            raise ValueError("Ranger field of view must be in [0, pi) radians")
        if not isinstance(cone_rays, (int, np.integer)) or cone_rays < 1:
            raise ValueError("Ranger cone_rays must be a positive integer")
        self.raycast, self.noise, self.seed = raycast, bool(noise), int(seed)
        self.std_m, self.bias_m = float(std_m), float(bias_m)
        self.delay_ms, self.max_distance_m = int(delay_ms), float(max_distance_m)
        origins = RANGER_ORIGINS_BODY_M if origins_body_m is None else origins_body_m
        if set(origins) != set(RANGER_DIRECTIONS_BODY):
            raise ValueError("Provide one body-frame origin for each of the five sensors")
        self.origins = {name: vector3(value).copy() for name, value in origins.items()}
        self.fov_rad, self.cone_rays = float(fov_rad), int(cone_rays)
        self.rays_body = {name: cone_directions(axis, self.fov_rad, self.cone_rays)
                          for name, axis in RANGER_DIRECTIONS_BODY.items()}
        self.reset()

    def reset(self):
        self.rng = np.random.default_rng(np.random.SeedSequence([self.seed, 0x52414E47]))
        self.next_ms = 0
        self.pending = []
        self.generated_frames = self.delivered_frames = 0
        self.latest = None  # Firmware range registers start at 0; not yet valid.

    def measure(self, truth):
        """Produce one instantaneous frame; sample() owns scheduling/delivery."""
        rotation, _ = rotation_and_rpy(truth["orientation_wxyz"])
        position = vector3(truth["position_m"])
        sample_ms = round(float(truth["time_s"]) * 1000)
        readings = []
        for name, body_direction in RANGER_DIRECTIONS_BODY.items():
            origin = position + rotation @ self.origins[name]
            direction = rotation @ np.asarray(body_direction)
            hit = None  # Nearest return anywhere in the cone, as the sensor reports.
            for ray in self.rays_body[name]:
                candidate = self.raycast(origin, rotation @ ray, self.max_distance_m)
                if candidate is not None and (hit is None or candidate.distance_m < hit.distance_m):
                    hit = candidate
            valid = hit is not None and math.isfinite(hit.distance_m) and 0 <= hit.distance_m <= self.max_distance_m
            ideal = float(hit.distance_m) if valid else math.nan
            status = "valid" if valid else "no_return"
            range_mm = RANGER_INVALID_MM
            if valid:
                # Noise switch controls the random term; bias is independently
                # configured. Quantize to integer mm, matching range.* log units.
                measured = ideal + self.bias_m + (self.rng.normal(0, self.std_m) if self.noise else 0.)
                valid = 0 <= measured <= self.max_distance_m
                if valid:
                    range_mm = int(round(measured * 1000))
                else:
                    status = "out_of_range"
            row = {"sample_time_s": sample_ms/1000,
                   "delivery_time_s": (sample_ms+self.delay_ms)/1000,
                   "direction": name, "range_mm": range_mm, "valid": int(valid),
                   "distance_m": range_mm/1000 if valid else math.nan,
                   "ideal_distance_m": ideal, "status": status,
                   "hit_prim": hit.collision_path if hit is not None else ""}
            for axis, xyz, component in zip("xyz", origin, direction):
                row[f"origin_{axis}_m"] = float(xyz)
                row[f"direction_{axis}"] = float(component)
            readings.append(row)
        return readings

    def sample(self, truth):
        """Call once per 1 ms from t=0. Measure at 0, 100, ... ms, as the driver starts them."""
        now = float(truth["time_s"])
        if not math.isfinite(now):
            raise ValueError("Ranger time must be finite")
        ms = round(now * 1000)
        if ms != self.next_ms or abs(now-ms/1000) > 1e-8:
            raise ValueError("Sample Ranger exactly once per 1 ms, starting at t=0")
        self.next_ms += 1
        if ms % RANGER_PERIOD_MS == 0:
            frame = self.measure(truth)
            heapq.heappush(self.pending, (ms+self.delay_ms, ms, frame))
            self.generated_frames += 1
        ready = []
        while self.pending and self.pending[0][0] <= ms:
            _, _, frame = heapq.heappop(self.pending)
            self.latest = frame
            self.delivered_frames += 1
            ready.extend(frame)
        return ready

    def snapshot(self):
        """Held values for full-rate flight CSV; use timestamps, not value changes."""
        fields = {"ranger_sample_time_s": math.nan, "ranger_delivery_time_s": math.nan}
        for name in RANGER_DIRECTIONS_BODY:
            fields[f"range.{name}"] = 0
            fields[f"range_{name}_valid"] = 0
        if self.latest is not None:
            fields["ranger_sample_time_s"] = self.latest[0]["sample_time_s"]
            fields["ranger_delivery_time_s"] = self.latest[0]["delivery_time_s"]
            for row in self.latest:
                fields[f"range.{row['direction']}"] = row["range_mm"]
                fields[f"range_{row['direction']}_valid"] = row["valid"]
        return fields

    def settings(self):
        return {"model": "nearest_hit_in_ray_cone", "period_ms": RANGER_PERIOD_MS,
                "fov_deg": math.degrees(self.fov_rad), "cone_rays": self.cone_rays,
                "rays_per_sensor": len(self.rays_body["front"]),
                "max_distance_m": self.max_distance_m, "invalid_mm": RANGER_INVALID_MM,
                "noise": self.noise, "noise_std_m": self.std_m, "bias_m": self.bias_m,
                "delay_ms": self.delay_ms, "seed": self.seed,
                "origins_body_m": {k: v.tolist() for k, v in self.origins.items()},
                "directions_body": RANGER_DIRECTIONS_BODY,
                "assumptions": "datasheet sensor positions, unmeasured heights; nearest hit in the cone; "
                               "no material/lighting/status/dropout model; no avoidance"}


class RangerLog:
    """One row per delivered direction/sample, plus reproducible model settings."""

    def __init__(self, path, sensor):
        self.path, self.sensor = Path(path), sensor
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("w", newline="")
        self.writer = None
        self.rows = 0

    def write(self, readings):
        for row in readings:
            if self.writer is None:
                self.writer = csv.DictWriter(self.file, fieldnames=list(row))
                self.writer.writeheader()
            self.writer.writerow(row)
            self.rows += 1
        if readings:
            self.file.flush()

    def close(self):
        self.file.close()
        self.path.with_suffix(".json").write_text(json.dumps(
            {"settings": self.sensor.settings(), "rows": self.rows,
             "generated_frames": self.sensor.generated_frames,
             "delivered_frames": self.sensor.delivered_frames}, indent=2) + "\n")
