"""Measurement-level Flow deck v2 + BMI088 simulation, without an estimator.

Source references are relative to the firmware root; see BASELINE_EKF.md.
Flat floor at world Z=0, valid textured surface, calibrated/booted IMU, no bias.
Continuous-valued interface measurements intentionally omit ADC/pixel quantization.
IMU and ToF noise follow the team's static bench tests (motors off); Flow noise
is still an assumption until it is measured on a consistent floor and lighting.
"""

from dataclasses import dataclass
import heapq
import math

import numpy as np

from controllers.firmware_pid import FirmwareGyroFilter, rotation_and_rpy, vector3

# Rates/filters from sensors_bmi088_bmp3xx.c:61,140-141,546-547;
# flowdeck_v1v2.c:88 and FreeRTOSConfig.h:77; zranger2.c:125,132.
IMU_HZ = 1000
FLOW_PERIOD_MS = 10  # Nominal. Hardware task also spends time reading SPI.
TOF_PERIOD_MS = 25   # Flow deck v2; hardware additionally waits for data-ready.
GYRO_LPF_HZ = 80
ACCEL_LPF_HZ = 30
GRAVITY_M_S2 = 9.81


def lpf2p_noise_gain(sample_hz, cutoff_hz):
    """Output/input std of white noise through the firmware 2-pole LPF.

    Same coefficients as lpf2pSetCutoffFreq (src/utils/src/filter.c); the gain
    is the root energy of its impulse response.
    """
    ohm = math.tan(math.pi * cutoff_hz / sample_hz)
    c = 1 + 2 * math.cos(math.pi / 4) * ohm + ohm * ohm
    b0 = ohm * ohm / c
    a1, a2 = 2 * (ohm * ohm - 1) / c, (1 - 2 * math.cos(math.pi / 4) * ohm + ohm * ohm) / c
    d1 = d2 = energy = 0.0
    for i in range(5000):
        d0 = (1.0 if i == 0 else 0.0) - d1 * a1 - d2 * a2
        energy += (b0 * d0 + 2 * b0 * d1 + b0 * d2) ** 2
        d2, d1 = d1, d0
    return math.sqrt(energy)


# Static bench tests of the team's drone, motors off: std of the logged
# gyro.* / acc.* (AFTER the firmware filters) and of range.zrange.
GYRO_MEASURED_STD_DEG_S = (0.114, 0.169, 0.126)  # x, y, z
ACCEL_MEASURED_STD_G = (0.00133, 0.00101, 0.00138)  # x, y, z
TOF_MEASURED_STD_M = (0.00165, 0.00206)  # range over runs at 0.5-0.7 m height

# Independent, zero-mean Gaussian noise injected BEFORE the IMU software
# filters, scaled up so the filtered output matches the bench std.
GYRO_NOISE_STD_DEG_S = tuple(s / lpf2p_noise_gain(IMU_HZ, GYRO_LPF_HZ) for s in GYRO_MEASURED_STD_DEG_S)
ACCEL_NOISE_STD_G = tuple(s / lpf2p_noise_gain(IMU_HZ, ACCEL_LPF_HZ) for s in ACCEL_MEASURED_STD_G)
FLOW_NOISE_STD_COUNTS = 2.0  # Assumption: bench results varied with floor texture and lighting.
TOF_NOISE_STD_M = TOF_MEASURED_STD_M[1]  # Upper end; only 0.5-0.7 m was measured.
GYRO_DELAY_MS = 0
ACCEL_DELAY_MS = 0
FLOW_DELAY_MS = 0
TOF_DELAY_MS = 0
DEFAULT_SEED = 7

# Stock EKF uncertainty, retained even when simulated random noise is disabled.
# flowdeck_v1v2.c:45,73,77,98,119-120; zranger2.c:49-55,140-143.
FLOW_EKF_STD_COUNTS = 2.0
FLOW_OUTLIER_LIMIT = 100
TOF_RANGE_LIMIT_M = 5.0
TOF_STD_A_M = 0.0025
TOF_STD_B_M = 0.2
TOF_DISTANCE_A_M = 2.5
TOF_DISTANCE_B_M = 4.0

# mm_flow.c:31,47-48,63-83; platform_defaults.h:239-247 (CF2 inherits zeros).
FLOW_RESOLUTION = 0.1
FLOW_NPIX = 35.0
FLOW_THETAPIX = 0.71674
FLOW_MIN_HEIGHT_M = 0.1
FLOW_DECK_POSITION_BODY_M = np.zeros(3)
TOF_HALF_CONE_RAD = math.radians(7.5)  # mm_tof.c:35-40.


def flow_model(height_m, rotation, velocity_body_m_s, omega_body_rad_s, dt_s):
    """Return firmware dpixel counts, including rotation and camera lever arm.

    Reconstructs mm_flow.c:70-97. EKF multiplies these counts by 0.1 to obtain
    its measurement-space pixels. This model uses the instantaneous state,
    exactly as the stock model assumes for the interval flow.dt.
    """
    v = np.asarray(velocity_body_m_s) + np.cross(omega_body_rad_s, FLOW_DECK_POSITION_BODY_M)
    omega = np.asarray(omega_body_rad_s)
    scale = rotation[2, 2] / max(float(height_m), FLOW_MIN_HEIGHT_M)
    return (dt_s * FLOW_NPIX / FLOW_THETAPIX / FLOW_RESOLUTION
            * np.array([v[0]*scale - omega[1], v[1]*scale + omega[0]]))


def tof_model(height_m, r22):
    """Stock flat-floor cone model, not a scene raycast (mm_tof.c:35-40)."""
    if r22 <= 0.1:
        return math.nan  # No modeled downward floor return at this attitude.
    angle = max(0.0, abs(math.acos(float(np.clip(r22, -1, 1)))) - TOF_HALF_CONE_RAD)
    return float(height_m) / math.cos(angle)


def tof_stock_std(distance_m):
    coefficient = math.log(TOF_STD_B_M / TOF_STD_A_M) / (TOF_DISTANCE_B_M - TOF_DISTANCE_A_M)
    return TOF_STD_A_M * (1 + math.exp(coefficient * (distance_m - TOF_DISTANCE_A_M)))


@dataclass
class Measurement:
    kind: str
    sample_ms: int
    delivery_ms: int
    value: np.ndarray
    dt_s: float = 0.0
    std: float = 0.0


class SensorSuite:
    """Sample at firmware nominal rates and deliver delayed packets in FIFO order.

    Equal-time order is gyro, acc, Flow, ToF, a deterministic scheduling choice.
    A heap orders by delivery time then insertion sequence, never by sample time.
    Stock queue processing does not rewind EKF history for delayed measurements.
    """

    def __init__(self, noise=False, seed=DEFAULT_SEED, delays_ms=None):
        self.noise, self.seed = bool(noise), int(seed)
        self.delays = {"gyro": GYRO_DELAY_MS, "acc": ACCEL_DELAY_MS,
                       "flow": FLOW_DELAY_MS, "tof": TOF_DELAY_MS}
        if delays_ms is not None:
            self.delays.update(delays_ms)
        if set(self.delays) != {"gyro", "acc", "flow", "tof"} or any(
            not math.isfinite(d) or d < 0 or d != int(d) for d in self.delays.values()
        ):
            raise ValueError("Sensor delays must be nonnegative integer milliseconds")
        self.reset()

    def reset(self):
        self.rng = np.random.default_rng(self.seed)
        self.gyro_filter = FirmwareGyroFilter(IMU_HZ, GYRO_LPF_HZ)
        self.acc_filter = FirmwareGyroFilter(IMU_HZ, ACCEL_LPF_HZ)
        self.previous_velocity = None
        self.next_ms = 0
        self.last_flow_read_ms = self.last_flow_valid_ms = 0
        self.pending = []
        self.sequence = 0
        self.latest_gyro = np.zeros(3)
        self.latest_acc = np.array([0., 0., 1.])
        self.latest_imu_sample_ms = 0
        self.generated = dict.fromkeys(self.delays, 0)
        self.dropped = {"flow": 0, "tof": 0}

    def _noise(self, std, size):
        return self.rng.normal(0, std, size) if self.noise else np.zeros(size)

    def _enqueue(self, kind, now_ms, value, dt_s=0.0, std=0.0):
        packet = Measurement(kind, now_ms, now_ms + int(self.delays[kind]),
                             np.asarray(value, dtype=float).copy(), dt_s, std)
        heapq.heappush(self.pending, (packet.delivery_ms, self.sequence, packet))
        self.sequence += 1
        self.generated[kind] += 1

    def sample(self, truth):
        now = float(truth["time_s"])
        ms = round(now * 1000)
        if not math.isfinite(now) or ms != self.next_ms or abs(now-ms/1000) > 1e-8:
            raise ValueError("Sample sensors exactly once per 1 ms, starting at t=0")
        self.next_ms += 1
        rotation, _ = rotation_and_rpy(truth["orientation_wxyz"])
        velocity = vector3(truth["linear_velocity_m_s"])
        omega_body = rotation.T @ vector3(truth["angular_velocity_rad_s"])
        acceleration = (np.zeros(3) if self.previous_velocity is None else
                        (velocity - self.previous_velocity) * IMU_HZ)
        self.previous_velocity = velocity.copy()
        # Body specific force in Gs, sensorData_t.acc; gyro in body deg/s.
        acc = rotation.T @ (acceleration / GRAVITY_M_S2 + [0, 0, 1])
        gyro = np.degrees(omega_body)
        # First filter sample is primed to emulate an already booted/calibrated
        # IMU. Firmware itself calibrates before flight; boot calibration is omitted.
        self._enqueue("gyro", ms, self.gyro_filter.apply(gyro + self._noise(GYRO_NOISE_STD_DEG_S, 3)))
        self._enqueue("acc", ms, self.acc_filter.apply(acc + self._noise(ACCEL_NOISE_STD_G, 3)))
        z = float(vector3(truth["position_m"])[2])

        if ms and ms % FLOW_PERIOD_MS == 0:
            read_dt = (ms - self.last_flow_read_ms) / 1000
            self.last_flow_read_ms = ms
            counts = flow_model(z, rotation, rotation.T @ velocity, omega_body, read_dt)
            counts += self._noise(FLOW_NOISE_STD_COUNTS, 2)
            # Preserve stock pre-EKF range rejection; no new innovation gating.
            if np.all(np.abs(counts) < FLOW_OUTLIER_LIMIT):
                driver_dt = (ms - self.last_flow_valid_ms) / 1000
                self.last_flow_valid_ms = ms
                # Assume valid PMW3901 motion/status 0xB0 on the textured floor.
                self._enqueue("flow", ms, counts, driver_dt, FLOW_EKF_STD_COUNTS)
            else:
                self.dropped["flow"] += 1

        if ms and ms % TOF_PERIOD_MS == 0:
            ideal = tof_model(z, rotation[2, 2])
            distance = max(0.0, ideal + float(self._noise(TOF_NOISE_STD_M, 1)[0])) if math.isfinite(ideal) else math.nan
            if math.isfinite(distance) and distance < TOF_RANGE_LIMIT_M:
                self._enqueue("tof", ms, [distance], std=tof_stock_std(distance))
            else:
                self.dropped["tof"] += 1

        ready = []
        while self.pending and self.pending[0][0] <= ms:
            packet = heapq.heappop(self.pending)[2]
            ready.append(packet)
            if packet.kind == "gyro":
                self.latest_gyro = packet.value.copy()
                self.latest_imu_sample_ms = packet.sample_ms
            elif packet.kind == "acc":
                self.latest_acc = packet.value.copy()
        return ready

    def settings(self):
        return {"noise": self.noise, "seed": self.seed, "deck": "Flow v2 / VL53L1X",
                "delays_ms": self.delays, "accel_noise_std_G": ACCEL_NOISE_STD_G,
                "gyro_noise_std_deg_s": GYRO_NOISE_STD_DEG_S,
                "accel_filtered_std_G": ACCEL_MEASURED_STD_G,
                "gyro_filtered_std_deg_s": GYRO_MEASURED_STD_DEG_S,
                "flow_noise_std_counts": FLOW_NOISE_STD_COUNTS, "tof_noise_std_m": TOF_NOISE_STD_M,
                "imu_hz": IMU_HZ, "flow_nominal_hz": 1000/FLOW_PERIOD_MS,
                "tof_nominal_hz": 1000/TOF_PERIOD_MS,
                "assumptions": "white Gaussian noise, no bias/quantization/dropout, flat floor, valid texture"}
