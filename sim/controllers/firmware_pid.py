"""Crazyflie C PID driven by ground truth or externally supplied firmware state.

Source paths below are relative to the firmware repository root.
The C PID has process-global state: use ONE active controller/drone per process.
"""

import ctypes
import math

import numpy as np

from .base import Controller

# BMI088 sensor preprocessing, src/hal/src/sensors_bmi088_bmp3xx.c:140,330,546.
GYRO_LPF_CUTOFF_HZ = 80.0


class _Lpf2pData(ctypes.Structure):
    # Exact ABI from src/utils/interface/filter.h:35-43, seven consecutive floats.
    _fields_ = [(name, ctypes.c_float) for name in
                ("a1", "a2", "b0", "b1", "b2", "delay_element_1", "delay_element_2")]


class FirmwareGyroFilter:
    """Reuse the compiled firmware's lpf2p functions, without an estimator/noise.

    filter.h is included in the extension but not SWIG's %include list. Its C
    symbols ARE exported by this build, so ctypes avoids firmware/binding edits.
    This is the sensor driver's filter, separate from PID's optional D filter.
    """

    def __init__(self, sample_hz, cutoff_hz):
        import _cffirmware
        self.library = ctypes.CDLL(_cffirmware.__file__)
        pointer = ctypes.POINTER(_Lpf2pData)
        self.library.lpf2pInit.argtypes = [pointer, ctypes.c_float, ctypes.c_float]
        self.library.lpf2pInit.restype = None
        for name in ("lpf2pApply", "lpf2pReset"):
            function = getattr(self.library, name)
            function.argtypes = [pointer, ctypes.c_float]
            function.restype = ctypes.c_float
        self.axes = [_Lpf2pData() for _ in range(3)]
        for axis in self.axes:
            self.library.lpf2pInit(ctypes.byref(axis), sample_hz, cutoff_hz)
        self.first_sample = True

    def apply(self, gyro_body_deg_s):
        # Prime to the initial true rate, avoiding a fictitious startup transient.
        # C implementation: src/utils/src/filter.c:74-125.
        function = self.library.lpf2pReset if self.first_sample else self.library.lpf2pApply
        self.first_sample = False
        return np.array([function(ctypes.byref(axis), float(sample))
                         for axis, sample in zip(self.axes, gyro_body_deg_s)])


def rotation_and_rpy(orientation_wxyz):
    """Return body-to-world R and right-handed ZYX Euler angles in radians."""
    q = np.asarray(orientation_wxyz, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError("Expected a finite, nonzero wxyz quaternion")
    w, x, y, z = q / np.linalg.norm(q)
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    rpy = np.array([math.atan2(rotation[2, 1], rotation[2, 2]),
                    math.asin(float(np.clip(-rotation[2, 0], -1, 1))),
                    math.atan2(rotation[1, 0], rotation[0, 0])])
    return rotation, rpy


def vector3(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError("Expected a finite XYZ vector")
    return value


def assign_xyz(destination, values):
    destination.x, destination.y, destination.z = map(float, values)


class FirmwarePIDController(Controller):
    """Call the unmodified C PID once per 1 ms simulation step.

    Normal target: position_m (world XYZ, m), optional yaw_rad (RH yaw).
    Sign-test/manual target: attitude_legacy_deg (roll, inverted pitch, yaw)
    and collective_command (firmware units), with position control disabled.
    Output motor_commands contains raw, uncapped M1..M4 force commands, NOT PWM.
    """

    def __init__(self, dt_s=0.001, gravity_m_s2=9.81, gyro_filter_hz=GYRO_LPF_CUTOFF_HZ):
        try:
            import cffirmware
        except ImportError as exc:
            raise RuntimeError("Build cffirmware with Isaac Sim Python 3.11 and add build/ to PYTHONPATH") from exc
        self.cf = cffirmware
        if not math.isclose(dt_s, 1.0 / cffirmware.RATE_MAIN_LOOP, rel_tol=0, abs_tol=1e-12):
            raise ValueError("Firmware PID requires a 1 kHz physics/control loop")
        if not math.isfinite(gravity_m_s2) or gravity_m_s2 <= 0:
            raise ValueError("gravity_m_s2 must be positive")
        self.dt_s, self.gravity_m_s2 = dt_s, gravity_m_s2
        if not math.isfinite(gyro_filter_hz) or not 0 <= gyro_filter_hz < .5/dt_s:
            raise ValueError("gyro_filter_hz must be below Nyquist; zero bypasses it for diagnosis")
        self.gyro_filter_hz = gyro_filter_hz
        self.reset()

    def reset(self):
        cf = self.cf
        cf.controllerPidInit()
        self.firmware_state = cf.state_t()
        self.sensors = cf.sensorData_t()
        self.setpoint = cf.setpoint_t()
        self.control = cf.control_t()  # Keep this object across odd/even ticks.
        self.motors = cf.motors_thrust_uncapped_t()
        self._steps = 0
        self._epoch = None
        self._previous_velocity = None
        self._reset_at_pose = True
        self._gyro_filter = (FirmwareGyroFilter(1/self.dt_s, self.gyro_filter_hz)
                             if self.gyro_filter_hz else None)

    def _update_state(self, state):
        """All conversions mirror kalmanCoreExternalizeState, without running EKF."""
        if "firmware_state" in state:
            # EKF path: consume C's externalized state and the sensor driver's
            # already-filtered IMU directly. No ground-truth reconstruction,
            # second gyro filter, Euler re-conversion, or velocity differentiation.
            self.firmware_state = state["firmware_state"]
            self.sensors = state["sensor_data"]
            self.gyro_body_deg_s = np.array([self.sensors.gyro.x, self.sensors.gyro.y, self.sensors.gyro.z])
            attitude = self.firmware_state.attitude
            return np.array([attitude.roll, attitude.pitch, attitude.yaw])
        cf_state, sensors = self.firmware_state, self.sensors
        position = vector3(state["position_m"])
        velocity = vector3(state["linear_velocity_m_s"])
        omega_world = vector3(state["angular_velocity_rad_s"])
        rotation, rpy = rotation_and_rpy(state["orientation_wxyz"])
        q = np.asarray(state["orientation_wxyz"], dtype=float)
        q = q / np.linalg.norm(q)

        # PhysX and firmware share a Z-up world and x-forward/y-left/z-up body.
        # Position and linear velocity stay in WORLD m and m/s, not body frame.
        # src/modules/src/kalman_core/kalman_core.c:774-788.
        assign_xyz(cf_state.position, position)
        assign_xyz(cf_state.velocity, velocity)
        # Legacy Euler pitch alone is inverted. Quaternion is NOT reflected.
        # kalman_core.c:799-818; stabilizer_types.h:70-85,177-182.
        legacy_deg = np.degrees(rpy) * np.array([1, -1, 1])
        cf_state.attitude.roll, cf_state.attitude.pitch, cf_state.attitude.yaw = map(float, legacy_deg)
        cf_state.attitudeQuaternion.w = float(q[0])
        cf_state.attitudeQuaternion.x = float(q[1])
        cf_state.attitudeQuaternion.y = float(q[2])
        cf_state.attitudeQuaternion.z = float(q[3])
        # PhysX angular velocity is WORLD rad/s. Sensor gyro needs BODY deg/s.
        # Do NOT invert gyro.y here: controller_pid.c:142 does that internally.
        gyro = np.degrees(rotation.T @ omega_world)
        # sensorData.gyro in real firmware is ALREADY low-pass filtered by its
        # BMI088 driver at 1 kHz/80 Hz. Preserve this deterministic preprocessing.
        # state_t remains unfiltered ground truth. No EKF, sensor noise or bias.
        self.gyro_body_deg_s = self._gyro_filter.apply(gyro) if self._gyro_filter else gyro
        assign_xyz(sensors.gyro, self.gyro_body_deg_s)

        # Noise-free acceleration from successive ground-truth velocities.
        # First sample has no history: initialize inertial acceleration to zero.
        # The C PID uses gyro for feedback; accelerometer is only logged there.
        acceleration = (np.zeros(3) if self._previous_velocity is None else
                        (velocity - self._previous_velocity) / self.dt_s)
        self._previous_velocity = velocity.copy()
        # Accelerometer measures specific force R^T(a - g_world), in Gs.
        specific_world_g = acceleration / self.gravity_m_s2 + [0, 0, 1]
        assign_xyz(sensors.acc, rotation.T @ specific_world_g)
        # Firmware state.acc is world acceleration in Gs, gravity removed:
        # kalman_core.c:790-796. Unused mag/baro fields remain zero.
        assign_xyz(cf_state.acc, acceleration / self.gravity_m_s2)
        now_ms = int(round(float(state["time_s"]) * 1000)) & 0xFFFFFFFF
        for quantity in (cf_state.position, cf_state.velocity, cf_state.acc, cf_state.attitude):
            quantity.timestamp = now_ms
        sensors.interruptTimestamp = int(round(float(state["time_s"]) * 1e6))
        return legacy_deg

    def compute(self, state: dict, target: dict) -> dict:
        cf = self.cf
        now = float(state["time_s"])
        if not math.isfinite(now) or now < 0:
            raise ValueError("Simulation time must be finite and nonnegative")
        if self._epoch is None:
            self._epoch = now
        if not math.isclose(now, self._epoch + self._steps * self.dt_s, rel_tol=0, abs_tol=1e-8):
            raise ValueError("Call firmware compute exactly once per 1 ms physics step; reset on rewind")
        legacy_deg = self._update_state(state)

        if self._reset_at_pose:
            # controllerPidInit alone does NOT reset all static attitude state.
            # A discarded, zero-thrust call invokes the firmware reset path at
            # the actual pose (controller_pid.c:163-178). No force/time advances.
            zero = cf.setpoint_t()
            zero.thrust = 0
            cf.controllerPid(self.control, zero, self.sensors, self.firmware_state, 0)
            self._reset_at_pose = False

        sp = self.setpoint
        # Clear all mode flags, so manual sign tests and position flight can switch.
        for axis in ("x", "y", "z", "roll", "pitch", "yaw", "quat"):
            setattr(sp.mode, axis, cf.modeDisable)
        sp.timestamp = self.firmware_state.position.timestamp
        sp.velocity_body = False
        if "attitude_legacy_deg" in target:
            angles = vector3(target["attitude_legacy_deg"])
            collective = float(target["collective_command"])
            if not math.isfinite(collective) or not 0 <= collective <= 65535:
                raise ValueError("collective_command must be in [0,65535]")
            sp.attitude.roll, sp.attitude.pitch, sp.attitude.yaw = map(float, angles)
            sp.thrust = collective
        else:
            assign_xyz(sp.position, vector3(target["position_m"]))
            sp.mode.x = sp.mode.y = sp.mode.z = cf.modeAbs
            yaw = float(target.get("yaw_rad", 0.0))
            if not math.isfinite(yaw):
                raise ValueError("yaw_rad must be finite")
            sp.attitude.yaw = math.degrees(math.atan2(math.sin(yaw), math.cos(yaw)))
        # Absolute yaw is RH-positive about +Z. This bypasses CRTP packet-level
        # yaw negation; controller_pid.c:104-109,149 performs its own mixing sign.
        sp.mode.yaw = cf.modeAbs

        # The C code gates position on tick % 10, attitude/rate on tick % 2.
        # stabilizer_types.h:363-377; controller_pid.c:85,115,119.
        tick = self._steps & 0xFFFFFFFF
        cf.controllerPid(self.control, sp, self.sensors, self.firmware_state, tick)
        if self.control.controlMode != cf.controlModeLegacy:
            raise RuntimeError("Expected controllerPid to produce legacy control")
        cf.powerDistribution(self.control, self.motors)
        self._steps += 1
        return {
            "motor_commands": np.array([getattr(self.motors.motors, f"m{i}") for i in range(1, 5)], dtype=np.int32),
            "control_legacy": (float(self.control.thrust), self.control.roll, self.control.pitch, self.control.yaw),
            "attitude_legacy_deg": legacy_deg,
            "gyro_body_deg_s": self.gyro_body_deg_s.copy(),
            "tick": tick,
        }
