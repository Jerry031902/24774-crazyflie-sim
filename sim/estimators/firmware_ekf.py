"""Stock kalmanCore scheduling with Flow/ToF. No custom measurement gating.

Mirrors src/modules/src/estimator/estimator_kalman.c:214-366. Ground truth is
deliberately absent from this module's interface. See ../BASELINE_EKF.md.
"""

import ctypes
import math
import numpy as np

from sensors import FLOW_RESOLUTION, flow_model, tof_model
from controllers.firmware_pid import assign_xyz

PREDICTION_PERIOD_MS = 10  # estimator_kalman.c:119-120.
MAX_POSITION_M = 100.0    # kalman_supervisor.c:34-56, not exposed by SWIG.
MAX_BODY_VELOCITY_M_S = 10.0


def read_float_array(pointer, shape):
    """Copy a SWIG-exposed C float array, never retain borrowed views or write it.

    Shapes are public fields of kalmanCoreData_t in kalman_core.h:82-91.
    No complete-struct ABI layout or padding assumptions are needed.
    """
    length = math.prod(shape)
    data = (ctypes.c_float * length).from_address(int(pointer))
    return np.ctypeslib.as_array(data).copy().reshape(shape)


def state_dict(state):
    return {"position_m": np.array([state.position.x, state.position.y, state.position.z]),
            "linear_velocity_m_s": np.array([state.velocity.x, state.velocity.y, state.velocity.z]),
            "orientation_wxyz": np.array([state.attitudeQuaternion.w, state.attitudeQuaternion.x,
                                           state.attitudeQuaternion.y, state.attitudeQuaternion.z]),
            "attitude_legacy_deg": np.array([state.attitude.roll, state.attitude.pitch, state.attitude.yaw])}


class FirmwareEKF:
    def __init__(self):
        import cffirmware
        self.cf = cffirmware
        self.reset()

    def reset(self):
        cf = self.cf
        self.core = cf.kalmanCoreData_t()
        self.params = cf.kalmanCoreParams_t()
        cf.kalmanCoreDefaultParams(self.params)
        # Flow v2 requests Kalman, but DOES NOT request attitudeReversionOff.
        # Keep default initial XYZ/yaw and covariance, no truth initialization.
        self.acc_sub = cf.Axis3fSubSampler_t()
        self.gyro_sub = cf.Axis3fSubSampler_t()
        self.acc_latest, self.gyro_latest = cf.Axis3f(), cf.Axis3f()
        self._init_core(0)
        self.next_ms = self.next_prediction_ms = 0
        self.reset_pending = False
        self.reset_count = 0
        self.counts = dict.fromkeys(("predict", "process_noise", "finalize_calls", "finalized", "externalize", "gyro", "acc", "flow", "tof"), 0)
        # A valid initial quaternion for diagnostics. PID sees the same zero
        # Euler/position/velocity as the stock initially zero task-state buffer.
        self.external_state = cf.state_t()
        cf.kalmanCoreExternalizeState(self.core, self.external_state, self.acc_latest)

    def _init_core(self, ms):
        cf = self.cf
        cf.axis3fSubSamplerInit(self.acc_sub, 9.81)
        cf.axis3fSubSamplerInit(self.gyro_sub, math.pi/180)
        cf.kalmanCoreInit(self.core, self.params, ms)

    def step(self, now_ms, packets, quad_is_flying):
        if now_ms != self.next_ms:
            raise ValueError("Run estimator task once per millisecond from t=0")
        self.next_ms += 1
        cf = self.cf
        if self.reset_pending:
            self._init_core(now_ms)
            self.reset_pending = False
            self.reset_count += 1
        # Prediction uses samples accumulated during PREVIOUS task iterations.
        # Source uses >= and next = now + interval, unlike the older emulator.
        if now_ms >= self.next_prediction_ms:
            cf.axis3fSubSamplerFinalize(self.acc_sub)
            cf.axis3fSubSamplerFinalize(self.gyro_sub)
            cf.kalmanCorePredict(self.core, self.params, self.acc_sub.subSample,
                                 self.gyro_sub.subSample, now_ms, bool(quad_is_flying))
            self.next_prediction_ms = now_ms + PREDICTION_PERIOD_MS
            self.counts["predict"] += 1
        cf.kalmanCoreAddProcessNoise(self.core, self.params, now_ms)
        self.counts["process_noise"] += 1

        innovations = []
        for packet in packets:
            if packet.delivery_ms > now_ms:
                raise ValueError("Cannot consume a measurement before arrival")
            kind = packet.kind
            self.counts[kind] += 1
            if kind in ("gyro", "acc"):
                latest = self.gyro_latest if kind == "gyro" else self.acc_latest
                assign_xyz(latest, packet.value)
                sub = self.gyro_sub if kind == "gyro" else self.acc_sub
                cf.axis3fSubSamplerAccumulate(sub, latest)
                continue

            # Diagnostic prediction uses the state immediately before this
            # measurement. It NEVER decides whether to call the C update.
            s = read_float_array(self.core.S, (cf.KC_STATE_DIM,))
            rotation = read_float_array(self.core.R, (3, 3))
            row = {"time_s": now_ms/1000, "sensor": kind,
                   "sample_time_s": packet.sample_ms/1000,
                   "delivery_time_s": packet.delivery_ms/1000, "dt_s": packet.dt_s,
                   "measured_x": float(packet.value[0]), "measured_y": math.nan,
                   "predicted_x": math.nan, "predicted_y": math.nan,
                   "innovation_x": math.nan, "innovation_y": math.nan,
                   "std_dev": packet.std, "used_by_core": True}
            if kind == "flow":
                gyro = np.array([self.gyro_latest.x, self.gyro_latest.y, self.gyro_latest.z])
                predicted_counts = flow_model(s[cf.KC_STATE_Z], rotation,
                                               s[cf.KC_STATE_PX:cf.KC_STATE_PZ+1],
                                               np.radians(gyro), packet.dt_s)
                # Log in the C update's pixel units, not its x10 interface counts.
                measured = packet.value * FLOW_RESOLUTION
                predicted = predicted_counts * FLOW_RESOLUTION
                row.update(measured_x=measured[0], measured_y=measured[1],
                           predicted_x=predicted[0], predicted_y=predicted[1],
                           innovation_x=measured[0]-predicted[0], innovation_y=measured[1]-predicted[1],
                           std_dev=packet.std*FLOW_RESOLUTION)
                measurement = cf.flowMeasurement_t()
                measurement.timestamp = packet.sample_ms
                measurement.dpixelx, measurement.dpixely = map(float, packet.value)
                measurement.dt = packet.dt_s
                measurement.stdDevX = measurement.stdDevY = packet.std
                cf.kalmanCoreUpdateWithFlow(self.core, measurement, self.gyro_latest)
            elif kind == "tof":
                predicted = tof_model(s[cf.KC_STATE_Z], rotation[2, 2])
                row.update(predicted_x=predicted, innovation_x=packet.value[0]-predicted,
                           used_by_core=bool(rotation[2, 2] > .1))
                measurement = cf.tofMeasurement_t()
                measurement.timestamp = packet.sample_ms
                measurement.distance, measurement.stdDev = float(packet.value[0]), packet.std
                # Call C even at high tilt: its EXISTING attitude check decides.
                cf.kalmanCoreUpdateWithTof(self.core, measurement)
            else:
                raise ValueError(f"Unsupported measurement {kind}")
            innovations.append(row)

        self.counts["finalize_calls"] += 1
        self.counts["finalized"] += int(cf.kalmanCoreFinalize(self.core))
        s = read_float_array(self.core.S, (cf.KC_STATE_DIM,))
        # Same check and delayed reset as kalman_supervisor.c / task:268-269.
        self.reset_pending = bool(np.any(np.abs(s[:3]) > MAX_POSITION_M)
                                  or np.any(np.abs(s[3:6]) > MAX_BODY_VELOCITY_M_S))
        self.external_state = cf.state_t()  # Preserve snapshots held by controller.
        cf.kalmanCoreExternalizeState(self.core, self.external_state, self.acc_latest)
        self.counts["externalize"] += 1
        return innovations
