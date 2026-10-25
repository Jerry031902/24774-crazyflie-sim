"""Compose synthetic sensors, the stock EKF, and the existing C PID/actuator API."""

import numpy as np

from .base import Controller
from .firmware_pid import FirmwarePIDController, assign_xyz
from estimators.firmware_ekf import FirmwareEKF, state_dict
from sensors import SensorSuite, DEFAULT_SEED
from actuators import THRUST_MIN_N, THRUST_MAX_N, COMMAND_MAX

SUPERVISOR_PERIOD_MS = 40  # stabilizer_types.h:375, RATE_SUPERVISOR = 25 Hz.
FLYING_HYSTERESIS_MS = 2000  # supervisor.c:56,278-299.


class EKFPIDController(Controller):
    """A calibrated, armed vehicle; full arming/watchdog state machine omitted.

    Flying status comes from motor activity, never from true altitude. Mirror
    the stock 25 Hz over-idle detection and 2 s hysteresis on the flight path.
    The estimator's own bounds/reset supervisor IS preserved.
    """

    def __init__(self, mode, noise=False, seed=DEFAULT_SEED):
        if mode not in ("ekf_shadow", "ekf_closed_loop"):
            raise ValueError(mode)
        self.mode = mode
        self.suite = SensorSuite(noise=noise, seed=seed)
        self.ekf = FirmwareEKF()
        self.pid = FirmwarePIDController()
        self.gyro_filter_hz = self.pid.gyro_filter_hz
        self.reset()

    def reset(self):
        self.suite.reset()
        self.ekf.reset()
        self.pid.reset()
        self.sensor_data = self.pid.cf.sensorData_t()
        self.previous_motors_running = False
        self.last_thrust_ms = 0
        self.is_flying = False

    def compute(self, truth, target):
        ms = round(float(truth["time_s"])*1000)
        packets = self.suite.sample(truth)
        assign_xyz(self.sensor_data.gyro, self.suite.latest_gyro)
        assign_xyz(self.sensor_data.acc, self.suite.latest_acc)
        self.sensor_data.interruptTimestamp = self.suite.latest_imu_sample_ms * 1000

        # estimatorKalman copies the OLD task output before waking the lower
        # priority worker (estimator_kalman.c:289-300; config.h:65,93).
        used = self.ekf.external_state
        if self.mode == "ekf_closed_loop":
            controller_input = {"time_s": truth["time_s"], "firmware_state": used,
                                "sensor_data": self.sensor_data}
        else:
            controller_input = truth  # Existing noise-free GT controller unchanged.
        command = self.pid.compute(controller_input, target)

        # Stock flight-condition check runs before new motor commands are output.
        if ms % SUPERVISOR_PERIOD_MS == 0:
            if self.previous_motors_running:
                self.last_thrust_ms = ms
            self.is_flying = bool(self.last_thrust_ms and ms-self.last_thrust_ms < FLYING_HYSTERESIS_MS)
        # Over-idle PWM is equivalent to nonzero ideal motor force in this model.
        self.previous_motors_running = bool(np.max(command["motor_commands"])
                                           * THRUST_MAX_N / COMMAND_MAX >= THRUST_MIN_N)

        # Worker gets CPU once the stabilizer has finished, before the next tick.
        innovations = self.ekf.step(ms, packets, self.is_flying)
        command.update(estimate=state_dict(self.ekf.external_state),
                       estimate_used=state_dict(used), ekf_innovations=innovations,
                       imu_acc_G=self.suite.latest_acc.copy(), imu_gyro_deg_s=self.suite.latest_gyro.copy(),
                       ekf_reset_count=self.ekf.reset_count, quad_is_flying=self.is_flying)
        return command
