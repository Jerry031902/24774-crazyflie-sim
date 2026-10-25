"""Sensor/unit/timing tests against the compiled C core, no Isaac app needed."""

import ctypes
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    import cffirmware as cf
except ImportError:
    cf = None
if cf is not None:
    import numpy as np
    from sensors import SensorSuite, Measurement, flow_model, tof_model, tof_stock_std
    from estimators.firmware_ekf import FirmwareEKF, read_float_array
    from controllers.ekf_pid import EKFPIDController
    from controllers.firmware_pid import rotation_and_rpy


@unittest.skipIf(cf is None, "Use Isaac Sim Python with built cffirmware")
class EKFTests(unittest.TestCase):
    @staticmethod
    def truth(ms, q=(1, 0, 0, 0), omega=(0, 0, 0), velocity=(0, 0, 0)):
        return {"time_s": ms/1000, "position_m": (0, 0, 1), "orientation_wxyz": q,
                "angular_velocity_rad_s": omega, "linear_velocity_m_s": velocity}

    def test_rates_fifo_and_noise_off_retains_stock_uncertainty(self):
        suite, ekf = SensorSuite(), FirmwareEKF()
        at_50 = None
        for ms in range(1001):
            packets = suite.sample(self.truth(ms))
            if ms == 50:
                at_50 = [p.kind for p in packets]
            for p in packets:
                if p.kind in ("flow", "tof"):
                    self.assertGreater(p.std, 0)
            ekf.step(ms, packets, True)
        self.assertEqual(at_50, ["gyro", "acc", "flow", "tof"])
        self.assertEqual(suite.generated, {"gyro": 1001, "acc": 1001, "flow": 100, "tof": 40})
        self.assertEqual(ekf.counts["predict"], 101)  # Includes t=0, dt=0.
        self.assertEqual(ekf.counts["process_noise"], 1001)
        self.assertEqual(ekf.counts["externalize"], 1001)
        self.assertAlmostEqual(ekf.external_state.position.z, 1, places=3)
        self.assertAlmostEqual(ekf.acc_sub.subSample.z, 9.81, places=5)
        self.assertEqual(ekf.reset_count, 0)

    def test_stock_call_order_and_previous_samples(self):
        suite, ekf = SensorSuite(), FirmwareEKF()
        calls = []
        names = ["axis3fSubSamplerFinalize", "kalmanCorePredict", "kalmanCoreAddProcessNoise",
                 "axis3fSubSamplerAccumulate", "kalmanCoreUpdateWithFlow", "kalmanCoreFinalize", "kalmanCoreExternalizeState"]
        from contextlib import ExitStack
        with ExitStack() as stack:
            for name in names:
                original = getattr(cf, name)
                def wrapped(*args, _name=name, _original=original):
                    calls.append(_name)
                    return _original(*args)
                stack.enter_context(patch.object(cf, name, wrapped))
            for ms in range(11):
                calls.clear()
                ekf.step(ms, suite.sample(self.truth(ms)), True)
        self.assertEqual(calls, ["axis3fSubSamplerFinalize"]*2 + ["kalmanCorePredict", "kalmanCoreAddProcessNoise"]
                         + ["axis3fSubSamplerAccumulate"]*2 + ["kalmanCoreUpdateWithFlow", "kalmanCoreFinalize", "kalmanCoreExternalizeState"])
        self.assertEqual(ekf.acc_sub.count, 1)  # t=10 sample awaits NEXT prediction.

    def test_imu_body_units_specific_force_and_filter(self):
        suite = SensorSuite()
        q = (math.sqrt(.5), 0, 0, math.sqrt(.5))
        suite.sample(self.truth(0, q=q, omega=(1, 2, 3)))
        np.testing.assert_allclose(suite.latest_gyro, np.degrees([2, -1, 3]), rtol=2e-6)
        np.testing.assert_allclose(suite.latest_acc, [0, 0, 1], atol=1e-6)
        for ms in range(1, 201):
            suite.sample(self.truth(ms, q=q, velocity=(0, 0, -9.81*ms/1000)))
        np.testing.assert_allclose(suite.latest_acc, [0, 0, 0], atol=1e-5)

    def test_delay_timestamps_and_noise_repeatability(self):
        suite = SensorSuite(noise=True, seed=5, delays_ms={"flow": 7})
        recorded = []
        for ms in range(18):
            packets = suite.sample(self.truth(ms))
            recorded.append(suite.latest_gyro.copy())
            flows = [p for p in packets if p.kind == "flow"]
            if ms < 17:
                self.assertFalse(flows)
            else:
                self.assertEqual((flows[0].sample_ms, flows[0].delivery_ms, flows[0].dt_s), (10, 17, .01))
        suite.reset()
        for ms in range(18):
            suite.sample(self.truth(ms))
            np.testing.assert_array_equal(suite.latest_gyro, recorded[ms])

    def test_true_model_measurements_have_zero_c_core_correction(self):
        ekf = FirmwareEKF()
        q = (math.cos(.15), 0, math.sin(.15), 0)
        r, _ = rotation_and_rpy(q)
        # Test fixture only: fill the C state with a known truth. Production EKF
        # initialization never reads truth or writes these arrays.
        s = np.ctypeslib.as_array((ctypes.c_float*9).from_address(int(ekf.core.S)))
        rr = np.ctypeslib.as_array((ctypes.c_float*9).from_address(int(ekf.core.R))).reshape(3, 3)
        s[:6] = [0, 0, 1.2, .23, -.11, 0]
        rr[:] = r
        before = s.copy()
        omega = np.array([.2, -.3, .1])
        gyro = cf.Axis3f()
        gyro.x, gyro.y, gyro.z = map(float, np.degrees(omega))
        flow = cf.flowMeasurement_t()
        flow.dt = .01
        flow.stdDevX = flow.stdDevY = 2
        flow.dpixelx, flow.dpixely = map(float, flow_model(s[2], rr, s[3:6], omega, .01))
        cf.kalmanCoreUpdateWithFlow(ekf.core, flow, gyro)
        np.testing.assert_allclose(s, before, atol=5e-6)
        tof = cf.tofMeasurement_t()
        tof.distance = tof_model(s[2], rr[2, 2])
        tof.stdDev = tof_stock_std(tof.distance)
        before = s.copy()
        cf.kalmanCoreUpdateWithTof(ekf.core, tof)
        np.testing.assert_allclose(s, before, atol=5e-6)

    def test_stock_high_tilt_tof_check_and_no_new_innovation_gate(self):
        ekf = FirmwareEKF()
        matrix = np.ctypeslib.as_array((ctypes.c_float*9).from_address(int(ekf.core.R))).reshape(3, 3)
        matrix[2, 2] = .05
        before = read_float_array(ekf.core.S, (9,))
        tof = cf.tofMeasurement_t()
        tof.distance, tof.stdDev = 1., .003
        cf.kalmanCoreUpdateWithTof(ekf.core, tof)
        np.testing.assert_array_equal(read_float_array(ekf.core.S, (9,)), before)
        # A large innovation at valid attitude is passed directly to stock C.
        matrix[2, 2] = 1.
        cf.kalmanCoreUpdateWithTof(ekf.core, tof)
        self.assertGreater(read_float_array(ekf.core.S, (9,))[2], .99)

    def test_closed_loop_pid_receives_only_estimator_and_imu(self):
        pipeline = EKFPIDController("ekf_closed_loop")
        with patch.object(pipeline.pid, "compute", wraps=pipeline.pid.compute) as compute:
            command = pipeline.compute(self.truth(0), {"position_m": (0, 0, 1)})
            supplied = compute.call_args.args[0]
            self.assertEqual(set(supplied), {"time_s", "firmware_state", "sensor_data"})
            self.assertEqual(supplied["firmware_state"].position.z, 0.)
            self.assertEqual(command["estimate_used"]["position_m"][2], 0.)
            self.assertAlmostEqual(supplied["sensor_data"].acc.z, 1., places=5)


if __name__ == "__main__":
    unittest.main()
