"""Fault/driver contracts and both shuttle schedules; no Isaac app or hardware."""

import csv
from dataclasses import replace
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from flow_faults import FlowFault, FlowFaultConfig, add_flow_arguments, config_from_args
try:
    import cffirmware
except ImportError:
    cffirmware = None


class FaultConfigTests(unittest.TestCase):
    def test_validation_and_cli_defaults_and_sweep(self):
        import argparse
        parser = argparse.ArgumentParser()
        add_flow_arguments(parser, sweep=True)
        args = parser.parse_args([])
        self.assertEqual(config_from_args(args), FlowFaultConfig())
        args = parser.parse_args("--trajectory flow-shuttle-x --flow-fault spike --flow-fault-offset 6 --flow-fault-duration 4 --flow-spike-amplitudes 5 10 20 50".split())
        self.assertEqual(args.flow_spike_amplitudes, [5, 10, 20, 50])
        self.assertEqual(config_from_args(args, hover_hold_s=5).start_s, 26)
        for kwargs in ({"kind": "bad"}, {"seed": -1}, {"start_s": math.nan},
                       {"duration_s": 0}, {"spike_amplitude": 100}, {"spike_probability": 2},
                       {"scale_factor": 0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                FlowFaultConfig(**kwargs)

    def test_fault_arithmetic_and_half_open_window(self):
        ideal, noise = np.array([2., -3.]), np.array([.4, -.7])
        clean = ideal+noise
        for kind in ("zero", "dropout", "scale", "spike"):
            model = FlowFault(FlowFaultConfig(kind, start_s=.01, duration_s=.02, spike_probability=1), 7)
            for ms in (9, 10, 29, 30):
                counts, status, clipped, labels = model.apply(ms, ideal, clean, noise)
                active = 10 <= ms < 30
                self.assertEqual(labels["fault_active"], active)
                if not active:
                    np.testing.assert_array_equal(counts, clean)
                    self.assertEqual(status, 0xB0)
                elif kind == "zero":
                    np.testing.assert_array_equal(counts, [0, 0])
                    self.assertEqual(status, 0xB0)
                elif kind == "dropout":
                    np.testing.assert_array_equal(counts, clean)
                    self.assertEqual(status, 0x30)
                    self.assertFalse(labels["fault_changed_x"])
                elif kind == "scale":
                    np.testing.assert_array_equal(counts, .5*ideal+noise)
                else:
                    np.testing.assert_allclose(np.abs(counts-clean), 20)

    def test_spike_clip_probability_and_reproducible_schedule(self):
        config = FlowFaultConfig("spike", start_s=0, duration_s=10, spike_probability=.5)
        a, b = FlowFault(config, 7), FlowFault(config, 7)
        different = FlowFault(replace(config, seed=8), 7)
        schedule, other = [], []
        for ms in range(10, 1010, 10):
            ra = a.apply(ms, np.zeros(2), np.array([99., -99.]), np.array([99., -99.]))
            rb = b.apply(ms, np.zeros(2), np.zeros(2), np.zeros(2))
            self.assertEqual(ra[3]["fault_active"], rb[3]["fault_active"])
            if ra[3]["fault_active"]:
                self.assertTrue(np.all(np.abs(ra[0]) <= 99))
            schedule.append(ra[3]["fault_active"])
            other.append(different.apply(ms, np.zeros(2), np.zeros(2), np.zeros(2))[3]["fault_active"])
        self.assertNotEqual(schedule, other)
        self.assertTrue(any(schedule) and not all(schedule))
        none = FlowFault(replace(config, spike_probability=0), 7)
        self.assertFalse(none.apply(10, np.zeros(2), np.zeros(2), np.zeros(2))[3]["fault_active"])


@unittest.skipIf(cffirmware is None, "Use Isaac Python with cffirmware")
class FlowIntegrationTests(unittest.TestCase):
    @staticmethod
    def truth(ms, velocity=(.4, -.2, 0)):
        return dict(time_s=ms/1000, position_m=(0, 0, 1), orientation_wxyz=(1, 0, 0, 0),
                    linear_velocity_m_s=velocity, angular_velocity_rad_s=(0, 0, 0))

    def test_noise_rng_independence_all_faults_and_reset(self):
        from sensors import SensorSuite
        suites = [SensorSuite(noise=True, seed=7, flow_audit=True,
                              flow_fault=FlowFaultConfig(kind, start_s=.01, duration_s=.08, spike_probability=1))
                  for kind in ("off", "spike", "zero", "dropout", "scale")]
        first = []
        for ms in range(101):
            packets = [s.sample(self.truth(ms)) for s in suites]
            for i in range(1, len(suites)):
                for a, b in zip([p for p in packets[0] if p.kind != "flow"],
                                [p for p in packets[i] if p.kind != "flow"]):
                    self.assertEqual(a.kind, b.kind)
                    np.testing.assert_array_equal(a.value, b.value)
                self.assertEqual(suites[0].rng.bit_generator.state, suites[i].rng.bit_generator.state)
                for a, b in zip(suites[0].flow_samples, suites[i].flow_samples):
                    for axis in "xy":
                        self.assertEqual(a[f"noise_{axis}_counts"], b[f"noise_{axis}_counts"])
            first.extend(suites[1].flow_samples)
        suites[1].reset()
        second = []
        for ms in range(101):
            suites[1].sample(self.truth(ms))
            second.extend(suites[1].flow_samples)
        self.assertEqual(first, second)

    def test_dropout_driver_clock_delay_and_no_fake_updates(self):
        from sensors import SensorSuite
        from estimators.firmware_ekf import FirmwareEKF
        suite = SensorSuite(flow_fault=FlowFaultConfig("dropout", start_s=.01, duration_s=.02),
                            delays_ms={"flow": 7})
        ekf = FirmwareEKF(flow_audit=True, fault_labels=True)
        samples, updates = [], []
        for ms in range(38):
            rows = ekf.step(ms, suite.sample(self.truth(ms)), True)
            samples.extend(suite.flow_samples)
            updates.extend(r for r in rows if r["sensor"] == "flow")
        self.assertEqual([r["enqueued"] for r in samples], [False, False, True])
        self.assertEqual([r["driver_dt_s"] for r in samples], [.01]*3)
        self.assertEqual(ekf.counts["flow"], 1)
        self.assertEqual(len(updates), 1)
        self.assertEqual((updates[0]["sample_time_s"], updates[0]["delivery_time_s"]), (.03, .037))
        self.assertFalse(updates[0]["fault_active"])
        self.assertEqual(suite.flow_drop_reasons, {"count_limit": 0, "motion_status": 2})

    def test_missing_tof_return_cannot_shift_flow_or_imu_noise(self):
        from unittest.mock import patch
        from sensors import SensorSuite
        valid, missing = [SensorSuite(noise=True, seed=7, flow_audit=True) for _ in range(2)]
        for ms in range(101):
            valid.sample(self.truth(ms))
            with patch("sensors.tof_model", return_value=math.nan):
                missing.sample(self.truth(ms))
            self.assertEqual(valid.rng.bit_generator.state, missing.rng.bit_generator.state)
            np.testing.assert_array_equal(valid.latest_gyro, missing.latest_gyro)
            np.testing.assert_array_equal(valid.latest_acc, missing.latest_acc)
            self.assertEqual(valid.flow_samples, missing.flow_samples)
        self.assertEqual(missing.dropped["tof"], 4)
        self.assertEqual(valid.generated["tof"], 4)

    def test_delayed_labels_belong_to_sample_not_delivery(self):
        from sensors import SensorSuite
        suite = SensorSuite(flow_fault=FlowFaultConfig("zero", start_s=.01, duration_s=.01),
                            delays_ms={"flow": 15})
        flows = []
        for ms in range(36):
            flows.extend(p for p in suite.sample(self.truth(ms)) if p.kind == "flow")
        self.assertEqual([p.flow_labels["fault_active"] for p in flows], [True, False])
        self.assertEqual([p.delivery_ms for p in flows], [25, 35])

    def test_count_limit_strict_and_count_rejection_clock(self):
        from unittest.mock import patch
        from sensors import SensorSuite
        for value, accepted in ((99., True), (100., False), (-99., True), (-100., False)):
            suite = SensorSuite(flow_audit=True)
            for ms in range(11):
                with patch("sensors.flow_model", return_value=np.array([value, 0.])):
                    suite.sample(self.truth(ms))
            self.assertEqual(suite.flow_samples[0]["enqueued"], accepted)
            for ms in range(11, 21):
                suite.sample(self.truth(ms))
            self.assertEqual(suite.flow_samples[0]["driver_dt_s"], .01 if accepted else .02)

    def test_labels_do_not_change_estimator_and_off_audit_preserves_csv(self):
        from controllers.ekf_pid import EKFPIDController
        from flight_demo import FlightLog
        from estimators.firmware_ekf import read_float_array
        from actuators import MotorActuator
        pipelines = [EKFPIDController("ekf_shadow", noise=True, flow_audit=a) for a in (False, True)]
        # Same values with different labels must produce identical C state/covariance.
        with tempfile.TemporaryDirectory() as tmp:
            paths = [Path(tmp)/f"{i}.csv" for i in range(2)]
            logs = [FlightLog(p) for p in paths]
            for pipeline, log in zip(pipelines, logs):
                pipeline.reset()  # Firmware PID has process-global state.
                actuator = MotorActuator(None)
                for ms in range(101):
                    truth = self.truth(ms)
                    target = {"position_m": (0, 0, 1)}
                    command = pipeline.compute(truth, target)
                    command.update(actuator.convert(command["motor_commands"]))
                    log.write(truth, target, command, "hover")
            for log in logs:
                log.close()
            for suffix in (".csv", ".measurements.csv"):
                self.assertEqual(paths[0].with_suffix(suffix).read_bytes(), paths[1].with_suffix(suffix).read_bytes())
            self.assertFalse(paths[0].with_suffix(".flow_samples.csv").exists())
            self.assertTrue(paths[1].with_suffix(".flow_samples.csv").exists())
        np.testing.assert_array_equal(read_float_array(pipelines[0].ekf.core.P, (9, 9)),
                                      read_float_array(pipelines[1].ekf.core.P, (9, 9)))

    def test_fault_measurement_schema_and_audit_join(self):
        from controllers.ekf_pid import EKFPIDController
        from flight_demo import FlightLog
        from actuators import MotorActuator
        pipeline = EKFPIDController("ekf_shadow", flow_fault=FlowFaultConfig("zero", start_s=.01, duration_s=.01))
        actuator = MotorActuator(None)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"flight.csv"
            log = FlightLog(path)
            for ms in range(51):
                truth, target = self.truth(ms), {"position_m": (0, 0, 1)}
                command = pipeline.compute(truth, target)
                command.update(actuator.convert(command["motor_commands"]))
                log.write(truth, target, command, "hover")
            log.close()
            with path.with_suffix(".measurements.csv").open() as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), 7)
            self.assertEqual([r["fault_active"] for r in rows if r["sensor"] == "flow"], ["True"]+["False"]*4)
            self.assertTrue(all(r["fault_type"] == "not_applicable" for r in rows if r["sensor"] == "tof"))
            with path.with_suffix(".flow_updates.csv").open() as file:
                updates = list(csv.DictReader(file))
            self.assertEqual([r["flow_sample_id"] for r in updates], [str(i) for i in range(1, 6)])

    def test_changing_labels_alone_cannot_change_filter(self):
        from sensors import SensorSuite
        from estimators.firmware_ekf import FirmwareEKF, read_float_array
        suite = SensorSuite(noise=True, flow_audit=True)
        cores = [FirmwareEKF(fault_labels=True) for _ in range(2)]
        for ms in range(101):
            packets = suite.sample(self.truth(ms))
            relabeled = [replace(p, flow_labels=dict(p.flow_labels, fault_type="zero", fault_active=True))
                         if p.kind == "flow" else p for p in packets]
            cores[0].step(ms, packets, True)
            cores[1].step(ms, relabeled, True)
            for name, shape in (("S", (9,)), ("P", (9, 9))):
                np.testing.assert_array_equal(read_float_array(getattr(cores[0].core, name), shape),
                                              read_float_array(getattr(cores[1].core, name), shape))

    def test_shuttle_both_axes_endpoints_peak_speed_and_symmetry(self):
        from flight_demo import FlowShuttleDemo
        x, y = FlowShuttleDemo("x"), FlowShuttleDemo("y")
        for t, p in zip(x.knots_s, x.knots_m):
            self.assertEqual(x.reference(t), (p, 0., 0.))
        for t in np.arange(0, 62.001, .01):
            px = x.target({"time_s": t})["position_m"]
            py = y.target({"time_s": t})["position_m"]
            self.assertEqual(px, (py[1], py[0], py[2]))
            self.assertLessEqual(abs(x.reference(t)[1]), .46875+1e-12)
        self.assertAlmostEqual(x.reference(28)[1], -.46875)
        self.assertEqual(x.phase, "complete")
        self.assertGreater(x.time_limit_s, x.end_time_s)
        self.assertEqual(x.target({"time_s": 55})["position_m"], (0, 0, 1))


if __name__ == "__main__":
    unittest.main()
