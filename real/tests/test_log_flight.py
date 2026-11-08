"""Offline fault injection. No CRTP init, link opening, arming, or USB access."""

import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import log_flight as app
from cflib.crazyflie.log import Log, LogTocElement
from cflib.utils.callbacks import Caller


class Clock:
    def __init__(self):
        self.now = 0.0
    def __call__(self):
        return self.now
    def sleep(self, seconds):
        self.now += max(seconds, 1e-6)


def fake_cf(missing=None):
    cf = SimpleNamespace(link=object(), connection_lost=Caller(), disconnected=Caller(),
                         commander=Mock(), supervisor=Mock(), close_link=Mock(), is_connected=lambda: True,
                         platform=SimpleNamespace(get_protocol_version=lambda: 11),
                         param=SimpleNamespace(values={"firmware": {"revision0": "123"}}))
    cf.add_port_callback = Mock()
    cf.log = Log(crazyflie=cf)
    elements = {}
    for name in sum({**app.LOG_BLOCKS, **app.RANGER_LOG_BLOCK, **app.LQR_LOG_BLOCKS}.values(), []):
        ctype = "float"
        if name in ("motion.deltaX", "motion.deltaY"):
            ctype = "int16_t"
        elif name.startswith(("motor.", "range.")) or name in ("motion.shutter", "supervisor.info"):
            ctype = "uint16_t"
        elif name in ("motion.motion", "motion.squal", "motion.outlierCount"):
            ctype = "uint8_t"
        elif name in ("hlqr.enabled", "hlqr.mode", "hlqr.saturated"):
            ctype = "uint8_t"
        elif name in ("hlqr.updates", "hlqr.dtUs", "hlqr.execUs", "hlqr.maxExecUs"):
            ctype = "uint32_t"
        elements[name] = SimpleNamespace(ctype=ctype)
    cf.log.toc = SimpleNamespace(get_element_by_complete_name=lambda name: None if name == missing else elements.get(name))
    return cf


def emit(config, timestamp=1000, overrides=None):
    data = {v.name: 0.0 for v in config.variables}
    data.update(overrides or {})
    config.data_received_cb.call(timestamp, data, config)


class LoggingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.cf, self.clock = fake_cf(), Clock()
        self.rec = app.Recorder(self.cf, Path(self.temp.name), 10, clock=self.clock)
        self.starts = self.start_patch(patch.object(app.LogConfig, "start", autospec=True, side_effect=emit))
        self.stops = self.start_patch(patch.object(app.LogConfig, "stop"))
        self.deletes = self.start_patch(patch.object(app.LogConfig, "delete"))
        self.addCleanup(self.rec.close)

    def start_patch(self, patcher):
        result = patcher.start()
        self.addCleanup(patcher.stop)
        return result

    def test_native_types_payloads_and_separate_files(self):
        self.rec.start()
        self.rec.wait_ready()
        self.assertEqual([b["payload_bytes"] for b in self.rec.blocks.values()], [24, 24, 16, 23, 14])
        self.rec.close()
        for name, variables in app.LOG_BLOCKS.items():
            with (Path(self.temp.name) / f"{name}.csv").open() as file:
                rows = list(csv.reader(file))
            self.assertEqual(rows[0][4:], variables)
            self.assertEqual(len(rows), 2)

    def test_lqr_blocks_fit_radio_packet_and_missing_firmware_fails_before_start(self):
        self.rec.close()
        self.rec = app.Recorder(self.cf, Path(self.temp.name), 10, clock=self.clock, lqr=True)
        self.rec.start()
        self.assertEqual(self.rec.blocks["lqr"]["payload_bytes"], 19)
        self.assertEqual(self.rec.blocks["lqr_timing"]["payload_bytes"], 16)
        self.rec.close()
        self.starts.reset_mock()
        self.rec = app.Recorder(fake_cf(missing="hlqr.mode"), Path(self.temp.name),
                                10, clock=self.clock, lqr=True)
        self.addCleanup(self.rec.close)
        with self.assertRaises(KeyError):
            self.rec.start()
        self.starts.assert_not_called()

    def test_missing_later_variable_starts_no_blocks_and_closes_files(self):
        self.rec.cf = fake_cf(missing="motion.shutter")
        with self.assertRaises(KeyError):
            self.rec.start()
        self.starts.assert_not_called()
        self.rec.close()
        self.assertTrue(all(b["file"].closed for b in self.rec.blocks.values()))

    def test_optional_ranger_block_rate_units_and_invalid_preserved(self):
        self.rec.close()
        self.rec = app.Recorder(self.cf, Path(self.temp.name), 10, clock=self.clock, ranger=True)
        self.addCleanup(self.rec.close)
        self.rec.start()
        self.rec.wait_ready()
        block = self.rec.blocks["ranger"]
        self.assertEqual(block["payload_bytes"], 10)
        self.assertEqual(block["period_ms"], 100)
        self.assertTrue(all(t == "uint16_t" for t in block["types"].values()))
        emit(block["config"], 1100, {"range.front": 32767, "range.left": 350})
        self.rec.close()
        with (Path(self.temp.name)/"ranger.csv").open() as file:
            rows = list(csv.DictReader(file))
        self.assertEqual(rows[-1]["range.front"], "32767")
        self.assertEqual(rows[-1]["range.left"], "350")

    def test_missing_ranger_variable_starts_no_blocks(self):
        self.rec.close()
        self.rec = app.Recorder(fake_cf(missing="range.up"), Path(self.temp.name), 10, ranger=True)
        self.addCleanup(self.rec.close)
        with self.assertRaises(KeyError):
            self.rec.start()
        self.starts.assert_not_called()

    def test_missing_first_packet_times_out(self):
        self.starts.side_effect = None
        self.rec.start()
        with self.assertRaises(app.LoggingError):
            self.rec.wait_ready(timeout=0)

    def test_async_error_and_stale_logging_are_detected(self):
        self.rec.start()
        config = self.rec.blocks["imu"]["config"]
        config.error_cb.call(config, "No memory")
        with self.assertRaisesRegex(app.LoggingError, "No memory"):
            self.rec.check()
        self.rec.errors.clear()
        self.clock.sleep(0.6)
        with self.assertRaisesRegex(app.LoggingError, "stale"):
            self.rec.check()

    def test_stop_notification_and_nonfinite_sample_fail(self):
        self.rec.start()
        config = self.rec.blocks["imu"]["config"]
        config.started_cb.call(config, False)
        with self.assertRaises(app.LoggingError):
            self.rec.check()
        self.rec.errors.clear()
        emit(config, 1010, {"acc.x": float("nan")})
        with self.assertRaisesRegex(app.LoggingError, "nonfinite"):
            self.rec.check()

    def test_late_callback_after_close_cannot_write_to_closed_file(self):
        self.rec.start()
        config = self.rec.blocks["imu"]["config"]
        dispatched_callback = config.data_received_cb.callbacks[0]
        self.rec.close()
        dispatched_callback(1010, {}, config)
        self.assertEqual(self.rec.blocks["imu"]["count"], 1)
        self.assertFalse(self.rec.errors)

    def test_timestamp_wrap_and_gap(self):
        self.starts.side_effect = None
        self.rec.start()
        config = self.rec.blocks["imu"]["config"]
        emit(config, app.TIMESTAMP_MODULUS - 5)
        emit(config, 5)
        self.assertEqual(self.rec.blocks["imu"]["unwrapped_ms"], app.TIMESTAMP_MODULUS + 5)
        self.assertEqual(self.rec.blocks["imu"]["max_gap_ms"], 10)

    def test_disconnect_is_not_reported_as_success(self):
        self.rec.start()
        self.cf.connection_lost.call("uri", "lost")
        with self.assertRaisesRegex(app.LoggingError, "link lost"):
            self.rec.check()


class FlightTests(unittest.TestCase):
    def setUp(self):
        self.cf, self.clock, self.phases = fake_cf(), Clock(), []
        self.rec = SimpleNamespace(origin=0, check=Mock(), latest=lambda: {"supervisor.info": 0xA})
        self.flight = app.PositionFlight(self.cf, self.rec, self.phases.append,
                                        csv.writer(io.StringIO()), clock=self.clock, sleep=self.clock.sleep)

    def assert_stopped(self):
        self.assertTrue(self.cf.commander.send_stop_setpoint.called)
        self.cf.supervisor.send_arming_request.assert_any_call(False)

    def test_normal_flight_position_mode_and_complete_cleanup(self):
        self.flight.run(0.1, 0.1)
        self.assertEqual(self.phases, ["takeoff", "settle", "hover", "landing", "landing"])
        calls = self.cf.commander.send_position_setpoint.call_args_list
        self.assertTrue(all(c.args[0] == c.args[1] == c.args[3] == 0 for c in calls))
        self.assertEqual(calls[-1].args[2], 0)
        self.assert_stopped()

    def test_short_waypoint_returns_then_lands(self):
        self.flight.run(.1, .1, waypoint_x=.25)
        self.assertIn("waypoint", self.phases)
        self.assertIn("return", self.phases)
        calls = self.cf.commander.send_position_setpoint.call_args_list
        self.assertTrue(any(c.args[0] == .25 for c in calls))
        self.assertEqual(calls[-1].args[:3], (0., 0., 0.))
        self.assert_stopped()

    def test_interrupted_waypoint_lands_at_current_reference(self):
        def check():
            if self.phases and self.phases[-1] == "waypoint":
                raise app.LoggingError("lost log")
        self.rec.check.side_effect = check
        with self.assertRaises(app.LoggingError):
            self.flight.run(.1, .1, waypoint_x=.25)
        self.assertNotIn("return", self.phases)
        self.assertEqual(self.cf.commander.send_position_setpoint.call_args_list[-1].args[:3],
                         (.25, 0., 0.))
        self.assert_stopped()

    def test_ctrl_c_during_takeoff_still_lands_and_disarms(self):
        interrupted = False
        def send(x, y, z, yaw):
            nonlocal interrupted
            if z > 0.04 and not interrupted:
                interrupted = True
                raise KeyboardInterrupt()
        self.cf.commander.send_position_setpoint.side_effect = send
        with self.assertRaises(KeyboardInterrupt):
            self.flight.run(0.1, 0.1)
        self.assertIn("landing", self.phases)
        self.assertEqual(self.flight.z, 0)
        self.assert_stopped()

    def test_logging_loss_during_hover_attempts_descent(self):
        def check():
            if self.phases and self.phases[-1] == "hover":
                raise app.LoggingError("no data")
        self.rec.check.side_effect = check
        with self.assertRaises(app.LoggingError):
            self.flight.run(0.1, 0.1)
        self.assertEqual(self.flight.z, 0)
        self.assert_stopped()

    def test_second_interrupt_during_landing_still_stops(self):
        def send(*_):
            if self.phases[-1] in ("hover", "landing"):
                raise KeyboardInterrupt()
        self.cf.commander.send_position_setpoint.side_effect = send
        with self.assertRaises(KeyboardInterrupt):
            self.flight.run(0.1, 0.1)
        self.assertTrue(self.flight.cleanup_errors)
        self.assert_stopped()

    def test_arm_request_exception_still_disarms(self):
        def arm(enabled):
            if enabled:
                raise OSError("send failed")
        self.cf.supervisor.send_arming_request.side_effect = arm
        with self.assertRaises(OSError):
            self.flight.run(0.1, 0.1)
        self.cf.commander.send_position_setpoint.assert_not_called()
        self.assert_stopped()

    def test_disk_error_does_not_prevent_landing_setpoints(self):
        self.flight.writer = Mock()
        def write(_):
            if self.phases[-1] in ("hover", "landing"):
                raise OSError("disk full")
        self.flight.writer.writerow.side_effect = write
        with self.assertRaisesRegex(OSError, "disk full"):
            self.flight.run(0.1, 0.1)
        self.assertEqual(self.flight.z, 0)
        self.assertTrue(self.flight.cleanup_errors)
        self.assert_stopped()

    def test_radio_loss_during_takeoff_still_attempts_stop_and_disarm(self):
        self.cf.is_connected = lambda: self.clock() < 0.1
        with self.assertRaises(ConnectionError):
            self.flight.run(0.1, 0.1)
        self.assertTrue(self.flight.cleanup_errors)
        self.assert_stopped()


class SessionTests(unittest.TestCase):
    def test_parameter_selection_requires_acknowledgement_and_removes_callback(self):
        cf = SimpleNamespace(param=Mock())
        callbacks = []
        cf.param.add_update_callback.side_effect = lambda **kw: callbacks.append(kw["cb"])
        cf.param.set_value.side_effect = lambda name, value: callbacks[0](name, value)
        app.set_confirmed(cf, "stabilizer.controller", 1)
        cf.param.remove_update_callback.assert_called_once()

    def test_connect_failure_is_bounded_and_callbacks_removed(self):
        cf = SimpleNamespace(connected=Caller(), connection_failed=Caller())
        cf.open_link = lambda uri: cf.connection_failed.call(uri, "missing radio")
        with self.assertRaisesRegex(ConnectionError, "missing radio"):
            app.connect(cf, "fake://")
        self.assertFalse(cf.connected.callbacks)
        self.assertFalse(cf.connection_failed.callbacks)

    def test_cli_rejects_nan_and_truncated_period_before_radio(self):
        for flags in (["--height", "nan"], ["--duration", "-1"], ["--period", "15"],
                      ["--height", "1.1"], ["--height", "0.5", "--period", "100"],
                      ["--name", "../escape"], ["--waypoint-x", "nan"],
                      ["--height", ".5", "--waypoint-x", ".6"], ["--waypoint-x", ".25"]):
            with self.subTest(flags=flags), patch("sys.stderr", io.StringIO()), self.assertRaises(SystemExit):
                app.parse_args(["--name", "test"] + flags)

    def test_passive_session_never_arms_or_sends_setpoints(self):
        cf = fake_cf()
        args = app.parse_args(["--name", "static", "--duration", "0.01"])
        with tempfile.TemporaryDirectory() as directory, patch.object(app, "connect"), \
                patch.object(app.LogConfig, "start", autospec=True, side_effect=emit), \
                patch.object(app.LogConfig, "stop"), patch.object(app.LogConfig, "delete"):
            self.assertEqual(app.collect(cf, args, {}, Path(directory)), 0)
            meta = json.loads((Path(directory) / "meta.json").read_text())
            self.assertEqual(meta["status"], "completed")
            self.assertEqual(meta["firmware_reported"], {"revision0": "123"})
        cf.supervisor.send_arming_request.assert_not_called()
        cf.commander.send_position_setpoint.assert_not_called()
        cf.commander.send_stop_setpoint.assert_not_called()
        cf.close_link.assert_called_once()

    def test_partial_start_failure_persists_failure_metadata_and_closes_link(self):
        cf = fake_cf(missing="motion.shutter")
        args = app.parse_args(["--name", "bad"])
        with tempfile.TemporaryDirectory() as directory, patch.object(app, "connect"), \
                patch.object(app.LogConfig, "stop"), patch.object(app.LogConfig, "delete"):
            self.assertEqual(app.collect(cf, args, {}, Path(directory)), 1)
            meta = json.loads((Path(directory) / "meta.json").read_text())
            self.assertEqual(meta["status"], "failed")
            self.assertIn("motion.shutter", meta["error"])
        cf.close_link.assert_called_once()
        cf.supervisor.send_arming_request.assert_not_called()

    def test_static_run_with_motor_activity_fails_without_sending_commands(self):
        cf = fake_cf()
        args = app.parse_args(["--name", "bad_static", "--duration", "0.01"])
        def noisy_start(config):
            emit(config, overrides={"motor.m1": 1000})
        with tempfile.TemporaryDirectory() as directory, patch.object(app, "connect"), \
                patch.object(app.LogConfig, "start", autospec=True, side_effect=noisy_start), \
                patch.object(app.LogConfig, "stop"), patch.object(app.LogConfig, "delete"):
            self.assertEqual(app.collect(cf, args, {}, Path(directory)), 1)
        cf.commander.send_position_setpoint.assert_not_called()
        cf.commander.send_stop_setpoint.assert_not_called()
        cf.supervisor.send_arming_request.assert_not_called()


if __name__ == "__main__":
    unittest.main()
