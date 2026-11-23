"""Exercise the real recorder/flight loop with synthetic CRTP log packets and a virtual clock."""

from contextlib import ExitStack, closing, redirect_stderr, redirect_stdout
import csv
import io
import itertools
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from test_log_flight import app, emit, fake_cf


MASK = 0b1010  # Stations on channels 2 and 4 (not necessarily channels 1 and 2).
LH_TYPES = {"lighthouse." + key: "float" for key in ("x", "y", "z", "delta")}
LH_TYPES.update({"lighthouse." + key: "uint16_t"
                 for key in ("bsReceive", "bsActive", "bsGeoVal", "bsCalVal", "bsCalCon")})


class RadioBench:
    """No drivers or link: use cflib's real LogConfig packing, callbacks and the same clock for both sides."""

    def __init__(self, module=app):
        self.app, self.cf = module, fake_cf(nis=True)
        self.now = 0.0
        self.pose = (2.0, -3.0, -0.4, 90.0)
        self.armed = False
        self.configs, self.callbacks = [], {}
        self.trace = []
        self.adjust_sample = lambda name, data: None
        self.on_write = lambda name, value: None
        self.on_read = lambda name, value: value
        self.cf.supervisor.send_arming_request.side_effect = self.arm
        self.cf.commander.send_position_setpoint.side_effect = self.command
        toc = self.cf.log.toc.get_element_by_complete_name
        self.missing_logs = set()
        def log_element(key):
            if key in self.missing_logs:
                return None
            return SimpleNamespace(ctype=LH_TYPES[key]) if key in LH_TYPES else toc(key)
        self.cf.log.toc.get_element_by_complete_name = log_element
        params = self.cf.param
        params.values.update(deck={"bcLighthouse4": "1", "bcFlow2": "1", "bcMultiranger": "1"},
                             lighthouse={"fwdToEstimator": "1", "method": "1"},
                             motion={"disable": "0"}, stabilizer={"estimator": "2", "controller": "1"},
                             kalman={"initialX": "2", "initialY": "-3", "initialZ": "-0.4", "initialYaw": "90",
                                     "resetEstimation": "0"})
        # Independent of the cached parameter values, just like the downloaded device TOC.
        self.param_toc = {f"{group}.{key}" for group, values in params.values.items() for key in values}
        def param_element(key):
            return key if key in self.param_toc else None
        params.toc = SimpleNamespace(get_element_by_complete_name=param_element,
                                     get_element=lambda group, key: param_element(f"{group}.{key}"))
        params.get_value = lambda key: params.values[key.split(".")[0]][key.split(".")[1]]
        params.add_update_callback = lambda group, name, cb: self.callbacks.update({group + "." + name: cb})
        params.remove_update_callback = lambda group, name, cb: self.callbacks.pop(group + "." + name)
        params.set_value = self.write
        params.request_param_update = self.read

    def write(self, name, value):
        group, key = name.split(".")
        self.trace.append(("write", name, value))
        self.cf.param.values[group][key] = value
        self.on_write(name, value)
        if name in self.callbacks:
            self.callbacks[name](name, value)

    def read(self, name):
        value = self.on_read(name, self.cf.param.get_value(name))
        self.trace.append(("read", name, value))
        self.callbacks[name](name, value)

    def arm(self, enabled):
        self.trace.append(("arm", enabled))
        self.armed = enabled

    def command(self, *pose):
        self.trace.append(("setpoint", *pose))
        self.pose = pose

    def packet(self, config, tick):
        data = {v.name: 0 for v in config.variables}
        if config.name == "lighthouse":
            data.update({name: MASK for name in LH_TYPES if ".bs" in name})
        if config.name == "pos_vel":
            data.update(zip(("stateEstimate.x", "stateEstimate.y", "stateEstimate.z"), self.pose[:3]))
        if config.name == "attitude":
            data.update({"stateEstimate.yaw": self.pose[3], "pm.vbat": 4.0})
        if config.name == "flow_motor":
            data.update({"motion.motion": 0xb0, "range.zrange": 30, "motion.deltaX": 2, "motion.deltaY": -1})
        if config.name == "status":
            data["supervisor.info"] = 0xa if self.armed else 0
        self.adjust_sample(config.name, data)
        emit(config, timestamp=1000 + tick, overrides=data)

    def start(self, config):
        self.configs.append(config)
        self.packet(config, 0)

    def sleep(self, duration):
        # Generate every due device packet, including multiple packets between host setpoints.
        end = self.now + max(duration, 0.000001)
        for ms in range(math.floor(self.now * 1000 + 1e-7) + 1, math.floor(end * 1000 + 1e-7) + 1):
            self.now = ms / 1000
            for config in self.configs:
                if ms % config.period_in_ms == 0:
                    self.packet(config, ms)
        self.now = end

    def run(self, directory, flags):
        module = self.app
        args = module.parse_args(["--name", "bench", *flags])
        clock = SimpleNamespace(monotonic=lambda: self.now, sleep=self.sleep)
        recorder, flight = module.Recorder, module.PositionFlight
        with ExitStack() as stack:
            for p in (patch.object(module, "connect"), patch.object(module, "time", clock),
                      patch.object(module, "Recorder", side_effect=lambda *a, **kw: recorder(*a, clock=clock.monotonic, **kw)),
                      patch.object(module, "PositionFlight", side_effect=lambda *a, **kw: flight(*a, clock=clock.monotonic, sleep=self.sleep, **kw)),
                      patch.object(module.LogConfig, "start", autospec=True, side_effect=self.start),
                      patch.object(module.LogConfig, "delete"), patch.object(module.LogConfig, "stop")):
                stack.enter_context(p)
            stack.enter_context(redirect_stdout(io.StringIO()))
            stderr = stack.enter_context(redirect_stderr(io.StringIO()))
            code = module.collect(self.cf, args, {}, directory)
            self.stderr = stderr.getvalue()
        return code, json.loads((directory / "meta.json").read_text())


class LighthouseTests(unittest.TestCase):
    def setUp(self):
        self.bench = RadioBench()
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.path = Path(temp.name)

    def flight(self, *flags):
        return self.bench.run(self.path, ["--lighthouse", "--height", "0.2", "--duration", "0.1", *flags])

    def test_cli_limits(self):
        args = app.parse_args(["--name", "a", "--lighthouse", "--height", "1", "--shuttle", "1.5,3",
                               "--speed", "0.3", "--duration", "0"])
        self.assertEqual((args.shuttle, args.speed), ((1.5, 3), 0.3))
        self.assertEqual(app.parse_args(["--name", "a"]).speed, 0.2)
        for option in (["--shuttle", "1,1"], ["--height", "0.09"], ["--height", "1.01"],
                       ["--duration", "0"], ["--speed", "0"], ["--speed", "0.31"], ["--speed", "nan"],
                       *(["--height", ".5", "--shuttle", v] for v in
                         ("0,1", "-1,1", "1.51,1", "nan,1", "inf,1", "1,0", "1,-1", "1,1.5", "1"))):
            with self.subTest(option=option), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                app.parse_args(["--name", "a", *option])

    def test_reference_block_rate_and_original_one_restored(self):
        self.bench.cf.param.values["motion"]["disable"] = "1"
        code, meta = self.bench.run(self.path, ["--lighthouse", "--period", "20", "--duration", "0.1"])
        self.assertEqual(code, 0, meta)
        block = meta["logging"]["lighthouse"]
        self.assertEqual((block["types"], block["payload_bytes"], block["period_ms"]), (LH_TYPES, 26, 100))
        self.assertEqual(meta["requested_log_packets_s"], 230)
        for name in ("flow_motor", "imu"):
            self.assertEqual(meta["logging"][name]["period_ms"], 20)
        self.assertEqual(meta["lighthouse"]["method"], 1)
        self.assertEqual(meta["lighthouse"]["fwdToEstimator"], 1)
        self.assertEqual(meta["lighthouse"]["forwarding_mode"], "runtime_switch_enabled")
        self.assertEqual(meta["lighthouse"]["motion.disable"],
                         {"previous": 1, "requested": 1, "active_readback": 1, "restored_readback": 1})
        self.assertEqual(sum(op[0] == "read" for op in self.bench.trace), 3)
        self.bench.cf.supervisor.send_arming_request.assert_not_called()
        with (self.path / "flow_motor.csv").open() as f:
            self.assertEqual(next(csv.DictReader(f))["motion.deltaX"], "2")

    def test_setpoint_profile_in_absolute_coordinates_and_time(self):
        code, meta = self.flight("--shuttle", "0.1,2", "--speed", "0.2")
        self.assertEqual(code, 0, meta)
        self.assertEqual(meta["requested_log_packets_s"], 430)
        for name in ("flow_motor", "imu"):
            self.assertEqual(meta["logging"][name]["period_ms"], 10)
        self.assertEqual(meta["logging"]["lighthouse"]["period_ms"], 100)
        self.assertEqual(meta["origin_stability"]["station_mask"], MASK)
        self.assertEqual(meta["lighthouse"]["motion.disable"]["restored_readback"], 0)
        self.assertAlmostEqual(meta["flight_origin"]["z_m"], -0.4)
        with (self.path / "setpoints.csv").open() as f:
            rows = list(csv.DictReader(f))
        for row, call in zip(rows, self.bench.cf.commander.send_position_setpoint.call_args_list, strict=True):
            self.assertEqual(tuple(float(row[k]) for k in ("x_m", "y_m", "z_m", "yaw_deg")), call.args)
            self.assertAlmostEqual(float(row["x_m"]), 2.0)
            self.assertAlmostEqual(float(row["yaw_deg"]), 90.0)
        phases = [(name, list(group)) for name, group in itertools.groupby(rows, key=lambda r: r["phase"])]
        self.assertEqual([name for name, _ in phases],
                         ["takeoff", "settle", *(["shuttle_out", "hold_far", "shuttle_back", "hold_home"] * 2),
                          "hover", "landing"])
        for name, phase in phases:
            t0 = float(phase[0]["host_elapsed_s"])
            if name in ("shuttle_out", "shuttle_back", "hold_far", "hold_home"):
                self.assertAlmostEqual(float(phase[-1]["host_elapsed_s"]) - t0, 0.5, delta=0.021)
                for row in phase:
                    t = min(0.5, float(row["host_elapsed_s"]) - t0)
                    offset = {"shuttle_out": 0.2*t, "shuttle_back": 0.1-0.2*t,
                              "hold_far": 0.1, "hold_home": 0}[name]
                    self.assertAlmostEqual(float(row["y_m"]), -3 + offset, places=6)
                    self.assertAlmostEqual(float(row["z_m"]), -0.2)
        self.assertAlmostEqual(float(rows[-1]["y_m"]), -3.0)
        self.assertAlmostEqual(float(rows[-1]["z_m"]), -0.4)

    def assert_missing_toc(self, height, params=(), logs=(), flags=()):
        bench = RadioBench()
        bench.param_toc.difference_update(params)
        bench.missing_logs.update(logs)
        code, meta = bench.run(self.path, ["--lighthouse", "--height", height, *flags])
        self.assertEqual((code, meta["status"]), (1, "failed"))
        prefix = "LoggingError: --lighthouse missing device TOC fields: "
        self.assertTrue(meta["error"].startswith(prefix), meta["error"])
        expected = ["PARAM " + k for k in params] + ["LOG " + k for k in logs]
        self.assertCountEqual(meta["error"][len(prefix):].split(", "), expected)
        self.assertIn(meta["error"], bench.stderr)
        self.assertEqual(bench.trace, [])  # No explicit PARAM reads/writes, motor commands or arming.
        self.assertEqual(bench.configs, [])
        bench.cf.supervisor.send_arming_request.assert_not_called()
        bench.cf.close_link.assert_called_once()
        self.assertFalse((self.path / "setpoints.csv").exists())

    def test_each_missing_lighthouse_field_in_static_and_flight_preflight(self):
        for height in ("0", "0.2"):
            for key in ("deck.bcLighthouse4", "deck.bcFlow2", "lighthouse.method",
                        "motion.disable", "stabilizer.estimator"):
                with self.subTest(height=height, param=key):
                    self.assert_missing_toc(height, params=[key])
            for key in LH_TYPES:
                with self.subTest(height=height, log=key):
                    self.assert_missing_toc(height, logs=[key])

    def test_all_missing_fields_reported_together_including_selected_blocks(self):
        for height in ("0", "0.2"):
            params = ["deck.bcLighthouse4", "lighthouse.method", "motion.disable",
                      "deck.bcMultiranger", "flowGate.enable", "firmware.revision1"]
            if height != "0":
                params.extend(("kalman.resetEstimation", "stabilizer.controller"))
            with self.subTest(height=height):
                self.assert_missing_toc(height, params=params,
                    logs=["lighthouse.x", "lighthouse.bsCalVal", "stateEstimate.x", "motion.motion",
                          "range.front", "kalman_pred.nisX"], flags=["--ranger", "--nis"])

    def test_legacy_forwarding_in_static_and_five_trip_shuttle(self):
        for height in ("0", "0.6"):
            with self.subTest(height=height):
                bench = RadioBench()
                bench.param_toc.remove("lighthouse.fwdToEstimator")
                del bench.cf.param.values["lighthouse"]["fwdToEstimator"]
                flags = ["--duration", ".1"] if height == "0" else ["--shuttle", "1,5", "--duration", "0"]
                directory = self.path / height
                directory.mkdir()
                code, meta = bench.run(directory, ["--lighthouse", "--height", height, *flags])
                self.assertEqual(code, 0, meta)
                self.assertIsNone(meta["lighthouse"]["fwdToEstimator"])
                self.assertEqual(meta["lighthouse"]["forwarding_mode"], "always_forwarded_no_switch")
                self.assertEqual(meta["lighthouse"]["motion.disable"]["restored_readback"], 0)
                if height == "0":
                    bench.cf.supervisor.send_arming_request.assert_not_called()
                else:
                    phases = [event["phase"] for event in meta["events"]]
                    self.assertEqual((phases.count("hold_far"), phases.count("hold_home")), (5, 5))
                    with (directory / "setpoints.csv").open() as file:
                        for row in csv.DictReader(file):
                            if row["phase"] in ("hold_far", "hold_home"):
                                self.assertAlmostEqual(float(row["x_m"]), 2)
                                self.assertAlmostEqual(float(row["y_m"]), -2 if row["phase"] == "hold_far" else -3)

    def test_present_but_disabled_deck_or_fusion_still_blocks(self):
        for height in ("0", "0.2"):
            for group, key in (("deck", "bcLighthouse4"), ("deck", "bcFlow2"), ("lighthouse", "fwdToEstimator")):
                with self.subTest(height=height, param=key):
                    bench = RadioBench()
                    bench.cf.param.values[group][key] = "0"
                    code, meta = bench.run(self.path, ["--lighthouse", "--height", height])
                    self.assertEqual(code, 1)
                    self.assertIn(f"requires {group}.{key}=1", meta["error"])
                    self.assertFalse(bench.trace)
                    bench.cf.supervisor.send_arming_request.assert_not_called()

    def test_unstable_or_missing_tracking_and_legacy_flow_tof_checks(self):
        for fault, message in (("drift", "Unstable"), ("yaw", "Unstable"), ("masks", "stations"),
                               ("flow", "Flow status"), ("tof", "near-ground ToF")):
            with self.subTest(fault=fault):
                self.bench = RadioBench()
                def adjust(name, data):
                    if fault == "drift" and name == "pos_vel":
                        data["stateEstimate.x"] += self.bench.now * 0.1
                    if fault == "yaw" and name == "attitude":
                        data["stateEstimate.yaw"] += self.bench.now * 6
                    if fault == "masks" and name == "lighthouse":
                        data["lighthouse.bsCalVal"] = 2
                    if fault == "flow" and name == "flow_motor":
                        data["motion.motion"] = 0x30
                    if fault == "tof" and name == "flow_motor":
                        data["range.zrange"] = 0
                self.bench.adjust_sample = adjust
                code, meta = self.flight()
                self.assertEqual(code, 1)
                self.assertIn(message, meta["error"])
                self.assertEqual(meta["lighthouse"]["motion.disable"]["restored_readback"], 0)
                self.bench.cf.supervisor.send_arming_request.assert_not_called()

    def test_wrapped_yaw_and_missing_device_samples(self):
        window = {"pos_vel": [(i, {"stateEstimate.x": 1, "stateEstimate.y": 2, "stateEstimate.z": -1})
                              for i in range(0, 1001, 50)],
                  "attitude": [(i, {"stateEstimate.yaw": 179 if i % 100 else -179}) for i in range(0, 1001, 50)]}
        origin, stats = app.stable_origin(window)
        self.assertLess(abs(abs(origin.yaw_deg) - 180), 0.1)
        self.assertEqual(stats["yaw_spread_deg"], 2)
        window["pos_vel"] = [window["pos_vel"][0], window["pos_vel"][-1]]
        with self.assertRaisesRegex(app.LoggingError, "fresh pos_vel"):
            app.stable_origin(window)

    def test_same_two_stations_required_for_entire_window(self):
        for changed_ms in (2200, 2700, 3200):
            with self.subTest(changed_ms=changed_ms):
                self.bench = RadioBench()
                def change_channel(name, data):
                    if name == "lighthouse" and round(self.bench.now * 1000) == changed_ms:
                        for key in ("bsReceive", "bsActive", "bsGeoVal", "bsCalVal"):
                            data["lighthouse." + key] = 0b1100  # One packet reports a different station pair.
                self.bench.adjust_sample = change_channel
                code, meta = self.flight()
                self.assertEqual(code, 1)
                self.assertIn("throughout preflight", meta["error"])
                self.bench.cf.supervisor.send_arming_request.assert_not_called()

    def test_pose_and_lighthouse_must_share_a_full_second_even_across_rollover(self):
        wrap = app.TIMESTAMP_MODULUS
        window = {"pos_vel": [(wrap + t, None) for t in range(-80, 921, 10)],
                  "attitude": [(t, None) for t in range(20, 1021, 10)],
                  "lighthouse": [(t, None) for t in range(0, 1001, 100)]}
        self.assertIsNone(app.common_origin_window(window, "lighthouse"))  # Overlap is only 900 ms.
        window["pos_vel"].extend((wrap + t, None) for t in range(930, 1141, 10))
        window["lighthouse"].append((1100, None))
        covered = app.common_origin_window(window, "lighthouse")
        self.assertEqual((covered["pos_vel"][0][0], covered["pos_vel"][-1][0]), (0, 1100))
        self.assertEqual(covered["lighthouse"], window["lighthouse"])

    def test_origin_checks_latest_lighthouse_host_age_separately_from_device_gaps(self):
        recorder = app.Recorder(None, self.path, 10, clock=lambda: 2.0, lighthouse=True)
        recorder.blocks = {key: {"last_host": 2.0} for key in ("pos_vel", "attitude", "lighthouse")}
        def deliver(ready, *_):
            for name, rows in recorder.origin_window.items():
                period = 100 if name == "lighthouse" else 10
                rows.extend((t, {}) for t in range(0, 1001, period))
            recorder.blocks["lighthouse"]["last_host"] = 1.849  # Age 151 ms.
            self.assertFalse(ready())
            recorder.blocks["lighthouse"]["last_host"] = 1.851  # Age 149 ms.
            self.assertTrue(ready())
        with patch.object(app, "wait_for", side_effect=deliver), patch.object(app, "stable_origin") as estimate:
            recorder.measure_origin(lighthouse=True)
            estimate.assert_called_once()

    def test_ten_hz_station_window_accepts_jitter_but_rejects_missing_packet(self):
        for drop in (False, True):
            with self.subTest(drop=drop):
                self.bench = RadioBench()
                packet = self.bench.packet
                def deliver(config, tick):
                    if config.name == "lighthouse":
                        if drop and tick == 2600:
                            return
                        if tick % 200 == 100:
                            tick += 10  # Intervals alternate between 110 and 90 device ms.
                    packet(config, tick)
                self.bench.packet = deliver
                code, meta = self.flight()
                if drop:
                    self.assertEqual(code, 1)
                    self.assertIn("fresh lighthouse samples", meta["error"])
                    self.bench.cf.supervisor.send_arming_request.assert_not_called()
                else:
                    self.assertEqual(code, 0, meta)
                    self.assertEqual(meta["origin_stability"]["station_mask"], MASK)
                    with (self.path / "lighthouse.csv").open() as file:
                        times = [int(r["timestamp_ms"]) for r in csv.DictReader(file) if r["phase"] == "measure_origin"]
                    self.assertGreaterEqual(times[-1] - times[0], 1000)
                    self.assertEqual({b-a for a, b in zip(times, times[1:])}, {90, 110})

    def test_slow_static_period_is_respected_by_both_deck_blocks(self):
        code, meta = self.bench.run(self.path, ["--lighthouse", "--ranger", "--period", "200", "--duration", ".4"])
        self.assertEqual(code, 0, meta)
        self.assertTrue(all(block["period_ms"] == 200 for block in meta["logging"].values()))
        self.assertEqual(meta["requested_log_packets_s"], 35)  # Seven 5 Hz blocks.

    def test_shuttle_alone_uses_measured_heading_and_default_mode_has_no_new_outputs(self):
        self.bench.cf.param.values["kalman"] = dict.fromkeys(("initialX", "initialY", "initialZ", "initialYaw"), "0")
        self.bench.pose = (0.01, -0.02, 0.03, -90)
        code, meta = self.bench.run(self.path, ["--height", ".1", "--shuttle", ".1,1", "--duration", "0"])
        self.assertEqual(code, 0, meta)
        commands = self.bench.cf.commander.send_position_setpoint.call_args_list
        self.assertAlmostEqual(min(c.args[1] for c in commands), -0.12)
        self.assertAlmostEqual(commands[-1].args[1], -0.02)
        self.assertNotIn("lighthouse", meta)
        self.assertFalse(any(op[0:2] == ("write", "motion.disable") for op in self.bench.trace))
        with tempfile.TemporaryDirectory() as tmp:
            code, meta = RadioBench().run(Path(tmp), ["--duration", ".1"])
            self.assertEqual(code, 0)
            self.assertTrue({"lighthouse", "shuttle", "speed"}.isdisjoint(meta["arguments"]))
            self.assertTrue({"lighthouse", "flight_origin", "requested_log_packets_s"}.isdisjoint(meta))
            self.assertFalse((Path(tmp) / "lighthouse.csv").exists())

    def test_static_requires_running_ekf_and_link_loss_records_unconfirmed_restore(self):
        self.bench.cf.param.values["stabilizer"]["estimator"] = "1"
        code, meta = self.bench.run(self.path, ["--lighthouse", "--duration", ".1"])
        self.assertEqual(code, 1)
        self.assertIn("stabilizer.estimator", meta["error"])
        self.assertFalse(self.bench.trace)
        self.bench = RadioBench()
        self.bench.cf.is_connected = lambda: self.bench.now < 0.05
        code, meta = self.bench.run(self.path, ["--lighthouse", "--duration", ".1"])
        self.assertEqual(code, 1)
        result = meta["lighthouse"]["motion.disable"]
        self.assertIsNone(result["restored_readback"])
        self.assertIn("Link lost", result["restore_error"])
        self.bench.cf.close_link.assert_called_once()

    def test_geofence_single_packet_brakes_even_if_next_sample_returns_inside(self):
        injected = []
        def adjust(name, data):
            if (name == "pos_vel" and self.bench.armed and self.bench.pose[1] > -2.95
                    and not injected):
                data["stateEstimate.x"] += 0.55  # Cross-track at yaw 90.
                injected.append(self.bench.now)
        self.bench.adjust_sample = adjust
        code, meta = self.flight("--shuttle", "0.2,1")
        self.assertEqual(code, 1)
        self.assertIn("geofence", meta["error"])
        self.assertIn("landing", [e["phase"] for e in meta["events"]])
        self.bench.cf.supervisor.send_arming_request.assert_called_with(False)
        self.assertEqual(meta["lighthouse"]["motion.disable"]["restored_readback"], 0)
        with (self.path / "setpoints.csv").open() as f:
            landing = [r for r in csv.DictReader(f) if r["phase"] == "landing"]
        self.assertTrue(landing)
        self.assertTrue(all(float(r["y_m"]) > -2.96 for r in landing))  # Land in place.
        self.assertAlmostEqual(float(landing[-1]["z_m"]), -0.4)

    def test_abort_descent_freezes_estimate_or_last_command_when_position_is_stale(self):
        for cause in ("geofence", "ctrl-c", "logging", "stale position", "no stations", "stale lighthouse"):
            with self.subTest(cause=cause):
                bench = self.bench = RadioBench()
                def measure(name, data):
                    if bench.armed and bench.pose[1] > -2.98:
                        if name == "pos_vel":
                            data["stateEstimate.x"] += 0.55 if cause == "geofence" else 0.15
                        if cause == "logging" and name == "imu" and bench.pose[1] > -2.93:
                            data["gyro.x"] = math.nan
                        if cause == "no stations" and name == "lighthouse":
                            data["lighthouse.bsActive"] = 0
                bench.adjust_sample = measure
                if cause in ("stale position", "stale lighthouse"):
                    dropped = "pos_vel" if cause == "stale position" else "lighthouse"
                    packet = bench.packet
                    def deliver(config, tick):
                        if config.name != dropped or bench.pose[1] <= -2.93:
                            packet(config, tick)
                    bench.packet = deliver
                if cause == "ctrl-c":
                    def interrupt(*pose):
                        bench.command(*pose)
                        if pose[1] > -2.93:
                            bench.cf.commander.send_position_setpoint.side_effect = bench.command
                            raise KeyboardInterrupt()
                    bench.cf.commander.send_position_setpoint.side_effect = interrupt
                code, meta = self.flight("--shuttle", ".4,1")
                self.assertEqual(code, 130 if cause == "ctrl-c" else 1, meta)
                with (self.path / "setpoints.csv").open() as f:
                    commands = list(csv.DictReader(f))
                with (self.path / "pos_vel.csv").open() as f:
                    last_pose = [r for r in csv.DictReader(f) if r["phase"] == "shuttle_out"][-1]
                last_estimate = (float(last_pose["stateEstimate.x"]), float(last_pose["stateEstimate.y"]))
                out = [r for r in commands if r["phase"] == "shuttle_out"][-1]
                last_command = (float(out["x_m"]), float(out["y_m"]))
                self.assertNotEqual(last_estimate, last_command)
                expected = last_command if cause == "stale position" else last_estimate
                self.assertNotEqual(expected, (2, -3))  # All aborts occur away from takeoff.
                landing_xy = {(float(r["x_m"]), float(r["y_m"])) for r in commands if r["phase"] == "landing"}
                self.assertEqual(landing_xy, {expected})
                bench.cf.supervisor.send_arming_request.assert_called_with(False)

    def test_one_station_is_logged_and_method_determines_whether_flight_continues(self):
        for method in ("0", "1"):
            with self.subTest(method=method):
                bench = self.bench = RadioBench()
                bench.cf.param.values["lighthouse"]["method"] = method
                loss_times = []
                def lose_one(name, data):
                    if name == "lighthouse" and bench.armed and bench.pose[2] > -0.32:
                        data["lighthouse.bsCalVal"] = 0b0010  # Both stations still report received/active.
                        loss_times.append(bench.now)
                bench.adjust_sample = lose_one
                code, meta = self.flight()
                with (self.path / "lighthouse.csv").open() as file:
                    rows = list(csv.DictReader(file))
                self.assertTrue(any(int(row["lighthouse.bsCalVal"]) == 2
                                    and int(row["lighthouse.bsActive"]) == MASK for row in rows))
                if method == "1":
                    self.assertEqual((code, meta["status"]), (0, "completed"))
                    self.assertIn("hover", [e["phase"] for e in meta["events"]])
                else:
                    self.assertEqual(code, 1)
                    self.assertIn("usable station count below 2", meta["error"])
                    landing = next(e["host_elapsed_s"] for e in meta["events"] if e["phase"] == "landing")
                    self.assertGreaterEqual(landing - loss_times[0], app.LIGHTHOUSE_LOSS_S - 1e-6)
                    self.assertLessEqual(landing - loss_times[0], app.LIGHTHOUSE_LOSS_S + 0.021)

    def test_transient_zero_station_intervals_reset_on_recovery(self):
        bad_start_ms = None
        def blink(name, data):
            nonlocal bad_start_ms
            if name == "lighthouse" and self.bench.armed and self.bench.pose[2] > -0.32:
                tick = round(self.bench.now * 1000)
                if bad_start_ms is None:
                    bad_start_ms = tick
                if tick - bad_start_ms in (0, 100, 200, 400, 500, 600):
                    data["lighthouse.bsReceive"] = 0
        self.bench.adjust_sample = blink
        code, meta = self.flight()
        self.assertEqual((code, meta["status"]), (0, "completed"))
        with (self.path / "lighthouse.csv").open() as file:
            self.assertEqual(sum(int(row["lighthouse.bsReceive"]) == 0 for row in csv.DictReader(file)), 6)

    def test_healthy_packet_burst_after_372_ms_host_stall(self):
        bench = self.bench
        recorder = app.Recorder(bench.cf, self.path, 10, clock=lambda: bench.now, lighthouse=True)
        with (patch.object(app.LogConfig, "start", autospec=True, side_effect=bench.start),
              patch.object(app.LogConfig, "stop"), patch.object(app.LogConfig, "delete"), closing(recorder)):
            recorder.start()
            recorder.watch_lighthouse_flight(lambda _: None)
            # Both stations remain valid. All packets due during the stall arrive together.
            bench.now = 0.372
            for device_ms in range(10, 371, 10):
                for config in bench.configs:
                    if device_ms % config.period_in_ms == 0:
                        bench.packet(config, device_ms)
            recorder.check()
            for name, count, gap in (("imu", 38, 10), ("lighthouse", 4, 100)):
                block = recorder.blocks[name]
                self.assertEqual((block["count"], block["max_gap_ms"]), (count, gap))

    def test_a_late_healthy_packet_cannot_hide_a_missed_deadline(self):
        for fault in ("silence", "masks"):
            with self.subTest(fault=fault):
                bench = RadioBench()
                recorder = app.Recorder(bench.cf, self.path, 10, clock=lambda: bench.now, lighthouse=True)
                with (patch.object(app.LogConfig, "start", autospec=True, side_effect=bench.start),
                      patch.object(app.LogConfig, "stop"), patch.object(app.LogConfig, "delete")):
                    try:
                        recorder.start()
                        recorder.watch_lighthouse_flight(lambda _: None)
                        config = recorder.blocks["lighthouse"]["config"]
                        def masks(name, data):
                            if fault == "masks" and name == "lighthouse" and bench.now < 0.51:
                                data["lighthouse.bsGeoVal"] = 0
                        bench.adjust_sample = masks
                        # Deliver a healthy packet at or past the deadline, before the next control check.
                        for ms in ([500] if fault == "silence" else [100, 200, 300, 400, 510]):
                            bench.now = ms / 1000
                            bench.packet(config, ms)
                        reason = "Stale Lighthouse telemetry" if fault == "silence" else "Lighthouse tracking lost"
                        with self.assertRaisesRegex(app.LoggingError, reason):
                            recorder.check()
                    finally:
                        recorder.close()

    def test_silent_or_duplicate_lighthouse_packets_expire_while_other_blocks_continue(self):
        for duplicate in (False, True):
            with self.subTest(duplicate=duplicate):
                bench = self.bench = RadioBench()
                packet = bench.packet
                last_new_tick = None
                def deliver(config, tick):
                    nonlocal last_new_tick
                    if config.name == "lighthouse":
                        if bench.armed and bench.pose[2] > -0.32:
                            if duplicate:
                                packet(config, last_new_tick)
                            return
                        last_new_tick = tick
                    packet(config, tick)
                bench.packet = deliver
                code, meta = self.flight()
                self.assertEqual(code, 1)
                self.assertIn("Stale Lighthouse telemetry", meta["error"])
                landing = next(e["host_elapsed_s"] for e in meta["events"] if e["phase"] == "landing")
                with (self.path / "lighthouse.csv").open() as file:
                    rows = [r for r in csv.DictReader(file) if float(r["host_elapsed_s"]) <= landing]
                times = [float(r["host_elapsed_s"]) for r in rows if r["timestamp_ms"] == rows[-1]["timestamp_ms"]]
                self.assertEqual(len(times) > 1, duplicate)
                self.assertGreaterEqual(landing - times[0], app.LIGHTHOUSE_STALE_S - 1e-6)
                self.assertLessEqual(landing - times[0], app.LIGHTHOUSE_STALE_S + 0.021)

    def test_loss_during_normal_descent_finishes_landing_before_flow_restore(self):
        bench = self.bench
        descending = False
        def command(x, y, z, yaw):
            nonlocal descending
            if z < bench.pose[2]:
                descending = True
            bench.command(x, y, z, yaw)
        def lose_tracking(name, data):
            if descending and name == "lighthouse":
                data["lighthouse.bsActive"] = 0
            if descending and name == "pos_vel":
                data["stateEstimate.x"] += 0.12  # Observe the abort using the latest estimate for descent.
        bench.cf.commander.send_position_setpoint.side_effect = command
        bench.adjust_sample = lose_tracking
        code, meta = self.flight()
        self.assertEqual(code, 1)
        self.assertIn("usable station count below 1", meta["error"])
        with (self.path / "setpoints.csv").open() as file:
            landing = [r for r in csv.DictReader(file) if r["phase"] == "landing"]
        self.assertAlmostEqual(float(landing[0]["x_m"]), 2)
        self.assertAlmostEqual(float(landing[-1]["x_m"]), 2.12)
        self.assertAlmostEqual(float(landing[-1]["z_m"]), -0.4)
        armed = bench.trace.index(("arm", True))
        disarmed = bench.trace.index(("arm", False))
        self.assertFalse(any(op[0] in ("write", "read") for op in bench.trace[armed:disarmed]))
        self.assertGreater(bench.trace.index(("write", "motion.disable", "0")), disarmed)
        bench.cf.commander.send_stop_setpoint.assert_called()
        self.assertEqual(meta["lighthouse"]["motion.disable"]["restored_readback"], 0)

    def test_ground_recording_continues_logging_lost_station_masks(self):
        def lose_on_ground(name, data):
            if name == "lighthouse" and self.bench.now > 0:
                data["lighthouse.bsReceive"] = 0
        self.bench.adjust_sample = lose_on_ground
        code, meta = self.bench.run(self.path, ["--lighthouse", "--height", "0", "--duration", "1"])
        self.assertEqual((code, meta["status"]), (0, "completed"))
        self.bench.cf.supervisor.send_arming_request.assert_not_called()

    def test_rotated_geofence_all_boundaries(self):
        origin = app.FlightOrigin(2, -3, -0.4, 90)
        for x, y, up, inside in [(0, 1.49, 0.49, True), (0, 1.51, 0, False),
                                 (0, -0.51, 0, False), (0.51, 0, 0, False),
                                 (-0.51, 0, 0, False), (0, 0, -0.31, False), (0, 0, 0.51, False)]:
            data = dict(zip(("stateEstimate.x", "stateEstimate.y", "stateEstimate.z"), (2+x, -3+y, -0.4+up)))
            if inside:
                origin.check_fence(data, 1, 0.2)
            else:
                with self.assertRaisesRegex(app.LoggingError, "geofence"):
                    origin.check_fence(data, 1, 0.2)

    def test_interrupted_write_mismatched_readback_and_failed_restore(self):
        def interrupt(name, value):
            if (name, value) == ("motion.disable", "1"):
                raise KeyboardInterrupt()
        self.bench.on_write = interrupt
        code, meta = self.flight()
        self.assertEqual(code, 130)
        self.assertEqual(meta["lighthouse"]["motion.disable"]["restored_readback"], 0)
        self.bench.cf.supervisor.send_arming_request.assert_not_called()
        for failed_read, field in ((2, "error"), (3, "restore_error")):
            self.bench = RadioBench()
            answers = iter(["0", "0" if failed_read == 2 else "1", "1" if failed_read == 3 else "0"])
            self.bench.on_read = lambda key, value: next(answers)
            code, meta = self.bench.run(self.path, ["--lighthouse", "--duration", ".1"])
            self.assertEqual(code, 1)
            result = meta["lighthouse"]["motion.disable"]
            self.assertIn("read-back", (meta if failed_read == 2 else result)[field])
            self.bench.cf.close_link.assert_called_once()

    def test_ctrl_c_in_air_and_zero_final_hover(self):
        def interrupt(*pose):
            self.bench.command(*pose)
            if pose[1] > -2.95:
                self.bench.cf.commander.send_position_setpoint.side_effect = self.bench.command
                raise KeyboardInterrupt()
        self.bench.cf.commander.send_position_setpoint.side_effect = interrupt
        code, meta = self.flight("--shuttle", ".1,1")
        self.assertEqual(code, 130)
        self.assertIn("landing", [e["phase"] for e in meta["events"]])
        self.assertEqual(meta["lighthouse"]["motion.disable"]["restored_readback"], 0)
        self.bench = RadioBench()
        code, meta = self.flight("--shuttle", ".1,1", "--duration", "0")
        self.assertEqual(code, 0, meta)
        self.assertNotIn("hover", [e["phase"] for e in meta["events"]])
