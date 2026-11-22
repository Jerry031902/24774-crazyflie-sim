#!/usr/bin/env python3
"""Record Flow-v2 baseline data. Default: motors off; --height > 0 flies.

Host Python, cflib 0.1.34. No Isaac Sim dependency. See README.md before flight.
Uses public position-setpoint API in one thread, not MotionCommander's __enter__.
"""

import argparse
import csv
from dataclasses import asdict, dataclass
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig, LogTocElement

REAL_DIR = Path(__file__).resolve().parent
DEFAULT_URI = "radio://0/80/2M/E7E7E7E701"
# Experiment limits, not experimentally validated flight-safety thresholds.
MAX_HEIGHT_M = 1.0
MAX_DURATION_S = 120.0
SETPOINT_HZ = 50
VERTICAL_SPEED_M_S = 0.2
HOVER_SETTLE_S = 3.0
SHUTTLE_SPEED_M_S = 0.2
TURN_HOLD_S = 0.5
ORIGIN_WINDOW_MS = 1000
# Limits for both device timestamp gaps and the host age of the latest packet.
# At 10 Hz this allows 50 ms of Lighthouse jitter and rejects a 200 ms gap.
ORIGIN_MAX_GAP_MS = {"pos_vel": 100, "attitude": 100, "lighthouse": 150}
ORIGIN_SPREAD_M = 0.05
ORIGIN_YAW_SPREAD_DEG = 5.0
# Flight limits measured on the host monotonic clock. Station masks refresh at ~5 Hz.
LIGHTHOUSE_LOSS_S = 0.40
LIGHTHOUSE_STALE_S = 0.35
FENCE_XY_MARGIN_M = 0.5
FENCE_Z_MARGIN_M = 0.3
CONNECT_TIMEOUT_S = 20.0
LOG_START_TIMEOUT_S = 5.0
LOG_STALE_S = 0.5
TIMESTAMP_MODULUS = 1 << 24  # cflib/firmware log packet: 24-bit milliseconds.

# Native TOC types are used, and actual sizes/types are saved in meta.json.
# Current firmware: 24, 24, 16, 23, 14 bytes; limit is 26 per block.
LOG_BLOCKS = {
    "imu": ["gyro.x", "gyro.y", "gyro.z", "acc.x", "acc.y", "acc.z"],
    "pos_vel": ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
                "stateEstimate.vx", "stateEstimate.vy", "stateEstimate.vz"],
    "attitude": ["stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw", "pm.vbat"],
    "flow_motor": ["motion.deltaX", "motion.deltaY", "range.zrange",
                   "motor.m1", "motor.m2", "motor.m3", "motor.m4",
                   "motion.motion", "motion.squal", "motion.shutter",
                   "motion.std", "motion.outlierCount"],
    "status": ["kalman.varPX", "kalman.varPY", "kalman.varPZ", "supervisor.info"],
}
RANGER_LOG_BLOCK = {"ranger": ["range.front", "range.back", "range.left", "range.right", "range.up"]}
NIS_VALUES = ("kalman_pred.nisX", "kalman_pred.nisY")
NIS_LOG_BLOCK = {"nis": [*NIS_VALUES, "flowGate.stateX", "flowGate.stateY",
                         "flowGate.rejX", "flowGate.rejY", "flowGate.entryX", "flowGate.entryY"]}
GATE_PARAMS = ("enable", "threshold", "maxReject", "recoverK")
# Four native float32 values plus five uint16 station masks = 26 bytes.
LIGHTHOUSE_LOG_BLOCK = {"lighthouse": [
    "lighthouse.x", "lighthouse.y", "lighthouse.z", "lighthouse.delta",
    "lighthouse.bsReceive", "lighthouse.bsActive", "lighthouse.bsGeoVal",
    "lighthouse.bsCalVal", "lighthouse.bsCalCon",
]}
UNITS = {
    "gyro.*": "body deg/s, calibrated and filtered",
    "acc.*": "body Gs, calibrated/aligned and filtered; includes gravity at rest",
    "stateEstimate.x/y/z": "world m, ESTIMATE, not ground truth",
    "stateEstimate.vx/vy/vz": "world m/s, ESTIMATE",
    "stateEstimate.roll/pitch/yaw": "deg, legacy pitch sign",
    "pm.vbat": "V", "range.zrange": "mm",
    "range.front/back/left/right/up": "mm; 32767 is invalid/no return, 0 may mean not sampled yet",
    "motion.deltaX/deltaY": "raw sensor counts; EKF x=-deltaY, y=-deltaX",
    "motion.motion": "status byte, stock accepts 0xB0",
    "motion.squal": "surface quality count", "motion.shutter": "clock cycles",
    "motion.std": "stock Flow std in interface counts",
    "motion.outlierCount": "uint8 cumulative range-reject count, wraps",
    "motor.m1..m4": "uint16 PWM AFTER battery compensation, not sim force commands",
    "kalman.varPX/PY/PZ": "body-velocity covariance (m/s)^2",
    "supervisor.info": "bitfield, see src/modules/src/supervisor.c",
}


class LoggingError(RuntimeError):
    pass


def tracking_stations(data):
    """One bit per channel. Require the same stations in all four masks."""
    return (int(data["lighthouse.bsReceive"]) & int(data["lighthouse.bsActive"])
            & int(data["lighthouse.bsGeoVal"]) & int(data["lighthouse.bsCalVal"]))


@dataclass
class LighthouseFlightWatch:
    minimum_stations: int
    loss_since: float | None = None

    def check(self, now, last_new_packet):
        if now >= last_new_packet + LIGHTHOUSE_STALE_S:
            raise LoggingError(f"Stale Lighthouse telemetry: no new device timestamp for "
                               f"{now - last_new_packet:.3f} s (limit {LIGHTHOUSE_STALE_S:.2f} s)")
        if self.loss_since is not None and now >= self.loss_since + LIGHTHOUSE_LOSS_S:
            raise LoggingError(f"Lighthouse tracking lost: usable station count below "
                               f"{self.minimum_stations} for {now - self.loss_since:.3f} s")

    def observe(self, data, now):
        if tracking_stations(data).bit_count() >= self.minimum_stations:
            self.loss_since = None
        elif self.loss_since is None:
            self.loss_since = now


@dataclass(frozen=True)
class FlightOrigin:
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0
    yaw_deg: float = 0.0

    def xy_at(self, forward_m):
        yaw = math.radians(self.yaw_deg)
        return self.x_m + forward_m * math.cos(yaw), self.y_m + forward_m * math.sin(yaw)

    def check_fence(self, data, distance, height):
        # Express the world estimate in the fixed takeoff heading, even at yaw 90 degrees.
        yaw = math.radians(self.yaw_deg)
        dx, dy = data["stateEstimate.x"] - self.x_m, data["stateEstimate.y"] - self.y_m
        forward = dx * math.cos(yaw) + dy * math.sin(yaw)
        lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
        up = data["stateEstimate.z"] - self.z_m
        if not (-FENCE_XY_MARGIN_M <= forward <= distance + FENCE_XY_MARGIN_M
                and abs(lateral) <= FENCE_XY_MARGIN_M
                and -FENCE_Z_MARGIN_M <= up <= height + FENCE_Z_MARGIN_M):
            raise LoggingError(f"Lighthouse geofence exceeded (forward={forward:.3f}, "
                               f"lateral={lateral:.3f}, up={up:.3f} m)")


def common_origin_window(window, reference):
    """Select a shared second or more of pose and reference observations."""
    reference_rows = window[reference]
    start = reference_rows[0][0]
    span = reference_rows[-1][0] - start
    if span < ORIGIN_WINDOW_MS:
        return None

    covered = {}
    for name, rows in window.items():
        # The streams share a 24-bit device clock but can start on opposite sides
        # of a rollover. Only keep samples bracketed by the reference packets.
        offsets = (((timestamp - start) % TIMESTAMP_MODULUS, data) for timestamp, data in rows)
        covered[name] = [(offset, data) for offset, data in offsets if offset <= span]
        if not covered[name]:
            return None
    common_start = max(rows[0][0] for rows in covered.values())
    common_end = min(rows[-1][0] for rows in covered.values())
    return covered if common_end - common_start >= ORIGIN_WINDOW_MS else None


def stable_origin(window):
    """Use distinct post-settle device timestamps, including yaw across +/-180 degrees."""
    for name, rows in window.items():
        times = [t for t, _ in rows]
        if (len(times) < 2 or times[-1] - times[0] < ORIGIN_WINDOW_MS
                or any(b - a > ORIGIN_MAX_GAP_MS[name] for a, b in zip(times, times[1:]))):
            raise LoggingError(f"Insufficient fresh {name} samples for flight origin")
    positions = [(d["stateEstimate.x"], d["stateEstimate.y"], d["stateEstimate.z"])
                 for _, d in window["pos_vel"]]
    yaw = [d["stateEstimate.yaw"] for _, d in window["attitude"]]
    yaw = [yaw[0] + (v - yaw[0] + 180) % 360 - 180 for v in yaw]
    xy_spread = max(math.hypot(x - a, y - b) for x, y, _ in positions for a, b, _ in positions)
    z = [p[2] for p in positions]
    stats = dict(xy_spread_m=xy_spread, z_spread_m=max(z) - min(z),
                 yaw_spread_deg=max(yaw) - min(yaw))
    if (xy_spread > ORIGIN_SPREAD_M or stats["z_spread_m"] > ORIGIN_SPREAD_M
            or stats["yaw_spread_deg"] > ORIGIN_YAW_SPREAD_DEG):
        raise LoggingError(f"Unstable flight origin (limits {ORIGIN_SPREAD_M} m XY/Z, "
                           f"{ORIGIN_YAW_SPREAD_DEG} deg yaw): {stats}")
    if "lighthouse" in window:
        common = 0xffff
        for _, sample in window["lighthouse"]:
            common &= tracking_stations(sample)
        if common.bit_count() < 2:
            raise LoggingError("Two Lighthouse stations must be received, active, calibrated "
                               f"and have valid geometry throughout preflight (mask={common:#06x})")
        stats["station_mask"] = common
    x, y, z = (sum(axis) / len(positions) for axis in zip(*positions))
    return FlightOrigin(x, y, z, (sum(yaw) / len(yaw) + 180) % 360 - 180), stats


class Recorder:
    """Serialize callbacks/close with a lock; fail on missing/stale/erroring blocks."""

    def __init__(self, cf, directory, period_ms, origin=None, clock=time.monotonic, ranger=False,
                 nis=False, lighthouse=False):
        self.cf, self.directory, self.clock = cf, Path(directory), clock
        self.origin = clock() if origin is None else origin
        self.period_ms = period_ms
        self.log_blocks = {**LOG_BLOCKS, **(RANGER_LOG_BLOCK if ranger else {})}
        if nis:
            self.log_blocks.update(NIS_LOG_BLOCK)
        if lighthouse:
            self.log_blocks.update(LIGHTHOUSE_LOG_BLOCK)
        self.origin_window = None
        self.position_guard = None
        self.lighthouse_watch = None
        self.lock = threading.RLock()
        self.blocks = {}
        self.errors = []
        self.accepting = True
        self.phase = "initializing"
        self.callbacks = []
        self.connection_callbacks = []

    def start(self):
        # Validate ALL TOC entries and open files before starting any block.
        for name, variables in self.log_blocks.items():
            period = (max(100, self.period_ms) if name in ("ranger", "lighthouse") else
                      max(50, self.period_ms) if name == "status" else self.period_ms)
            config = LogConfig(name=name, period_in_ms=period)
            for variable in variables:
                config.add_variable(variable)
            self.cf.log.add_config(config)
            types = {v.name: LogTocElement.get_cstring_from_id(v.fetch_as) for v in config.variables}
            payload = sum(LogTocElement.get_size_from_id(v.fetch_as) for v in config.variables)
            file = (self.directory / f"{name}.csv").open("w", newline="")
            writer = csv.writer(file)
            self.blocks[name] = dict(config=config, file=file, writer=writer, variables=variables,
                                     types=types, payload_bytes=payload, period_ms=period,
                                     count=0, first_ms=None, last_ms=None, unwrapped_ms=None,
                                     last_host=None, max_gap_ms=0, latest={}, flushed=self.clock())
            writer.writerow(["timestamp_ms", "timestamp_unwrapped_ms", "host_elapsed_s", "phase"] + variables)

            def on_data(timestamp, data, logconf, name=name):
                with self.lock:
                    if not self.accepting:
                        return
                    block = self.blocks[name]
                    try:
                        values = [data[v] for v in block["variables"]]
                        for variable, value in zip(block["variables"], values):
                            if math.isfinite(value):
                                continue
                            # Firmware initializes NIS to NaN until the first flow update.
                            if (variable in NIS_VALUES and math.isnan(value)
                                    and math.isnan(block["latest"].get(variable, math.nan))):
                                continue
                            raise ValueError("nonfinite measurement")
                        now = self.clock()
                        previous_ms = block["last_ms"]
                        if previous_ms is None:
                            block["first_ms"] = timestamp
                            block["unwrapped_ms"] = timestamp
                        else:
                            gap = (timestamp - previous_ms) % TIMESTAMP_MODULUS
                            if gap >= TIMESTAMP_MODULUS // 2:
                                raise ValueError("device timestamp went backwards or device rebooted")
                            block["unwrapped_ms"] += gap
                            block["max_gap_ms"] = max(block["max_gap_ms"], gap)
                        block["writer"].writerow([timestamp, block["unwrapped_ms"],
                                                  now - self.origin, self.phase] + values)
                        block.update(last_ms=timestamp, last_host=now, latest=dict(data), count=block["count"] + 1)
                        if name == "lighthouse" and timestamp != previous_ms:
                            last_new = block.get("last_new_host")
                            block["last_new_host"] = now
                            if self.lighthouse_watch and not self.errors:
                                # A recovery packet cannot erase a deadline reached between control ticks.
                                self.lighthouse_watch.check(now, last_new)
                                self.lighthouse_watch.observe(data, now)
                        if self.origin_window is not None and name in self.origin_window:
                            rows = self.origin_window[name]
                            if not rows or rows[-1][0] != block["unwrapped_ms"]:
                                rows.append((block["unwrapped_ms"], block["latest"]))
                        if name == "pos_vel" and self.position_guard and not self.errors:
                            self.position_guard(data)  # Any breach is latched by the callback error handler.
                        if now - block["flushed"] >= 1.0:
                            block["file"].flush()
                            block["flushed"] = now
                    except Exception as error:
                        self.errors.append(f"{name}: {error}")

            def on_error(*args, name=name):
                with self.lock:
                    if self.accepting:
                        self.errors.append(f"{name}: {args[-1]}")

            def on_started(*args, name=name):
                if args and args[-1] is False:
                    on_error("logging start/stop rejected or unexpectedly stopped", name=name)

            for caller, callback in ((config.data_received_cb, on_data),
                                     (config.error_cb, on_error), (config.started_cb, on_started)):
                caller.add_callback(callback)
                self.callbacks.append((caller, callback))

        def disconnected(*args):
            with self.lock:
                if self.accepting:
                    self.errors.append(f"link lost: {args}")

        for caller in (self.cf.connection_lost, self.cf.disconnected):
            caller.add_callback(disconnected)
            self.connection_callbacks.append((caller, disconnected))
        for block in self.blocks.values():
            block["config"].start()

    def wait_ready(self, timeout=LOG_START_TIMEOUT_S):
        deadline = self.clock() + timeout
        while True:
            with self.lock:
                if self.errors:
                    raise LoggingError(self.errors[0])
                ready = len(self.blocks) == len(self.log_blocks) and all(b["count"] for b in self.blocks.values())
            if ready:
                self.check()
                return
            if self.clock() >= deadline:
                raise LoggingError("Not all log blocks delivered data before the startup timeout")
            time.sleep(0.02)

    def check(self):
        with self.lock:
            if self.errors:
                raise LoggingError(self.errors[0])
            if self.lighthouse_watch:
                self.lighthouse_watch.check(self.clock(), self.blocks["lighthouse"]["last_new_host"])
            for name, block in self.blocks.items():
                timeout = max(LOG_STALE_S, 3 * block["period_ms"] / 1000)
                if not block["count"] or self.clock() - block["last_host"] > timeout:
                    raise LoggingError(f"Missing/stale log block: {name}")

    def latest(self):
        with self.lock:
            return {key: value for b in self.blocks.values() for key, value in b["latest"].items()}

    def measure_origin(self, lighthouse):
        window = {"pos_vel": [], "attitude": []}
        if lighthouse:
            window["lighthouse"] = []
        samples = None
        def ready():
            nonlocal samples
            with self.lock:
                now = self.clock()
                if any(not rows or now - self.blocks[name]["last_host"] > ORIGIN_MAX_GAP_MS[name] / 1000
                       for name, rows in window.items()):
                    return False
                samples = common_origin_window(window, "lighthouse" if lighthouse else "pos_vel")
                return samples is not None
        with self.lock:
            self.origin_window = window
        try:
            wait_for(ready, 3, "Timed out collecting fresh flight-origin samples", self.check)
        finally:
            with self.lock:
                self.origin_window = None
        return stable_origin(samples)

    def watch_lighthouse_flight(self, position_guard):
        with self.lock:
            if position_guard is None:
                self.position_guard = self.lighthouse_watch = None
                return
            self.check()
            block = self.blocks["lighthouse"]
            if tracking_stations(block["latest"]).bit_count() < 2:
                raise LoggingError("Lost two-station Lighthouse tracking before arming")
            # lighthouse_setup already validated this downloaded parameter before flight.
            method = int(self.cf.param.values["lighthouse"]["method"])
            watch = LighthouseFlightWatch(minimum_stations=1 if method == 1 else 2)
            watch.check(self.clock(), block["last_new_host"])
            position_guard(self.blocks["pos_vel"]["latest"])
            self.position_guard, self.lighthouse_watch = position_guard, watch

    def fresh_position(self):
        with self.lock:
            block = self.blocks["pos_vel"]
            if block["count"] and self.clock() - block["last_host"] <= LOG_STALE_S:
                return block["latest"]

    def close(self):
        # Already-dispatched callbacks see accepting=False under the same lock.
        with self.lock:
            self.accepting = False
        for caller, callback in self.callbacks + self.connection_callbacks:
            try:
                caller.remove_callback(callback)
            except BaseException as error:
                self.errors.append(f"callback cleanup: {type(error).__name__}: {error}")
        for block in self.blocks.values():
            for action in (block["config"].stop, block["config"].delete):
                try:
                    action()
                except BaseException as error:
                    self.errors.append(f"log cleanup: {type(error).__name__}: {error}")
            try:
                block["file"].close()
            except BaseException as error:
                self.errors.append(f"file cleanup: {type(error).__name__}: {error}")

    def summary(self):
        with self.lock:
            return {name: {key: b[key] for key in ("count", "first_ms", "last_ms", "unwrapped_ms",
                                                 "max_gap_ms", "period_ms", "types", "payload_bytes")}
                    for name, b in self.blocks.items()}


def wait_for(predicate, timeout, description, check=lambda: None):
    deadline = time.monotonic() + timeout
    while True:
        check()
        if predicate():
            return
        if time.monotonic() >= deadline:
            raise TimeoutError(description)
        time.sleep(0.02)


def connect(cf, uri):
    """Bound connection/TOC/parameter waits, without a blocking SyncCrazyflie enter."""
    connected, failed = threading.Event(), []
    def on_connected(*_):
        connected.set()
    def on_failed(*args):
        failed.append(str(args))
    def check():
        if failed:
            raise ConnectionError(failed[0])
    cf.connected.add_callback(on_connected)
    cf.connection_failed.add_callback(on_failed)
    try:
        cf.open_link(uri)
        wait_for(connected.is_set, CONNECT_TIMEOUT_S, "Connection/TOC timeout", check)
        wait_for(lambda: cf.param.is_updated, CONNECT_TIMEOUT_S, "Parameter download timeout", check)
    finally:
        cf.connected.remove_callback(on_connected)
        cf.connection_failed.remove_callback(on_failed)


def set_confirmed(cf, name, value):
    group, key = name.split(".")
    received = threading.Event()
    def updated(_, actual):
        if float(actual) == float(value):
            received.set()
    cf.param.add_update_callback(group=group, name=key, cb=updated)
    try:
        cf.param.set_value(name, str(value))
        wait_for(received.is_set, 3, f"Parameter acknowledgement timeout: {name}")
    finally:
        cf.param.remove_update_callback(group=group, name=key, cb=updated)


def parameter_snapshot(cf):
    return {group: dict(values) for group, values in cf.param.values.items()}


def read_motion_disable(cf):
    """Request a fresh device read. cflib get_value() alone just returns the cache."""
    received = threading.Event()
    replies = []
    def updated(_, value):
        replies.append(int(value))
        received.set()
    cf.param.add_update_callback(group="motion", name="disable", cb=updated)
    try:
        cf.param.request_param_update("motion.disable")
        wait_for(received.is_set, 3, "Read-back timeout: motion.disable")
        return replies[-1]
    finally:
        cf.param.remove_update_callback(group="motion", name="disable", cb=updated)


def write_motion_disable(cf, value):
    set_confirmed(cf, "motion.disable", value)
    actual = read_motion_disable(cf)
    if actual != value:
        raise LoggingError(f"motion.disable requested {value}, device read-back {actual}")
    return actual


def lighthouse_setup(cf, parameters, args, log_blocks):
    """Check everything this session needs in the device TOCs before changing any PARAM."""
    required_params = ["deck.bcLighthouse4", "deck.bcFlow2", "lighthouse.method",
                       "motion.disable", "stabilizer.estimator"]
    if args.height:
        required_params.extend(("stabilizer.controller", "kalman.resetEstimation"))
    if args.ranger:
        required_params.append("deck.bcMultiranger")
    if args.nis:
        required_params.extend("flowGate." + key for key in GATE_PARAMS)
        required_params.extend(("firmware.revision0", "firmware.revision1"))
    missing = ["PARAM " + name for name in required_params
               if cf.param.toc.get_element_by_complete_name(name) is None]
    missing.extend("LOG " + name for variables in log_blocks.values() for name in variables
                   if cf.log.toc.get_element_by_complete_name(name) is None)
    if missing:
        raise LoggingError("--lighthouse missing device TOC fields: " + ", ".join(missing))

    for key in ("bcLighthouse4", "bcFlow2"):
        if int(parameters["deck"][key]) != 1:
            raise LoggingError(f"--lighthouse requires deck.{key}=1")
    settings = parameters["lighthouse"]
    # Stock 6532d3876741 forwards valid measurements without a runtime switch.
    forwarding_value = None
    if cf.param.toc.get_element("lighthouse", "fwdToEstimator") is not None:
        if settings.get("fwdToEstimator") not in ("1", 1):
            raise LoggingError("--lighthouse requires lighthouse.fwdToEstimator=1")
        forwarding_value = 1
    if int(settings["method"]) not in (0, 1):
        raise LoggingError("--lighthouse requires lighthouse.method 0 or 1")
    if not args.height and int(parameters["stabilizer"]["estimator"]) != 2:
        raise LoggingError("Static Lighthouse reference requires stabilizer.estimator=2 (EKF)")
    return {"method": int(settings["method"]), "fwdToEstimator": forwarding_value,
            "forwarding_mode": "always_forwarded_no_switch" if forwarding_value is None else "runtime_switch_enabled",
            "reference": "stateEstimate.x/y/z (Lighthouse-aided onboard EKF)"}


def nis_metadata(cf, parameters):
    missing = [name for name in NIS_LOG_BLOCK["nis"]
               if cf.log.toc.get_element_by_complete_name(name) is None]
    gate = parameters.get("flowGate", {})
    firmware = parameters.get("firmware", {})
    missing.extend("flowGate." + key for key in GATE_PARAMS if key not in gate)
    missing.extend("firmware." + key for key in ("revision0", "revision1") if key not in firmware)
    if missing:
        raise LoggingError("--nis requires firmware telemetry missing from this device: " + ", ".join(missing))
    # version.vtpl exports the first 8 and next 4 Git hex digits as unsigned integers.
    revision = f"{int(firmware['revision0']):08x}{int(firmware['revision1']):04x}"
    return {"flowGate": dict(gate), "firmware_revision": revision}


class PositionFlight:
    """Single-threaded setpoints. Every arm attempt is paired with stop/disarm cleanup.

    Ctrl+C or logging failure attempts a bounded descent. Link loss or a second
    interrupt falls back to stop/disarm; delivery cannot be guaranteed over a lost link.
    """

    def __init__(self, cf, recorder, event, setpoint_writer, clock=time.monotonic, sleep=time.sleep,
                 *, origin=None, lighthouse=False):
        self.cf, self.recorder, self.event, self.writer = cf, recorder, event, setpoint_writer
        self.clock, self.sleep = clock, sleep
        self.z = 0.0
        self.origin = origin or FlightOrigin()
        self.x, self.y = self.origin.x_m, self.origin.y_m
        self.lighthouse = lighthouse
        self.arm_attempted = False
        self.cleanup_errors = []

    def stream(self, duration, z_at, phase, monitor=True, forward_at=None):
        # Failed storage must not prevent a descent while the radio still works.
        try:
            self.event(phase)
        except Exception as error:
            if monitor:
                raise
            self.cleanup_errors.append(f"landing event log: {error}")
        start, deadline = self.clock(), self.clock()
        while True:
            if not self.cf.is_connected():
                raise ConnectionError("Radio link lost during flight")
            if monitor:
                self.recorder.check()
            elapsed = min(duration, self.clock() - start)
            self.z = float(z_at(elapsed))
            if forward_at is not None:
                self.x, self.y = self.origin.xy_at(forward_at(elapsed))
            command = (self.x, self.y, self.origin.z_m + self.z, self.origin.yaw_deg)
            self.cf.commander.send_position_setpoint(*command)
            try:
                self.writer.writerow([self.clock() - self.recorder.origin, phase, *command])
            except Exception as error:
                if monitor:
                    raise
                message = f"landing setpoint log: {error}"
                if message not in self.cleanup_errors:
                    self.cleanup_errors.append(message)
            if elapsed >= duration:
                return
            deadline = max(deadline + 1 / SETPOINT_HZ, self.clock())
            self.sleep(max(0, deadline - self.clock()))

    def land(self, monitor=False):
        start_z = self.z
        duration = start_z / VERTICAL_SPEED_M_S
        self.stream(duration, lambda t: max(0, start_z - VERTICAL_SPEED_M_S * t), "landing", monitor=monitor)
        self.stream(0.5, lambda _: 0, "landing", monitor=monitor)

    def stop_and_disarm(self):
        # No background thread can overwrite these stop commands.
        for _ in range(3):
            for action in (self.cf.commander.send_stop_setpoint,
                           lambda: self.cf.supervisor.send_arming_request(False)):
                try:
                    action()
                except BaseException as error:
                    self.cleanup_errors.append(f"stop/disarm: {type(error).__name__}: {error}")

    def run(self, height, duration, shuttle=None, speed=SHUTTLE_SPEED_M_S):
        completed = False
        if self.lighthouse:
            length = shuttle[0] if shuttle else 0.0
            self.recorder.watch_lighthouse_flight(lambda d: self.origin.check_fence(d, length, height))
        try:
            self.arm_attempted = True  # Set BEFORE the request, including exceptions in that request.
            self.cf.supervisor.send_arming_request(True)
            wait_for(lambda: int(self.recorder.latest()["supervisor.info"]) & 0xA == 0xA,
                     3, "Armed/can-fly status not observed", self.recorder.check)
            self.stream(height / VERTICAL_SPEED_M_S,
                        lambda t: min(height, VERTICAL_SPEED_M_S * t), "takeoff")
            self.stream(HOVER_SETTLE_S, lambda _: height, "settle")
            if shuttle:
                distance, trips = shuttle
                leg_s = distance / speed
                for _ in range(trips):
                    self.stream(leg_s, lambda _: height, "shuttle_out",
                                forward_at=lambda t: distance if t >= leg_s else t * speed)
                    self.stream(TURN_HOLD_S, lambda _: height, "hold_far")
                    self.stream(leg_s, lambda _: height, "shuttle_back",
                                forward_at=lambda t: 0.0 if t >= leg_s else distance - t * speed)
                    self.stream(TURN_HOLD_S, lambda _: height, "hold_home")
            if duration:
                self.stream(duration, lambda _: height, "hover")
            if self.lighthouse:
                # A fault during normal descent also enters the land-in-place cleanup below.
                self.land(monitor=True)
            completed = True
        finally:
            if self.arm_attempted:
                try:
                    # Abort where the aircraft is. With stale logging, hold the last command XY.
                    if self.lighthouse and not completed and self.z > 0:
                        pose = self.recorder.fresh_position()
                        if pose:
                            self.x, self.y = pose["stateEstimate.x"], pose["stateEstimate.y"]
                            measured_height = pose["stateEstimate.z"] - self.origin.z_m
                            self.z = max(self.z, min(measured_height, height + FENCE_Z_MARGIN_M))
                    if self.z > 0:
                        self.land()
                except BaseException as error:
                    self.cleanup_errors.append(f"landing: {type(error).__name__}: {error}")
                finally:
                    self.stop_and_disarm()
            if self.lighthouse:
                self.recorder.watch_lighthouse_flight(None)
        if completed and self.cleanup_errors:
            raise RuntimeError("Flight cleanup failed: " + "; ".join(self.cleanup_errors))


def parse_shuttle(value):
    try:
        distance_text, trips_text = value.split(",")
        distance, trips = float(distance_text), int(trips_text)
    except ValueError:
        raise argparse.ArgumentTypeError("--shuttle expects DIST,TRIPS, e.g. 1,5") from None
    if not 0 < distance <= 1.5 or trips < 1:
        raise argparse.ArgumentTypeError("shuttle requires 0 < DIST <= 1.5 m and positive integer TRIPS")
    return distance, trips


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--name", required=True)
    p.add_argument("--duration", type=float, default=30,
                   help="Static or steady-hover seconds, excluding ramps/settle; may be 0 with --shuttle")
    p.add_argument("--height", type=float, default=0, help="0: log only; 0.1..1.0 m: fly from floor")
    p.add_argument("--known-height-m", type=float, help="Tape-measured floor-to-ToF-sensor height, static runs only")
    p.add_argument("--period", type=int, default=10, help="Log ms, multiple of 10; status uses at least 50 ms")
    p.add_argument("--uri", default=DEFAULT_URI)
    p.add_argument("--note", default="")
    p.add_argument("--metadata", type=Path, help="Session JSON, see session.example.json")
    p.add_argument("--ranger", action="store_true", help="Also log Multi-ranger front/back/left/right/up at at most 10 Hz")
    p.add_argument("--nis", action="store_true", help="Also record flow NIS, gate states and counters at --period")
    p.add_argument("--lighthouse", action="store_true",
                   help="Use a two-station Lighthouse reference and temporarily disable onboard Flow fusion")
    p.add_argument("--shuttle", type=parse_shuttle, metavar="DIST,TRIPS",
                   help="Fly out and back along the measured takeoff yaw after settling")
    p.add_argument("--speed", type=float, default=SHUTTLE_SPEED_M_S, help="Shuttle speed in m/s (default 0.2)")
    p.add_argument("--out", type=Path, default=REAL_DIR / "data")
    args = p.parse_args(argv)
    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.name):
        p.error("--name must contain only letters, digits, _ or -")
    if args.shuttle and not args.height:
        p.error("--shuttle requires --height between 0.1 and 1.0 m")
    if not 0 < args.speed <= 0.3:
        p.error("--speed must be positive and at most 0.3 m/s")
    if args.shuttle and not math.isfinite(args.shuttle[0] / args.speed):
        p.error("Shuttle leg duration must be finite")
    if not (0 < args.duration <= MAX_DURATION_S or (args.shuttle and args.duration == 0)):
        p.error(f"--duration must be finite and in (0, {MAX_DURATION_S}] seconds")
    if not math.isfinite(args.height) or not (args.height == 0 or 0.1 <= args.height <= MAX_HEIGHT_M):
        p.error("--height must be 0 (no flight) or between 0.1 and 1.0 m")
    if not 10 <= args.period <= 2540 or args.period % 10:
        p.error("--period must be 10..2540 ms, in multiples of 10")
    if args.height and args.period > 50:
        p.error("Flight logging requires --period <= 50 ms")
    if args.known_height_m is not None and (not math.isfinite(args.known_height_m)
                                          or args.known_height_m <= 0 or args.height):
        p.error("--known-height-m must be positive/finite and is for static runs only")
    return args


def checked_metadata(path):
    data = {} if path is None else json.loads(path.read_text())
    if not isinstance(data, dict):
        raise ValueError("Session metadata must be a JSON object")
    for field in ("mass_g", "scale_resolution_g", "height_uncertainty_m"):
        value = data.get(field)
        if value is not None and (type(value) not in (int, float) or not math.isfinite(value) or value <= 0):
            raise ValueError(f"{field} must be positive/finite, or null if not measured")
    readings = data.get("mass_readings_g", [])
    if not isinstance(readings, list) or any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in readings):
        raise ValueError("mass_readings_g must be a list of positive finite measurements")
    return data


def collect(cf, args, session, directory):
    """Own all file/link resources, including partial logging startup failures."""
    origin = time.monotonic()
    recorder = Recorder(cf, directory, args.period, origin, ranger=args.ranger, nis=args.nis,
                        lighthouse=args.lighthouse)
    arguments = {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()
                 if (k not in ("nis", "lighthouse", "shuttle") or v) and (k != "speed" or args.shuttle)}
    meta = {"schema_version": 1, "start_time_utc": datetime.now(timezone.utc).isoformat(),
            "arguments": arguments,
            "session": session, "cflib_version": importlib.metadata.version("cflib"),
            "python_version": sys.version, "units": UNITS, "log_blocks": recorder.log_blocks,
            "logger_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "control_mode": "XYZ position + absolute yaw" if args.height else "passive logging",
            "setpoint_hz": SETPOINT_HZ, "vertical_speed_m_s": VERTICAL_SPEED_M_S,
            "settle_s": HOVER_SETTLE_S, "status": "starting", "events": []}
    try:
        meta["repo_commit"] = subprocess.check_output(
            ["git", "-C", str(REAL_DIR), "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        meta["repo_commit"] = None

    def save_meta():
        temp = directory / "meta.json.tmp"
        temp.write_text(json.dumps(meta, indent=2, allow_nan=False) + "\n")
        temp.replace(directory / "meta.json")

    def event(phase):
        with recorder.lock:
            recorder.phase = phase
        meta["events"].append({"host_elapsed_s": time.monotonic() - origin, "phase": phase})
        print(f"Phase: {phase}", flush=True)
        save_meta()

    flight, code = None, 0
    restore_motion = None
    save_meta()
    try:
        connect(cf, args.uri)
        meta["protocol_version"] = cf.platform.get_protocol_version()
        meta["parameters_before"] = parameter_snapshot(cf)
        # Automatically recorded raw revision fields; do not invent a firmware SHA.
        meta["firmware_reported"] = meta["parameters_before"].get("firmware", {})
        if args.lighthouse:
            meta["lighthouse"] = lighthouse_setup(cf, meta["parameters_before"], args, recorder.log_blocks)
        if args.nis:
            meta.update(nis_metadata(cf, meta["parameters_before"]))
        if args.ranger and cf.param.get_value("deck.bcMultiranger") != "1":
            raise RuntimeError("--ranger requested but Multi-ranger deck not detected")
        if args.lighthouse:
            meta["units"] = {**UNITS,
                "stateEstimate.x/y/z": "world m, Lighthouse-aided onboard EKF reference",
                "lighthouse.x/y/z/delta": "world position / ray separation in m, crossing-beam method only",
                "lighthouse.bs*": "uint16 channel bit masks: bit 0 = channel 1",
            }
            original = read_motion_disable(cf)
            motion = {"previous": original, "requested": 1, "active_readback": None, "restored_readback": None}
            meta["lighthouse"]["motion.disable"] = motion
            save_meta()
            restore_motion = original  # Set before writing: an interrupted write may still have reached the device.
            motion["active_readback"] = write_motion_disable(cf, 1)
        if args.height:
            if not args.lighthouse:
                if cf.param.get_value("deck.bcFlow2") != "1":
                    raise RuntimeError("Flow deck v2 not detected")
                for key in ("initialX", "initialY", "initialZ", "initialYaw"):
                    if float(cf.param.get_value("kalman." + key)) != 0:
                        raise RuntimeError("Floor-origin experiment requires kalman.initialX/Y/Z/Yaw = 0")
            set_confirmed(cf, "stabilizer.estimator", 2)
            set_confirmed(cf, "stabilizer.controller", 1)
        meta["parameters_active"] = parameter_snapshot(cf)
        recorder.start()
        if args.lighthouse:
            rate = sum(1000 / b["period_ms"] for b in recorder.blocks.values())
            meta["requested_log_packets_s"] = rate
            print(f"Requested {rate:g} log packets/s (downlink)", flush=True)
        recorder.wait_ready()
        if args.lighthouse:
            wait_for(lambda: tracking_stations(recorder.latest()).bit_count() >= 2, LOG_START_TIMEOUT_S,
                     "Two Lighthouse stations must be received, active, calibrated and have valid geometry",
                     recorder.check)
        if args.height:
            event("estimator_reset")
            cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(0.1)
            cf.param.set_value("kalman.resetEstimation", "0")
            # A fixed settling period, NOT a claim of EKF covariance convergence.
            until = time.monotonic() + 2
            wait_for(lambda: time.monotonic() >= until, 3, "EKF settling wait", recorder.check)
            flight_origin = None
            if args.lighthouse or args.shuttle:
                event("measure_origin")
                flight_origin, stats = recorder.measure_origin(args.lighthouse)
                meta["flight_origin"] = asdict(flight_origin)
                meta["origin_stability"] = stats
                save_meta()  # Persist the actual reference before any arm request.
            data = recorder.latest()
            if not (0 < data["range.zrange"] < 200 and (args.lighthouse or 0 <= data["stateEstimate.z"] < 0.2)):
                raise RuntimeError("Takeoff must start on the floor with a valid near-ground ToF reading")
            if int(data["motion.motion"]) != 0xB0:
                raise RuntimeError("Flow status is invalid at preflight")
            if any(data[f"motor.m{i}"] != 0 for i in range(1, 5)):
                raise RuntimeError("Motors were already running before this flight")
            if max(abs(data["stateEstimate.roll"]), abs(data["stateEstimate.pitch"])) > 15:
                raise RuntimeError("Drone must be upright before takeoff")
            with (directory / "setpoints.csv").open("w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["host_elapsed_s", "phase", "x_m", "y_m", "z_m", "yaw_deg"])
                flight = PositionFlight(cf, recorder, event, writer, origin=flight_origin, lighthouse=args.lighthouse)
                flight.run(args.height, args.duration, args.shuttle, args.speed)
            event("post_landing")
            end = time.monotonic() + 1
            wait_for(lambda: time.monotonic() >= end, 2, "Post-landing logging", recorder.check)
            if any(recorder.latest()[f"motor.m{i}"] != 0 for i in range(1, 5)):
                raise RuntimeError("Zero motor PWM was not observed after landing/stop")
        else:
            event("static")
            end = time.monotonic() + args.duration
            def static_check():
                recorder.check()
                if any(recorder.latest()[f"motor.m{i}"] != 0 for i in range(1, 5)):
                    raise RuntimeError("Static noise recording requires motors off")
            wait_for(lambda: time.monotonic() >= end, args.duration + 1, "Static recording", static_check)
        meta["status"] = "completed"
    except KeyboardInterrupt:
        meta["status"], code = "interrupted", 130
    except Exception as error:
        meta["status"], meta["error"], code = "failed", f"{type(error).__name__}: {error}", 1
        print(meta["error"], file=sys.stderr)
    finally:
        try:
            recorder.close()
        except BaseException as error:
            recorder.errors.append(f"recorder cleanup: {type(error).__name__}: {error}")
            if isinstance(error, KeyboardInterrupt) and code == 0:
                meta["status"], code = "interrupted", 130
        if restore_motion is not None:
            try:
                if not cf.is_connected():
                    raise ConnectionError("Link lost before motion.disable could be restored")
                motion["restored_readback"] = write_motion_disable(cf, restore_motion)
            except BaseException as error:
                motion["restore_error"] = f"{type(error).__name__}: {error}"
                print(f"Check motion.disable on the device: restore unconfirmed ({error})", file=sys.stderr)
                if code == 0:
                    meta["status"], code = ("interrupted", 130) if isinstance(error, KeyboardInterrupt) else ("failed", 1)
        meta["logging"] = recorder.summary()
        meta["logging_errors"] = recorder.errors
        meta["flight_cleanup_errors"] = flight.cleanup_errors if flight else []
        meta["end_time_utc"] = datetime.now(timezone.utc).isoformat()
        meta["elapsed_s"] = time.monotonic() - origin
        if recorder.errors and code == 0:
            meta["status"], code = "failed", 1
        try:
            cf.close_link()
        except BaseException as error:
            meta["close_link_error"] = f"{type(error).__name__}: {error}"
            if code == 0:
                meta["status"], code = "failed", 1
        finally:
            save_meta()
    return code


def main(argv=None):
    args = parse_args(argv)
    session = checked_metadata(args.metadata)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    directory = args.out.resolve() / f"{stamp}_{args.name}"
    directory.mkdir(parents=True, exist_ok=False)
    print(f"Output: {directory}", flush=True)
    cflib.crtp.init_drivers()
    return collect(Crazyflie(rw_cache=str(REAL_DIR / "cache")), args, session, directory)


if __name__ == "__main__":
    raise SystemExit(main())
