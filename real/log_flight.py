#!/usr/bin/env python3
"""Record Flow-v2 baseline data. Default: motors off; --height > 0 flies.

Host Python, cflib 0.1.34. No Isaac Sim dependency. See README.md before flight.
Uses public position-setpoint API in one thread, not MotionCommander's __enter__.
"""

import argparse
import csv
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
LQR_LOG_BLOCKS = {
    "lqr": ["controller.roll", "controller.pitch", "hlqr.rollRef", "hlqr.pitchRef",
            "hlqr.enabled", "hlqr.mode", "hlqr.saturated"],
    "lqr_timing": ["hlqr.updates", "hlqr.dtUs", "hlqr.execUs", "hlqr.maxExecUs"],
}
UNITS = {
    "controller.roll/pitch": "actual desired legacy angles for BOTH PID and LQR, degrees",
    "hlqr.rollRef/pitchRef": "legacy angle references, degrees",
    "hlqr.dtUs/execUs/maxExecUs": "onboard horizontal-update period/execution time, microseconds",
    "hlqr.mode": "0 bypass, 1 position hold, 2 velocity tracking, 3 brake, 4 invalid horizontal input",
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


class Recorder:
    """Serialize callbacks/close with a lock; fail on missing/stale/erroring blocks."""

    def __init__(self, cf, directory, period_ms, origin=None, clock=time.monotonic, ranger=False, lqr=False):
        self.cf, self.directory, self.clock = cf, Path(directory), clock
        self.origin = clock() if origin is None else origin
        self.period_ms = period_ms
        self.log_blocks = {**LOG_BLOCKS, **(RANGER_LOG_BLOCK if ranger else {}), **(LQR_LOG_BLOCKS if lqr else {})}
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
            period = (max(100, self.period_ms) if name == "ranger" else
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
                        if not all(math.isfinite(v) for v in values):
                            raise ValueError("nonfinite measurement")
                        now = self.clock()
                        if block["last_ms"] is None:
                            block["first_ms"] = timestamp
                            block["unwrapped_ms"] = timestamp
                        else:
                            gap = (timestamp - block["last_ms"]) % TIMESTAMP_MODULUS
                            if gap >= TIMESTAMP_MODULUS // 2:
                                raise ValueError("device timestamp went backwards or device rebooted")
                            block["unwrapped_ms"] += gap
                            block["max_gap_ms"] = max(block["max_gap_ms"], gap)
                        block["writer"].writerow([timestamp, block["unwrapped_ms"],
                                                  now - self.origin, self.phase] + values)
                        block.update(last_ms=timestamp, last_host=now, latest=dict(data), count=block["count"] + 1)
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
            for name, block in self.blocks.items():
                timeout = max(LOG_STALE_S, 3 * block["period_ms"] / 1000)
                if not block["count"] or self.clock() - block["last_host"] > timeout:
                    raise LoggingError(f"Missing/stale log block: {name}")

    def latest(self):
        with self.lock:
            return {key: value for b in self.blocks.values() for key, value in b["latest"].items()}

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


class PositionFlight:
    """Single-threaded setpoints. Every arm attempt is paired with stop/disarm cleanup.

    Ctrl+C or logging failure attempts a bounded descent. Link loss or a second
    interrupt falls back to stop/disarm; delivery cannot be guaranteed over a lost link.
    """

    def __init__(self, cf, recorder, event, setpoint_writer, clock=time.monotonic, sleep=time.sleep):
        self.cf, self.recorder, self.event, self.writer = cf, recorder, event, setpoint_writer
        self.clock, self.sleep = clock, sleep
        self.x = self.y = self.z = 0.0
        self.arm_attempted = False
        self.cleanup_errors = []

    def stream(self, duration, z_at, phase, monitor=True):
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
            self.cf.commander.send_position_setpoint(self.x, self.y, self.z, 0.0)
            try:
                self.writer.writerow([self.clock() - self.recorder.origin, phase, self.x, self.y, self.z, 0.0])
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

    def land(self):
        start_z = self.z
        duration = start_z / VERTICAL_SPEED_M_S
        self.stream(duration, lambda t: max(0, start_z - VERTICAL_SPEED_M_S * t), "landing", monitor=False)
        self.stream(0.5, lambda _: 0, "landing", monitor=False)

    def stop_and_disarm(self):
        # No background thread can overwrite these stop commands.
        for _ in range(3):
            for action in (self.cf.commander.send_stop_setpoint,
                           lambda: self.cf.supervisor.send_arming_request(False)):
                try:
                    action()
                except BaseException as error:
                    self.cleanup_errors.append(f"stop/disarm: {type(error).__name__}: {error}")

    def run(self, height, duration, waypoint_x=0.0):
        completed = False
        try:
            self.arm_attempted = True  # Set BEFORE the request, including exceptions in that request.
            self.cf.supervisor.send_arming_request(True)
            wait_for(lambda: int(self.recorder.latest()["supervisor.info"]) & 0xA == 0xA,
                     3, "Armed/can-fly status not observed", self.recorder.check)
            self.stream(height / VERTICAL_SPEED_M_S,
                        lambda t: min(height, VERTICAL_SPEED_M_S * t), "takeoff")
            self.stream(HOVER_SETTLE_S, lambda _: height, "settle")
            self.stream(duration, lambda _: height, "hover")
            if waypoint_x:
                self.x = waypoint_x
                self.stream(duration, lambda _: height, "waypoint")
                self.x = 0.0
                self.stream(duration, lambda _: height, "return")
            completed = True
        finally:
            if self.arm_attempted:
                try:
                    if self.z > 0:
                        self.land()
                except BaseException as error:
                    self.cleanup_errors.append(f"landing: {type(error).__name__}: {error}")
                finally:
                    self.stop_and_disarm()
        if completed and self.cleanup_errors:
            raise RuntimeError("Flight cleanup failed: " + "; ".join(self.cleanup_errors))


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--name", required=True)
    p.add_argument("--duration", type=float, default=30, help="Static or steady-hover seconds, excluding ramps/settle")
    p.add_argument("--height", type=float, default=0, help="0: log only; 0.1..1.0 m: fly from floor")
    p.add_argument("--known-height-m", type=float, help="Tape-measured floor-to-ToF-sensor height, static runs only")
    p.add_argument("--period", type=int, default=10, help="Log ms, multiple of 10; status uses at least 50 ms")
    p.add_argument("--uri", default=DEFAULT_URI)
    p.add_argument("--note", default="")
    p.add_argument("--metadata", type=Path, help="Session JSON, see session.example.json")
    p.add_argument("--ranger", action="store_true", help="Also log Multi-ranger front/back/left/right/up at at most 10 Hz")
    p.add_argument("--out", type=Path, default=REAL_DIR / "data")
    p.add_argument("--horizontal", choices=("pid", "lqr"), default="pid",
                   help="Both retain stabilizer.controller=1; LQR enables hlqr.enable")
    p.add_argument("--lqr-log", action="store_true", help="Require additional patched-firmware LQR logs")
    p.add_argument("--waypoint-x", type=float, default=0.,
                   help="Optional supervised out-and-back X step, max +/-0.5 m; first validate hover")
    args = p.parse_args(argv)
    if not math.isfinite(args.waypoint_x) or abs(args.waypoint_x) > .5:
        p.error("--waypoint-x must be finite and within +/-0.5 m")
    if args.waypoint_x and not args.height:
        p.error("--waypoint-x requires a flight height")
    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.name):
        p.error("--name must contain only letters, digits, _ or -")
    if not math.isfinite(args.duration) or not 0 < args.duration <= MAX_DURATION_S:
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
    recorder = Recorder(cf, directory, args.period, origin, ranger=args.ranger,
                        lqr=args.lqr_log or args.horizontal == "lqr")
    meta = {"schema_version": 1, "start_time_utc": datetime.now(timezone.utc).isoformat(),
            "arguments": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
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
    save_meta()
    try:
        connect(cf, args.uri)
        meta["protocol_version"] = cf.platform.get_protocol_version()
        meta["parameters_before"] = parameter_snapshot(cf)
        # Automatically recorded raw revision fields; do not invent a firmware SHA.
        meta["firmware_reported"] = meta["parameters_before"].get("firmware", {})
        if args.ranger and cf.param.get_value("deck.bcMultiranger") != "1":
            raise RuntimeError("--ranger requested but Multi-ranger deck not detected")
        if args.height:
            if cf.param.get_value("deck.bcFlow2") != "1":
                raise RuntimeError("Flow deck v2 not detected")
            for key in ("initialX", "initialY", "initialZ", "initialYaw"):
                if float(cf.param.get_value("kalman." + key)) != 0:
                    raise RuntimeError("Floor-origin experiment requires kalman.initialX/Y/Z/Yaw = 0")
            set_confirmed(cf, "stabilizer.estimator", 2)
            set_confirmed(cf, "stabilizer.controller", 1)
            # LQR is a horizontal replacement INSIDE PID; Z/attitude/rate are retained.
            has_lqr = "hlqr" in cf.param.values
            if args.horizontal == "lqr" and not has_lqr:
                raise RuntimeError("Firmware lacks hlqr.enable; build and flash the LQR changes first")
            if has_lqr:
                set_confirmed(cf, "hlqr.brake", 0)
                set_confirmed(cf, "hlqr.enable", int(args.horizontal == "lqr"))
            set_confirmed(cf, "posCtlPid.rLimit", 10)
            set_confirmed(cf, "posCtlPid.pLimit", 10)
        meta["parameters_active"] = parameter_snapshot(cf)
        recorder.start()
        recorder.wait_ready()
        if args.height:
            event("estimator_reset")
            cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(0.1)
            cf.param.set_value("kalman.resetEstimation", "0")
            # A fixed settling period, NOT a claim of EKF covariance convergence.
            until = time.monotonic() + 2
            wait_for(lambda: time.monotonic() >= until, 3, "EKF settling wait", recorder.check)
            data = recorder.latest()
            if not (0 < data["range.zrange"] < 200 and 0 <= data["stateEstimate.z"] < 0.2):
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
                flight = PositionFlight(cf, recorder, event, writer)
                flight.run(args.height, args.duration, waypoint_x=args.waypoint_x)
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
        recorder.close()
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
