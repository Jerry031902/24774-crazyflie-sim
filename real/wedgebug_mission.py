#!/usr/bin/env python3
"""Fly a WedgeBug mission on the Crazyflie running apps/wedgebug, and log it.

The navigation runs on the drone (apps/wedgebug/src/wedgebug_app.c); this
script uploads the mission as parameters, starts it, records telemetry and the
drone's console, and lands or stops it on request:

    Ctrl+C once    wedgebug.start = 0: the drone lands where it is
    Ctrl+C twice   emergency stop: motors off at once (the drone drops)

Losing the radio does NOT stop the mission at once: the app lands after
wedgebug.linkLoss seconds (default 1 s) without a link.

    python real/wedgebug_mission.py real/missions/one_box.json --dry-run   # bench check, no flight
    python real/wedgebug_mission.py real/missions/one_box.json             # fly

Host Python with cflib 0.1.34 (real/requirements.txt). See apps/wedgebug/README.md.
"""

import argparse
import csv
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
from cflib.crazyflie.log import LogConfig

from log_flight import DEFAULT_URI, REAL_DIR, connect, parameter_snapshot, set_confirmed, wait_for

STATES = ("IDLE", "PREFLIGHT", "TAKEOFF", "SETTLE", "NAVIGATE", "ARRIVE", "HOLD", "LAND")
MODES = ("-", "GO_TO_GOAL", "AVOID", "DONE")
PHASES = ("", "sweep", "pass", "align", "follow", "overshoot")
REASONS = {
    0: "none", 1: "done", 2: "aborted (start = 0)", 3: "bad mission parameters",
    4: "Flow deck v2 / Multi-ranger / Kalman estimator missing", 5: "preflight check failed",
    6: "no Multi-ranger frames", 7: "supervisor did not allow flight", 8: "did not settle at the start",
    9: "mission timeout", 10: "left the fence", 11: "radio link lost", 12: "Multi-ranger frames stopped",
    13: "hand over the up sensor", 14: "supervisor: tumbled/crashed/locked/emergency stop",
    15: "non-finite estimate or command", 16: "battery low",
}
MAX_TARGETS = 8
INVALID_MM = 32767
# Log blocks, at most 26 bytes each. wedgebug.x/y/yaw are in the mission frame,
# stateEstimate.* in the EKF frame (origin at takeoff, x along the nose).
LOG_BLOCKS = {
    "wb_state": (20, ["wedgebug.state", "wedgebug.reason", "wedgebug.mode", "wedgebug.phase",
                      "wedgebug.wp", "wedgebug.x", "wedgebug.y", "wedgebug.yaw", "wedgebug.seq",
                      "wedgebug.sweeps", "wedgebug.hits", "wedgebug.reached", "wedgebug.unreachable",
                      "wedgebug.count"]),
    "wb_cmd": (20, ["wedgebug.cmdVx", "wedgebug.cmdVy", "wedgebug.cmdYawRate", "range.front",
                    "range.left", "range.back", "range.right", "range.up"]),
    "estimate": (20, ["stateEstimate.x", "stateEstimate.y", "stateEstimate.z", "stateEstimate.vx",
                      "stateEstimate.vy", "stateEstimate.yaw"]),
    "status": (100, ["pm.vbat", "supervisor.info", "stateEstimate.roll", "stateEstimate.pitch",
                     "range.zrange"]),
}


# ----- mission ------------------------------------------------------------------------

def load_mission(args):
    """Mission JSON (same fields as sim/navigation/missions.py) with command-line overrides."""
    m = json.loads(args.mission.read_text()) if args.mission else {}
    if args.start:
        m["start"] = args.start
    if args.target:
        m["waypoints"], m["goal"] = args.target[:-1], args.target[-1]
    for key in ("psi0_deg", "height", "max_time", "hold_time"):
        if getattr(args, key) is not None:
            m[key] = getattr(args, key)
    if args.fence:
        m["fence"] = args.fence
    m.setdefault("name", args.mission.stem if args.mission else "custom")
    m.setdefault("start", [0.0, 0.0])
    m.setdefault("waypoints", [])
    m.setdefault("psi0_deg", 0.0)
    m.setdefault("height", 0.5)
    m.setdefault("max_time", 120.0)
    m.setdefault("hold_time", 2.0)
    nav = dict(m.get("nav", {}))
    for item in args.nav:
        key, _, value = item.partition("=")
        nav[key.strip()] = float(value)
    m["nav"] = nav
    if "goal" not in m:
        raise ValueError("No goal: give a mission file or --target X Y")
    if "fence" not in m:
        if "xlim" not in m or "ylim" not in m:
            raise ValueError("Give a fence [xmin, xmax, ymin, ymax] (or the room's xlim/ylim)")
        margin = 0.1
        m["fence"] = [m["xlim"][0] + margin, m["xlim"][1] - margin, m["ylim"][0] + margin, m["ylim"][1] - margin]

    targets = [list(map(float, t)) for t in m["waypoints"]] + [list(map(float, m["goal"]))]
    fx0, fx1, fy0, fy1 = map(float, m["fence"])
    def inside(p):
        return fx0 <= p[0] <= fx1 and fy0 <= p[1] <= fy1
    problems = []
    if not 1 <= len(targets) <= MAX_TARGETS:
        problems.append(f"{len(targets)} targets; the app takes 1..{MAX_TARGETS}")
    if not 0.2 <= m["height"] <= 1.0:
        problems.append("height must be 0.2..1.0 m")
    if not (fx0 < fx1 and fy0 < fy1):
        problems.append("fence must be [xmin, xmax, ymin, ymax] with min < max")
    if not inside(m["start"]):
        problems.append(f"start {m['start']} is outside the fence")
    problems += [f"target {t} is outside the fence" for t in targets if not inside(t)]
    if problems:
        raise ValueError("Mission rejected:\n  " + "\n  ".join(problems))
    m["targets"] = targets
    return m


def mission_params(m, args):
    """The wedgebug.* and wbnav.* parameter values for this mission."""
    params = {
        "wedgebug.nTargets": len(m["targets"]),
        "wedgebug.startX": m["start"][0], "wedgebug.startY": m["start"][1],
        "wedgebug.psi0": math.radians(m["psi0_deg"]),
        "wedgebug.height": m["height"], "wedgebug.maxTime": m["max_time"], "wedgebug.holdTime": m["hold_time"],
        "wedgebug.fenceXMin": m["fence"][0], "wedgebug.fenceXMax": m["fence"][1],
        "wedgebug.fenceYMin": m["fence"][2], "wedgebug.fenceYMax": m["fence"][3],
        "wedgebug.linkLoss": args.link_loss, "wedgebug.upStop": args.up_stop,
        "wedgebug.tsRanges": 0 if args.baseline else 1,
    }
    for i, (x, y) in enumerate(m["targets"]):
        params[f"wedgebug.x{i}"], params[f"wedgebug.y{i}"] = x, y
    for key, value in m["nav"].items():
        params[f"wbnav.{key}"] = value
    return params


def set_param(cf, name, value):
    """Set and wait for the drone's acknowledgement. Floats compare at float32 precision."""
    group, key = name.split(".")
    done = threading.Event()
    def updated(_, actual):
        if math.isclose(float(actual), float(value), rel_tol=1e-6, abs_tol=1e-6):
            done.set()
    element = cf.param.toc.get_element_by_complete_name(name)
    if element is None:
        raise KeyError(f"No parameter {name} on the drone")
    if element.pytype not in ("<f", "<d", "<e"):
        if float(value) != int(value):
            raise ValueError(f"{name} takes an integer, not {value}")
        value = int(value)  # cflib parses integer parameters with int(), which rejects "12.0"
    cf.param.add_update_callback(group=group, name=key, cb=updated)
    try:
        cf.param.set_value(name, str(value))
        wait_for(done.is_set, 3, f"No acknowledgement setting {name}")
    finally:
        cf.param.remove_update_callback(group=group, name=key, cb=updated)


# ----- logging ------------------------------------------------------------------------

class Telemetry:
    """One CSV per log block, the latest values, and the drone's console in console.txt."""

    def __init__(self, cf, directory, origin):
        self.cf, self.directory, self.origin = cf, directory, origin
        self.lock = threading.Lock()
        self.latest, self.counts, self.errors = {}, {}, []
        self.files, self.configs = [], []
        self.console = (directory / "console.txt").open("w")
        self.console_line = ""

    def start(self):
        for name, (period, variables) in LOG_BLOCKS.items():
            config = LogConfig(name=name, period_in_ms=period)
            for variable in variables:
                config.add_variable(variable)
            self.cf.log.add_config(config)
            file = (self.directory / f"{name}.csv").open("w", newline="")
            writer = csv.writer(file)
            writer.writerow(["timestamp_ms", "host_s"] + variables)
            self.files.append(file)
            self.counts[name] = 0

            def on_data(timestamp, data, _, name=name, variables=variables, writer=writer):
                with self.lock:
                    host_s = round(time.monotonic() - self.origin, 4)
                    writer.writerow([timestamp, host_s] + [data[v] for v in variables])
                    self.latest.update(data)
                    self.counts[name] += 1

            def on_error(_, message, name=name):
                with self.lock:
                    self.errors.append(f"{name}: {message}")

            config.data_received_cb.add_callback(on_data)
            config.error_cb.add_callback(on_error)
            self.configs.append(config)
        for config in self.configs:
            config.start()
        self.cf.console.receivedChar.add_callback(self.on_console)

    def on_console(self, text):
        with self.lock:
            self.console.write(text)
            self.console.flush()
            self.console_line += text
            *lines, self.console_line = self.console_line.split("\n")
        for line in lines:
            print(f"  cf> {line}", flush=True)

    def get(self, name, default=math.nan):
        with self.lock:
            return self.latest.get(name, default)

    def ready(self):
        with self.lock:
            return all(self.counts.values())

    def close(self):
        try:
            self.cf.console.receivedChar.remove_callback(self.on_console)
        except Exception:
            pass
        for config in self.configs:
            try:
                config.stop()
                config.delete()
            except Exception as error:
                self.errors.append(f"log cleanup: {error}")
        with self.lock:
            for file in self.files:
                file.close()
            self.console.close()


def metres(mm):
    return math.inf if mm >= INVALID_MM else mm / 1000


def status_line(t):
    state = int(t.get("wedgebug.state", 0))
    mode, phase = int(t.get("wedgebug.mode", 0)), int(t.get("wedgebug.phase", 0))
    ranges = " ".join(f"{d[0].upper()} {metres(t.get('range.' + d, INVALID_MM)):4.2f}"
                      for d in ("front", "left", "back", "right"))
    nav = f"{MODES[mode]}{'/' + PHASES[phase] if phase else ''} wp {int(t.get('wedgebug.wp', 0))}" if mode else ""
    return (f"{STATES[state]:9s} {nav:24s} ({t.get('wedgebug.x'):5.2f}, {t.get('wedgebug.y'):5.2f}) "
            f"z {t.get('stateEstimate.z'):4.2f}  {ranges}  {t.get('pm.vbat'):4.2f} V")


# ----- flight --------------------------------------------------------------------------

def check_drone(cf):
    toc = cf.param.toc.toc
    if "wedgebug" not in toc or "wbnav" not in toc:
        raise RuntimeError("This Crazyflie has no WedgeBug app: flash it with apps/wedgebug/build.sh flash")
    for deck, label in (("bcFlow2", "Flow deck v2"), ("bcMultiranger", "Multi-ranger deck")):
        if cf.param.get_value(f"deck.{deck}") != "1":
            raise RuntimeError(f"{label} not detected")
    for key in ("initialX", "initialY", "initialYaw"):
        if float(cf.param.get_value(f"kalman.{key}")) != 0:
            raise RuntimeError(f"kalman.{key} must be 0: the app maps the mission onto an EKF reset at the drone")


def fly(cf, telemetry, event, timeout_s):
    """Start the mission and follow it until the app is idle again. Returns the end reason."""
    count0 = int(telemetry.get("wedgebug.count", 0))
    cf.param.set_value("wedgebug.start", "1")
    event("start")
    started = time.monotonic()
    last_state, last_print, interrupts = None, 0.0, 0
    while True:
        try:
            if not cf.is_connected():
                raise ConnectionError("Radio link lost: the app lands after wedgebug.linkLoss seconds")
            state = int(telemetry.get("wedgebug.state", 0))
            taken = int(telemetry.get("wedgebug.count", count0)) != count0
            if not taken and time.monotonic() - started > 3:
                raise RuntimeError("The app did not take up wedgebug.start")
            if taken and state != last_state:
                event(STATES[state])
                last_state = state
            reason = int(telemetry.get("wedgebug.reason", 0))
            if taken and state == 0 and reason != 0:  # every ending sets a reason; 0 = just starting
                return reason
            if time.monotonic() - last_print >= 1.0 and taken:
                print("  " + status_line(telemetry), flush=True)
                last_print = time.monotonic()
            if time.monotonic() - started > timeout_s:
                raise TimeoutError("Mission did not end in time; landing")
            time.sleep(0.05)
        except KeyboardInterrupt:
            interrupts += 1
            if interrupts == 1:
                print("\nLanding (Ctrl+C again: emergency stop)", flush=True)
                cf.param.set_value("wedgebug.start", "0")
                event("abort_requested")
            else:
                print("\nEMERGENCY STOP", flush=True)
                cf.supervisor.send_emergency_stop()
                event("emergency_stop")
                return None
        except (ConnectionError, RuntimeError, TimeoutError):
            if cf.is_connected():
                cf.param.set_value("wedgebug.start", "0")
            raise


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mission", nargs="?", type=Path, help="Mission JSON, e.g. real/missions/one_box.json")
    p.add_argument("--uri", default=DEFAULT_URI)
    p.add_argument("--start", nargs=2, type=float, metavar=("X", "Y"), help="Takeoff point, mission frame [m]")
    p.add_argument("--psi0-deg", type=float, help="Takeoff heading, counter-clockwise from mission +x [deg]")
    p.add_argument("--target", nargs=2, type=float, action="append", metavar=("X", "Y"),
                   help="Target [m]; repeat for waypoints, the last is the goal (replaces the file's)")
    p.add_argument("--height", type=float, help="Flight height, 0.2..1.0 m")
    p.add_argument("--fence", nargs=4, type=float, metavar=("XMIN", "XMAX", "YMIN", "YMAX"))
    p.add_argument("--max-time", type=float, help="Navigation time limit [s]")
    p.add_argument("--hold-time", type=float, help="Hover at the goal before landing [s]")
    p.add_argument("--nav", action="append", default=[], metavar="NAME=VALUE",
                   help="Navigator setting (wbnav.*), e.g. --nav vCruise=0.3; repeatable")
    p.add_argument("--link-loss", type=float, default=1.0, help="Land after this long without radio [s]; 0 = never")
    p.add_argument("--up-stop", type=float, default=0.2,
                   help="A hand within this of the up sensor lands the drone [m]; 0 = off")
    p.add_argument("--baseline", action="store_true", help="Baseline navigator (no timestamped ranger frames)")
    p.add_argument("--dry-run", action="store_true", help="Upload, check and log for --dry-seconds; no flight")
    p.add_argument("--dry-seconds", type=float, default=5.0)
    p.add_argument("--yes", action="store_true", help="Take off without asking for Enter")
    p.add_argument("--name", help="Run name (default: the mission name)")
    p.add_argument("--note", default="")
    p.add_argument("--out", type=Path, default=REAL_DIR / "data")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        mission = load_mission(args)
    except (ValueError, KeyError, json.JSONDecodeError) as error:
        print(error, file=sys.stderr)
        return 2
    params = mission_params(mission, args)
    name = args.name or mission["name"]
    if not re.fullmatch(r"[A-Za-z0-9_-]+", name):
        print("Run name must contain only letters, digits, _ or -", file=sys.stderr)
        return 2
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    directory = args.out.resolve() / f"{stamp}_wedgebug_{name}{'_dry' if args.dry_run else ''}"
    directory.mkdir(parents=True, exist_ok=False)
    print(f"Output: {directory}")

    origin = time.monotonic()
    meta = {"schema_version": 1, "kind": "wedgebug_mission", "start_time_utc": datetime.now(timezone.utc).isoformat(),
            "mission": mission, "parameters_set": params, "dry_run": args.dry_run, "note": args.note,
            "arguments": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
            "cflib_version": importlib.metadata.version("cflib"), "events": [], "status": "starting"}
    try:
        meta["repo_commit"] = subprocess.check_output(["git", "-C", str(REAL_DIR), "rev-parse", "HEAD"],
                                                      text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        meta["repo_commit"] = None

    def save_meta():
        (directory / "meta.json").write_text(json.dumps(meta, indent=2, default=str) + "\n")

    def event(what):
        meta["events"].append({"host_s": round(time.monotonic() - origin, 3), "event": what})
        print(f"[{time.monotonic() - origin:6.1f} s] {what}", flush=True)

    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache=str(REAL_DIR / "cache"))
    telemetry, code = None, 1
    save_meta()
    try:
        connect(cf, args.uri)
        check_drone(cf)
        meta["firmware_reported"] = parameter_snapshot(cf).get("firmware", {})
        set_confirmed(cf, "stabilizer.estimator", 2)   # Kalman
        set_confirmed(cf, "stabilizer.controller", 1)  # PID, as the sim
        for key, value in params.items():
            set_param(cf, key, value)
        meta["parameters"] = {g: v for g, v in parameter_snapshot(cf).items() if g in ("wedgebug", "wbnav")}
        telemetry = Telemetry(cf, directory, origin)
        telemetry.start()
        wait_for(telemetry.ready, 5, "Telemetry did not start")
        time.sleep(0.5)
        print("  " + status_line(telemetry))
        save_meta()

        targets = ", ".join(f"({x:.2f}, {y:.2f})" for x, y in mission["targets"])
        print(f"Mission {mission['name']}: start ({mission['start'][0]:.2f}, {mission['start'][1]:.2f}) "
              f"heading {mission['psi0_deg']:.0f} deg -> {targets} at {mission['height']:.2f} m; "
              f"fence {mission['fence']}")
        if args.dry_run:
            event("dry_run")
            end = time.monotonic() + args.dry_seconds
            while time.monotonic() < end:
                time.sleep(1.0)
                print("  " + status_line(telemetry), flush=True)
            meta["status"], code = "dry_run", 0
        else:
            if not args.yes:
                input("Place the drone at the start, nose along the heading, area clear.\n"
                      "Press Enter to take off (Ctrl+C to cancel): ")
            timeout = mission["max_time"] + 60
            reason = fly(cf, telemetry, event, timeout)
            meta["result"] = {"reason": reason, "reason_text": REASONS.get(reason, "emergency stop"),
                              "reached": int(telemetry.get("wedgebug.reached", 0)),
                              "unreachable": int(telemetry.get("wedgebug.unreachable", 0)),
                              "hits": int(telemetry.get("wedgebug.hits", 0)),
                              "sweeps": int(telemetry.get("wedgebug.sweeps", 0))}
            print(f"Mission ended: {meta['result']['reason_text']}; reached {meta['result']['reached']} of "
                  f"{len(mission['targets'])}, unreachable {meta['result']['unreachable']}")
            time.sleep(1.0)  # the last log packets
            meta["status"] = "completed" if reason == 1 else "ended"
            code = 0 if reason == 1 else 1
    except KeyboardInterrupt:
        meta["status"], code = "interrupted", 130
        if cf.is_connected():
            cf.param.set_value("wedgebug.start", "0")
    except Exception as error:
        meta["status"], meta["error"] = "failed", f"{type(error).__name__}: {error}"
        print(meta["error"], file=sys.stderr)
    finally:
        if telemetry:
            telemetry.close()
            meta["telemetry_counts"], meta["telemetry_errors"] = telemetry.counts, telemetry.errors
        meta["end_time_utc"] = datetime.now(timezone.utc).isoformat()
        save_meta()
        try:
            cf.close_link()
        except Exception:
            pass
    return code


if __name__ == "__main__":
    raise SystemExit(main())
