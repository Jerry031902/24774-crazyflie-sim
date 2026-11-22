"""Gate-off controller/estimator replay of saved PhysX truth, without physics.

Recreates all sensor samples, C updates, PID outputs and motor telemetry. Truth
is an input, not a prediction: this supplements, never replaces, live flights.
Historical obstacle-contaminated fault runs can be checked without flying in
the old arena again. Contact reports are not replayed.
"""

import argparse
import csv
from dataclasses import fields
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np

from actuators import MotorActuator
from controllers.ekf_pid import EKFPIDController
from flight_demo import FlightLog, make_timed_demo
from flow_faults import FlowFaultConfig
from flow_gate import FlowGateConfig


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replay(path, output):
    meta = json.loads(path.with_suffix(".json").read_text())
    settings = meta["settings"]
    raw_fault = settings.get("flow_fault", {})
    config = FlowFaultConfig(**{f.name:raw_fault[f.name] for f in fields(FlowFaultConfig) if f.name in raw_fault})
    controller = EKFPIDController(meta["mode"], noise=settings["noise"], seed=settings["seed"],
                                 flow_fault=config, flow_audit=settings.get("flow_audit",False),
                                 flow_gate=FlowGateConfig(0))
    actuator = MotorActuator(None)
    demo = make_timed_demo(meta.get("trajectory","hover-move"))
    out = output/path.parent.name/path.name
    logger = FlightLog(out)
    count = 0
    try:
        with path.open(newline="") as stream:
            for row in csv.DictReader(stream):
                # PhysX reads these as float32. Restore those exact input bits;
                # parsing the shortest float32 CSV strings as float64 is wrong.
                truth = dict(time_s=float(row["time_s"]),
                    position_m=np.array([row[f"{a}_m"] for a in "xyz"],dtype=np.float32),
                    linear_velocity_m_s=np.array([row[f"v{a}_m_s"] for a in "xyz"],dtype=np.float32),
                    angular_velocity_rad_s=np.array([row[f"omega_world_{a}_rad_s"] for a in "xyz"],dtype=np.float32),
                    orientation_wxyz=np.array([row[f"q{a}"] for a in "wxyz"],dtype=np.float32))
                target = demo.target(truth)
                command = controller.compute(truth,target)
                command.update(actuator.convert(command["motor_commands"]))
                logger.write(truth,target,command,demo.phase)
                count += 1
    finally:
        logger.close()
    comparisons = []
    for suffix in (".csv",".measurements.csv",".flow_samples.csv",".flow_updates.csv"):
        original, actual = path.with_suffix(suffix),out.with_suffix(suffix)
        if not original.exists():
            if actual.exists():
                raise AssertionError(f"Unexpected replay CSV: {actual}")
            continue
        before,after = digest(original),digest(actual)
        comparisons.append(dict(reference=str(original),replay=str(actual),sha256_reference=before,
                                sha256_replay=after,identical=before==after))
    report = dict(case=str(path),rows=count,comparisons=comparisons,
                  identical=all(r["identical"] for r in comparisons),
                  method="recorded float32 PhysX truth; fresh sensor/C estimator/PID/actuator replay; no physics or contacts")
    out.with_suffix(".replay.json").write_text(json.dumps(report,indent=2))
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("cases",nargs="+",type=Path)
    p.add_argument("--output-dir",required=True,type=Path)
    args=p.parse_args()
    reports=[]
    for case in args.cases:
        report=replay(case,args.output_dir)
        reports.append(report)
        print(json.dumps({k:v for k,v in report.items() if k!="comparisons"}),flush=True)
        (args.output_dir/"summary.json").write_text(json.dumps(reports,indent=2))
    return int(not all(r["identical"] for r in reports))


if __name__=="__main__":
    raise SystemExit(main())
