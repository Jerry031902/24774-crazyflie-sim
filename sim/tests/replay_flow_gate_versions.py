"""Replay every accepted Step 4 input trace through modes 0/1; no physics/GPU."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import csv
from dataclasses import fields
import hashlib
import json
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replay(path, output):
    import numpy as np
    from actuators import MotorActuator
    from controllers.ekf_pid import EKFPIDController
    from flight_demo import FlightLog, make_timed_demo
    from flow_faults import FlowFaultConfig
    from flow_gate import FlowGateConfig
    meta = json.loads(path.with_suffix(".json").read_text())
    settings, gate = meta["settings"], meta["flow_gate"]
    fault = FlowFaultConfig(**{f.name:settings["flow_fault"][f.name]
                              for f in fields(FlowFaultConfig) if f.name in settings["flow_fault"]})
    controller = EKFPIDController(meta["mode"], noise=settings["noise"], seed=settings["seed"],
        flow_fault=fault, flow_audit=settings["flow_audit"],
        flow_gate=FlowGateConfig(gate["enabled"], gate["threshold"], gate["max_reject"]),
        flow_gate_log=gate["logging"])
    output.mkdir(parents=True, exist_ok=True)
    destination = output/path.name
    logger = FlightLog(destination)
    actuator = MotorActuator(None)
    demo = make_timed_demo(meta["trajectory"])
    try:
        with path.open(newline="") as stream:
            for row in csv.DictReader(stream):
                truth = dict(time_s=float(row["time_s"]),
                    position_m=np.array([row[f"{a}_m"] for a in "xyz"], dtype=np.float32),
                    linear_velocity_m_s=np.array([row[f"v{a}_m_s"] for a in "xyz"], dtype=np.float32),
                    angular_velocity_rad_s=np.array([row[f"omega_world_{a}_rad_s"] for a in "xyz"], dtype=np.float32),
                    orientation_wxyz=np.array([row[f"q{a}"] for a in "wxyz"], dtype=np.float32))
                target = demo.target(truth)
                command = controller.compute(truth, target)
                command.update(actuator.convert(command["motor_commands"]))
                logger.write(truth, target, command, demo.phase)
    finally:
        logger.close()
    comparisons = []
    for suffix in (".csv", ".measurements.csv", ".flow_samples.csv", ".flow_updates.csv", ".flow_gate.csv"):
        a, b = path.with_suffix(suffix), destination.with_suffix(suffix)
        comparisons.append(dict(suffix=suffix, reference_sha256=digest(a), replay_sha256=digest(b),
                                identical=digest(a)==digest(b)))
    result = dict(reference=str(path), replay=str(destination), mode=gate["enabled"],
                  comparisons=comparisons, identical=all(c["identical"] for c in comparisons),
                  method="Recorded float32 PhysX truth; fresh sensor/EKF/PID/motor replay. Contacts not replayed.")
    (output/"replay.json").write_text(json.dumps(result, indent=2))
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--reference", type=Path, default=Path("build/step4_threshold9"))
    p.add_argument("--output", type=Path, default=Path("build/step5_flow_gate_v2/legacy_replay"))
    p.add_argument("--one", type=Path)
    args = p.parse_args()
    if args.one:
        return int(not replay(args.one, args.output)["identical"])
    entries = json.loads((args.reference/"runs.json").read_text())
    args.output.mkdir(parents=True, exist_ok=True)
    def worker(entry):
        output = args.output/f"seed{entry['seed']}"/entry["mode"]/entry["case"]
        output.mkdir(parents=True, exist_ok=True)
        with (output/"replay.log").open("w") as log:
            subprocess.run([sys.executable, str(Path(__file__)), "--one", entry["path"], "--output", str(output)],
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        return json.loads((output/"replay.json").read_text())
    results = []
    with ThreadPoolExecutor(max_workers=4) as pool:
        for future in as_completed([pool.submit(worker, e) for e in entries]):
            results.append(future.result())
            (args.output/"summary.json").write_text(json.dumps(results, indent=2))
            print(f"LEGACY_REPLAY {len(results)}/{len(entries)} identical", flush=True)
    assert len(results)==80 and all(r["identical"] for r in results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
