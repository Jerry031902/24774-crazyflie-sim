"""Run the matched stock EKF demo/shuttles, optionally with synthetic flow faults.

docker exec cf_sim_dev /isaac-sim/python.sh /workspace/sim/tests/smoke_firmware_ekf.py
Artifacts: build/ekf_baseline/*.csv, *.measurements.csv, *.png, *.json.
Default: original 30 s demo, both modes, noise off/on, faults off. No NIS gate.
"""

import argparse
import hashlib
from datetime import datetime, timezone
from itertools import product
import json
from pathlib import Path
import sys
import traceback


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("all", "ekf_shadow", "ekf_closed_loop"), default="all")
    parser.add_argument("--noise", choices=("both", "off", "on"), default="both")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--scene", choices=("arena", "flow-empty"), default="arena")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parents[2]/"build/ekf_baseline")
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from flow_faults import add_flow_arguments, config_from_args
    from flow_gate import add_gate_arguments, gate_from_args
    add_flow_arguments(parser, sweep=True)
    add_gate_arguments(parser)
    args = parser.parse_args()
    if args.flow_fault != "off" and args.scene != "flow-empty":
        parser.error("Fault flights require --scene flow-empty; the old arena results are historical records only")
    if args.seed < 0:
        parser.error("Seed must be nonnegative")
    if args.flow_spike_amplitudes and args.flow_fault != "spike":
        parser.error("--flow-spike-amplitudes requires --flow-fault spike")
    if args.flow_spike_amplitudes and len({f"{a:g}" for a in args.flow_spike_amplitudes}) != len(args.flow_spike_amplitudes):
        parser.error("Spike sweep amplitudes must have unique output names (six significant digits)")
    try:
        faults = [config_from_args(args, amplitude=a) for a in (args.flow_spike_amplitudes or [None])]
        gate = gate_from_args(args)
    except ValueError as exc:
        parser.error(str(exc))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    exit_code = 0
    results = []
    try:
        import numpy as np
        import _cffirmware
        import run_sim as runtime
        from flight_demo import FlightLog, make_timed_demo
        from plot_ekf import analyze, plot_ekf, load
        modes = runtime.EKF_MODES if args.mode == "all" else (args.mode,)
        noises = (False, True) if args.noise == "both" else (args.noise == "on",)
        # Keep the same stage/body, reset physics/controller/RNG between runs.
        empty = args.scene == "flow-empty"
        scene_path, scene_info = runtime.ARENA_PATH, {"scene": "arena", "position_bound_m": 3.}
        if empty:
            from flow_test_scene import make_empty_scene, ContactLog
            scene_path, scene_info = make_empty_scene(runtime.ARENA_PATH, args.output_dir/"flow_empty.usda")
        sim, _ = runtime.setup_simulation(modes[0], scene_path=scene_path, contact_reports=empty)
        for mode in modes:
            for noise, fault in product(noises, faults):
                name = f"{mode}_noise_{'on' if noise else 'off'}"
                if args.trajectory != "hover-move":
                    name = f"{args.trajectory}_{name}"
                if fault.kind != "off":
                    name += f"_{fault.kind}" + (f"_{fault.spike_amplitude:g}" if fault.kind == "spike" else "")
                csv_path = args.output_dir / f"{name}.csv"
                controller = runtime.make_controller(mode, noise=noise, seed=args.seed,
                                                     flow_fault=fault, flow_audit=args.flow_audit,
                                                     flow_gate=gate, flow_gate_log=args.flow_gate_log)
                runtime.reset_runtime(sim, controller)
                demo = make_timed_demo(args.trajectory)
                logger = FlightLog(csv_path)
                contacts = ContactLog(csv_path.with_suffix(".contacts.csv")) if empty else None
                report = {"mode": mode, "settings": controller.suite.settings(),
                          "mass_kg": runtime.MASS_KG, "inertia_kgm2": runtime.INERTIA_KGM2,
                          "started_utc": datetime.now(timezone.utc).isoformat()}
                report["trajectory"] = args.trajectory
                report["scene"] = scene_info
                report["flow_gate"] = controller.ekf.flow_gate_settings
                report["bindings_sha256"] = hashlib.sha256(Path(_cffirmware.__file__).read_bytes()).hexdigest()
                if fault.kind != "off":
                    report["flow_fault_schedule"] = {"phase": args.flow_fault_phase,
                                                      "offset_s": args.flow_fault_offset,
                                                      "duration_s": args.flow_fault_duration}
                print(f"BEGIN {name}", flush=True)
                try:
                    for step in range(round(demo.end_time_s/runtime.PHYSICS_DT)+1):
                        truth = runtime.read_state(step*runtime.PHYSICS_DT)
                        if not np.isfinite(truth["position_m"]).all() or np.linalg.norm(truth["position_m"]) > scene_info["position_bound_m"]:
                            raise AssertionError("Flight left the test arena")
                        target = demo.target(truth)
                        command = controller.compute(truth, target)
                        runtime.apply_command(command)
                        logger.write(truth, target, command, demo.phase)
                        if contacts is not None:
                            contacts.time_s = (step+1)*runtime.PHYSICS_DT
                        sim.step(render=False)
                        if (step+1) % runtime.RENDER_EVERY == 0:
                            sim.render()
                        if step % 5000 == 0:
                            print(f"{name} t={truth['time_s']:.1f} phase={demo.phase} "
                                  f"true={truth['position_m'].tolist()} est={command['estimate']['position_m'].tolist()}", flush=True)
                except Exception as exc:
                    report["error"] = repr(exc)
                    traceback.print_exc()
                finally:
                    logger.close()
                    if contacts is not None:
                        contacts.close()
                        report["contacts"] = contacts.summary()

                try:
                    report.update(analyze(csv_path))
                    report["plot"] = str(plot_ekf(csv_path, gate_enabled=gate.enabled))
                    packets = load(csv_path.with_suffix(".measurements.csv"))
                    for sensor in ("flow", "tof"):
                        selected = packets[packets["sensor"] == sensor]
                        for axis in ("xy" if sensor == "flow" else "x"):
                            error = selected[f"innovation_{axis}"]
                            report[f"{sensor}_innovation_{axis}_rms"] = float(np.sqrt(np.nanmean(error**2)))
                except Exception as exc:
                    report["analysis_error"] = repr(exc)
                    report["stable"] = False
                    traceback.print_exc()
                report["estimator_calls"] = controller.ekf.counts
                report["generated_measurements"] = controller.suite.generated
                report["driver_rejected_measurements"] = controller.suite.dropped
                report["flow_driver_drop_reasons"] = controller.suite.flow_drop_reasons
                if "error" in report:
                    report["stable"] = False
                results.append(report)
                (args.output_dir/f"{name}.json").write_text(json.dumps(report, indent=2))
                (args.output_dir/"summary.json").write_text(json.dumps(results, indent=2))
                print(f"{'PASS' if report['stable'] else 'FAIL'} {name}: {json.dumps(report)}", flush=True)
                exit_code |= int(not report["stable"])
        sim.stop()
        print("EKF_BASELINE_VALIDATION_" + ("PASSED" if exit_code == 0 else "FAILED"), flush=True)
    except Exception:
        traceback.print_exc()
        exit_code = 1
    finally:
        if exit_code:
            import omni.kit.app
            omni.kit.app.get_app().post_quit(exit_code)
        app.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
