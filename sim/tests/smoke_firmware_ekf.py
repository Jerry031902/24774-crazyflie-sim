"""Run the matched 30 s Flow-v2 baseline in both modes, noise off/on.

docker exec cf_sim_dev /isaac-sim/python.sh /workspace/sim/tests/smoke_firmware_ekf.py
Artifacts: build/ekf_baseline/*.csv, *.measurements.csv, *.png, *.json.
No firmware source, tuning, NIS gate, true-state injection or scene file changes.
"""

import argparse
import json
from pathlib import Path
import sys
import traceback


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("all", "ekf_shadow", "ekf_closed_loop"), default="all")
    parser.add_argument("--noise", choices=("both", "off", "on"), default="both")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parents[2]/"build/ekf_baseline")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    exit_code = 0
    results = []
    try:
        import numpy as np
        import run_sim as runtime
        from flight_demo import FlightLog, TimedHoverMoveDemo
        from plot_ekf import analyze, plot_ekf, load
        modes = runtime.EKF_MODES if args.mode == "all" else (args.mode,)
        noises = (False, True) if args.noise == "both" else (args.noise == "on",)
        # Keep the same stage/body, reset physics/controller/RNG between runs.
        sim, _ = runtime.setup_simulation(modes[0])
        for mode in modes:
            for noise in noises:
                name = f"{mode}_noise_{'on' if noise else 'off'}"
                csv_path = args.output_dir / f"{name}.csv"
                controller = runtime.make_controller(mode, noise=noise, seed=args.seed)
                runtime.reset_runtime(sim, controller)
                demo = TimedHoverMoveDemo()
                logger = FlightLog(csv_path)
                report = {"mode": mode, "settings": controller.suite.settings(),
                          "mass_kg": runtime.MASS_KG, "inertia_kgm2": runtime.INERTIA_KGM2}
                print(f"BEGIN {name}", flush=True)
                try:
                    for step in range(round(demo.end_time_s/runtime.PHYSICS_DT)+1):
                        truth = runtime.read_state(step*runtime.PHYSICS_DT)
                        if not np.isfinite(truth["position_m"]).all() or np.linalg.norm(truth["position_m"]) > 3:
                            raise AssertionError("Flight left the test arena")
                        target = demo.target(truth)
                        command = controller.compute(truth, target)
                        runtime.apply_command(command)
                        logger.write(truth, target, command, demo.phase)
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

                try:
                    report.update(analyze(csv_path))
                    report["plot"] = str(plot_ekf(csv_path))
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
