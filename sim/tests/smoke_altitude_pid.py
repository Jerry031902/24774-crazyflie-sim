"""Isaac Sim 5.1 altitude step/disturbance tests with CSV and JSON results.

    /isaac-sim/python.sh /workspace/sim/tests/smoke_altitude_pid.py

No scene or asset is saved. Output defaults to /tmp/cf_altitude_pid.
"""

import argparse
import csv
import json
import sys
import traceback
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/cf_altitude_pid"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        import run_sim as runtime

        sim, controller = runtime.setup_simulation("altitude_pid", target_z_m=1.0)
        target = {"position_m": (0.0, 0.0, 1.0)}
        rows = []
        stage_metrics = {}
        steps = 0

        def advance(duration_s, phase):
            nonlocal steps
            for _ in range(round(duration_s / runtime.PHYSICS_DT)):
                state = runtime.read_state(steps * runtime.PHYSICS_DT)
                command = controller.compute(state, target)
                thrust = command["total_thrust_N"]
                assert np.isfinite(thrust) and 0 <= thrust <= runtime.MAX_THRUST_N
                runtime.apply_thrust(thrust)
                sim.step(render=False)
                steps += 1
                if steps % runtime.RENDER_EVERY == 0:
                    sim.render()
                    state = runtime.read_state(steps * runtime.PHYSICS_DT)
                    z, vz = float(state["position_m"][2]), float(state["linear_velocity_m_s"][2])
                    assert np.isfinite(z) and np.isfinite(vz)
                    rows.append({"time_s": steps * runtime.PHYSICS_DT, "phase": phase,
                                 "target_z_m": 1.0, "z_m": z, "vz_m_s": vz, "thrust_N": thrust})
            final = runtime.read_state(steps * runtime.PHYSICS_DT)
            stage_metrics[phase] = {"final_z_m": float(final["position_m"][2]),
                                    "final_vz_m_s": float(final["linear_velocity_m_s"][2])}
            return final

        # Step from the specified initial 0.1 m to 1 m, without changing attitude.
        step = advance(6.0, "height_step")
        assert abs(step["position_m"][2] - 1.0) < .02, step
        assert abs(step["linear_velocity_m_s"][2]) < .03, step
        peak = max(row["z_m"] for row in rows)
        assert peak < 1.10, peak
        assert np.linalg.norm(step["position_m"][:2]) < 1e-4
        outside = [row["time_s"] for row in rows if abs(row["z_m"] - 1.0) > .02]
        stage_metrics["height_step"].update(peak_z_m=peak, settling_time_2cm_s=max(outside, default=0.0))

        # Inject downward/upward velocity impulses. Do not reset the controller.
        for kick, phase in ((-.6, "downward_kick"), (.6, "upward_kick")):
            velocities = runtime._body.get_linear_velocities()
            velocities[0, 2] = kick
            runtime._body.set_linear_velocities(velocities)
            recovered = advance(4.0, phase)
            assert abs(recovered["position_m"][2] - 1.0) < .03, recovered
            assert abs(recovered["linear_velocity_m_s"][2]) < .03, recovered
            assert np.linalg.norm(recovered["position_m"][:2]) < 1e-4

        # Existing modes must still select their original constant thrusts.
        for mode in ("hover_ff", "free_fall"):
            constant = runtime.make_controller(mode)
            runtime.reset_runtime(sim, constant)
            controller = constant
            final = advance(1.0, mode)
            expected_z = .1 if mode == "hover_ff" else .015
            assert abs(final["position_m"][2] - expected_z) < .002, (mode, final)

        with (args.output_dir / "trajectory.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        report = {"passed": True, "physics_dt_s": runtime.PHYSICS_DT,
                  "mass_kg": runtime.MASS_KG,
                  "gains": [runtime.ALTITUDE_KP, runtime.ALTITUDE_KI, runtime.ALTITUDE_KD],
                  "max_thrust_N": runtime.MAX_THRUST_N, "tests": stage_metrics}
        (args.output_dir / "summary.json").write_text(json.dumps(report, indent=2))
        print("ALTITUDE_PID_VALIDATION_PASSED", json.dumps(report), flush=True)
        sim.stop()
    except Exception:
        traceback.print_exc()
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
