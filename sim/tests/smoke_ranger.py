"""Verify live PhysX ranging and an unchanged EKF/PID flight, with noise off/on.

docker exec cf_sim_dev /isaac-sim/python.sh /workspace/sim/tests/smoke_ranger.py
No firmware, USD asset, real drone, or controller modifications.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time
import traceback


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parents[2]/"build/ranger_validation")
    parser.add_argument("--geometry-only", action="store_true")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    result = {"geometry": {}, "flights": {}}
    exit_code = 0
    try:
        import numpy as np
        import run_sim as runtime
        from ranger import MultiRanger, PhysXRaycaster, RangerLog, RANGER_INVALID_MM
        from flight_demo import FlightLog, TimedHoverMoveDemo
        from plot_ekf import analyze, plot_ekf
        from plot_ranger import plot_ranger

        sim, _ = runtime.setup_simulation("ekf_closed_loop", ranger=True)
        sensor = runtime._ranger
        caster = sensor.raycast

        def measure(position, q=(1, 0, 0, 0), active=sensor):
            return {r["direction"]: r for r in active.measure(
                {"time_s": .1, "position_m": position, "orientation_wxyz": q})}

        def check(label, actual, expected):
            if not math.isclose(actual, expected, abs_tol=2e-5):
                raise AssertionError(f"{label}: expected {expected}, got {actual}")
            result["geometry"][label] = {"actual_m": actual, "expected_m": expected}
            print(f"PASS {label}: {actual:.6f} m", flush=True)

        # World-aligned bounds, independently audited from the actual USD stage.
        from pxr import Usd, UsdGeom
        from isaacsim.core.utils.stage import get_current_stage
        stage = get_current_stage()
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
        for path in ("/World/Walls/East", "/World/Obstacles/Box03"):
            print(f"WORLD BBOX {path}: {cache.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()}", flush=True)
        center = measure((0, 0, 1))
        for direction in ("front", "back", "left", "right"):
            check(f"center_{direction}", center[direction]["ideal_distance_m"], 2.95)
        assert center["up"]["valid"] == 0 and center["up"]["range_mm"] == RANGER_INVALID_MM
        print("PASS open ceiling: invalid, never a zero-distance obstacle", flush=True)
        moved = measure((.2, 0, 1))
        check("translation_front", moved["front"]["ideal_distance_m"], 2.75)
        check("translation_back", moved["back"]["ideal_distance_m"], 3.15)
        pillar = measure((0, -.6, 1))
        check("nearest_pillar", pillar["front"]["ideal_distance_m"], .35)
        assert pillar["front"]["hit_prim"].startswith("/World/Obstacles/Box03")
        yaw = measure((0, -.6, 1), (math.sqrt(.5), 0, 0, math.sqrt(.5)))
        check("yaw_90_right_sees_pillar", yaw["right"]["ideal_distance_m"], .35)
        check("yaw_90_front_sees_north", yaw["front"]["ideal_distance_m"], 3.55)
        pitch = measure((0, 0, 1), (math.sqrt(.5), 0, math.sqrt(.5), 0))
        check("pitch_90_front_sees_floor", pitch["front"]["ideal_distance_m"], 1)
        check("pitch_90_up_sees_east", pitch["up"]["ideal_distance_m"], 2.95)
        origins = {name: (.1, 0, .02) for name in center}
        offset = measure((0, -.6, 1), active=MultiRanger(caster, origins_body_m=origins))
        check("configurable_optical_origin", offset["front"]["ideal_distance_m"], .25)
        short = measure((0, 0, 1), active=MultiRanger(caster, max_distance_m=2))
        assert all(r["valid"] == 0 for r in short.values())
        print("PASS beyond maximum range: invalid", flush=True)

        # A ray crosses the live drone; raw queries must see self, filtered ones
        # must continue to the farther wall. This catches raycast_closest misuse.
        raw = PhysXRaycaster(excluded_roots=())((-.3, 0, .1), (1, 0, 0), 4)
        assert raw is not None and raw.collision_path.startswith(runtime.DRONE_PRIM_PATH), raw
        nonself = caster((-.3, 0, .1), (1, 0, 0), 4)
        check("self_ignored_farther_wall_kept", nonself.distance_m, 3.25)

        if not args.geometry_only:
            trajectories = {}
            # Reference uses the SAME noisy EKF sequence. Ranger has its own RNG.
            for label, ekf_noise, ranger_enabled, ranger_noise in (
                ("noise_off", False, True, False),
                ("noise_on", True, True, True),
                ("reference_no_ranger", True, False, False),
            ):
                controller = runtime.make_controller("ekf_closed_loop", noise=ekf_noise, seed=7)
                runtime._ranger = MultiRanger(caster, noise=ranger_noise, seed=7) if ranger_enabled else None
                runtime.reset_runtime(sim, controller)
                demo = TimedHoverMoveDemo(hover_hold_s=5)
                path = args.output_dir/f"{label}.csv"
                flight = FlightLog(path)
                ranging = RangerLog(path.with_suffix(".ranger.csv"), runtime._ranger) if ranger_enabled else None
                start = time.perf_counter()
                traces = []
                try:
                    for tick in range(round(demo.end_time_s/runtime.PHYSICS_DT)+1):
                        state = runtime.read_state(tick*runtime.PHYSICS_DT)
                        readings = runtime.sample_ranger(state)
                        if ranging:
                            ranging.write(readings)
                        target = demo.target(state)
                        command = controller.compute(state, target)
                        runtime.apply_command(command)
                        flight.write(state, target, command, demo.phase)
                        traces.append(np.concatenate((state["position_m"], state["orientation_wxyz"],
                                                       state["linear_velocity_m_s"], command["motor_commands"])))
                        sim.step(render=False)
                        if (tick+1) % runtime.RENDER_EVERY == 0:
                            sim.render()
                        if tick % 5000 == 0:
                            print(f"{label} t={state['time_s']:.1f}, phase={demo.phase}, position={state['position_m']}", flush=True)
                finally:
                    flight.close()
                    if ranging:
                        ranging.close()
                seconds = time.perf_counter()-start
                report = analyze(path)
                report["wall_seconds"] = seconds
                report["physics_steps_per_wall_second"] = (tick+1)/seconds
                result["flights"][label] = report
                assert report["stable"], report
                trajectories[label] = np.asarray(traces)
                if ranging:
                    assert runtime._ranger.generated_frames == runtime._ranger.delivered_frames == 250
                    assert ranging.rows == 1250
                    plot_ranger(ranging.path)
                    plot_ekf(path)
                print(f"PASS {label}: stable, {seconds:.2f} wall s", flush=True)
            np.testing.assert_array_equal(trajectories["noise_on"], trajectories["reference_no_ranger"])
            result["ranger_does_not_change_flight"] = True
            print("PASS Ranger on/off: bit-identical true states and motor commands with EKF noise on", flush=True)
        sim.stop()
        result["passed"] = True
        print("RANGER_VALIDATION_PASSED", flush=True)
    except Exception as exc:
        traceback.print_exc()
        result.update(passed=False, error=repr(exc))
        exit_code = 1
    finally:
        (args.output_dir/"summary.json").write_text(json.dumps(result, indent=2)+"\n")
        if exit_code:
            import omni.kit.app
            omni.kit.app.get_app().post_quit(exit_code)
        app.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
