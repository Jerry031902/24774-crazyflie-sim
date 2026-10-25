"""PhysX sign tests, THEN C PID takeoff/10 s hover/0.5 m translation.

Host command (inside repo root, built bindings required):
  docker exec cf_sim_dev /isaac-sim/python.sh /workspace/sim/tests/smoke_firmware_pid.py
Outputs: build/firmware_pid/{sign_tests.json,summary.json,flight.csv,flight.png}.
"""

import argparse
import json
from pathlib import Path
import sys
import traceback


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parents[2] / "build/firmware_pid")
    parser.add_argument("--sign-only", action="store_true")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    report = {"passed": False, "sign_tests": []}
    logger = None
    exit_code = 0
    try:
        import numpy as np
        import cffirmware as cf
        import run_sim as runtime
        from actuators import COMMAND_MAX, THRUST_MAX_N
        from controllers.firmware_pid import FirmwarePIDController, rotation_and_rpy
        from flight_demo import FlightLog, HoverMoveDemo, HOVER_HOLD_S
        from plot_flight import plot_flight

        sim, controller = runtime.setup_simulation("firmware_pid")
        body, actuator = runtime._body, runtime._motor_actuator
        dt = runtime.PHYSICS_DT
        hover = round(runtime.MASS_KG * runtime.GRAVITY_M_S2 / (4 * THRUST_MAX_N) * COMMAND_MAX)

        def check(name, condition, **metrics):
            entry = {"name": name, "passed": bool(condition), **metrics}
            report["sign_tests"].append(entry)
            print(f"{'PASS' if condition else 'FAIL'} {name}: {json.dumps(metrics)}", flush=True)
            (args.output_dir / "sign_tests.json").write_text(json.dumps(report["sign_tests"], indent=2))
            if not condition:
                raise AssertionError(name)

        def place(q=(1, 0, 0, 0)):
            body.set_world_poses(positions=np.array([[0, 0, 1]], dtype=np.float32),
                                orientations=np.array([q], dtype=np.float32))
            body.set_linear_velocities(np.zeros((1, 3), dtype=np.float32))
            body.set_angular_velocities(np.zeros((1, 3), dtype=np.float32))

        def pulse(commands, count=20):
            for i in range(count):
                applied = actuator.apply(commands)
                sim.step(render=False)
                if (i+1) % runtime.RENDER_EVERY == 0:
                    sim.render()
            state = runtime.read_state(count*dt)
            _, rpy = rotation_and_rpy(state["orientation_wxyz"])
            return state, np.degrees(rpy) * [1, -1, 1], applied

        place()
        state, angles, applied = pulse([hover]*4, count=100)
        force_error = abs(applied["total_thrust_N"] - runtime.MASS_KG*runtime.GRAVITY_M_S2)
        check("equal motors: hover force and no rotation",
              force_error < 5e-6 and np.linalg.norm(state["angular_velocity_rad_s"]) < 1e-4
              and abs(state["position_m"][2]-1) < 1e-4,
              total_force_N=applied["total_thrust_N"], mg_N=runtime.MASS_KG*runtime.GRAVITY_M_S2,
              omega_rad_s=state["angular_velocity_rad_s"].tolist(), z_m=float(state["position_m"][2]))

        # Expected WORLD/RH angular velocity signs at identity, derived directly
        # from firmware SI mixer columns (power_distribution_quadrotor.c:104-107).
        # This oracle does not use the adapter's motor-position/sign arrays.
        expected_signs = [(-1, -1, -1), (-1, 1, 1), (1, 1, -1), (1, -1, 1)]
        for motor, signs in enumerate(expected_signs):
            place()
            commands = [hover]*4
            commands[motor] += 1000
            state, angles, _ = pulse(commands)
            omega = state["angular_velocity_rad_s"]
            check(f"M{motor+1} increase: roll/pitch/yaw signs", np.all(omega*np.array(signs) > 1e-4),
                  expected_RH_signs=signs, omega_rad_s=omega.tolist(), attitude_legacy_deg=angles.tolist())

        # A positive raw LEGACY control field has signs +X, -Y, -Z. In particular
        # raw control.yaw is negated by controllerPid relative to desired yaw.
        # controller_pid.c:149; power_distribution_quadrotor.c:84-92.
        for axis, index, expected in (("roll", 0, 1), ("pitch", 1, -1), ("yaw", 2, -1)):
            place()
            control = cf.control_t()
            control.controlMode = cf.controlModeLegacy
            control.thrust = hover
            setattr(control, axis, 500)
            motors = cf.motors_thrust_uncapped_t()
            cf.powerDistribution(control, motors)
            state, angles, _ = pulse([getattr(motors.motors, f"m{i}") for i in range(1, 5)])
            omega = state["angular_velocity_rad_s"]
            off_axis = np.delete(omega, index)
            check(f"positive legacy control.{axis}", omega[index]*expected > .001 and np.linalg.norm(off_axis) < .001,
                  expected_RH_sign=expected, omega_rad_s=omega.tolist())

        # Now use actual controllerPid outputs for positive desired legacy Euler
        # roll/pitch/yaw. Desired yaw positive produces raw control.yaw NEGATIVE.
        for index, axis in enumerate(("roll", "pitch", "yaw")):
            place()
            controller.reset()
            angles_target = [0.0]*3
            angles_target[index] = 2.0
            command = controller.compute(runtime.read_state(0),
                                         {"attitude_legacy_deg": angles_target, "collective_command": hover})
            raw = command["control_legacy"][index+1]
            state, angles, _ = pulse(command["motor_commands"])
            check(f"C PID positive {axis} setpoint", angles[index] > .005 and raw*(1 if index < 2 else -1) > 0,
                  raw_control=raw, attitude_legacy_deg=angles.tolist())

        for pair, expected, spin in (((0, 2), -1, "CCW"), ((1, 3), 1, "CW")):
            place()
            commands = [hover]*4
            for i in pair:
                commands[i] += 1000
            state, angles, applied = pulse(commands)
            omega = state["angular_velocity_rad_s"]
            check(f"{spin} rotor pair: opposite yaw reaction", omega[2]*expected > .001 and np.linalg.norm(omega[:2]) < 1e-4,
                  omega_rad_s=omega.tolist(), yaw_torque_Nm=float(sum(applied["motor_yaw_torques_Nm"])))

        # Check force frame with a tilted body, not just identity orientation.
        place(q=(np.cos(np.pi/12), 0, np.sin(np.pi/12), 0))
        state, _, _ = pulse([hover]*4)
        check("equal thrust follows tilted body +Z", state["linear_velocity_m_s"][0] > .05
              and state["linear_velocity_m_s"][2] < -.01
              and np.linalg.norm(state["angular_velocity_rad_s"]) < 1e-4,
              velocity_world_m_s=state["linear_velocity_m_s"].tolist())
        print("ALL_SIGN_TESTS_PASSED", flush=True)

        if not args.sign_only:
            # Flight begins only after every physical sign test has passed.
            runtime.reset_runtime(sim, controller)
            demo = HoverMoveDemo(target_z_m=1.0)
            csv_path = args.output_dir / "flight.csv"
            logger = FlightLog(csv_path)
            previous_phase = None
            for step in range(round(40 / dt)):
                state = runtime.read_state(step*dt)
                target = demo.target(state)
                if demo.phase != previous_phase:
                    print(f"FLIGHT t={step*dt:.3f} phase={demo.phase} position={state['position_m'].tolist()}", flush=True)
                    previous_phase = demo.phase
                command = controller.compute(state, target)
                runtime.apply_command(command)
                logger.write(state, target, command, demo.phase)
                sim.step(render=False)
                if (step+1) % runtime.RENDER_EVERY == 0:
                    sim.render()
                if not np.isfinite(state["position_m"]).all() or np.linalg.norm(state["position_m"]) > 3:
                    raise AssertionError("Flight diverged")
                if demo.done:
                    break
            if not demo.done:
                raise AssertionError("Flight did not finish within 40 simulation seconds")
            logger.close()
            logger = None
            data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
            hover_mask = (data["time_s"] >= demo.move_time_s-HOVER_HOLD_S) & (data["time_s"] < demo.move_time_s)
            xyz = np.column_stack([data[f"{axis}_m"] for axis in "xyz"])
            hover_error = np.linalg.norm(xyz[hover_mask]-[0, 0, 1], axis=1)
            final_error = np.linalg.norm(xyz[-1]-[.5, 0, 1])
            assert np.max(hover_error) < .03 and final_error < .03
            report["flight"] = {"duration_s": float(data["time_s"][-1]),
                                "hover_start_s": demo.hover_start_s, "move_time_s": demo.move_time_s,
                                "continuous_hover_s": demo.move_time_s-demo.hover_start_s,
                                "max_hover_position_error_m": float(np.max(hover_error)),
                                "final_position_m": xyz[-1].tolist(), "final_position_error_m": float(final_error),
                                "peak_takeoff_z_m": float(np.max(data["z_m"][data["time_s"] < demo.move_time_s])),
                                "max_abs_roll_pitch_deg": float(max(np.max(abs(data["roll_deg"])), np.max(abs(data["pitch_legacy_deg"])))),
                                "saturated_steps": int(sum(data["saturated"]))}
            print("PASS closed-loop flight:", json.dumps(report["flight"]), flush=True)
            print("PLOT", plot_flight(csv_path), flush=True)

        report["passed"] = True
        report["mass_kg"] = runtime.MASS_KG
        report["inertia_kgm2"] = runtime.INERTIA_KGM2
        report["gyro_lpf_hz"] = controller.gyro_filter_hz
        sim.stop()
        print("FIRMWARE_PID_VALIDATION_PASSED", flush=True)
    except Exception as exc:
        report["error"] = repr(exc)
        print(f"FAIL firmware validation: {exc}", flush=True)
        traceback.print_exc()
        exit_code = 1
    finally:
        if logger is not None:
            logger.close()
        (args.output_dir / "summary.json").write_text(json.dumps(report, indent=2))
        if exit_code:
            # Kit's normal fast shutdown exits before main() can return a code.
            # Set its public quit status first so FAIL is visible to automation.
            import omni.kit.app
            omni.kit.app.get_app().post_quit(exit_code)
        app.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
