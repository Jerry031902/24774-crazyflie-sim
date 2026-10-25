#!/usr/bin/env python3
"""Crazyflie single-body simulation with Python/C PID for Isaac Sim 5.1.0.

Run in the official container:
    /isaac-sim/python.sh /workspace/sim/run_sim.py
    /isaac-sim/python.sh /workspace/sim/run_sim.py --mode altitude_pid --target-z 1.0
    /isaac-sim/python.sh /workspace/sim/run_sim.py --mode free_fall
    /isaac-sim/python.sh /workspace/sim/run_sim.py --mode hover_ff
    /isaac-sim/python.sh /workspace/sim/run_sim.py --mode firmware_pid --demo
    /isaac-sim/python.sh /workspace/sim/run_sim.py --mode ekf_shadow --demo --noise off
    /isaac-sim/python.sh /workspace/sim/run_sim.py --mode ekf_closed_loop --demo --noise on

The bundled official model comes from NVIDIA's Isaac 5.1 asset collection:
https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/Bitcraze/Crazyflie/cf2x.usd
Its body is /crazyflie/body, remapped to /World/Crazyflie/body when referenced.
The four propellers are visual-only in this single-rigid-body approximation.
"""

import argparse
import math
from pathlib import Path

from controllers.constant_thrust import ConstantThrustController
from controllers.altitude_pid import AltitudePIDController

# Editable settings. Replace both placeholders with the team's measurements.
# INERTIA_KGM2 is about the COM, with principal axes aligned to body XYZ.
MASS_KG = 0.030
INERTIA_KGM2 = (1.6572e-5, 1.6656e-5, 2.9262e-5)  # Placeholder, kg*m^2.
MODE = "altitude_pid"  # Also "firmware_pid", "free_fall" or "hover_ff".
EKF_MODES = ("ekf_shadow", "ekf_closed_loop")
MOTOR_MODES = ("firmware_pid",) + EKF_MODES
GRAVITY_M_S2 = 9.81
PHYSICS_DT = 0.001
RENDER_EVERY = 10
PRINT_HZ = 10
INITIAL_POSITION = (0.0, 0.0, 0.1)
INITIAL_ORIENTATION_WXYZ = (1.0, 0.0, 0.0, 0.0)
# v0.3 altitude-only feedback. Gains act on acceleration, independent of mass.
TARGET_Z_M = 1.0
ALTITUDE_KP = 6.0  # 1/s^2
ALTITUDE_KI = 0.2  # 1/s^3
ALTITUDE_KD = 4.0  # 1/s
MAX_THRUST_N = 2.0 * MASS_KG * GRAVITY_M_S2  # Test limit, not a measured motor limit.
DRONE_PRIM_PATH = "/World/Crazyflie"
BODY_PRIM_PATH = DRONE_PRIM_PATH + "/body"
PHYSICS_PRIM_PATH = "/World/PhysicsScene"
SIM_DIR = Path(__file__).resolve().parent
ARENA_PATH = SIM_DIR / "scenes" / "arena.usd"
CRAZYFLIE_USD_PATH = SIM_DIR / "assets" / "crazyflie" / "cf2x.usd"

# Initialized after Kit and PhysX start. Forces use the body's actual local COM.
_body = None
_body_com = None
_motor_actuator = None


def apply_thrust(total_thrust_N):
    """Apply body +Z thrust at its COM for the next physics step, in newtons."""
    import numpy as np

    if not math.isfinite(total_thrust_N) or total_thrust_N < 0:
        raise ValueError("Thrust must be finite and nonnegative")
    if _body is None or not _body.is_physics_handle_valid():
        raise RuntimeError("Initialize/reset physics before applying thrust")
    if total_thrust_N == 0:
        return  # Free fall: no external force is submitted.
    _body.apply_forces_and_torques_at_pos(
        forces=np.array([[0.0, 0.0, total_thrust_N]], dtype=np.float32),
        positions=_body_com,
        is_global=False,  # Both force and application point are in body space.
    )


def apply_command(command):
    """Route total thrust or firmware motor commands, and return applied telemetry."""
    if "motor_commands" in command:
        if _motor_actuator is None:
            raise RuntimeError("Select firmware_pid mode to initialize motor actuators")
        command.update(_motor_actuator.apply(command["motor_commands"]))
    else:
        apply_thrust(command["total_thrust_N"])
    return command


def read_state(time_s):
    """Read live PhysX state every step, including steps without rendering."""
    position, quaternion = _body.get_world_poses()
    return {
        "time_s": time_s,
        "position_m": position[0].copy(),
        "orientation_wxyz": quaternion[0].copy(),  # Body-to-world, scalar first.
        "linear_velocity_m_s": _body.get_linear_velocities()[0].copy(),
        "angular_velocity_rad_s": _body.get_angular_velocities()[0].copy(),
    }


def _use_single_body(stage, root, body_prim, motor_positions=None):
    """Keep propeller geometry attached to body, without extra masses/joints.

    The source asset has four 0.8 g propeller bodies. Leaving them dynamic
    would require more than MASS_KG*g even after overriding the body mass.
    Runtime overrides affect only the open stage, not the source USD files.
    """
    from pxr import Gf, UsdGeom, UsdPhysics

    root.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    for index in range(1, 5):
        original = stage.GetPrimAtPath(f"{DRONE_PRIM_PATH}/m{index}_prop")
        joint = stage.GetPrimAtPath(f"{BODY_PRIM_PATH}/m{index}_joint")
        if not original or not joint:
            raise RuntimeError("Unexpected Crazyflie asset structure")
        # Reference the original propeller under body so it follows body motion.
        visual = stage.DefinePrim(f"{BODY_PRIM_PATH}/m{index}_visual", "Xform")
        visual.GetReferences().AddReference(
            str(CRAZYFLIE_USD_PATH), f"/crazyflie/m{index}_prop"
        )
        visual.RemoveAPI(UsdPhysics.RigidBodyAPI)
        visual.RemoveAPI(UsdPhysics.MassAPI)
        if motor_positions is not None:
            # NVIDIA's visual has 31 mm X/Y offsets. Match the firmware's
            # 46/sqrt(2) mm offsets at runtime; preserve visual height/orientation.
            transform = UsdGeom.Xformable(visual)
            matrix = transform.GetLocalTransformation()
            xyz = matrix.ExtractTranslation()
            xy = motor_positions[index - 1]
            matrix.SetTranslateOnly(Gf.Vec3d(float(xy[0]), float(xy[1]), xyz[2]))
            transform.MakeMatrixXform().Set(matrix)
        original.SetActive(False)
        joint.SetActive(False)

    # Keep robot metadata consistent with the simplified runtime model.
    root.GetRelationship("isaac:physics:robotLinks").SetTargets([body_prim.GetPath()])
    root.GetRelationship("isaac:physics:robotJoints").SetTargets([])
    mass = UsdPhysics.MassAPI.Apply(body_prim)
    mass.CreateMassAttr(MASS_KG)
    mass.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*INERTIA_KGM2))
    mass.CreatePrincipalAxesAttr(Gf.Quatf(1.0))


def reset_runtime(sim, controller):
    """Reinitialize handles, pose, velocities and controller after Stop/Play."""
    global _body_com
    import numpy as np

    sim.reset()
    _body.initialize()
    # Reset after PhysX initialization so startup updates cannot create drift.
    _body.set_world_poses(
        positions=np.array([INITIAL_POSITION]),
        orientations=np.array([INITIAL_ORIENTATION_WXYZ]),
    )
    _body.set_linear_velocities(np.zeros((1, 3), dtype=np.float32))
    _body.set_angular_velocities(np.zeros((1, 3), dtype=np.float32))
    _body_com = np.asarray(_body.get_coms()[0], dtype=np.float32).reshape(1, 3)
    actual_mass = float(_body.get_masses()[0])
    actual_inertia = np.asarray(_body.get_inertias()[0]).reshape(3, 3)
    print(f"PhysX total mass: {actual_mass:.9f} kg (one rigid body)", flush=True)
    print(f"PhysX inertia matrix about COM [kg*m^2]: {actual_inertia.tolist()}", flush=True)
    print(f"PhysX local COM [m]: {_body_com[0].tolist()}", flush=True)
    if not np.isclose(actual_mass, MASS_KG, rtol=1e-6, atol=1e-9):
        raise RuntimeError("PhysX body mass does not match MASS_KG")
    if not np.allclose(actual_inertia, np.diag(INERTIA_KGM2), rtol=1e-5, atol=1e-10):
        raise RuntimeError("PhysX inertia does not match INERTIA_KGM2")
    controller.reset()


def make_controller(mode, noise=False, seed=7):
    """Select a controller without importing Isaac Sim into controller code."""
    if mode == "altitude_pid":
        return AltitudePIDController(
            mass_kg=MASS_KG, gravity_m_s2=GRAVITY_M_S2, dt_s=PHYSICS_DT,
            kp=ALTITUDE_KP, ki=ALTITUDE_KI, kd=ALTITUDE_KD,
            max_thrust_N=MAX_THRUST_N,
        )
    if mode == "firmware_pid":
        from controllers.firmware_pid import FirmwarePIDController
        return FirmwarePIDController(dt_s=PHYSICS_DT, gravity_m_s2=GRAVITY_M_S2)
    if mode in EKF_MODES:
        from controllers.ekf_pid import EKFPIDController
        if PHYSICS_DT != 0.001 or GRAVITY_M_S2 != 9.81:
            raise ValueError("Stock EKF baseline requires 1 ms steps and gravity 9.81")
        return EKFPIDController(mode, noise=noise, seed=seed)
    if mode in ("free_fall", "hover_ff"):
        return ConstantThrustController(0.0 if mode == "free_fall" else MASS_KG * GRAVITY_M_S2)
    raise ValueError(f"Unknown mode: {mode}")


def setup_simulation(mode, target_z_m=TARGET_Z_M, noise=False, seed=7):
    """Open the arena, reference the drone, and start fixed-step physics."""
    global _body, _motor_actuator
    # All Isaac Sim/USD imports occur after main() has started SimulationApp.
    from isaacsim.core.api import SimulationContext
    from isaacsim.core.prims import RigidPrim
    from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage, open_stage
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

    controller = make_controller(mode, noise=noise, seed=seed)
    if not math.isfinite(MASS_KG) or MASS_KG <= 0:
        raise ValueError("MASS_KG must be positive and finite")
    if len(INERTIA_KGM2) != 3 or any(not math.isfinite(i) or i <= 0 for i in INERTIA_KGM2):
        raise ValueError("INERTIA_KGM2 must contain three positive finite moments")
    if 2 * max(INERTIA_KGM2) > sum(INERTIA_KGM2):
        raise ValueError("Principal moments must satisfy the inertia triangle inequality")
    for path in (ARENA_PATH, CRAZYFLIE_USD_PATH):
        if not path.is_file():
            raise FileNotFoundError(f"Required USD is missing: {path}")
    if not open_stage(str(ARENA_PATH)):
        raise RuntimeError(f"Could not open {ARENA_PATH}")
    stage = get_current_stage()
    if UsdGeom.GetStageUpAxis(stage) != "Z" or UsdGeom.GetStageMetersPerUnit(stage) != 1.0:
        raise ValueError("The arena must use meters and Z up")
    if not stage.GetPrimAtPath(PHYSICS_PRIM_PATH).IsA(UsdPhysics.Scene):
        raise RuntimeError(f"Arena has no PhysicsScene at {PHYSICS_PRIM_PATH}")

    root = add_reference_to_stage(str(CRAZYFLIE_USD_PATH), DRONE_PRIM_PATH)
    body_prim = stage.GetPrimAtPath(BODY_PRIM_PATH)
    if not body_prim or not body_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError(f"No rigid body at {BODY_PRIM_PATH}")
    UsdGeom.XformCommonAPI(root).SetTranslate(Gf.Vec3d(*INITIAL_POSITION))
    motor_positions = None
    _motor_actuator = None
    if mode in MOTOR_MODES:
        from actuators import MOTOR_POSITIONS_BODY_M, MotorActuator
        motor_positions = MOTOR_POSITIONS_BODY_M
    _use_single_body(stage, root, body_prim, motor_positions)
    if motor_positions is not None:
        import numpy as np
        cache = UsdGeom.XformCache()
        body_inverse = cache.GetLocalToWorldTransform(body_prim).GetInverse()
        for index, expected in enumerate(motor_positions, 1):
            visual = stage.GetPrimAtPath(f"{BODY_PRIM_PATH}/m{index}_visual")
            local = (cache.GetLocalToWorldTransform(visual) * body_inverse).ExtractTranslation()
            if not np.allclose(tuple(local)[:2], expected[:2], atol=1e-7):
                raise RuntimeError(f"M{index} visual and firmware geometry disagree")
            print(f"M{index} visual position in body [m]: {tuple(local)}", flush=True)
        bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
        print(f"Drone world-aligned bbox: {bbox.ComputeWorldBound(root).ComputeAlignedRange()}", flush=True)
    rigid_paths = [str(p.GetPath()) for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if rigid_paths != [BODY_PRIM_PATH] or any(p.IsA(UsdPhysics.Joint) for p in Usd.PrimRange(root)):
        raise RuntimeError("Expected exactly one rigid body and no active joints")
    # Small contact margins suit the drone's centimeter-scale collider.
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collision.CreateContactOffsetAttr(0.001)
            collision.CreateRestOffsetAttr(0.0)

    sim = SimulationContext(
        physics_dt=PHYSICS_DT,
        rendering_dt=PHYSICS_DT * RENDER_EVERY,
        stage_units_in_meters=1.0,
        physics_prim_path=PHYSICS_PRIM_PATH,
        backend="numpy",
        device="cpu",
    )
    sim.get_physics_context().set_gravity(-GRAVITY_M_S2)
    _body = RigidPrim(BODY_PRIM_PATH, name="crazyflie_body", reset_xform_properties=False)
    reset_runtime(sim, controller)
    if mode in MOTOR_MODES:
        _motor_actuator = MotorActuator(_body)
        print("Firmware PID: 1 kHz tick, position 100 Hz, attitude/rate 500 Hz; "
              f"gyro LPF {controller.gyro_filter_hz:g} Hz; 47-17 force model", flush=True)
    if mode in ("altitude_pid",) + MOTOR_MODES:
        # Frame both the starting position and target height during the ascent.
        set_camera_view(eye=[1.5, -1.5, max(1.2, target_z_m + 0.4)],
                        target=[0.0, 0.0, (INITIAL_POSITION[2] + target_z_m) / 2])
    else:
        set_camera_view(eye=[1.0, -1.0, 0.7], target=list(INITIAL_POSITION))
    print(f"Drone: {CRAZYFLIE_USD_PATH}\nBody: {BODY_PRIM_PATH}", flush=True)
    print(f"Mode: {mode}; mass: {MASS_KG:.6f} kg", flush=True)
    if mode in EKF_MODES:
        print(f"Sensors: {controller.suite.settings()}", flush=True)
        print("Stock EKF: predict 100 Hz; worker 1 kHz; FIFO updates; previous worker state to PID", flush=True)
    if mode == "altitude_pid":
        print(f"Target Z: {target_z_m:.3f} m; PID gains: {ALTITUDE_KP}, {ALTITUDE_KI}, {ALTITUDE_KD}; "
              f"thrust limit: {MAX_THRUST_N:.6f} N", flush=True)
    return sim, controller


def main():
    # GUI is the default. Optional bounded/headless runs support smoke checks.
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("free_fall", "hover_ff", "altitude_pid") + MOTOR_MODES, default=MODE)
    parser.add_argument("--target-z", type=float, default=TARGET_Z_M, help="Target altitude in meters")
    parser.add_argument("--headless", action="store_true", help="Run without a window")
    parser.add_argument("--steps", type=int, default=0, help="Exit after N physics steps; 0 runs until closed")
    parser.add_argument("--demo", action="store_true", help="Firmware PID: hover for 10 s, then move to X=0.5 m")
    parser.add_argument("--csv", type=Path, help="Firmware PID flight log, one row per physics step")
    parser.add_argument("--noise", choices=("off", "on"), default="off", help="EKF simulated sensor noise")
    parser.add_argument("--seed", type=int, default=7, help="EKF sensor noise seed")
    args = parser.parse_args()
    if args.steps < 0:
        parser.error("--steps cannot be negative")
    if not math.isfinite(args.target_z) or args.target_z <= 0:
        parser.error("--target-z must be positive and finite")
    if (args.demo or args.csv) and args.mode not in MOTOR_MODES:
        parser.error("--demo and --csv require a firmware controller mode")
    if args.noise == "on" and args.mode not in EKF_MODES:
        parser.error("--noise on requires an EKF mode")
    if args.seed < 0:
        parser.error("--seed must be nonnegative")

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless})  # False by default.
    logger = None
    try:
        sim, controller = setup_simulation(args.mode, args.target_z, noise=args.noise == "on", seed=args.seed)
        demo = None
        if args.demo or args.csv:
            from flight_demo import FlightLog, HoverMoveDemo, TimedHoverMoveDemo
            demo_type = TimedHoverMoveDemo if args.mode in EKF_MODES else HoverMoveDemo
            demo = demo_type(args.target_z) if args.demo else None
            default_csv = (f"build/ekf_baseline/{args.mode}_noise_{args.noise}.csv"
                           if args.mode in EKF_MODES else "build/firmware_pid/demo.csv")
            csv_path = args.csv or SIM_DIR.parent / default_csv
            logger = FlightLog(csv_path)
            print(f"Flight CSV: {csv_path}", flush=True)
        target = {"position_m": (INITIAL_POSITION[0], INITIAL_POSITION[1], args.target_z)}
        print_every = round(1.0 / (PRINT_HZ * PHYSICS_DT))
        steps = 0
        needs_reset = False
        while app.is_running():
            # Continue processing window events while the timeline is paused.
            if not sim.is_playing():
                needs_reset = needs_reset or sim.is_stopped()
                sim.render()
                continue
            if needs_reset:
                reset_runtime(sim, controller)
                steps = 0
                needs_reset = False
                if demo is not None:
                    demo.reset()
                if logger is not None:
                    logger.close()
                    logger = FlightLog(csv_path)  # A fresh CSV for the restarted episode.

            state = read_state(steps * PHYSICS_DT)
            if demo is not None:
                target = demo.target(state)
            command = controller.compute(state, target)
            apply_command(command)
            if logger is not None:
                logger.write(state, target, command, demo.phase if demo else "hold")
            sim.step(render=False)  # Exactly one 1 ms physics step per command.
            steps += 1
            if steps % print_every == 0:
                state = read_state(steps * PHYSICS_DT)
                x, y, z = state["position_m"]
                vz = state["linear_velocity_m_s"][2]
                print(f"t={state['time_s']:.3f} s  position=({x:.5f}, {y:.5f}, {z:.5f}) m  "
                      f"vz={vz:.4f} m/s  thrust={command['total_thrust_N']:.5f} N", flush=True)
            if steps % RENDER_EVERY == 0:
                sim.render()  # UI/render update only, with physics disabled.
            if args.steps and steps >= args.steps:
                break
            if args.headless and demo is not None:
                if demo.done:
                    print("FIRMWARE_DEMO_COMPLETE", flush=True)
                    break
                if steps * PHYSICS_DT >= 40.0:
                    raise RuntimeError("Firmware demo did not complete within 40 simulation seconds")
        if app.is_running():
            sim.stop()
    except KeyboardInterrupt:
        pass
    except Exception:
        # Print errors before Kit shutdown, which may terminate Python itself.
        import traceback

        traceback.print_exc()
        import omni.kit.app
        omni.kit.app.get_app().post_quit(1)
        raise
    finally:
        if logger is not None:
            logger.close()
        app.close()


if __name__ == "__main__":
    main()
