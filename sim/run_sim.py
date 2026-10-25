#!/usr/bin/env python3
"""Crazyflie rigid-body smoke simulation for Isaac Sim 5.1.0.

Run in the official container:
    /isaac-sim/python.sh /workspace/sim/run_sim.py
    /isaac-sim/python.sh /workspace/sim/run_sim.py --mode hover_ff

The bundled official model comes from NVIDIA's Isaac 5.1 asset collection:
https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/Bitcraze/Crazyflie/cf2x.usd
Its body is /crazyflie/body, remapped to /World/Crazyflie/body when referenced.
The four propellers are visual-only in this single-rigid-body approximation.
"""

import argparse
import math
from pathlib import Path

from controllers.constant_thrust import ConstantThrustController

# Editable settings. Replace both placeholders with the team's measurements.
# INERTIA_KGM2 is about the COM, with principal axes aligned to body XYZ.
MASS_KG = 0.030
INERTIA_KGM2 = (1.6572e-5, 1.6656e-5, 2.9262e-5)  # Placeholder, kg*m^2.
MODE = "free_fall"  # "free_fall" or "hover_ff"
GRAVITY_M_S2 = 9.81
PHYSICS_DT = 0.001
RENDER_EVERY = 10
PRINT_HZ = 10
INITIAL_POSITION = (0.0, 0.0, 0.1)
INITIAL_ORIENTATION_WXYZ = (1.0, 0.0, 0.0, 0.0)
DRONE_PRIM_PATH = "/World/Crazyflie"
BODY_PRIM_PATH = DRONE_PRIM_PATH + "/body"
PHYSICS_PRIM_PATH = "/World/PhysicsScene"
SIM_DIR = Path(__file__).resolve().parent
ARENA_PATH = SIM_DIR / "scenes" / "arena.usd"
CRAZYFLIE_USD_PATH = SIM_DIR / "assets" / "crazyflie" / "cf2x.usd"

# Initialized after Kit and PhysX start. Forces use the body's actual local COM.
_body = None
_body_com = None


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


def _use_single_body(stage, root, body_prim):
    """Keep propeller geometry attached to body, without extra masses/joints.

    The source asset has four 0.8 g propeller bodies. Leaving them dynamic
    would require more than MASS_KG*g even after overriding the body mass.
    Runtime overrides affect only the open stage, not the source USD files.
    """
    from pxr import Gf, UsdPhysics

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


def setup_simulation(mode):
    """Open the arena, reference the drone, and start fixed-step physics."""
    global _body
    # All Isaac Sim/USD imports occur after main() has started SimulationApp.
    from isaacsim.core.api import SimulationContext
    from isaacsim.core.prims import RigidPrim
    from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage, open_stage
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

    if mode not in ("free_fall", "hover_ff"):
        raise ValueError(f"Unknown mode: {mode}")
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
    _use_single_body(stage, root, body_prim)
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
    thrust = 0.0 if mode == "free_fall" else MASS_KG * GRAVITY_M_S2
    controller = ConstantThrustController(thrust)
    reset_runtime(sim, controller)
    set_camera_view(eye=[1.0, -1.0, 0.7], target=list(INITIAL_POSITION))
    print(f"Drone: {CRAZYFLIE_USD_PATH}\nBody: {BODY_PRIM_PATH}", flush=True)
    print(f"Mode: {mode}; mass: {MASS_KG:.6f} kg; thrust: {thrust:.6f} N", flush=True)
    return sim, controller


def main():
    # GUI is the default. Optional bounded/headless runs support smoke checks.
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("free_fall", "hover_ff"), default=MODE)
    parser.add_argument("--headless", action="store_true", help="Run without a window")
    parser.add_argument("--steps", type=int, default=0, help="Exit after N physics steps; 0 runs until closed")
    args = parser.parse_args()
    if args.steps < 0:
        parser.error("--steps cannot be negative")

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless})  # False by default.
    try:
        sim, controller = setup_simulation(args.mode)
        target = {"position_m": INITIAL_POSITION}
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

            state = read_state(steps * PHYSICS_DT)
            command = controller.compute(state, target)
            apply_thrust(command["total_thrust_N"])
            sim.step(render=False)  # Exactly one 1 ms physics step per command.
            steps += 1
            if steps % print_every == 0:
                state = read_state(steps * PHYSICS_DT)
                x, y, z = state["position_m"]
                print(f"t={state['time_s']:.3f} s  position=({x:.5f}, {y:.5f}, {z:.5f}) m", flush=True)
            if steps % RENDER_EVERY == 0:
                sim.render()  # UI/render update only, with physics disabled.
            if args.steps and steps >= args.steps:
                break
        if app.is_running():
            sim.stop()
    except KeyboardInterrupt:
        pass
    except Exception:
        # Print errors before Kit shutdown, which may terminate Python itself.
        import traceback

        traceback.print_exc()
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
