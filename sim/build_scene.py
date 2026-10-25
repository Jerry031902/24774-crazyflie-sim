#!/usr/bin/env python3
"""Build an offline arena with Isaac Sim 5.1.0.

Run from the project root inside the official container:
    /isaac-sim/python.sh sim/build_scene.py

Re-running this script replaces sim/scenes/arena.usd.
"""

from pathlib import Path

# Scene configuration. Dimensions and positions are in meters; colors are RGB.
ARENA_SIZE = (6.0, 6.0)
GROUND_THICKNESS = 0.05
WALL_HEIGHT = 2.0
WALL_THICKNESS = 0.05
OBSTACLE_SIZE = (0.30, 0.30, 2.0)  # Tall square pillars, XYZ dimensions.
GROUND_COLOR = (0.35, 0.38, 0.42)
WALL_COLOR = (0.75, 0.78, 0.82)
GRAVITY_DIRECTION = (0.0, 0.0, -1.0)
GRAVITY_MAGNITUDE = 9.81
CONTACT_OFFSET = 0.002
DISTANT_LIGHT_INTENSITY = 1500.0
DISTANT_LIGHT_ROTATION = (-30.0, -20.0, 0.0)  # XYZ Euler angles, degrees.
DOME_LIGHT_INTENSITY = 500.0
LIGHT_COLOR = (1.0, 1.0, 1.0)

# Positions specify each pillar's bottom-center (x, y, z). z=0 rests on the floor.
# Overall flight direction: +X, from x=-2.4 to x=2.4.
# Pillars appear in sequence, staggered left/right around the route center.
# Their vertical faces extend from z=0 to z=2 m for horizontal ranging.
# Both ends stay clear for takeoff/arrival.
OBSTACLES = [
    {"name": "Box01", "position": (-1.5, -0.45, 0.0),
     "size": OBSTACLE_SIZE, "color": (0.85, 0.25, 0.15)},
    {"name": "Box02", "position": (-0.5, 0.55, 0.0),
     "size": OBSTACLE_SIZE, "color": (0.15, 0.45, 0.85)},
    {"name": "Box03", "position": (0.5, -0.60, 0.0),
     "size": OBSTACLE_SIZE, "color": (0.20, 0.65, 0.30)},
    {"name": "Box04", "position": (1.5, 0.40, 0.0),
     "size": OBSTACLE_SIZE, "color": (0.90, 0.65, 0.15)},
]
OUTPUT_PATH = Path(__file__).resolve().parent / "scenes" / "arena.usd"


def build_arena():
    # These imports must happen AFTER SimulationApp starts Kit.
    import numpy as np
    from isaacsim.core.api.objects import FixedCuboid
    from isaacsim.core.utils.stage import create_new_stage, get_current_stage, save_stage
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

    # Start a fresh, meter-scale, Z-up stage with a default root prim.
    # In 5.1, create_new_stage returns a bool despite its documented Stage type.
    create_new_stage()
    stage = get_current_stage()
    if stage is None:
        raise RuntimeError("Failed to create a USD stage")
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # Gravity is a unit direction plus a magnitude in meters per second squared.
    physics = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics.CreateGravityDirectionAttr(Gf.Vec3f(*GRAVITY_DIRECTION))
    physics.CreateGravityMagnitudeAttr(GRAVITY_MAGNITUDE)

    def add_box(path, center, size, color):
        # size=1 makes scale equal to the final XYZ dimensions in meters.
        # FixedCuboid authors collision, but does not add rigid-body dynamics.
        # Its default colored material is an embedded USD Preview Surface.
        box = FixedCuboid(
            prim_path=path,
            name=path.rsplit("/", 1)[-1],
            position=np.array(center, dtype=float),
            scale=np.array(size, dtype=float),
            size=1.0,
            color=np.array(color, dtype=float),
        )
        # 5.1's cuboid wrapper includes scale in extent, although USD extent
        # must be local and unscaled. Correct it for this unit-sized cube.
        UsdGeom.Cube(box.prim).GetExtentAttr().Set(
            [Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)]
        )
        # Use a small contact margin appropriate for a nano quadrotor.
        box.set_contact_offset(CONTACT_OFFSET)
        box.set_rest_offset(0.0)

    # A thin slab gives a finite 6x6 m ground collider, with its top at z=0.
    # This also avoids the infinite collision plane used by GroundPlane.
    width, depth = ARENA_SIZE
    add_box("/World/Ground", (0.0, 0.0, -GROUND_THICKNESS / 2),
            (width, depth, GROUND_THICKNESS), GROUND_COLOR)

    # Walls sit within the floor footprint; the outer bounds remain 6x6 m.
    UsdGeom.Xform.Define(stage, "/World/Walls")
    half_x = (width - WALL_THICKNESS) / 2
    half_y = (depth - WALL_THICKNESS) / 2
    wall_z = WALL_HEIGHT / 2
    for name, center, size in (
        ("North", (0.0, half_y, wall_z), (width, WALL_THICKNESS, WALL_HEIGHT)),
        ("South", (0.0, -half_y, wall_z), (width, WALL_THICKNESS, WALL_HEIGHT)),
        ("East", (half_x, 0.0, wall_z),
         (WALL_THICKNESS, depth - 2 * WALL_THICKNESS, WALL_HEIGHT)),
        ("West", (-half_x, 0.0, wall_z),
         (WALL_THICKNESS, depth - 2 * WALL_THICKNESS, WALL_HEIGHT)),
    ):
        add_box(f"/World/Walls/{name}", center, size, WALL_COLOR)

    # Convert bottom-center positions to the centers expected by FixedCuboid.
    UsdGeom.Xform.Define(stage, "/World/Obstacles")
    for obstacle in OBSTACLES:
        x, y, bottom_z = obstacle["position"]
        size = obstacle["size"]
        add_box(f"/World/Obstacles/{obstacle['name']}",
                (x, y, bottom_z + size[2] / 2), size, obstacle["color"])

    # Both lights are procedural: the dome uses a constant color, no HDR image.
    UsdGeom.Xform.Define(stage, "/World/Lights")
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/DistantLight")
    sun.CreateIntensityAttr(DISTANT_LIGHT_INTENSITY)
    sun.CreateColorAttr(Gf.Vec3f(*LIGHT_COLOR))
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(
        Gf.Vec3f(*DISTANT_LIGHT_ROTATION)
    )
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/DomeLight")
    dome.CreateIntensityAttr(DOME_LIGHT_INTENSITY)
    dome.CreateColorAttr(Gf.Vec3f(*LIGHT_COLOR))

    # Save one self-contained USD file. No robot or external asset is added.
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    if not save_stage(str(OUTPUT_PATH), save_and_reload_in_place=False):
        raise RuntimeError(f"Failed to save arena: {OUTPUT_PATH}")
    print(f"Saved arena: {OUTPUT_PATH}", flush=True)
    print(f"File size: {OUTPUT_PATH.stat().st_size / 1024:.2f} KB", flush=True)


if __name__ == "__main__":
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": True})
    try:
        build_arena()
    except Exception:
        # Kit shutdown may terminate Python; report failures before closing it.
        import traceback

        traceback.print_exc()
        raise
    finally:
        simulation_app.close()
