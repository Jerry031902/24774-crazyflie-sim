#!/usr/bin/env python3
"""Build the Isaac Sim scene for a navigation mission (sim/navigation/missions.py).

Run from the project root inside the official container:
    /isaac-sim/python.sh sim/build_mission_scene.py                  # matlab_main
    /isaac-sim/python.sh sim/build_mission_scene.py --mission NAME

Writes sim/scenes/<mission.scene> (e.g. matlab_room.usd). It never touches
arena.usd; that scene stays build_scene.py's. Room walls stand just outside
the mission's xlim/ylim, so their inner faces are the walls the 2-D
simulator uses. Obstacles are fixed colliders from the floor to
mission.obstacle_height. Lighting, physics and contact settings match
build_scene.py.
"""

import argparse
import math
from pathlib import Path
import sys

SIM_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM_DIR))

import build_scene as arena  # noqa: E402  Constants only; arena.usd is not rebuilt.
from navigation.missions import MISSIONS  # noqa: E402

DEFAULT_OBSTACLE_COLOR = (0.55, 0.55, 0.60)


def build_mission_scene(mission, output_path):
    # These imports must happen AFTER SimulationApp starts Kit.
    import numpy as np
    from isaacsim.core.api.objects import FixedCuboid, FixedCylinder
    from isaacsim.core.utils.stage import create_new_stage, get_current_stage, save_stage
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

    if output_path.resolve() == arena.OUTPUT_PATH.resolve():
        raise ValueError("Refusing to overwrite arena.usd; give the mission its own scene file")
    create_new_stage()
    stage = get_current_stage()
    if stage is None:
        raise RuntimeError("Failed to create a USD stage")
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    physics = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics.CreateGravityDirectionAttr(Gf.Vec3f(*arena.GRAVITY_DIRECTION))
    physics.CreateGravityMagnitudeAttr(arena.GRAVITY_MAGNITUDE)

    def finish(obj):
        obj.set_contact_offset(arena.CONTACT_OFFSET)
        obj.set_rest_offset(0.0)

    def add_box(path, center, size, color, yaw_deg=0.0):
        half = math.radians(yaw_deg) / 2
        box = FixedCuboid(prim_path=path, name=path.rsplit("/", 1)[-1],
                          position=np.array(center, dtype=float),
                          orientation=np.array([math.cos(half), 0.0, 0.0, math.sin(half)]),
                          scale=np.array(size, dtype=float), size=1.0, color=np.array(color, dtype=float))
        # Same 5.1 extent correction as build_scene.py.
        UsdGeom.Cube(box.prim).GetExtentAttr().Set([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
        finish(box)

    (x0, x1), (y0, y1) = mission.xlim, mission.ylim
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    width, depth = x1 - x0, y1 - y0
    t, h = arena.WALL_THICKNESS, arena.WALL_HEIGHT

    # Floor slab under the room and its walls, top at z = 0.
    add_box("/World/Ground", (cx, cy, -arena.GROUND_THICKNESS / 2),
            (width + 2 * t, depth + 2 * t, arena.GROUND_THICKNESS), arena.GROUND_COLOR)
    UsdGeom.Xform.Define(stage, "/World/Walls")
    for name, center, size in (
        ("North", (cx, y1 + t / 2, h / 2), (width + 2 * t, t, h)),
        ("South", (cx, y0 - t / 2, h / 2), (width + 2 * t, t, h)),
        ("East", (x1 + t / 2, cy, h / 2), (t, depth, h)),
        ("West", (x0 - t / 2, cy, h / 2), (t, depth, h)),
    ):
        add_box(f"/World/Walls/{name}", center, size, arena.WALL_COLOR)

    UsdGeom.Xform.Define(stage, "/World/Obstacles")
    height = mission.obstacle_height
    for o in mission.obstacles:
        path = f"/World/Obstacles/{o['name']}"
        color = mission.colors.get(o["name"], DEFAULT_OBSTACLE_COLOR)
        x, y = o["center"]
        if o["kind"] == "box":
            add_box(path, (x, y, height / 2), (o["size"][0], o["size"][1], height), color, o.get("angle_deg", 0.0))
        else:
            finish(FixedCylinder(prim_path=path, name=o["name"], position=np.array([x, y, height / 2]),
                                 radius=float(o["radius"]), height=height, color=np.array(color, dtype=float)))

    UsdGeom.Xform.Define(stage, "/World/Lights")
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/DistantLight")
    sun.CreateIntensityAttr(arena.DISTANT_LIGHT_INTENSITY)
    sun.CreateColorAttr(Gf.Vec3f(*arena.LIGHT_COLOR))
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(*arena.DISTANT_LIGHT_ROTATION))
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/DomeLight")
    dome.CreateIntensityAttr(arena.DOME_LIGHT_INTENSITY)
    dome.CreateColorAttr(Gf.Vec3f(*arena.LIGHT_COLOR))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not save_stage(str(output_path), save_and_reload_in_place=False):
        raise RuntimeError(f"Failed to save scene: {output_path}")
    print(f"Saved mission scene: {output_path}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mission", choices=sorted(MISSIONS), default="matlab_main")
    args = parser.parse_args()
    mission = MISSIONS[args.mission]

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        build_mission_scene(mission, SIM_DIR / "scenes" / mission.scene)
    except Exception:
        import traceback

        traceback.print_exc()
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
