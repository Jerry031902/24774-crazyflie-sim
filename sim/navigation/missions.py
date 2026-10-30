"""Missions: a room, its obstacles, and start -> waypoints -> goal.

Each mission is used three ways with the same numbers: the 2-D test
simulator (navigation/sim2d.py), the Isaac Sim scene builder
(build_mission_scene.py), and the Isaac Sim flight (run_sim.py --mission).

Coordinates are Isaac Sim world X/Y in metres. Obstacles are vertical
prisms standing on the floor:
    {"kind": "box", "name", "center": (x, y), "size": (w, d), "angle_deg"}
    {"kind": "cylinder", "name", "center": (x, y), "radius"}
"""

from dataclasses import dataclass, field

from .geometry import Environment, obstacle_circle, obstacle_rect


@dataclass(frozen=True)
class Mission:
    name: str
    description: str
    xlim: tuple
    ylim: tuple
    obstacles: tuple
    start: tuple
    waypoints: tuple
    goal: tuple
    scene: str                      # USD file in sim/scenes/
    obstacle_height: float = 2.0    # [m]
    colors: dict = field(default_factory=dict)

    @property
    def targets(self):
        return list(self.waypoints) + [self.goal]

    def environment(self):
        polys = []
        for o in self.obstacles:
            if o["kind"] == "box":
                polys.append(obstacle_rect(o["center"], o["size"], o.get("angle_deg", 0.0)))
            elif o["kind"] == "cylinder":
                polys.append(obstacle_circle(o["center"], o["radius"]))
            else:
                raise ValueError(f"Unknown obstacle kind: {o['kind']}")
        return Environment(self.xlim, self.ylim, polys)


# matlab-trajectory-sim/main.m, unchanged: a 10 x 8 m room with its corner at
# the origin. Each leg has one obstacle in the way:
#   start -> 1: box    1 -> 2: long wall    2 -> 3: pillar    3 -> goal: bench
MATLAB_MAIN = Mission(
    name="matlab_main",
    description="matlab-trajectory-sim/main.m mission: box, long wall, pillar, bench",
    xlim=(0.0, 10.0),
    ylim=(0.0, 8.0),
    obstacles=(
        {"kind": "box", "name": "Box", "center": (2.5, 3.0), "size": (1.0, 1.0)},
        {"kind": "box", "name": "LongWall", "center": (5.5, 4.0), "size": (0.2, 3.4)},
        {"kind": "cylinder", "name": "Pillar", "center": (8.0, 2.9), "radius": 0.3},
        {"kind": "box", "name": "Bench", "center": (7.6, 5.3), "size": (1.6, 0.5)},
    ),
    start=(0.8, 0.8),
    waypoints=((3.4, 4.2), (6.6, 2.0), (9.4, 3.8)),
    goal=(6.4, 6.4),
    scene="matlab_room.usd",
    colors={"Box": (0.85, 0.25, 0.15), "LongWall": (0.15, 0.45, 0.85),
            "Pillar": (0.20, 0.65, 0.30), "Bench": (0.90, 0.65, 0.15)},
)

MISSIONS = {m.name: m for m in (MATLAB_MAIN,)}
