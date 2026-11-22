"""Obstacle-free flow experiment scene and actual PhysX contact reports.

The tracked arena is never edited. A 200 m floor extends beyond the 20 m
software termination bound; there are no walls or pillars to contact.
"""

import csv
from pathlib import Path


def make_empty_scene(source, output):
    from pxr import Usd, UsdGeom, UsdPhysics
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    source_stage = Usd.Stage.Open(str(source))  # Keep the stage alive while exporting its layer.
    source_stage.GetRootLayer().Export(str(output))
    stage = Usd.Stage.Open(str(output))
    for path in ("/World/Walls", "/World/Obstacles"):
        stage.RemovePrim(path)
    ground = stage.GetPrimAtPath("/World/Ground")
    # The source uses an orient op, which XformCommonAPI cannot edit reliably.
    scale = ground.GetAttribute("xformOp:scale")
    scale.Set(type(scale.Get())(200., 200., .05))
    bound = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(ground).ComputeAlignedRange()
    if any(abs(bound.GetSize()[i]-200.) > 1e-5 for i in (0, 1)) or abs(bound.GetMax()[2]) > 1e-6:
        raise AssertionError(f"Unexpected floor bounds: {bound}")
    colliders = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    if colliders != ["/World/Ground"]:
        raise AssertionError(f"Unexpected empty-scene colliders: {colliders}")
    stage.GetRootLayer().Save()
    return output, {"scene": "flow-empty", "floor_size_m": 200., "position_bound_m": 20.,
                    "static_colliders": colliders, "source_scene": str(source),
                    "floor_bounds_m": [list(bound.GetMin()), list(bound.GetMax())]}


class ContactLog:
    """Subscribe to the installed PhysX contact API; no geometry heuristics."""

    @staticmethod
    def enable(stage, body_path):
        from pxr import PhysxSchema
        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(body_path)).CreateThresholdAttr(0.)

    def __init__(self, path):
        from omni.physx import get_physx_simulation_interface
        self.file = Path(path).open("w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=(
            "time_s", "event", "actor0", "actor1", "collider0", "collider1", "contact_points"))
        self.writer.writeheader()
        self.time_s, self.events, self.first_contact_s = 0., 0, None
        self.subscription = get_physx_simulation_interface().subscribe_contact_report_events(self._event)

    def _event(self, headers, data):
        from pxr import PhysicsSchemaTools
        for h in headers:
            paths = {k: str(PhysicsSchemaTools.intToSdfPath(getattr(h, k)))
                     for k in ("actor0", "actor1", "collider0", "collider1")}
            if not any(p.startswith("/World/Crazyflie") for p in paths.values()):
                continue
            self.writer.writerow(dict(time_s=self.time_s, event=str(h.type),
                                      contact_points=h.num_contact_data, **paths))
            self.events += 1
            if h.num_contact_data and self.first_contact_s is None:
                self.first_contact_s = self.time_s

    def close(self):
        self.subscription = None
        self.file.close()

    def summary(self):
        return {"events": self.events, "first_contact_s": self.first_contact_s,
                "method": "PhysX contact report, threshold 0; callback timestamp is physics step end"}
