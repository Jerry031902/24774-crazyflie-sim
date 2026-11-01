"""Offline Ranger contract tests, no SimulationApp, GPU, or firmware build."""

import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ranger import (MultiRanger, PhysXRaycaster, RangerLog, RayHit,
                    RANGER_DIRECTIONS_BODY, RANGER_INVALID_MM)


def truth(ms, q=(1., 0., 0., 0.), position=(0., 0., 1.)):
    return {"time_s": ms/1000, "position_m": position, "orientation_wxyz": q}


class RangerTests(unittest.TestCase):
    def test_all_hits_ignore_self_and_choose_nearest_regardless_of_order(self):
        # Exercise the real callback without importing the Kit extension.
        caster = PhysXRaycaster.__new__(PhysXRaycaster)
        caster.excluded_roots = ("/World/Crazyflie",)
        caster.vec = lambda *values: values
        hits = [SimpleNamespace(collision=p, rigid_body=b, distance=d) for p, b, d in (
            ("/World/Wall", "", 3),
            ("/World/Crazyflie/body/mesh", "/World/Crazyflie/body", .01),
            ("/World/OtherCollider", "/World/Crazyflie/body", .02),
            ("/World/CrazyflieOther", "", .5),
            ("/World/Pillar", "", 1))]
        def query(origin, direction, distance, callback, both_sides):
            for hit in hits:
                self.assertTrue(callback(hit))
        caster.query = SimpleNamespace(raycast_all=query)
        hit = caster((0, 0, 0), (1, 0, 0), 4)
        self.assertEqual(hit, RayHit(.5, "/World/CrazyflieOther"))

    def test_origins_and_axes_rotate_with_body(self):
        calls = []
        def cast(origin, direction, distance):
            calls.append((origin.copy(), direction.copy()))
            return RayHit(1.2344, "/Wall")
        origins = {name: (.1, 0, .02) for name in RANGER_DIRECTIONS_BODY}
        sensor = MultiRanger(cast, origins_body_m=origins)
        rows = sensor.measure(truth(100, q=(math.sqrt(.5), 0, 0, math.sqrt(.5))))
        np.testing.assert_allclose(calls[0][0], (0, .1, 1.02), atol=1e-12)
        np.testing.assert_allclose(calls[0][1], (0, 1, 0), atol=1e-12)
        np.testing.assert_allclose(calls[2][1], (-1, 0, 0), atol=1e-12)
        self.assertEqual(rows[0]["range_mm"], 1234)
        tilted = sensor.measure(truth(200, q=(math.sqrt(.5), 0, math.sqrt(.5), 0)))
        np.testing.assert_allclose([tilted[0][f"direction_{a}"] for a in "xyz"], (0, 0, -1), atol=1e-12)

    def test_10_hz_delivery_delay_and_held_values(self):
        sensor = MultiRanger(lambda *args: RayHit(1, "/Wall"), delay_ms=25)
        self.assertEqual(sensor.snapshot()["range.front"], 0)
        deliveries = []
        for ms in range(1026):
            rows = sensor.sample(truth(ms, position=(ms/1000, 0, 1)))
            if ms == 124:
                self.assertEqual(sensor.snapshot()["range_front_valid"], 0)
            if rows:
                deliveries.append(ms)
                self.assertEqual(len(rows), 5)
                self.assertEqual(rows[0]["origin_x_m"], (ms-25)/1000)
                self.assertEqual(rows[0]["sample_time_s"], (ms-25)/1000)
                self.assertEqual(rows[0]["delivery_time_s"], ms/1000)
        self.assertEqual(deliveries, list(range(125, 1026, 100)))
        self.assertEqual(sensor.generated_frames, 10)
        self.assertEqual(sensor.delivered_frames, 10)
        self.assertEqual(sensor.snapshot()["range.front"], 1000)
        with self.assertRaises(ValueError):
            sensor.sample(truth(1025))  # Duplicate scheduling is a bug.
        sensor.reset()
        self.assertIsNone(sensor.latest)
        self.assertEqual(sensor.sample(truth(0)), [])

    def test_invalid_range_and_noise_never_turn_no_return_into_obstacle(self):
        for hit in (None, RayHit(4.001, "/Far"), RayHit(-.1, "/Bad")):
            sensor = MultiRanger(lambda *args: hit, noise=True)
            for row in sensor.measure(truth(100)):
                self.assertEqual(row["range_mm"], RANGER_INVALID_MM)
                self.assertEqual(row["valid"], 0)
                self.assertTrue(math.isnan(row["distance_m"]))
        sensor = MultiRanger(lambda *args: RayHit(3.999, "/Wall"), bias_m=.002)
        self.assertEqual(sensor.measure(truth(100))[0]["status"], "out_of_range")

    def test_repeatable_noise_reset_and_off(self):
        sensor = MultiRanger(lambda *args: RayHit(1, "/Wall"), noise=True, seed=9)
        first = [r["range_mm"] for r in sensor.measure(truth(100))]
        sensor.reset()
        self.assertEqual(first, [r["range_mm"] for r in sensor.measure(truth(100))])
        self.assertNotEqual(first, [1000]*5)
        sensor.noise = False
        self.assertEqual([r["range_mm"] for r in sensor.measure(truth(200))], [1000]*5)

    def test_packet_csv_and_metadata(self):
        sensor = MultiRanger(lambda *args: RayHit(.5, "/Wall"))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"flight.ranger.csv"
            log = RangerLog(path, sensor)
            for ms in range(201):
                log.write(sensor.sample(truth(ms)))
            log.close()
            with path.open() as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), 10)
            self.assertEqual(rows[-1]["range_mm"], "500")
            self.assertEqual(json.loads(path.with_suffix(".json").read_text())["delivered_frames"], 2)

    def test_reject_bad_configuration(self):
        for options in ({"delay_ms": -.1}, {"delay_ms": .5}, {"std_m": -1},
                        {"bias_m": math.nan}, {"seed": -1}, {"max_distance_m": 0},
                        {"max_distance_m": 32.767}, {"origins_body_m": {}}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                MultiRanger(lambda *args: None, **options)


if __name__ == "__main__":
    unittest.main()
