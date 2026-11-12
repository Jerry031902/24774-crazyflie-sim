"""WedgeBug navigator: Python reference port, the C firmware core, and their agreement.

Python-only tests run with any Python 3 + numpy. C tests need cffirmware built
with the WedgeBug core (make bindings_python) and are skipped otherwise:
    /isaac-sim/python.sh -m pytest sim/tests/test_wedgebug.py
Ports of matlab-trajectory-sim/tests/test_drone_sim.m where they apply.
"""

import math
from pathlib import Path
import sys
import unittest
import warnings

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.geometry import (Environment, obstacle_circle, obstacle_rect, obstacle_square,  # noqa: E402
                                 read_rangers, rot2d)
from navigation.missions import MATLAB_MAIN  # noqa: E402
from navigation.params import default_params  # noqa: E402
from navigation.sim2d import compute_metrics, run_simulation  # noqa: E402
from navigation.wedgebug import RangerFrame, WedgeBugNavigator  # noqa: E402

try:
    import cffirmware as cf
    HAVE_C = hasattr(cf, "wedgebugStep")
except ImportError:
    cf, HAVE_C = None, False

ROOM = ((0.0, 10.0), (0.0, 8.0))
BACKENDS = ("python", "firmware") if HAVE_C else ("python",)


def room(*obstacles):
    return Environment(*ROOM, obstacles)


def fly(env, start, goal, waypoints=None, backend="python", p=None):
    p = p or default_params()
    log = run_simulation(start, waypoints, goal, env, p, backend=backend)
    return log, compute_metrics(log, env, p)


def sideways_speed(log):
    """Speed across the body axes; near zero = every move is along a sensor's line of sight."""
    v = np.array([min(abs(rot2d(psi).T @ x[2:4])) for psi, x in zip(log["psi"], log["x"])])
    v[log["dir"] == 0] = 0
    return v


class RangerTests(unittest.TestCase):
    def test_rangers_see_walls_and_obstacles(self):
        p = default_params()
        p.ranger.noise = 0                          # exact geometry, no bench noise
        p.ranger.fov, p.ranger.offset, p.ranger.resolution = 0, 0, 0  # exact centre-ray geometry
        env = room(obstacle_rect((5, 4), (1, 1)))
        np.testing.assert_allclose(read_rangers((3, 4), 0, env, p), [1.5, 4, 3, 4], atol=1e-12)
        np.testing.assert_allclose(read_rangers((3, 4), math.pi / 2, env, p), [4, 3, 4, 1.5], atol=1e-12)

    def test_sensor_faces_and_whole_millimetres(self):
        # Multi-ranger datasheet: faces 16.25 mm (front/back) and 16.3 mm (left/right) from the centre.
        p = default_params()
        p.ranger.noise = 0                          # exact geometry, no bench noise
        p.ranger.fov = 0
        env = room(obstacle_rect((5, 4), (1, 1)))
        np.testing.assert_allclose(read_rangers((3, 4), 0, env, p), [1.484, 3.984, 2.984, 3.984], atol=1e-9)

    def test_readings_arrive_one_driver_period_late(self):
        p = default_params()
        p.ranger.noise = 0                          # exact geometry, no bench noise
        log, _ = fly(room(), (1, 4), (2, 4), p=p)
        ticks = round(p.ranger.latency / p.dt)
        self.assertTrue(np.all(np.isinf(log["ranges"][:ticks])))
        expected = read_rangers(log["x"][0, 0:2], log["psi"][0], room(), p)
        np.testing.assert_allclose(log["ranges"][ticks], expected)

    def test_invalid_readings_are_nan(self):
        p = default_params()
        p.ranger.p_invalid = 1.0
        env = room(obstacle_rect((5, 4), (1, 1)))
        self.assertTrue(np.all(np.isnan(read_rangers((3, 4), 0, env, p, np.random.default_rng(0)))))

    def test_cone_sees_block_off_the_centre_line(self):
        p = default_params()
        p.ranger.noise = 0                          # exact geometry, no bench noise
        env = room(obstacle_rect((5, 4.2), (0.2, 0.2)))
        self.assertAlmostEqual(read_rangers((2, 4), 0, env, p)[0], 2.9, delta=0.05)
        p.ranger.fov = 0
        self.assertTrue(math.isinf(read_rangers((2, 4), 0, env, p)[0]))


class ReadingRuleTests(unittest.TestCase):
    """Hysteresis on range thresholds, and unknown (nan) readings."""

    def navigator(self):
        return WedgeBugNavigator((0, 0), [(5, 0)], default_params())

    def test_hysteresis_turns_on_at_threshold_and_off_past_it(self):
        nav = self.navigator()
        # d_side = 0.25 (strict <), hysteresis 0.05; nan changes nothing.
        for left, near in ((0.26, False), (0.25, False), (0.24, True), (0.29, True), (0.31, False),
                           (math.nan, False), (0.24, True), (math.nan, True)):
            nav.step(np.zeros(4), 0.0, [math.inf, left, math.inf, math.inf], 0.0)
            self.assertEqual(bool(nav.near["side"][1]), near, f"left = {left}")

    def test_unknown_ahead_holds_until_it_reads(self):
        nav = self.navigator()
        u, _ = nav.step(np.zeros(4), 0.0, [math.nan, math.inf, math.inf, math.inf], 0.0)
        np.testing.assert_allclose(u, [0, 0])
        u, _ = nav.step(np.zeros(4), 0.0, [3.0, math.inf, math.inf, math.inf], 0.02)
        np.testing.assert_allclose(u, [nav.p.nav.v_cruise, 0])

    def test_slows_down_passing_close_to_a_side_obstacle(self):
        nav = self.navigator()
        u, _ = nav.step(np.zeros(4), 0.0, [3.0, 0.40, math.inf, math.inf], 0.0)
        np.testing.assert_allclose(u, [nav.p.nav.k_stop * (0.40 - nav.p.nav.d_close), 0])
        nav = self.navigator()
        u, _ = nav.step(np.zeros(4), 0.0, [3.0, math.nan, math.inf, math.inf], 0.0)
        np.testing.assert_allclose(u, [nav.p.nav.v_cruise, 0])

    def test_wall_following_turns_parallel_to_the_wall(self):
        # Following a wall on the left that closes in at 10 deg: successive
        # left readings fit its direction and psi_hold turns to match it.
        nav = WedgeBugNavigator((0, 0), [(-5, 0)], default_params())
        nav.mode, nav.phase, nav.episode, nav.hit_dist = "AVOID", "follow", True, 0.0
        nav.hand, nav.travel_dir, nav.exec_dir, nav.following, nav.wall_seen = 1, 1, 1, True, True
        slope = math.tan(math.radians(10))
        for seq in range(1, 40):
            px = 0.03 * seq
            frame = RangerFrame(seq, 0.1 * seq, 0.0, np.array([math.inf, 0.45 - slope * px, math.inf, math.inf]))
            nav.step([px, 0.0, 0.3, 0.0], 0.0, frame.ranges, 0.1 * seq, frame)
        self.assertAlmostEqual(math.degrees(nav.psi_hold), -10.0, delta=0.5)

    def test_unknown_is_neither_an_edge_nor_free_space(self):
        nav = self.navigator()
        nav.sweep_center = 0.0
        ang = np.radians(np.arange(-45.0, 46.0))
        rng = np.ones(ang.size)
        rng[50:53] = math.nan                       # unknown at +5..+7 deg, inside a wall 1 m ahead
        nav.scan = {"t": [0.0] * ang.size, "ang": list(ang), "rng": list(rng)}
        w = nav._read_wedge(np.zeros(2), np.array([5.0, 0.0]))
        self.assertTrue(w["blocked"])
        self.assertEqual(w["seen"], [False, False])
        rng[50:53] = math.inf                       # a real gap with no return does read as an edge
        nav.scan["rng"] = list(rng)
        self.assertTrue(nav._read_wedge(np.zeros(2), np.array([5.0, 0.0]))["seen"][0])
        nav.scan["rng"] = [math.nan] * ang.size     # nothing known ahead: blocked, not clear
        self.assertTrue(nav._read_wedge(np.zeros(2), np.array([5.0, 0.0]))["blocked"])
        self.assertEqual(nav._free_toward(np.array([2.0, 0.0]), np.array([math.nan, math.inf, math.inf, math.inf])), 0.0)
        self.assertTrue(math.isinf(nav._free_toward(np.array([2.0, 0.0]), np.full(4, math.inf))))


class NavigationTests(unittest.TestCase):
    def test_matlab_main_mission_matches_matlab_results(self):
        # Default (bench-measured, 2 mm) ranger noise: 75.4 s, closest gap 0.13 m.
        for backend in BACKENDS:
            with self.subTest(backend=backend):
                env = MATLAB_MAIN.environment()
                _, m = fly(env, MATLAB_MAIN.start, MATLAB_MAIN.goal, MATLAB_MAIN.waypoints, backend)
                self.assertTrue(m["arrived"])
                self.assertFalse(m["collided"])
                self.assertEqual(m["targets_reached"], 4)
                self.assertAlmostEqual(m["t_finish"], 75.4, delta=0.1)
                self.assertAlmostEqual(m["min_clearance"], 0.126, delta=0.01)

    def test_open_room_flies_straight_line(self):
        a, b = np.array([1, 1]), np.array([8.5, 6.5])
        for backend in BACKENDS:
            with self.subTest(backend=backend):
                log, m = fly(room(), a, b, backend=backend)
                self.assertTrue(m["arrived"])
                n = (b - a) / np.linalg.norm(b - a)
                off = np.abs((log["x"][:, 0] - a[0]) * n[1] - (log["x"][:, 1] - a[1]) * n[0])
                self.assertLess(off.max(), 0.02)
                self.assertEqual(np.count_nonzero(log["mode"] == 2), 0)

    def test_shapes_are_passed_with_every_move_sensed(self):
        shapes = {
            "square": ([obstacle_square((5, 4), 1.5)], (1, 4), (9, 4.3)),
            "long wall": ([obstacle_rect((5, 4.2), (0.2, 6.4))], (2, 4), (8, 3.5)),
            "rotated rect": ([obstacle_rect((5, 4), (2.5, 0.5), 30)], (1, 3), (9, 5)),
            "big circle": ([obstacle_circle((5, 4), 1.0)], (1, 4), (9, 4.2)),
            "pillar": ([obstacle_circle((5, 4), 0.2)], (1, 4), (9, 4)),
            "L of rects": ([obstacle_rect((5, 2.3), (2, 0.6)), obstacle_rect((5.7, 4), (0.6, 4))], (1, 1), (8, 5)),
            "U of rects": ([obstacle_rect((5.5, 2.3), (3, 0.6)), obstacle_rect((5.5, 5.7), (3, 0.6)),
                            obstacle_rect((6.7, 4), (0.6, 4))], (1, 4), (8.5, 4)),
        }
        p = default_params()
        for backend in BACKENDS:
            for name, (obstacles, start, goal) in shapes.items():
                with self.subTest(backend=backend, shape=name):
                    log, m = fly(room(*obstacles), start, goal, backend=backend)
                    self.assertTrue(m["arrived"])
                    self.assertFalse(m["collided"])
                    self.assertGreaterEqual(m["avoid_episodes"], 1)
                    self.assertGreater(m["min_clearance"], 0.1)
                    self.assertLessEqual(sideways_speed(log).max(), p.nav.v_turn + 0.01)

    def test_goes_round_the_shorter_side(self):
        for backend in BACKENDS:
            for center, side in (((5, 3.4), +1), ((5, 4.6), -1)):
                with self.subTest(backend=backend, side=side):
                    log, m = fly(room(obstacle_rect(center, (0.3, 2.4))), (1, 4), (9, 4), backend=backend)
                    self.assertTrue(m["arrived"])
                    self.assertEqual(log["sweeps"][0]["side"], side)
                    y = log["x"][np.abs(log["x"][:, 0] - 5) < 0.2, 1]
                    if side > 0:
                        self.assertGreater(y.min(), 4.6)
                    else:
                        self.assertLess(y.max(), 3.4)
                    self.assertLess(m["path_ratio"], 1.2)

    def test_enclosed_waypoint_is_reported_and_skipped(self):
        ring = [obstacle_rect((7, 2.1), (2.2, 0.2)), obstacle_rect((7, 5.9), (2.2, 0.2)),
                obstacle_rect((6, 4), (0.2, 4)), obstacle_rect((8, 4), (0.2, 4))]
        for backend in BACKENDS:
            with self.subTest(backend=backend):
                log, m = fly(room(*ring), (1, 4), (9, 7), waypoints=[(7, 4)], backend=backend)
                self.assertEqual(log["unreachable"], [0])
                self.assertTrue(m["arrived"])
                self.assertFalse(m["collided"])

    def test_waypoint_inside_obstacle_is_rejected(self):
        with self.assertRaises(ValueError):
            fly(room(obstacle_rect((5, 4), (2, 2))), (1, 1), (9, 7), waypoints=[(5, 4)])

    def test_blind_drone_collides(self):
        p = default_params()
        p.ranger.range_max = 0.01
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            log, _ = fly(room(obstacle_rect((5, 4), (1, 1))), (1, 4), (9, 4), p=p)
        self.assertTrue(log["collided"])
        self.assertLess(log["x"][-1, 0], 4.5)


@unittest.skipUnless(HAVE_C, "Build cffirmware with the WedgeBug core (make bindings_python)")
class FirmwareCoreTests(unittest.TestCase):
    def test_c_defaults_match_python_params(self):
        from navigation.firmware_nav import c_default_params
        p = default_params()
        for name, value in c_default_params().items():
            group, _, field = name.rpartition(".")
            expected = getattr(getattr(p, group) if group else (p if name == "yaw_rate_max" else p.nav), field)
            self.assertAlmostEqual(value, expected, places=5, msg=name)

    def test_c_replays_python_decisions(self):
        """Feed the Python run's exact inputs to the C core, tick by tick."""
        from navigation.firmware_nav import FirmwareWedgeBug
        from navigation.wedgebug import MODES
        p = default_params()
        env = MATLAB_MAIN.environment()
        log = run_simulation(MATLAB_MAIN.start, MATLAB_MAIN.waypoints, MATLAB_MAIN.goal, env, p)
        nav = FirmwareWedgeBug(MATLAB_MAIN.start, MATLAB_MAIN.targets, p)
        u_err, modes_differ = [], 0
        for k in range(len(log["t"])):
            u, r = nav.step(log["x"][k], log["psi"][k], log["ranges"][k], log["t"][k])
            u_err.append(max(np.max(np.abs(u - log["u_body"][k])), abs(r - log["yaw_rate"][k])))
            modes_differ += MODES.index(nav.mode) + 1 != log["mode"][k]
        u_err = np.array(u_err)
        # Single precision: tiny differences, never a different decision.
        self.assertEqual(modes_differ, 0)
        self.assertLess(np.mean(u_err > 1e-3), 0.002)

    def test_firmware_pid_hover_setpoint(self):
        from controllers.firmware_pid import FirmwarePIDController
        c = FirmwarePIDController()
        state = {"time_s": 0.0, "position_m": (0, 0, 1), "orientation_wxyz": (1, 0, 0, 0),
                 "angular_velocity_rad_s": (0, 0, 0), "linear_velocity_m_s": (0, 0, 0)}
        c.compute(state, {"position_m": (9, 9, 1.2), "velocity_body_m_s": (0.3, -0.1), "yaw_rate_rad_s": 0.5})
        sp = c.setpoint
        self.assertEqual((sp.mode.x, sp.mode.y, sp.mode.z, sp.mode.yaw),
                         (cf.modeVelocity, cf.modeVelocity, cf.modeAbs, cf.modeVelocity))
        self.assertTrue(sp.velocity_body)
        self.assertAlmostEqual(sp.velocity.x, 0.3, places=6)
        self.assertAlmostEqual(sp.velocity.y, -0.1, places=6)
        self.assertAlmostEqual(sp.position.z, 1.2, places=6)
        self.assertAlmostEqual(sp.attitudeRate.yaw, math.degrees(0.5), places=4)


if __name__ == "__main__":
    unittest.main()
