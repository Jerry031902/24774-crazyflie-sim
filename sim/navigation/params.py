"""Every navigation setting in one object.

Port of matlab-trajectory-sim/src/config/default_params.m, same names and
values. Change fields after calling default_params() rather than editing
the defaults, e.g.  p = default_params(); p.nav.v_cruise = 0.3
"""

import math
from types import SimpleNamespace


def default_params():
    p = SimpleNamespace()

    # Simulation (2-D test simulator only; Isaac Sim uses its own 1 kHz loop)
    p.dt = 0.02               # navigator tick [s]
    p.t_max = 300.0           # hard cap on simulated time [s]
    p.seed = 1                # RNG seed (wind gusts, sensor noise)
    p.stop_on_collision = True

    # Vehicle: 2-D kinematic point that can also yaw; the rangers turn with the body
    p.radius = 0.07           # body radius including propellers [m]
    p.v_max = 1.0             # speed limit [m/s]
    p.a_max = 2.0             # acceleration limit [m/s^2]
    p.tau = 0.25              # velocity time constant [s]
    p.yaw_rate_max = math.radians(180)  # [rad/s]
    p.psi0 = 0.0              # initial yaw [rad], 0 = forward along +x

    # Disturbance (2-D test simulator only)
    p.wind = (0.0, 0.0)       # constant wind drift [m/s]
    p.wind_gust = 0.0         # std dev of random gusts [m/s], 0 = off

    # Multi-ranger deck: VL53L1x time-of-flight sensors facing front, left, back, right
    p.ranger = SimpleNamespace(
        range_max=4.0,              # beyond this: no return (inf) [m]
        fov=math.radians(27),       # cone width; 0 = a single centre ray [rad]
        n_rays=9,                   # rays cast across the cone; the nearest hit is reported
        period=0.04,                # update period [s]
        noise=0.0,                  # Gaussian range noise: std dev [m]...
        noise_rel=0.0,              # ...plus this fraction of the range [-]
    )

    # Navigation: straight-line GO_TO_GOAL; WedgeBug AVOID
    p.nav = SimpleNamespace(
        v_cruise=0.5,               # GO_TO_GOAL speed [m/s]
        v_follow=0.3,               # wall-following speed [m/s]
        k_stop=2.0,                 # speed = k_stop * distance to the next stop point [1/s]
        k_yaw=4.0,                  # yaw rate = k_yaw * heading error [1/s]
        yaw_tol=math.radians(10),   # heading error above which the drone stops and turns on the spot
        yaw_free_radius=0.2,        # within this of a target, slide onto it without turning [m]
        d_sweep=1.0,                # stop this far from an obstacle ahead and sweep [m]
        sweep_half=math.radians(45),  # sweep +/- this about the blocked heading
        sweep_rate=math.radians(60),  # yaw rate while sweeping [rad/s]
        edge_jump=0.5,              # a range jump this big marks the end of an obstacle's outline [m]
        pass_margin=0.4,            # fly at least this far outside a seen edge [m]
        tangent_clearance=0.5,      # ...and far enough out that the next leg keeps this from the obstacle [m]
        pass_max=1.5,               # but never more than this far out [m]
        max_sweeps=12,              # sweeps on one obstacle before the target counts as unreachable
        max_winding=2.5 * math.pi,  # circling the target this far without getting closer: unreachable [rad]
        d_side=0.25,                # side reading that switches GO_TO_GOAL to wall-following [m]
        d_detect=0.6,               # wall-following: a way counts as clear beyond this [m]
        d_wall=0.3,                 # stop this far from whatever is ahead [m]
        d_open=0.6,                 # wall-side reading above this means the wall has ended [m]
        d_close=0.22,               # wall-side reading below this: too close, turn away [m]
        corner_margin=0.3,          # extra travel past a wall's end before turning back toward it [m]
        v_turn=0.05,                # brake below this speed before changing direction [m/s]
        wp_tol=0.10,                # a waypoint counts as reached within this distance [m]
        leave_margin=0.10,          # leave the wall only this much closer to the target than the hit point [m]
        loop_tol=0.15,              # back within this of where wall-following began...  [m]
        loop_min_travel=1.0,        # ...after following at least this far = full loop, unreachable [m]
    )

    # End of run and metrics
    p.goal_vtol = 0.05        # the run ends once the goal is reached and speed is below this [m/s]
    p.near_miss = 0.10        # a body-to-obstacle gap below this counts as a near miss [m]
    return p
