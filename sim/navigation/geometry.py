"""2-D rooms, obstacles, distances and the planar Multi-ranger model.

Ports of matlab-trajectory-sim/src/env, src/sensors/read_rangers.m and
src/util. Points are numpy arrays [x, y] in metres; angles in radians,
counter-clockwise, 0 along +x (the same as the Isaac Sim world frame seen
from above).
"""

import math

import numpy as np


def wrap_to_pi(a):
    """Wrap angles into [-pi, pi)."""
    return np.mod(np.asarray(a, dtype=float) + math.pi, 2 * math.pi) - math.pi


def rot2d(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def sat_norm(v, vmax):
    """Scale v down so its 2-norm does not exceed vmax, keeping its direction."""
    n = float(np.linalg.norm(v))
    return v * (vmax / n) if n > vmax else v


# ----- obstacles -----------------------------------------------------------

def obstacle_rect(center, size, angle_deg=0.0):
    """w-by-h rectangle centred at center, rotated counter-clockwise."""
    hw, hh = size[0] / 2, size[1] / 2
    V = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]])
    V = V @ rot2d(math.radians(angle_deg)).T + np.asarray(center, dtype=float)
    return {"type": "rect", "vertices": V}


def obstacle_square(center, side, angle_deg=0.0):
    obs = obstacle_rect(center, (side, side), angle_deg)
    obs["type"] = "square"
    return obs


def obstacle_circle(center, r, n=48):
    """Circle as an n-sided polygon that circumscribes it."""
    th = np.arange(n) * (2 * math.pi / n)
    V = np.asarray(center, dtype=float) + (r / math.cos(math.pi / n)) * np.column_stack([np.cos(th), np.sin(th)])
    return {"type": "circle", "vertices": V}


class Environment:
    """A closed rectangular room plus polygon obstacles.

    edges holds every wall segment, room walls included, one per row as
    [ax ay bx by]; used by the 2-D rangers and the collision check.
    """

    def __init__(self, xlim, ylim, obstacles=()):
        self.xlim = tuple(map(float, xlim))
        self.ylim = tuple(map(float, ylim))
        self.obstacles = list(obstacles)
        room = np.array([[self.xlim[0], self.ylim[0]], [self.xlim[1], self.ylim[0]],
                         [self.xlim[1], self.ylim[1]], [self.xlim[0], self.ylim[1]]])
        edges = [np.hstack([room, np.roll(room, -1, axis=0)])]
        for obs in self.obstacles:
            V = obs["vertices"]
            edges.append(np.hstack([V, np.roll(V, -1, axis=0)]))
        self.edges = np.vstack(edges)


def _inside_polygon(P, V):
    """Even-odd point-in-polygon test for each row of P."""
    x, y = P[:, 0:1], P[:, 1:2]
    W = np.roll(V, -1, axis=0)
    ya, yb = V[:, 1], W[:, 1]
    crosses = (ya > y) != (yb > y)
    with np.errstate(divide="ignore", invalid="ignore"):
        x_cross = V[:, 0] + (y - ya) * (W[:, 0] - V[:, 0]) / (yb - ya)
    return np.count_nonzero(crosses & (x < x_cross), axis=1) % 2 == 1


def obstacle_distance(env, P):
    """Signed distance from points P (Nx2) to the nearest obstacle or room wall.

    Negative inside an obstacle or outside the room.
    """
    P = np.atleast_2d(np.asarray(P, dtype=float))
    d = np.min(np.column_stack([P[:, 0] - env.xlim[0], env.xlim[1] - P[:, 0],
                                P[:, 1] - env.ylim[0], env.ylim[1] - P[:, 1]]), axis=1)
    for obs in env.obstacles:
        V = obs["vertices"]
        W = np.roll(V, -1, axis=0)
        dk = np.full(len(P), np.inf)
        for a, b in zip(V, W):
            ab = b - a
            ap = P - a
            s = np.clip((ap @ ab) / (ab @ ab), 0, 1)
            dk = np.minimum(dk, np.linalg.norm(ap - s[:, None] * ab, axis=1))
        inside = _inside_polygon(P, V)
        dk[inside] = -dk[inside]
        d = np.minimum(d, dk)
    return d


def target_names(n):
    return [f"waypoint {i}" for i in range(1, n)] + ["goal"]


def validate_waypoints(start, targets, env, d_wall):
    """Raise ValueError if the start or any target is within d_wall of an obstacle or wall."""
    points = np.vstack([np.asarray(start, dtype=float)[None, :], np.asarray(targets, dtype=float)])
    names = ["start"] + target_names(len(targets))
    d = obstacle_distance(env, points)
    bad = np.flatnonzero(d < d_wall)
    if bad.size == 0:
        return
    lines = []
    for b in bad:
        where = "is inside an obstacle" if d[b] < 0 else f"is only {d[b]:.2f} m from an obstacle or wall"
        lines.append(f"  {names[b]} ({points[b, 0]:.2f}, {points[b, 1]:.2f}) {where}")
    raise ValueError("These points cannot be reached:\n" + "\n".join(lines)
                     + f"\nKeep every point at least d_wall = {d_wall:.2f} m from obstacles and walls.")


# ----- planar rangers --------------------------------------------------------

def ranger_spread(p):
    """Ray angles across one sensor's cone, relative to its axis."""
    r = p.ranger
    if r.n_rays > 1 and r.fov > 0:
        return np.linspace(-r.fov / 2, r.fov / 2, r.n_rays)
    return np.zeros(1)


def apply_ranger_noise(r, p, rng):
    """Gaussian noise on each return, then the range limit (MATLAB read_rangers order)."""
    r = np.array(r, dtype=float)
    if p.ranger.noise > 0 or p.ranger.noise_rel > 0:
        hit = np.isfinite(r)
        sigma = p.ranger.noise + p.ranger.noise_rel * r[hit]
        r[hit] = r[hit] + sigma * rng.standard_normal(np.count_nonzero(hit))
    r[r > p.ranger.range_max] = np.inf
    return np.maximum(r, 0.0)


def read_rangers(pos, psi, env, p, rng=None):
    """[front, left, back, right] distances for a drone at pos with yaw psi.

    Each sensor casts p.ranger.n_rays rays over its cone against every wall
    segment and reports the nearest hit; inf = no return within range_max.
    """
    A = env.edges[:, 0:2]
    E = env.edges[:, 2:4] - A
    W = A - np.asarray(pos, dtype=float)
    spread = ranger_spread(p)
    r = np.full(4, np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for j in range(4):
            th = psi + j * math.pi / 2 + spread
            U = np.vstack([np.cos(th), np.sin(th)])
            den = np.outer(E[:, 1], U[0]) - np.outer(E[:, 0], U[1])
            t = ((W[:, 0] * E[:, 1] - W[:, 1] * E[:, 0])[:, None]) / den
            s = (np.outer(W[:, 0], U[1]) - np.outer(W[:, 1], U[0])) / den
            t[(np.abs(den) < 1e-12) | (t < 0) | (s < 0) | (s > 1) | ~np.isfinite(t)] = np.inf
            r[j] = t.min()
    return apply_ranger_noise(r, p, rng)
