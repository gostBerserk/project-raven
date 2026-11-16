"""Guidance: decide where to dodge. Pure functions, no camera or serial code.

Coordinates: x right, y UP, any consistent length unit (metres in the simulation,
pixels in the camera script after flipping image y). Velocities in the same unit per second.
"""
import math

from . import params


def time_to_closest_approach(rx, ry, rvx, rvy):
    """Threat at (rx, ry) relative to the platform, moving at (rvx, rvy) relative to it.
    Returns (t_closest, min_dist); t is None if there is no relative motion."""
    s2 = rvx * rvx + rvy * rvy
    if s2 < 1e-6:
        return None, math.hypot(rx, ry)
    t = max(0.0, -(rx * rvx + ry * rvy) / s2)
    return t, math.hypot(rx + rvx * t, ry + rvy * t)


def dodge_vector(platform_pos, platform_vel, threats, platform_radius, target=None,
                 horizon=params.HORIZON_S, danger_factor=params.DANGER_FACTOR):
    """threats: list of (x, y, vx, vy). For each ball that will pass too close soon,
    step SIDEWAYS (perpendicular to its relative velocity), toward the side away from
    where it will pass. Several threats are blended, weighted by urgency.

    Returns (ex, ey, urgency) with urgency in 0..1 (1 = about to arrive), or None when
    no dodge is needed. `target` (x, y) only breaks ties on a dead-on ball."""
    px, py = platform_pos
    pvx, pvy = platform_vel
    danger = danger_factor * platform_radius
    dead_on = params.DEAD_ON_FRAC * platform_radius

    ex = ey = 0.0
    best = None                      # (urgency, (nx, ny)) fallback if blended vectors cancel
    for (ox, oy, ovx, ovy) in threats:
        rvx, rvy = ovx - pvx, ovy - pvy
        rx, ry = ox - px, oy - py
        t, d = time_to_closest_approach(rx, ry, rvx, rvy)
        if t is None or t > horizon or d > danger:
            continue

        speed = math.hypot(rvx, rvy)
        n1 = (-rvy / speed, rvx / speed)          # unit vector perpendicular to the ball's path
        mx, my = rx + rvx * t, ry + rvy * t       # ball relative to platform at closest approach
        if math.hypot(mx, my) > dead_on:
            side = -(n1[0] * mx + n1[1] * my)     # >0: n1 points away from where the ball passes
        elif target is not None:
            side = n1[0] * (target[0] - px) + n1[1] * (target[1] - py)
        else:
            side = 1.0
        nx, ny = (n1[0], n1[1]) if side >= 0 else (-n1[0], -n1[1])

        urg = 1.0 - t / horizon
        w = 0.5 + urg
        ex += nx * w
        ey += ny * w
        if best is None or urg > best[0]:
            best = (urg, (nx, ny))

    if best is None:
        return None
    if math.hypot(ex, ey) < 1e-6:                 # opposing dodges cancelled: use the most urgent
        ex, ey = best[1]
    return ex, ey, best[0]
