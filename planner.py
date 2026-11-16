"""Follow-and-dodge planner. Pure functions: no camera, no flight controller.

Frame: body, x forward, y LEFT, metres (a downward camera with image-up = forward).
Everything is relative to the drone, so the drone sits at the origin.

Behaviour
  - FOLLOW: keep the target at a standoff distance. It never closes in on the target:
    inside the standoff distance it backs off.
  - DODGE: if a ball will pass inside the danger radius soon, sidestep (gnc.guidance).
    Dodging has priority over following.
  - HOLD: nothing to follow and nothing to dodge, so command zero velocity.
"""
import math
from dataclasses import dataclass

from gnc.guidance import dodge_vector


@dataclass
class PilotConfig:
    max_speed: float = 1.5        # m/s, hard cap on any commanded speed
    standoff_m: float = 2.0       # keep the target at least this far away
    follow_gain: float = 0.8      # 1/s, proportional gain on the standoff error
    deadband_m: float = 0.15      # no follow command inside this error
    drone_radius_m: float = 0.35  # half the prop-tip to prop-tip span, plus a little margin


@dataclass
class Plan:
    vx: float                     # forward, m/s
    vy: float                     # left, m/s
    mode: str                     # "DODGE", "FOLLOW" or "HOLD"
    urgency: float = 0.0          # 0..1, how soon the nearest dodge threat arrives
    heading_deg: float = 0.0      # direction of (vx, vy), body frame, 0 = forward, 90 = left


def _limit(vx, vy, cap):
    s = math.hypot(vx, vy)
    if s > cap > 0:
        return vx * cap / s, vy * cap / s
    return vx, vy


def follow_velocity(target_rel, cfg):
    tx, ty = target_rel
    dist = math.hypot(tx, ty)
    if dist < 1e-6:
        return 0.0, 0.0
    ux, uy = tx / dist, ty / dist
    err = dist - cfg.standoff_m              # >0: too far (go toward), <0: too close (back off)
    if abs(err) < cfg.deadband_m:
        return 0.0, 0.0
    return _limit(ux * err * cfg.follow_gain, uy * err * cfg.follow_gain, cfg.max_speed)


def plan(target_rel, threats, cfg=PilotConfig()):
    """target_rel: (x, y) of the target relative to the drone, or None if not seen.
    threats: [(x, y, vx, vy)] of balls relative to the drone (velocities are relative too).
    Returns a Plan."""
    res = dodge_vector((0.0, 0.0), (0.0, 0.0), threats, cfg.drone_radius_m, target=target_rel)
    if res is not None:
        ex, ey, urg = res
        n = math.hypot(ex, ey)
        vx, vy = ex / n * cfg.max_speed, ey / n * cfg.max_speed
        return Plan(vx, vy, "DODGE", urg, math.degrees(math.atan2(vy, vx)) % 360.0)
    if target_rel is None:
        return Plan(0.0, 0.0, "HOLD")
    vx, vy = follow_velocity(target_rel, cfg)
    mode = "FOLLOW" if (vx or vy) else "HOLD"
    h = math.degrees(math.atan2(vy, vx)) % 360.0 if (vx or vy) else 0.0
    return Plan(vx, vy, mode, 0.0, h)


class PixelToBody:
    """Downward camera at a known height above flat ground -> body-frame metres.
    Assumes the camera is level and image-up is the drone's forward direction."""

    def __init__(self, width, height, hfov_deg, altitude_m):
        self.cx, self.cy = width / 2.0, height / 2.0
        self.m_per_px = 2.0 * altitude_m * math.tan(math.radians(hfov_deg) / 2.0) / width

    def __call__(self, u, v):
        return -(v - self.cy) * self.m_per_px, -(u - self.cx) * self.m_per_px   # (forward, left)
