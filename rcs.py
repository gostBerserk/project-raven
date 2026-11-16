"""Nozzle layout for the 6-valve cold-gas assist (mirror of firmware/drone_rcs)."""
import math

# Direction each valve pushes the DRONE, body frame, degrees (0 = forward, 90 = left, CCW positive).
# A nozzle exhausting to the left pushes the drone to the right (270).
PUSH_DEG = [270.0, 90.0, 270.0, 90.0, 180.0, 0.0]
NAMES = ["nose-left", "nose-right", "tail-left", "tail-right", "front-centre", "back-centre"]
# Where each nozzle sits in the horizontal plane at the drone's centre of mass, metres, body frame
# (x forward, y left). Estimated from the render; replace with your measurements.
# Valves 5 and 6 are on the fuselage centreline so their push passes through the centre: no spin.
POSITION_M = [(0.0, 0.10), (0.0, -0.10), (0.0, 0.10), (0.0, -0.10), (0.10, 0.0), (-0.10, 0.0)]
MAX_ANGLE_DIFF = 50.0   # same rule as the firmware: fire the best valve and any within this angle


def ang_diff(a, b):
    return abs((a - b + 540.0) % 360.0 - 180.0)


def select_valves(heading_deg):
    """Indices of the valves the firmware would open for this body-frame heading."""
    h = heading_deg % 360.0
    d = [ang_diff(h, p) for p in PUSH_DEG]
    best = min(range(len(d)), key=lambda i: d[i])
    return [i for i in range(len(d)) if i == best or d[i] <= MAX_ANGLE_DIFF]


def net_push_deg(valves):
    """Direction of the summed push of the chosen valves (for checking the layout)."""
    x = sum(math.cos(math.radians(PUSH_DEG[i])) for i in valves)
    y = sum(math.sin(math.radians(PUSH_DEG[i])) for i in valves)
    return math.degrees(math.atan2(y, x)) % 360.0


def yaw_torque(valves, positions=None, force_n=1.0):
    """Net yaw torque (N*m, CCW positive) if the given valves fire together with force_n each.
    Should be about zero: a nozzle whose push line misses the centre of mass makes the drone spin."""
    pos = positions or POSITION_M
    tz = 0.0
    for i in valves:
        fx = force_n * math.cos(math.radians(PUSH_DEG[i]))
        fy = force_n * math.sin(math.radians(PUSH_DEG[i]))
        x, y = pos[i]
        tz += x * fy - y * fx
    return tz
