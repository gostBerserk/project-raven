"""2D simulation of the drone pilot: follow a moving target and dodge thrown foam balls.

It runs the SAME planner (drone/planner.py), guidance (gnc/guidance.py) and tracker
(gnc/association.py) as the real pilot. The drone is a velocity-commanded point with a first-order
response and an acceleration limit; the camera sees only a ground footprint below it, with
position noise and latency. The cold-gas assist is not modelled: at 0.1 to 0.3 N it adds almost
nothing to a drone of this size.

    python drone/sim.py --runs 30                  # follow + dodge, realistic sensing
    python drone/sim.py --runs 30 --no-dodge       # follow only (the baseline)
"""
import argparse
import math
import os
import sys
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gnc import params                                   # noqa: E402
from gnc.association import ThreatTracker                # noqa: E402
from drone.planner import PilotConfig, plan              # noqa: E402

DT = 0.01
T_END = 20.0
TAU = 0.30             # s, velocity response time constant of the drone
A_MAX = 4.0            # m/s^2, acceleration limit
LATENCY = 0.08         # s, camera + processing
POS_NOISE = 0.02       # m, 1-sigma position noise
ALTITUDE = 4.0         # m
HFOV_DEG = 80.0
ASPECT = 0.75          # image height / width
R_BALL = 0.07
BALL_SPEED = (1.0, 2.5)


def footprint():
    hx = ALTITUDE * math.tan(math.radians(HFOV_DEG / 2))
    return hx * ASPECT, hx            # half-extent forward (x), left (y)


def run(seed, dodge=True, ideal=False, cfg=PilotConfig()):
    rng = np.random.default_rng(seed)
    lat, noise = (0.0, 0.0) if ideal else (LATENCY, POS_NOISE)
    fx, fy = footprint()
    p, v = np.zeros(2), np.zeros(2)
    v_cmd = np.zeros(2)
    balls, hits, next_spawn = [], 0, 1.0
    q = deque()
    tracker = ThreatTracker(gate=0.4)
    target_rel = None
    cam_period = 1.0 / params.CAMERA_FPS
    next_cam = 0.0
    standoff_err = []

    def target_pos(t):                # a person walking a slow circle
        a = 0.15 * t
        return np.array([3.0 * math.cos(a), 3.0 * math.sin(a)])

    for i in range(int(T_END / DT)):
        t = i * DT
        if t >= next_spawn:
            ang = rng.uniform(0, 2 * math.pi)
            start = p + 7.0 * np.array([math.cos(ang), math.sin(ang)])
            aim = p + rng.uniform(-0.2, 0.2, 2)
            speed = rng.uniform(*BALL_SPEED)
            balls.append({"p": start, "v": (aim - start) / np.linalg.norm(aim - start) * speed, "hit": False})
            next_spawn += rng.uniform(1.6, 2.6)

        if t >= next_cam:             # camera frame: only things under the drone are visible
            next_cam += cam_period
            seen = []
            for b in balls:
                r = b["p"] - p
                if abs(r[0]) < fx and abs(r[1]) < fy:
                    seen.append(r + rng.normal(0, noise, 2))
            tr = target_pos(t) - p
            tgt = tr + rng.normal(0, noise, 2) if (abs(tr[0]) < fx and abs(tr[1]) < fy) else None
            q.append((t + lat, t, seen, tgt))
        while q and q[0][0] <= t:
            _, tm, seen, tgt = q.popleft()
            threats = tracker.update([(s[0], s[1]) for s in seen], tm)
            target_rel = None if tgt is None else (tgt[0], tgt[1])
            pl = plan(target_rel, threats if dodge else [], cfg)
            v_cmd = np.array([pl.vx, pl.vy])

        dv = (v_cmd - v) / TAU
        n = np.linalg.norm(dv)
        if n > A_MAX:
            dv *= A_MAX / n
        v += dv * DT
        p += v * DT

        for b in balls:
            b["p"] += b["v"] * DT
            if not b["hit"] and np.linalg.norm(b["p"] - p) < cfg.drone_radius_m - 0.05 + R_BALL:
                b["hit"] = True
                hits += 1
        balls = [b for b in balls if np.linalg.norm(b["p"] - p) < 12.0]
        if i % 20 == 0:
            standoff_err.append(abs(np.linalg.norm(target_pos(t) - p) - cfg.standoff_m))
    return hits, float(np.mean(standoff_err))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=30)
    ap.add_argument("--first-seed", type=int, default=0)
    ap.add_argument("--no-dodge", action="store_true")
    ap.add_argument("--ideal", action="store_true", help="no latency or noise")
    ap.add_argument("--max-speed", type=float, default=1.5)
    a = ap.parse_args()
    cfg = PilotConfig(max_speed=a.max_speed)
    res = np.array([run(s, not a.no_dodge, a.ideal, cfg) for s in range(a.first_seed, a.first_seed + a.runs)])
    label = ("follow only" if a.no_dodge else "follow + dodge") + (", ideal sensing" if a.ideal else ", realistic sensing")
    print(f"{label} | max speed {a.max_speed} m/s | {a.runs} runs of {T_END:.0f} s")
    print(f"  runs with no ball hit: {np.mean(res[:, 0] == 0) * 100:.0f}%   hits per run: {res[:, 0].mean():.2f}"
          f"   mean standoff error: {res[:, 1].mean():.2f} m")


if __name__ == "__main__":
    main()
