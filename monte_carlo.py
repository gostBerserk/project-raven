"""
Run many random trials of the simulation and report how often the platform
gets hit, for several thrust levels. Thrust is the big unknown on real
hardware, so this tells you how light/strong the platform needs to be.

    cd simulation && python monte_carlo.py --runs 30 --thrust 2.0 1.0 0.5 0.2
"""
import argparse, csv, os
import numpy as np
import thruster_sim as ts

REAL_LAT, REAL_NOISE, REAL_VALVE = ts.LATENCY, ts.POS_NOISE, ts.VALVE_DELAY


def trial(thrust, mass, seed, ball_v, ideal=False):
    ts.BALL_V = tuple(ball_v)
    ts.LATENCY, ts.POS_NOISE, ts.VALVE_DELAY = (0.0, 0.0, 0.0) if ideal else (REAL_LAT, REAL_NOISE, REAL_VALVE)
    ts.F_MAX, ts.M, ts.A_MAX = thrust, mass, thrust / mass
    ts.rng = np.random.default_rng(seed)
    _, dist_t, _, touches, hits = ts.run()
    return touches, hits, float(np.nanmean(dist_t))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=30)
    ap.add_argument("--mass", type=float, default=1.2)
    ap.add_argument("--ball-speed", type=float, nargs=2, default=[0.8, 1.2], metavar=("MIN", "MAX"),
                    help="ball speed range, m/s")
    ap.add_argument("--ideal", action="store_true", help="no camera latency, measurement noise or valve delay")
    ap.add_argument("--first-seed", type=int, default=0, help="seeds used are first-seed .. first-seed+runs-1")
    ap.add_argument("--thrust", type=float, nargs="+", default=[2.0, 1.0, 0.5, 0.2])
    ap.add_argument("--out", default="../results/montecarlo.csv")
    a = ap.parse_args()

    rows = []
    print(f"{'thrust N':>8} {'accel':>7} {'runs hit-free':>14} {'hits/run':>9} {'touches/run':>12} {'mean dist m':>12}")
    for f in a.thrust:
        res = np.array([trial(f, a.mass, s, a.ball_speed, a.ideal) for s in range(a.first_seed, a.first_seed + a.runs)])
        hit_free = float(np.mean(res[:, 1] == 0)) * 100
        row = [f, f / a.mass, hit_free, res[:, 1].mean(), res[:, 0].mean(), res[:, 2].mean()]
        rows.append(row)
        print(f"{row[0]:8.2f} {row[1]:7.2f} {row[2]:13.0f}% {row[3]:9.2f} {row[4]:12.2f} {row[5]:12.2f}")

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["thrust_N", "accel_m_s2", "pct_runs_without_hit", "hits_per_run", "touches_per_run", "mean_dist_m"])
        w.writerows([[round(x, 3) for x in r] for r in rows])
    print("saved", a.out, f"({a.runs} runs per thrust level, mass {a.mass} kg, balls {a.ball_speed[0]}-{a.ball_speed[1]} m/s)")
