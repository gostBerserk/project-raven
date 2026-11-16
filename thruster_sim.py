"""
Tabletop cold-gas thruster platform: pursue a target and dodge foam balls.
Run:  python thruster_sim.py --thrust 2.0 --seed 7   (see --help)
2D point-mass model on a near-frictionless surface (air bearing).
Thrusters are on/off valves, so thrust is PWM-modulated (pulse-width control).
"""
import os, sys
from collections import deque
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle, Rectangle

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gnc import params                                  # noqa: E402  shared with the camera script
from gnc.estimator import TrackedPoint                  # noqa: E402
from gnc.guidance import dodge_vector                   # noqa: E402

rng = np.random.default_rng(7)

# ---------- Physical parameters ----------
TABLE_W, TABLE_H = 2.0, 1.5        # m
M = 1.2                            # platform mass, kg
R_P = 0.12                         # platform radius, m
R_T = 0.05                         # target radius, m
R_B = 0.04                         # foam ball radius, m
F_MAX = 2.0                        # thrust per valve, N
A_MAX = F_MAX / M                  # max accel, m/s^2
BALL_V = (0.8, 1.2)                # ball speed range, m/s
DAMP = 0.05                        # tiny air-bearing drag, 1/s
PWM_T = 0.05                       # valve PWM period, s (20 Hz)
LATENCY = 0.06                     # camera + USB + processing delay, s (0 = ideal)
POS_NOISE = 0.008                  # 1-sigma position measurement noise, m (0 = ideal)
VALVE_DELAY = 0.02                 # command-to-thrust delay of the valve, s (0 = ideal)
DT, T_END = 0.005, 20.0

# ---------- Controller gains ----------
KP, KD = 2.2, 2.6
# dodge horizon and danger radius come from gnc/params.py (shared with the camera script)

def target_pos(t):
    return np.array([1.0 + 0.65 * np.sin(0.35 * t),
                     0.75 + 0.45 * np.sin(0.55 * t + 1.0)])

def spawn_ball(p_plat, t):
    side = rng.integers(0, 4)
    if side == 0:   s = np.array([0.0, rng.uniform(0, TABLE_H)])
    elif side == 1: s = np.array([TABLE_W, rng.uniform(0, TABLE_H)])
    elif side == 2: s = np.array([rng.uniform(0, TABLE_W), 0.0])
    else:           s = np.array([rng.uniform(0, TABLE_W), TABLE_H])
    aim = p_plat + rng.normal(0, 0.03, 2)       # aimed near the platform
    d = aim - s
    v = d / np.linalg.norm(d) * rng.uniform(*BALL_V)
    return {"p": s.copy(), "v": v, "hit": False}

def run():
    n = int(T_END / DT)
    p = np.array([0.3, 0.3]); v = np.zeros(2)
    balls, next_spawn, next_id = [], 1.0, 0
    log = {k: [] for k in ("t", "p", "tgt", "balls", "thr", "mode")}
    touches, hits, touching = 0, 0, False
    dist_t, min_ball = [], []

    # --- sensing chain: camera frames -> latency -> trackers -> guidance ---
    cam_period = 1.0 / params.CAMERA_FPS
    next_cam = 0.0
    meas_q = deque()                       # (time available, measurement)
    plat_track = TrackedPoint()
    ball_tracks = {}                       # ball id -> TrackedPoint (association assumed perfect)
    tgt_meas = target_pos(0.0)
    cmd, mode = np.zeros(2), "PURSUE"
    valve_steps = int(round(VALVE_DELAY / DT))
    fire_buf = deque([np.zeros(2)] * valve_steps)

    def noisy(x):
        return x + rng.normal(0, POS_NOISE, 2) if POS_NOISE > 0 else x.copy()

    for i in range(n):
        t = i * DT
        tgt = target_pos(t)
        if t >= next_spawn:
            b = spawn_ball(p, t); b["id"] = next_id; next_id += 1
            balls.append(b); next_spawn += rng.uniform(1.6, 2.6)

        # --- camera frame (true state + noise), delivered after LATENCY ---
        if t >= next_cam:
            next_cam += cam_period
            meas_q.append((t + LATENCY, {"t": t, "plat": noisy(p), "tgt": noisy(tgt),
                                        "balls": {b["id"]: noisy(b["p"]) for b in balls}}))
        while meas_q and meas_q[0][0] <= t:
            _, m = meas_q.popleft()
            plat_track.update(m["plat"][0], m["plat"][1], m["t"])
            tgt_meas = m["tgt"]
            for bid, bp in m["balls"].items():
                ball_tracks.setdefault(bid, TrackedPoint()).update(bp[0], bp[1], m["t"])
            for bid in [k for k in ball_tracks if k not in m["balls"]]:
                del ball_tracks[bid]

            # --- guidance (runs once per camera frame, command held in between) ---
            pe = np.array(plat_track.position()); ve = np.array(plat_track.velocity())
            threats = [(*tr.position(), *tr.velocity()) for tr in ball_tracks.values()]
            res = dodge_vector(pe, ve, threats, R_P, target=tgt_meas)
            if res is not None:
                vec = np.array(res[:2]); cmd = vec / np.linalg.norm(vec); mode = "DODGE"
            else:
                cmd = np.clip((KP * (tgt_meas - pe) - KD * ve) / A_MAX, -1, 1); mode = "PURSUE"

        # --- 4 on/off valves (+x, -x, +y, -y) with PWM, then valve delay ---
        phase = (t % PWM_T) / PWM_T
        fire = np.zeros(2)
        for ax in range(2):
            if abs(cmd[ax]) > phase:
                fire[ax] = np.sign(cmd[ax])
        if valve_steps:
            fire_buf.append(fire); fire = fire_buf.popleft()
        a = fire * A_MAX - DAMP * v

        # --- integrate ---
        v += a * DT
        p += v * DT
        for ax, lim in enumerate((TABLE_W, TABLE_H)):  # soft wall bounce
            if p[ax] < R_P:       p[ax], v[ax] = R_P, abs(v[ax]) * 0.3
            if p[ax] > lim - R_P: p[ax], v[ax] = lim - R_P, -abs(v[ax]) * 0.3

        for b in balls:
            b["p"] += b["v"] * DT
            if not b["hit"] and np.linalg.norm(b["p"] - p) < R_P + R_B:
                b["hit"] = True; hits += 1
        balls = [b for b in balls
                 if -0.2 < b["p"][0] < TABLE_W + 0.2 and -0.2 < b["p"][1] < TABLE_H + 0.2]

        d_t = np.linalg.norm(tgt - p)
        if d_t < R_P + R_T:
            if not touching: touches += 1
            touching = True
        elif d_t > R_P + R_T + 0.05:
            touching = False
        dist_t.append(d_t)
        min_ball.append(min([np.linalg.norm(b["p"] - p) for b in balls], default=np.nan))

        if i % 12 == 0:
            log["t"].append(t); log["p"].append(p.copy()); log["tgt"].append(tgt.copy())
            log["balls"].append([b["p"].copy() for b in balls])
            log["thr"].append(fire.copy()); log["mode"].append(mode)
    return log, np.array(dist_t), np.array(min_ball), touches, hits

def animate(log, path):
    fig, ax = plt.subplots(figsize=(6, 4.5))
    ax.set_xlim(0, TABLE_W); ax.set_ylim(0, TABLE_H); ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    ax.add_patch(Rectangle((0, 0), TABLE_W, TABLE_H, fc="#f4f1ea", ec="#333", lw=2))
    plat = Circle((0, 0), R_P, fc="#2a6fdb", ec="k", zorder=3)
    tgt = Circle((0, 0), R_T, fc="#f2b705", ec="k", zorder=2)
    ax.add_patch(plat); ax.add_patch(tgt)
    trail, = ax.plot([], [], color="#2a6fdb", alpha=0.35, lw=1.5)
    ball_sc = ax.scatter([], [], s=45, c="#d8342c", zorder=4)
    plume = ax.scatter([], [], s=30, c="#7fd4ff", marker="*", zorder=5)
    txt = ax.text(0.03, 0.95, "", transform=ax.transAxes, fontsize=10, weight="bold", va="top")
    P = np.array(log["p"])

    def upd(i):
        plat.center = P[i]; tgt.center = log["tgt"][i]
        trail.set_data(P[:i + 1, 0], P[:i + 1, 1])
        bl = log["balls"][i]
        ball_sc.set_offsets(np.array(bl) if bl else np.empty((0, 2)))
        f = log["thr"][i]
        puffs = [P[i] - np.array([np.sign(f[0]) * (R_P + 0.04), 0])] if f[0] else []
        if f[1]: puffs.append(P[i] - np.array([0, np.sign(f[1]) * (R_P + 0.04)]))
        plume.set_offsets(np.array(puffs) if puffs else np.empty((0, 2)))
        txt.set_text(f"t={log['t'][i]:4.1f}s  {log['mode'][i]}")
        txt.set_color("#d8342c" if log["mode"][i] == "DODGE" else "#1b7a3d")
        return plat, tgt, trail, ball_sc, plume, txt

    ani = FuncAnimation(fig, upd, frames=len(P), interval=60, blit=False)
    ani.save(path, writer=PillowWriter(fps=16), dpi=70)
    plt.close(fig)

def summary(log, dist_t, min_ball, touches, hits, path):
    t = np.arange(len(dist_t)) * DT
    fig, axs = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
    axs[0].plot(t, dist_t, color="#2a6fdb")
    axs[0].axhline(R_P + R_T, color="#f2b705", ls="--", label="contact distance")
    axs[0].set_ylabel("Distance to target (m)"); axs[0].legend(frameon=False)
    axs[0].set_title(f"Target touches: {touches}   Ball hits: {hits}", fontweight="bold")
    axs[1].plot(t, min_ball, color="#d8342c")
    axs[1].axhline(R_P + R_B, color="k", ls="--", label="collision distance")
    axs[1].set_ylabel("Nearest ball (m)"); axs[1].set_xlabel("Time (s)")
    axs[1].legend(frameon=False)
    for a in axs:
        a.spines["top"].set_visible(False); a.spines["right"].set_visible(False)
    plt.tight_layout(); plt.savefig(path, dpi=130); plt.close(fig)

if __name__ == "__main__":
    import argparse, os
    ap = argparse.ArgumentParser(description="Simulate the thruster platform (pursue target, dodge balls).")
    ap.add_argument("--thrust", type=float, default=F_MAX, help="thrust per valve, N (default %(default)s)")
    ap.add_argument("--mass", type=float, default=M, help="platform mass, kg (default %(default)s)")
    ap.add_argument("--ball-speed", type=float, nargs=2, default=list(BALL_V), metavar=("MIN", "MAX"),
                    help="ball speed range, m/s (default %(default)s)")
    ap.add_argument("--ideal", action="store_true", help="no camera latency, measurement noise or valve delay")
    ap.add_argument("--seed", type=int, default=7, help="random seed for ball spawns")
    ap.add_argument("--out", default="../results", help="output folder for the GIF and summary plot")
    ap.add_argument("--no-gif", action="store_true", help="skip the slow GIF export")
    args = ap.parse_args()

    F_MAX, M = args.thrust, args.mass
    BALL_V = tuple(args.ball_speed)
    if args.ideal:
        LATENCY, POS_NOISE, VALVE_DELAY = 0.0, 0.0, 0.0
    A_MAX = F_MAX / M
    rng = np.random.default_rng(args.seed)
    os.makedirs(args.out, exist_ok=True)

    log, dist_t, min_ball, touches, hits = run()
    if not args.no_gif:
        animate(log, os.path.join(args.out, "thruster_sim.gif"))
    summary(log, dist_t, min_ball, touches, hits, os.path.join(args.out, "thruster_sim_summary.png"))
    print(f"{'ideal sensing' if args.ideal else f'latency={LATENCY*1000:.0f} ms noise={POS_NOISE*1000:.0f} mm valve delay={VALVE_DELAY*1000:.0f} ms'}")
    print(f"thrust={F_MAX} N  accel={A_MAX:.2f} m/s^2  touches={touches}  hits={hits}  "
          f"mean_dist={np.nanmean(dist_t):.2f} m  closest_ball={np.nanmin(min_ball):.3f} m")
