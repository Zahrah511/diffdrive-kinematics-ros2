"""
plot_run.py — figure + numeric summary of one Gazebo run, from the kinematics_node CSV.

  python3 plot_run.py results/v1_straight.csv   -> results/v1_straight.png + summary in the terminal

Left panel : exact, Euler and true (Gazebo) paths
Right panel: position gap of exact & Euler to the true pose, COMPARED AT THE SAME INSTANT
             (true pose arrives at 50 Hz, wheel speeds at ~1000 Hz; our pose is interpolated to
             the time the true pose was sampled, so no false gap appears from timing)
"""
import csv
import math
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

path = sys.argv[1]
rows = list(csv.DictReader(open(path)))
col = lambda k: np.array([float(r[k]) for r in rows])
t, vr, vl = col("t"), col("vr"), col("vl")
xe, ye, the = col("x_exact"), col("y_exact"), col("th_exact")
xu, yu = col("x_euler"), col("y_euler")
xt, yt, tht = col("x_true"), col("y_true"), col("th_true")

# --- trim to the motion window (drop the waiting time before the robot moves)
moving = np.where(np.maximum(abs(vr), abs(vl)) > 1e-3)[0]
i0 = max(0, moving[0] - 1) if len(moving) else 0
t_move0 = t[i0]

# --- express the true pose RELATIVE to the robot pose when it started moving.
# Our computation always starts at (0, 0, 0); the Gazebo robot may start anywhere
# (e.g. Gazebo was not restarted). Transform: shift to the start point, then rotate by -th0.
j0 = next((j for j in range(i0, -1, -1) if not np.isnan(xt[j])),
          next(j for j in range(len(xt)) if not np.isnan(xt[j])))
x0, y0, th0 = xt[j0], yt[j0], tht[j0]
dx, dy = xt - x0, yt - y0
xt, yt = np.cos(th0) * dx + np.sin(th0) * dy, -np.sin(th0) * dx + np.cos(th0) * dy
tht = np.arctan2(np.sin(tht - th0), np.cos(tht - th0))
moved_from_origin = math.hypot(x0, y0) > 0.01

# --- comparison points: one per true-pose sample, at the time it was taken
if "t_true" in rows[0]:
    ttr = col("t_true")
    ok = ~np.isnan(ttr) & ~np.isnan(xt)
    _, first = np.unique(ttr[ok], return_index=True)
    idx = np.where(ok)[0][first]
    tc = ttr[idx]
    keep = (tc >= t_move0) & (tc <= t[-1])
    idx, tc = idx[keep], tc[keep]
    gx, gy = xt[idx], yt[idx]
    ex, ey = np.interp(tc, t, xe), np.interp(tc, t, ye)
    ux, uy = np.interp(tc, t, xu), np.interp(tc, t, yu)
    sync = True
else:                                   # older CSV: no time synchronisation
    idx = np.where(~np.isnan(xt) & (t >= t_move0))[0]
    tc, gx, gy = t[idx], xt[idx], yt[idx]
    ex, ey, ux, uy = xe[idx], ye[idx], xu[idx], yu[idx]
    sync = False
de = np.hypot(ex - gx, ey - gy) * 100
du = np.hypot(ux - gx, uy - gy) * 100

# --- figure
fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.8))
sel = t >= t_move0
a1.plot(gx, gy, color="#1a9e77", lw=4, alpha=.5, label="True (Gazebo)")
a1.plot(xe[sel], ye[sel], color="#5b4bc4", lw=1.6, label="Our odometry: exact")
a1.plot(xu[sel], yu[sel], color="#d9622b", lw=1.2, ls="--", label="Our odometry: Euler")
a1.plot(xe[sel][0], ye[sel][0], "ko", ms=5, label="Start")
allx = np.concatenate([gx, xe[sel]]); ally = np.concatenate([gy, ye[sel]])
cx, cy = (allx.max() + allx.min()) / 2, (ally.max() + ally.min()) / 2
half = max(allx.max() - allx.min(), ally.max() - ally.min(), 1.0) / 2 * 1.15
a1.set_xlim(cx - half, cx + half); a1.set_ylim(cy - half, cy + half)
a1.set_aspect("equal"); a1.grid(alpha=.3); a1.set_xlabel("x (m)"); a1.set_ylabel("y (m)")
a1.legend(fontsize=8, loc="best"); a1.set_title("Path")

a2.plot(tc - t_move0, de, color="#5b4bc4", label="exact vs true")
a2.plot(tc - t_move0, du, color="#d9622b", ls="--", label="Euler vs true")
a2.grid(alpha=.3); a2.set_xlabel("time since motion start (s, simulation time)")
a2.set_ylabel("position gap (cm)"); a2.legend(fontsize=8)
a2.set_title("Gap to the true pose" + ("" if sync else " (old CSV, not synchronised)"))
fig.tight_layout()
out = path.rsplit(".", 1)[0] + ".png"
fig.savefig(out, dpi=150)

# --- numeric summary
wrap = lambda a: math.degrees(math.atan2(math.sin(a), math.cos(a)))
f = lambda x, y, th: f"x={x:7.3f}  y={y:7.3f}  th={wrap(th):7.1f} deg"
print("Saved:", out)
if moved_from_origin:
    print(f"Note: robot started at ({x0:.3f}, {y0:.3f}, {math.degrees(th0):.1f} deg) in the Gazebo world."
          " The true pose below is relative to that point.")
print("Final pose, exact     :", f(xe[-1], ye[-1], the[-1]))
print("Final pose, true      :", f(xt[-1], yt[-1], tht[-1]))
print(f"Final gap             : exact {de[-1]:.1f} cm, Euler {du[-1]:.1f} cm")
print(f"Maximum gap           : exact {de.max():.1f} cm, Euler {du.max():.1f} cm")

# circle radius (V1 turn & V2): least-squares circle fit to the true path
turn = abs(np.unwrap(tht[~np.isnan(tht)])[-1] - np.unwrap(tht[~np.isnan(tht)])[0])
name = path.rsplit("/", 1)[-1]
if any(k in name for k in ("turn", "circle")) and turn > math.pi and len(gx) > 10:
    A = np.c_[2 * gx, 2 * gy, np.ones_like(gx)]
    sol, *_ = np.linalg.lstsq(A, gx**2 + gy**2, rcond=None)
    R = math.sqrt(sol[2] + sol[0]**2 + sol[1]**2)
    print(f"True path radius (circle fit): {R:.3f} m")
