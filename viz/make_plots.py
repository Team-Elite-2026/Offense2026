"""
Generate trajectory comparison figures: CURRENT goal-blind orbit vs PROPOSED
goal-aware orbit, sweeping ball angle, ball distance, and goal angle.

Outputs PNGs into viz/out/.
"""
import os
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

from orbit_sim import orbit_current, orbit_goal_aware, simulate, norm180

OUT = os.path.join(os.path.dirname(__file__), "out")
os.makedirs(OUT, exist_ok=True)

CUR = lambda ba, d, ga: orbit_current(ba, d)
NEW = lambda ba, d, ga: orbit_goal_aware(ba, d, ga)[0]

BALL_C = "#d33"
GOAL_C = "#2a7"
CUR_C = "#e08020"
NEW_C = "#1f6feb"


def draw_field(ax, ball, goal, title):
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=10)
    # goal mouth
    ax.plot([goal[0] - 25, goal[0] + 25], [goal[1], goal[1]], color=GOAL_C, lw=4,
            solid_capstyle="round")
    ax.plot(*goal, "o", color=GOAL_C, ms=5)
    ax.text(goal[0], goal[1] + 8, "GOAL", color=GOAL_C, ha="center", fontsize=8)
    # ball
    ax.plot(*ball, "o", color=BALL_C, ms=11, zorder=5)
    # ball->goal shooting line
    d = np.array(goal) - np.array(ball)
    d = d / (np.linalg.norm(d) + 1e-9)
    ax.plot([ball[0] - d[0] * 200, goal[0]], [ball[1] - d[1] * 200, goal[1]],
            ":", color=GOAL_C, lw=1, alpha=0.6)
    ax.grid(alpha=0.15)


def path_with_capture(ax, sim, color, label):
    p = sim["path"]
    ax.plot(p[:, 0], p[:, 1], color=color, lw=2, label=label, zorder=4)
    ax.plot(*p[0], "s", color=color, ms=6, zorder=4)
    # contact-alignment error annotation handled by caller
    return sim["align_err"]


# ---------------------------------------------------------------------------
# FIGURE 1: fan of approach angles, goal straight above the ball.
# Robot starts on a ring around the ball at varying bearings.
# ---------------------------------------------------------------------------
def fig_fan():
    ball = (0.0, 0.0)
    goal = (0.0, 130.0)
    radius = 80.0
    # bearings of robot position around the ball (field deg, cw from +y)
    bearings = [200, 230, 250, 270, 290, 130, 90, 45]
    fig, axes = plt.subplots(2, 2, figsize=(11, 10))
    for ctl, color, ax, name in [
        (CUR, CUR_C, axes[0, 0], "CURRENT orbit (goal-blind)"),
        (NEW, NEW_C, axes[0, 1], "PROPOSED orbit (goal-aware)"),
    ]:
        draw_field(ax, ball, goal, name)
        errs = []
        for b in bearings:
            rx = ball[0] + radius * math.sin(math.radians(b))
            ry = ball[1] + radius * math.cos(math.radians(b))
            s = simulate((rx, ry), ball, goal, ctl)
            ax.plot(s["path"][:, 0], s["path"][:, 1], color=color, lw=1.6, alpha=0.9)
            ax.plot(rx, ry, "s", color=color, ms=5)
            if s["align_err"] is not None:
                errs.append(s["align_err"])
        ax.set_xlim(-140, 140); ax.set_ylim(-140, 160)
        ax.text(0.02, 0.02,
                f"mean contact miss: {np.mean(errs):.1f} deg\nmax: {np.max(errs):.1f} deg",
                transform=ax.transAxes, fontsize=9, va="bottom",
                bbox=dict(boxstyle="round", fc="white", ec=color, alpha=0.9))

    # bottom row: alignment error bar chart per bearing
    width = 0.38
    idx = np.arange(len(bearings))
    cur_errs, new_errs = [], []
    for b in bearings:
        rx = ball[0] + radius * math.sin(math.radians(b))
        ry = ball[1] + radius * math.cos(math.radians(b))
        cur_errs.append(simulate((rx, ry), ball, goal, CUR)["align_err"] or 0)
        new_errs.append(simulate((rx, ry), ball, goal, NEW)["align_err"] or 0)
    ax = axes[1, 0]
    ax.bar(idx - width / 2, cur_errs, width, color=CUR_C, label="current")
    ax.bar(idx + width / 2, new_errs, width, color=NEW_C, label="proposed")
    ax.set_xticks(idx); ax.set_xticklabels([f"{b}" for b in bearings], fontsize=8)
    ax.set_xlabel("robot start bearing around ball (deg)")
    ax.set_ylabel("contact-line miss (deg)")
    ax.set_title("Shot-line error at contact (lower = straighter shot)", fontsize=10)
    ax.axhline(12, ls="--", color="gray", lw=1)
    ax.text(0, 13, "12 deg kick tolerance", fontsize=7, color="gray")
    ax.legend(fontsize=8); ax.grid(alpha=0.2, axis="y")

    # explanatory panel
    ax = axes[1, 1]; ax.axis("off")
    ax.text(0.0, 1.0,
            "Why the difference\n\n"
            "CURRENT orbit only knows ball angle + distance, so it always\n"
            "curves the ball to the robot's FRONT by the shortest arc. Where\n"
            "the robot ends up relative to the goal is whatever falls out --\n"
            "often it contacts the ball moving across the shot line.\n\n"
            "PROPOSED orbit aims at the target point a fixed distance BEHIND\n"
            "the ball on the ball->goal line, and adds a tangential go-around\n"
            "term when it is on the goal side of the ball. Every approach\n"
            "converges onto the shooting line, so contact pushes the ball\n"
            "straight at the goal.",
            transform=ax.transAxes, va="top", fontsize=9, family="monospace")
    fig.suptitle("Fan of approaches -- goal directly beyond the ball", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(os.path.join(OUT, "fig1_fan.png"), dpi=130)
    print("wrote fig1_fan.png  mean miss  cur=%.1f new=%.1f"
          % (np.mean(cur_errs), np.mean(new_errs)))


# ---------------------------------------------------------------------------
# FIGURE 2: vary GOAL angle. Robot + ball fixed, goal swept left->right.
# ---------------------------------------------------------------------------
def fig_goal_angle():
    ball = (0.0, 0.0)
    robot = (-70.0, -40.0)
    goal_dirs = [-55, -25, 0, 25, 55]   # goal bearing from ball (deg cw from +y)
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    for ctl, color, ax, name in [
        (CUR, CUR_C, axes[0], "CURRENT orbit"),
        (NEW, NEW_C, axes[1], "PROPOSED orbit"),
    ]:
        ax.set_aspect("equal"); ax.set_title(name, fontsize=11)
        ax.plot(*ball, "o", color=BALL_C, ms=11, zorder=5)
        ax.plot(*robot, "s", color="black", ms=7, zorder=5)
        ax.text(robot[0], robot[1] - 12, "robot start", ha="center", fontsize=8)
        cmap = plt.cm.viridis(np.linspace(0, 0.9, len(goal_dirs)))
        for gd, gc in zip(goal_dirs, cmap):
            gx = ball[0] + 150 * math.sin(math.radians(gd))
            gy = ball[1] + 150 * math.cos(math.radians(gd))
            s = simulate(robot, ball, (gx, gy), ctl)
            ax.plot(s["path"][:, 0], s["path"][:, 1], color=gc, lw=2,
                    label=f"goal {gd:+d} deg (miss {s['align_err']:.0f})")
            ax.plot([ball[0], gx], [ball[1], gy], ":", color=gc, lw=1, alpha=0.5)
            ax.plot(gx, gy, "o", color=gc, ms=5)
        ax.legend(fontsize=7, loc="lower right")
        ax.grid(alpha=0.15)
        ax.set_xlim(-150, 160); ax.set_ylim(-90, 175)
    fig.suptitle("Same start, swept goal direction -- proposed adapts the arc to the goal",
                 fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(os.path.join(OUT, "fig2_goal_angle.png"), dpi=130)
    print("wrote fig2_goal_angle.png")


# ---------------------------------------------------------------------------
# FIGURE 3: heatmap of contact miss over (ball bearing around robot, distance)
# ---------------------------------------------------------------------------
def fig_heatmap():
    ball_bearings = np.arange(20, 341, 20)   # ball position bearing around the ball
    dists = np.arange(40, 161, 20)
    goal = (0.0, 140.0)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    grids = {}
    for ctl, name in [(CUR, "current"), (NEW, "proposed")]:
        g = np.zeros((len(dists), len(ball_bearings)))
        for i, dist in enumerate(dists):
            for j, b in enumerate(ball_bearings):
                rx = 0 + dist * math.sin(math.radians(b))
                ry = 0 + dist * math.cos(math.radians(b))
                s = simulate((rx, ry), (0.0, 0.0), goal, ctl)
                g[i, j] = s["align_err"] if s["align_err"] is not None else 90
        grids[name] = g
    vmax = max(grids["current"].max(), grids["proposed"].max())
    for ax, name in zip(axes, ["current", "proposed"]):
        im = ax.imshow(grids[name], origin="lower", aspect="auto", cmap="RdYlGn_r",
                       vmin=0, vmax=vmax,
                       extent=[ball_bearings[0], ball_bearings[-1],
                               dists[0], dists[-1]])
        ax.set_title(f"{name}  (mean {grids[name].mean():.1f} deg)", fontsize=11)
        ax.set_xlabel("robot start bearing around ball (deg)")
        ax.set_ylabel("start distance to ball (cm)")
        fig.colorbar(im, ax=ax, label="contact-line miss (deg)")
    fig.suptitle("Contact shot-line error across start positions (green = accurate)",
                 fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(os.path.join(OUT, "fig3_heatmap.png"), dpi=130)
    print("wrote fig3_heatmap.png  mean miss cur=%.1f new=%.1f"
          % (grids["current"].mean(), grids["proposed"].mean()))


if __name__ == "__main__":
    fig_fan()
    fig_goal_angle()
    fig_heatmap()
    print("done ->", OUT)
