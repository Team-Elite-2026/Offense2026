"""
Second-order (momentum) orbit simulator to evaluate SPEED laws.

The kinematic sim (orbit_sim.py) integrates position = speed*dir and therefore has
NO inertia -- it cannot reproduce momentum overshoot. This one carries a velocity
state with a first-order lag (drag time constant tau), so when the commanded
direction swings, the actual velocity lags and the robot overshoots -- exactly the
effect we are trying to tame with the speed law.

Angle conventions identical to the firmware (see orbit_sim.py): +y toward goal,
bearings clockwise from +y, movement(a) drives toward body bearing heading+a.
"""
import math
import numpy as np
from orbit_sim import orbit_current, norm180, norm360


def run(robot_xy, ball_xy, goal_xy, speed_law,
        v_max=90.0, tau=0.18, dt=0.01, omega_max=240.0,
        max_steps=4000, contact_cm=12.0):
    """speed_law(robot_angle, ball_angle, dist) -> speed factor in [0,1]."""
    R = np.array(robot_xy, float)
    G = np.array(goal_xy, float)
    B = np.array(ball_xy, float)
    V = np.array([0.0, 0.0])
    h = math.degrees(math.atan2(G[0] - R[0], G[1] - R[1]))  # face goal

    # shot line unit vector (ball -> goal) and its perpendicular, for overshoot.
    line = G - B
    line /= (np.linalg.norm(line) + 1e-9)
    perp = np.array([line[1], -line[0]])

    path = [R.copy()]
    max_overshoot = 0.0     # worst signed cross-line excursion once near the ball
    crossings = 0
    prev_side = None
    captured = False
    align_err = None

    for _ in range(max_steps):
        d = B - R
        dist = math.hypot(d[0], d[1])
        ball_field = math.degrees(math.atan2(d[0], d[1]))
        ball_angle = norm360(ball_field - h)

        if dist <= contact_cm and abs(norm180(ball_angle)) < 35:
            captured = True
            b2g = math.degrees(math.atan2(G[0] - B[0], G[1] - B[1]))
            align_err = abs(norm180(b2g - ball_field))
            break

        robot_angle = orbit_current(ball_angle, dist)     # legacy goal-blind orbit
        sf = speed_law(robot_angle, ball_angle, dist)
        sf = max(0.0, min(1.0, sf))

        world = math.radians(h + robot_angle)
        v_des = sf * v_max * np.array([math.sin(world), math.cos(world)])
        V += (v_des - V) * min(1.0, dt / tau)             # first-order momentum lag
        R = R + V * dt
        path.append(R.copy())

        # heading tracks goal
        gf = math.degrees(math.atan2(G[0] - R[0], G[1] - R[1]))
        dh = max(-omega_max * dt, min(omega_max * dt, norm180(gf - h)))
        h += dh

        # overshoot bookkeeping once we are in the endgame (within 30 cm of ball)
        if dist < 30:
            cross = float(np.dot(R - B, perp))
            if prev_side is not None and (cross > 0) != (prev_side > 0):
                crossings += 1
            prev_side = cross
            max_overshoot = max(max_overshoot, abs(cross))

    return {
        "path": np.array(path), "captured": captured, "align_err": align_err,
        "max_overshoot": max_overshoot, "crossings": crossings,
        "steps": len(path), "ball": B, "goal": G,
    }


# ---- speed laws -----------------------------------------------------------
FLOOR, AMP = 0.17, 0.11   # -> range [0.17, 0.28], matching offenseSpeedFactor

def law_constant(ra, ba, dist):
    return 0.28

def law_cos_robot(ra, ba, dist):
    return FLOOR + AMP * abs(math.cos(math.radians(ra)))

def law_cos_offset(ra, ba, dist):
    off = norm180(ra - ba)
    return FLOOR + AMP * max(0.0, math.cos(math.radians(off)))


LAWS = {"constant": law_constant,
        "|cos(robotAngle)|": law_cos_robot,
        "cos(offset)": law_cos_offset}


if __name__ == "__main__":
    ball = (0.0, 0.0)
    goal = (0.0, 140.0)
    # a spread of starts that force real swings to get behind the ball
    starts = [(-70, -35), (75, -30), (-90, 20), (60, 40), (-40, -70)]
    print(f"{'law':18s} {'meanOvershoot':>13s} {'maxOvershoot':>12s} "
          f"{'crossings':>10s} {'meanAlign':>10s} {'cap':>5s}")
    for name, law in LAWS.items():
        ov, mx, cr, al, cap = [], 0, 0, [], 0
        for s in starts:
            r = run(s, ball, goal, law)
            ov.append(r["max_overshoot"]); mx = max(mx, r["max_overshoot"])
            cr += r["crossings"]; cap += r["captured"]
            if r["align_err"] is not None:
                al.append(r["align_err"])
        print(f"{name:18s} {np.mean(ov):13.2f} {mx:12.2f} {cr:10d} "
              f"{(np.mean(al) if al else float('nan')):10.2f} {cap:3d}/{len(starts)}")
